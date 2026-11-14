package org.technodot.ftc.twentysix.roboctrl;

import org.technodot.ftc.twentysix.common.Alliance;

import java.util.ArrayDeque;

/** Converts fresh hive focus measurements into clockwise-positive turret targets. */
public final class HiveTurretAim {
    public static class Config {
        /** Camera's horizontal mounting angle relative to turret forward; positive right. */
        public double cameraYawDegrees = 0;
        /** Camera optical-center offsets from the turret's yaw axis. */
        public double cameraRightInches = 0;
        public double cameraForwardInches = 0;
    }

    private static final long LOST_TAG_NANOS = 1_000_000_000L;
    private static final long HISTORY_NANOS = 1_000_000_000L;

    private static final class Heading {
        final long time;
        final double degrees;

        Heading(long time, double degrees) { this.time = time; this.degrees = degrees; }
    }

    private final Alliance alliance;
    private final SimpleHiveSolver.Config camera;
    private final Config config;
    private final ArrayDeque<Heading> headings = new ArrayDeque<>();
    private long startTime, lastTagSeen, lastAimedFrame;
    private double targetHeading;
    private String status = "holding";

    public HiveTurretAim(Alliance alliance, SimpleHiveSolver.Config camera, Config config) {
        this.alliance = alliance;
        this.camera = camera;
        this.config = config;
    }

    public void start(double currentHeading, long now) {
        headings.clear();
        headings.addLast(new Heading(now, currentHeading));
        startTime = now; lastTagSeen = 0; lastAimedFrame = 0;
        targetHeading = normalize(currentHeading);
        status = "holding";
    }

    /** currentHeading is the unwrapped encoder angle, positive right, with zero forward. */
    public double update(double currentHeading, HiveObservation observation, long lastAllianceTagSeen, long now) {
        if (headings.isEmpty()) start(currentHeading, now);
        Heading latest = headings.peekLast();
        if (now > latest.time) headings.addLast(new Heading(now, currentHeading));
        while (headings.size() > 1 && headings.peekFirst().time < now - HISTORY_NANOS) headings.removeFirst();

        if (lastAllianceTagSeen > 0 && lastAllianceTagSeen <= now) {
            lastTagSeen = Math.max(lastTagSeen, lastAllianceTagSeen);
        }
        boolean ourHive = observation != null && observation.alliance == alliance;
        if (ourHive && !observation.tagIds.isEmpty() && observation.captureTimeNanos > 0
                && observation.captureTimeNanos <= now) {
            lastTagSeen = Math.max(lastTagSeen, observation.captureTimeNanos);
        }
        status = "holding";
        if (ourHive && observation.valid
                && observation.captureTimeNanos <= now && observation.captureTimeNanos > 0
                && observation.imageAgeMillis(now) <= camera.maxAgeMillis) {
            if (observation.captureTimeNanos > lastAimedFrame) {
                double captureHeading = headingAt(observation.captureTimeNanos);
                double correction = yawCorrection(observation);
                if (Double.isFinite(captureHeading) && Double.isFinite(correction)) {
                    targetHeading = normalize(captureHeading + correction);
                    lastAimedFrame = observation.captureTimeNanos;
                }
            }
            if (observation.captureTimeNanos == lastAimedFrame) status = "tracking";
        }
        if (now - (lastTagSeen == 0 ? startTime : lastTagSeen) >= LOST_TAG_NANOS) {
            targetHeading = 0;
            status = "forward: no alliance tag for 3 s";
        }
        return targetHeading;
    }

    public String getStatus() { return status; }

    private double headingAt(long time) {
        Heading previous = null;
        for (Heading sample : headings) {
            if (time == sample.time) return sample.degrees;
            if (sample.time > time) {
                if (previous == null) return Double.NaN;
                double fraction = (double) (time - previous.time) / (sample.time - previous.time);
                return previous.degrees + fraction * (sample.degrees - previous.degrees);
            }
            previous = sample;
        }
        return Double.NaN;
    }

    private double yawCorrection(HiveObservation observation) {
        if (!Double.isFinite(camera.cameraPitchDegrees)
                || !Double.isFinite(camera.cameraRollDegrees) || !Double.isFinite(config.cameraYawDegrees)
                || !Double.isFinite(config.cameraRightInches) || !Double.isFinite(config.cameraForwardInches)) {
            return Double.NaN;
        }
        double[][] mount = HiveSolver.mountRotation(camera.cameraPitchDegrees, camera.cameraRollDegrees);
        double[] level = HiveMath.multiply(HiveMath.transpose(mount), new double[] {
                observation.focusRightInches, observation.focusDownInches, observation.focusForwardInches});
        level = HiveMath.multiply(HiveMath.ry(Math.toRadians(config.cameraYawDegrees)), level);
        double right = level[0] + config.cameraRightInches;
        double forward = level[2] + config.cameraForwardInches;
        if (!Double.isFinite(right) || !Double.isFinite(forward) || Math.hypot(right, forward) < 1e-6) return Double.NaN;
        return Math.toDegrees(Math.atan2(right, forward));
    }

    private static double normalize(double degrees) { return ((degrees % 360) + 360) % 360; }
}
