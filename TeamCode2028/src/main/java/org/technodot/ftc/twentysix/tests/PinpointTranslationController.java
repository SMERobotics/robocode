package org.technodot.ftc.twentysix.tests;

/**
 * Short calibration leg controller. Position only determines when to stop and
 * the desired speed; velocity feedback learns the power needed to overcome
 * friction instead of tapering to an arbitrarily small open-loop power.
 */
final class PinpointTranslationController {
    enum Status {
        RUNNING, COMPLETE, WRONG_DIRECTION, DRIVE_STALLED,
        WHEELS_WITHOUT_POD_TRAVEL, DRIVE_ENCODERS_NOT_TRACKING, TIMED_OUT
    }

    static final double MAX_SECONDS = 15;
    private static final double POSITION_TOLERANCE_MM = 2;
    private static final double CRUISE_MM_PER_SECOND = 120;
    private static final double MIN_MOVING_MM_PER_SECOND = 40;

    final double targetMm, maxPower;
    private Status status = Status.RUNNING;
    private double elapsed, progress, wheelTicks, speed, targetSpeed;
    private double power, peakPower, integral;
    private double progressAnchor, progressTime, progressWheelTicks;

    PinpointTranslationController(double targetMm, double maxPower) {
        if (!finite(targetMm) || targetMm <= POSITION_TOLERANCE_MM
                || !finite(maxPower) || maxPower <= 0 || maxPower > 1) {
            throw new IllegalArgumentException("Invalid translation target or power limit");
        }
        this.targetMm = targetMm;
        this.maxPower = maxPower;
    }

    /** progress is signed along the requested direction; ticks average the working encoders only. */
    void update(double progressMm, double absoluteWheelTicks, double dt) {
        if (status != Status.RUNNING) return;
        if (!finite(progressMm) || !finite(absoluteWheelTicks) || !finite(dt) || dt <= 0
                || absoluteWheelTicks < wheelTicks) {
            throw new IllegalArgumentException("Invalid translation feedback");
        }
        elapsed += dt;
        double observedSpeed = (progressMm - progress) / dt;
        speed += dt / (0.15 + dt) * (observedSpeed - speed);
        progress = progressMm;
        wheelTicks = absoluteWheelTicks;
        if (progress < -25) { finish(Status.WRONG_DIRECTION); return; }
        if (progress >= targetMm - POSITION_TOLERANCE_MM) { finish(Status.COMPLETE); return; }
        if (progress >= 50 && wheelTicks < 5) { finish(Status.DRIVE_ENCODERS_NOT_TRACKING); return; }
        if (elapsed >= MAX_SECONDS) { finish(Status.TIMED_OUT); return; }

        double desiredSpeed = Math.min(CRUISE_MM_PER_SECOND,
                Math.max(MIN_MOVING_MM_PER_SECOND, 2.5 * (targetMm - progress)));
        targetSpeed += clip(desiredSpeed - targetSpeed, -180 * dt, 120 * dt);
        double error = targetSpeed - speed;
        double nextIntegral = clip(integral + 0.0015 * error * dt, -0.15, maxPower);
        double feedforward = 0.10 + 0.001 * targetSpeed;
        double proportional = 0.0015 * error;
        // Conditional integration prevents winding up against the power cap.
        if (feedforward + proportional + nextIntegral <= maxPower || error < 0) integral = nextIntegral;
        double wantedPower = clip(feedforward + proportional + integral, 0, maxPower);
        power += clip(wantedPower - power, -0.8 * dt, 0.4 * dt);
        peakPower = Math.max(peakPower, power);

        if (progress >= progressAnchor + 2) {
            progressAnchor = progress;
            progressTime = elapsed;
            progressWheelTicks = wheelTicks;
        }
        // Allow breakaway power to build before declaring a stall. Slow but
        // continuing travel resets this window and is not treated as a stall.
        if (elapsed > 3 && elapsed - progressTime > 2.5 && power >= 0.95 * maxPower) {
            finish(wheelTicks - progressWheelTicks > 40
                    ? Status.WHEELS_WITHOUT_POD_TRAVEL : Status.DRIVE_STALLED);
        }
    }

    private void finish(Status outcome) {
        status = outcome;
        power = 0;
    }

    Status status() { return status; }
    double power() { return power; }
    double peakPower() { return peakPower; }
    double progressMm() { return progress; }
    double speedMmPerSecond() { return speed; }
    double wheelTicks() { return wheelTicks; }
    double elapsedSeconds() { return elapsed; }

    private static double clip(double value, double min, double max) { return Math.max(min, Math.min(max, value)); }
    private static boolean finite(double value) { return !Double.isNaN(value) && !Double.isInfinite(value); }
}
