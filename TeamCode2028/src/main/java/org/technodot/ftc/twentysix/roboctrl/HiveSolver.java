package org.technodot.ftc.twentysix.roboctrl;

import org.technodot.ftc.twentysix.common.Alliance;
import org.technodot.ftc.twentysix.common.Hive;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.EnumMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/** Fits image corners jointly; independent tag poses are hypotheses, never averaged outputs. */
public final class HiveSolver {
    public static class Config {
        // Set this only after measuring pitch/roll; numeric defaults alone do not enable tracking.
        public boolean cameraOrientationConfigured = true;
        public double cameraPitchDegrees = 70;
        public double cameraRollDegrees = 0;
        public double aimRightInches = 0;
        public double aimDownInches = 0;
        public double aimIntoCellInches = 0;
        public double maxAgeMillis = 200;
        public double maxRmsPixels = 2;
        public double maxTagRmsPixels = 3;
        public double huberPixels = 2;
        public double ambiguityGapPixels = 0.5;
        public double materialFocusDifferenceInches = 1;
        public double materialAngleDifferenceDegrees = 1;
        // Five extra degrees allow roughly two more inches of travel at the cell focus.
        public double enterStopDegrees = 10;
        public double leaveStopDegrees = 13;
        public double settleMillis = 100;
        /** Median window for state classification only; focus/RBE use the current pose. */
        public int stateFilterFrames = 5;
        /** Continuous evidence required before leaving a resting state for TIPPING. */
        public double tipConfirmMillis = 150;
        public double sideDeadbandInches = 4;
        public double extraTravelDegrees = 8;
        public boolean allowSingleTag = true;

        String error() {
            if (!cameraOrientationConfigured || !Double.isFinite(cameraPitchDegrees) || !Double.isFinite(cameraRollDegrees)) {
                return "configure camera pitch and roll";
            }
            double[] positive = {maxAgeMillis, maxRmsPixels, maxTagRmsPixels, huberPixels,
                    ambiguityGapPixels, materialFocusDifferenceInches, materialAngleDifferenceDegrees,
                    enterStopDegrees, leaveStopDegrees, sideDeadbandInches};
            for (double v : positive) if (!Double.isFinite(v) || v <= 0) return "invalid solver thresholds";
            if (!Double.isFinite(settleMillis) || settleMillis < 0 || !Double.isFinite(extraTravelDegrees)
                    || extraTravelDegrees < 0 || extraTravelDegrees > 30
                    || enterStopDegrees >= 30 || leaveStopDegrees < enterStopDegrees || leaveStopDegrees >= 30
                    || stateFilterFrames < 1 || stateFilterFrames > 15 || stateFilterFrames % 2 == 0
                    || !Double.isFinite(tipConfirmMillis) || tipConfirmMillis < 0
                    || !Double.isFinite(aimRightInches) || !Double.isFinite(aimDownInches)
                    || !Double.isFinite(aimIntoCellInches)) return "invalid solver configuration";
            return "";
        }
    }

    public static final class Calibration {
        public final int width, height;
        public final double fx, fy, cx, cy;
        private final double[] distortion;

        public Calibration(int width, int height, double fx, double fy, double cx, double cy, double[] distortion) {
            if (width <= 0 || height <= 0 || !Double.isFinite(fx) || !Double.isFinite(fy) || fx <= 0 || fy <= 0
                    || !Double.isFinite(cx) || !Double.isFinite(cy) || distortion.length != 5) {
                throw new IllegalArgumentException("Invalid camera calibration");
            }
            for (double d : distortion) if (!Double.isFinite(d)) throw new IllegalArgumentException("Invalid distortion");
            this.width = width; this.height = height;
            this.fx = fx; this.fy = fy; this.cx = cx; this.cy = cy;
            this.distortion = distortion.clone();
        }

        public double[] distortion() { return distortion.clone(); }

        public Calibration scaledTo(int width, int height) {
            if (Math.abs((double) width / height - (double) this.width / this.height) > 1e-6) {
                throw new IllegalArgumentException("Calibration aspect ratio differs; zoom/crop is unsupported");
            }
            double sx = (double) width / this.width, sy = (double) height / this.height;
            return new Calibration(width, height, fx * sx, fy * sy, cx * sx, cy * sy, distortion);
        }

        /** Brown-Conrady projection of an optical-camera point to raw pixels. */
        public double[] project(double[] p) {
            if (p[2] <= 0.01) return null;
            double x = p[0] / p[2], y = p[1] / p[2], r2 = x * x + y * y;
            double radial = 1 + distortion[0] * r2 + distortion[1] * r2 * r2 + distortion[4] * r2 * r2 * r2;
            double xd = x * radial + 2 * distortion[2] * x * y + distortion[3] * (r2 + 2 * x * x);
            double yd = y * radial + distortion[2] * (r2 + 2 * y * y) + 2 * distortion[3] * x * y;
            double u = fx * xd + cx, v = fy * yd + cy;
            return Double.isFinite(u) && Double.isFinite(v) ? new double[] {u, v} : null;
        }
    }

    public static final class TagObservation {
        public final int id;
        private final double[][] pixels;
        private final double[][] modelCorners;

        public TagObservation(int id, double[][] corners) {
            if (corners.length != 4) throw new IllegalArgumentException("Need four decoded corners");
            this.id = id;
            modelCorners = HiveGeometry.cellForTag(id) == null ? null : HiveGeometry.corners(id);
            pixels = new double[4][2];
            for (int i = 0; i < 4; i++) {
                if (corners[i].length != 2 || !Double.isFinite(corners[i][0]) || !Double.isFinite(corners[i][1])) {
                    throw new IllegalArgumentException("Invalid corner coordinates");
                }
                pixels[i] = corners[i].clone();
            }
        }

        public double[][] corners() {
            double[][] result = new double[4][];
            for (int i = 0; i < 4; i++) result[i] = pixels[i].clone();
            return result;
        }

        boolean usable(Calibration calibration) {
            double sign = 0;
            for (int i = 0; i < 4; i++) {
                double[] a = pixels[i], b = pixels[(i + 1) % 4], c = pixels[(i + 2) % 4];
                if (a[0] < 0 || a[0] >= calibration.width || a[1] < 0 || a[1] >= calibration.height) return false;
                double cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]);
                if (Math.abs(cross) < 1 || (i > 0 && cross * sign <= 0)) return false;
                sign = cross;
            }
            return true;
        }
    }

    public static final class Pose {
        final double[][] rotation;
        final double[] translation;

        public Pose(double[][] rotation, double[] translation) {
            this.rotation = new double[3][];
            for (int i = 0; i < 3; i++) this.rotation[i] = rotation[i].clone();
            this.translation = translation.clone();
        }
    }

    public interface PoseSeeds {
        List<Pose> solve(List<TagObservation> tags, Calibration calibration);
    }

    private static final class Fit {
        double[] parameters;
        double[][] rotation;
        List<TagObservation> inliers;
        double rms;
        double[] focus;
        HiveGeometry.Cell cell;
    }

    private static final class Track {
        double[] angles = new double[0];
        int angleCount, angleNext, pendingFrames;
        double filteredAngle = Double.NaN;
        long time, pendingSince;
        HiveGeometry.Cell cell;
        Hive stable, pending;
    }

    private final Config config;
    private final PoseSeeds seeds;
    private final EnumMap<Alliance, HiveObservation> observations = new EnumMap<>(Alliance.class);
    private final EnumMap<Alliance, Track> tracks = new EnumMap<>(Alliance.class);
    private long lastFrameTime;
    private double lastPitch = Double.NaN, lastRoll = Double.NaN;

    public HiveSolver(Config config, PoseSeeds seeds) {
        this.config = config;
        this.seeds = seeds;
        reset("not started");
    }

    public void reset(String reason) {
        lastFrameTime = 0;
        for (Alliance alliance : Alliance.values()) {
            tracks.put(alliance, new Track());
            observations.put(alliance, HiveObservation.invalid(alliance, reason, 0));
        }
    }

    public void update(List<TagObservation> detections, Calibration calibration, long captureNanos, long nowNanos) {
        String error = config.error();
        if (!error.isEmpty()) { reset(error); return; }
        if (calibration == null) { reset("no camera calibration"); return; }
        if (config.cameraPitchDegrees != lastPitch || config.cameraRollDegrees != lastRoll) {
            reset("camera orientation changed");
            lastPitch = config.cameraPitchDegrees; lastRoll = config.cameraRollDegrees;
        }
        if (captureNanos == 0 || captureNanos > nowNanos || (nowNanos - captureNanos) / 1e6 > config.maxAgeMillis) {
            reset("stale image"); return;
        }
        if (captureNanos == lastFrameTime) { expire(nowNanos); return; }
        if (captureNanos < lastFrameTime) reset("image clock restarted");
        lastFrameTime = captureNanos;
        double[][] mount = mountRotation(config.cameraPitchDegrees, config.cameraRollDegrees);
        for (Alliance alliance : Alliance.values()) {
            List<TagObservation> tags = new ArrayList<>();
            Set<Integer> ids = new HashSet<>();
            boolean duplicate = false;
            for (TagObservation tag : detections) {
                HiveGeometry.Cell cell = HiveGeometry.cellForTag(tag.id);
                if (cell == null || cell.alliance != alliance) continue;
                if (!ids.add(tag.id)) duplicate = true;
                if (tag.usable(calibration)) tags.add(tag);
            }
            tags.sort(Comparator.comparingInt(t -> t.id));
            if (duplicate || tags.isEmpty()) {
                invalidate(alliance, duplicate ? "duplicate tag IDs" : "no usable tag corners", captureNanos);
                continue;
            }
            solve(alliance, tags, calibration, mount, captureNanos);
        }
    }

    public void expire(long nowNanos) {
        for (Alliance alliance : Alliance.values()) {
            HiveObservation observation = observations.get(alliance);
            if (observation.captureTimeNanos != 0 && observation.imageAgeMillis(nowNanos) > config.maxAgeMillis) {
                invalidate(alliance, "stale image", observation.captureTimeNanos);
            }
        }
    }

    public HiveObservation getObservation(Alliance alliance, long nowNanos) {
        // Invalidate a processed frame if configuration changes, but preserve an
        // explicit reset reason (e.g. native/calibration failure) until the next update.
        if (lastFrameTime != 0) {
            String error = config.error();
            if (!error.isEmpty()) reset(error);
        }
        expire(nowNanos);
        return observations.get(alliance);
    }

    private void invalidate(Alliance alliance, String reason, long captureNanos) {
        tracks.put(alliance, new Track());
        observations.put(alliance, HiveObservation.invalid(alliance, reason, captureNanos));
    }

    private void solve(Alliance alliance, List<TagObservation> tags, Calibration calibration, double[][] mount, long time) {
        List<Pose> hypotheses = new ArrayList<>(seeds.solve(tags, calibration));
        if (tags.size() > 1) for (TagObservation tag : tags) {
            hypotheses.addAll(seeds.solve(Collections.singletonList(tag), calibration));
        }
        List<Fit> fits = new ArrayList<>();
        for (Pose hypothesis : hypotheses) {
            Fit fit = fit(hypothesis, tags, calibration, mount);
            if (fit != null) fits.add(fit);
        }
        // Re-solve each consensus set jointly to recover both of its planar branches.
        Set<String> jointSets = new HashSet<>();
        for (Fit consensus : new ArrayList<>(fits)) {
            StringBuilder key = new StringBuilder();
            for (TagObservation tag : consensus.inliers) key.append(tag.id).append(',');
            if (!jointSets.add(key.toString())) continue;
            for (Pose joint : seeds.solve(consensus.inliers, calibration)) {
                Fit fitted = fit(joint, tags, calibration, mount);
                if (fitted != null) fits.add(fitted);
            }
        }
        fits.sort(Comparator.<Fit>comparingInt(f -> -f.inliers.size()).thenComparingDouble(f -> f.rms));
        if (fits.isEmpty()) { invalidate(alliance, "no physical pose within reprojection limits", time); return; }
        Fit best = fits.get(0);
        if (best.inliers.size() == 1 && !config.allowSingleTag) {
            invalidate(alliance, "multiple tags required", time); return;
        }
        double margin = Double.POSITIVE_INFINITY;
        for (int i = 1; i < fits.size(); i++) {
            Fit other = fits.get(i);
            if (other.inliers.size() != best.inliers.size() || !materiallyDifferent(best, other)) continue;
            margin = Math.min(margin, other.rms - best.rms);
        }
        List<Integer> inlierIds = new ArrayList<>();
        for (TagObservation tag : best.inliers) inlierIds.add(tag.id);
        String reason = "";
        Hive state = null;
        double stateAngle = Double.NaN;
        if (margin < config.ambiguityGapPixels) {
            reason = "ambiguous planar pose";
            tracks.put(alliance, new Track());
        } else {
            Track track = tracks.get(alliance);
            if (track.time != 0 && (time - track.time) / 1e6 > config.maxAgeMillis) track = new Track();
            tracks.put(alliance, track);
            state = state(track, best.cell, best.parameters[4], time);
            stateAngle = track.filteredAngle;
            if (state == null) reason = track.pending == Hive.TIPPING ? "confirming tip" : "settling at stop";
        }
        observations.put(alliance, new HiveObservation(alliance, reason, best.cell, state, time, inlierIds,
                tags.size(), best.rms, margin, Math.toDegrees(best.parameters[4]), stateAngle, best.focus));
    }

    private boolean materiallyDifferent(Fit a, Fit b) {
        if (a.cell != b.cell || geometricState(a) != geometricState(b)) return true;
        double[] delta = new double[3];
        for (int i = 0; i < 3; i++) delta[i] = a.focus[i] - b.focus[i];
        if (HiveMath.norm(delta) > config.materialFocusDifferenceInches * HiveGeometry.INCH) return true;
        double ba = Math.atan2(-a.focus[0], a.focus[2]), bb = Math.atan2(-b.focus[0], b.focus[2]);
        double ea = Math.atan2(-a.focus[1], Math.hypot(a.focus[0], a.focus[2]));
        double eb = Math.atan2(-b.focus[1], Math.hypot(b.focus[0], b.focus[2]));
        return Math.toDegrees(Math.abs(HiveMath.wrap(ba - bb))) > config.materialAngleDifferenceDegrees
                || Math.toDegrees(Math.abs(ea - eb)) > config.materialAngleDifferenceDegrees;
    }

    private Hive geometricState(Fit fit) {
        double a = Math.toDegrees(fit.parameters[4]) * fit.cell.side;
        if (Math.abs(a - 30) <= config.leaveStopDegrees) return Hive.READY;
        if (Math.abs(a + 30) <= config.leaveStopDegrees) return Hive.TIPPED;
        return Hive.TIPPING;
    }

    private Hive state(Track track, HiveGeometry.Cell cell, double angle, long time) {
        if (track.cell != cell || track.angles.length != config.stateFilterFrames) {
            track.angles = new double[config.stateFilterFrames];
            track.angleCount = 0; track.angleNext = 0;
            track.pending = null; track.stable = null; track.pendingFrames = 0;
        }
        track.cell = cell; track.time = time;
        track.angles[track.angleNext] = Math.toDegrees(angle);
        track.angleNext = (track.angleNext + 1) % track.angles.length;
        track.angleCount = Math.min(track.angleCount + 1, track.angles.length);
        double[] sorted = Arrays.copyOf(track.angles, track.angleCount);
        Arrays.sort(sorted);
        track.filteredAngle = (sorted[(sorted.length - 1) / 2] + sorted[sorted.length / 2]) / 2;
        double a = track.filteredAngle * cell.side;
        double readyTolerance = track.stable == Hive.READY ? config.leaveStopDegrees : config.enterStopDegrees;
        double tippedTolerance = track.stable == Hive.TIPPED ? config.leaveStopDegrees : config.enterStopDegrees;
        Hive candidate = Math.abs(a - 30) <= readyTolerance ? Hive.READY
                : Math.abs(a + 30) <= tippedTolerance ? Hive.TIPPED : Hive.TIPPING;
        if (candidate == track.stable) {
            track.pending = null; track.pendingFrames = 0;
            return track.stable;
        }
        if (track.pending != candidate) {
            track.pending = candidate; track.pendingSince = time; track.pendingFrames = 0;
        }
        track.pendingFrames++;
        double confirmMillis = candidate == Hive.TIPPING ? config.tipConfirmMillis : config.settleMillis;
        // A differentiated noisy pose is not evidence of tipping. Require a
        // persistent position change across distinct frames in both directions.
        if (confirmMillis == 0 || (track.pendingFrames >= 3 && (time - track.pendingSince) / 1e6 >= confirmMillis)) {
            track.stable = candidate;
            track.pending = null; track.pendingFrames = 0;
        }
        return track.stable;
    }

    private Fit fit(Pose pose, List<TagObservation> tags, Calibration calibration, double[][] mount) {
        double[] parameters = parameters(pose, mount);
        if (!finite(parameters)) return null;
        parameters = refine(parameters, tags, calibration, mount, true);
        if (parameters == null) return null;
        List<TagObservation> inliers = new ArrayList<>();
        for (TagObservation tag : tags) {
            double[] e = residuals(parameters, Collections.singletonList(tag), calibration, mount);
            if (e != null && rms(e) <= config.maxTagRmsPixels) inliers.add(tag);
        }
        if (inliers.isEmpty()) return null;
        parameters = refine(parameters, inliers, calibration, mount, false);
        if (parameters == null) return null;
        // A whole tag must remain consistent after the final fit.
        for (TagObservation tag : inliers) {
            double[] e = residuals(parameters, Collections.singletonList(tag), calibration, mount);
            if (e == null || rms(e) > config.maxTagRmsPixels) return null;
        }
        double[] errors = residuals(parameters, inliers, calibration, mount);
        if (errors == null || rms(errors) > config.maxRmsPixels) return null;
        double[][] rotation = rotation(parameters, mount);
        double[] translation = {parameters[0], parameters[1], parameters[2]};
        double[] camera = HiveMath.multiply(HiveMath.transpose(rotation), translation);
        for (int i = 0; i < 3; i++) camera[i] = -camera[i];
        // All tags face away from +Z. Looking at their back is physically invalid.
        if (camera[2] >= 0) return null;
        double[] upCamera = HiveMath.multiply(mount, new double[] {0, -1, 0});
        double[] upHive = HiveMath.multiply(HiveMath.transpose(rotation), upCamera);
        double[] sideDirection = {-upHive[0] * upHive[1], 1 - upHive[1] * upHive[1], -upHive[2] * upHive[1]};
        double length = HiveMath.norm(sideDirection);
        double sideDistance = (camera[0] * sideDirection[0] + camera[1] * sideDirection[1]
                + (camera[2] - HiveGeometry.PIVOT_HEIGHT) * sideDirection[2]) / length;
        if (!Double.isFinite(sideDistance) || Math.abs(sideDistance) < config.sideDeadbandInches * HiveGeometry.INCH) return null;
        Alliance alliance = HiveGeometry.cellForTag(inliers.get(0).id).alliance;
        HiveGeometry.Cell cell = HiveGeometry.cellForSide(alliance, sideDistance < 0 ? -1 : 1);
        double[] focus = HiveMath.transform(rotation, translation,
                cell.focus(config.aimRightInches, config.aimDownInches, config.aimIntoCellInches));
        if (focus[2] <= 0 || !finite(focus)) return null;
        Fit fit = new Fit();
        fit.parameters = parameters; fit.rotation = rotation; fit.inliers = inliers;
        fit.rms = rms(errors); fit.cell = cell; fit.focus = focus;
        return fit;
    }

    static double[][] mountRotation(double pitchDegrees, double rollDegrees) {
        return HiveMath.multiply(HiveMath.rz(-Math.toRadians(rollDegrees)), HiveMath.rx(-Math.toRadians(pitchDegrees)));
    }

    static double[][] rotation(double[] p, double[][] mount) {
        return HiveMath.multiply(mount, HiveMath.multiply(HiveMath.ry(p[3]), HiveMath.rx(Math.PI / 2 + p[4])));
    }

    private double[] parameters(Pose pose, double[][] mount) {
        double[][] level = HiveMath.multiply(HiveMath.transpose(mount), pose.rotation);
        double yaw = Math.atan2(-level[2][0], level[0][0]);
        double[][] untilted = HiveMath.multiply(HiveMath.ry(-yaw), level);
        double tilt = HiveMath.wrap(Math.atan2(untilted[2][1], untilted[1][1]) - Math.PI / 2);
        return new double[] {pose.translation[0], pose.translation[1], pose.translation[2], yaw, tilt};
    }

    private double[] residuals(double[] p, List<TagObservation> tags, Calibration calibration, double[][] mount) {
        double[][] r = rotation(p, mount);
        double[] t = {p[0], p[1], p[2]};
        double[] residuals = new double[tags.size() * 8];
        int i = 0;
        for (TagObservation tag : tags) for (int k = 0; k < 4; k++) {
            double[] projected = calibration.project(HiveMath.transform(r, t, tag.modelCorners[k]));
            if (projected == null) return null;
            residuals[i++] = projected[0] - tag.pixels[k][0];
            residuals[i++] = projected[1] - tag.pixels[k][1];
        }
        return finite(residuals) ? residuals : null;
    }

    private double[] refine(double[] initial, List<TagObservation> tags, Calibration calibration, double[][] mount, boolean robust) {
        double[] p = initial.clone();
        double bound = HiveGeometry.STOP_RADIANS + Math.toRadians(config.extraTravelDegrees);
        if (Math.abs(p[4]) > bound + Math.toRadians(15)) return null;
        p[4] = Math.max(-bound, Math.min(bound, p[4]));
        double[] e = residuals(p, tags, calibration, mount);
        if (e == null) return null;
        double lambda = 1e-3;
        for (int iteration = 0; iteration < 35; iteration++) {
            double[][] j = new double[e.length][5];
            for (int k = 0; k < 5; k++) {
                double[] shifted = p.clone(); shifted[k] += 1e-5;
                double[] next = residuals(shifted, tags, calibration, mount);
                if (next == null) return null;
                for (int i = 0; i < e.length; i++) j[i][k] = (next[i] - e[i]) / 1e-5;
            }
            double[][] a = new double[5][5];
            double[] b = new double[5];
            for (int i = 0; i < e.length; i++) {
                int pair = i & ~1;
                double magnitude = Math.hypot(e[pair], e[pair + 1]);
                double w = robust && magnitude > config.huberPixels ? config.huberPixels / magnitude : 1;
                for (int k = 0; k < 5; k++) {
                    b[k] -= w * j[i][k] * e[i];
                    for (int l = 0; l < 5; l++) a[k][l] += w * j[i][k] * j[i][l];
                }
            }
            for (int k = 0; k < 5; k++) a[k][k] += lambda * (a[k][k] + 1e-6);
            double[] step = HiveMath.solve(a, b);
            if (step == null || !finite(step)) return null;
            if (HiveMath.norm(step) < 1e-7) break;
            double[] trial = p.clone();
            for (int k = 0; k < 5; k++) trial[k] += step[k];
            trial[3] = HiveMath.wrap(trial[3]);
            trial[4] = Math.max(-bound, Math.min(bound, trial[4]));
            double[] next = residuals(trial, tags, calibration, mount);
            if (next != null && cost(next, robust) < cost(e, robust)) {
                p = trial; e = next; lambda = Math.max(1e-8, lambda / 3);
                if (HiveMath.norm(step) < 1e-7) break;
            } else {
                lambda *= 10;
                if (lambda > 1e8) break;
            }
        }
        return p;
    }

    private double cost(double[] e, boolean robust) {
        double cost = 0;
        for (int i = 0; i < e.length; i += 2) {
            double d = Math.hypot(e[i], e[i + 1]);
            cost += robust && d > config.huberPixels ? 2 * config.huberPixels * d - config.huberPixels * config.huberPixels : d * d;
        }
        return cost;
    }

    private static double rms(double[] e) {
        double sum = 0;
        for (double v : e) sum += v * v;
        return Math.sqrt(sum / (e.length / 2.0));
    }

    private static boolean finite(double[] a) {
        for (double v : a) if (!Double.isFinite(v)) return false;
        return true;
    }
}
