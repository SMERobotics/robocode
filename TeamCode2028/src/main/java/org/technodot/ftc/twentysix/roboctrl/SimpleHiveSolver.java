package org.technodot.ftc.twentysix.roboctrl;

import org.technodot.ftc.twentysix.common.Alliance;
import org.technodot.ftc.twentysix.common.Hive;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

/** Direct tag-size geometry. Each tag contributes once; its result is trusted. */
public final class SimpleHiveSolver {
    public static class Config {
        public double cameraPitchDegrees = 70;
        public double cameraRollDegrees = 0;
        public double aimRightInches = 0;
        public double aimDownInches = 0;
        public double aimIntoCellInches = 3;
        public double stopThresholdDegrees = 15;
        public double maxAgeMillis = 3000;
    }

    private final Config config;
    private final EnumMap<Alliance, HiveObservation> observations = new EnumMap<>(Alliance.class);

    public SimpleHiveSolver(Config config) { this.config = config; reset("no tags"); }

    public void reset(String reason) {
        for (Alliance alliance : Alliance.values()) observations.put(alliance, HiveObservation.invalid(alliance, reason, 0));
    }

    public void update(List<HiveSolver.TagObservation> tags, HiveSolver.Calibration calibration, long capture, long now) {
        if (calibration == null) { reset("no camera calibration"); return; }
        double[][] mount = HiveSolver.mountRotation(config.cameraPitchDegrees, config.cameraRollDegrees);
        double[][] unmount = HiveMath.transpose(mount);
        for (Alliance alliance : Alliance.values()) {
            List<Integer> ids = new ArrayList<>();
            double[] origin = new double[3], xAxis = new double[3], yAxis = new double[3], zAxis = new double[3];
            for (HiveSolver.TagObservation tag : tags) {
                HiveGeometry.Cell printedCell = HiveGeometry.cellForTag(tag.id);
                if (printedCell == null || printedCell.alliance != alliance) continue;
                double[][] pixels = tag.corners(), rays = new double[4][];
                for (int i = 0; i < 4; i++) rays[i] = HiveMath.multiply(unmount, new double[] {
                        (pixels[i][0] - calibration.cx) / calibration.fx,
                        (pixels[i][1] - calibration.cy) / calibration.fy, 1});
                // Decoded order is BL, BR, TR, TL. The tag's horizontal edges
                // stay horizontal as the hive tips, so their endpoints share a height.
                double[][] bottom = edge(rays[0], rays[1]), top = edge(rays[3], rays[2]);
                if (bottom == null || top == null) continue; // coincident corners have no scale
                double[] right = new double[3], down = new double[3], center = new double[3];
                for (int k = 0; k < 3; k++) {
                    right[k] = bottom[1][k] - bottom[0][k] + top[1][k] - top[0][k];
                    down[k] = bottom[0][k] + bottom[1][k] - top[0][k] - top[1][k];
                    center[k] = (bottom[0][k] + bottom[1][k] + top[0][k] + top[1][k]) / 4;
                }
                unit(right); unit(down);
                double[] normal = {right[1] * down[2] - right[2] * down[1],
                        right[2] * down[0] - right[0] * down[2], right[0] * down[1] - right[1] * down[0]};
                unit(normal);
                double[][] model = HiveGeometry.corners(tag.id);
                double tagX = (model[0][0] + model[1][0]) / 2;
                double tagY = (model[0][1] + model[3][1]) / 2;
                double direction = -printedCell.side;
                for (int k = 0; k < 3; k++) {
                    double x = direction * right[k], y = direction * down[k];
                    origin[k] += center[k] - x * tagX - y * tagY;
                    xAxis[k] += x; yAxis[k] += y; zAxis[k] += normal[k];
                }
                ids.add(tag.id);
            }
            if (ids.isEmpty()) {
                observations.put(alliance, HiveObservation.invalid(alliance, "no tags", capture));
                continue;
            }
            for (int k = 0; k < 3; k++) origin[k] /= ids.size();
            unit(xAxis); unit(yAxis); unit(zAxis);
            double pivotX = origin[0] + zAxis[0] * HiveGeometry.PIVOT_HEIGHT;
            double pivotZ = origin[2] + zAxis[2] * HiveGeometry.PIVOT_HEIGHT;
            int side = -(pivotX * yAxis[0] + pivotZ * yAxis[2]) < 0 ? -1 : 1;
            HiveGeometry.Cell cell = HiveGeometry.cellForSide(alliance, side);
            double tilt = Math.toDegrees(Math.asin(Math.max(-1, Math.min(1, -yAxis[1]))));
            double angle = tilt * side;
            Hive state = angle >= config.stopThresholdDegrees ? Hive.READY
                    : angle <= -config.stopThresholdDegrees ? Hive.TIPPED : Hive.TIPPING;
            double[] modelFocus = cell.focus(config.aimRightInches, config.aimDownInches, config.aimIntoCellInches);
            double[] focus = new double[3];
            for (int k = 0; k < 3; k++) focus[k] = origin[k] + xAxis[k] * modelFocus[0]
                    + yAxis[k] * modelFocus[1] + zAxis[k] * modelFocus[2];
            focus = HiveMath.multiply(mount, focus);
            observations.put(alliance, new HiveObservation(alliance, "", cell, state, capture, ids,
                    ids.size(), Double.NaN, Double.NaN, tilt, tilt, focus));
        }
    }

    public void expire(long now) {
        for (Alliance alliance : Alliance.values()) {
            HiveObservation o = observations.get(alliance);
            if (o.captureTimeNanos != 0 && o.imageAgeMillis(now) > config.maxAgeMillis) {
                observations.put(alliance, HiveObservation.invalid(alliance, "no recent tags", o.captureTimeNanos));
            }
        }
    }

    public HiveObservation getObservation(Alliance alliance, long now) { expire(now); return observations.get(alliance); }

    private static double[][] edge(double[] a, double[] b) {
        double ratio = Math.abs(b[1]) > 1e-9 ? a[1] / b[1] : 0;
        double width = Math.hypot(ratio * b[0] - a[0], ratio * b[2] - a[2]);
        if (ratio <= 0 || width < 1e-9) {
            ratio = 1; // Edge on the camera's horizon: ordinary tag-width range estimate.
            width = Math.sqrt(Math.pow(b[0] - a[0], 2) + Math.pow(b[1] - a[1], 2) + Math.pow(b[2] - a[2], 2));
        }
        if (width < 1e-9) return null;
        double scale = HiveGeometry.TAG_SIZE / width;
        return new double[][] {{a[0] * scale, a[1] * scale, a[2] * scale},
                {b[0] * scale * ratio, b[1] * scale * ratio, b[2] * scale * ratio}};
    }

    private static void unit(double[] vector) {
        double length = Math.max(1e-9, HiveMath.norm(vector));
        for (int k = 0; k < 3; k++) vector[k] /= length;
    }
}
