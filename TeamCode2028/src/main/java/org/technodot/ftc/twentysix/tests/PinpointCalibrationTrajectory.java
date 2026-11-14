package org.technodot.ftc.twentysix.tests;

/** Independent frequencies separate translation from yaw and world-frame drift. */
final class PinpointCalibrationTrajectory {
    final double x, y, vx, vy;

    PinpointCalibrationTrajectory(double seconds) {
        double ramp = Math.min(1, Math.max(0, seconds / 2));
        double amplitude = 100 * ramp * ramp * (3 - 2 * ramp);
        double derivative = seconds > 0 && seconds < 2 ? 300 * ramp * (1 - ramp) : 0;
        double wx = 2 * Math.PI * .11, wy = 2 * Math.PI * .17;
        x = amplitude * Math.sin(wx * seconds);
        y = amplitude * Math.sin(wy * seconds);
        vx = derivative * Math.sin(wx * seconds) + amplitude * wx * Math.cos(wx * seconds);
        vy = derivative * Math.sin(wy * seconds) + amplitude * wy * Math.cos(wy * seconds);
    }
}
