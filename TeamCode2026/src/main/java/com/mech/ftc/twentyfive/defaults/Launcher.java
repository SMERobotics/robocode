package com.mech.ftc.twentyfive.defaults;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Launcher {

    public DcMotorEx launcherMotor;
    private final Velocity v;

    // Stability controls
    private boolean enabled = false;
    private double filteredDistance = Double.NaN;
    private final double alpha = 0.3;
    private double lastPower = 0.0;
    private double targetPower = 0.0;
    private final double slew = 0.05;
    private final double minPower = 0.10;
    private final double minRange = 0.5;
    private final double maxRange = 4.0;

    public Launcher(DcMotorEx motor, Velocity one) {
        v = one;
        launcherMotor = motor;
    }
    public void launch(Gamepad gamepad, double distance) {
        setEnabled(gamepad.right_bumper);
        update(distance);
    }

    public void launch(double distance) {
        setEnabled(true);
        update(distance);
    }

    public void setEnabled(boolean on) {
        enabled = on;
        if (!enabled) targetPower = 0.0;
    }

    public void update(double distanceMeters) {
        // Filter distance if valid
        if (Double.isFinite(distanceMeters) && distanceMeters > 0) {
            if (Double.isNaN(filteredDistance)) filteredDistance = distanceMeters;
            else filteredDistance = alpha * distanceMeters + (1 - alpha) * filteredDistance;
        }

        // Compute target power when enabled and in range
        if (enabled && Double.isFinite(filteredDistance)
                && filteredDistance >= minRange && filteredDistance <= maxRange) {

            double p = launchPower(filteredDistance);
            if (p > 0 && Double.isFinite(p)) {
                targetPower = Math.max(minPower, Math.min(1.0, p));
            } else {
                targetPower = 0.0;
            }
        } else {
            targetPower = 0.0;
        }

        // Slew-rate limit to avoid flicker
        double delta = targetPower - lastPower;
        if (delta > slew) delta = slew;
        if (delta < -slew) delta = -slew;
        lastPower += delta;

        launcherMotor.setPower(lastPower);
    }

    public double launchPower(double distanceMeters) {
        double y = 0.789;
        double x = distanceMeters - 2.5*0.0254;
        double u = v.getForwardVelocity();
        double maxInitialSpeed = 7.6;
        double angle = Math.toRadians(70);
        double g = 9.8;

        double initialNeeded = initialNeeded(x, y, u, angle, g);
        if (initialNeeded <= 0) return 0;

        double maxHeight = Math.pow(initialNeeded * Math.sin(angle), 2) / (2 * g);
        if (maxHeight < y) return 0;

        return initialNeeded / maxInitialSpeed;
    }

    public double initialNeeded(double x, double y, double u, double angle, double g) {
        double A = Math.cos(angle) * (y * Math.cos(angle) - x * Math.sin(angle));
        double B = u * (2 * y * Math.cos(angle) - x * Math.sin(angle));
        double C = y * u * u + 0.5 * g * x * x;

        if (Math.abs(A) < 1e-12) return 0;

        double disc = B * B - 4 * A * C;
        if (disc <= 0) return 0;

        double one = (-B + Math.sqrt(disc)) / (2 * A);
        double two = (-B - Math.sqrt(disc)) / (2 * A);
        double best = Double.POSITIVE_INFINITY;
        if (one > 0) best = Math.min(best, one);
        if (two > 0) best = Math.min(best, two);
        if (!Double.isFinite(best)) return 0;

        double vx = best * Math.cos(angle) + u;
        if (vx <= 0) return 0;

        double t = x / vx;
        if (t <= 0) return 0;

        return best;
    }
    public double getTargetPower() { return targetPower; }
}