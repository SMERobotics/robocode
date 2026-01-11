package com.mech.ftc.twentyfive.defaults;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@SuppressWarnings("FieldCanBeLocal")
public class Launcher {

    public DcMotorEx launcherMotor;
    private final Velocity v;

    private boolean enabled = false;
    private double filteredDistance = Double.NaN;
    private final double alpha = 0.3;

    private double targetFrac = 0.0;
    private final double maxTicksPerSec;

    public Launcher(DcMotorEx motor, Velocity one) {
        v = one;
        launcherMotor = motor;
        maxTicksPerSec = 2500;

        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setEnabled(boolean on) {
        enabled = on;
        if (!enabled) targetFrac = 0;
    }

    public void update(double distanceMeters) {
        if (distanceMeters > 0) {
            if (Double.isNaN(filteredDistance)) filteredDistance = distanceMeters;
            else filteredDistance = alpha * distanceMeters + (1 - alpha) * filteredDistance;
        }

        if (enabled && Double.isFinite(filteredDistance)) {

            double p = launchPower(filteredDistance);
            if (p > 0) {
                targetFrac = p;
            }
            else {
                targetFrac = 0;
            }
        }
        else {
            targetFrac = 0;
        }

        double targetVelocity = targetFrac * maxTicksPerSec;

        if (distanceMeters > 2.3 && targetVelocity != 0 || distanceMeters == 0 && targetVelocity != 0) {
            launcherMotor.setVelocity(targetVelocity+100);
        }
        else {
            launcherMotor.setVelocity(targetVelocity);
        }
    }

    public double launchPower(double distanceMeters) {
        double y = 0.789;
        double x = distanceMeters;
        double u = v.getForwardVelocity();
        double maxInitialSpeed = 8.1;
        double angle = Math.toRadians(50);
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

        double root = Math.sqrt(disc);
        double one = (-B + root) / (2 * A);
        double two = (-B - root) / (2 * A);
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
    public double getTargetVelocity() { return targetFrac*maxTicksPerSec; }
}