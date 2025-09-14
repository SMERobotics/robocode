package com.mech.ftc.twentyfive.defaults;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Launcher {
    Velocity v;
    Camera camera;
    public Launcher(HardwareMap hardwareMap) {
        v = new Velocity(hardwareMap);
        camera = new Camera();
    }
    public double launchPower() {
        double y = 1; //change with 1.17 meters - height of launcher in meters;
        double x = camera.getTagDistance();
        double u = v.getForwardVelocity();
        double maxInitialSpeed = 1321; // calculate in m/s
        double angle = Math.toRadians(45); // angle of launcher in degrees change later
        double g = 9.8;
        double initialNeeded = initialNeeded(x, y, u, angle, g);
        //0 = initialYNeeded^2 + 2*-9.8*y
        //19.6y = initialYNeeded^2
        // y = initialYNeeded^2/19.6
        if (initialNeeded < 0) {
            return 0;
        }
        if ((Math.pow(initialNeeded*Math.sin(angle), 2))/(2*g) < y) {
            return 0;
        }
        return (initialNeeded/maxInitialSpeed);


    }
    public double initialNeeded(double x, double y, double u, double angle, double g) {
        double A = Math.cos(angle) * (y * Math.cos(angle) - x * Math.sin(angle));
        double B = u * (2 * y * Math.cos(angle) - x * Math.sin(angle));
        double C = y * u * u + .5 * g * x * x;

        // if A is really small then it won't work
        if (Math.abs(A) < 1e-12) {
            return 0;
        }

        double insideSquareRoot = B * B - 4 * A * C;
        if (insideSquareRoot > 0) {
            double one = (-B + Math.sqrt(insideSquareRoot)) / (2 * A);
            double two = (-B - Math.sqrt(insideSquareRoot)) / (2 * A);
            double best = Double.POSITIVE_INFINITY;
            if (one > 0.0) best = Math.min(best, one);
            if (two > 0.0) best = Math.min(best, two);
            if (!Double.isFinite(best)) return 0.0;

            // verify positive flight time
            double three = best * Math.cos(angle) + u;
            if (three <= 0.0) return 0.0;
            double t = x / three;
            if (t <= 0.0) return 0.0;

            return best;
        }
        return 0;
    }

}
