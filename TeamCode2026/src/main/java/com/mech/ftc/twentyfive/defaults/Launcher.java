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
        double x = camera.getTagDistance(); //change with distance to target in meters
        double u = v.getForwardVelocity();
        double maxInitialSpeed = 1321; // calculate in m/s
        //y = (1/2) * g * t^2
        // 1 = (1/2) * 9.81 * t^2
        double t = Math.sqrt((2 * y) / 9.81);
        // x = u * t + (1/2) * a * t^2
        // x-(.5*a*t^2) = u*t
        // (x-(.5*a*t^2))/t = u
        // x = u * t + (.5) * 9.81 * t^2
        // 1 = u * t + (.5) * 9.81 * t^2
        // 1 - (.5) * 9.81 * t^2 = u * t
        // (1 - (.5) * 9.81 * t^2) / t = u
        double initialNeeded = ((x - (0.5 * 9.81 * t * t)) / t) - u;
        return (initialNeeded/maxInitialSpeed) * 100; // return percentage of max speed since that is what the motors use


    }
}
