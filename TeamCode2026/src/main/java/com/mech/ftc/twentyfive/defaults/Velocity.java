package com.mech.ftc.twentyfive.defaults;

import com.qualcomm.robotcore.hardware.DcMotorEx;

@SuppressWarnings("FieldCanBeLocal")
public class Velocity {
    private final double wheelDiameter = .05; // meters
    public DcMotorEx parallelEncoder;
    public DcMotorEx perpendicularEncoder;

    public Velocity(DcMotorEx one, DcMotorEx two) {
        parallelEncoder = one;
        perpendicularEncoder = two;
    }

    private double encoderTicksToLinearVelocity(double radiansPerSecond) {
        double wheelRevolutionPerSecond = radiansPerSecond / 8192.0;
        return wheelRevolutionPerSecond * (Math.PI * wheelDiameter);
    }

    public double getForwardVelocity() {
        return encoderTicksToLinearVelocity(parallelEncoder.getVelocity());
    }

    public double getLateralVelocity() {
        return encoderTicksToLinearVelocity(perpendicularEncoder.getVelocity());
    }

    public double[] getRobotVelocity() {
        return new double[] { getForwardVelocity(), getLateralVelocity() };
    }
}
