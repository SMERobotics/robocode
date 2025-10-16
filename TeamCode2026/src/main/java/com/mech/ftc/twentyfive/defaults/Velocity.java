package com.mech.ftc.twentyfive.defaults;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@SuppressWarnings("FieldCanBeLocal")
public class Velocity {
    private final double wheelDiameter = .05; // meters
    public DcMotorEx parallelEncoder;
    public DcMotorEx perpendicularEncoder;

    public Velocity(HardwareMap hardwareMap) {
        parallelEncoder = hardwareMap.get(DcMotorEx.class, "parallel");
        perpendicularEncoder = hardwareMap.get(DcMotorEx.class, "perpendicular");
    }

    private double encoderTicksToLinearVelocity(double radiansPerSecond) {
        double wheelRadius = wheelDiameter / 2.0;
        return radiansPerSecond * wheelRadius;
    }

    public double getForwardVelocity() {
        return encoderTicksToLinearVelocity(parallelEncoder.getVelocity(AngleUnit.RADIANS));
    }

    public double getLateralVelocity() {
        return encoderTicksToLinearVelocity(perpendicularEncoder.getVelocity(AngleUnit.RADIANS));
    }

    public double[] getRobotVelocity() {
        return new double[] { getForwardVelocity(), getLateralVelocity() };
    }
}
