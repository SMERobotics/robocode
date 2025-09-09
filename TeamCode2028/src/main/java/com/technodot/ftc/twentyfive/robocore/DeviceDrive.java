package com.technodot.ftc.twentyfive.robocore;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DeviceDrive extends Device {

    public DcMotor motorFrontLeft;
    public DcMotor motorFrontRight;
    public DcMotor motorBackLeft;
    public DcMotor motorBackRight;

    public float speedMultiplier = 1.0F;

    @Override
    public void init(HardwareMap hardwareMap) {
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");

        // toggle all of them to change robot drive direction
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void update(Gamepad gamepad) {
        float leftX = gamepad.left_stick_x;
        float leftY = gamepad.left_stick_y;
        float rightX = gamepad.right_stick_x;

        update(leftY, leftX, rightX);
    }

    public void update(float drive, float turn, float strafe) {
        float powerFrontLeft = drive - turn - strafe;
        float powerFrontRight = drive + turn + strafe;
        float powerBackLeft = drive + turn - strafe;
        float powerBackRight = drive - turn + strafe;

        powerFrontLeft *= speedMultiplier;
        powerFrontRight *= speedMultiplier;
        powerBackLeft *= speedMultiplier;
        powerBackRight *= speedMultiplier;

        motorFrontLeft.setPower(powerFrontLeft);
        motorFrontRight.setPower(powerFrontRight);
        motorBackLeft.setPower(powerBackLeft);
        motorBackRight.setPower(powerBackRight);
    }

    public void zero() {
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }
}
