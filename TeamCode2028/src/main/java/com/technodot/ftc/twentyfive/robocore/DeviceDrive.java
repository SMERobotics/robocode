package com.technodot.ftc.twentyfive.robocore;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DeviceDrive extends Device {

    public DcMotor motorFrontLeft;
    public DcMotor motorFrontRight;
    public DcMotor motorBackLeft;
    public DcMotor motorBackRight;

    public float speedMultiplier = 1.0;

    @Override
    public void init(HardwareMap hardwareMap) {
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
    }

    @Override
    public void update(Gamepad gamepad) {
        float leftX = gamepad.left_stick_x;
        float leftY = gamepad.left_stick_y;
        float rightX = gamepad.right_stick_x;

        float powerFrontLeft = leftY - leftX - rightX;
        float powerFrontRight = leftY + leftX + rightX;
        float powerBackLeft = -(leftY + leftX - rightX);
        float powerBackRight = -(leftY - leftX + rightX);

        powerFrontLeft *= speedMultiplier;
        powerFrontRight *= speedMultiplier;
        powerBackLeft *= speedMultiplier;
        powerBackRight *= speedMultiplier;

        motorFrontLeft.setPower(powerFrontLeft);
        motorFrontRight.setPower(powerFrontRight);
        motorBackLeft.setPower(powerBackLeft);
        motorBackRight.setPower(powerBackRight);
    }
}
