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
        update(-gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x);
    }

    public void update(float forward, float strafe, float rotate) {
        float frontLeft = forward + strafe + rotate;
        float frontRight = forward - strafe - rotate;
        float backLeft = forward - strafe + rotate;
        float backRight = forward + strafe - rotate;

        float max = Math.max(1.0f, Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)), Math.max(Math.abs(backLeft), Math.abs(backRight))));

        frontLeft = (frontLeft / max) * speedMultiplier;
        frontRight = (frontRight / max) * speedMultiplier;
        backLeft = (backLeft / max) * speedMultiplier;
        backRight = (backRight / max) * speedMultiplier;

        motorFrontLeft.setPower(frontLeft);
        motorFrontRight.setPower(frontRight);
        motorBackLeft.setPower(backLeft);
        motorBackRight.setPower(backRight);
    }

    public void zero() {
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }
}
