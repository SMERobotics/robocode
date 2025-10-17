package com.technodot.ftc.twentyfive.robocore;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class DeviceDrive extends Device {

    public DcMotor motorFrontLeft;
    public DcMotor motorFrontRight;
    public DcMotor motorBackLeft;
    public DcMotor motorBackRight;

    public float speedMultiplier = 1.0F;

    private static final float DEADZONE = 0.02f;
    private static final float ACTIVATION = 0.2f;

    @Override
    public void init(HardwareMap hardwareMap) {
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");

        // toggle all of them to change robot drive direction
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void update(Gamepad gamepad) {
        update(-gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x);
    }

    public void update(float forward, float strafe, float rotate) {
        if (Math.abs(forward) < DEADZONE) forward = 0f;
        if (Math.abs(strafe) < DEADZONE) strafe = 0f;
        if (Math.abs(rotate) < DEADZONE) rotate = 0f;

        // robot-centric kinematics
        // if it ain't broke, don't fix it
        // if it ain't broke, don't even flipping TOUCH it
        float fl = forward + strafe + rotate;
        float fr = forward - strafe - rotate;
        float bl = forward - strafe + rotate;
        float br = forward + strafe - rotate;

        // normalize
        float max = Math.max(1.0f, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max; fr /= max; bl /= max; br /= max;

        fl *= speedMultiplier;
        fr *= speedMultiplier;
        bl *= speedMultiplier;
        br *= speedMultiplier;

        update(fl, fr, bl, br);
    }

    public void update(float fl, float fr, float bl, float br) {
        if (motorFrontLeft != null) motorFrontLeft.setPower(scaleInput(fl));
        if (motorFrontRight != null) motorFrontRight.setPower(scaleInput(fr));
        if (motorBackLeft != null) motorBackLeft.setPower(scaleInput(bl));
        if (motorBackRight != null) motorBackRight.setPower(scaleInput(br));
    }

    public void zero() {
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    private float scaleInput(float value) {
        if (value > 0) {
            return -value * ACTIVATION + value + ACTIVATION;
        } else if (value < 0) {
            return -value * ACTIVATION + value - ACTIVATION;
        } else {
            return 0;
        }
    }
}
