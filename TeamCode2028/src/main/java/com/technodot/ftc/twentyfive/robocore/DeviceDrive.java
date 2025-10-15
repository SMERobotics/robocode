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

    // small stick deadzone to ignore tiny noise
    private static final float DEADZONE = 0.02f;

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
        // Apply deadzone
        if (Math.abs(forward) < DEADZONE) forward = 0f;
        if (Math.abs(strafe) < DEADZONE) strafe = 0f;
        if (Math.abs(rotate) < DEADZONE) rotate = 0f;

        // Optional: non-linear scaling for finer low-speed control (comment out if undesired)
        forward = scaleInput(forward);
        strafe = scaleInput(strafe);
        rotate = scaleInput(rotate);

        // Basic mecanum kinematics (robot-centric)
        float fl = forward + strafe + rotate;    // Front Left
        float fr = forward - strafe - rotate;    // Front Right
        float bl = forward - strafe + rotate;    // Back Left
        float br = forward + strafe - rotate;    // Back Right

        // Find the maximum magnitude to normalize if needed
        float max = Math.max(1.0f, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max; fr /= max; bl /= max; br /= max;

        // Apply overall speed multiplier
        fl *= speedMultiplier;
        fr *= speedMultiplier;
        bl *= speedMultiplier;
        br *= speedMultiplier;

        // Safety clamp (in case multiplier pushes over 1)
        fl = clamp(fl);
        fr = clamp(fr);
        bl = clamp(bl);
        br = clamp(br);

        // Set motor powers (null checks just in case init not called yet)
        if (motorFrontLeft != null) motorFrontLeft.setPower(fl);
        if (motorFrontRight != null) motorFrontRight.setPower(fr);
        if (motorBackLeft != null) motorBackLeft.setPower(bl);
        if (motorBackRight != null) motorBackRight.setPower(br);
    }

    private float clamp(float v) {
        if (v > 1f) return 1f;
        if (v < -1f) return -1f;
        return v;
    }

    private float scaleInput(float v) {
        // Cubic scaling keeps sign and gives finer control at low speeds
        return v * v * v + 0.0f * v; // easy to tweak later
    }

    public void zero() {
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }
}
