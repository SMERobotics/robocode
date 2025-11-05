package com.technodot.ftc.twentyfive.robocore;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technodot.ftc.twentyfive.common.Controls;
import com.technodot.ftc.twentyfive.common.Toggle;
import com.technodot.ftc.twentyfive.common.PIDController;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class DeviceDrive extends Device {

    public DcMotorEx motorFrontLeft;
    public DcMotorEx motorFrontRight;
    public DcMotorEx motorBackLeft;
    public DcMotorEx motorBackRight;

    public float speedMultiplier = 1.0F;
    public AprilTagDetection currentTag = null;

    public Toggle preciseToggle = new Toggle();
    public Toggle aimToggle = new Toggle();

    private static final float DEADZONE = 0.02f;
    private static final float ACTIVATION = 0.2f;

    // PID constants for tag yaw aiming (units: degrees -> rotate power)
    private static final float AIM_KP = 0.03f; // proportional gain per degree
    private static final float AIM_KI = 0.0f;  // integral gain (start at 0 to avoid windup)
    private static final float AIM_KD = 0.002f; // derivative gain per (degree/second)
    private static final float AIM_MAX_OUTPUT = 0.6f; // cap rotation while aiming
    private static final float AIM_TOLERANCE_DEG = 1.0f; // within this yaw, consider aimed
    private static final float AIM_INTEGRAL_LIMIT = 1.0f; // anti-windup clamp for integral term

    // Shared PID controller for aiming
    private PIDController aimPid;

    @Override
    public void init(HardwareMap hardwareMap) {
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight");

        // toggle all of them to change robot drive direction
        motorFrontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorEx.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorEx.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorEx.Direction.REVERSE);

        // configure PID controller for yaw aiming
        aimPid = new PIDController(AIM_KP, AIM_KI, AIM_KD);
        aimPid.setSetpoint(0.0);
        aimPid.setOutputLimits(-AIM_MAX_OUTPUT, AIM_MAX_OUTPUT);
        aimPid.setIntegralLimits(-AIM_INTEGRAL_LIMIT, AIM_INTEGRAL_LIMIT);
    }

    @Override
    public void start() {

    }

    public void update(Gamepad gamepad) {
        float forward = Controls.driveForward(gamepad);
        float strafe = Controls.driveStrafe(gamepad);
        float rotate = Controls.driveRotate(gamepad);

        if (Math.abs(forward) < DEADZONE) forward = 0f;
        if (Math.abs(strafe) < DEADZONE) strafe = 0f;
        if (Math.abs(rotate) < DEADZONE) rotate = 0f;

        preciseToggle.update(Controls.drivePrecise(gamepad));
        if (preciseToggle.getState()) {
            setMultiplier(0.5F);
        } else {
            resetMultiplier();
        }

        aimToggle.update(Controls.driveAim(gamepad));
        if (rotate != 0f) {
            aimToggle.reset();
            if (aimPid != null) aimPid.reset();
        }
        if (aimToggle.getState()) {
            // Implement PID aiming with bearing of currentTag
            // Overwrite the rotate value
            if (currentTag != null && currentTag.ftcPose != null) {
                float yawDeg = (float) currentTag.ftcPose.bearing; // +CCW, degrees

                // If within tolerance, stop and reset PID state
                if (Math.abs(yawDeg) <= AIM_TOLERANCE_DEG) {
                    if (aimPid != null) aimPid.reset();
                    rotate = 0f;
                } else {
                    double nowSec = System.nanoTime() / 1_000_000_000.0;
                    double output = aimPid.calculate(yawDeg, nowSec);
                    rotate = (float) output;
                }
            } else {
                // no tag to aim at; don't spin
                rotate = 0f;
                if (aimPid != null) aimPid.reset();
            }
        } else {
            // aiming not active; clear PID state so it doesn't carry over
            if (aimPid != null) aimPid.reset();
        }

        update(forward, strafe, rotate);
    }

    public void update(float forward, float strafe, float rotate) {
        // robot-centric kinematics
        // TODO: field-centric kinematics
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

    public void updatePose(AprilTagDetection tag) {
        // DeviceCamera handles which team
        currentTag = tag;
    }

    @Override
    public void stop() {

    }

    public void zero() {
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    public void getMultiplier(float multiplier) {
        speedMultiplier = multiplier;
    }

    public void setMultiplier(float multiplier) {
        speedMultiplier = multiplier;
    }

    public void resetMultiplier() {
        speedMultiplier = 1.0F;
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
