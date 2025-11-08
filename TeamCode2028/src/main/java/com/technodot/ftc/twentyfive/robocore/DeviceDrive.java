package com.technodot.ftc.twentyfive.robocore;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technodot.ftc.twentyfive.common.Controls;
import com.technodot.ftc.twentyfive.common.Toggle;
import com.technodot.ftc.twentyfive.common.PIDController;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

public class DeviceDrive extends Device {

    public DcMotorEx motorFrontLeft;
    public DcMotorEx motorFrontRight;
    public DcMotorEx motorBackLeft;
    public DcMotorEx motorBackRight;

    public float speedMultiplier = 1.0F;
    public AprilTagDetection currentTag = null;
    public long lastTagTime = 0;

    public Toggle preciseToggle = new Toggle();
    public Toggle aimToggle = new Toggle();

    private static final float DEADZONE = 0.02f;
    private static final float ACTIVATION = 0.2f;

    // PID constants for tag yaw aiming (units: degrees -> rotate power)
    private static final float AIM_KP = 0.015f; // proportional gain per degree
    private static final float AIM_KI = 0.0f;  // integral gain (start at 0 to avoid windup)
    private static final float AIM_KD = 0.002f; // derivative gain per (degree/second)
    private static final float AIM_MAX_OUTPUT = 0.6f; // cap rotation while aiming
    private static final float AIM_TOLERANCE_DEG = 1.0f; // within this yaw, consider aimed
    private static final float AIM_INTEGRAL_LIMIT = 1.0f; // anti-windup clamp for integral term

    public static final int DIRECTIONAL_ENCODER_TILE = 1400;
    public static final int ROTATIONAL_ENCODER_REVOLUTION = 4000;
    public static final int MAX_ENCODER_VELOCITY = 2000;

    // Shared PID controller for aiming
    private PIDController aimPid;

    // A list to hold movement requests from various callbacks
    private final List<Movement> movementRequests = new ArrayList<>();

    // Represents a single movement request.
    public static class Movement {
        public final float forward;
        public final float strafe;
        public final float rotate;

        public Movement(float forward, float strafe, float rotate) {
            this.forward = forward;
            this.strafe = strafe;
            this.rotate = rotate;
        }
    }

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
        motorBackRight.setDirection(DcMotorEx.Direction.FORWARD); // ts should be reverse. CHAT MOUNTED IT BACKWARDS WHAT THE FUU
//        motorBackRight.setDirection(DcMotorEx.Direction.REVERSE);

//        motorFrontLeft.setTargetPositionTolerance(10);
//        motorFrontRight.setTargetPositionTolerance(10);
//        motorBackLeft.setTargetPositionTolerance(10);
//        motorBackRight.setTargetPositionTolerance(10);

        // configure PID controller for yaw aiming
        aimPid = new PIDController(AIM_KP, AIM_KI, AIM_KD);
        aimPid.setSetpoint(0.0);
        aimPid.setOutputLimits(-AIM_MAX_OUTPUT, AIM_MAX_OUTPUT);
        aimPid.setIntegralLimits(-AIM_INTEGRAL_LIMIT, AIM_INTEGRAL_LIMIT);
    }

    @Override
    public void start() {
        motorFrontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // will be using REV Control Hub built in PID controllers for now
        // TODO: switch to in-house PID controller implementation
        motorFrontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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
            // THE FUCKING CAMERA IS UPSIDE DOWN
            // BEARING NEGATED AS A RESULT
            if (currentTag != null && currentTag.ftcPose != null) {
                float yawDeg = (float) -currentTag.ftcPose.bearing; // +CCW, degrees

                // If within tolerance, stop and reset PID state
                if (Math.abs(yawDeg) <= AIM_TOLERANCE_DEG) {
                    if (aimPid != null) aimPid.reset();
                    rotate = 0f;
                } else {
                    double output = aimPid.calculate(yawDeg, lastTagTime / 1_000_000_000.0);
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
        lastTagTime = System.nanoTime();
        currentTag = tag;
    }

    public void updatePose(AprilTagDetection tag, long timestampNs) {
        // DeviceCamera handles which team
        lastTagTime = timestampNs;
        currentTag = tag;
    }

    @Override
    public void stop() {
        zero();
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

    /**
     * Applies a movement request to be processed in the next flush.
     * Callbacks can use this to contribute to the robot's movement.
     * @param forward The forward power component.
     * @param strafe The strafe power component.
     * @param rotate The rotational power component.
     */
    public void applyMovement(float forward, float strafe, float rotate) {
        movementRequests.add(new Movement(forward, strafe, rotate));
    }

    /**
     * Flushes all applied movement requests, combining them and sending the
     * result to the motors. This should be called once per control loop.
     */
    public void flushMovement() {
        if (movementRequests.isEmpty()) {
            zero();
            return;
        }

        float totalForward = 0;
        float totalStrafe = 0;
        float totalRotate = 0;

        for (Movement move : movementRequests) {
            totalForward += move.forward;
            totalStrafe += move.strafe;
            totalRotate += move.rotate;
        }

        // TODO: encoder shit here

        int encoderForward = (int) (totalForward * DIRECTIONAL_ENCODER_TILE / 2);
        int encoderStrafe = (int) (totalStrafe * DIRECTIONAL_ENCODER_TILE / 2);
        int encoderRotate = (int) (totalRotate * ROTATIONAL_ENCODER_REVOLUTION / 360);

        // robot-centric kinematics
        int fl = encoderForward + encoderStrafe + encoderRotate;
        int fr = encoderForward - encoderStrafe - encoderRotate;
        int bl = encoderForward - encoderStrafe + encoderRotate;
        int br = encoderForward + encoderStrafe - encoderRotate;

        // normalize
        float max = Math.max(1.0f, Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br))));
        float flNormalized = fl / max;
        float frNormalized = fr / max;
        float blNormalized = bl / max;
        float brNormalized = br / max;

        motorFrontLeft.setTargetPosition(motorFrontLeft.getCurrentPosition() + fl);
        motorFrontRight.setTargetPosition(motorFrontRight.getCurrentPosition() + fr);
        motorBackLeft.setTargetPosition(motorBackLeft.getCurrentPosition() + bl);
        motorBackRight.setTargetPosition(motorBackRight.getCurrentPosition() + br);

        if (motorFrontLeft != null) motorFrontLeft.setVelocity(flNormalized * MAX_ENCODER_VELOCITY);
        if (motorFrontRight != null) motorFrontRight.setVelocity(frNormalized * MAX_ENCODER_VELOCITY);
        if (motorBackLeft != null) motorBackLeft.setVelocity(blNormalized * MAX_ENCODER_VELOCITY);
        if (motorBackRight != null) motorBackRight.setVelocity(brNormalized * MAX_ENCODER_VELOCITY);

        // Clear the list for the next loop cycle.
        movementRequests.clear();
    }

    public void resetMovement() {

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
