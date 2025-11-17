package com.technodot.ftc.twentyfive.robocore;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technodot.ftc.twentyfive.common.Controls;
import com.technodot.ftc.twentyfive.common.Toggle;
import com.technodot.ftc.twentyfive.common.PIDController;
import com.technodot.ftc.twentyfive.shotsolver.ShotSolver;
import com.technodot.ftc.twentyfive.shotsolver.RelativeRBE;

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
    public float rotationalOffset = 0.0f;
    public float lastRotationalOffset = 0.0f;

    public Toggle preciseToggle = new Toggle();
    public Toggle aimToggle = new Toggle();

    private static final float DEADZONE = 0.02f;
    private static final float ACTIVATION = 0.2f;

    // PID constants for tag yaw aiming (units: degrees -> rotate power)
    private static final float AIM_KP = 0.013f; // proportional gain per degree
    private static final float AIM_KI = 0.0f;  // integral gain (start at 0 to avoid windup)
    private static final float AIM_KD = 0.006f; // derivative gain per (degree/second)
    private static final float AIM_MAX_OUTPUT = 0.6f; // cap rotation while aiming
    private static final float AIM_TOLERANCE_DEG = 1.0f; // within this yaw, consider aimed
    private static final float AIM_INTEGRAL_LIMIT = 1.0f; // anti-windup clamp for integral term

    public static final int DIRECTIONAL_ENCODER_TILE = 1400;
    public static final int ROTATIONAL_ENCODER_REVOLUTION = 4000;
    public static final int MAX_ENCODER_VELOCITY = 2000;

    // Shared PID controller for aiming
    private PIDController aimPid;

    // Custom PID controllers for autonomous movement (one per motor)
    private PIDController flPid;
    private PIDController frPid;
    private PIDController blPid;
    private PIDController brPid;

    // PID constants for motor position control
    private static final double MOTOR_KP = 0.003;
    private static final double MOTOR_KI = 0.0;
    private static final double MOTOR_KD = 0.0001;
    private static final double MOTOR_MAX_POWER = 1.0;

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

        // configure PID controller for yaw aiming
        aimPid = new PIDController(AIM_KP, AIM_KI, AIM_KD);
        aimPid.setSetpoint(0.0);
        aimPid.setOutputLimits(-AIM_MAX_OUTPUT, AIM_MAX_OUTPUT);
        aimPid.setIntegralLimits(-AIM_INTEGRAL_LIMIT, AIM_INTEGRAL_LIMIT);

        // configure PID controllers for autonomous motor control
        flPid = new PIDController(MOTOR_KP, MOTOR_KI, MOTOR_KD);
        flPid.setOutputLimits(-MOTOR_MAX_POWER, MOTOR_MAX_POWER);

        frPid = new PIDController(MOTOR_KP, MOTOR_KI, MOTOR_KD);
        frPid.setOutputLimits(-MOTOR_MAX_POWER, MOTOR_MAX_POWER);

        blPid = new PIDController(MOTOR_KP, MOTOR_KI, MOTOR_KD);
        blPid.setOutputLimits(-MOTOR_MAX_POWER, MOTOR_MAX_POWER);

        brPid = new PIDController(MOTOR_KP, MOTOR_KI, MOTOR_KD);
        brPid.setOutputLimits(-MOTOR_MAX_POWER, MOTOR_MAX_POWER);
    }

    @Override
    public void start() {
        resetMovement();
    }

    public void update(Gamepad gamepad) {
        // If rotational offset is active, ignore user input and execute rotation
        // TODO: ts bot fixed. remove legacy code at some point

//        if (rotationalOffset != 0 && lastRotationalOffset != rotationalOffset) {
//            resetMovement();
//            // Apply the rotational offset as a movement request
//            applyMovement(0, 0, rotationalOffset);
//            // Execute the movement using autonomous system
//            flushMovement();
//
//            lastRotationalOffset = rotationalOffset;
//            return;
//        } else if (rotationalOffset == 0) {
//            lastRotationalOffset = 0;
//        }

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

        if (Controls.driveAim(gamepad)) {
            aimToggle.activate();
        }
        if (rotate != 0f) {
            aimToggle.reset();
            if (aimPid != null) aimPid.reset();
        }
        if (aimToggle.getState()) {
            // Implement PID aiming using ShotSolver to project goal point offset from tag
            if (currentTag != null && currentTag.ftcPose != null) {
                // Use ShotSolver to compute the goal point accounting for tag offsets and yaw
                RelativeRBE goalPose = ShotSolver.projectGoal(currentTag);
                float goalBearingDeg = (float) goalPose.bearing; // bearing to goal point

                // If within tolerance, stop and reset PID state
                if (Math.abs(goalBearingDeg) <= AIM_TOLERANCE_DEG) {
                    if (aimPid != null) aimPid.reset();
                    rotate = 0f;
                } else {
                    double output = aimPid.calculate(goalBearingDeg, lastTagTime / 1_000_000_000.0);
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

    public void updateAim() {
        float rotate = 0f;

        // Implement PID aiming using ShotSolver to project goal point offset from tag
        if (currentTag != null && currentTag.ftcPose != null) {
            // Use ShotSolver to compute the goal point accounting for tag offsets and yaw
            RelativeRBE goalPose = ShotSolver.projectGoal(currentTag);
            float goalBearingDeg = (float) goalPose.bearing; // bearing to goal point

            // If within tolerance, stop and reset PID state
            if (Math.abs(goalBearingDeg) <= AIM_TOLERANCE_DEG) {
                if (aimPid != null) aimPid.reset();
                rotate = 0f;
            } else {
                double output = aimPid.calculate(goalBearingDeg, lastTagTime / 1_000_000_000.0);
                rotate = (float) output;
            }
        } else {
            // no tag to aim at; don't spin
            rotate = 0f;
            if (aimPid != null) aimPid.reset();
        }

        update(0, 0, rotate);
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
     * Uses custom PID controllers instead of REV Control Hub PID.
     */
    public void flushMovement() {
        // Sum all movement requests
        float totalForward = 0;
        float totalStrafe = 0;
        float totalRotate = 0;

        for (Movement move : movementRequests) {
            totalForward += move.forward;
            totalStrafe += move.strafe;
            totalRotate += move.rotate;
        }

        // Convert to encoder targets
        int encoderForward = (int) (totalForward * DIRECTIONAL_ENCODER_TILE / 2);
        int encoderStrafe = (int) (totalStrafe * DIRECTIONAL_ENCODER_TILE / 2);
        int encoderRotate = (int) (totalRotate * ROTATIONAL_ENCODER_REVOLUTION / 360);

        // Robot-centric kinematics - calculate target increments
        int flIncrement = encoderForward + encoderStrafe + encoderRotate;
        int frIncrement = encoderForward - encoderStrafe - encoderRotate;
        int blIncrement = encoderForward - encoderStrafe + encoderRotate;
        int brIncrement = encoderForward + encoderStrafe - encoderRotate;

        // Calculate new target positions
        int flTarget = motorFrontLeft.getTargetPosition() + flIncrement;
        int frTarget = motorFrontRight.getTargetPosition() + frIncrement;
        int blTarget = motorBackLeft.getTargetPosition() + blIncrement;
        int brTarget = motorBackRight.getTargetPosition() + brIncrement;

        // Update internal targets for PID
        motorFrontLeft.setTargetPosition(flTarget);
        motorFrontRight.setTargetPosition(frTarget);
        motorBackLeft.setTargetPosition(blTarget);
        motorBackRight.setTargetPosition(brTarget);

        // Get current positions
        int flCurrent = motorFrontLeft.getCurrentPosition();
        int frCurrent = motorFrontRight.getCurrentPosition();
        int blCurrent = motorBackLeft.getCurrentPosition();
        int brCurrent = motorBackRight.getCurrentPosition();

        // Calculate position errors
//        int flError = flTarget - flCurrent;
//        int frError = frTarget - frCurrent;
//        int blError = blTarget - blCurrent;
//        int brError = brTarget - brCurrent;

        // Update PID setpoints to the target positions
        flPid.setSetpoint(flTarget);
        frPid.setSetpoint(frTarget);
        blPid.setSetpoint(blTarget);
        brPid.setSetpoint(brTarget);

        // Calculate PID outputs (power values)
        double flPower = flPid.calculate(flCurrent);
        double frPower = frPid.calculate(frCurrent);
        double blPower = blPid.calculate(blCurrent);
        double brPower = brPid.calculate(brCurrent);

        // Apply power to motors
        if (motorFrontLeft != null) motorFrontLeft.setPower(flPower);
        if (motorFrontRight != null) motorFrontRight.setPower(frPower);
        if (motorBackLeft != null) motorBackLeft.setPower(blPower);
        if (motorBackRight != null) motorBackRight.setPower(brPower);

        // Clear the list for the next loop cycle
        movementRequests.clear();
    }

    /**
     * Prepares the drive system for autonomous mode with custom PID control.
     * Switches motors to RUN_WITHOUT_ENCODER and resets PID controllers.
     * Call this at the start of autonomous period.
     */
    public void resetMovement() {
        // Switch to RUN_WITHOUT_ENCODER to disable REV Control Hub PID
        motorFrontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Reset encoder positions to 0 for clean autonomous start
        motorFrontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Switch back to RUN_WITHOUT_ENCODER (resetting clears the mode)
        motorFrontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Reset target positions to 0
        motorFrontLeft.setTargetPosition(0);
        motorFrontRight.setTargetPosition(0);
        motorBackLeft.setTargetPosition(0);
        motorBackRight.setTargetPosition(0);

        // Reset custom PID controllers
        if (flPid != null) flPid.reset();
        if (frPid != null) frPid.reset();
        if (blPid != null) blPid.reset();
        if (brPid != null) brPid.reset();

        // Clear any pending movement requests
        movementRequests.clear();
    }

//    public void getRotationalOffset(float offset) {
//        rotationalOffset = offset;
//    }
//
//    public void setRotationalOffset(float offset) {
//        rotationalOffset = offset;
//    }

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
