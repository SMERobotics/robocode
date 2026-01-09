package org.technodot.ftc.twentyfivebeta.robocore;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.technodot.ftc.twentyfivebeta.Configuration;
import org.technodot.ftc.twentyfivebeta.common.Alliance;
import org.technodot.ftc.twentyfivebeta.common.Movement;
import org.technodot.ftc.twentyfivebeta.common.Vector2D;
import org.technodot.ftc.twentyfivebeta.roboctrl.InputController;
import org.technodot.ftc.twentyfivebeta.roboctrl.PIDFController;
import org.technodot.ftc.twentyfivebeta.roboctrl.SilentRunner101;

import java.util.ArrayList;

public class DeviceDrive extends Device {

    public DcMotorEx motorFrontLeft;
    public DcMotorEx motorFrontRight;
    public DcMotorEx motorBackLeft;
    public DcMotorEx motorBackRight;

    public static DriveState driveState;
    private DcMotorEx.RunMode runMode;

    private boolean aiming;
    private boolean rotating;
    private long lastRotateNs;
    private boolean snapped;

    private PIDFController aimPID;
    private PIDFController rotatePID;

    private ArrayList<Movement> movements = new ArrayList<>();

    // actually i need to think about how i'm gonna build ts
    public enum DriveState {
        TELEOP,
        AUTO
    }

    public DeviceDrive(Alliance alliance) {
        super(alliance);
    }

    @Override
    public void init(HardwareMap hardwareMap, InputController inputController) {
        this.inputController = inputController;
        this.driveState = DriveState.TELEOP;

        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight");

        // directions verified as of 12-09-2025
        motorFrontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorEx.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorEx.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorEx.Direction.FORWARD);

        motorFrontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        aimPID = new PIDFController(Configuration.DRIVE_AIM_KP, Configuration.DRIVE_AIM_KI, Configuration.DRIVE_AIM_KD, Configuration.DRIVE_AIM_KF);
        aimPID.setSetPoint(0.0);
        aimPID.setIntegrationBounds(-Configuration.DRIVE_AIM_INTEGRATION_BOUNDS, Configuration.DRIVE_AIM_INTEGRATION_BOUNDS);

        rotatePID = new PIDFController(Configuration.DRIVE_ROTATE_KP, Configuration.DRIVE_ROTATE_KI, Configuration.DRIVE_ROTATE_KD, Configuration.DRIVE_ROTATE_KF);
        rotatePID.setSetPoint(0.0);
        rotatePID.setIntegrationBounds(-1.0, 1.0);
    }

    @Override
    public void start() {
        resetMovement();
        if (driveState == DriveState.TELEOP) {
            setRunMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        lastRotateNs = System.nanoTime();
    }

    @Override
    public void update() {
        switch (driveState) {
            case TELEOP:
                // uncommented for testing purposes only
                aimPID.setPIDF(Configuration.DRIVE_AIM_KP, Configuration.DRIVE_AIM_KI, Configuration.DRIVE_AIM_KD, Configuration.DRIVE_AIM_KF);
                rotatePID.setPIDF(Configuration.DRIVE_ROTATE_KP, Configuration.DRIVE_ROTATE_KI, Configuration.DRIVE_ROTATE_KD, Configuration.DRIVE_ROTATE_KF);

                SilentRunner101 ctrl = (SilentRunner101) inputController;
                double rotate = ctrl.driveRotate();

                if (ctrl.driveAim()) aiming = true; // drive aim can ONLY enable
                if (rotate != 0) aiming = false; // rotation can ONLY override

                rotating = rotate != 0;
                long nowish = System.nanoTime();
                if (rotating) { // we need to take another snapshot
                    lastRotateNs = nowish;
                    snapped = false;
                } else if (!snapped && nowish > lastRotateNs + Configuration.DRIVE_ROTATE_SNAPSHOT_DELAY_NS) { // if its been a while since last rotate, take ts snapshot
                    DeviceIMU.setSnapshotYaw();
                    snapped = true;
                }

                if (aiming) {
                    rotate = calculateAim();
                } else {
                    if (aimPID != null) aimPID.reset();
                    if (!rotating) { // if we're tryna stay still, we stay the fuck still
                        rotate = Range.clip(rotatePID.calculate(DeviceIMU.getSnapshotYawError()), -1.0, 1.0);
                    }
                }
                this.update(ctrl.driveForward(), ctrl.driveStrafe(), rotate);
                break;
            case AUTO:
                if (aiming) {
                    setRunMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    this.update(0, 0, calculateAim());
                    aiming = false;
                } else {
                    // sum all the queued requests
                    double totalForward = 0;
                    double totalStrafe = 0;
                    double totalRotate = 0;

                    for (Movement mvmt : movements) {
                        totalForward += mvmt.forward;
                        totalStrafe += mvmt.strafe;
                        totalRotate += mvmt.rotate;
                    }

                    // translate feet & degrees into encoder ticks
                    int encoderForward = (int) (totalForward * Configuration.DRIVE_MOTOR_FORWARD_TILE_TICKS / 2);
                    int encoderStrafe = (int) (totalStrafe * Configuration.DRIVE_MOTOR_STRAFE_TILE_TICKS / 2);
                    int encoderRotate = (int) (totalRotate * Configuration.DRIVE_MOTOR_ROTATE_CIRCLE_TICKS / 360);

                    // calculate target positions for each motor
                    int flIncrement = encoderForward + encoderStrafe + encoderRotate;
                    int frIncrement = encoderForward - encoderStrafe - encoderRotate;
                    int blIncrement = encoderForward - encoderStrafe + encoderRotate;
                    int brIncrement = encoderForward + encoderStrafe - encoderRotate;

                    int flTarget = motorFrontLeft.getTargetPosition() + flIncrement;
                    int frTarget = motorFrontRight.getTargetPosition() + frIncrement;
                    int blTarget = motorBackLeft.getTargetPosition() + blIncrement;
                    int brTarget = motorBackRight.getTargetPosition() + brIncrement;

                    // get current positions
                    int flCurrent = motorFrontLeft.getCurrentPosition();
                    int frCurrent = motorFrontRight.getCurrentPosition();
                    int blCurrent = motorBackLeft.getCurrentPosition();
                    int brCurrent = motorBackRight.getCurrentPosition();

                    // calculate distance each motor needs to travel
                    double flDistance = Math.abs(flTarget - flCurrent);
                    double frDistance = Math.abs(frTarget - frCurrent);
                    double blDistance = Math.abs(blTarget - blCurrent);
                    double brDistance = Math.abs(brTarget - brCurrent);

                    // find the max dist to normalize velocities
                    double maxDistance = Math.max(flDistance, Math.max(frDistance, Math.max(blDistance, brDistance)));

                    // calculate normalized velocities (proportional to distance, capped at max velocity)
                    double flVelocity = 0;
                    double frVelocity = 0;
                    double blVelocity = 0;
                    double brVelocity = 0;

                    if (maxDistance > 0) {
                        flVelocity = (flDistance / maxDistance) * Configuration.DRIVE_AUTO_MAX_VELOCITY;
                        frVelocity = (frDistance / maxDistance) * Configuration.DRIVE_AUTO_MAX_VELOCITY;
                        blVelocity = (blDistance / maxDistance) * Configuration.DRIVE_AUTO_MAX_VELOCITY;
                        brVelocity = (brDistance / maxDistance) * Configuration.DRIVE_AUTO_MAX_VELOCITY;
                    }

                    // Set target positions
                    motorFrontLeft.setTargetPosition(flTarget);
                    motorFrontRight.setTargetPosition(frTarget);
                    motorBackLeft.setTargetPosition(blTarget);
                    motorBackRight.setTargetPosition(brTarget);

                    // Set motor velocities
                    motorFrontLeft.setVelocity(flVelocity);
                    motorFrontRight.setVelocity(frVelocity);
                    motorBackLeft.setVelocity(blVelocity);
                    motorBackRight.setVelocity(brVelocity);

                    // tell ts sdk that we wanna run to position
                    setRunMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                    movements.clear();
                }

                break;
        }
    }

    @Override
    public void stop() {
        aimPID.reset();
        rotatePID.reset();
    }

    public void update(double forward, double strafe, double rotate) {
        // ts field-centric kinematic model
        Vector2D fieldCentric = DeviceIMU.rotateVector(new Vector2D(forward, strafe));
        forward = fieldCentric.x;
        strafe = fieldCentric.y;

        // counteract strafe friction
        strafe *= Configuration.DRIVE_STRAFE_MULTIPLIER;

        // ts mecanum drive kinematic model
        double fl = forward + strafe + rotate;
        double fr = forward - strafe - rotate;
        double bl = forward - strafe + rotate;
        double br = forward + strafe - rotate;

        // normalization to 1.0f
        double max = Math.max(1.0f, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max; fr /= max; bl /= max; br /= max;

        // respect speed configuration
        fl *= Configuration.DRIVE_SPEED_MULTIPLIER;
        fr *= Configuration.DRIVE_SPEED_MULTIPLIER;
        bl *= Configuration.DRIVE_SPEED_MULTIPLIER;
        br *= Configuration.DRIVE_SPEED_MULTIPLIER;

        // safely update motors
        update(fl, fr, bl, br);
    }

    public void update(double fl, double fr, double bl, double br) {
//        if (motorFrontLeft != null) motorFrontLeft.setPower(scaleInput(fl));
//        if (motorFrontRight != null) motorFrontRight.setPower(scaleInput(fr));
//        if (motorBackLeft != null) motorBackLeft.setPower(scaleInput(bl));
//        if (motorBackRight != null) motorBackRight.setPower(scaleInput(br));

        if (motorFrontLeft != null) motorFrontLeft.setPower(fl);
        if (motorFrontRight != null) motorFrontRight.setPower(fr);
        if (motorBackLeft != null) motorBackLeft.setPower(bl);
        if (motorBackRight != null) motorBackRight.setPower(br);
    }
    
    public double calculateAim() {
        AprilTagDetection tag = DeviceCamera.goalTagDetection;

        if (tag != null && tag.ftcPose != null) {
            double bearing = tag.ftcPose.bearing + (alliance == Alliance.BLUE ? Configuration.DRIVE_AIM_OFFSET : -Configuration.DRIVE_AIM_OFFSET);
//            double bearing = ShotSolver.projectGoal(new Vector3D(tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z), tag.ftcPose.yaw);

//            if (Math.abs(bearing) < Configuration.DRIVE_AIM_TOLERANCE) {
//                if (aimPID != null) aimPID.reset();
//                return 0.0;
//            } else {
//                return aimPID.calculate(bearing, DeviceCamera.goalTagTimestamp / 1_000_000_000.0);
//            }

            return Range.clip(aimPID.calculate(bearing), -1.0, 1.0);
        } else {
//            if (aimPID != null) aimPID.reset();
            return 0.0;
        }
    }

    /**
     * Stage an aim command to be processed in next ts update cycle.
     */
    public void stageAim() {
        aiming = true;
    }

    /**
     * Stage a movement command to be processed in next ts update cycle.
     * @param forward robot-centric forward, in feet
     * @param strafe robot-centric strafe, in feet
     * @param rotate robot-centric rotate, in degrees
     */
    public void addMovement(double forward, double strafe, double rotate) {
        movements.add(new Movement(forward, strafe, rotate));
    }

    /**
     * Stage a movement command to be processed in next ts update cycle.
     * @param movement The movement to add.
     */
    public void addMovement(Movement movement) {
        movements.add(movement);
    }

    /**
     * Reset all movement commands and encoder targets.
     */
    public void resetMovement() {
        // Switch to RUN_WITHOUT_ENCODER to disable REV Control Hub PID
//        motorFrontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        motorFrontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        motorBackLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        motorBackRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        setRunMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Reset encoder positions to 0 for clean autonomous start
//        motorFrontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        motorFrontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        motorBackLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        motorBackRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Switch back to RUN_WITHOUT_ENCODER (resetting clears the mode)
//        motorFrontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        motorFrontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        motorBackLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        motorBackRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Reset target positions to 0
//        motorFrontLeft.setTargetPosition(0);
//        motorFrontRight.setTargetPosition(0);
//        motorBackLeft.setTargetPosition(0);
//        motorBackRight.setTargetPosition(0);

        movements.clear();
    }

    /**
     * Set the drive state (TELEOP or AUTO). Changes will take effect next update cycle.
     * @param state The DriveState to set.
     */
    public void setDriveState(DriveState state) {
        this.driveState = state;
    }

    /**
     * Check if any drive motors are busy (AUTO mode only).
     * @return If any drive motors are busy.
     */
    public boolean isBusy() {
        if (aiming && DeviceCamera.goalTagDetection != null && DeviceCamera.goalTagDetection.ftcPose != null) return DeviceCamera.goalTagDetection.ftcPose.bearing + (alliance == Alliance.BLUE ? Configuration.DRIVE_AIM_OFFSET : -Configuration.DRIVE_AIM_OFFSET) >= Configuration.DRIVE_AIM_TOLERANCE;
        return motorFrontLeft.isBusy() || motorFrontRight.isBusy() || motorBackLeft.isBusy() || motorBackRight.isBusy();
    }

    /**
     * Check if the drive is ready for next command (AUTO mode only).
     * @return If the drive is ready.
     */
    public boolean isReady() {
        return !isBusy();
    }

    /**
     * Set the run mode for all drive motors.
     * @param mode The DcMotorEx.RunMode to set.
     */
    private void setRunMode(DcMotorEx.RunMode mode) {
        if (this.runMode == mode) return;
        this.runMode = mode;
        motorFrontLeft.setMode(mode);
        motorFrontRight.setMode(mode);
        motorBackLeft.setMode(mode);
        motorBackRight.setMode(mode);
    }

    private double scaleInput(double value) {
        // robot does not begin to move until power overcomes ts weight and friction forces idk
        // but yeah it doesnt start moving at 0.0 power so we gotta scale it
        if (value > 0) {
            return -value * Configuration.DRIVE_MOTOR_ACTIVATION + value + Configuration.DRIVE_MOTOR_ACTIVATION;
        } else if (value < 0) {
            return -value * Configuration.DRIVE_MOTOR_ACTIVATION + value - Configuration.DRIVE_MOTOR_ACTIVATION;
        } else {
            return 0;
        }
    }
}
