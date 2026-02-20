package org.technodot.ftc.twentyfivebeta.robocore;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.technodot.ftc.twentyfivebeta.Configuration;
import org.technodot.ftc.twentyfivebeta.common.Alliance;
import org.technodot.ftc.twentyfivebeta.common.Movement;
import org.technodot.ftc.twentyfivebeta.common.Vector2D;
import org.technodot.ftc.twentyfivebeta.roboctrl.DebounceController;
import org.technodot.ftc.twentyfivebeta.roboctrl.InputController;
import org.technodot.ftc.twentyfivebeta.roboctrl.PIDFController;
import org.technodot.ftc.twentyfivebeta.roboctrl.ShotSolver;
import org.technodot.ftc.twentyfivebeta.roboctrl.SignedPIDFController;
import org.technodot.ftc.twentyfivebeta.roboctrl.SilentRunner101;

import java.util.ArrayList;

public class DeviceDrive extends Device {

    public DcMotorEx motorFrontLeft;
    public DcMotorEx motorFrontRight;
    public DcMotorEx motorBackLeft;
    public DcMotorEx motorBackRight;

    public DriveState driveState;
    private AutoControl autoControl; // only takes effect in DriveState.AUTO mode
    private DcMotorEx.RunMode runMode;

    private boolean aiming; // teleop only
    private boolean rotating; // teleop only
    private long lastRotateNs;
    private boolean snapped;
    private DebounceController rotateGamepadDebounce; // specifically for teleop gamepad control
    private DebounceController translateDebounce;
    private DebounceController rotateDebounce; // specifically for AutoControl.IMU_ABSOLUTE

    private SignedPIDFController pinpointPIDF;
    private SignedPIDFController aimPIDF;
    private SignedPIDFController bearingPIDF;

    private boolean rotateLockToggleTriggered;
    private boolean rotateLockToggleActive;

    // below thingys deprecated but its not broken so i'm not removing it

    private PIDFController rotationLockPID;
    private PIDFController forwardPID;
    private PIDFController strafePID;
    private PIDFController rotatePID;

    private ArrayList<Movement> movements = new ArrayList<>();
    public double targetFieldHeading;
    private boolean rotationQueued;
    private double queuedTargetHeading;

    // actually i need to think about how i'm gonna build ts
    public enum DriveState {
        TELEOP,
        AUTO
    }

    public enum AutoControl {
        IDLE,
        ROBOT_TRANSLATE,
        FIELD_TRANSLATE,
        CAMERA_AIM,
        CAMERA_ABSOLUTE,
        IMU_ABSOLUTE
    }

    public DeviceDrive(Alliance alliance) {
        super(alliance);
    }

    @Override
    public void init(HardwareMap hardwareMap, InputController inputController) {
        this.inputController = inputController;
        this.driveState = DriveState.TELEOP;
        this.autoControl = AutoControl.ROBOT_TRANSLATE;

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

        pinpointPIDF = new SignedPIDFController(Configuration.PINPOINT_HEADING_P, Configuration.PINPOINT_HEADING_I, Configuration.PINPOINT_HEADING_D, Configuration.PINPOINT_HEADING_F);
        pinpointPIDF.setSetPoint(0.0);
        pinpointPIDF.setIntegrationBounds(-1.0, 1.0);

        aimPIDF = new SignedPIDFController(Configuration.PINPOINT_AIM_P, Configuration.PINPOINT_AIM_I, Configuration.PINPOINT_AIM_D, Configuration.PINPOINT_AIM_F);
        aimPIDF.setSetPoint(0.0);
        aimPIDF.setIntegrationBounds(-Configuration.DRIVE_AIM_INTEGRATION_BOUNDS, Configuration.DRIVE_AIM_INTEGRATION_BOUNDS);

        bearingPIDF = new SignedPIDFController(Configuration.PINPOINT_BEARING_P, Configuration.PINPOINT_BEARING_I, Configuration.PINPOINT_BEARING_D, Configuration.PINPOINT_BEARING_F);
        bearingPIDF.setSetPoint(0.0);
        bearingPIDF.setIntegrationBounds(-Configuration.DRIVE_AIM_INTEGRATION_BOUNDS, Configuration.DRIVE_AIM_INTEGRATION_BOUNDS);

        // some of below thingys deprecated but its not broken so i'm not removing it

        rotationLockPID = new PIDFController(Configuration.DRIVE_ROTATE_KP, Configuration.DRIVE_ROTATE_KI, Configuration.DRIVE_ROTATE_KD, Configuration.DRIVE_ROTATE_KF);
        rotationLockPID.setSetPoint(0.0);
        rotationLockPID.setIntegrationBounds(-1.0, 1.0);

        forwardPID = new PIDFController(Configuration.DRIVE_FORWARD_KP, Configuration.DRIVE_FORWARD_KI, Configuration.DRIVE_FORWARD_KD, Configuration.DRIVE_FORWARD_KF);
        forwardPID.setSetPoint(0.0);
        forwardPID.setIntegrationBounds(-1.0, 1.0);

        strafePID = new PIDFController(Configuration.DRIVE_STRAFE_KP, Configuration.DRIVE_STRAFE_KI, Configuration.DRIVE_STRAFE_KD, Configuration.DRIVE_STRAFE_KF);
        strafePID.setSetPoint(0.0);
        strafePID.setIntegrationBounds(-1.0, 1.0);

        rotatePID = new PIDFController(Configuration.DRIVE_AIM_KP, Configuration.DRIVE_AIM_KI, Configuration.DRIVE_AIM_KD, Configuration.DRIVE_AIM_KF);
        rotatePID.setSetPoint(0.0);
        rotatePID.setIntegrationBounds(-1.0, 1.0);

        rotateGamepadDebounce = new DebounceController(0.001);
        rotateGamepadDebounce.setpoint = 0.0;

        translateDebounce = new DebounceController(Configuration.DRIVE_MOTOR_CONTROL_TOLERANCE_TICKS, Configuration.DRIVE_MOTOR_CONTROL_DEBOUNCE_MS * 1_000_000L);
        translateDebounce.setpoint = 0.0;

        rotateDebounce = new DebounceController(Configuration.DRIVE_ROTATE_TOLERANCE, Configuration.DRIVE_MOTOR_CONTROL_DEBOUNCE_MS * 1_000_000L);
        rotateDebounce.setpoint = 0.0;
    }

    @Override
    public void start() {
        resetMovement();
        if (driveState == DriveState.TELEOP) {
            setRunMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        lastRotateNs = System.nanoTime();
        rotateLockToggleActive = true;
    }

    @Override
    public void update() {
//        switch (driveState) {
//            case TELEOP:
//                if (Configuration.DEBUG) aimPIDF.setPIDF(Configuration.DRIVE_AIM_KP, Configuration.DRIVE_AIM_KI, Configuration.DRIVE_AIM_KD, Configuration.DRIVE_AIM_KF);
//                if (Configuration.DEBUG) rotationLockPID.setPIDF(Configuration.DRIVE_ROTATE_KP, Configuration.DRIVE_ROTATE_KI, Configuration.DRIVE_ROTATE_KD, Configuration.DRIVE_ROTATE_KF);
//
//                SilentRunner101 ctrl = (SilentRunner101) inputController;
//                double rotateInput = ctrl.driveRotate();
//                long nowish = System.nanoTime();
//
//                if (ctrl.driveAim()) aiming = true; // drive aim can ONLY enable
//                if (DeviceExtake.extakeState == DeviceExtake.ExtakeState.IDLE || DeviceExtake.extakeState == DeviceExtake.ExtakeState.ZERO) aiming = false;
//                boolean tagAvailable = DeviceCamera.goalTagDetection != null && DeviceCamera.goalTagDetection.ftcPose != null;
//
//                rotating = !rotateGamepadDebounce.update(rotateInput, nowish);
//                if (rotating) { // we need to take another snapshot
//                    lastRotateNs = nowish;
//                    snapped = false;
//                } else if (!snapped && nowish > lastRotateNs + Configuration.DRIVE_ROTATE_SNAPSHOT_DELAY_NS) { // if its been a while since last rotate, take ts snapshot
//                    DevicePinpoint.setSnapshotYaw();
//                    snapped = true;
//                }
//
//                // apply the aiming and rotation pid loops
//                double rotate = rotateInput;
//                if (aiming) {
//                    if (rotateInput != 0) {
//                        rotate = rotateInput;
//                    } else if (tagAvailable) {
//                        rotate = calculateAim();
//                    } else {
//                        rotate = rotateInput; // allow manual rotate to find a tag
//                    }
//                } else {
//                    if (aimPIDF != null) aimPIDF.reset();
//                    if (!rotating) { // if we're tryna stay still, we stay the fuck still
////                        rotate = Range.clip(rotationLockPID.calculate(DeviceIMU.getSnapshotYawError()), -1.0, 1.0);
//                        rotate = Range.clip(rotationLockPID.calculate(DevicePinpoint.getSnapshotYawError()), -1.0, 1.0);
//                    }
//                }
//
//                // field-centric kinematics for teleop
//                Vector2D fieldCentric = DevicePinpoint.rotateVector(new Vector2D(ctrl.driveForward(), ctrl.driveStrafe()));
//                this.update(fieldCentric.x, fieldCentric.y, rotate);
//
//                break;
//            case AUTO:
//                switch (autoControl) {
//                    case CAMERA_AIM:
//                        setRunMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//                        this.update(0, 0, calculateAim());
//                        break;
//
//                    case CAMERA_ABSOLUTE:
//                        // TODO: implement camera-based absolute positioning
//
//                        AprilTagDetection tag = DeviceCamera.goalTagDetection;
//
//                        if (tag != null && tag.ftcPose != null) {
//                            this.update(
//                                    Range.clip(forwardPID.calculate(tag.ftcPose.range * Math.cos(Math.toRadians(tag.ftcPose.elevation))), -1.0, 1.0) / 3, // i think i did 2D correctly? irdfk
//                                    Range.clip(strafePID.calculate(tag.ftcPose.bearing), -1.0, 1.0) / 3, // CHECK THE NEGATIVE SIGNS
//                                    // which PID is better? rotate or aim? which coefficients are mroe optimized?
//                                    Range.clip(rotatePID.calculate(tag.ftcPose.yaw), -1.0, 1.0) / 3 // CHECK THE NEGATIVE SINGS
////                                    Range.clip(aimPIDF.calculate(tag.ftcPose.yaw), -1.0, 1.0) / 3 // CHECK THE NEGATIVE SINGS
//                            );
//                        }
//
//                        break;
//
//                    case IMU_ABSOLUTE:
////                        this.update(0, 0, Range.clip(rotationLockPID.calculate(DeviceIMU.calculateYawError(targetFieldHeading)), -1.0, 1.0));
//                        break;
//
//                    case FIELD_TRANSLATE:
//                        executeTranslateMovement(true);
//                        break;
//
//                    case ROBOT_TRANSLATE:
//                    default:
//                        executeTranslateMovement(false);
//                        break;
//                }
//
//                break;
//        }

//        if (Configuration.DEBUG) aimPIDF.setPIDF(Configuration.DRIVE_AIM_KP, Configuration.DRIVE_AIM_KI, Configuration.DRIVE_AIM_KD, Configuration.DRIVE_AIM_KF);
//        if (Configuration.DEBUG) rotationLockPID.setPIDF(Configuration.DRIVE_ROTATE_KP, Configuration.DRIVE_ROTATE_KI, Configuration.DRIVE_ROTATE_KD, Configuration.DRIVE_ROTATE_KF);

//        pinpointPIDF.setPIDF(Configuration.PINPOINT_HEADING_P, Configuration.PINPOINT_HEADING_I, Configuration.PINPOINT_HEADING_D, Configuration.PINPOINT_HEADING_F);
        if (Configuration.DEBUG) pinpointPIDF.setPIDF(Configuration.PINPOINT_HEADING_P, Configuration.PINPOINT_HEADING_I, Configuration.PINPOINT_HEADING_D, Configuration.PINPOINT_HEADING_F);
        if (Configuration.DEBUG) aimPIDF.setPIDF(Configuration.PINPOINT_AIM_P, Configuration.PINPOINT_AIM_I, Configuration.PINPOINT_AIM_D, Configuration.PINPOINT_AIM_F);
        if (Configuration.DEBUG) bearingPIDF.setPIDF(Configuration.PINPOINT_BEARING_P, Configuration.PINPOINT_BEARING_I, Configuration.PINPOINT_BEARING_D, Configuration.PINPOINT_BEARING_F);

//        SilentRunner101 ctrl = (SilentRunner101) inputController;
//        double rotateInput = ctrl.driveRotate();
//        long nowish = System.nanoTime();

        SilentRunner101 ctrl = (SilentRunner101) inputController;
        double rotateInput = ctrl.driveRotate();

        boolean togglePressed = ctrl.driveRotationLockToggle();
        if (togglePressed && !rotateLockToggleTriggered) {
            rotateLockToggleActive = !rotateLockToggleActive;
            rotateLockToggleTriggered = true;

            // when enabling lock, take a fresh snapshot so lock holds "now"
            if (rotateLockToggleActive) {
                DevicePinpoint.setSnapshotYaw();
                if (pinpointPIDF != null) pinpointPIDF.reset();
            }
        } else if (!togglePressed) {
            rotateLockToggleTriggered = false;
        }

        if (ctrl.driveAim()) {
            aiming = true; // drive aim can ONLY enable
        }
        if (DeviceExtake.extakeState == DeviceExtake.ExtakeState.IDLE || DeviceExtake.extakeState == DeviceExtake.ExtakeState.ZERO) {
            aiming = false;
            pinpointPIDF.reset();
        }

        long nowish = System.nanoTime();
        rotating = !rotateGamepadDebounce.update(rotateInput, nowish);
        if (rotating) { // we need to take another snapshot
            lastRotateNs = nowish;
            snapped = false;
        } else if (!snapped && nowish > lastRotateNs + Configuration.DRIVE_ROTATE_SNAPSHOT_DELAY_NS) { // if its been a while since last rotate, take ts snapshot
            DevicePinpoint.setSnapshotYaw();
            snapped = true;
        }

//        // apply the aiming and rotation pid loops
//        if (aiming) {
//            if (rotateInput != 0) {
//                rotate = rotateInput;
//            } else if (DeviceCamera.goalTagDetection != null && DeviceCamera.goalTagDetection.ftcPose != null) {
//                rotate = calculateAim();
//            } else {
//                rotate = rotateInput; // allow manual rotate to find a tag
//            }
//        } else {
//            if (aimPIDF != null) aimPIDF.reset();
//            if (!rotating) { // if we're tryna stay still, we stay the fuck still
////                        rotate = Range.clip(rotationLockPID.calculate(DeviceIMU.getSnapshotYawError()), -1.0, 1.0);
//                rotate = Range.clip(rotationLockPID.calculate(DevicePinpoint.getSnapshotYawError()), -1.0, 1.0);
//            }
//        }

        // apply pinpoint PIDF
        double rotate = rotateInput;
//        double error = ShotSolver.getGoalYawError(DeviceCamera.goalTagDetection, this.alliance);
        if (aiming) {
            if (rotateInput != 0) {
                rotate = rotateInput;
            } else if (aimPIDF != null && DeviceCamera.goalTagDetection != null && DeviceCamera.goalTagDetection.ftcPose != null) {
//                if (Double.isFinite(error) && DeviceExtake.extakeState != DeviceExtake.ExtakeState.DUAL_SHORT) {
                if (DeviceExtake.extakeState != DeviceExtake.ExtakeState.DUAL_SHORT) {
//                    FtcDashboard.getInstance().getTelemetry().addData("aim_e", error);
                    // Camera-absolute yaw error sign is opposite of this SignedPIDF path's expected PV sign.
                    // Keep telemetry as geometric error, but invert only for controller input.
//                    rotate = Range.clip(aimPIDF.calculate(error), -1.0, 1.0);

                    // FUCK THE PINPOINT
                    double error = DeviceCamera.goalTagDetection.ftcPose.bearing + (alliance == Alliance.RED ? 1.0 : -1.0) * ((DeviceIntake.targetSide == DeviceIntake.IntakeSide.LEFT ? Configuration.PINPOINT_ANGLE_SIDE_OFFSET : -Configuration.PINPOINT_ANGLE_SIDE_OFFSET) + Configuration.PINPOINT_ANGLE_OFFSET);
                    FtcDashboard.getInstance().getTelemetry().addData("aim_e", error);

//                    rotate = Range.clip(aimPIDF.calculate(error), -1.0, 1.0);
//                    bearingPIDF.reset();

                    // only when using far bearing, use THIS part
                    rotate = Range.clip(bearingPIDF.calculate(error), -1.0, 1.0);
                    aimPIDF.reset();
                } else {
                    double error = DeviceCamera.goalTagDetection.ftcPose.bearing;
                    error = DeviceCamera.goalTagDetection.ftcPose.bearing;
                    FtcDashboard.getInstance().getTelemetry().addData("aim_e", error);
                    rotate = Range.clip(bearingPIDF.calculate(error), -1.0, 1.0);
                    aimPIDF.reset();
                }
                DevicePinpoint.setSnapshotYaw();
            } else {
                if (pinpointPIDF != null) pinpointPIDF.reset();
                if (aimPIDF != null) aimPIDF.reset();
                if (bearingPIDF != null) bearingPIDF.reset();
                rotate = rotateInput;
            }
        } else if (pinpointPIDF != null) {
            if (rotateLockToggleActive && !rotating) {
                double e = DevicePinpoint.getSnapshotYawError();
//                FtcDashboard.getInstance().getTelemetry().addData("rot_e", error);
                rotate = Range.clip(pinpointPIDF.calculate(e), -1.0, 1.0);
            } else {
                pinpointPIDF.reset();
                aimPIDF.reset();
                bearingPIDF.reset();
            }
        }

        // field-centric kinematics for teleop
        Vector2D fieldCentric = DevicePinpoint.rotateVector(new Vector2D(ctrl.driveForward(), ctrl.driveStrafe()));
        this.update(scaleInput(fieldCentric.x), scaleInput(fieldCentric.y), rotate);
    }

    @Override
    public void stop() {
        aimPIDF.reset();
        pinpointPIDF.reset();
        bearingPIDF.reset();
    }

    public void update(double forward, double strafe, double rotate) {
        // counteract strafe friction
        // **???**
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
            double bearing = tag.ftcPose.bearing + (alliance == Alliance.BLUE ? Configuration.DRIVE_AIM_OFFSET : -Configuration.DRIVE_AIM_OFFSET) + (DeviceIntake.targetSide == DeviceIntake.IntakeSide.LEFT ? -Configuration.DRIVE_AIM_INTAKE_OFFSET : Configuration.DRIVE_AIM_INTAKE_OFFSET);
            if (Configuration.DEBUG) FtcDashboard.getInstance().getTelemetry().addData("b", bearing);
//            double bearing = ShotSolver.projectGoal(new Vector3D(tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z), tag.ftcPose.yaw);

//            if (Math.abs(bearing) < Configuration.DRIVE_AIM_TOLERANCE) {
//                if (aimPIDF != null) aimPIDF.reset();
//                return 0.0;
//            } else {
//                return aimPIDF.calculate(bearing, DeviceCamera.goalTagTimestamp / 1_000_000_000.0);
//            }

            return Range.clip(aimPIDF.calculate(bearing), -1.0, 1.0);
        } else {
//            if (aimPIDF != null) aimPIDF.reset();
            return 0.0;
        }
    }

    /**
     * Set the autonomous control mode. Changes will take effect next update cycle.
     * @param control The AutoControl mode to set.
     */
    public void setAutoControl(AutoControl control) {
        this.autoControl = control;
    }

    /**
     * Get the current autonomous control mode.
     * @return The current AutoControl mode.
     */
    public AutoControl getAutoControl() {
        return autoControl;
    }

    /**
     * Stage a camera aiming command to be processed in next ts update cycle.
     * Does not require an input. Only applies in AUTO mode.
     */
    public void stageAim() {
        this.autoControl = AutoControl.CAMERA_AIM;
    }

    /**
     * Stage a movement command to be processed in next ts update cycle.
     * Resets current AutoControl to ROBOT_TRANSLATE mode!!!
     * If FIELD_TRANSLATE is desired, setAutoControl should follow this call.
     * @param forward robot-centric forward, in feet
     * @param strafe robot-centric strafe, in feet
     * @param rotate robot-centric rotate, in degrees
     */
    public void addMovement(double forward, double strafe, double rotate) {
        movements.add(new Movement(forward, strafe, rotate));
        this.autoControl = AutoControl.ROBOT_TRANSLATE;
    }

    /**
     * Stage a movement command with a custom max velocity override to be processed in next ts update cycle.
     * Resets current AutoControl to ROBOT_TRANSLATE mode!!!
     * @param forward robot-centric forward, in feet
     * @param strafe robot-centric strafe, in feet
     * @param rotate robot-centric rotate, in degrees
     * @param velocity Max motor velocity override (encoder ticks/sec).
     */
    public void addMovement(double forward, double strafe, double rotate, double velocity) {
        movements.add(new Movement(forward, strafe, rotate, velocity));
        this.autoControl = AutoControl.ROBOT_TRANSLATE;
    }

    /**
     * Stage a movement command to be processed in next ts update cycle.
     * @param forward field-centric forward, in feet
     * @param strafe field-centric strafe, in feet
     * @param rotate robot-centric rotate, in degrees
     * @param fieldCentric If true, applies field-centric rotation to the movement vector.
     */
    public void addMovement(double forward, double strafe, double rotate, boolean fieldCentric) {
        movements.add(new Movement(forward, strafe, rotate));
        this.autoControl = fieldCentric ? AutoControl.FIELD_TRANSLATE : AutoControl.ROBOT_TRANSLATE;
    }

    /**
     * Stage a movement command with a custom max velocity override to be processed in next ts update cycle.
     * @param forward field-centric forward, in feet
     * @param strafe field-centric strafe, in feet
     * @param rotate robot-centric rotate, in degrees
     * @param fieldCentric If true, applies field-centric rotation to the movement vector.
     * @param maxVelocityOverride Max motor velocity override (encoder ticks/sec).
     */
    public void addMovement(double forward, double strafe, double rotate, boolean fieldCentric, double maxVelocityOverride) {
        movements.add(new Movement(forward, strafe, rotate, maxVelocityOverride));
        this.autoControl = fieldCentric ? AutoControl.FIELD_TRANSLATE : AutoControl.ROBOT_TRANSLATE;
    }

    /**
     * Stage a movement command to be processed in next ts update cycle.
     * @param movement The movement to add.
     */
    public void addMovement(Movement movement) {
        movements.add(movement);
        this.autoControl = AutoControl.ROBOT_TRANSLATE;
    }

    /**
     * Set the target field heading for IMU_ABSOLUTE control.
     * @param heading The target field heading in degrees.
     */
    public void setTargetFieldHeading(double heading) {
        this.targetFieldHeading = heading;
        this.autoControl = AutoControl.IMU_ABSOLUTE;
    }

    public void setCameraAbsolutePositioning(double range, double bearing, double yaw) {
        forwardPID.setSetPoint(range);
        strafePID.setSetPoint(yaw);
        rotatePID.setSetPoint(bearing);
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
        motorFrontLeft.setTargetPosition(0);
        motorFrontRight.setTargetPosition(0);
        motorBackLeft.setTargetPosition(0);
        motorBackRight.setTargetPosition(0);

        movements.clear();
    }
    
    /**
     * Resynchronize encoder target positions with current positions.
     * Useful if encoders get out of sync due to manual movement.
     */
    public void resyncEncoders() {
        int flCurrent = motorFrontLeft.getCurrentPosition();
        int frCurrent = motorFrontRight.getCurrentPosition();
        int blCurrent = motorBackLeft.getCurrentPosition();
        int brCurrent = motorBackRight.getCurrentPosition();

        motorFrontLeft.setTargetPosition(flCurrent);
        motorFrontRight.setTargetPosition(frCurrent);
        motorBackLeft.setTargetPosition(blCurrent);
        motorBackRight.setTargetPosition(brCurrent);
    }

    /**
     * Check if the drive is busy running the current command (AUTO mode only).
     * @return If the drive is busy.
     */
    public boolean isBusy() {
        return !isReady();
    }

    /**
     * Check if the drive is ready for next command (AUTO mode only).
     * @return If the drive is ready.
     */
    public boolean isReady() {
        switch (autoControl) {
            case ROBOT_TRANSLATE:
            case FIELD_TRANSLATE:
                // Calculate max error across all motors
                int flError = Math.abs(motorFrontLeft.getTargetPosition() - motorFrontLeft.getCurrentPosition());
                int frError = Math.abs(motorFrontRight.getTargetPosition() - motorFrontRight.getCurrentPosition());
                int blError = Math.abs(motorBackLeft.getTargetPosition() - motorBackLeft.getCurrentPosition());
                int brError = Math.abs(motorBackRight.getTargetPosition() - motorBackRight.getCurrentPosition());
                int maxError = Math.max(flError, Math.max(frError, Math.max(blError, brError)));

                // Use debounce controller to check if within tolerance for debounce period
                boolean translationDone = translateDebounce.update(maxError);
                if (translationDone && rotationQueued) {
                    setTargetFieldHeading(queuedTargetHeading);
                    rotationQueued = false;
                    return false;
                }
                return translationDone;
            case CAMERA_AIM:
                if (DeviceCamera.goalTagDetection != null && DeviceCamera.goalTagDetection.ftcPose != null) return Math.abs(DeviceCamera.goalTagDetection.ftcPose.bearing + (alliance == Alliance.BLUE ? Configuration.DRIVE_AIM_OFFSET : -Configuration.DRIVE_AIM_OFFSET) + (DeviceIntake.targetSide == DeviceIntake.IntakeSide.LEFT ? -Configuration.DRIVE_AIM_INTAKE_OFFSET : Configuration.DRIVE_AIM_INTAKE_OFFSET)) <= 2.0;
                return false;
            case CAMERA_ABSOLUTE:
                return false; // TODO
            case IMU_ABSOLUTE:
//                double yawError = Math.abs(DeviceIMU.calculateYawError(targetFieldHeading));
//                return rotateDebounce.update(yawError);
            default:
                return false;
        }
    }

    /**
     * Set the drive state (TELEOP or AUTO). Changes will take effect next update cycle.
     * @param state The DriveState to set.
     */
    public void setDriveState(DriveState state) {
        this.driveState = state;
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

    /**
     * Execute translate movement for AUTO mode.
     * @param fieldCentric If true, applies field-centric rotation to the movement vector.
     */
    private void executeTranslateMovement(boolean fieldCentric) {
        boolean hasMovement = !movements.isEmpty();

        // sum all the queued requests
        double totalForward = 0;
        double totalStrafe = 0;
        double totalRotate = 0;
        double maxVelocity = Configuration.DRIVE_AUTO_MAX_VELOCITY;
        boolean hasVelocityOverride = false;
        double velocityOverride = 0.0;

        for (Movement mvmt : movements) {
            totalForward += mvmt.forward;
            totalStrafe += mvmt.strafe;
            totalRotate += mvmt.rotate;
            if (!Double.isNaN(mvmt.velocity) && mvmt.velocity > 0) {
                if (!hasVelocityOverride) {
                    velocityOverride = mvmt.velocity;
                    hasVelocityOverride = true;
                } else {
                    velocityOverride = Math.min(velocityOverride, mvmt.velocity);
                }
            }
        }

        if (hasVelocityOverride) {
            maxVelocity = velocityOverride;
        }

        // apply field-centric rotation if needed
        if (fieldCentric) {
//            Vector2D fieldVector = DeviceIMU.rotateVector(new Vector2D(totalForward, totalStrafe));
            Vector2D fieldVector = DevicePinpoint.rotateVector(new Vector2D(totalForward, totalStrafe));
            totalForward = fieldVector.x;
            totalStrafe = fieldVector.y;
        }

        // compensate translation for any simultaneous rotation so movement stays field-centric to the start pose
        double compensatedForward = totalForward;
        double compensatedStrafe = totalStrafe;
        double rotationForEncoders = totalRotate;
        if (hasMovement) {
            double thetaRad = Math.toRadians(totalRotate);
            boolean canBlendRotation = Math.abs(thetaRad) <= 1e-6; // tiny rotation, safe to blend
            if (!canBlendRotation) {
                double sinTheta = Math.sin(thetaRad);
                double oneMinusCos = 1 - Math.cos(thetaRad);
                double a = sinTheta / thetaRad;
                double b = oneMinusCos / thetaRad;
                double det = a * a + b * b;
                if (det > 1e-6) {
                    double invDet = 1.0 / det;
                    compensatedForward = (a * totalForward + b * totalStrafe) * invDet;
                    compensatedStrafe = (-b * totalForward + a * totalStrafe) * invDet;
                    canBlendRotation = true;
                }
            }

            if (!canBlendRotation && Math.abs(totalRotate) > 1e-6) {
                // rotation is too large to cleanly combine with translation; queue an IMU-based rotation after the translate step
                rotationQueued = true;
//                queuedTargetHeading = DeviceIMU.yaw + totalRotate;
                queuedTargetHeading = DevicePinpoint.pinpoint.getHeading(AngleUnit.DEGREES) + totalRotate;
                rotationForEncoders = 0;
                compensatedForward = totalForward;
                compensatedStrafe = totalStrafe;
            } else {
                rotationQueued = false;
            }
        }

        // translate feet & degrees into encoder ticks
        int encoderForward = (int) (compensatedForward * Configuration.DRIVE_MOTOR_FORWARD_TILE_TICKS / 2);
        int encoderStrafe = (int) (compensatedStrafe * Configuration.DRIVE_MOTOR_STRAFE_TILE_TICKS / 2);
        int encoderRotate = (int) (rotationForEncoders * Configuration.DRIVE_MOTOR_ROTATE_CIRCLE_TICKS / 360);

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
            flVelocity = (flDistance / maxDistance) * maxVelocity;
            frVelocity = (frDistance / maxDistance) * maxVelocity;
            blVelocity = (blDistance / maxDistance) * maxVelocity;
            brVelocity = (brDistance / maxDistance) * maxVelocity;
        }

        // Set target positions
        motorFrontLeft.setTargetPosition(flTarget);
        motorFrontRight.setTargetPosition(frTarget);
        motorBackLeft.setTargetPosition(blTarget);
        motorBackRight.setTargetPosition(brTarget);

        // tell ts sdk that we wanna run to position
        setRunMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Set motor velocities
        motorFrontLeft.setVelocity(flVelocity);
        motorFrontRight.setVelocity(frVelocity);
        motorBackLeft.setVelocity(blVelocity);
        motorBackRight.setVelocity(brVelocity);

        movements.clear();
    }

    private double scaleInput(double value) {
//        // robot does not begin to move until power overcomes ts weight and friction forces idk
//        // but yeah it doesnt start moving at 0.0 power so we gotta scale it
//        if (value > 0) {
//            return -value * Configuration.DRIVE_MOTOR_ACTIVATION + value + Configuration.DRIVE_MOTOR_ACTIVATION;
//        } else if (value < 0) {
//            return -value * Configuration.DRIVE_MOTOR_ACTIVATION + value - Configuration.DRIVE_MOTOR_ACTIVATION;
//        } else {
//            return 0;
//        }

//        double x = Range.clip(value, -1.0, 1.0);
//        return Range.clip(
//                (x / (2.0 - Math.abs(x))), // f\left(x\right)=\frac{x}{2-\left|x\right|}
//        -1.0, 1.0);

        return value;
    }
}
