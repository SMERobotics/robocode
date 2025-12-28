package org.technodot.ftc.twentyfivebeta.robocore;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.technodot.ftc.twentyfivebeta.Configuration;
import org.technodot.ftc.twentyfivebeta.common.Alliance;
import org.technodot.ftc.twentyfivebeta.common.Vector2D;
import org.technodot.ftc.twentyfivebeta.roboctrl.InputController;
import org.technodot.ftc.twentyfivebeta.roboctrl.PIDController;
import org.technodot.ftc.twentyfivebeta.roboctrl.SilentRunner101;

public class DeviceDrive extends Device {

    public DcMotorEx motorFrontLeft;
    public DcMotorEx motorFrontRight;
    public DcMotorEx motorBackLeft;
    public DcMotorEx motorBackRight;

    private boolean aiming = false;
    private PIDController aimPID;

    public DeviceDrive(Alliance alliance) {
        super(alliance);
    }

    @Override
    public void init(HardwareMap hardwareMap, InputController inputController) {
        this.inputController = inputController;

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

        aimPID = new PIDController(Configuration.DRIVE_AIM_KP, Configuration.DRIVE_AIM_KI, Configuration.DRIVE_AIM_KD);
        aimPID.setSetpoint(0.0);
        aimPID.setOutputLimits(-1.0, 1.0);
        aimPID.setIntegralLimits(-1.0, 1.0);
    }

    @Override
    public void start() {
        resetMovement();
    }

    @Override
    public void update() {
        SilentRunner101 ctrl = (SilentRunner101) inputController;
        if (ctrl.driveAim()) aiming = true; // drive aim can ONLY enable
        double rotate = ctrl.driveRotate();
        if (ctrl.driveRotate() != 0) aiming = false; // rotation can ONLY override
        if (aiming) {
            rotate = calculateAim();
        } else {
            if (aimPID != null) aimPID.reset();
        }
        this.update(ctrl.driveForward(), ctrl.driveStrafe(), rotate);
    }

    @Override
    public void stop() {

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
            double bearing = tag.ftcPose.bearing;

            if (Math.abs(bearing) < Configuration.DRIVE_AIM_TOLERANCE) {
                if (aimPID != null) aimPID.reset();
                return 0.0;
            } else {
                return aimPID.calculate(bearing, DeviceCamera.goalTagTimestamp / 1_000_000_000.0);
            }
        } else {
            if (aimPID != null) aimPID.reset();
            return 0.0;
        }
    }

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
