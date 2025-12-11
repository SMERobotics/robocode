package com.technodot.ftc.twentyfivebeta.robocore;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technodot.ftc.twentyfivebeta.Configuration;
import com.technodot.ftc.twentyfivebeta.common.Alliance;
import com.technodot.ftc.twentyfivebeta.common.Vector2D;
import com.technodot.ftc.twentyfivebeta.roboctrl.InputController;
import com.technodot.ftc.twentyfivebeta.roboctrl.SilentRunner101;

public class DeviceDrive extends Device {

    public DcMotorEx motorFrontLeft;
    public DcMotorEx motorFrontRight;
    public DcMotorEx motorBackLeft;
    public DcMotorEx motorBackRight;

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
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        SilentRunner101 ctrl = (SilentRunner101) inputController;
        this.update(ctrl.driveForward(), ctrl.driveStrafe(), ctrl.driveRotate());
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
        if (motorFrontLeft != null) motorFrontLeft.setPower(scaleInput(fl));
        if (motorFrontRight != null) motorFrontRight.setPower(scaleInput(fr));
        if (motorBackLeft != null) motorBackLeft.setPower(scaleInput(bl));
        if (motorBackRight != null) motorBackRight.setPower(scaleInput(br));
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
