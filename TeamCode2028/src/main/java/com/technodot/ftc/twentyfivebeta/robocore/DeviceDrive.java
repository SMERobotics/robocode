package com.technodot.ftc.twentyfivebeta.robocore;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technodot.ftc.twentyfivebeta.Configuration;
import com.technodot.ftc.twentyfivebeta.common.Alliance;
import com.technodot.ftc.twentyfivebeta.roboctrl.InputController;

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

    }

    @Override
    public void stop() {

    }

    public void update(float forward, float strafe, float rotate) {
        // TODO: field-centric kinematics
        float fl = forward + strafe + rotate;
        float fr = forward - strafe - rotate;
        float bl = forward - strafe + rotate;
        float br = forward + strafe - rotate;

        // normalization to 1.0f
        float max = Math.max(1.0f, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max; fr /= max; bl /= max; br /= max;

        // respect speed configuration
        fl *= Configuration.DRIVE_MULTIPLIER;
        fr *= Configuration.DRIVE_MULTIPLIER;
        bl *= Configuration.DRIVE_MULTIPLIER;
        br *= Configuration.DRIVE_MULTIPLIER;

        // safely update motors
        update(fl, fr, bl, br);
    }

    public void update(float fl, float fr, float bl, float br) {
        if (motorFrontLeft != null) motorFrontLeft.setPower(scaleInput(fl));
        if (motorFrontRight != null) motorFrontRight.setPower(scaleInput(fr));
        if (motorBackLeft != null) motorBackLeft.setPower(scaleInput(bl));
        if (motorBackRight != null) motorBackRight.setPower(scaleInput(br));
    }

    private float scaleInput(float value) {
        if (value > 0) {
            return -value * Configuration.DRIVE_ACTIVATION + value + Configuration.DRIVE_ACTIVATION;
        } else if (value < 0) {
            return -value * Configuration.DRIVE_ACTIVATION + value - Configuration.DRIVE_ACTIVATION;
        } else {
            return 0;
        }
    }
}
