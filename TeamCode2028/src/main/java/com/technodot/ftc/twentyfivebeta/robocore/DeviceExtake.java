package com.technodot.ftc.twentyfivebeta.robocore;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technodot.ftc.twentyfivebeta.Configuration;
import com.technodot.ftc.twentyfivebeta.common.Alliance;
import com.technodot.ftc.twentyfivebeta.roboctrl.InputController;
import com.technodot.ftc.twentyfivebeta.roboctrl.SilentRunner101;

public class DeviceExtake extends Device {

    public DcMotorEx motorExtakeLeft;
    public DcMotorEx motorExtakeRight;

    boolean prevExtakeClose;
    boolean prevExtakeFar;

    public ExtakeState extakeState = ExtakeState.IDLE;
    public double extakeOverride;

    public enum ExtakeState {
        IDLE,
        SHORT,
        LONG,
        REVERSE,
        ZERO,
        OVERRIDE
    }

    public DeviceExtake(Alliance alliance) {
        super(alliance);
    }

    @Override
    public void init(HardwareMap hardwareMap, InputController inputController) {
        this.inputController = inputController;

        motorExtakeLeft = hardwareMap.get(DcMotorEx.class, "motorExtakeLeft");
        motorExtakeRight = hardwareMap.get(DcMotorEx.class, "motorExtakeRight");

        motorExtakeLeft.setDirection(DcMotorEx.Direction.FORWARD);
        motorExtakeRight.setDirection(DcMotorEx.Direction.REVERSE);

        motorExtakeLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motorExtakeRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

//        motorExtakeLeft.setVelocityPIDFCoefficients(Configuration.EXTAKE_MOTOR_KP, Configuration.EXTAKE_MOTOR_KI, Configuration.EXTAKE_MOTOR_KD, Configuration.EXTAKE_MOTOR_KF);
//        motorExtakeRight.setVelocityPIDFCoefficients(Configuration.EXTAKE_MOTOR_KP, Configuration.EXTAKE_MOTOR_KI, Configuration.EXTAKE_MOTOR_KD, Configuration.EXTAKE_MOTOR_KF);

        extakeState = ExtakeState.IDLE;
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        SilentRunner101 ctrl = (SilentRunner101) inputController;

        boolean extakeFar = ctrl.extakeFar();
        boolean extakeClose = ctrl.extakeClose();

        if (ctrl.extakeReverse()) {
            extakeState = ExtakeState.REVERSE;
        } else if (extakeFar && !prevExtakeFar) {
            extakeState = extakeState == ExtakeState.LONG ? ExtakeState.IDLE : ExtakeState.LONG;
        } else if (extakeClose && !prevExtakeClose) {
            extakeState = extakeState == ExtakeState.SHORT ? ExtakeState.IDLE : ExtakeState.SHORT;
        } else if (extakeState == ExtakeState.REVERSE) {
            extakeState = ExtakeState.ZERO;
        }

        prevExtakeFar = extakeFar;
        prevExtakeClose = extakeClose;

        // PIDF coefficients should be applied only at init
        // temp moved to update cycle so Configuration changes will take effect
        motorExtakeLeft.setVelocityPIDFCoefficients(Configuration.EXTAKE_MOTOR_KP, Configuration.EXTAKE_MOTOR_KI, Configuration.EXTAKE_MOTOR_KD, Configuration.EXTAKE_MOTOR_KF);
        motorExtakeRight.setVelocityPIDFCoefficients(Configuration.EXTAKE_MOTOR_KP, Configuration.EXTAKE_MOTOR_KI, Configuration.EXTAKE_MOTOR_KD, Configuration.EXTAKE_MOTOR_KF);

        switch (extakeState) {
            case IDLE:
                motorExtakeLeft.setPower(0.0);
                motorExtakeRight.setPower(0.0);
                break;
            case SHORT:
                setExtakeVelocity(Configuration.EXTAKE_MOTOR_SPEED_SHORT);
                break;
            case LONG:
                setExtakeVelocity(Configuration.EXTAKE_MOTOR_SPEED_LONG);
                break;
            case REVERSE:
                motorExtakeLeft.setPower(-1.0);
                motorExtakeRight.setPower(-1.0);
                break;
            case ZERO:
                setExtakeVelocity(0.0);
                break;
            case OVERRIDE:
                setExtakeVelocity(extakeOverride);
                break;
        }
    }

    @Override
    public void stop() {

    }

    /**
     * Set the extake state. Changes will take effect next motor update.
     * @param extakeState The ExtakeState to set.
     */
    public void setExtakeState(ExtakeState extakeState) {
        this.extakeState = extakeState;
    }

    /**
     * Get the current extake state.
     * @return Current extake state.
     */
    public ExtakeState getExtakeState() {
        return this.extakeState;
    }

    /**
     * Set the extake override velocity. Velocity is only applied when DeviceExtake.currentState == ExtakeState.OVERRIDE.
     * @param extakeOverride The velocity in default units.
     */
    public void setExtakeOverride(double extakeOverride) {
        this.extakeOverride = extakeOverride;
    }

    /**
     * Get the extake override velocity.
     * @return The velocity in default units.
     */
    public double getExtakeOverride() {
        return this.extakeOverride;
    }

    /**
     * Check if the extake motors are physically idle (not spinning).
     * @return If the extake motors are idle.
     */
    public boolean isIdle() {
        return this.motorExtakeLeft.getVelocity() <= 20 && this.motorExtakeRight.getVelocity() <= 20;
    }

    /**
     * Set the velocity of both extake motors.
     * @param extakeVelocity The velocity in default units.
     */
    private void setExtakeVelocity(double extakeVelocity) {
        // TODO: real PID ctrlr impl
        motorExtakeLeft.setVelocity(extakeVelocity);
        motorExtakeRight.setVelocity(extakeVelocity);
    }
}
