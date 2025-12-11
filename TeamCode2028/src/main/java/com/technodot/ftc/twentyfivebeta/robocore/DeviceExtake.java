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

    public ExtakeState extakeState;
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
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        SilentRunner101 ctrl = (SilentRunner101) inputController;
//        if (ctrl.extakeFar()) {
//            motorExtakeLeft.setVelocity(Configuration.EXTAKE_MOTOR_SPEED_SHORT);
//            motorExtakeRight.setVelocity(Configuration.EXTAKE_MOTOR_SPEED_SHORT);
//        } else if (ctrl.extakeShort()) {
//            motorExtakeLeft.setVelocity(Configuration.EXTAKE_MOTOR_SPEED_LONG);
//            motorExtakeRight.setVelocity(Configuration.EXTAKE_MOTOR_SPEED_LONG);
//        } else {
//            motorExtakeLeft.setVelocity(0);
//            motorExtakeRight.setVelocity(0);
//        }

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
        } else {
            extakeState = ExtakeState.IDLE;
        }

        prevExtakeFar = extakeFar;
        prevExtakeClose = extakeClose;

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

    public void setExtakeState(ExtakeState extakeState) {
        this.extakeState = extakeState;
    }

    public ExtakeState getExtakeState() {
        return this.extakeState;
    }

    public void setExtakeOverride(double extakeOverride) {
        this.extakeOverride = extakeOverride;
    }

    public double getExtakeOverride() {
        return this.extakeOverride;
    }

    private void setExtakeVelocity(double extakeVelocity) {
        motorExtakeLeft.setVelocity(extakeVelocity);
        motorExtakeRight.setVelocity(extakeVelocity);
    }
}
