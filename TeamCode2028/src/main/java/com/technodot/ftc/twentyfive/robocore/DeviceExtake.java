package com.technodot.ftc.twentyfive.robocore;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technodot.ftc.twentyfive.common.Controls;

public class DeviceExtake extends Device {
    public DcMotorEx motorExtake;
    public ExtakeState currentState = ExtakeState.IDLE;

    public enum ExtakeState {
        IDLE,
        SHOOTING_LOW,
        SHOOTING_HIGH,
        REVERSING
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        motorExtake = hardwareMap.get(DcMotorEx.class, "motorExtake");
        motorExtake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void start() {

    }

    @Override
    public void update(Gamepad gamepad) {
        // TODO: use velocity control instead of power control!!!

        if (Controls.extakeShootReverse(gamepad)) {
            currentState = ExtakeState.REVERSING;
        } else if (Controls.extakeShootLow(gamepad)) {
            if (currentState.equals(ExtakeState.SHOOTING_LOW)) {
                currentState = ExtakeState.IDLE;
            } else {
                currentState = ExtakeState.SHOOTING_LOW;
            }
        } else if (Controls.extakeShootHigh(gamepad)) {
            if (currentState.equals(ExtakeState.SHOOTING_HIGH)) {
                currentState = ExtakeState.IDLE;
            } else {
                currentState = ExtakeState.SHOOTING_HIGH;
            }
        } else {
            if (currentState.equals(ExtakeState.REVERSING)) {
                currentState = ExtakeState.IDLE;
            }
        }

        if (currentState.equals(ExtakeState.REVERSING) && !Controls.extakeShootReverse(gamepad)) {
            currentState = ExtakeState.IDLE;
        }

        switch (currentState) {
            case REVERSING:
                motorExtake.setPower(-1.0);
                break;
            case SHOOTING_LOW:
                motorExtake.setPower(0.67);
                break;
            case SHOOTING_HIGH:
                motorExtake.setPower(1.0);
                break;
            case IDLE:
                motorExtake.setPower(0.01);
                break;
        }
    }

    public void update(double power) {
        motorExtake.setPower(power);
    }

    @Override
    public void stop() {

    }
}
