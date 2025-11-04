package com.technodot.ftc.twentyfive.robocore;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technodot.ftc.twentyfive.common.Controls;

public class DeviceExtake extends Device {
    public DcMotorEx motorExtake;
    public boolean reversing;

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
            motorExtake.setPower(-1.0);
            reversing = true;
        } else if (Controls.extakeShootLow(gamepad)) {
            motorExtake.setPower(0.67);
            reversing = false;
        } else if (Controls.extakeShootHigh(gamepad)) {
            motorExtake.setPower(1.0);
            reversing = false;
        } else {
            reversing = false;
        }

        if (reversing && !Controls.extakeShootReverse(gamepad)) {
            motorExtake.setPower(0.01);
        }
    }

    public void update(double power) {
        motorExtake.setPower(power);
    }

    @Override
    public void stop() {

    }
}
