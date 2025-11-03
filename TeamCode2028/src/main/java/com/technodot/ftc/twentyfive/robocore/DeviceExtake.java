package com.technodot.ftc.twentyfive.robocore;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DeviceExtake extends Device {
    public DcMotorEx motorExtake;

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
//        motorExtake.setPower(gamepad.right_trigger - gamepad.left_trigger);

        // TODO: calculate what velocity corresponds to 67% power!!!

        if (gamepad.right_trigger > 0.2) {
            motorExtake.setPower(0.67);
        } else {
            motorExtake.setPower(0);
        }
    }

    public void update(double power) {
        motorExtake.setPower(power);
    }

    @Override
    public void stop() {

    }
}
