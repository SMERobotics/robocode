package com.technodot.ftc.twentyfive.robocore;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DeviceExtake extends Device {
    public DcMotor motorExtake;

    @Override
    public void init(HardwareMap hardwareMap) {
        motorExtake = hardwareMap.get(DcMotor.class, "motorExtake");
        motorExtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {

    }

    @Override
    public void update(Gamepad gamepad) {
        motorExtake.setPower(gamepad.left_trigger - gamepad.right_trigger);
    }

    @Override
    public void stop() {

    }
}
