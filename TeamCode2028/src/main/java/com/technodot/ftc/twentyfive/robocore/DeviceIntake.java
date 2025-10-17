package com.technodot.ftc.twentyfive.robocore;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DeviceIntake extends Device {
    public DcMotor motorIntake;

    @Override
    public void init(HardwareMap hardwareMap) {
        motorIntake = hardwareMap.get(DcMotor.class, "motorIntake");
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {

    }

    @Override
    public void update(Gamepad gamepad) {
        motorIntake.setPower((gamepad.right_bumper ? 1 : 0) + (gamepad.left_bumper ? -1 : 0));
    }

    @Override
    public void stop() {

    }
}
