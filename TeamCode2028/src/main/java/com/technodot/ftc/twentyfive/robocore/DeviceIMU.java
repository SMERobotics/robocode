package com.technodot.ftc.twentyfive.robocore;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DeviceIMU extends Device {

    @Override
    public void init(HardwareMap hardwareMap) {

    }

    @Override
    public void update(Gamepad gamepad) {

    }

    public double[] update() {
        return new double[]{0, 0, 0};
    }
}
