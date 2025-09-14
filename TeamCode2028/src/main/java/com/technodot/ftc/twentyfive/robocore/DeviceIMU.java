package com.technodot.ftc.twentyfive.robocore;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technodot.ftc.twentyfive.common.TimedVector3;

public class DeviceIMU extends Device {

    public MPU6050 imu;

    @Override
    public void init(HardwareMap hardwareMap) {
        imu = hardwareMap.get(MPU6050.class, "imu");
    }

    @Override
    public void update(Gamepad gamepad) {

    }

    public TimedVector3 getVelocity() {
        return new TimedVector3(0, 0, 0);
    }
}
