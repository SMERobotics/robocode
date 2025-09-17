package com.technodot.ftc.twentyfive.robocore;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technodot.ftc.twentyfive.common.TimedVector3;

public class DeviceIMU extends Device {

    public MPU6050 imu;

    public double biasAx = 0, biasAy = 0, biasAz = 0;
    public double biasGx = 0, biasGy = 0, biasGz = 0;

    public double lastAx = 0, lastAy = 0, lastAz = 0;
    public long lastUpdateNs = 0;

    public double velX = 0, velY = 0, velZ = 0;

    @Override
    public void init(HardwareMap hardwareMap) {
        imu = hardwareMap.get(MPU6050.class, "imu");
    }

    @Override
    public void update(Gamepad gamepad) {
        TimedVector3 accel = imu.getAcceleration();
        TimedVector3 gyro = imu.getAngularVelocity();
    }

    public TimedVector3 getVelocity() {
        return new TimedVector3(0, 0, 0);
    }
}