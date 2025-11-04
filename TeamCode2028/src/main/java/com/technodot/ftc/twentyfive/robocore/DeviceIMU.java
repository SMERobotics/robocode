package com.technodot.ftc.twentyfive.robocore;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technodot.ftc.twentyfive.common.IMUCalibrationData;
import com.technodot.ftc.twentyfive.common.TimedVector3;

public class DeviceIMU extends Device {

    public MPU6050 imu;

    public double biasAx = 0, biasAy = 0, biasAz = 0;
    public double biasGx = 0, biasGy = 0, biasGz = 0;

    public double lastAx = 0, lastAy = 0, lastAz = 0;
    public long lastUpdateNs = 0;

    public double velX = 0, velY = 0, velZ = 0;

    public static final double ACCEL_SMOOTH_ALPHA = 0.2;

    @Override
    public void init(HardwareMap hardwareMap) {
        imu = hardwareMap.get(MPU6050.class, "imu");
    }

    public void init(HardwareMap hardwareMap, IMUCalibrationData calibration) {
        imu = hardwareMap.get(MPU6050.class, "imu");
        applyCalibration(calibration);
    }

    @Override
    public void start() {

    }

    public void applyCalibration(IMUCalibrationData calibration) {
        this.biasAx = calibration.biasAx;
        this.biasAy = calibration.biasAy;
        this.biasAz = calibration.biasAz;
        this.biasGx = calibration.biasGx;
        this.biasGy = calibration.biasGy;
        this.biasGz = calibration.biasGz;
    }

    public TimedVector3[] offsetCalibration(TimedVector3 accel, TimedVector3 gyro) {
        accel.x -= biasAx;
        accel.y -= biasAy;
        accel.z -= biasAz;

        gyro.x -= biasGx;
        gyro.y -= biasGy;
        gyro.z -= biasGz;

        return new TimedVector3[]{accel, gyro};
    }

    public TimedVector3[] offsetCalibration(TimedVector3 accel, TimedVector3 gyro, IMUCalibrationData calibration) {
        accel.x -= calibration.biasAx;
        accel.y -= calibration.biasAy;
        accel.z -= calibration.biasAz;

        gyro.x -= calibration.biasGx;
        gyro.y -= calibration.biasGy;
        gyro.z -= calibration.biasGz;

        return new TimedVector3[]{accel, gyro};
    }

    public TimedVector3[] offsetCalibration(TimedVector3[] imuData) {
        imuData[0].x -= biasAx;
        imuData[0].y -= biasAy;
        imuData[0].z -= biasAz;

        imuData[1].x -= biasGx;
        imuData[1].y -= biasGy;
        imuData[1].z -= biasGz;

        return imuData;
    }

    public TimedVector3[] offsetCalibration(TimedVector3[] imuData, IMUCalibrationData calibration) {
        imuData[0].x -= calibration.biasAx;
        imuData[0].y -= calibration.biasAy;
        imuData[0].z -= calibration.biasAz;

        imuData[1].x -= calibration.biasGx;
        imuData[1].y -= calibration.biasGy;
        imuData[1].z -= calibration.biasGz;

        return imuData;
    }

    @Override
    public void update(Gamepad gamepad) {
        TimedVector3[] data = offsetCalibration(imu.getRawData());
        TimedVector3 rawAccel = data[0];
        TimedVector3 rawGyro = data[1];
        long now = rawAccel.time;
    }

    public TimedVector3 getVelocity() {
        return new TimedVector3(0, 0, 0);
    }

    @Override
    public void stop() {

    }
}