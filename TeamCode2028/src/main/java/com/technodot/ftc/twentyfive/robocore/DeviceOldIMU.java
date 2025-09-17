package com.technodot.ftc.twentyfive.robocore;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technodot.ftc.twentyfive.common.Quaternion;
import com.technodot.ftc.twentyfive.common.TimedVector3;

public class DeviceOldIMU extends Device {

    public static class CalibrationData {
        public final double gWx, gWy, gWz, biasGx, biasGy, biasGz;

        public CalibrationData(double gWx, double gWy, double gWz, double biasGx, double biasGy, double biasGz) {
            this.gWx = gWx; this.gWy = gWy; this.gWz = gWz; this.biasGx = biasGx; this.biasGy = biasGy; this.biasGz = biasGz;
        }

        @Override
        public String toString() {
            return String.format("CalibrationData[g=(%.5f,%.5f,%.5f), bias=(%.5f,%.5f,%.5f)]", gWx, gWy, gWz, biasGx, biasGy, biasGz);
        }
    }

    public MPU6050 imu;

    public static final double GRAVITY = 9.80665;

    private double gWx = 0;
    private double gWy = 0;
    private double gWz = GRAVITY;
    private Quaternion qWB = Quaternion.IDENTITY;

    private boolean calibrated = false;
    private double biasGx = 0, biasGy = 0, biasGz = 0;
    private double filtAx = 0, filtAy = 0, filtAz = 0;
    private double velX = 0, velY = 0, velZ = 0;
    private double lastAx = 0, lastAy = 0, lastAz = 0;

    private long lastUpdateNs = 0;
    private long lastActiveNs = 0;

    public static final int CALIBRATION_SAMPLES = 500; // ~1s at 500Hz
    public static final double ACCEL_SMOOTH_ALPHA = 0.2; // exponential smoothing factor
    public static final double ACCEL_NOISE_FLOOR = 0.05; // m/s^2
    public static final double STATIONARY_ACCEL_THRESH = 0.15; // m/s^2
    public static final long STATIONARY_DWELL_NS = (long)(0.30 * 1e9); // 300ms

    @Override
    public void init(HardwareMap hardwareMap) {
        imu = hardwareMap.get(MPU6050.class, "imu");
        reset();
    }

    private synchronized void reset() {
        calibrated = false;
        gWx = 0;
        gWy = 0;
        gWz = GRAVITY;

        biasGx = biasGy = biasGz = 0;

        qWB = Quaternion.IDENTITY;
        velX = velY = velZ = 0;
        lastUpdateNs = 0;
        lastActiveNs = System.nanoTime();

        lastAx = lastAy = lastAz = 0;
        filtAx = filtAy = filtAz = 0;
    }

    public synchronized boolean isCalibrated() {
        return calibrated;
    }

    public synchronized Quaternion getOrientation() {
        return qWB;
    }

    public synchronized void applyCalibration(CalibrationData data) {
        this.gWx = data.gWx;
        this.gWy = data.gWy;
        this.gWz = data.gWz;

        this.biasGx = data.biasGx;
        this.biasGy = data.biasGy;
        this.biasGz = data.biasGz;

        this.qWB = Quaternion.IDENTITY;
        this.velX = this.velY = this.velZ = 0;
        this.lastAx = this.lastAy = this.lastAz = 0;
        this.filtAx = this.filtAy = this.filtAz = 0;
        this.lastUpdateNs = System.nanoTime();
        this.lastActiveNs = this.lastUpdateNs;
        this.calibrated = true;
    }

    @Override
    public void update(Gamepad gamepad) {
        TimedVector3 rawAccel = imu.getAcceleration();
        TimedVector3 rawGyro = imu.getAngularVelocity();
        long now = rawAccel.time;

        double dt = (lastUpdateNs == 0) ? 0 : (now - lastUpdateNs) * 1e-9;
        if (dt < 0) {
            lastUpdateNs = now;
            return;
        }

        // --- ORIENTATION UPDATE ---
        if (dt > 0 && dt < 0.1) {
            double wx = Math.toRadians(rawGyro.x - biasGx);
            double wy = Math.toRadians(rawGyro.y - biasGy);
            double wz = Math.toRadians(rawGyro.z - biasGz);

            Quaternion dq = Quaternion.fromAngularVelocity(wx, wy, wz, dt);
            synchronized (this) {
                qWB = qWB.multiply(dq).normalize();
            }
        }

        Quaternion q;
        synchronized (this) {
            q = qWB;
        }

        double[] aWorldTotal = q.rotateBodyToWorld(rawAccel.x, rawAccel.y, rawAccel.z);

        double linAx = aWorldTotal[0] - gWx;
        double linAy = aWorldTotal[1] - gWy;
        double linAz = aWorldTotal[2] - gWz;

        // --- ACCEL SMOOTHING ---
        if (lastUpdateNs == 0) {
            filtAx = linAx;
            filtAy = linAy;
            filtAz = linAz;
        } else {
            filtAx = ACCEL_SMOOTH_ALPHA * filtAx + (1.0 - ACCEL_SMOOTH_ALPHA) * linAx;
            filtAy = ACCEL_SMOOTH_ALPHA * filtAy + (1.0 - ACCEL_SMOOTH_ALPHA) * linAy;
            filtAz = ACCEL_SMOOTH_ALPHA * filtAz + (1.0 - ACCEL_SMOOTH_ALPHA) * linAz;
        }

        // --- NOISE FLOOR ---
        double aX = Math.abs(filtAx) < ACCEL_NOISE_FLOOR ? 0.0 : filtAx;
        double aY = Math.abs(filtAy) < ACCEL_NOISE_FLOOR ? 0.0 : filtAy;
        double aZ = Math.abs(filtAz) < ACCEL_NOISE_FLOOR ? 0.0 : filtAz;

        if (lastUpdateNs == 0) {
            lastUpdateNs = now;
            lastAx = aX; lastAy = aY; lastAz = aZ;
            return;
        }

        dt = (now - lastUpdateNs) * 1e-9;
        if (dt <= 0 || dt > 0.1) {
            lastUpdateNs = now;
            lastAx = aX; lastAy = aY; lastAz = aZ;
            return;
        }

        // --- VELOCITY INTEGRATION ---
        velX += 0.5 * (lastAx + aX) * dt;
        velY += 0.5 * (lastAy + aY) * dt;
        velZ += 0.5 * (lastAz + aZ) * dt;

        lastAx = aX; lastAy = aY; lastAz = aZ;
        lastUpdateNs = now;

        // --- ZERO-VELOCITY UPDATE ---
        double accelMag = Math.sqrt(aX * aX + aY * aY + aZ * aZ);
        if (accelMag > STATIONARY_ACCEL_THRESH) {
            lastActiveNs = now;
        } else if ((now - lastActiveNs) > STATIONARY_DWELL_NS) {
            velX = velY = velZ = 0;
        }
    }

    public synchronized TimedVector3 getVelocity() {
        return new TimedVector3(velX, velY, velZ, lastUpdateNs);
    }
}
