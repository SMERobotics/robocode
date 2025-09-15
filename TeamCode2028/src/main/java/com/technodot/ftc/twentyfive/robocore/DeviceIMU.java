package com.technodot.ftc.twentyfive.robocore;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technodot.ftc.twentyfive.common.TimedVector3;
import com.technodot.ftc.twentyfive.common.Quaternion;

public class DeviceIMU extends Device {

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
    private int calibCount = 0;
    private static final int CALIBRATION_SAMPLES = 300; // ~ 3/5 second at 500Hz; adjust per actual sample rate
    private double sumAx = 0, sumAy = 0, sumAz = 0;
    private double sumGx = 0, sumGy = 0, sumGz = 0;
    private double biasGx = 0, biasGy = 0, biasGz = 0;
    private double filtAx = 0, filtAy = 0, filtAz = 0;
    private double velX = 0, velY = 0, velZ = 0;
    private double lastAx = 0, lastAy = 0, lastAz = 0;

    private long lastUpdateNs = 0;
    private long lastActiveNs = 0;

    private static final double ACCEL_SMOOTH_ALPHA = 0.2; // exponential smoothing factor: higher -> more smoothing
    private static final double ACCEL_NOISE_FLOOR = 0.05; // m/s^2 below which accel component set to zero
    private static final double STATIONARY_ACCEL_THRESH = 0.15; // magnitude threshold (linear accel) to consider stationary
    private static final long STATIONARY_DWELL_NS = (long)(0.30 * 1e9); // dwell time before zeroing velocity

    @Override
    public void init(HardwareMap hardwareMap) {
        imu = hardwareMap.get(MPU6050.class, "imu");
        resetState();
    }

    private synchronized void resetState() {
        calibrated = false;
        calibCount = 0;
        sumAx = sumAy = sumAz = 0;
        sumGx = sumGy = sumGz = 0;
        gWx = 0; gWy = 0; gWz = GRAVITY; // fallback assumption
        biasGx = biasGy = biasGz = 0;
        qWB = Quaternion.IDENTITY;
        velX = velY = velZ = 0;
        lastUpdateNs = 0;
        lastActiveNs = System.nanoTime();
        lastAx = lastAy = lastAz = 0;
        filtAx = filtAy = filtAz = 0;
    }

    private synchronized void applyCalibration(CalibrationData d) {
        this.gWx = d.gWx; this.gWy = d.gWy; this.gWz = d.gWz;
        this.biasGx = d.biasGx; this.biasGy = d.biasGy; this.biasGz = d.biasGz;
        this.qWB = Quaternion.IDENTITY; // world frame defined at calibration capture
        this.velX = this.velY = this.velZ = 0;
        this.lastAx = this.lastAy = this.lastAz = 0;
        this.filtAx = this.filtAy = this.filtAz = 0;
        this.lastUpdateNs = System.nanoTime();
        this.lastActiveNs = this.lastUpdateNs;
        this.calibrated = true;
        this.calibCount = CALIBRATION_SAMPLES; // mark as complete
    }

    public synchronized CalibrationData getCalibrationData() {
        return new CalibrationData(gWx, gWy, gWz, biasGx, biasGy, biasGz);
    }

    public synchronized void reset() {
        resetState();
    }

    public synchronized boolean isCalibrated() {
        return calibrated;
    }

    public synchronized Quaternion getOrientation() {
        return qWB;
    }

    @Override
    public void update(Gamepad gamepad) {
        TimedVector3 accelBody = imu.getAcceleration();
        TimedVector3 gyroDegPerSec = imu.getAngularVelocity();
        long now = accelBody.time;

        if (!calibrated) {
            sumAx += accelBody.x; sumAy += accelBody.y; sumAz += accelBody.z;
            sumGx += gyroDegPerSec.x; sumGy += gyroDegPerSec.y; sumGz += gyroDegPerSec.z;
            calibCount++;
            if (calibCount >= CALIBRATION_SAMPLES) {
                double avgAx = sumAx / calibCount;
                double avgAy = sumAy / calibCount;
                double avgAz = sumAz / calibCount;
                double avgGx = sumGx / calibCount;
                double avgGy = sumGy / calibCount;
                double avgGz = sumGz / calibCount;
                gWx = avgAx; gWy = avgAy; gWz = avgAz;
                biasGx = avgGx; biasGy = avgGy; biasGz = avgGz;
                velX = velY = velZ = 0;
                lastAx = lastAy = lastAz = 0;
                filtAx = filtAy = filtAz = 0;
                lastUpdateNs = now;
                calibrated = true;
            }
            return;
        }

        double dt = (lastUpdateNs == 0) ? 0 : (now - lastUpdateNs) * 1e-9;
        if (dt < 0) { lastUpdateNs = now; return; }

        if (dt > 0 && dt < 0.1) {
            double wx = Math.toRadians(gyroDegPerSec.x - biasGx);
            double wy = Math.toRadians(gyroDegPerSec.y - biasGy);
            double wz = Math.toRadians(gyroDegPerSec.z - biasGz);
            Quaternion dq = Quaternion.fromAngularVelocity(wx, wy, wz, dt);
            synchronized (this) { qWB = qWB.multiply(dq).normalize(); }
        }

        Quaternion q; synchronized (this) { q = qWB; }
        double[] aWorldTotal = q.rotateBodyToWorld(accelBody.x, accelBody.y, accelBody.z);
        double linAx = aWorldTotal[0] - gWx;
        double linAy = aWorldTotal[1] - gWy;
        double linAz = aWorldTotal[2] - gWz;

        // Exponential smoothing in world frame
        if (lastUpdateNs == 0) {
            filtAx = linAx; filtAy = linAy; filtAz = linAz;
        } else {
            filtAx = ACCEL_SMOOTH_ALPHA * filtAx + (1.0 - ACCEL_SMOOTH_ALPHA) * linAx;
            filtAy = ACCEL_SMOOTH_ALPHA * filtAy + (1.0 - ACCEL_SMOOTH_ALPHA) * linAy;
            filtAz = ACCEL_SMOOTH_ALPHA * filtAz + (1.0 - ACCEL_SMOOTH_ALPHA) * linAz;
        }

        // Apply per-axis noise floor thresholding.
        double aX = Math.abs(filtAx) < ACCEL_NOISE_FLOOR ? 0.0 : filtAx;
        double aY = Math.abs(filtAy) < ACCEL_NOISE_FLOOR ? 0.0 : filtAy;
        double aZ = Math.abs(filtAz) < ACCEL_NOISE_FLOOR ? 0.0 : filtAz;

        if (lastUpdateNs == 0) {
            lastUpdateNs = now;
            lastAx = aX; lastAy = aY; lastAz = aZ;
            return;
        }

        dt = (now - lastUpdateNs) * 1e-9;
        if (dt <= 0 || dt > 0.1) { // guard
            lastUpdateNs = now;
            lastAx = aX; lastAy = aY; lastAz = aZ;
            return;
        }

        // Trapezoidal integration for each component: v += 0.5*(a_prev + a_curr)*dt
        velX += 0.5 * (lastAx + aX) * dt;
        velY += 0.5 * (lastAy + aY) * dt;
        velZ += 0.5 * (lastAz + aZ) * dt;

        lastAx = aX; lastAy = aY; lastAz = aZ;
        lastUpdateNs = now;

        // Stationary detection for zero-velocity update.
        double accelMag = Math.sqrt(aX * aX + aY * aY + aZ * aZ);
        if (accelMag > STATIONARY_ACCEL_THRESH) {
            lastActiveNs = now;
        } else if ((now - lastActiveNs) > STATIONARY_DWELL_NS) {
            // Snap velocities to zero after being stationary for dwell time.
            velX = velY = velZ = 0;
        }
    }

    public synchronized TimedVector3 getVelocity() {
        return new TimedVector3(velX, velY, velZ);
    }
}
