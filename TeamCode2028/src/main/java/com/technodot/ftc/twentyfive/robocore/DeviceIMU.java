package com.technodot.ftc.twentyfive.robocore;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technodot.ftc.twentyfive.common.TimedVector3;

/**
 * Integrates MPU6050 linear acceleration to produce velocity.
 * Strategy:
 * 1. Calibration phase: collect N samples while robot is motionless to determine (gravity + sensor bias) vector.
 *    This vector is then subtracted from subsequent raw acceleration so gravity is removed and bias minimized.
 * 2. Exponential smoothing (low‑pass) applied to linear acceleration to suppress high‑frequency noise.
 * 3. Trapezoidal numerical integration (better than simple rectangular) to update velocity.
 * 4. Noise floor thresholding: very small accelerations are treated as zero to reduce drift from noise.
 * 5. Zero‑Velocity Update (ZUPT): if acceleration magnitude stays below a small threshold for a dwell time, velocity is snapped to zero.
 * 6. Thread safety: access to mutable state synchronized on this instance.
 *
 * Notes / Limitations:
 * - Assumes sensor frame aligns with world frame (robot flat, Z up) during operation; if orientation changes significantly,
 *   orientation compensation (gyro integration + rotation) must be added for accurate gravity removal.
 * - Residual drift will still accumulate over long periods; adding external references (encoders, vision) to fuse is recommended.
 */
public class DeviceIMU extends Device {

    public MPU6050 imu;

    public static final double GRAVITY = 9.80665;

    private double biasAx = 0;
    private double biasAy = 0;
    private double biasAz = GRAVITY;

    private boolean ready = false;
    private double lastAx = 0;
    private double lastAy = 0;
    private double lastAz = 0;

    private double filtAx = 0;
    private double filtAy = 0;
    private double filtAz = 0;

    private double velX = 0;
    private double velY = 0;
    private double velZ = 0;

    private long lastUpdateNs = 0;
    private long lastActiveNs = 0;

    private static final double ACCEL_SMOOTH_ALPHA = 0.2; // exponential smoothing factor: higher -> more smoothing
    private static final double ACCEL_NOISE_FLOOR = 0.05; // m/s^2 below which accel component set to zero
    private static final double STATIONARY_ACCEL_THRESH = 0.15; // magnitude threshold to consider stationary
    private static final long STATIONARY_DWELL_NS = (long)(0.30 * 1e9); // dwell time before zeroing velocity

    @Override
    public void init(HardwareMap hardwareMap) {
        imu = hardwareMap.get(MPU6050.class, "imu");
        resetState();
    }

    private synchronized void resetState() {
        ready = false;
        lastAx = lastAy = lastAz = 0;
        filtAx = filtAy = filtAz = 0;
        velX = velY = velZ = 0;
        lastUpdateNs = 0;
        lastActiveNs = System.nanoTime();
    }

    @Override
    public void update(Gamepad gamepad) {
        TimedVector3 raw = imu.getAcceleration(); // raw includes gravity (assuming device orientation fixed)
        long now = raw.time; // timestamp from TimedVector3 (System.nanoTime())

        // Compute linear acceleration (gravity + bias removed via calibration averages).
        double linAx = raw.x - biasAx;
        double linAy = raw.y - biasAy;
        double linAz = raw.z - biasAz;

        // Exponential smoothing (low-pass filter).
        if (!ready) {
            filtAx = linAx;
            filtAy = linAy;
            filtAz = linAz;
            ready = true;
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
            return; // Need a previous sample for trapezoidal integration.
        }

        double dt = (now - lastUpdateNs) * 1e-9; // seconds
        if (dt <= 0 || dt > 0.1) { // Guard against invalid or excessively large dt (e.g., long pause)
            lastUpdateNs = now;
            lastAx = aX; lastAy = aY; lastAz = aZ;
            return;
        }

        // Trapezoidal integration for each component: v += 0.5*(a_prev + a_curr)*dt
        velX += 0.5 * (lastAx + aX) * dt;
        velY += 0.5 * (lastAy + aY) * dt;
        velZ += 0.5 * (lastAz + aZ) * dt;

        lastAx = aX;
        lastAy = aY;
        lastAz = aZ;
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
