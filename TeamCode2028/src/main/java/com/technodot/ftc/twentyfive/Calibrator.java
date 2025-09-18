package com.technodot.ftc.twentyfive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.technodot.ftc.twentyfive.common.IMUCalibrationData;
import com.technodot.ftc.twentyfive.common.Quaternion;
import com.technodot.ftc.twentyfive.common.TimedVector3;
import com.technodot.ftc.twentyfive.robocore.DeviceDrive;
import com.technodot.ftc.twentyfive.robocore.MPU6050;

@TeleOp(name="Calibrator", group="TechnoCode")
public class Calibrator extends OpMode {

    public DeviceDrive deviceDrive;
    public MPU6050 imu;

    private double sumAx = 0, sumAy = 0, sumAz = 0;
    private double sumGx = 0, sumGy = 0, sumGz = 0;
    public double biasAx = 0, biasAy = 0, biasAz = 0;
    public double biasGx = 0, biasGy = 0, biasGz = 0;
    private int calibrationSampleCount = 0;
    public IMUCalibrationData calibrationData = null;
    public final int CALIBRATION_SAMPLES = 1000;

    private double sumFx = 0, sumFy = 0, sumFz = 0;
    private double forwardX = 1, forwardY = 0, forwardZ = 0;
    private double rightX = 0, rightY = 1, rightZ = 0;
    private double upX = 0, upY = 0, upZ = 1;
    private int orientationSampleCount = 0;
    private Quaternion orientation = Quaternion.IDENTITY;
    public final int ORIENTAITON_SAMPLES = 100;

    public TimedVector3[] applyCalibration(TimedVector3 accel, TimedVector3 gyro) {
        accel.x -= biasAx;
        accel.y -= biasAy;
        accel.z -= biasAz;

        gyro.x -= biasGx;
        gyro.y -= biasGy;
        gyro.z -= biasGz;

        return new TimedVector3[]{accel, gyro};
    }

    public TimedVector3[] applyCalibration(TimedVector3 accel, TimedVector3 gyro, IMUCalibrationData calibration) {
        accel.x -= calibration.biasAx;
        accel.y -= calibration.biasAy;
        accel.z -= calibration.biasAz;

        gyro.x -= calibration.biasGx;
        gyro.y -= calibration.biasGy;
        gyro.z -= calibration.biasGz;

        return new TimedVector3[]{accel, gyro};
    }

    @Override
    public void init() {
        deviceDrive.init(hardwareMap);
        imu = hardwareMap.get(MPU6050.class, "imu");
    }

    @Override
    public void init_loop() {
        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        telemetry.addData("status", "starting");
        telemetry.update();
    }

    @Override
    public void loop() {
        TimedVector3 accel = imu.getRawAcceleration();
        TimedVector3 gyro = imu.getRawAngularVelocity();
        TimedVector3[] calibrated = applyCalibration(accel, gyro);

        if (calibrationSampleCount < CALIBRATION_SAMPLES) {
            sumAx += accel.x;
            sumAy += accel.y;
            sumAz += accel.z;

            sumGx += gyro.x;
            sumGy += gyro.y;
            sumGz += gyro.z;

            calibrationSampleCount++;
        } else if (calibrationSampleCount == CALIBRATION_SAMPLES) {
            biasAx = sumAx / CALIBRATION_SAMPLES;
            biasAy = sumAy / CALIBRATION_SAMPLES;
            biasAz = sumAz / CALIBRATION_SAMPLES;

            biasGx = sumGx / CALIBRATION_SAMPLES;
            biasGy = sumGy / CALIBRATION_SAMPLES;
            biasGz = sumGz / CALIBRATION_SAMPLES;

            calibrationData = new IMUCalibrationData(biasAx, biasAy, biasAz, biasGx, biasGy, biasGz);
            calibrationSampleCount++;
        } else if (orientationSampleCount < ORIENTAITON_SAMPLES) {
            deviceDrive.update(1, 0, 0);

            double gyroMag = Math.sqrt(gyro.x * gyro.x + gyro.y * gyro.y + gyro.z * gyro.z);
            if (gyroMag < 10.0) {
                sumFx += calibrated[0].x;
                sumFy += calibrated[0].y;
                sumFz += calibrated[0].z;
            }

            orientationSampleCount++;
        } else if (orientationSampleCount == ORIENTAITON_SAMPLES) {
            deviceDrive.update(0, 0, 0);

            // Up vector in sensor frame: accelerometer average during stationary phase points opposite gravity? For a resting IMU, proper acceleration is +g upward.
            double gX = calibrationData.biasAx;
            double gY = calibrationData.biasAy;
            double gZ = calibrationData.biasAz;
            double[] up = normalize(gX, gY, gZ);

            // Forward dynamic acceleration vector accumulated while driving forward.
            double[] fRaw = new double[]{sumFx, sumFy, sumFz};
            // Remove any component along up (ensure orthogonality).
            double dotFU = fRaw[0]*up[0] + fRaw[1]*up[1] + fRaw[2]*up[2];
            fRaw[0] -= dotFU * up[0];
            fRaw[1] -= dotFU * up[1];
            fRaw[2] -= dotFU * up[2];
            double[] forward = normalize(fRaw[0], fRaw[1], fRaw[2]);
            double fMag = magnitude(forward[0], forward[1], forward[2]);
            if (fMag < 1e-3 || Double.isNaN(fMag)) {
                // Fallback: keep default axes.
                forward = new double[]{1,0,0};
                up = new double[]{0,0,1};
            }
            // Right = up x forward (right-hand rule)
            double[] right = cross(up, forward);
            right = normalize(right[0], right[1], right[2]);
            // Recompute forward = right x up to ensure orthonormal triad.
            forward = cross(right, up);
            forward = normalize(forward[0], forward[1], forward[2]);

            forwardX = forward[0]; forwardY = forward[1]; forwardZ = forward[2];
            rightX = right[0]; rightY = right[1]; rightZ = right[2];
            upX = up[0]; upY = up[1]; upZ = up[2];

            orientation = Quaternion.fromBasis(forward, right, up); // rotation from sensor frame to robot frame

            orientationSampleCount++;
        }

        if (calibrationData == null) {
            telemetry.addData("progress", String.format("%.1f%%", (100.0 * calibrationSampleCount / CALIBRATION_SAMPLES)));
        } else {
            telemetry.addData("calibration", calibrationData.toString());
        }

        telemetry.addData("accel", calibrated[0].toString());
        telemetry.addData("gyro", calibrated[1].toString());
        if (!orientation.equals(Quaternion.IDENTITY)) {
            telemetry.addData("orientation", orientation.toString());
        } else if (calibrationData != null) {
            telemetry.addData("orientationProgress", String.format("%.1f%%", (100.0 * orientationSampleCount / ORIENTAITON_SAMPLES)));
        }
        telemetry.addData("status", "running");
        telemetry.update();
    }

    private static double[] normalize(double x, double y, double z) {
        double m = Math.sqrt(x*x + y*y + z*z);
        if (m < 1e-12) return new double[]{0,0,0};
        return new double[]{x/m, y/m, z/m};
    }

    private static double magnitude(double x, double y, double z) {
        return Math.sqrt(x*x + y*y + z*z);
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[]{
                a[1]*b[2] - a[2]*b[1],
                a[2]*b[0] - a[0]*b[2],
                a[0]*b[1] - a[1]*b[0]
        };
    }

    @Override
    public void stop() {
        telemetry.addData("status", "stopping");
        telemetry.update();
    }
}
