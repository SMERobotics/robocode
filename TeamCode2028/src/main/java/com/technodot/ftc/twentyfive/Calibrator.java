package com.technodot.ftc.twentyfive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.technodot.ftc.twentyfive.common.IMUCalibrationData;
import com.technodot.ftc.twentyfive.common.TimedVector3;
import com.technodot.ftc.twentyfive.robocore.MPU6050;

@TeleOp(name="Calibrator", group="TechnoCode")
public class Calibrator extends OpMode {

    public MPU6050 imu;

    private double sumAx = 0, sumAy = 0, sumAz = 0;
    private double sumGx = 0, sumGy = 0, sumGz = 0;
    public double biasAx = 0, biasAy = 0, biasAz = 0;
    public double biasGx = 0, biasGy = 0, biasGz = 0;
    private int sampleCount = 0;
    public IMUCalibrationData calibrationData = null;

    public final int CALIBRATION_SAMPLES = 1000;

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

        if (sampleCount < CALIBRATION_SAMPLES) {
            sumAx += accel.x;
            sumAy += accel.y;
            sumAz += accel.z;

            sumGx += gyro.x;
            sumGy += gyro.y;
            sumGz += gyro.z;

            sampleCount++;
        } else if (calibrationData == null && sampleCount == CALIBRATION_SAMPLES) {
            biasAx = sumAx / CALIBRATION_SAMPLES;
            biasAy = sumAy / CALIBRATION_SAMPLES;
            biasAz = sumAz / CALIBRATION_SAMPLES;

            biasGx = sumGx / CALIBRATION_SAMPLES;
            biasGy = sumGy / CALIBRATION_SAMPLES;
            biasGz = sumGz / CALIBRATION_SAMPLES;

            calibrationData = new IMUCalibrationData(biasAx, biasAy, biasAz, biasGx, biasGy, biasGz);
        }

        if (calibrationData == null) {
            telemetry.addData("progress", String.format("%.1f%%", (100.0 * sampleCount / CALIBRATION_SAMPLES)));
        } else {
            telemetry.addData("calibration", calibrationData.toString());
        }

        TimedVector3[] calibrated = applyCalibration(accel, gyro);

        telemetry.addData("accel", calibrated[0].toString());
        telemetry.addData("gyro", calibrated[1].toString());
        telemetry.addData("status", "running");
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("status", "stopping");
        telemetry.update();
    }
}
