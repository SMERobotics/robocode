package com.technodot.ftc.twentyfive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.technodot.ftc.twentyfive.common.TimedVector3;
import com.technodot.ftc.twentyfive.robocore.DeviceDrive;
import com.technodot.ftc.twentyfive.robocore.DeviceIMU;

@TeleOp(name="Calibrator", group="TechnoCode")
public class Calibrator extends OpMode {

    public DeviceDrive deviceDrive = new DeviceDrive();
    public DeviceIMU deviceIMU = new DeviceIMU();

    public boolean calibratingAccel = false;
    public boolean calibratingGyro = false;
    public DeviceIMU.CalibrationData calibrationData;
    public double sumAx = 0, sumAy = 0, sumAz = 0;
    public double sumGx = 0, sumGy = 0, sumGz = 0;
    public int sumSamples = 0;

    @Override
    public void init() {
        deviceDrive.init(hardwareMap);
        deviceIMU.init(hardwareMap);
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

        calibratingAccel = true;
    }

    @Override
    public void loop() {
        if (calibratingAccel) {
            TimedVector3 rawAccel = deviceIMU.imu.getAcceleration();
            TimedVector3 rawGyro = deviceIMU.imu.getAngularVelocity();

            sumAx += rawAccel.x;
            sumAy += rawAccel.y;
            sumAz += rawAccel.z;

            sumGx += rawGyro.x;
            sumGy += rawGyro.y;
            sumGz += rawGyro.z;

            sumSamples++;

            if (sumSamples >= DeviceIMU.CALIBRATION_SAMPLES) {
                double biasAx = sumAx / sumSamples;
                double biasAy = sumAy / sumSamples;
                double biasAz = (sumAz / sumSamples) - DeviceIMU.GRAVITY;

                double biasGx = sumGx / sumSamples;
                double biasGy = sumGy / sumSamples;
                double biasGz = sumGz / sumSamples;

                calibrationData = new DeviceIMU.CalibrationData(
                        0, 0, DeviceIMU.GRAVITY,
                        biasAx, biasAy, biasAz
                );

                telemetry.addData("calibration", calibrationData.toString());
                telemetry.update();

                calibratingAccel = false;
                calibratingGyro = true;

                sumAx = sumAy = sumAz = 0;
                sumGx = sumGy = sumGz = 0;
                sumSamples = 0;
            }

            try {
                Thread.sleep(2);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

        } else if (calibratingGyro) {
            deviceDrive.update(1, 0, 0);
        } else {
            deviceDrive.update(gamepad1);
            deviceIMU.update(gamepad1);
        }

        telemetry.addData("status", "running");
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("status", "stopping");
        telemetry.update();
    }
}
