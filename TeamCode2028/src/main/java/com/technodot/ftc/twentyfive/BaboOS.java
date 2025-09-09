package com.technodot.ftc.twentyfive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.technodot.ftc.twentyfive.robocore.DeviceCamera;
import com.technodot.ftc.twentyfive.robocore.DeviceDrive;

@TeleOp(name="BaboOS", group="TechnoCode")
public class BaboOS extends OpMode {

    public DeviceCamera deviceCamera = new DeviceCamera();
    public DeviceDrive deviceDrive = new DeviceDrive();

    public Team team = Team.BLUE;

    @Override
    public void init() {
        deviceCamera.init(hardwareMap);
        deviceDrive.init(hardwareMap);
    }

    @Override
    public void init_loop() {
        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        deviceCamera.start();

        telemetry.addData("status", "starting");
        telemetry.update();
    }

    @Override
    public void loop() {
        deviceCamera.update(gamepad1);
        deviceDrive.update(gamepad1);

        telemetry.addData("status", "running");
        telemetry.update();
    }

    @Override
    public void stop() {
        deviceCamera.stop();

        telemetry.addData("status", "stopping");
        telemetry.update();
    }
}