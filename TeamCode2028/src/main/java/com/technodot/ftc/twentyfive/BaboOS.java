package com.technodot.ftc.twentyfive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.technodot.ftc.twentyfive.common.Team;
import com.technodot.ftc.twentyfive.robocore.DeviceCamera;
import com.technodot.ftc.twentyfive.robocore.DeviceDrive;
import com.technodot.ftc.twentyfive.robocore.DeviceExtake;
import com.technodot.ftc.twentyfive.robocore.DeviceIntake;

@TeleOp(name="BaboOS", group="TechnoCode")
public class BaboOS extends OpMode {

    public DeviceCamera deviceCamera = new DeviceCamera();
    public DeviceDrive deviceDrive = new DeviceDrive();
    public DeviceIntake deviceIntake = new DeviceIntake();
    public DeviceExtake deviceExtake = new DeviceExtake();

    public Team team = Team.BLUE;

    public long now = 0;

    @Override
    public void init() {
        deviceCamera.init(hardwareMap);
        deviceDrive.init(hardwareMap);
        deviceIntake.init(hardwareMap);
        deviceExtake.init(hardwareMap);

        config();
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
        now = System.nanoTime();

        deviceDrive.updatePose(deviceCamera.update(), now);
        
        deviceDrive.update(gamepad1);
        deviceIntake.update(gamepad1);
        deviceExtake.update(gamepad1);

        deviceIntake.update(telemetry);

        telemetry.addData("lt", gamepad1.left_trigger);
        telemetry.addData("rt", gamepad1.right_trigger);
        telemetry.addData("lx", gamepad1.left_stick_x);
        telemetry.addData("ly", gamepad1.left_stick_y);
        telemetry.addData("rx", gamepad1.right_stick_x);
        telemetry.addData("ry", gamepad1.right_stick_y);

        telemetry.addData("status", "running");
        telemetry.update();
    }

    @Override
    public void stop() {
        deviceCamera.stop();

        telemetry.addData("status", "stopping");
        telemetry.update();
    }

    public void config() {
        team = Team.BLUE;
    }
}