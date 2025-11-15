package com.technodot.ftc.twentyfive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.technodot.ftc.twentyfive.common.Team;
import com.technodot.ftc.twentyfive.robocore.DeviceCamera;
import com.technodot.ftc.twentyfive.robocore.DeviceDrive;
import com.technodot.ftc.twentyfive.robocore.DeviceExtake;
import com.technodot.ftc.twentyfive.robocore.DeviceIntake;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="BaboOS", group="TechnoCode")
public class BaboOS extends OpMode {

    public DeviceCamera deviceCamera = new DeviceCamera();
    public DeviceDrive deviceDrive = new DeviceDrive();
    public DeviceIntake deviceIntake = new DeviceIntake();
    public DeviceExtake deviceExtake = new DeviceExtake();

    public Team team = Team.BLUE;

    public Telemetry t;

    public long now = 0;
    public long last = 0;
    public long delta = 0;

    @Override
    public void init() {
        config();

        t = FtcDashboard.getInstance().getTelemetry();

        deviceCamera.init(hardwareMap, team);
        deviceDrive.init(hardwareMap);
        deviceIntake.init(hardwareMap, team);
        deviceExtake.init(hardwareMap);
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

        deviceIntake.update(gamepad1);
        deviceIntake.update(t);

        deviceDrive.setRotationalOffset(deviceIntake.getRotationalOffset());
        deviceDrive.updatePose(deviceCamera.update(), now);
        deviceDrive.update(gamepad1);

        deviceExtake.update(gamepad1);

        telemetry.addData("lt", gamepad1.left_trigger);
        telemetry.addData("rt", gamepad1.right_trigger);
        telemetry.addData("lx", gamepad1.left_stick_x);
        telemetry.addData("ly", gamepad1.left_stick_y);
        telemetry.addData("rx", gamepad1.right_stick_x);
        telemetry.addData("ry", gamepad1.right_stick_y);

        t.addData("exv", deviceExtake.motorExtake.getVelocity());

        delta = now - last;
        t.addData("d", delta / 1_000_000.0);
        t.addData("r", 1_000_000_000.0 / delta);
        t.update();

        telemetry.addData("status", "running");
        telemetry.update();

        last = now;
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