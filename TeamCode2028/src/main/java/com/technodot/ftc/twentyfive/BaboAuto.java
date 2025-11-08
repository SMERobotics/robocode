package com.technodot.ftc.twentyfive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;
import com.technodot.ftc.twentyfive.batch.Batch;
import com.technodot.ftc.twentyfive.common.Team;
import com.technodot.ftc.twentyfive.robocore.DeviceCamera;
import com.technodot.ftc.twentyfive.robocore.DeviceDrive;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name="BaboAuto", group="TechnoCode")
public class BaboAuto extends OpMode {

    public DeviceCamera deviceCamera = new DeviceCamera();
    public DeviceDrive deviceDrive = new DeviceDrive();

    public MultipleTelemetry t;

    public Batch runtime = new Batch();

    public Team team = Team.BLUE;

    @Override
    public void init() {
        config();

        t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        deviceCamera.init(hardwareMap, team);
        deviceDrive.init(hardwareMap);

        // Plan to move backward for the first second using the new movement system.
        runtime.plan(0, (long startMs, long durationMs, long executionMs) -> {
            // applyMovement adds a request to the queue. It does not command the motors directly.
            deviceDrive.applyMovement(-4.0f, 0.0f, 0.0f);
            return false; // Don't end the action early.
        });
    }

    @Override
    public void init_loop() {
        t.addData("status", "initialized");
        t.update();
    }

    @Override
    public void start() {
        deviceCamera.start();
        deviceDrive.start(); // Initialize encoders and motor modes.

        t.addData("status", "starting");
        t.update();
    }

    @Override
    public void loop() {
        // This executes the actions planned in init(), which will call applyMovement().
        runtime.run();

        // This combines all movement requests from the last loop, and sends a single command to the motors.
        deviceDrive.flushMovement();

        t.addData("fl", deviceDrive.motorFrontLeft.getCurrentPosition());
        t.addData("fr", deviceDrive.motorFrontRight.getCurrentPosition());
        t.addData("bl", deviceDrive.motorBackLeft.getCurrentPosition());
        t.addData("br", deviceDrive.motorBackRight.getCurrentPosition());
        t.addData("flt", deviceDrive.motorFrontLeft.getTargetPosition());
        t.addData("frt", deviceDrive.motorFrontRight.getTargetPosition());
        t.addData("blt", deviceDrive.motorBackLeft.getTargetPosition());
        t.addData("brt", deviceDrive.motorBackRight.getTargetPosition());

        t.addData("status", "running");
        t.update();
    }

    @Override
    public void stop() {
        runtime.reset();

        deviceCamera.stop();
        deviceDrive.stop(); // Stops the drive motors.

        t.addData("status", "stopping");
        t.update();
    }

    public void config() {
        team = Team.BLUE;
    }
}
