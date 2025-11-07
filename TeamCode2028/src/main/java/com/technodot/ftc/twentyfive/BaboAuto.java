package com.technodot.ftc.twentyfive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;
import com.technodot.ftc.twentyfive.common.Team;
import com.technodot.ftc.twentyfive.robocore.DeviceCamera;
import com.technodot.ftc.twentyfive.robocore.DeviceDrive;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name="BaboAuto", group="TechnoCode")
public class BaboAuto extends OpMode {

    public DeviceCamera deviceCamera = new DeviceCamera();
    public DeviceDrive deviceDrive = new DeviceDrive();

    public MultipleTelemetry t;

    public final Team team = Team.BLUE;

    @Override
    public void init() {
        t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        deviceCamera.init(hardwareMap, team);
        deviceDrive.init(hardwareMap);
    }

    @Override
    public void init_loop() {
        t.addData("status", "initialized");
        t.update();
    }

    @Override
    public void start() {
        deviceCamera.start();

        t.addData("status", "starting");
        t.update();
    }

    @Override
    public void loop() {
        // ts goes here

        t.addData("status", "running");
        t.update();
    }

    @Override
    public void stop() {
        deviceCamera.stop();

        t.addData("status", "stopping");
        t.update();
    }
}
