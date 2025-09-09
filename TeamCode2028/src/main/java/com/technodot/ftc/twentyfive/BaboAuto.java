package com.technodot.ftc.twentyfive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;
import com.technodot.ftc.twentyfive.robocore.DeviceCamera;
import com.technodot.ftc.twentyfive.robocore.DeviceDrive;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name="BaboAuto", group="TechnoCode")
public class BaboAuto extends OpMode {

    public DeviceCamera deviceCamera = new DeviceCamera();
    public DeviceDrive deviceDrive = new DeviceDrive();

    public Team team = Team.BLUE;

    @Override
    public void init() {
        deviceCamera.init(hardwareMap, team);
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
        AprilTagDetection tag = deviceCamera.update();
        if (tag != null) {
            deviceDrive.update(
                    (float) Range.clip(tag.ftcPose.range - 12, -1, 1), // stop 12 inches away
                    (float) Range.clip(tag.ftcPose.bearing, -1, 1),
                    (float) Range.clip(tag.ftcPose.yaw, -1, 1)
            );
            telemetry.addData("tag", "found");
        } else {
            deviceDrive.zero();
            telemetry.addData("tag", "not found");
        }

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