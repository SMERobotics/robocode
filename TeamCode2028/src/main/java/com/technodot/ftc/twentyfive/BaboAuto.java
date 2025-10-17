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
        AprilTagDetection tag = deviceCamera.update();
        if (tag != null) {
            t.addData("tag", "found");

            double range = tag.ftcPose.range;
            double bearing = tag.ftcPose.bearing;
            double yaw = tag.ftcPose.yaw;

            t.addData("range", range);
            t.addData("bearing", bearing);
            t.addData("yaw", yaw);

            if (Math.abs(range - 30) < 3) range = 30;
            if (Math.abs(bearing) < 3) bearing = 0;
            if (Math.abs(yaw) < 3) yaw = 0;

            float forward = (float) Range.clip(2 + range / -15, -1, 1); // slow down beginning 45in away, stop at 30in away
            float strafe = (float) Range.clip(bearing / 17, -1, 1); // dynamically adjust bearing based on range?
            float rotate = (float) Range.clip(-yaw / 30, -1, 1);

            t.addData("forward", forward);
            t.addData("strafe", strafe);
            t.addData("rotate", rotate);

            forward *= 0.6f;
            strafe *= 0.6f;
            rotate *= 0.6f;

            deviceDrive.update(forward, strafe, rotate * 0.0f);
        } else {
            deviceDrive.zero();
            t.addData("tag", "not found");
        }

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
