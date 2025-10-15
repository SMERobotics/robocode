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
            telemetry.addData("tag", "found");

            double range = tag.ftcPose.range;
            double bearing = tag.ftcPose.bearing;
            double yaw = tag.ftcPose.yaw;

            telemetry.addData("range", range);
            telemetry.addData("bearing", bearing);
            telemetry.addData("yaw", yaw);

            if (Math.abs(range - 30) < 3) range = 30;
            if (Math.abs(bearing) < 1) bearing = 0;
            if (Math.abs(yaw) < 3) yaw = 0;

            float forward = (float) Range.clip(2 + range / -15, -1, 1); // slow down beginning 45in away, stop at 30in away
            float strafe = (float) Range.clip(bearing / 17, -1, 1); // dynamically adjust bearing based on range?
            float rotate = (float) Range.clip(-yaw / 30, -1, 1);

            forward /= 10;
            if (forward > 0) { // scale to interval +-0.7 to +-0.8
                forward += 0.7f;
            } else if (forward < 0) {
                forward -= 0.7f;
            }

            strafe /= 10;
            if (strafe > 0) { // scale to interval +-0.7 to +-0.8
                strafe += 0.7f;
            } else if (strafe < 0) {
                strafe -= 0.7f;
            }

            rotate /= 10;
            if (rotate > 0) { // scale to interval +-0.6 to +-0.7
                rotate += 0.6f;
            } else if (rotate < 0) {
                rotate -= 0.6f;
            }

            telemetry.addData("forward", forward);
            telemetry.addData("strafe", strafe);
            telemetry.addData("rotate", rotate);

            deviceDrive.update(forward, strafe, rotate);
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
