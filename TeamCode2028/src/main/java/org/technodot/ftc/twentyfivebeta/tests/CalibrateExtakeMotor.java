package org.technodot.ftc.twentyfivebeta.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.technodot.ftc.twentyfivebeta.common.Alliance;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceCamera;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceExtake;
import org.technodot.ftc.twentyfivebeta.roboctrl.SilentRunner101;

@TeleOp(name="CalibrateExtakeMotor", group="TechnoCode")
public class CalibrateExtakeMotor extends OpMode {

    public DeviceCamera deviceCamera;
    public DeviceExtake deviceExtake;

    private double velocity = 0;

    @Override
    public void init() {
        deviceExtake = new DeviceExtake(Alliance.BLUE);
        deviceExtake.init(hardwareMap, new SilentRunner101(null, null));
        deviceExtake.setExtakeState(DeviceExtake.ExtakeState.OVERRIDE);

        deviceCamera = new DeviceCamera(Alliance.BLUE);
        deviceCamera.init(hardwareMap, new SilentRunner101(null, null));
    }

    @Override
    public void init_loop() {
        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        deviceExtake.start();

        telemetry.addData("status", "starting");
        telemetry.update();
    }

    @Override
    public void loop() {
        deviceCamera.update();

        if (gamepad1.dpad_up) {
            velocity += 1;
        } else if (gamepad1.dpad_down) {
            velocity -= 1;
        }

        deviceExtake.setExtakeOverride(velocity * 20);

        deviceExtake.update();

        AprilTagDetection tag = deviceCamera.getGoalDetection();

        telemetry.addData("ext", velocity * 20);
        if (tag != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", tag.id, tag.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", tag.ftcPose.pitch, tag.ftcPose.roll, tag.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", tag.ftcPose.range, tag.ftcPose.bearing, tag.ftcPose.elevation));
        }
        telemetry.addData("status", "running");
        telemetry.update();
    }

    @Override
    public void stop() {
        deviceExtake.stop();

        telemetry.addData("status", "stopping");
        telemetry.update();
    }
}
