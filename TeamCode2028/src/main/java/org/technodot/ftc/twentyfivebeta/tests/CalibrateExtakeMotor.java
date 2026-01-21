package org.technodot.ftc.twentyfivebeta.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.technodot.ftc.twentyfivebeta.common.Alliance;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceCamera;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceExtake;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceIntake;
import org.technodot.ftc.twentyfivebeta.roboctrl.SilentRunner101;

import java.util.ArrayDeque;
import java.util.Queue;

@Disabled
@TeleOp(name="CalibrateExtakeMotor", group="TechnoCode")
public class CalibrateExtakeMotor extends OpMode {

    public DeviceCamera deviceCamera;
    public DeviceExtake deviceExtake;
    public DeviceIntake deviceIntake;

    private final Queue<Double> window = new ArrayDeque<>();
    private double sum;
    private double velocity = 0;

    @Override
    public void init() {
        deviceCamera = new DeviceCamera(Alliance.BLUE);
        deviceCamera.init(hardwareMap, new SilentRunner101(null, null));

        deviceExtake = new DeviceExtake(Alliance.BLUE);
        deviceExtake.init(hardwareMap, new SilentRunner101(null, null));
        deviceExtake.setExtakeState(DeviceExtake.ExtakeState.OVERRIDE);

        deviceIntake = new DeviceIntake(Alliance.BLUE);
        deviceIntake.init(hardwareMap, new SilentRunner101(gamepad1, gamepad2));
    }

    @Override
    public void init_loop() {
        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        deviceExtake.start();
        deviceIntake.start();

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
        deviceIntake.update();

        AprilTagDetection tag = deviceCamera.getGoalDetection();

        telemetry.addData("ext", velocity * 20);
        if (tag != null) {
            sum += tag.ftcPose.range;
            window.add(tag.ftcPose.range);
            if (window.size() > 67) sum -= window.remove();
            telemetry.addData("r", sum / window.size());
        }
        telemetry.addData("status", "running");
        telemetry.update();
    }

    @Override
    public void stop() {
        deviceExtake.stop();
        deviceIntake.stop();

        telemetry.addData("status", "stopping");
        telemetry.update();
    }
}
