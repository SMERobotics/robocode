package org.technodot.ftc.twentyfivebeta.tests;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.technodot.ftc.twentyfivebeta.Configuration;
import org.technodot.ftc.twentyfivebeta.common.Alliance;
import org.technodot.ftc.twentyfivebeta.pedro.Follower;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceCamera;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceExtake;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceIntake;
import org.technodot.ftc.twentyfivebeta.robocore.DevicePinpoint;
import org.technodot.ftc.twentyfivebeta.roboctrl.SilentRunner101;

import java.util.ArrayDeque;
import java.util.Queue;

//@Disabled
@TeleOp(name="CalibrateExtakeMotor", group="TechnoCode")
public class CalibrateExtakeMotor extends OpMode {

    public DeviceCamera deviceCamera;
    public DevicePinpoint devicePinpoint;
    public DeviceExtake deviceExtake;
    public DeviceIntake deviceIntake;

    public Follower follower;

    public final double TARGET_X = 0;
    public final double TARGET_Y = 144;

    private final Queue<Double> window = new ArrayDeque<>();
    private double sum;
    private double velocity = 0;

    @Override
    public void init() {
        follower = Configuration.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.PI / 2));
        follower.update();

        deviceCamera = new DeviceCamera(Alliance.BLUE);
        deviceCamera.init(hardwareMap, new SilentRunner101(null, null));

        devicePinpoint = new DevicePinpoint(Alliance.BLUE);
        devicePinpoint.init(hardwareMap, new SilentRunner101(gamepad1, gamepad2));

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
        deviceCamera.start();
        devicePinpoint.start();
        deviceExtake.start();
        deviceIntake.start();

        telemetry.addData("status", "starting");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y / 5,
                gamepad1.left_stick_x / 5,
                gamepad1.right_stick_x / 5,
                false
        );

        deviceCamera.update();
        devicePinpoint.update();

        if (gamepad1.dpad_up) {
            velocity += 1;
        } else if (gamepad1.dpad_down) {
            velocity -= 1;
        }

        deviceExtake.setExtakeOverride(velocity * 10);

        deviceExtake.update();
        deviceIntake.update();

        AprilTagDetection tag = deviceCamera.getGoalDetection();

        double x = devicePinpoint.pinpoint.getPosX(DistanceUnit.INCH);
        double y = devicePinpoint.pinpoint.getPosY(DistanceUnit.INCH);
        double h = devicePinpoint.pinpoint.getHeading(AngleUnit.RADIANS);

        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("h", h);

        telemetry.addData("d", Math.hypot(devicePinpoint.pinpoint.getPosX(DistanceUnit.INCH), devicePinpoint.pinpoint.getPosY(DistanceUnit.INCH)));

        double targetAngle = Math.atan2(TARGET_Y - y, TARGET_X - x);
        double headingError = targetAngle - h;

        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;

        telemetry.addData("e", Math.toDegrees(headingError));

        telemetry.addData("ext", velocity * 10);
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
        deviceCamera.stop();
        devicePinpoint.stop();
        deviceExtake.stop();
        deviceIntake.stop();

        telemetry.addData("status", "stopping");
        telemetry.update();
    }
}
