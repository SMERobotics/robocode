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
import org.technodot.ftc.twentyfivebeta.common.Vector2D;
import org.technodot.ftc.twentyfivebeta.pedro.Follower;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceCamera;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceDrive;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceExtake;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceIntake;
import org.technodot.ftc.twentyfivebeta.robocore.DevicePinpoint;
import org.technodot.ftc.twentyfivebeta.roboctrl.ShotSolver;
import org.technodot.ftc.twentyfivebeta.roboctrl.SilentRunner101;

import java.util.ArrayDeque;
import java.util.Queue;

//@Disabled
@TeleOp(name="CalibrateExtakeMotor", group="TechnoCode")
public class CalibrateExtakeMotor extends OpMode {

    public DeviceCamera deviceCamera;
    public DevicePinpoint devicePinpoint;
    public DeviceDrive deviceDrive;
    public DeviceExtake deviceExtake;
    public DeviceIntake deviceIntake;

    private final Queue<Double> window = new ArrayDeque<>();
    private double sum;
    private double velocity = 0;

    @Override
    public void init() {
        deviceCamera = new DeviceCamera(Alliance.RED);
        deviceCamera.init(hardwareMap, new SilentRunner101(null, null));

        devicePinpoint = new DevicePinpoint(Alliance.RED);
        devicePinpoint.init(hardwareMap, new SilentRunner101(gamepad1, gamepad2));

        deviceDrive = new DeviceDrive(Alliance.RED);
        deviceDrive.init(hardwareMap, new SilentRunner101(gamepad1, gamepad2));
        
        deviceExtake = new DeviceExtake(Alliance.RED);
        deviceExtake.init(hardwareMap, new SilentRunner101(null, null));
        deviceExtake.setExtakeState(DeviceExtake.ExtakeState.OVERRIDE);

        deviceIntake = new DeviceIntake(Alliance.RED);
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
        deviceDrive.start();
        deviceExtake.start();
        deviceIntake.start();

        telemetry.addData("status", "starting");
        telemetry.update();
    }

    @Override
    public void loop() {
        deviceCamera.update();
        devicePinpoint.update();
        deviceDrive.update();

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

        telemetry.addData("h", DevicePinpoint.pinpoint.getHeading(AngleUnit.DEGREES));

        if (DeviceCamera.goalTagDetection != null) {
            telemetry.addData("px", y);
            telemetry.addData("py", x);
            telemetry.addData("pz", Math.hypot(x, y));
            Vector2D relocalization = ShotSolver.getCameraPos(DeviceCamera.goalTagDetection, Alliance.RED);
            if (relocalization != null) {
                telemetry.addData("rx", relocalization.x);
                telemetry.addData("ry", relocalization.y);
                telemetry.addData("rz", Math.hypot(relocalization.x, relocalization.y));
            }
        }

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
        deviceDrive.stop();
        deviceExtake.stop();
        deviceIntake.stop();

        telemetry.addData("status", "stopping");
        telemetry.update();
    }
}
