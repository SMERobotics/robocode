package org.technodot.ftc.twentyfivebeta;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.technodot.ftc.twentyfivebeta.common.Alliance;
import org.technodot.ftc.twentyfivebeta.common.Vector2D;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceCamera;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceDrive;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceExtake;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceIntake;
import org.technodot.ftc.twentyfivebeta.robocore.DevicePinpoint;
import org.technodot.ftc.twentyfivebeta.roboctrl.InputController;
import org.technodot.ftc.twentyfivebeta.roboctrl.ShotSolver;
import org.technodot.ftc.twentyfivebeta.roboctrl.SilentRunner101;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@TeleOp(name="BaboOS", group="TechnoCode")
public class BaboOS extends OpMode {

    // WARNING: ts could not be very fun
    List<LynxModule> hubs;

    // all updates should be theoretically done in this order, based on usage
    public DeviceCamera deviceCamera;
    public DevicePinpoint devicePinpoint;
    public DeviceDrive deviceDrive;
    public DeviceExtake deviceExtake;
    public DeviceIntake deviceIntake;

    public InputController inputController;

    public Alliance alliance = Alliance.BLUE;

    public Telemetry t = FtcDashboard.getInstance().getTelemetry();

    public static long begin;
    public static long now;
    public static long last;

    // Endgame vibration state machine
    private int lastVibrationSecond = -1;

    protected void config() {
    }

    @Override
    public void init() {
        config();
        inputController = new SilentRunner101(gamepad1, gamepad2);

        // WARNING: ts could not be very fun
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        deviceCamera = new DeviceCamera(alliance);
        devicePinpoint = new DevicePinpoint(alliance);
        deviceDrive = new DeviceDrive(alliance);
        deviceExtake = new DeviceExtake(alliance);
        deviceIntake = new DeviceIntake(alliance);

        deviceCamera.init(hardwareMap, inputController);
        devicePinpoint.init(hardwareMap, inputController);
        deviceDrive.init(hardwareMap, inputController);
        deviceExtake.init(hardwareMap, inputController);
        deviceIntake.init(hardwareMap, inputController);

        Configuration.DRIVE_AIM_OFFSET = 2.0;
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

        lastVibrationSecond = -1;
        begin = System.nanoTime();

        Configuration.DRIVE_AIM_OFFSET = 2.0; // ts may have been overridden in auto
        Configuration.EXTAKE_MODEL_VELOCITY_SIMPLE_RANGE_SHIFT = 0.0;

        telemetry.addData("status", "starting");
        telemetry.update();
    }

    @Override
    public void loop() {
        now = System.nanoTime(); // ts call here ideally should be the only call

        // WARNING: ts could ESPECIALLY not be very fun
        for (LynxModule hub : hubs) hub.clearBulkCache();

        deviceCamera.update();
        devicePinpoint.update();
        deviceDrive.update();
        deviceExtake.update();
        deviceIntake.update();

        telemetry.addData("x", DevicePinpoint.pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("y", DevicePinpoint.pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("h", DevicePinpoint.pinpoint.getHeading(AngleUnit.DEGREES));

        t.addData("x", DevicePinpoint.pinpoint.getPosX(DistanceUnit.INCH));
        t.addData("y", DevicePinpoint.pinpoint.getPosY(DistanceUnit.INCH));

        if (DeviceCamera.goalTagDetection != null) {
            Vector2D goal = ShotSolver.getGoalPos(DeviceCamera.goalTagDetection, alliance);
            if (goal != null) {
                telemetry.addData("gx", goal.x);
                telemetry.addData("gy", goal.y);
            }
            Vector2D relocalization = ShotSolver.getCameraPos(DeviceCamera.goalTagDetection, alliance);
            if (relocalization != null) {
                telemetry.addData("rx", relocalization.x);
                telemetry.addData("ry", relocalization.y);
            }
        }

        t.addData("h", DevicePinpoint.pinpoint.getHeading(AngleUnit.DEGREES));
        t.addData("a", ShotSolver.getGoalYawError(DeviceCamera.goalTagDetection, this.alliance));
        if (DeviceCamera.goalTagDetection != null && DeviceCamera.goalTagDetection.ftcPose != null) t.addData("b", DeviceCamera.goalTagDetection.ftcPose.bearing);

            telemetry.addData("pb_kp", Configuration.PINPOINT_BEARING_P);
        telemetry.addData("pb_ki", Configuration.PINPOINT_BEARING_I);
        telemetry.addData("pb_kd", Configuration.PINPOINT_BEARING_D);
        telemetry.addData("pb_kf", Configuration.PINPOINT_BEARING_F);

        if (DeviceCamera.goalTagDetection != null && DeviceCamera.goalTagDetection.ftcPose != null) telemetry.addData("tag_goal", String.format("r=%f, b=%f, e=%f, y=%f", DeviceCamera.goalTagDetection.ftcPose.range, DeviceCamera.goalTagDetection.ftcPose.bearing, DeviceCamera.goalTagDetection.ftcPose.elevation, DeviceCamera.goalTagDetection.ftcPose.yaw));
        telemetry.addData("field_offset", deviceCamera.getFieldOffset());

        t.addData("ext_vel", deviceExtake.targetVelocity);
        if (deviceExtake.motorExtakeLeft != null) t.addData("exl_vel", deviceExtake.motorExtakeLeft.getVelocity());
        if (deviceExtake.motorExtakeRight != null) t.addData("exr_vel", deviceExtake.motorExtakeRight.getVelocity());
        t.addData("int_srv" , deviceIntake.statusTelem); // intake servo status, displayed for timing purposes

        telemetry.addData("cl_a", deviceIntake.leftArtifact);
        telemetry.addData("cr_a", deviceIntake.rightArtifact);
        telemetry.addData("int_s", deviceIntake.targetSide);
        
        // Time-based vibration reminders for endgame (only when not in DEBUG mode)
        if (!Configuration.DEBUG && inputController instanceof SilentRunner101) {
            SilentRunner101 ctrl = (SilentRunner101) inputController;
            int elapsedSeconds = (int) ((now - begin) / 1_000_000_000L);

            if (elapsedSeconds >= 100 && elapsedSeconds <= 120 && elapsedSeconds != lastVibrationSecond) {
                lastVibrationSecond = elapsedSeconds;

                if (elapsedSeconds == 100 || elapsedSeconds == 110 || elapsedSeconds == 115) {
                    ctrl.vibrateEndgame();
                 } else if (elapsedSeconds == 120) {
                    ctrl.vibrateEndgameFinale();
                } else {
//                    ctrl.vibrateEndgameTick(); // ARRI DOESNT LIKE TS
                }
            }
        }

        t.addData("t", (now - last) / 1e6);
        telemetry.addData("t", (now - last) / 1e6);
        last = now;

        t.update();

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
