package org.technodot.ftc.twentyfivebeta;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.technodot.ftc.twentyfivebeta.common.Alliance;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceCamera;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceDrive;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceExtake;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceIMU;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceIntake;
import org.technodot.ftc.twentyfivebeta.roboctrl.InputController;
import org.technodot.ftc.twentyfivebeta.roboctrl.SilentRunner101;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@TeleOp(name="BaboOS", group="TechnoCode")
public class BaboOS extends OpMode {

    // WARNING: ts could not be very fun
    List<LynxModule> hubs;

    // all updates should be theoretically done in this order, based on usage
    public DeviceCamera deviceCamera;
    public DeviceIMU deviceIMU;
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
        deviceIMU = new DeviceIMU(alliance);
        deviceDrive = new DeviceDrive(alliance);
        deviceExtake = new DeviceExtake(alliance);
        deviceIntake = new DeviceIntake(alliance);

        deviceCamera.init(hardwareMap, inputController);
        deviceIMU.init(hardwareMap, inputController);
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
        deviceIMU.start();
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
        deviceIMU.update();
        deviceDrive.update();
        deviceExtake.update();
        deviceIntake.update();

        if (DeviceCamera.goalTagDetection != null) telemetry.addData("tag_goal", String.format("r=%f, b=%f, e=%f, y=%f", DeviceCamera.goalTagDetection.ftcPose.range, DeviceCamera.goalTagDetection.ftcPose.bearing, DeviceCamera.goalTagDetection.ftcPose.elevation, DeviceCamera.goalTagDetection.ftcPose.yaw));
        telemetry.addData("field_offset", deviceCamera.getFieldOffset());
        telemetry.addData("h", DeviceIMU.yaw);

        if (DeviceCamera.goalTagDetection != null) t.addData("r", DeviceCamera.goalTagDetection.ftcPose.range);
        if (DeviceCamera.goalTagDetection != null) t.addData("b", DeviceCamera.goalTagDetection.ftcPose.bearing + (alliance == Alliance.BLUE ? Configuration.DRIVE_AIM_OFFSET : -Configuration.DRIVE_AIM_OFFSET) + (DeviceIntake.targetSide == DeviceIntake.IntakeSide.LEFT ? -Configuration.DRIVE_AIM_INTAKE_OFFSET : Configuration.DRIVE_AIM_INTAKE_OFFSET));

        t.addData("ext_vel", deviceExtake.targetVelocity);
        if (deviceExtake.motorExtakeLeft != null) t.addData("exl_vel", deviceExtake.motorExtakeLeft.getVelocity());
        if (deviceExtake.motorExtakeRight != null) t.addData("exr_vel", deviceExtake.motorExtakeRight.getVelocity());
        t.addData("int_srv" , deviceIntake.statusTelem); // intake servo status, displayed for timing purposes

        telemetry.addData("cl_a", deviceIntake.leftArtifact);
        telemetry.addData("cr_a", deviceIntake.rightArtifact);
        
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
        deviceIMU.stop();
        deviceDrive.stop();
        deviceExtake.stop();
        deviceIntake.stop();

        telemetry.addData("status", "stopping");
        telemetry.update();
    }
}
