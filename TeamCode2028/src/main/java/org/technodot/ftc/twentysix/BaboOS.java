package org.technodot.ftc.twentysix;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.technodot.ftc.twentysix.common.Alliance;
import org.technodot.ftc.twentysix.robocore.DeviceDrive;
import org.technodot.ftc.twentysix.robocore.DeviceIntake;
import org.technodot.ftc.twentysix.robocore.DeviceExtake;
import org.technodot.ftc.twentysix.robocore.DevicePinpoint;
import org.technodot.ftc.twentysix.robocore.DeviceLimelight;
import org.technodot.ftc.twentysix.roboctrl.GamepadBindings;

import java.util.List;

@TeleOp(name="BaboOS", group="TechnoCode")
public class BaboOS extends OpMode {

    public Alliance alliance = Alliance.BLUE;
    private List<LynxModule> hubs;

    public DeviceDrive deviceDrive;
    public DeviceIntake deviceIntake;
    public DeviceExtake deviceExtake;
    public GamepadBindings gamepad;
    public DevicePinpoint devicePinpoint;
    public DeviceLimelight deviceLimelight;

    protected void config() {
    }

    @Override
    public void init() {
        config();
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        gamepad = new GamepadBindings(gamepad1, gamepad2);

        deviceDrive = new DeviceDrive(alliance);
        deviceIntake = new DeviceIntake(alliance);
        deviceExtake = new DeviceExtake(alliance);
        devicePinpoint = new DevicePinpoint(alliance);
        deviceLimelight = new DeviceLimelight(alliance);

        deviceDrive.init(hardwareMap, gamepad, telemetry);
        deviceIntake.init(hardwareMap, gamepad, telemetry);
        deviceExtake.init(hardwareMap, gamepad, telemetry);
        devicePinpoint.init(hardwareMap, gamepad, telemetry);
        deviceLimelight.init(hardwareMap, gamepad, telemetry);
    }

    @Override
    public void init_loop() {
        deviceLimelight.update();
        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        deviceDrive.start();
        deviceIntake.start();
        deviceExtake.start();
        devicePinpoint.start();
        deviceLimelight.start();

        telemetry.addData("status", "starting");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Read a fresh bulk snapshot on each hub's first eligible read this cycle.
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        devicePinpoint.update();
        deviceDrive.updateFieldCentric(devicePinpoint.pinpoint.getHeading(AngleUnit.RADIANS));
        deviceLimelight.update();
        deviceIntake.update();
        deviceExtake.setHiveObservation(deviceLimelight.getHiveObservation(alliance),
                deviceLimelight.getLastHiveTagSeenNanos(alliance));
        deviceExtake.update();

        telemetry.addData("status", "running");
        telemetry.addData("alliance", alliance);
        telemetry.update();
    }

    @Override
    public void stop() {
        deviceDrive.stop();
        deviceIntake.stop();
        deviceExtake.stop();
        devicePinpoint.stop();
        deviceLimelight.stop();

        telemetry.addData("status", "stopping");
        telemetry.update();
    }
}
