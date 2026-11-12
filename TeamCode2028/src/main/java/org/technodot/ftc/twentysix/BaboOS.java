package org.technodot.ftc.twentysix;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.technodot.ftc.twentysix.common.Alliance;
import org.technodot.ftc.twentysix.robocore.DeviceDrive;
import org.technodot.ftc.twentysix.robocore.DeviceIntake;
import org.technodot.ftc.twentysix.robocore.DeviceExtake;
import org.technodot.ftc.twentysix.roboctrl.GamepadBindings;

@TeleOp(name="BaboOS", group="TechnoCode")
public class BaboOS extends OpMode {

    public DeviceDrive deviceDrive;
    public DeviceIntake deviceIntake;
    public DeviceExtake deviceExtake;
    public GamepadBindings gamepad;

    @Override
    public void init() {
        gamepad = new GamepadBindings(gamepad1, gamepad2);

        deviceDrive = new DeviceDrive(Alliance.BLUE);
        deviceIntake = new DeviceIntake(Alliance.BLUE);
        deviceExtake = new DeviceExtake(Alliance.BLUE);

        deviceDrive.init(hardwareMap, gamepad, telemetry);
        deviceIntake.init(hardwareMap, gamepad, telemetry);
        deviceExtake.init(hardwareMap, gamepad, telemetry);
    }

    @Override
    public void init_loop() {
        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        deviceDrive.start();
        deviceIntake.start();
        deviceExtake.start();

        telemetry.addData("status", "starting");
        telemetry.update();
    }

    @Override
    public void loop() {
        deviceDrive.update();
        deviceIntake.update();
        deviceExtake.update();

        telemetry.addData("status", "running");
        telemetry.update();
    }

    @Override
    public void stop() {
        deviceDrive.stop();
        deviceIntake.stop();
        deviceExtake.stop();

        telemetry.addData("status", "stopping");
        telemetry.update();
    }
}
