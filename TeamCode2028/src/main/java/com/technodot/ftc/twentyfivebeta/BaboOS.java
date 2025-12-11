package com.technodot.ftc.twentyfivebeta;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.technodot.ftc.twentyfivebeta.common.Alliance;
import com.technodot.ftc.twentyfivebeta.robocore.DeviceCamera;
import com.technodot.ftc.twentyfivebeta.robocore.DeviceDrive;
import com.technodot.ftc.twentyfivebeta.robocore.DeviceExtake;
import com.technodot.ftc.twentyfivebeta.robocore.DeviceIMU;
import com.technodot.ftc.twentyfivebeta.robocore.DeviceIntake;
import com.technodot.ftc.twentyfivebeta.roboctrl.InputController;
import com.technodot.ftc.twentyfivebeta.roboctrl.SilentRunner101;

@TeleOp(name="BaboOS", group="TechnoCode")
public class BaboOS extends OpMode {

    // all updates should be theoretically done in this order, based on usage
    public DeviceCamera deviceCamera;
    public DeviceIMU deviceIMU;
    public DeviceDrive deviceDrive;
    public DeviceExtake deviceExtake;
    public DeviceIntake deviceIntake;

    public InputController inputController;

    public Alliance alliance = Alliance.BLUE;

    protected void config() {
    }

    @Override
    public void init() {
        config();
        inputController = new SilentRunner101(gamepad1, gamepad2);

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

        telemetry.addData("status", "starting");
        telemetry.update();
    }

    @Override
    public void loop() {
        long now = System.nanoTime(); // ts call here ideally should be the only call

        deviceCamera.update();
        deviceIMU.update();
        deviceDrive.update();
        deviceExtake.update();
        deviceIntake.update();

        if (deviceCamera.goalTagDetection != null) telemetry.addData("tag_goal", String.format("b=%f, y=%f", deviceCamera.goalTagDetection.ftcPose.bearing, deviceCamera.goalTagDetection.ftcPose.yaw));
        telemetry.addData("field_offset", deviceCamera.getFieldOffset());
        telemetry.addData("heading_offset", deviceIMU.getHeadingOffset());

        if (deviceExtake.motorExtakeLeft != null) telemetry.addData("extake_left", deviceExtake.motorExtakeLeft.getCurrentPosition());
        if (deviceExtake.motorExtakeRight != null) telemetry.addData("extake_right", deviceExtake.motorExtakeRight.getCurrentPosition());

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
