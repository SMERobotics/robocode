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

    public static long now;
    public static long last;

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
        now = System.nanoTime(); // ts call here ideally should be the only call

        // WARNING: ts could ESPECIALLY not be very fun
        for (LynxModule hub : hubs) hub.clearBulkCache();

        deviceCamera.update();
        deviceIMU.update();
        deviceDrive.update();
        deviceExtake.update();
        deviceIntake.update();

        if (DeviceCamera.goalTagDetection != null) telemetry.addData("tag_goal", String.format("b=%f, y=%f", DeviceCamera.goalTagDetection.ftcPose.bearing, DeviceCamera.goalTagDetection.ftcPose.yaw));
        telemetry.addData("field_offset", deviceCamera.getFieldOffset());
//        telemetry.addData("heading_offset", deviceIMU.getHeadingOffset());
        telemetry.addData("h", DeviceIMU.yaw);

        t.addData("ext_vel", deviceExtake.targetVelocity);
//        if (deviceExtake.motorExtakeLeft != null) t.addData("exl_pos", deviceExtake.motorExtakeLeft.getCurrentPosition());
//        if (deviceExtake.motorExtakeRight != null) t.addData("exr_pos", deviceExtake.motorExtakeRight.getCurrentPosition());
        if (deviceExtake.motorExtakeLeft != null) t.addData("exl_vel", deviceExtake.motorExtakeLeft.getVelocity());
        if (deviceExtake.motorExtakeRight != null) t.addData("exr_vel", deviceExtake.motorExtakeRight.getVelocity());
//        if (deviceExtake.motorExtakeLeft != null) t.addData("exl_pwr", deviceExtake.motorExtakeLeft.getPower());
//        if (deviceExtake.motorExtakeRight != null) t.addData("exr_pwr", deviceExtake.motorExtakeRight.getPower());
        t.addData("int_srv" , deviceIntake.statusTelem); // intake servo status, displayed for timing purposes

        telemetry.addData("cl_a", deviceIntake.leftArtifact);
        telemetry.addData("cr_a", deviceIntake.rightArtifact);

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
