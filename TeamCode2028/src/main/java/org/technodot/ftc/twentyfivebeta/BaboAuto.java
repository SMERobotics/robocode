package org.technodot.ftc.twentyfivebeta;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.technodot.ftc.twentyfivebeta.common.Alliance;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceCamera;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceDrive;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceExtake;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceIMU;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceIntake;
import org.technodot.ftc.twentyfivebeta.roboctrl.InputController;
import org.technodot.ftc.twentyfivebeta.roboctrl.SilentRunner101;

import java.util.List;

@Autonomous(name="BaboAuto", group="TechnoCode")
public class BaboAuto extends OpMode {

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

        inputController = new SilentRunner101(null, null);

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

         deviceDrive.setDriveState(DeviceDrive.DriveState.AUTO);
    }

    @Override
    public void init_loop() {
        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        telemetry.addData("status", "starting");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("status", "running");
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("status", "stopping");
        telemetry.update();
    }
}
