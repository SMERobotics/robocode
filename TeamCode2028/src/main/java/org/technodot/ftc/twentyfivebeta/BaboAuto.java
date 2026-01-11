package org.technodot.ftc.twentyfivebeta;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.technodot.ftc.twentyfivebeta.batch.Batch;
import org.technodot.ftc.twentyfivebeta.batch.Callback;
import org.technodot.ftc.twentyfivebeta.batch.ContiguousSequence;
import org.technodot.ftc.twentyfivebeta.batch.InterruptibleCallback;
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

    public AutoType autoType = AutoType.CLOSE;

    public Batch runtime = new Batch();

    public Telemetry t = FtcDashboard.getInstance().getTelemetry();

    public final long S = 1000L;
    public final long X = 6767L; // general "idfk" delay HEHEHEHA

    public static long now;
    public static long last;

    public enum AutoType {
        CLOSE,
        FAR
    }

    public void configure() {
        switch (autoType) {
            case CLOSE:
                runtime.plan(new ContiguousSequence()
                        .then((Callback) () -> deviceDrive.addMovement(-4.4, alliance.apply(-0.1), alliance.apply(DeviceIMU.GOAL_DEG)))
                        .then(X, (InterruptibleCallback) () -> deviceDrive.isReady())

                        .then((Callback) () -> {
                            deviceDrive.addMovement(0, 0, alliance.apply(-DeviceIMU.GOAL_DEG));

                            // prepare to shoot
                            // may be kinda late
                            deviceExtake.setExtakeOverride(1200);
                            deviceExtake.setExtakeState(DeviceExtake.ExtakeState.OVERRIDE);
                        })
                        .then(X, (InterruptibleCallback) () -> deviceDrive.isReady())

                        .then(S, (InterruptibleCallback) () -> {
                            deviceDrive.stageAim();
                            return deviceDrive.isReady();
                        })
                        .then(X, (InterruptibleCallback) () -> deviceExtake.isReady())

                        // DOES NOT UTILIZE OBELISK & COLOR SENSOR YET, ONLY SHOOTS 2
                        // TODO: dynamically reconfigure shots based on obelisk
                        .then((Callback) () -> deviceIntake.triggerSequenceShoot())
                        .then(X, (InterruptibleCallback) () -> deviceIntake.isEmpty())
                        .delay(670)

                        .then((Callback) () -> {
                            deviceExtake.setExtakeState(DeviceExtake.ExtakeState.IDLE);
                            deviceExtake.setExtakeOverride(0);

                            deviceDrive.addMovement(-1.0, -2.0, 0.0);
                        })
                        .then(X, (InterruptibleCallback) () -> deviceDrive.isReady())
                );

                break;
            case FAR:

                // move the robot forward and start up the extake
                runtime.plan(0, (Callback) () -> deviceDrive.addMovement(0.67, 0.0, 0.0));
                runtime.plan(0, (Callback) () -> {
                    deviceExtake.setExtakeOverride(1536);
                    deviceExtake.setExtakeState(DeviceExtake.ExtakeState.OVERRIDE);
                });

                runtime.plan(new ContiguousSequence(1000)
                        // aim at the goal, nice and long
                        .then(3000, (InterruptibleCallback) () -> {
                            deviceDrive.stageAim();
                            return deviceDrive.isReady() && deviceExtake.isReady();
                        })
//                        .then(2500, (Callback) () -> deviceDrive.stageAim())

                        // trigger 2 shots
                        .then((Callback) () -> deviceIntake.triggerSequenceShoot())
                        .then(X, (InterruptibleCallback) () -> {
                            deviceDrive.stageAim();
                            return deviceIntake.isEmpty();
                        })

                        // grab the 3rd artifact
//                        .then((Callback) () -> deviceIntake.triggerNudge())
                        .then((Callback) () -> deviceIntake.setIntakeIn())
                        .then(500, (Callback) () -> deviceDrive.stageAim())
                        .then((Callback) () -> deviceIntake.setIntakeIdle())

                        // trigger the 3rd shot
                        .then((Callback) () -> deviceIntake.triggerSequenceShoot())
                        .then(X, (InterruptibleCallback) () -> {
                            deviceDrive.stageAim();
                            return deviceIntake.isEmpty();
                        })
                        .then(500, (Callback) () -> deviceDrive.stageAim())

                        // shutdown extake and go to ts balls
                        .then((Callback) () -> {
                            deviceExtake.setExtakeState(DeviceExtake.ExtakeState.IDLE);
                            deviceExtake.setExtakeOverride(0);

                            deviceDrive.addMovement(0.0, 0.0, alliance.apply(-92));
                        })
                        .then(1000, (InterruptibleCallback) () -> deviceDrive.isReady())

                        .then((Callback) () -> deviceIntake.setIntakeIn())

                        .then((Callback) () -> deviceDrive.addMovement(4.0, alliance.apply(-0.3), 0.0))
//                        .then(1000, (InterruptibleCallback) () -> deviceDrive.isReady())
                        .delay(3000)

                        .then((Callback) () -> deviceDrive.addMovement(-3.9, 0.0, alliance.apply(-2.0)))
                        .then((Callback) () -> {
                            deviceExtake.setExtakeOverride(1500);
                            deviceExtake.setExtakeState(DeviceExtake.ExtakeState.OVERRIDE);
                        })
//                        .then(X, (InterruptibleCallback) () -> deviceDrive.isReady())
                        .delay(3000) // realistically too long, just for testing purposes

                        .then((Callback) () -> deviceDrive.addMovement(0.0, 0.0, alliance.apply(90)))
//                        .then(X, (InterruptibleCallback) () -> deviceDrive.isReady())
                        .delay(1500) // realistically too long, just for testing purposes
                        .then((Callback) () -> deviceIntake.setIntakeIdle())

                        .then((Callback) () -> deviceDrive.addMovement(-0.3, 0.0, 0.0))
                        .delay(300)

                        .then(3000, (InterruptibleCallback) () -> {
                            deviceDrive.stageAim();
                            return deviceDrive.isReady() && deviceExtake.isReady();
                        })

                        // trigger 2 shots
                        .then((Callback) () -> deviceIntake.triggerSequenceShoot())
                        .then(X, (InterruptibleCallback) () -> {
                            deviceDrive.stageAim();
                            return deviceIntake.isEmpty();
                        })
                        .then(500, (Callback) () -> deviceDrive.stageAim())

                        .then((Callback) () -> {
                            deviceExtake.setExtakeState(DeviceExtake.ExtakeState.IDLE);
                            deviceExtake.setExtakeOverride(0);

                            deviceDrive.addMovement(2.67, alliance.apply(0.67), alliance.apply(-25.0));
                        })

                );

                break;
        }
    }

    protected void config() {
        autoType = AutoType.CLOSE;
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

        configure();
    }

    @Override
    public void init_loop() {
        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        config(); // tf??? IDFK Y BUT IT WORKS SO FUCK IT

        deviceCamera.start();
        deviceIMU.start();
        deviceDrive.start();
        deviceExtake.start();
        deviceIntake.start();

        // recalibrate IMU heading
        double headingOffset = 0;
        switch (autoType) {
            case CLOSE:
                headingOffset = alliance.apply(-36.0);
                break;
            case FAR:
                headingOffset = alliance.apply(-90.0);
                break;
        }
        deviceIMU.zeroYaw();
        deviceIMU.setHeadingOffset(headingOffset);

        // if IMU can't recalibrate in this time, put a delay before each batch auto

        telemetry.addData("status", "starting");
        telemetry.update();
    }

    @Override
    public void loop() {
        now = System.nanoTime(); // ts call here ideally should be the only call

        // WARNING: ts could ESPECIALLY not be very fun
        // especially in auto, bulk reading could result in poorer quality data
        for (LynxModule hub : hubs) hub.clearBulkCache();

        this.runtime.run();

        deviceCamera.update();
        deviceIMU.update();
        deviceDrive.update();
        deviceExtake.update();
        deviceIntake.update();

        if (DeviceCamera.goalTagDetection != null) telemetry.addData("tag_goal", String.format("b=%f, y=%f", DeviceCamera.goalTagDetection.ftcPose.bearing, DeviceCamera.goalTagDetection.ftcPose.yaw));
        telemetry.addData("field_offset", deviceCamera.getFieldOffset());
        telemetry.addData("h", DeviceIMU.yaw);

        t.addData("ext_vel", deviceExtake.targetVelocity);
        if (deviceExtake.motorExtakeLeft != null) t.addData("exl_vel", deviceExtake.motorExtakeLeft.getVelocity());
        if (deviceExtake.motorExtakeRight != null) t.addData("exr_vel" , deviceExtake.motorExtakeRight.getVelocity());
        t.addData("int_srv" , deviceIntake.statusTelem); // intake servo status, displayed for timing purposes

        telemetry.addData("cl_a", deviceIntake.leftArtifact);
        telemetry.addData("cr_a", deviceIntake.rightArtifact);

        if (DeviceCamera.goalTagDetection != null) t.addData("b", DeviceCamera.goalTagDetection.ftcPose.bearing);

        t.addData("t", (now - last) / 1e6);
        telemetry.addData("t", (now - last) / 1e6);
        last = now;

        t.update();

        telemetry.addData("status", "running");
        telemetry.update();
    }

    @Override
    public void stop() {
        this.runtime.reset();

        deviceCamera.stop();
        deviceIMU.stop();
        deviceDrive.stop();
        deviceExtake.stop();
        deviceIntake.stop();

        telemetry.addData("status", "stopping");
        telemetry.update();
    }
}
