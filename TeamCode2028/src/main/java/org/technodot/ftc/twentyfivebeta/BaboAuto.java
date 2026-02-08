package org.technodot.ftc.twentyfivebeta;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.technodot.ftc.twentyfivebeta.batch.Batch;
import org.technodot.ftc.twentyfivebeta.batch.Callback;
import org.technodot.ftc.twentyfivebeta.batch.ContiguousSequence;
import org.technodot.ftc.twentyfivebeta.batch.InterruptibleCallback;
import org.technodot.ftc.twentyfivebeta.common.Alliance;
import org.technodot.ftc.twentyfivebeta.pedro.Follower;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceCamera;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceDrive;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceExtake;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceIMU;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceIntake;
import org.technodot.ftc.twentyfivebeta.robocore.DevicePinpoint;
import org.technodot.ftc.twentyfivebeta.roboctrl.InputController;
import org.technodot.ftc.twentyfivebeta.roboctrl.SilentRunner101;

import java.util.List;

@Autonomous(name="BaboAuto", group="TechnoCode")
public class BaboAuto extends OpMode {

    // WARNING: ts could not be very fun
    List<LynxModule> hubs;

    private Follower follower;

    // all updates should be theoretically done in this order, based on usage
    public DeviceCamera deviceCamera;
    public DeviceExtake deviceExtake;
    public DeviceIntake deviceIntake;

    public InputController inputController;

    public Alliance alliance = Alliance.BLUE;

    public AutoType autoType = AutoType.CLOSE;

    public Batch runtime;

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
                break;
            case FAR:
                Pose start = P(48 + 9, 0 + 6.7, 0);
                Pose shootPreload = P(48, 24, 0);

                follower.setStartingPose(start);

                PathChain start_shootPreload = follower.pathBuilder().addPath(
                        new BezierLine(start, shootPreload)
                ).setLinearHeadingInterpolation(start.getHeading(), shootPreload.getHeading())
                .build();

                runtime.plan(new ContiguousSequence(0)
                        .then((Callback) () -> follower.followPath(start_shootPreload))
                        .then(X, (InterruptibleCallback) () -> follower.isReady())
                );

                break;
        }
    }

    protected void config() {
        autoType = AutoType.CLOSE;
    }

    @Override
    public void init() {
        runtime = new Batch(); // reset runtime on init?
        config();

        inputController = new SilentRunner101(null, null);

        // WARNING: ts could not be very fun
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        follower = Configuration.createFollower(hardwareMap);

        deviceCamera = new DeviceCamera(alliance);
        deviceExtake = new DeviceExtake(alliance);
        deviceIntake = new DeviceIntake(alliance);

        deviceCamera.init(hardwareMap, inputController);
        deviceExtake.init(hardwareMap, inputController);
        deviceIntake.init(hardwareMap, inputController);

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
        deviceExtake.start();
        deviceIntake.start();

        telemetry.addData("status", "starting");
        telemetry.update();
    }

    @Override
    public void loop() {
        now = System.nanoTime(); // ts call here ideally should be the only call

        // WARNING: ts could ESPECIALLY not be very fun
        // especially in auto, bulk reading could result in poorer quality data
        for (LynxModule hub : hubs) hub.clearBulkCache();

        deviceCamera.update();
        follower.update();
        this.runtime.run();
        deviceExtake.update();
        deviceIntake.update();

//        telemetry.addData("x", DevicePinpoint.pinpoint.getPosX(DistanceUnit.INCH));
//        telemetry.addData("y", DevicePinpoint.pinpoint.getPosY(DistanceUnit.INCH));
//        telemetry.addData("h", DevicePinpoint.pinpoint.getHeading(AngleUnit.DEGREES));

        telemetry.addData("p", follower.getPose());

        if (DeviceCamera.goalTagDetection != null) telemetry.addData("tag_goal", String.format("r=%f, b=%f, e=%f, y=%f", DeviceCamera.goalTagDetection.ftcPose.range, DeviceCamera.goalTagDetection.ftcPose.bearing, DeviceCamera.goalTagDetection.ftcPose.elevation, DeviceCamera.goalTagDetection.ftcPose.yaw));
        telemetry.addData("field_offset", deviceCamera.getFieldOffset());

        if (DeviceCamera.goalTagDetection != null) t.addData("r", DeviceCamera.goalTagDetection.ftcPose.range);
        if (DeviceCamera.goalTagDetection != null) t.addData("b", DeviceCamera.goalTagDetection.ftcPose.bearing + (alliance == Alliance.BLUE ? Configuration.DRIVE_AIM_OFFSET : -Configuration.DRIVE_AIM_OFFSET) + (DeviceIntake.targetSide == DeviceIntake.IntakeSide.LEFT ? -Configuration.DRIVE_AIM_INTAKE_OFFSET : Configuration.DRIVE_AIM_INTAKE_OFFSET));

        t.addData("ext_vel", deviceExtake.targetVelocity);
        if (deviceExtake.motorExtakeLeft != null) t.addData("exl_vel", deviceExtake.motorExtakeLeft.getVelocity());
        if (deviceExtake.motorExtakeRight != null) t.addData("exr_vel", deviceExtake.motorExtakeRight.getVelocity());
        t.addData("int_srv" , deviceIntake.statusTelem); // intake servo status, displayed for timing purposes

        telemetry.addData("cl_a", deviceIntake.leftArtifact);
        telemetry.addData("cr_a", deviceIntake.rightArtifact);
        telemetry.addData("int_s", deviceIntake.targetSide);

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
        deviceExtake.stop();
        deviceIntake.stop();

        telemetry.addData("status", "stopping");
        telemetry.update();

        follower.setHeading(follower.getHeading() + Math.toRadians(-90) + alliance.apply(Math.toRadians(-90)));
    }

    private Pose P(double x, double y, double h) {
        return new Pose(alliance == Alliance.BLUE ? x : 144 - x, y, Math.toRadians(90 + alliance.apply(h)));
    }
}
