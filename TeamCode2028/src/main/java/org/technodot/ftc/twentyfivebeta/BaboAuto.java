package org.technodot.ftc.twentyfivebeta;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.paths.curves.bezier.BezierCurve;
import com.pedropathing.paths.curves.Line;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.interpolator.Interpolator;
import static com.pedropathing.api.Paths.path;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.technodot.ftc.twentyfivebeta.batch.Batch;
import org.technodot.ftc.twentyfivebeta.batch.Callback;
import org.technodot.ftc.twentyfivebeta.batch.ContiguousSequence;
import org.technodot.ftc.twentyfivebeta.batch.InterruptibleCallback;
import org.technodot.ftc.twentyfivebeta.common.Alliance;
import org.technodot.ftc.twentyfivebeta.common.Drawing;
import org.technodot.ftc.twentyfivebeta.pedro.Follower;
import org.technodot.ftc.twentyfivebeta.robocore.Device;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceCamera;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceExtake;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceIntake;
import org.technodot.ftc.twentyfivebeta.roboctrl.InputController;
import org.technodot.ftc.twentyfivebeta.roboctrl.SilentRunner101;

import java.util.List;

@Disabled
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

    public final long HALF = 500L;
    public final long ONE = 1000L;
    public final long TWO = 2000L;
    public final long THREE = 3000L;
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
                Pose close_start = P(0 + (20.323), 144 - (24.236), Math.toDegrees(Math.atan(4.0 / 3.0)));
                Pose close_shootPreload = P(48, 98);

                follower.setPose(close_start);

                Path close_start_shootPreload = path(
                        new Line(
                            close_start,
                            close_shootPreload
                        )
                    )
                    .heading(
                        Interpolator.facingPoint(
//                            P(14.4, 144 - 12.8)
                            P(0, 132)
                        )
                    );

                Path close_shootPreload_bezierSecond = path(
                        new BezierCurve( // this bezier curve was designed for red goal, so it has `144 - x` on each point
//                                P(144 - 96.000, 96.000),
//                                P(144 - 98.602, 70.221),
//                                P(144 - 120.848, 63.196),
//                                P(144 - 126.976, 49.677),
//                                P(144 - 134.365, 53.560)

//                                P(144 - 96.000, 96.000),
//                                P(144 - 90.000, 73.221),
//                                P(144 - 120.848, 63.196),
//                                P(144 - 125.215, 48.434),
////                                P(144 - 131.364, 56.076),
////                                P(144 - 134.573, 56.253)
////                                P(144 - 131.364, 56.076)
//                                P(Configuration.LOCALIZER_LENGTH_FRONT_OFFSET, 56.076),
//                                P(Configuration.LOCALIZER_LENGTH_FRONT_OFFSET, 56.076)

//                                    close_shootPreload,
//                                    P(144 - 90.000, 73.221),
//                                    P(144 - 120.848, 63.196),
//                                    P(144 - 126.355, 54.252),
//                                    P(144 - 128.396, 41.935),
//                                    P(144 - 132.729, 40.514)
//                                    P(144 - 128.396, 40.514)

//                                new Pose(96.000, 98.000),
//                                new Pose(90.000, 73.221),
//                                new Pose(120.848, 63.196),
//                                new Pose(125.112, 54.148),
//                                new Pose(130.571, 50.845),
//                                new Pose(124.524, 40.921),
//                                new Pose(133.226, 41.853)

                            close_shootPreload,
                            P(144 - 101.681, 63.168),
                            P(144 - 120.123, 60.503),
                            P(144 - 125.112, 54.148),
                            P(144 - 132.158, 54.114)
                        )
                    )
                    .heading(
                        Interpolator.tangent
                    );

//                Path close_bezierSecond_shootPreload = follower.pathBuilder()
//                    .addPath(
//                        new BezierCurve(
//                            P(144 - 132.729, 40.514),
//                            P(144 - 96.000, 67.000),
//                            P(144 - 96.000, 98.000)
//                        )
//                    )
//                    .linear(Math.toRadians(-15), Math.toRadians(45))
//;
//
//                Path close_shootPreload_cycleGate = follower.pathBuilder().addPath(
//                        new BezierCurve(
//                            P(144 - 96.000, 98.000),
//                            P(144 - 96.000, 64.000),
//                            P(144 - 129.496, 58.811)
////                            P(144 - 120.496, 58.811) // DOES NOT OPEN GATE YET
//                        )
//
//                    ).linear(Math.toRadians(45), Math.toRadians(18))
//                    .build();

                Path close_bezierSecond_assistGate = path(
                        new BezierCurve(
                            P(144 - 132.158, 56.272),
                            P(144 - 122.383, 54.260),
                            P(144 - 122.045, 65.405),
                            P(144 - 136.158, 64.552)

                        )
                    )
                    .linear(A(72.5), A(105));

                Path close_assistGate_shootGate = path(
                        new BezierCurve(
                            P(144 - 136.158, 64.552),
                            P(144 - 99.807, 65.214),
                            P(144 - 86.000, 86.000)
                        )
                    )
                    .heading(
                        Interpolator.piecewise()
                            .until(0.2, Interpolator.constant(A(105)))
                            .until(1, Interpolator.facingPoint(P(0, 132)))
                    );

                // the servos can go fuck themselves

//                Path close_shootGate_openGate = follower.pathBuilder()
//                    .addPath(
//                        new BezierCurve(
//                            P(144 - 86.000, 90.000),
//                            P(144 - 96.000, 70.000),
//                            P(144 - 129.496, 58.811)
//                        )
//                    )
//                    .linear(A(45), A(72))
//                    .build();
//
//                Path close_openGate_intakeGate = follower.pathBuilder()
//                    .addPath(
//                        new BezierCurve(
//                            P(144 - 129.496, 58.811),
//                            P(144 - 126.751, 55.301),
//                            P(144 - 136.806, 56.301)
//                        )
//                    )
//                    .linear(A(72), A(83.3))
//                    .build();
//
//                Path close_intakeGate_shootGate = follower.pathBuilder()
//                    .addPath(
//                        new BezierCurve(
//                            P(144 - 136.806, 56.301),
//                            P(144 - 99.807, 65.214),
//                            P(144 - 86.000, 90.000)
//                        )
//                    )
//                    .heading(
//                        Interpolator.facingPoint(
//                            P(0, 132)
//                        )
//                    )
//                    .addParametricCallback(0.4, new Runnable() {
//                        @Override
//                        public void run() {
//                            deviceIntake.setIntakeIdle();
//                        }
//                    })
//                    .build();
//
//                Path close_shootGate_finishPos = follower.pathBuilder().addPath(
//                        new Line(
//                            P(144 - 86.000, 90.000),
//                            P(144 - 96.000, 72.000)
//                        )
//                    )
//                    .linear(A(45), A(60))
//                    .build();

                Path close_shootGate_bezierFirst = path(
                        new BezierCurve(
                            P(144 - 86.000, 90.000),
                            P(144 - 109.959, 70.828),
                            P(144 - 121.234, 88.648),
                            P(144 - 128.000, 88.500)
                        )
                    )
                    .tangent();

                Path close_bezierFirst_shootGate = path(
                        new Line(
                            P(144 - 128.000, 88.500),
                            P(144 - 96, 98.000)
                        )
                    )
                    .heading(
                        Interpolator.facingPoint(
                            P(0, 132)
                        )
                    );

                Path close_shootGate_finishPos = path(
                        new Line(
                            P(144 - 96, 98.000),
                            P(48, 72)
                        )
                    )
                    .linear(
                        A(45),
                        A(60)
                    );

                runtime.plan(
                    new ContiguousSequence(0)
                        .then((Callback) () -> follower.followPath(close_start_shootPreload))
                        .then((Callback) () -> deviceExtake.setExtakeState(DeviceExtake.ExtakeState.DUAL_SHORT))
                        .then(TWO, (InterruptibleCallback) () -> follower.isReady())

                        .then(THREE, (InterruptibleCallback) () -> deviceExtake.isReady())
                        .delay(HALF)
                        .then((Callback) () -> deviceIntake.triggerSequenceShoot())
                        .then(TWO, (InterruptibleCallback) () -> deviceIntake.isEmpty())
                        .delay(HALF)
                        .then(TWO, (InterruptibleCallback) () -> !deviceIntake.isEmpty())
                        .then((Callback) () -> deviceIntake.triggerSequenceShoot())
                        .then(TWO, (InterruptibleCallback) () -> deviceIntake.isEmpty())

                        // tucker is the goat
                        .then((Callback) () -> follower.followPath(close_shootPreload_bezierSecond))
//                        .then((Callback) () -> deviceExtake.setExtakeState(DeviceExtake.ExtakeState.OVERRIDE))
//                        .then((Callback) () -> deviceExtake.setExtakeOverride(670))
                        .then((Callback) () -> deviceExtake.setExtakeState(DeviceExtake.ExtakeState.IDLE))
                        .then((Callback) () -> deviceIntake.setIntakeIn())
                        .then(THREE, (InterruptibleCallback) () -> follower.isReady())

                        .then((Callback) () -> {
                            follower.followPath(close_bezierSecond_assistGate);
                            follower.onParametric(0.2, () -> {
                                deviceIntake.setIntakeIdle();
                                deviceExtake.setExtakeState(DeviceExtake.ExtakeState.DUAL_SHORT);
                            });
                        }) // callback: set intake idle and starts extake @ t=0.2
                        .then(ONE, (InterruptibleCallback) () -> follower.isReady())
                        .delay(ONE)
                        .then((Callback) () -> follower.followPath(close_assistGate_shootGate))
                        .then(TWO, (InterruptibleCallback) () -> follower.isReady())

                        .then(THREE, (InterruptibleCallback) () -> deviceExtake.isReady())
                        .delay(TWO)
                        .then((Callback) () -> deviceIntake.triggerSequenceShoot())
                        .then(TWO, (InterruptibleCallback) () -> deviceIntake.isEmpty())
                        .delay(HALF)
                        .then((Callback) () -> deviceIntake.triggerSequenceShoot())
                        .then(TWO, (InterruptibleCallback) () -> deviceIntake.isEmpty())

                        // TODO: go cycle gate!!!

                        // fuckass me literally forgot that we needed to open the gate after second row
//                        .then((Callback) () -> follower.followPath(close_bezierSecond_shootPreload))
//                        .then((Callback) () -> deviceExtake.setExtakeState(DeviceExtake.ExtakeState.DUAL_SHORT))
////                        .delay(ONE)
////                        .then((Callback) () -> deviceIntake.setIntakeIdle())
//                        .then(THREE, (InterruptibleCallback) () -> follower.isReady())
//
////                        .delay(TWO)
//                        .then(THREE, (InterruptibleCallback) () -> deviceExtake.isReady())
//                        .delay(ONE)
//                        .then((Callback) () -> deviceIntake.triggerSequenceShoot())
//                        .then(ONE, (InterruptibleCallback) () -> deviceIntake.isEmpty())
//                        .delay(HALF)
//                        .then((Callback) () -> deviceIntake.triggerSequenceShoot())
//                        .then(ONE, (InterruptibleCallback) () -> deviceIntake.isEmpty())
//
//                        .then((Callback) () -> follower.followPath(close_shootPreload_cycleGate))
////                        .then((Callback) () -> deviceIntake.setIntakeIn())
//                        .then(TWO, (InterruptibleCallback) () -> follower.isReady())

                    // i can go fuck myself

//                        .then((Callback) () -> follower.followPath(close_shootGate_openGate))
//                        .then((Callback) () -> deviceExtake.setExtakeState(DeviceExtake.ExtakeState.OVERRIDE))
//                        .then((Callback) () -> deviceExtake.setExtakeOverride(670))
//                        .then(TWO, (InterruptibleCallback) () -> follower.isReady())
//                        .delay(ONE)
//                        .then((Callback) () -> follower.followPath(close_openGate_intakeGate))
//                        .then((Callback) () -> deviceIntake.setIntakeIn())
//                        .then(ONE, (InterruptibleCallback) () -> follower.isReady())
//
//                        .then((Callback) () -> follower.followPath(close_intakeGate_shootGate)) // callback: set intake idle @ t=0.3
////                        .then((Callback) () -> deviceExtake.setExtakeState(deviceIntake.getArtifactCount() == 2 ? DeviceExtake.ExtakeState.DUAL_SHORT : DeviceExtake.ExtakeState.DYNAMIC))
//                        .then((Callback) () -> deviceExtake.setExtakeState(DeviceExtake.ExtakeState.DUAL_SHORT))
//                        .then(THREE, (InterruptibleCallback) () -> follower.isReady())
//
//                        .then(THREE, (InterruptibleCallback) () -> deviceExtake.isReady())
//                        .delay(ONE)
//                        .then((Callback) () -> deviceIntake.triggerSequenceShoot())
//                        .then(ONE, (InterruptibleCallback) () -> deviceIntake.isEmpty())
//                        .delay(HALF)
//                        .then((Callback) () -> deviceIntake.triggerSequenceShoot())
//                        .then(ONE, (InterruptibleCallback) () -> deviceIntake.isEmpty())
//
//                        .then((Callback) () -> follower.followPath(close_shootGate_finishPos))
//                        .then((Callback) () -> deviceExtake.setExtakeState(DeviceExtake.ExtakeState.IDLE))
//                        .then(ONE, (InterruptibleCallback) () -> follower.isReady())

                        .then((Callback) () -> follower.followPath(close_shootGate_bezierFirst, 0.5, true))
                        .then((Callback) () -> deviceIntake.setIntakeIn())
                        .then((Callback) () -> deviceExtake.setExtakeState(DeviceExtake.ExtakeState.IDLE))
                        .then(5000L, (InterruptibleCallback) () -> follower.isReady())

                        .then((Callback) () -> follower.followPath(close_bezierFirst_shootGate))
                        .then((Callback) () -> deviceIntake.setIntakeIdle())
                        .then((Callback) () -> deviceExtake.setExtakeState(DeviceExtake.ExtakeState.DUAL_SHORT))
                        .then(TWO, (InterruptibleCallback) () -> follower.isReady())

                        .then(THREE, (InterruptibleCallback) () -> deviceExtake.isReady())
                        .delay(ONE)
                        .then((Callback) () -> deviceIntake.triggerSequenceShoot())
                        .then(TWO, (InterruptibleCallback) () -> deviceIntake.isEmpty())
                        .delay(HALF)
                        .then((Callback) () -> deviceIntake.triggerSequenceShoot())
                        .then(TWO, (InterruptibleCallback) () -> deviceIntake.isEmpty())

                        .then((Callback) () -> follower.followPath(close_shootGate_finishPos))
                        .then((Callback) () -> deviceExtake.setExtakeState(DeviceExtake.ExtakeState.IDLE))
                        .then(TWO, (InterruptibleCallback) () -> follower.isReady())
                    );

                break;
            case FAR:
                Pose far_start = P(48 + Configuration.LOCALIZER_WIDTH_LEFT_OFFSET, 0 + Configuration.LOCALIZER_LENGTH_BACK_OFFSET, 0);
                Pose far_shootPreload = P(48 + Configuration.LOCALIZER_WIDTH_LEFT_OFFSET, 6 + Configuration.LOCALIZER_LENGTH_BACK_OFFSET);

                follower.setPose(far_start);

                Path far_start_shootPreload = path(
                        new Line(far_start, far_shootPreload)
                    )
                    .heading(
                        Interpolator.facingPoint(
                            P(0, 140)
                        )
                    );

//                Path far_shootPreload_finishPos = follower.pathBuilder()
//                    .addPath(
//                        new Line(
//                            far_shootPreload,
//                            P(48, 60)
//                        )
//                    )
//                    .linear(
//                        A(20), A(90)
//                    )
//                    .build();

                Path far_shootPreload_bezierThird = path(
                        new BezierCurve(
                            P(144 - 88.000, 15.000),
                            P(144 - 95.552, 69.614),
                            P(144 - 104.793, 36.966),
                            P(144 - 127.021, 29.172),
                            P(144 - 133.393, 33.248)
                        )
                    )
                    .tangent();

                runtime.plan(
                    new ContiguousSequence(0)
                        .then((Callback) () -> follower.followPath(far_start_shootPreload))
                        .then((Callback) () -> deviceExtake.setExtakeState(DeviceExtake.ExtakeState.DYNAMIC))
                        .then(ONE, (InterruptibleCallback) () -> follower.isReady())

                        .then(THREE, (InterruptibleCallback) () -> deviceExtake.isReady())
                        .delay(ONE)
                        .then((Callback) () -> deviceIntake.triggerSequenceShoot())
                        .then(TWO, (InterruptibleCallback) () -> deviceIntake.isEmpty())
                        .delay(HALF)
                        .then((Callback) () -> deviceIntake.triggerSequenceShoot())
                        .then(TWO, (InterruptibleCallback) () -> deviceIntake.isEmpty())

//                        .then((Callback) () -> follower.followPath(far_shootPreload_finishPos))
//                        .then((Callback) () -> deviceExtake.setExtakeState(DeviceExtake.ExtakeState.IDLE))
//                        .then(THREE, (InterruptibleCallback) () -> follower.isReady())

                        .then((Callback) () -> follower.followPath(far_shootPreload_bezierThird, 0.67, true))
                        .then((Callback) () -> deviceExtake.setExtakeState(DeviceExtake.ExtakeState.IDLE))
                        .then((Callback) () -> deviceIntake.setIntakeIn())
                        .then(6767L, (InterruptibleCallback) () -> follower.isReady())
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

        Drawing.init();
    }

    @Override
    public void init_loop() {
        telemetry.addData("status", "initialized");
        telemetry.update();

        Drawing.draw(follower);
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

        Drawing.draw(follower);

//        telemetry.addData("x", DevicePinpoint.pinpoint.getPosX(DistanceUnit.INCH));
//        telemetry.addData("y", DevicePinpoint.pinpoint.getPosY(DistanceUnit.INCH));
//        telemetry.addData("h", DevicePinpoint.pinpoint.getHeading(AngleUnit.DEGREES));

        telemetry.addData("p", follower.pose());

        if (DeviceCamera.goalTagDetection != null) telemetry.addData("tag_goal", String.format("r=%f, b=%f, e=%f, y=%f", DeviceCamera.goalTagDetection.ftcPose.range, DeviceCamera.goalTagDetection.ftcPose.bearing, DeviceCamera.goalTagDetection.ftcPose.elevation, DeviceCamera.goalTagDetection.ftcPose.yaw));
//        telemetry.addData("field_offset", deviceCamera.getFieldOffset());

//        if (DeviceCamera.goalTagDetection != null) t.addData("r", DeviceCamera.goalTagDetection.ftcPose.range);
//        if (DeviceCamera.goalTagDetection != null) t.addData("b", DeviceCamera.goalTagDetection.ftcPose.bearing + (alliance == Alliance.BLUE ? Configuration.DRIVE_AIM_OFFSET : -Configuration.DRIVE_AIM_OFFSET) + (DeviceIntake.targetSide == DeviceIntake.IntakeSide.LEFT ? -Configuration.DRIVE_AIM_INTAKE_OFFSET : Configuration.DRIVE_AIM_INTAKE_OFFSET));

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
        follower.stop();

        deviceCamera.stop();
        deviceExtake.stop();
        deviceIntake.stop();

        telemetry.addData("status", "stopping");
        telemetry.update();

        follower.setHeading(follower.pose().heading() + Math.toRadians(-90) + alliance.apply(Math.toRadians(-90)));
    }

    private Pose P(double x, double y, double h) {
        return new Pose(alliance == Alliance.BLUE ? x : 144 - x, y, Math.toRadians(90 + alliance.apply(h)));
    }

    private Pose P(double x, double y) {
        return P(x, y, 0);
    }

    private Pose R(double x, double y, double h) {
        double d = Configuration.LOCALIZER_LENGTH_BACK_OFFSET - Configuration.ROBOT_LENGTH / 2.0;
        double theta = Math.toRadians(h);
        return P(x - d * Math.sin(theta), y + d * Math.cos(theta), h);
    }

    private double A(double deg) {
        return Math.toRadians(90 + alliance.apply(deg));
    }
}
