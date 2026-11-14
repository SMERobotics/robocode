package org.technodot.ftc.twentysix;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.technodot.ftc.twentysix.common.Alliance;
import org.technodot.ftc.twentysix.common.FieldCoordinates;
import org.technodot.ftc.twentysix.robocore.DeviceExtake;
import org.technodot.ftc.twentysix.robocore.DeviceIntake;
import org.technodot.ftc.twentysix.robocore.DeviceLimelight;
import org.technodot.ftc.twentysix.roboctrl.AutoSequence;
import org.technodot.ftc.twentysix.roboctrl.GamepadBindings;
import org.technodot.ftc.twentysix.roboctrl.HiveObservation;

import java.util.List;

import static com.pedropathing.api.Paths.line;

/** Shared autonomous implementation; the targets package registers the two alliances. */
public abstract class BaboAuto extends OpMode {
    public Alliance alliance = Alliance.BLUE;
    private List<LynxModule> hubs;
    private Follower follower;
    private DeviceIntake intake;
    private DeviceExtake extake;
    private DeviceLimelight limelight;
    private AutoBindings controls;
    private AutoSequence runtime;
    private Pose startPose;
    private Pose shootingPose;
    private HiveObservation observation;
    private int shotsFed;
    private long readySince;
    private long startedNanos;
    private boolean mechanismsStopped;
    private String status;
    private String readinessReason = "not checked";
    private double checkedShooterVelocity;
    private double checkedShooterTarget;
    private double checkedTurretError;
    private double checkedPositionError;
    private double checkedDriveSpeed;
    private String checkedVision = "not checked";

    protected abstract void config();

    @Override
    public void init() {
        config();
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        // Pedro owns the drivetrain and Pinpoint during AUTO; TELEOP reads the same field pose without resetting it.
        follower = Configuration.create(hardwareMap);
        startPose = FieldCoordinates.startPose(alliance, Configuration.AUTO_CENTER_FROM_ALLIANCE_WALL,
                Configuration.AUTO_CENTER_FROM_STARTING_WALL);
        shootingPose = FieldCoordinates.moveRight(startPose, Configuration.AUTO_MOVE_RIGHT_INCHES);
        follower.setPose(startPose);
        follower.drivetrain.stop();

        controls = new AutoBindings();
        intake = new DeviceIntake(alliance);
        extake = new DeviceExtake(alliance);
        limelight = new DeviceLimelight(alliance);
        intake.init(hardwareMap, controls, telemetry);
        intake.transferPos = DeviceIntake.TRANSFER_CLOSED_POS;
        intake.servoTransfer.setPosition(intake.transferPos);
        extake.init(hardwareMap, controls, telemetry);
        limelight.init(hardwareMap, controls, telemetry);
        configureSequence();
        status = "initialized";
        showTelemetry();
    }

    private void configureSequence() {
        Path moveRight = line(startPose, shootingPose).constant(startPose.heading());
        runtime = new AutoSequence()
                .then("move right and spin up", () -> {
                    extake.extakeTargetVelocity = Configuration.AUTO_SHOOT_VELOCITY;
                    extake.servoExtake.setPosition(DeviceExtake.SERVO_POLLEN_POS);
                    follower.follow(moveRight);
                })
                .await("reach shooting position", Configuration.AUTO_MOVE_TIMEOUT_MS, this::atShootingPose)
                .then("hold shooting position", () -> follower.hold(shootingPose));

        for (int shot = 1; shot <= 4; shot++) {
            final int shotNumber = shot;
            runtime.then("prepare pollen " + shot, () -> readySince = 0)
                    .await("aim and recover speed for pollen " + shot,
                            Configuration.AUTO_READY_TIMEOUT_MS, this::readyToShoot)
                    .then("open transfer for pollen " + shot, () -> controls.shooting = true)
                    .await("servo clearance for pollen " + shot, 1000, intake::isShootingIntakeActive)
                    .delay("feed pollen " + shot, Configuration.AUTO_FEED_MS)
                    .then("close transfer after pollen " + shot, () -> {
                        controls.shooting = false;
                        shotsFed = shotNumber;
                    })
                    .delay("separate pollen " + shot, Configuration.AUTO_BETWEEN_SHOTS_MS);
        }
    }

    @Override
    public void init_loop() {
        clearBulkCaches();
        follower.localizer.update();
        limelight.update();
        showTelemetry();
    }

    @Override
    public void start() {
        clearBulkCaches();
        follower.setPose(startPose);
        intake.start();
        extake.start();
        limelight.start();
        startedNanos = System.nanoTime();
        status = "running";
    }

    @Override
    public void loop() {
        clearBulkCaches();
        if (!mechanismsStopped) {
            // Stop before the transition period even if a mechanism never becomes ready.
            if (System.nanoTime() - startedNanos >= 29_000_000_000L) runtime.cancel();
            if (!runtime.isFinished()) {
                follower.update();
                limelight.update();
                observation = limelight.getHiveObservation(alliance);
                extake.setHiveObservation(observation, limelight.getLastHiveTagSeenNanos(alliance));
                extake.update();
                runtime.update(System.nanoTime());
            }
            if (runtime.isFinished()) {
                stopMechanisms();
            } else {
                intake.update();
            }
        } else {
            follower.localizer.update();
        }
        status = runtime.getStatus();
        showTelemetry();
    }

    private boolean atShootingPose() {
        Pose pose = follower.pose();
        double headingError = Math.atan2(Math.sin(pose.heading() - shootingPose.heading()),
                Math.cos(pose.heading() - shootingPose.heading()));
        return pose.distance(shootingPose) <= Configuration.AUTO_POSITION_TOLERANCE_INCHES
                && Math.abs(headingError) <= Math.toRadians(Configuration.AUTO_HEADING_TOLERANCE_DEGREES)
                && follower.velocity().linSpeed() <= 2;
    }

    private boolean readyToShoot() {
        long now = System.nanoTime();
        checkedTurretError = Math.abs(extake.motorTurret.getTargetPosition()
                - extake.motorTurret.getCurrentPosition()) * 360.0 / DeviceExtake.TURRET_REVOLUTION_TICKS;
        checkedShooterVelocity = extake.motorExtake.getVelocity();
        checkedShooterTarget = extake.extakeTargetVelocity;
        checkedPositionError = follower.pose().distance(shootingPose);
        checkedDriveSpeed = follower.velocity().linSpeed();
        checkedVision = observation == null ? "no observation" : !observation.valid ? observation.reason
                : observation.imageAgeMillis(now) > HiveVisionConfig.solver.maxAgeMillis ? "stale observation"
                : String.valueOf(observation.state);
        StringBuilder blockers = new StringBuilder();
        // Arrival is already a completed sequence step; the follower holds that pose while shooting.
        // Do not reset shot readiness for small odometry/velocity fluctuations during the hold.
        if (Configuration.AUTO_REQUIRE_HIVE_VISION && (observation == null || !observation.valid
                || observation.imageAgeMillis(now) > HiveVisionConfig.solver.maxAgeMillis)) {
            blockers.append("vision: ").append(checkedVision).append("; ");
        }
        if (!(checkedShooterTarget > 0) || !(Math.abs(checkedShooterVelocity - checkedShooterTarget)
                <= Configuration.AUTO_VELOCITY_TOLERANCE)) blockers.append("shooter speed; ");
        if (!(checkedTurretError <= Configuration.AUTO_TURRET_TOLERANCE_DEGREES)) blockers.append("turret alignment; ");
        if (blockers.length() > 0) {
            readinessReason = blockers.toString();
            readySince = 0;
            return false;
        }
        if (readySince == 0) readySince = now;
        boolean settled = now - readySince >= Configuration.AUTO_READY_SETTLE_MS * 1_000_000L;
        readinessReason = settled ? "ready" : "settling";
        return settled;
    }

    @Override
    public void stop() {
        if (runtime != null) runtime.cancel();
        stopMechanisms();
    }

    private void stopMechanisms() {
        if (mechanismsStopped) return;
        if (follower != null) {
            follower.stop();
            follower.drivetrain.stop();
        }
        if (intake != null && intake.servoTransfer != null) {
            controls.shooting = false;
            intake.transferPos = DeviceIntake.TRANSFER_CLOSED_POS;
            intake.servoTransfer.setPosition(intake.transferPos);
            intake.stop();
        }
        if (extake != null && extake.motorTurret != null) extake.stop();
        if (limelight != null) limelight.stop();
        mechanismsStopped = true;
    }

    private void clearBulkCaches() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
    }

    private void showTelemetry() {
        telemetry.addData("status", status);
        telemetry.addData("alliance", alliance);
        telemetry.addData("start pose (in, rad)", startPose);
        telemetry.addData("shooting pose (in, rad)", shootingPose);
        telemetry.addData("pose (in, rad)", follower.pose());
        telemetry.addData("pollen feed pulses", shotsFed);
        // Preserve the last readiness measurements after stopping the motors on timeout.
        telemetry.addData("shot blocked by", readinessReason);
        telemetry.addData("shot speed / target", "%.0f / %.0f ticks/s", checkedShooterVelocity, checkedShooterTarget);
        telemetry.addData("shot turret error (deg)", "%.1f", checkedTurretError);
        telemetry.addData("shot position error (in)", "%.2f", checkedPositionError);
        telemetry.addData("shot drive speed (in/s)", "%.2f", checkedDriveSpeed);
        telemetry.addData("shot hive", checkedVision);
        telemetry.update();
    }

    /** Autonomous never reads driver input; only the sequence can open the feeder. */
    private static final class AutoBindings extends GamepadBindings {
        boolean shooting;

        AutoBindings() { super(null, null); }

        @Override
        public boolean sequenceShoot() { return shooting; }
    }
}
