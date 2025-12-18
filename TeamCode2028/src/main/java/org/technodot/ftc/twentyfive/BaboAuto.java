package org.technodot.ftc.twentyfive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.technodot.ftc.twentyfive.batch.Batch;
import org.technodot.ftc.twentyfive.common.Artifact;
import org.technodot.ftc.twentyfive.common.ArtifactInventory;
import org.technodot.ftc.twentyfive.common.Obelisk;
import org.technodot.ftc.twentyfive.common.Team;
import org.technodot.ftc.twentyfive.robocore.DeviceCamera;
import org.technodot.ftc.twentyfive.robocore.DeviceDrive;
import org.technodot.ftc.twentyfive.robocore.DeviceExtake;
import org.technodot.ftc.twentyfive.robocore.DeviceIntake;

import java.util.ArrayDeque;

@Disabled
@Autonomous(name="BaboAuto", group="TechnoCode")
public class BaboAuto extends OpMode {

    public DeviceCamera deviceCamera = new DeviceCamera();
    public DeviceDrive deviceDrive = new DeviceDrive();
    public DeviceIntake deviceIntake = new DeviceIntake();
    public DeviceExtake deviceExtake = new DeviceExtake();

    public MultipleTelemetry t;
    public Batch runtime = new Batch();
    public ArrayDeque<Artifact> artifactQuene = new ArrayDeque<>();

    private boolean nextShotLeft = false;

    public Team team = Team.BLUE;
    public Obelisk obelisk = Obelisk.PPG;

    @Override
    public void init() {
        config();

        t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        deviceCamera.init(hardwareMap, team);
        deviceIntake.init(hardwareMap);
        deviceExtake.init(hardwareMap);
        deviceDrive.init(hardwareMap, team);

        runtime.plan(0, (long startMs, long durationMs, long executionMs) -> {
            // applyMovement adds a request to the queue. It does not command the motors directly.
            deviceExtake.setVelocityOverride(1180);
            deviceExtake.setState(DeviceExtake.ExtakeState.SHOOTING_LOW);

//            deviceDrive.applyMovement(-3.8f, team.equals(Team.BLUE) ? -0.1f : 0.1f, 0.0f);
            deviceDrive.applyMovement(-3.8f, team.apply(-0.1f), team.apply(54.0f));
            return false;
        });

//        runtime.plan(3000, (long startMs, long durationMs, long executionMs) -> {
//            deviceDrive.applyMovement(0.0f, 0.0f, team.equals(Team.BLUE) ? 54.0f : -54.0f);
//            return false;
//        });

        runtime.plan(0, 2000, (long startMs, long durationMs, long executionMs) -> {
           deviceCamera.update();
           return false;
        });

        runtime.plan(2000, (long startMs, long durationMs, long executionMs) -> {
            deviceDrive.applyMovement(0.0f, 0.0f, team.apply(-54.0f));

            obelisk = deviceCamera.getObelisk();
            artifactQuene.clear();
            if (obelisk == null) return false; // we're cooked

            switch (obelisk) {
                case GPP:
                    artifactQuene.add(Artifact.GREEN);
                    artifactQuene.add(Artifact.PURPLE);
                    artifactQuene.add(Artifact.PURPLE);
                    break;
                case PGP:
                    artifactQuene.add(Artifact.PURPLE);
                    artifactQuene.add(Artifact.GREEN);
                    artifactQuene.add(Artifact.PURPLE);
                    break;
                case PPG:
                default:
                    artifactQuene.add(Artifact.PURPLE);
                    artifactQuene.add(Artifact.PURPLE);
                    artifactQuene.add(Artifact.GREEN);
                    break;
            }

            // TODO: ts obelisk solver doesnt work, fix
            if (obelisk.equals(Obelisk.GPP)) {
                nextShotLeft = true;
            }

            return false;
        });

        runtime.plan(3000, 6000, (long startMs, long durationMs, long executionMs) -> {
            deviceDrive.updatePose(deviceCamera.update(), executionMs * 1_000_000L);
            deviceDrive.updateAim();
            t.addData("obelisk", obelisk);
            return false;
        });

        runtime.plan(4000, (long startMs, long durationMs, long executionMs) -> shootNext());

//        runtime.plan(6500, 8000, (long startMs, long durationMs, long executionMs) -> {
//            deviceIntake.intake();
//            return false;
//        });

        runtime.plan(8000, (long startMs, long durationMs, long executionMs) -> shootNext());

//        runtime.plan(10500, 12000, (long startMs, long durationMs, long executionMs) -> {
//            deviceIntake.intake();
//            return false;
//        });

//        runtime.plan(14000, (long startMs, long durationMs, long executionMs) -> shootNext());

        // no more balls left so no need to intake
//        runtime.plan(14500, 16000, (long startMs, long durationMs, long executionMs) -> {
//            deviceIntake.intake();
//            return false;
//        });

        runtime.plan(9000, (long startMs, long durationMs, long executionMs) -> {
            deviceExtake.setState(DeviceExtake.ExtakeState.IDLE);
            deviceExtake.clearVelocityOverride();
            deviceDrive.applyMovement(-3.0f, team.apply(-2.0f), 0.0f);
            return false;
        });

        // done in 13000ms
    }

    @Override
    public void init_loop() {
        t.addData("status", "initialized");
        t.update();
    }

    @Override
    public void start() {
        deviceCamera.start();
        deviceIntake.start();
        deviceExtake.start();
        deviceDrive.start();

        t.addData("status", "starting");
        t.update();
    }

    @Override
    public void loop() {
        // This executes the actions planned in init(), which will call applyMovement().
        runtime.run();

        // This combines all movement requests from the last loop, and sends a single command to the motors.
        deviceDrive.flushMovement();
        deviceExtake.update();
        deviceIntake.update();

//        t.addData("fl", deviceDrive.motorFrontLeft.getCurrentPosition());
//        t.addData("fr", deviceDrive.motorFrontRight.getCurrentPosition());
//        t.addData("bl", deviceDrive.motorBackLeft.getCurrentPosition());
//        t.addData("br", deviceDrive.motorBackRight.getCurrentPosition());

        t.addData("exv", deviceExtake.motorExtake.getVelocity());

        t.addData("status", "running");
        t.update();
    }

    @Override
    public void stop() {
        deviceCamera.stop();
        deviceIntake.stop();
        deviceExtake.stop();
        deviceDrive.stop();

        runtime.reset();

        t.addData("status", "stopping");
        t.update();
    }

    public void config() {
        team = Team.BLUE;
    }

    private boolean shootNext() {
//        Artifact next = artifactQuene.pollFirst();
//        t.addData("shoot/queued", nextShotLeft);
//        if (next == null || next == Artifact.NONE) {
//            t.addData("shoot/status", "no artifact in queue");
//            return false; // nothing planned to shoot
//        }
//
        ArtifactInventory.Side sideToShoot = nextShotLeft
                ? ArtifactInventory.Side.LEFT
                : ArtifactInventory.Side.RIGHT;

        t.addData("shoot/queued", sideToShoot);

        deviceIntake.triggerShot(sideToShoot);

        // Flip for next call: LEFT then RIGHT then LEFT, etc.
        nextShotLeft = !nextShotLeft;

        return false;
    }
}
