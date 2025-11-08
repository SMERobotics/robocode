package com.technodot.ftc.twentyfive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.technodot.ftc.twentyfive.batch.Batch;
import com.technodot.ftc.twentyfive.common.Obelisk;
import com.technodot.ftc.twentyfive.common.Team;
import com.technodot.ftc.twentyfive.common.Artifact;
import com.technodot.ftc.twentyfive.common.ArtifactInventory;
import com.technodot.ftc.twentyfive.robocore.DeviceCamera;
import com.technodot.ftc.twentyfive.robocore.DeviceDrive;
import com.technodot.ftc.twentyfive.robocore.DeviceExtake;
import com.technodot.ftc.twentyfive.robocore.DeviceIntake;

@Autonomous(name="BaboAuto", group="TechnoCode")
public class BaboAuto extends OpMode {

    public DeviceCamera deviceCamera = new DeviceCamera();
    public DeviceDrive deviceDrive = new DeviceDrive();
    public DeviceIntake deviceIntake = new DeviceIntake();
    public DeviceExtake deviceExtake = new DeviceExtake();

    public MultipleTelemetry t;

    public Batch runtime = new Batch();

    public Team team = Team.BLUE;
    public Obelisk obelisk = Obelisk.PPG; // default pattern if not seen

    // --- Autonomous shooting state ---
    private final Artifact[] targetSequence = new Artifact[3];
    private int shotsFired = 0;
    private boolean extakeLowEngaged = false;
    private boolean intakeRunning = false;
    private long lastShotNs = 0;
    private boolean lateralMoveDone = false;

    private boolean servoShotInProgress = false;
    private long servoShotEndNs = 0;
    private float lastAutoRotationalOffset = 0.0f;

    private static final long SHOT_COOLDOWN_MS = 2000; // spin-up time between shots
    private static final double EXTAKE_READY_FACTOR = 0.9; // 90% of target speed
    private ArtifactInventory.Side lastShotSide = ArtifactInventory.Side.NONE;

    public void config() { team = Team.BLUE; }

    @Override
    public void init() {
        config();

        t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        deviceCamera.init(hardwareMap, team);
        deviceDrive.init(hardwareMap);
        deviceIntake.init(hardwareMap, team);
        deviceExtake.init(hardwareMap);

        // Initial backwards move
        runtime.plan(0, (long startMs, long durationMs, long executionMs) -> {
            deviceDrive.applyMovement(-5.5f, 0.0f, 0.0f);
            return false;
        });

        // Aim / rotate sequence (example retained)
        runtime.plan(3000, (long startMs, long durationMs, long executionMs) -> {
            deviceDrive.applyMovement(0.0f, 0.0f, team.equals(Team.BLUE) ? 54.0f : -54.0f);
            return false;
        });
        runtime.plan(4000, (long startMs, long durationMs, long executionMs) -> {
            // Engage extake low spin at 4000ms per requirements
            if (!extakeLowEngaged) {
                deviceExtake.setState(DeviceExtake.ExtakeState.SHOOTING_LOW);
                extakeLowEngaged = true;
            }
            // counter-rotation
            deviceDrive.applyMovement(0.0f, 0.0f, team.equals(Team.BLUE) ? -54.0f : 54.0f);
            return false;
        });

        runtime.plan(5000, (long startMs, long durationMs, long executionMs) -> {
            // Read obelisk at 5000ms
            Obelisk detected = deviceCamera.getObelisk();
            if (detected != null) {
                obelisk = detected;
            }
            buildSequenceFromObelisk();
            return false;
        });

        runtime.plan(7000, (long startMs, long durationMs, long executionMs) -> {
            // Attempt first shot at 7000ms
            attemptShot();
            return false;
        });

        // Removed late-time plan that conflicted with lateral move logic
    }

    @Override
    public void init_loop() {
        t.addData("status", "initialized");
        t.update();
    }

    @Override
    public void start() {
        deviceCamera.start();
        deviceDrive.start();
        t.addData("status", "starting");
        t.update();
    }

    @Override
    public void loop() {
        // Execute timed batch plans
        runtime.run();

        // First update sensors and subsystems
        deviceIntake.update((Gamepad) null);
        deviceExtake.update(null);
        deviceCamera.update(null);

        long elapsedMs = runtime.running ? (runtime.currentNs - runtime.startNs) / 1_000_000L : 0L;

        // Retry first shot if not successful exactly at 7000ms and conditions satisfied
        if (extakeLowEngaged && shotsFired == 0 && !servoShotInProgress && elapsedMs >= 7000) {
            attemptShot();
        }

        // After first shot, ensure intake running
        if (shotsFired > 0 && !intakeRunning) {
            deviceIntake.setMotorPower(1.0); // start pulling balls in
            intakeRunning = true;
        }

        // Subsequent shots after cooldown
        if (extakeLowEngaged && shotsFired < 3 && shotsFired > 0 && !servoShotInProgress) {
            long sinceLastShotMs = (lastShotNs == 0) ? Long.MAX_VALUE : (System.nanoTime() - lastShotNs) / 1_000_000L;
            if (sinceLastShotMs >= SHOT_COOLDOWN_MS) {
                attemptShot();
            }
        }

        // Immediately recalc rotational offset this same loop if a shot was just scheduled
        deviceIntake.update((Gamepad) null);

        // Mirror TeleOp: apply heading nudge based on intake rotation window
        float ro = deviceIntake.getRotationalOffset();
        if (ro != 0.0f && ro != lastAutoRotationalOffset) {
            deviceDrive.resetMovement();
            deviceDrive.applyMovement(0.0f, 0.0f, ro);
            deviceDrive.flushMovement();
            lastAutoRotationalOffset = ro;
        } else if (ro == 0.0f) {
            lastAutoRotationalOffset = 0.0f;
        }

        // Manage ongoing servo shot pulses
        long nowNs = System.nanoTime();
        if (servoShotInProgress && nowNs >= servoShotEndNs) {
            // Open both servos after pulse
            deviceIntake.setServoOverride(true);
            deviceIntake.setServoPositions(0.3, 0.56); // default open positions
            servoShotInProgress = false;
        }

        // Lateral move after all shots
        if (shotsFired >= 3 && !lateralMoveDone) {
            deviceDrive.applyMovement(0.0f, team.equals(Team.BLUE) ? -1.0f : 1.0f, 0.0f);
            deviceDrive.flushMovement(); // immediate execution
            lateralMoveDone = true;
        }

        // Flush movement requests from any plans this loop
        deviceDrive.flushMovement();

        // Telemetry
        t.addData("elapsedMs", elapsedMs);
        t.addData("rotOffset", ro);
        t.addData("fl", deviceDrive.motorFrontLeft.getCurrentPosition());
        t.addData("fr", deviceDrive.motorFrontRight.getCurrentPosition());
        t.addData("bl", deviceDrive.motorBackLeft.getCurrentPosition());
        t.addData("br", deviceDrive.motorBackRight.getCurrentPosition());
        t.addData("obelisk", obelisk);
        t.addData("shotsFired", shotsFired);
        t.addData("targetSeq", targetSequenceToString());
        t.addData("inventoryL", deviceIntake.getInventory().getArtifact(ArtifactInventory.Side.LEFT));
        t.addData("inventoryR", deviceIntake.getInventory().getArtifact(ArtifactInventory.Side.RIGHT));
        t.addData("extakeState", deviceExtake.currentState);
        t.addData("intakeRunning", intakeRunning);
        long sinceLastShotMs = (lastShotNs == 0) ? -1L : (System.nanoTime() - lastShotNs) / 1_000_000L;
        t.addData("sinceLastShotMs", sinceLastShotMs);
        t.addData("status", "running");
        t.update();
    }

    private boolean isExtakeReady() {
        double target = Math.max(1.0, deviceExtake.getTargetVelocity());
        double vel = Math.max(0.0, deviceExtake.getMeasuredVelocity());
        return vel >= EXTAKE_READY_FACTOR * target;
    }

    private void attemptShot() {
        if (shotsFired >= 3) return;
        // Enforce cooldown for subsequent shots
        if (lastShotNs != 0) {
            long deltaMs = (System.nanoTime() - lastShotNs) / 1_000_000L;
            if (deltaMs < SHOT_COOLDOWN_MS) return;
        }
        // Ensure flywheel back up to speed
        if (!isExtakeReady()) return;

        Artifact needed = targetSequence[shotsFired];
        if (needed == null || needed == Artifact.NONE) return; // nothing to shoot

        ArtifactInventory inventory = deviceIntake.getInventory();
        Artifact leftArt = inventory.getArtifact(ArtifactInventory.Side.LEFT);
        Artifact rightArt = inventory.getArtifact(ArtifactInventory.Side.RIGHT);

        ArtifactInventory.Side side;
        if (leftArt == needed && rightArt == needed) {
            // Choose opposite side of last shot to avoid back-to-back on same side
            side = (lastShotSide == ArtifactInventory.Side.LEFT) ? ArtifactInventory.Side.RIGHT : ArtifactInventory.Side.LEFT;
        } else if (leftArt == needed) {
            side = ArtifactInventory.Side.LEFT;
        } else if (rightArt == needed) {
            side = ArtifactInventory.Side.RIGHT;
        } else {
            return; // required artifact not yet present
        }

        // Trigger the same timing windows as TeleOp to drive servos and rotationalOffset
        deviceIntake.triggerShot(side);
        servoShotInProgress = true;
        servoShotEndNs = System.nanoTime() + 400_000_000L; // 400ms pulse

        // Mark shot
        shotsFired++;
        lastShotSide = side;
        lastShotNs = System.nanoTime();
    }

    private void buildSequenceFromObelisk() {
        // Interpret enum letters order for required artifact sequence.
        switch (obelisk) {
            case GPP: targetSequence[0] = Artifact.GREEN; targetSequence[1] = Artifact.PURPLE; targetSequence[2] = Artifact.PURPLE; break;
            case PGP: targetSequence[0] = Artifact.PURPLE; targetSequence[1] = Artifact.GREEN; targetSequence[2] = Artifact.PURPLE; break;
            case PPG: targetSequence[0] = Artifact.PURPLE; targetSequence[1] = Artifact.PURPLE; targetSequence[2] = Artifact.GREEN; break;
        }
    }

    private String targetSequenceToString() {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < targetSequence.length; i++) {
            if (i > 0) sb.append("-");
            sb.append(targetSequence[i] == null ? "?" : targetSequence[i]);
        }
        return sb.toString();
    }
}
