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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

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

    // === Aiming constants (mirror DeviceDrive PID) ===
    private static final double AIM_KP = 0.013;
    private static final double AIM_KD = 0.003;
    private static final double AIM_MAX_OUTPUT = 0.6;
    private static final double AIM_TOLERANCE_DEG = 1.0; // within this bearing we consider aligned
    private static final long AIM_MAX_TIME_MS = 1500; // fallback timeout
    private static final long AIM_LOSS_TIMEOUT_MS = 600; // if tag not seen this long, fallback shoot
    private boolean shotWindowOpen = false; // opens at 7000ms

    private boolean aimingActive = false;
    private long aimStartNs = 0;
    private long lastTagSeenNs = 0;
    private double lastAimYawDeg = 0.0;
    private long lastAimSampleNs = 0;
    private int aimStableCount = 0; // consecutive cycles inside tolerance
    private AprilTagDetection currentTeamTag = null;

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
            shotWindowOpen = true; // allow first shot sequence to begin (aim then feed)
            return false;
        });
        // Removed direct attemptShot() call; aiming precedes feeding now.
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
        runtime.run();

        // Update subsystems & get AprilTag team detection
        deviceIntake.update((Gamepad) null);
        deviceExtake.update(null);
        currentTeamTag = deviceCamera.update(); // returns team tag detection if visible
        // Obelisk may also update internally in camera

        long elapsedMs = runtime.running ? (runtime.currentNs - runtime.startNs) / 1_000_000L : 0L;

        // Ensure intake motor after first shot
        if (shotsFired > 0 && !intakeRunning) {
            deviceIntake.setMotorPower(1.0);
            intakeRunning = true;
        }

        // Evaluate shot eligibility (without actually feeding) -> start aiming if needed
        if (shotWindowOpen && extakeLowEngaged && shotsFired < 3 && !servoShotInProgress) {
            long sinceLastShotMs = (lastShotNs == 0) ? Long.MAX_VALUE : (System.nanoTime() - lastShotNs) / 1_000_000L;
            boolean cooldownOk = (shotsFired == 0 && elapsedMs >= 7000) || sinceLastShotMs >= SHOT_COOLDOWN_MS;
            if (cooldownOk && isExtakeReady() && hasNextArtifactAvailable()) {
                // Start aiming if not already
                if (!aimingActive && deviceIntake.getRotationalOffset() == 0.0f) {
                    aimingActive = true;
                    aimStartNs = System.nanoTime();
                    lastAimSampleNs = aimStartNs;
                    aimStableCount = 0;
                }
            }
        }

        // Aiming state machine
        if (aimingActive) {
            long nowNs = System.nanoTime();
            long aimElapsedMs = (nowNs - aimStartNs) / 1_000_000L;

            double yawDeg;
            boolean tagVisible = (currentTeamTag != null && currentTeamTag.ftcPose != null);
            if (tagVisible) {
                // THE CAMERA IS UPSIDE DOWN (comment from drive), bearing negated
                yawDeg = -currentTeamTag.ftcPose.bearing;
                lastTagSeenNs = nowNs;
            } else {
                yawDeg = lastAimYawDeg; // continue with last measurement
            }

            // Derivative (yaw rate)
            double yawRate = 0.0;
            if (lastAimSampleNs != 0 && nowNs > lastAimSampleNs) {
                double dtSec = (nowNs - lastAimSampleNs) / 1e9;
                yawRate = (yawDeg - lastAimYawDeg) / dtSec;
            }
            lastAimYawDeg = yawDeg;
            lastAimSampleNs = nowNs;

            boolean withinTolerance = Math.abs(yawDeg) <= AIM_TOLERANCE_DEG;
            if (withinTolerance) {
                aimStableCount++;
            } else {
                aimStableCount = 0; // reset stability counter
            }

            boolean timeoutExceeded = aimElapsedMs >= AIM_MAX_TIME_MS;
            boolean tagLossExceeded = tagVisible ? false : (lastTagSeenNs != 0 && (nowNs - lastTagSeenNs) / 1_000_000L >= AIM_LOSS_TIMEOUT_MS);
            boolean stableAchieved = aimStableCount >= 2; // two consecutive in tolerance cycles

            if (stableAchieved || timeoutExceeded || tagLossExceeded) {
                aimingActive = false; // finish aiming phase
                // Only feed if artifact still available & extake ready & cooldown still ok
                if (hasNextArtifactAvailable() && isExtakeReady()) {
                    attemptShot();
                    // After scheduling shot, rotational offset will appear; re-run intake update for immediate offset
                    deviceIntake.update((Gamepad) null);
                }
            } else {
                // Apply rotation correction this loop (avoid interfering with servo rotational offsets)
                if (deviceIntake.getRotationalOffset() == 0.0f) {
                    double rotatePower = AIM_KP * yawDeg + AIM_KD * yawRate;
                    if (rotatePower > AIM_MAX_OUTPUT) rotatePower = AIM_MAX_OUTPUT;
                    if (rotatePower < -AIM_MAX_OUTPUT) rotatePower = -AIM_MAX_OUTPUT;
                    // Request rotation movement
                    deviceDrive.resetMovement();
                    deviceDrive.applyMovement(0.0f, 0.0f, (float) rotatePower);
                    deviceDrive.flushMovement();
                }
            }
        }

        // Immediately recalc rotational offset for any servo pulses (independent of aiming)
        deviceIntake.update((Gamepad) null);
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
        long nowNsPulse = System.nanoTime();
        if (servoShotInProgress && nowNsPulse >= servoShotEndNs) {
            deviceIntake.setServoOverride(true);
            deviceIntake.setServoPositions(0.3, 0.56);
            servoShotInProgress = false;
        }

        // Lateral move after all shots
        if (shotsFired >= 3 && !lateralMoveDone) {
            deviceDrive.applyMovement(0.0f, team.equals(Team.BLUE) ? -1.0f : 1.0f, 0.0f);
            deviceDrive.flushMovement();
            lateralMoveDone = true;
        }

        // Final flush for any remaining requests
        deviceDrive.flushMovement();

        // Telemetry additions
        t.addData("aimingActive", aimingActive);
        t.addData("aimYawDeg", lastAimYawDeg);
        t.addData("aimStableCount", aimStableCount);
        t.addData("shotWindowOpen", shotWindowOpen);
        // ...existing telemetry additions...
    }

    private boolean hasNextArtifactAvailable() {
        if (shotsFired >= 3) return false;
        Artifact needed = targetSequence[shotsFired];
        if (needed == null || needed == Artifact.NONE) return false;
        ArtifactInventory inv = deviceIntake.getInventory();
        return inv.getArtifact(ArtifactInventory.Side.LEFT) == needed || inv.getArtifact(ArtifactInventory.Side.RIGHT) == needed;
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
