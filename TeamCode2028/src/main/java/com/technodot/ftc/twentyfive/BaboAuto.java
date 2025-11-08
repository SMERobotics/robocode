package com.technodot.ftc.twentyfive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.technodot.ftc.twentyfive.batch.Batch;
import com.technodot.ftc.twentyfive.common.Team;
import com.technodot.ftc.twentyfive.robocore.DeviceCamera;
import com.technodot.ftc.twentyfive.robocore.DeviceDrive;
import com.technodot.ftc.twentyfive.robocore.DeviceExtake;
import com.technodot.ftc.twentyfive.robocore.DeviceIntake;
import com.technodot.ftc.twentyfive.common.Obelisk;
import com.technodot.ftc.twentyfive.common.Artifact;
import com.technodot.ftc.twentyfive.common.ArtifactInventory;

@Autonomous(name="BaboAuto", group="TechnoCode")
public class BaboAuto extends OpMode {

    public DeviceCamera deviceCamera = new DeviceCamera();
    public DeviceDrive deviceDrive = new DeviceDrive();
    public DeviceIntake deviceIntake = new DeviceIntake();
    public DeviceExtake deviceExtake = new DeviceExtake();

    public MultipleTelemetry t;

    public Batch runtime = new Batch();

    public Team team = Team.BLUE;

    // === Autonomous shoot-by-color state ===
    private boolean shootingActive = false;
    private long shootingStartMs = 0L;
    private Artifact[] shootOrder = null; // length 3 when known
    private int shootStep = 0; // 0..2

    private enum ShootPhase { WAIT_SPIN, FEEDING, SETTLING, REASSESS, DONE }
    private ShootPhase shootPhase = ShootPhase.DONE;
    private long phaseUntilMs = 0L;

    // Tunables (ms)
    private static final long SPINUP_GRACE_MS = 800;   // minimum time to let wheel spin before first feed
    private static final double SPIN_TOLERANCE = 150;  // ticks/s tolerance to consider at speed
    private static final long FEED_MS = 550;           // duration to push a ball
    private static final long SETTLE_MS = 250;         // pause between balls

    @Override
    public void init() {
        config();

        t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        deviceCamera.init(hardwareMap, team);
        deviceDrive.init(hardwareMap);
        deviceIntake.init(hardwareMap);
        deviceExtake.init(hardwareMap);

        runtime.plan(0, 30_000, (long startMs, long durationMs, long executionMs) -> {
            deviceCamera.detectTags(false);
            return false;
        });

        // Plan to move backward for the first second using the new movement system.
        runtime.plan(0, (long startMs, long durationMs, long executionMs) -> {
            // applyMovement adds a request to the queue. It does not command the motors directly.
            deviceDrive.applyMovement(-4.0f, 0.0f, 0.0f);
            return false; // Don't end the action early.
        });

        runtime.plan(3000, (long startMs, long durationMs, long executionMs) -> {
            deviceDrive.applyMovement(0.0f, 0.0f, 54.0f);
            return false;
        });

        runtime.plan(4000, (long startMs, long durationMs, long executionMs) -> {
            deviceDrive.applyMovement(0.0f, 0.0f, -54.0f);
            return false;
        });

        runtime.plan(5000, (long startMs, long durationMs, long executionMs) -> {
            deviceExtake.setState(DeviceExtake.ExtakeState.SHOOTING_LOW);
            return false;
        });

        // Get the obelisk state, and then shoot three balls by color. Do your best to follow the obelisk pattern; if it is impossible, that's okay.
        runtime.plan(8000, (long startMs, long durationMs, long executionMs) -> {
            // Begin shooting sequence; determine pattern ASAP (or pick a safe default after timeout in loop)
            startShootingSequence();
            return false;
        });

        runtime.plan(20_000, (long startMs, long durationMs, long executionMs) -> {
            // Stop shooter
            deviceExtake.setState(DeviceExtake.ExtakeState.IDLE);
            deviceDrive.applyMovement(0.0f, -2.0f, 0.0f);
            return false;
        });
    }

    @Override
    public void init_loop() {
        t.addData("status", "initialized");
        t.update();
    }

    @Override
    public void start() {
        deviceCamera.start();
        deviceDrive.start(); // Initialize encoders and motor modes.

        t.addData("status", "starting");
        t.update();
    }

    @Override
    public void loop() {
        // This executes the actions planned in init(), which will call applyMovement().
        runtime.run();

        // Keep subsystems updated for reliable control/sensing
        deviceExtake.update(null); // maintain shooter velocity PID
        deviceIntake.update((com.qualcomm.robotcore.hardware.Gamepad) null); // explicit cast avoids overload ambiguity

        // If a shooting sequence is active, run it
        if (shootingActive) {
            runShootingSequence();
        }

        // This combines all movement requests from the last loop, and sends a single command to the motors.
        deviceDrive.flushMovement();

        // Telemetry
        t.addData("fl", deviceDrive.motorFrontLeft.getCurrentPosition());
        t.addData("fr", deviceDrive.motorFrontRight.getCurrentPosition());
        t.addData("bl", deviceDrive.motorBackLeft.getCurrentPosition());
        t.addData("br", deviceDrive.motorBackRight.getCurrentPosition());
        t.addData("obelisk", deviceCamera.getObelisk());
        t.addData("shoot_active", shootingActive);
        t.addData("shoot_phase", shootPhase);
        t.addData("shoot_step", shootStep);
        t.addData("extake_v_meas", deviceExtake.getMeasuredVelocity());
        t.addData("extake_v_tgt", deviceExtake.getTargetVelocity());
        t.addData("inv_left", deviceIntake.getInventory().getArtifact(ArtifactInventory.Side.LEFT));
        t.addData("inv_right", deviceIntake.getInventory().getArtifact(ArtifactInventory.Side.RIGHT));

        t.addData("status", "running");
        t.update();
    }

    @Override
    public void stop() {
        runtime.reset();

        // Ensure mechanisms are safe
        deviceExtake.setState(DeviceExtake.ExtakeState.IDLE);
        try { deviceIntake.motorIntake.setPower(0.0); } catch (Exception ignored) {}
        // Block both lanes by default
        try { blockBoth(); } catch (Exception ignored) {}
        try { deviceIntake.setServoOverride(false); } catch (Exception ignored) {}

        deviceCamera.stop();
        deviceDrive.stop(); // Stops the drive motors.

        t.addData("status", "stopping");
        t.update();
    }

    public void config() {
        team = Team.BLUE;
    }

    // === Autonomous shooting helpers ===

    private void startShootingSequence() {
        shootingActive = true;
        shootingStartMs = System.currentTimeMillis();
        shootStep = 0;
        shootPhase = ShootPhase.WAIT_SPIN;
        phaseUntilMs = 0L;
        // Try to grab an initial obelisk reading; if null we'll retry in loop
        Obelisk o = deviceCamera.getObelisk();
        if (o != null) {
            shootOrder = orderFromObelisk(o);
        } else {
            shootOrder = null; // will resolve later or fall back
        }
        // Make sure shooter is in a shooting state
        deviceExtake.setState(DeviceExtake.ExtakeState.SHOOTING_LOW);
        deviceExtake.setVelocityOverride(1200); // manual override per request
        // Default: block both until feeding starts
        blockBoth();
    }

    private void runShootingSequence() {
        long now = System.currentTimeMillis();

        // Resolve obelisk -> order if still unknown; fall back after 2s
        if (shootOrder == null) {
            Obelisk o = deviceCamera.getObelisk();
            if (o != null) {
                shootOrder = orderFromObelisk(o);
            } else if (now - shootingStartMs > 2000) {
                // safe default if not seen
                shootOrder = new Artifact[]{Artifact.PURPLE, Artifact.PURPLE, Artifact.GREEN};
            }
        }

        switch (shootPhase) {
            case WAIT_SPIN: {
                // Require minimal time and velocity near target before feeding first ball
                double vTgt = deviceExtake.getTargetVelocity();
                double vMeas = deviceExtake.getMeasuredVelocity();
                boolean timeOk = (now - shootingStartMs) >= SPINUP_GRACE_MS;
                boolean speedOk = vTgt > 0 && (vMeas >= (vTgt - SPIN_TOLERANCE));
                if (timeOk && speedOk && shootOrder != null) {
                    // Proceed to first feed
                    shootPhase = ShootPhase.FEEDING;
                    phaseUntilMs = now + FEED_MS;
                    prepareFeedForStep();
                }
                break;
            }
            case FEEDING: {
                // Keep intake running forward while feeding
                try { deviceIntake.motorIntake.setPower(1.0); } catch (Exception ignored) {}
                if (now >= phaseUntilMs) {
                    // Stop pushing and go to settling
                    try { deviceIntake.motorIntake.setPower(0.0); } catch (Exception ignored) {}
                    blockBoth();
                    shootPhase = ShootPhase.SETTLING;
                    phaseUntilMs = now + SETTLE_MS;
                }
                break;
            }
            case SETTLING: {
                if (now >= phaseUntilMs) {
                    shootStep++;
                    if (shootStep == 1) { // after first artifact shot
                        // enter reassess phase to run intake and refresh inventory
                        shootPhase = ShootPhase.REASSESS;
                        phaseUntilMs = now + 600; // run intake for 600ms
                        try { deviceIntake.motorIntake.setPower(1.0); } catch (Exception ignored) {}
                        // keep both lanes blocked to avoid accidental feed during reassess
                        blockBoth();
                    } else if (shootStep >= 3) {
                        shootPhase = ShootPhase.DONE;
                        shootingActive = false;
                        blockBoth();
                    } else {
                        // Next ball
                        shootPhase = ShootPhase.FEEDING;
                        phaseUntilMs = now + FEED_MS;
                        prepareFeedForStep();
                    }
                }
                break;
            }
            case REASSESS: {
                // After intake ran, stop intake, open both briefly to scan, recompute remaining order
                if (now >= phaseUntilMs) {
                    try { deviceIntake.motorIntake.setPower(0.0); } catch (Exception ignored) {}
                    // Update inventory once more (already refreshed each loop by update(null))
                    // Recalculate remaining desired order if obelisk changed (unlikely) or inventory changed
                    Obelisk oNow = deviceCamera.getObelisk();
                    if (oNow != null) {
                        Artifact[] full = orderFromObelisk(oNow);
                        // Replace remaining portion only
                        if (full.length == 3 && shootStep < 3) {
                            for (int i = shootStep; i < 3; i++) {
                                shootOrder[i] = full[i];
                            }
                        }
                    }
                    // Proceed to next feeding if shots remain
                    if (shootStep >= 3) {
                        shootPhase = ShootPhase.DONE;
                        shootingActive = false;
                        blockBoth();
                    } else {
                        shootPhase = ShootPhase.FEEDING;
                        phaseUntilMs = now + FEED_MS;
                        prepareFeedForStep();
                    }
                }
                break;
            }
            case DONE:
            default: {
                shootingActive = false;
                blockBoth();
                break;
            }
        }
    }

    private Artifact[] orderFromObelisk(Obelisk o) {
        switch (o) {
            case GPP: return new Artifact[]{Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE};
            case PGP: return new Artifact[]{Artifact.PURPLE, Artifact.GREEN, Artifact.PURPLE};
            case PPG: return new Artifact[]{Artifact.PURPLE, Artifact.PURPLE, Artifact.GREEN};
            default: return new Artifact[]{Artifact.PURPLE, Artifact.PURPLE, Artifact.GREEN};
        }
    }

    private void prepareFeedForStep() {
        // Choose which side to feed based on desired color and available inventory
        Artifact desired = (shootOrder != null && shootStep < shootOrder.length) ? shootOrder[shootStep] : Artifact.NONE;
        Artifact left = deviceIntake.getInventory().getArtifact(ArtifactInventory.Side.LEFT);
        Artifact right = deviceIntake.getInventory().getArtifact(ArtifactInventory.Side.RIGHT);

        ArtifactInventory.Side chosen;
        if (desired != Artifact.NONE) {
            if (left == desired) chosen = ArtifactInventory.Side.LEFT;
            else if (right == desired) chosen = ArtifactInventory.Side.RIGHT;
            else {
                // fallback: shoot whatever is available
                if (left != Artifact.NONE) chosen = ArtifactInventory.Side.LEFT;
                else if (right != Artifact.NONE) chosen = ArtifactInventory.Side.RIGHT;
                else chosen = ArtifactInventory.Side.BOTH; // nothing present; let both open as last resort
            }
        } else {
            // No specific desire -> shoot any
            if (left != Artifact.NONE) chosen = ArtifactInventory.Side.LEFT;
            else if (right != Artifact.NONE) chosen = ArtifactInventory.Side.RIGHT;
            else chosen = ArtifactInventory.Side.BOTH;
        }

        // Gate servos so only the chosen side can feed
        switch (chosen) {
            case LEFT: allowLeftBlockRight(); break;
            case RIGHT: allowRightBlockLeft(); break;
            case BOTH: default: openBoth(); break; // removed NONE unreachable label
        }
    }

    // Servo helper patterns inferred from DeviceIntake defaults
    // Left: open 0.3, closed 0.56
    // Right: open 0.56, closed 0.3
    private void openBoth() {
        try {
            deviceIntake.setServoOverride(true);
            deviceIntake.setServoPositions(0.3, 0.56);  // left open, right open
        } catch (Exception ignored) {}
    }

    private void blockBoth() {
        try {
            deviceIntake.setServoOverride(true);
            deviceIntake.setServoPositions(0.56, 0.3); // left closed, right closed
        } catch (Exception ignored) {}
    }

    private void allowLeftBlockRight() {
        try {
            deviceIntake.setServoOverride(true);
            deviceIntake.setServoPositions(0.3, 0.3);  // left open, right closed
        } catch (Exception ignored) {}
    }

    private void allowRightBlockLeft() {
        try {
            deviceIntake.setServoOverride(true);
            deviceIntake.setServoPositions(0.56, 0.56); // left closed, right open
        } catch (Exception ignored) {}
    }
}
