package org.technodot.ftc.twentysix.tests;

import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.technodot.ftc.twentysix.Configuration;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.OptionalDouble;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

/**
 * Place the robot on a clear, level area with its normal pod loading. INIT while
 * stationary, then START. The robot moves about 20 cm in each direction and makes
 * repeated slow turns automatically; B aborts. Allow about three minutes.
 *
 * Results target the symmetric mecanum drive's kinematic center. Translation is
 * measured with FR/BL/BR encoders, not assumed zero when a revolution finishes.
 * The broken FL encoder is never read; its motor still participates in driving.
 * Pod/IMU position feedback opposes accumulated drift during turns. The initial
 * offsets define the point held near the starting position and the travel guard;
 * they do not constrain the fitted offsets or assume that turns close.
 * Copy the printed configuration lines only after a VALIDATED result, and check
 * repeatability on the actual playing surface. Consistent drive-wheel slip can
 * bias even repeatable results. This OpMode never saves offsets to Configuration.
 */
@TeleOp(name="TestPinpointOffsets", group="TechnoCode")
public class TestPinpointOffsets extends OpMode {
    private static final String TAG = "PinpointOffsets";
    private static final double LEG_MM = 200;
    private static final double ROTATION_BIN = Math.toRadians(5);
    private static final double MAX_POWER = 0.7;
    private static final double MAX_SECONDS = 240;
    private static final double MAX_TRAVEL_MM = 750;
    private static final double RATE_ACCELERATION = Math.toRadians(60);
    private static final double[] SPIN_DEGREES = {30, -30, -60, 60, 45, -45};
    private static final String[] TRANSLATION_NAMES = {"Forward", "Backward", "Left", "Right"};

    private enum State { WAIT_READY, TRANSLATE, SPIN_RAMP, SPIN_COLLECT, BRAKE, WAIT_FIT, FINAL_ANALYSIS, DONE, ABORTED }

    private final DcMotorEx[] motors = new DcMotorEx[4]; // FL, FR, BL, BR
    private final List<LynxModule> hubs = new ArrayList<>();
    private final List<LynxModule.BulkCachingMode> cacheModes = new ArrayList<>();
    private final List<PinpointOffsetEstimator.Sample> samples = new ArrayList<>();
    private final double[] movementWheelTicks = new double[4];
    private final int[] trainingTurns = new int[4];
    private final PinpointOffsetEstimator.Accumulator bin = new PinpointOffsetEstimator.Accumulator();
    private final ExecutorService analysis = Executors.newSingleThreadExecutor(r -> {
        Thread thread = new Thread(r, "PinpointOffsetFit");
        thread.setDaemon(true);
        return thread;
    });

    private GoBildaPinpointDriver pinpoint;
    private Future<PinpointOffsetEstimator.Result> pendingFit;
    private PinpointOffsetEstimator.Result result;
    private PinpointTranslationController translation;
    private PinpointPositionHold positionHold;
    private State state = State.WAIT_READY;
    private String message = "Keep the robot stationary for IMU calibration";
    private String fitMessage = "Waiting for independent translation and rotation data";
    private Snapshot previous, current;
    private double mmPerTick, xSign, ySign, initialXOffset, initialYOffset;
    private DistanceUnit outputUnit;
    private double initTime, startTime, stateTime, stableSince = Double.NaN;
    private double finishedTime = Double.NaN;
    private double lastTelemetry, heading, podForward, podLeft;
    private double legHeading, legStartForward, legStartLeft, legWheelTicks;
    private double turnStart, rateCommand, rateIntegral, lastHeadingChange, stalledSince = Double.NaN;
    private int step, turnsInRun, turnId, nextTurnId;
    private int[] encoderPolarity;
    private boolean started, initialized, brakingTranslation, latestFitSucceeded;

    @Override
    public void init() {
        initTime = seconds();
        telemetry.setMsTransmissionInterval(100);
        try {
            MecanumConfig drive = Configuration.driveConfig;
            String[] names = {drive.frontLeftName.get(), drive.frontRightName.get(),
                    drive.backLeftName.get(), drive.backRightName.get()};
            DcMotorSimple.Direction[] directions = {drive.frontLeftDirection.get(), drive.frontRightDirection.get(),
                    drive.backLeftDirection.get(), drive.backRightDirection.get()};
            for (int i = 0; i < 4; i++) {
                motors[i] = hardwareMap.get(DcMotorEx.class, names[i]);
                motors[i].setPower(0);
                motors[i].setDirection(directions[i]);
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
                hubs.add(hub);
                cacheModes.add(hub.getBulkCachingMode());
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }

            PinpointConfig config = Configuration.localizerConfig;
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, config.name.get());
            OptionalDouble customResolution = config.ticksPerUnit.get();
            if (customResolution.isPresent()) {
                double ticks = customResolution.getAsDouble();
                mmPerTick = config.encoderResolutionUnit.get().toMm(1.0 / ticks);
                pinpoint.setEncoderResolution(ticks, config.encoderResolutionUnit.get());
            } else {
                GoBildaPinpointDriver.GoBildaOdometryPods type = config.podType.get();
                pinpoint.setEncoderResolution(type);
                // Same nominal ticks/mm values as the SDK's pod presets.
                mmPerTick = 1.0 / (type == GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
                        ? 13.26291192 : 19.89436789);
            }
            if (!PinpointOffsetEstimator.finite(mmPerTick) || mmPerTick <= 0) {
                throw new IllegalArgumentException("Invalid pod encoder resolution");
            }
            pinpoint.setEncoderDirections(config.xPodDirection.get(), config.yPodDirection.get());
            // getEncoderX/Y return raw counters; firmware direction settings only
            // affect the fused pose. Apply the configured signs to raw deltas once.
            xSign = config.xPodDirection.get() == GoBildaPinpointDriver.EncoderDirection.REVERSED ? -1 : 1;
            ySign = config.yPodDirection.get() == GoBildaPinpointDriver.EncoderDirection.REVERSED ? -1 : 1;
            outputUnit = config.offsetUnits.get();
            initialXOffset = outputUnit.toMm(config.xPodOffset.get());
            initialYOffset = outputUnit.toMm(config.yPodOffset.get());
            positionHold = new PinpointPositionHold(initialXOffset, initialYOffset);
            pinpoint.setOffsets(initialXOffset, initialYOffset, DistanceUnit.MM);
            pinpoint.resetPosAndIMU();
            initialized = true;
        } catch (RuntimeException e) {
            abort("Initialization: " + e.getMessage());
        }
        showTelemetry(seconds(), true);
    }

    @Override
    public void init_loop() {
        if (initialized && state != State.ABORTED) {
            try {
                readSensors();
                updateReadiness(seconds());
            } catch (RuntimeException e) {
                abort("Sensor initialization: " + e.getMessage());
            }
        }
        showTelemetry(seconds(), false);
    }

    @Override
    public void start() {
        started = true;
        startTime = stateTime = seconds();
        if (state != State.ABORTED) {
            state = State.WAIT_READY;
            message = "Waiting for stationary, READY Pinpoint";
        }
    }

    @Override
    public void loop() {
        double now = seconds();
        try {
            if (gamepad1.b && state != State.ABORTED) abort("Aborted with B");
            pollFit();
            if (active()) {
                if (now - startTime > MAX_SECONDS) {
                    abort("Four-minute limit reached; measurements are inconclusive");
                } else {
                    Delta delta = readSensors();
                    if (state == State.WAIT_READY) {
                        updateReadiness(now);
                        if (ready(now)) beginStep(now);
                        else if (now - stateTime > 8) abort("Pinpoint did not become stationary and READY");
                    } else {
                        if (current == null || current.status != GoBildaPinpointDriver.DeviceStatus.READY) {
                            abort("Pinpoint status: " + (current == null ? "missing data" : current.status));
                        } else if (delta != null) {
                            checkMotion(delta, now);
                            if (active()) advance(delta, now);
                        }
                    }
                }
            }
        } catch (RuntimeException e) {
            abort("Calibration error: " + e.getMessage());
        }
        showTelemetry(now, false);
    }

    @Override
    public void stop() {
        stopMotors();
        if (pendingFit != null) pendingFit.cancel(true);
        analysis.shutdownNow();
        for (int i = 0; i < hubs.size(); i++) hubs.get(i).setBulkCachingMode(cacheModes.get(i));
        logResult("Stopped");
    }

    private boolean active() {
        return state != State.DONE && state != State.ABORTED;
    }

    private static final class Snapshot {
        double time, rawHeading, rate;
        int podX, podY;
        final int[] motor = new int[4];
        GoBildaPinpointDriver.DeviceStatus status;
    }

    private static final class Delta {
        double dt, forward, left, angle, middleHeading;
        final double[] ticks = new double[4];
    }

    private Delta readSensors() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        double before = seconds();
        pinpoint.update();
        Snapshot next = new Snapshot();
        next.status = pinpoint.getDeviceStatus();
        next.rawHeading = pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS);
        next.rate = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        next.podX = pinpoint.getEncoderX();
        next.podY = pinpoint.getEncoderY();
        for (int i = PinpointOffsetEstimator.FIRST_ENCODER; i < 4; i++) next.motor[i] = motors[i].getCurrentPosition();
        next.time = (before + seconds()) / 2;
        previous = current;
        current = next;
        if (next.status != GoBildaPinpointDriver.DeviceStatus.READY) return null;
        if (!PinpointOffsetEstimator.finite(next.rawHeading) || !PinpointOffsetEstimator.finite(next.rate)) {
            throw new IllegalArgumentException("Non-finite IMU reading");
        }
        if (previous == null || previous.status != GoBildaPinpointDriver.DeviceStatus.READY) return null;
        Delta d = new Delta();
        d.dt = next.time - previous.time;
        d.angle = PinpointOffsetEstimator.headingDelta(next.rawHeading, previous.rawHeading);
        d.forward = xSign * mmPerTick * PinpointOffsetEstimator.counterDelta(next.podX, previous.podX);
        d.left = ySign * mmPerTick * PinpointOffsetEstimator.counterDelta(next.podY, previous.podY);
        for (int i = PinpointOffsetEstimator.FIRST_ENCODER; i < 4; i++) {
            d.ticks[i] = PinpointOffsetEstimator.counterDelta(next.motor[i], previous.motor[i]);
        }
        if (d.dt <= 0) throw new IllegalArgumentException("Non-increasing sample clock");
        d.middleHeading = heading + d.angle / 2;
        heading += d.angle;
        podForward += d.forward;
        podLeft += d.left;
        if (started && state != State.WAIT_READY) {
            positionHold.observe(d.forward, d.left, d.angle, d.middleHeading, d.dt);
        }
        return d;
    }

    private void updateReadiness(double now) {
        boolean still = current != null && current.status == GoBildaPinpointDriver.DeviceStatus.READY
                && PinpointOffsetEstimator.finite(current.rate) && Math.abs(current.rate) < Math.toRadians(0.1)
                && (previous == null || (Math.abs((double) PinpointOffsetEstimator.counterDelta(current.podX, previous.podX)) * mmPerTick < 0.3
                && Math.abs((double) PinpointOffsetEstimator.counterDelta(current.podY, previous.podY)) * mmPerTick < 0.3));
        if (!still) stableSince = Double.NaN;
        else if (Double.isNaN(stableSince)) stableSince = now;
        message = ready(now) ? "Ready: START begins automatic movement; B aborts" : "Keep stationary: waiting for stable IMU and pod readings";
    }

    private boolean ready(double now) {
        return now - initTime >= 1.5 && !Double.isNaN(stableSince) && now - stableSince >= 1;
    }

    private void beginStep(double now) {
        bin.clear();
        Arrays.fill(movementWheelTicks, 0);
        stateTime = now;
        lastHeadingChange = now;
        stableSince = stalledSince = Double.NaN;
        rateCommand = rateIntegral = 0;
        if (step < 4) {
            legHeading = heading;
            legStartForward = podForward;
            legStartLeft = podLeft;
            legWheelTicks = 0;
            translation = new PinpointTranslationController(LEG_MM, MAX_POWER);
            state = State.TRANSLATE;
            message = TRANSLATION_NAMES[step] + " 200 mm";
        } else if (step < 10) {
            turnsInRun = step < 8 ? trainingTurns[step - 4] : 0;
            state = State.SPIN_RAMP;
            message = String.format(Locale.US, "%s %s at %.0f deg/s",
                    step < 8 ? "Measuring" : "Validating", targetRate() > 0 ? "CCW" : "CW", Math.abs(SPIN_DEGREES[step - 4]));
        } else {
            stopMotors();
            state = State.FINAL_ANALYSIS;
            message = "Computing repeatability and independent validation";
            submitFit(true);
        }
    }

    private void advance(Delta d, double now) {
        switch (state) {
            case TRANSLATE:
                translate(d, now);
                break;
            case SPIN_RAMP:
                controlSpin(d);
                if (Math.abs(current.rate - targetRate()) <= 0.15 * Math.abs(targetRate())) {
                    if (Double.isNaN(stableSince)) stableSince = now;
                    if (now - stableSince >= 0.3) {
                        state = State.SPIN_COLLECT;
                        turnStart = heading;
                        turnId = nextTurnId++;
                        bin.clear();
                    }
                } else stableSince = Double.NaN;
                if (now - stateTime > 8) abort("Could not reach a stable turn rate at the power limit");
                break;
            case SPIN_COLLECT:
                controlSpin(d);
                collect(d, now);
                break;
            case BRAKE:
                stopMotors();
                if (Math.abs(current.rate) < Math.toRadians(3)
                        && Math.hypot(d.forward, d.left) / d.dt < 10) {
                    if (Double.isNaN(stableSince)) stableSince = now;
                    if (now - stableSince > 0.35 && now - stateTime > 0.6) {
                        if (brakingTranslation || step >= 8) nextStep(now);
                        else { state = State.WAIT_FIT; stateTime = now; }
                    }
                } else stableSince = Double.NaN;
                if (now - stateTime > 3) abort("Robot did not settle after braking");
                break;
            case WAIT_FIT:
                stopMotors();
                if (pendingFit == null) {
                    int next = PinpointOffsetEstimator.nextTrainingRun(trainingTurns,
                            latestFitSucceeded && result != null ? result.acceptedTurns : null);
                    if (next >= 0) {
                        step = next == 4 ? 8 : next + 4;
                        beginStep(now);
                    } else abort("Calibration remained inconsistent after collecting both directions and speeds: " + fitMessage);
                }
                if (now - stateTime > 12) abort("Offset fit timed out");
                break;
            case FINAL_ANALYSIS:
                stopMotors();
                if (pendingFit == null) {
                    state = State.DONE;
                    finishedTime = now;
                    message = result != null && result.validated() ? "VALIDATED: copy the configuration values below"
                            : "INCONCLUSIVE: see quality checks below";
                    logResult("Finished");
                    analysis.shutdown();
                }
                break;
            default:
                break;
        }
    }

    private void translate(Delta d, double now) {
        bin.add(d.ticks, d.forward, d.left, d.angle, d.dt, d.middleHeading);
        if (Math.hypot(bin.forward, bin.left) >= 20 || (bin.seconds >= 0.25 && Math.hypot(bin.forward, bin.left) >= 3)) {
            samples.add(bin.take(-1 - step, -1));
        }
        double sign = step % 2 == 0 ? 1 : -1;
        double distance = step < 2 ? podForward - legStartForward + initialXOffset * (heading - legHeading)
                : podLeft - legStartLeft - initialYOffset * (heading - legHeading);
        double progress = sign * distance;
        if (Math.abs(heading - legHeading) > Math.toRadians(5)) { abort("Excessive rotation during translation; check drivetrain"); return; }
        for (int i = PinpointOffsetEstimator.FIRST_ENCODER; i < 4; i++) {
            legWheelTicks += Math.abs(d.ticks[i]) / PinpointOffsetEstimator.ENCODER_COUNT;
            movementWheelTicks[i] += d.ticks[i];
        }
        translation.update(progress, legWheelTicks, d.dt);
        if (translation.status() == PinpointTranslationController.Status.COMPLETE) {
            int[] measuredPolarity = PinpointOffsetEstimator.encoderPolarities(movementWheelTicks, step);
            if (encoderPolarity == null) encoderPolarity = measuredPolarity;
            else PinpointOffsetEstimator.checkEncoderPolarities(encoderPolarity, measuredPolarity);
            if (bin.seconds > 0 && Math.hypot(bin.forward, bin.left) >= 1) samples.add(bin.take(-1 - step, -1));
            RobotLog.ii(TAG, "%s leg complete: %s; raw working encoder counts %s; forward counting signs FR/BL/BR %s",
                    TRANSLATION_NAMES[step], translationDetails(),
                    PinpointOffsetEstimator.formatCounts(movementWheelTicks), Arrays.toString(Arrays.copyOfRange(encoderPolarity, 1, 4)));
            brake(now, true);
            return;
        }
        if (translation.status() != PinpointTranslationController.Status.RUNNING) {
            String reason;
            switch (translation.status()) {
                case WRONG_DIRECTION:
                    reason = "pod travel opposes the command; check pod/motor directions";
                    break;
                case DRIVE_STALLED:
                    reason = "no drive or pod movement at the power limit; check obstruction or motor power";
                    break;
                case WHEELS_WITHOUT_POD_TRAVEL:
                    reason = "drive encoders moved without pod travel; check traction, pods, and wheel directions";
                    break;
                case DRIVE_ENCODERS_NOT_TRACKING:
                    reason = "pods moved but drive encoders did not; check motor encoder connections";
                    break;
                default:
                    reason = "translation time limit reached";
                    break;
            }
            abort(TRANSLATION_NAMES[step] + " leg: " + reason + "; " + translationDetails());
            return;
        }
        double power = translation.power();
        double turn = clip(1.2 * (legHeading - heading) - 0.08 * current.rate, -0.12, 0.12);
        drive(step < 2 ? sign * power : 0, step < 2 ? 0 : sign * power, turn);
    }

    private String translationDetails() {
        return String.format(Locale.US, "%.1f/%.0f mm, %.1f mm/s, peak power %.2f, drive %.0f ticks, %.1f s",
                translation.progressMm(), LEG_MM, translation.speedMmPerSecond(), translation.peakPower(),
                translation.wheelTicks(), translation.elapsedSeconds());
    }

    private void collect(Delta d, double now) {
        for (int i = PinpointOffsetEstimator.FIRST_ENCODER; i < 4; i++) movementWheelTicks[i] += d.ticks[i];
        bin.add(d.ticks, d.forward, d.left, d.angle, d.dt, d.middleHeading);
        if (Math.abs(bin.heading) >= ROTATION_BIN) samples.add(bin.take(step - 4, turnId));
        if ((heading - turnStart) * Math.signum(targetRate()) >= PinpointOffsetEstimator.FULL_TURN) {
            if (bin.seconds > 0) samples.add(bin.take(step - 4, turnId));
            turnsInRun++;
            RobotLog.ii(TAG, "Run %d, revolution %d: working encoder counts since movement start %s",
                    step - 3, turnsInRun, PinpointOffsetEstimator.formatCounts(movementWheelTicks));
            if (step < 8) {
                trainingTurns[step - 4]++;
                submitFit(false);
            }
            if (turnsInRun >= (step < 8 ? PinpointOffsetEstimator.REQUIRED_TURNS : 1)) brake(now, false);
            else {
                turnStart = heading;
                turnId = nextTurnId++;
            }
        }
    }

    private void controlSpin(Delta d) {
        double dt = d.dt;
        rateCommand += clip(targetRate() - rateCommand, -RATE_ACCELERATION * dt, RATE_ACCELERATION * dt);
        double error = rateCommand - current.rate;
        rateIntegral = clip(rateIntegral + 0.12 * error * dt, -0.12, 0.12);
        double turn = clip(0.08 * Math.signum(rateCommand) + 0.16 * error + rateIntegral, -MAX_POWER, MAX_POWER);
        positionHold.control(heading, dt, MAX_POWER - Math.abs(turn));
        drive(positionHold.forwardPower(), positionHold.leftPower(), turn);
    }

    private double targetRate() { return Math.toRadians(SPIN_DEGREES[step - 4]); }

    private void brake(double now, boolean translation) {
        stopMotors();
        positionHold.pause();
        state = State.BRAKE;
        brakingTranslation = translation;
        stateTime = now;
        stableSince = stalledSince = Double.NaN;
        bin.clear();
    }

    private void nextStep(double now) {
        step++;
        turnsInRun = 0;
        beginStep(now);
    }

    private void checkMotion(Delta d, double now) {
        if (d.dt > 0.2 || Math.abs(d.angle) > 4 * d.dt + 0.02
                || Math.hypot(d.forward, d.left) > 2000 * d.dt + 5) {
            abort("Stale, delayed, or implausible sensor reading");
            return;
        }
        if (positionHold.distanceMm() > MAX_TRAVEL_MM) {
            abort(String.format(Locale.US, "Travel limit: %.0f / %.0f mm from start (X %.0f, Y %.0f); position correction could not contain drift",
                    positionHold.distanceMm(), MAX_TRAVEL_MM, positionHold.xMm(), positionHold.yMm()));
            return;
        }
        if (state == State.SPIN_RAMP || state == State.SPIN_COLLECT) {
            if (Math.abs(d.angle) > Math.toRadians(0.01)) lastHeadingChange = now;
            if (now - lastHeadingChange > 1.5) {
                abort("IMU heading stopped changing during commanded rotation");
            } else if (current.rate * Math.signum(targetRate()) < -Math.toRadians(5)) {
                abort("Turn direction opposes IMU heading; check drivetrain or Pinpoint mounting");
            } else if (Math.abs(current.rate) < Math.toRadians(3)) {
                if (Double.isNaN(stalledSince)) stalledSince = now;
                if (now - stalledSince > 2) abort("Rotation stalled or IMU readings stopped changing");
            } else stalledSince = Double.NaN;
        }
    }

    private void submitFit(boolean finalAnalysis) {
        // Immutable snapshots only; the worker never accesses hardware or telemetry.
        List<PinpointOffsetEstimator.Sample> snapshot = new ArrayList<>(samples);
        if (pendingFit != null && !pendingFit.isDone()) pendingFit.cancel(true);
        latestFitSucceeded = false;
        pendingFit = analysis.submit(() -> PinpointOffsetEstimator.estimate(snapshot, finalAnalysis));
    }

    private void pollFit() {
        if (pendingFit == null || !pendingFit.isDone()) return;
        try {
            result = pendingFit.get();
            latestFitSucceeded = true;
            StringBuilder checks = new StringBuilder();
            for (String issue : result.issues) {
                if (checks.length() > 0) checks.append("; ");
                checks.append(issue);
            }
            fitMessage = checks.length() == 0 ? "Measurement groups agree" : checks.toString();
        } catch (ExecutionException e) {
            fitMessage = e.getCause().getMessage();
            latestFitSucceeded = false;
            RobotLog.ee(TAG, "Offset fit: %s", fitMessage);
            if (state == State.FINAL_ANALYSIS) {
                // Retain provisional values for inspection, never label them validated.
                message = "INCONCLUSIVE: " + fitMessage;
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            abort("Analysis interrupted");
        }
        pendingFit = null;
    }

    private void drive(double forward, double left, double ccw) {
        double[] power = {forward - left - ccw, forward + left + ccw,
                forward + left - ccw, forward - left + ccw};
        double max = MAX_POWER;
        for (double value : power) max = Math.max(max, Math.abs(value));
        for (int i = 0; i < 4; i++) motors[i].setPower(power[i] * MAX_POWER / max);
    }

    private void stopMotors() {
        for (DcMotorEx motor : motors) {
            if (motor != null) {
                try { motor.setPower(0); }
                catch (RuntimeException e) { RobotLog.ee(TAG, "Unable to stop motor: %s", e.getMessage()); }
            }
        }
    }

    private void abort(String reason) {
        stopMotors();
        state = State.ABORTED;
        finishedTime = seconds();
        message = reason;
        if (pendingFit != null) pendingFit.cancel(true);
        pendingFit = null;
        analysis.shutdownNow();
        RobotLog.ee(TAG, "Calibration stopped: %s", reason);
    }

    private void showTelemetry(double now, boolean force) {
        if (!force && now - lastTelemetry < 0.1) return;
        lastTelemetry = now;
        telemetry.addData("Status", "%s: %s", state, message);
        telemetry.addLine("Automatic 20 cm moves and turns; clear the floor. B = abort.");
        telemetry.addLine("Using FR/BL/BR encoders. FL encoder ignored.");
        if (started && positionHold != null) {
            telemetry.addData("Travel from start", "%.0f / %.0f mm; X %.0f, Y %.0f",
                    positionHold.distanceMm(), MAX_TRAVEL_MM, positionHold.xMm(), positionHold.yMm());
            if (state == State.SPIN_RAMP || state == State.SPIN_COLLECT) {
                telemetry.addData("Position correction F/L power", "%.3f / %.3f",
                        positionHold.forwardPower(), positionHold.leftPower());
            }
        }
        if (started) telemetry.addData("Elapsed", "%.1f / %.0f s",
                (Double.isNaN(finishedTime) ? now : finishedTime) - startTime, MAX_SECONDS);
        if (current != null) telemetry.addData("Pinpoint", "%s; %.1f deg/s", current.status, Math.toDegrees(current.rate));
        if (started && step < 10) {
            telemetry.addData("Working encoder changes", PinpointOffsetEstimator.formatCounts(movementWheelTicks));
        }
        if (encoderPolarity != null) telemetry.addData("Encoder forward signs FR/BL/BR",
                Arrays.toString(Arrays.copyOfRange(encoderPolarity, 1, 4)));
        if (step < 4 && translation != null) {
            telemetry.addData("Translation leg", "%d / 4: %s", step + 1, TRANSLATION_NAMES[step]);
            telemetry.addData("Leg travel", "%.1f / %.0f mm", translation.progressMm(), LEG_MM);
            telemetry.addData("Leg speed", "%.1f mm/s", translation.speedMmPerSecond());
            telemetry.addData("Translation power / peak / limit", "%.3f / %.3f / %.3f",
                    state == State.TRANSLATE ? translation.power() : 0, translation.peakPower(), MAX_POWER);
            telemetry.addData("Drive encoder travel", "%.0f mean ticks", translation.wheelTicks());
        }
        if (step >= 4 && step < 10) {
            telemetry.addData("Run", "%d / 6; revolutions collected %d", step - 3, turnsInRun);
            if (state == State.SPIN_COLLECT) telemetry.addData("Current turn", "%.0f deg", Math.abs(Math.toDegrees(heading - turnStart)));
        }
        telemetry.addData("Measurements", samples.size());
        if (result != null) {
            telemetry.addData(result.validated() && state == State.DONE ? "VALIDATED X pod offset" : "Estimated X pod offset",
                    "%.3f mm / %.6f in", result.xMm, result.xMm / 25.4);
            telemetry.addData("Estimated Y pod offset", "%.3f mm / %.6f in", result.yMm, result.yMm / 25.4);
            telemetry.addLine("X: forward pod's left/right offset. Y: sideways pod's forward/back offset.");
            telemetry.addData("Accepted turns: CCW slow, CW slow, CW fast, CCW fast", Arrays.toString(result.acceptedTurns));
            telemetry.addData("Accepted data / residual", "%.1f%% / %.3f mm", 100 * result.inlierFraction, result.residualMm);
            telemetry.addData("Drive scales F/L (mm/tick)", "%.6f / %.6f", result.forwardMmPerTick, result.leftMmPerTick);
            telemetry.addData("Signed working encoder gains", PinpointOffsetEstimator.formatCounts(result.wheelScales));
            telemetry.addData("95% repeatability +/- (mm)", "X %s; Y %s", measurement(result.xIntervalMm), measurement(result.yIntervalMm));
            telemetry.addData("Direction / speed / validation gaps (mm)", "%s / %s / %s",
                    measurement(result.directionGapMm), measurement(result.speedGapMm), measurement(result.validationGapMm));
            telemetry.addLine("Repeatability does not measure systematic drive-wheel slip.");
            if (state == State.DONE && result.validated()) {
                telemetry.addData("Configuration units", outputUnit);
                telemetry.addLine(String.format(Locale.US, "c.xPodOffset.set(%.8f);", outputUnit.fromMm(result.xMm)));
                telemetry.addLine(String.format(Locale.US, "c.yPodOffset.set(%.8f);", outputUnit.fromMm(result.yMm)));
            }
        }
        telemetry.addData("Quality checks", fitMessage);
        telemetry.update();
    }

    private void logResult(String event) {
        RobotLog.ii(TAG, "%s: %s; %s", event, message, fitMessage);
        if (result == null) return;
        RobotLog.ii(TAG, "X=%.6f mm Y=%.6f mm; repeatability +/- %.6f, %.6f mm; inliers %.4f; direction/speed/validation gaps %.6f/%.6f/%.6f mm; wheel gains %s",
                result.xMm, result.yMm, result.xIntervalMm, result.yIntervalMm, result.inlierFraction,
                result.directionGapMm, result.speedGapMm, result.validationGapMm, PinpointOffsetEstimator.formatCounts(result.wheelScales));
        if (result.validated() && state == State.DONE) {
            RobotLog.ii(TAG, "Configuration (%s): c.xPodOffset.set(%.8f); c.yPodOffset.set(%.8f);",
                    outputUnit, outputUnit.fromMm(result.xMm), outputUnit.fromMm(result.yMm));
        }
    }

    private static double clip(double value, double min, double max) { return Math.max(min, Math.min(max, value)); }

    private String measurement(double value) {
        return PinpointOffsetEstimator.finite(value) ? String.format(Locale.US, "%.3f", value)
                : (state == State.DONE || state == State.ABORTED ? "unavailable" : "pending");
    }

    private static double seconds() { return System.nanoTime() * 1e-9; }
}
