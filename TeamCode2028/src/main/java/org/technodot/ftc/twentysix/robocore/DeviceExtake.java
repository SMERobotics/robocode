package org.technodot.ftc.twentysix.robocore;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.technodot.ftc.twentysix.HiveVisionConfig;
import org.technodot.ftc.twentysix.Configuration;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import org.technodot.ftc.twentysix.common.Alliance;
import org.technodot.ftc.twentysix.roboctrl.GamepadBindings;
import org.technodot.ftc.twentysix.roboctrl.HiveObservation;
import org.technodot.ftc.twentysix.roboctrl.HiveTurretAim;

public class DeviceExtake extends Device {

//    public static final double SERVO_POLLEN_POS = 0;
public static final double SERVO_POLLEN_POS = 0.00;
    public static final double SERVO_NECTAR_POS = 0.28;

    public static final double TURRET_REVOLUTION_TICKS = (1.0 + 46.0 / 17.0) * (1.0 + 46.0 / 11.0) * 28.0 * (176.0 / 64.0);
    public static final double TURRET_POSITION_POWER = 1;
    public static final double TURRET_LOWER_BOUND = (-0.5 + -0.0625) * TURRET_REVOLUTION_TICKS;
    public static final double TURRET_UPPER_BOUND = (0.5 + 0.0625) * TURRET_REVOLUTION_TICKS;

    public DcMotorEx motorExtake;
    public DcMotorEx motorTurret;
    public Servo servoExtake;

    public double turretHeading = 0.0;
    private HiveTurretAim hiveAim;
    private HiveObservation hiveObservation;
    private long lastHiveTagSeen;
    public double extakeTargetVelocity;
    private boolean velocityIncreaseHeld;
    private boolean velocityDecreaseHeld;
    private boolean spinUpHeld;
    private boolean preset1520Held;
    private boolean preset1560Held;
    private boolean preset1600Held;
    private boolean preset1780Held;
    private PIDFCoefficients extakeVelocityPIDF;
    private PIDFCoefficients turretVelocityPIDF;
    private double turretPositionP = -1;
    private TelemetryManager panelsTelemetry;
    private double feedbackTargetVelocity = Double.NaN;
    private long speedReadySince;
    private boolean speedReadyNotified;

    public DeviceExtake(Alliance alliance) {
        super(alliance);
    }

    @Override
    public void init(HardwareMap hardwareMap, GamepadBindings gamepad, Telemetry telemetry) {
        super.init(hardwareMap, gamepad, telemetry);
        hiveAim = new HiveTurretAim(alliance, HiveVisionConfig.solver, HiveVisionConfig.turretAim);

        motorExtake = hardwareMap.get(DcMotorEx.class, "motorExtake");
        motorExtake.setDirection(DcMotorEx.Direction.REVERSE);
        motorExtake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motorExtake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorExtake.setVelocity(0);
        extakeTargetVelocity = 0;

        motorTurret = hardwareMap.get(DcMotorEx.class, "motorTurret");
        motorTurret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        initializePIDF();
        applyPIDF();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        servoExtake = hardwareMap.get(Servo.class, "servoExtake");
    }

    @Override
    public void start() {
        feedbackTargetVelocity = Double.NaN;
        speedReadySince = 0;
        speedReadyNotified = false;
        velocityIncreaseHeld = gamepad.extakeVelocityIncrease();
        velocityDecreaseHeld = gamepad.extakeVelocityDecrease();
        spinUpHeld = gamepad.extakeSpinUp();
        preset1520Held = gamepad.extakePreset1520();
        preset1560Held = gamepad.extakePreset1560();
        preset1600Held = gamepad.extakePreset1600();
        preset1780Held = gamepad.extakePreset1780();
        int currentTicks = motorTurret.getCurrentPosition();
        this.turretHeading = normalizeHeading(currentTicks * 360.0 / TURRET_REVOLUTION_TICKS);
        hiveAim.start(currentTicks * 360.0 / TURRET_REVOLUTION_TICKS, System.nanoTime());
        motorTurret.setTargetPosition(calculateTurretTarget(turretHeading, currentTicks));
        motorTurret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorTurret.setPower(TURRET_POSITION_POWER);
    }

    @Override
    public void update() {
        applyPIDF();
        int currentTicks = motorTurret.getCurrentPosition();
        long now = System.nanoTime();
        turretHeading = hiveAim.update(currentTicks * 360.0 / TURRET_REVOLUTION_TICKS,
                hiveObservation, lastHiveTagSeen, now);
        motorTurret.setTargetPosition(calculateTurretTarget(turretHeading, currentTicks));
        motorTurret.setPower(TURRET_POSITION_POWER);

        boolean increase = gamepad.extakeVelocityIncrease();
        boolean decrease = gamepad.extakeVelocityDecrease();
        boolean spinUp = gamepad.extakeSpinUp();
        if (spinUp && !spinUpHeld) {
            extakeTargetVelocity = 1650;
        }
        spinUpHeld = spinUp;
        boolean preset1520 = gamepad.extakePreset1520();
        boolean preset1560 = gamepad.extakePreset1560();
        boolean preset1600 = gamepad.extakePreset1600();
        boolean preset1780 = gamepad.extakePreset1780();
        if (preset1520 && !preset1520Held) extakeTargetVelocity = 1520;
        if (preset1560 && !preset1560Held) extakeTargetVelocity = 1560;
        if (preset1600 && !preset1600Held) extakeTargetVelocity = 1600;
        if (preset1780 && !preset1780Held) extakeTargetVelocity = 1780;
        preset1520Held = preset1520;
        preset1560Held = preset1560;
        preset1600Held = preset1600;
        preset1780Held = preset1780;
        extakeTargetVelocity = Math.max(0, extakeTargetVelocity
                + (increase && !velocityIncreaseHeld ? 20 : 0)
                - (decrease && !velocityDecreaseHeld ? 20 : 0));
        velocityIncreaseHeld = increase;
        velocityDecreaseHeld = decrease;
        if (gamepad.extakePresetCancel()) {
            extakeTargetVelocity = 0;
        }
        motorExtake.setVelocity(extakeTargetVelocity);

        if (gamepad.extakePollen()) {
            servoExtake.setPosition(SERVO_POLLEN_POS);
        } else if (gamepad.extakeNectar()) {
            servoExtake.setPosition(SERVO_NECTAR_POS);
        }

        telemetry.addData("turret_pos", currentTicks);
        telemetry.addData("turret_target_pos", motorTurret.getTargetPosition());
        telemetry.addData("turret_deg", currentTicks * 360.0 / TURRET_REVOLUTION_TICKS);
        telemetry.addData("turret_target_deg", turretHeading);
        telemetry.addData("turret aim", hiveAim.getStatus());
        double extakeVelocity = motorExtake.getVelocity();
        updateSpeedFeedback(extakeVelocity, System.nanoTime());
        telemetry.addData("extake_velocity", extakeVelocity);
        telemetry.addData("extake_target_velocity", extakeTargetVelocity);
        // Numeric telemetry is available as selectable series in the Panels Graph widget.
        panelsTelemetry.addData("turret_pos", currentTicks);
        panelsTelemetry.addData("turret_target_pos", motorTurret.getTargetPosition());
        panelsTelemetry.addData("extake_velocity", extakeVelocity);
        panelsTelemetry.addData("extake_target_velocity", extakeTargetVelocity);
        panelsTelemetry.update();
    }

    public void setHiveObservation(HiveObservation observation, long lastAllianceTagSeenNanos) {
        hiveObservation = observation;
        lastHiveTagSeen = lastAllianceTagSeenNanos;
    }

    private void updateSpeedFeedback(double velocity, long now) {
        if (feedbackTargetVelocity != extakeTargetVelocity) {
            feedbackTargetVelocity = extakeTargetVelocity;
            speedReadySince = 0;
            speedReadyNotified = false;
        }
        if (!(extakeTargetVelocity > 0)
                || !(Math.abs(velocity - extakeTargetVelocity) <= Configuration.EXTAKE_READY_VELOCITY_TOLERANCE)) {
            speedReadySince = 0;
            speedReadyNotified = false;
            return;
        }
        if (speedReadySince == 0) speedReadySince = now;
        if (!speedReadyNotified && now - speedReadySince >= Configuration.EXTAKE_READY_SETTLE_MS * 1_000_000L) {
            gamepad.vibrateExtakeReady(Configuration.EXTAKE_READY_RUMBLE_MS);
            speedReadyNotified = true;
        }
    }

    @Override
    public void stop() {
        extakeTargetVelocity = 0;
        motorExtake.setVelocity(0);
        motorExtake.setPower(0);
        motorTurret.setPower(0);
    }

    private void initializePIDF() {
        PIDFCoefficients extake = motorExtake.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
        if (Configuration.EXTAKE_VELOCITY_KP < 0) Configuration.EXTAKE_VELOCITY_KP = extake.p;
        if (Configuration.EXTAKE_VELOCITY_KI < 0) Configuration.EXTAKE_VELOCITY_KI = extake.i;
        if (Configuration.EXTAKE_VELOCITY_KD < 0) Configuration.EXTAKE_VELOCITY_KD = extake.d;
        if (Configuration.EXTAKE_VELOCITY_KF < 0) Configuration.EXTAKE_VELOCITY_KF = extake.f;
        PIDFCoefficients turret = motorTurret.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
        if (Configuration.TURRET_VELOCITY_KP < 0) Configuration.TURRET_VELOCITY_KP = turret.p;
        if (Configuration.TURRET_VELOCITY_KI < 0) Configuration.TURRET_VELOCITY_KI = turret.i;
        if (Configuration.TURRET_VELOCITY_KD < 0) Configuration.TURRET_VELOCITY_KD = turret.d;
        if (Configuration.TURRET_VELOCITY_KF < 0) Configuration.TURRET_VELOCITY_KF = turret.f;
        if (Configuration.TURRET_POSITION_KP < 0) {
            Configuration.TURRET_POSITION_KP = motorTurret.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION).p;
        }
    }

    private void applyPIDF() {
        PIDFCoefficients extake = new PIDFCoefficients(Configuration.EXTAKE_VELOCITY_KP,
                Configuration.EXTAKE_VELOCITY_KI, Configuration.EXTAKE_VELOCITY_KD, Configuration.EXTAKE_VELOCITY_KF);
        if (!samePIDF(extake, extakeVelocityPIDF)) {
            motorExtake.setVelocityPIDFCoefficients(extake.p, extake.i, extake.d, extake.f);
            extakeVelocityPIDF = extake;
        }
        PIDFCoefficients turret = new PIDFCoefficients(Configuration.TURRET_VELOCITY_KP,
                Configuration.TURRET_VELOCITY_KI, Configuration.TURRET_VELOCITY_KD, Configuration.TURRET_VELOCITY_KF);
        if (!samePIDF(turret, turretVelocityPIDF)) {
            motorTurret.setVelocityPIDFCoefficients(turret.p, turret.i, turret.d, turret.f);
            turretVelocityPIDF = turret;
        }
        if (turretPositionP != Configuration.TURRET_POSITION_KP) {
            turretPositionP = Configuration.TURRET_POSITION_KP;
            motorTurret.setPositionPIDFCoefficients(turretPositionP);
        }
    }

    private static boolean samePIDF(PIDFCoefficients a, PIDFCoefficients b) {
        return b != null && a.p == b.p && a.i == b.i && a.d == b.d && a.f == b.f;
    }

    private static int calculateTurretTarget(double heading, int currentTicks) {
        int lowerTicks = (int) Math.ceil(TURRET_LOWER_BOUND);
        int upperTicks = (int) Math.floor(TURRET_UPPER_BOUND);
        double baseTicks = normalizeHeading(heading) / 360.0 * TURRET_REVOLUTION_TICKS;

        return (int) Math.max(
            lowerTicks,
            Math.min(
                upperTicks,
                Math.round( // target ticks
                    baseTicks + Math.max( // turn
                        Math.ceil((lowerTicks - baseTicks) / TURRET_REVOLUTION_TICKS), // minimum turn
                        Math.min(
                            Math.floor((upperTicks - baseTicks) / TURRET_REVOLUTION_TICKS), // maximum turn
                            Math.round((currentTicks - baseTicks) / TURRET_REVOLUTION_TICKS) // nearest turn
                        )
                    ) * TURRET_REVOLUTION_TICKS
                )
            )
        );
    }

    private static double normalizeHeading(double heading) {
        return ((heading % 360.0) + 360.0) % 360.0;
    }
}
