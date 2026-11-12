package org.technodot.ftc.twentysix.robocore;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.technodot.ftc.twentysix.common.Alliance;
import org.technodot.ftc.twentysix.roboctrl.GamepadBindings;

public class DeviceExtake extends Device {

    public static final double SERVO_POLLEN_POS = 0;
    public static final double SERVO_NECTAR_POS = 0.28;

    public static final double TURRET_REVOLUTION_TICKS = (1.0 + 46.0 / 17.0) * (1.0 + 46.0 / 11.0) * 28.0 * (176.0 / 64.0);
    public static final double TURRET_POSITION_POWER = 1;
    public static final double TURRET_LOWER_BOUND = (-0.5 + -0.0625) * TURRET_REVOLUTION_TICKS;
    public static final double TURRET_UPPER_BOUND = (0.5 + 0.0625) * TURRET_REVOLUTION_TICKS;

    public DcMotorEx motorExtake;
    public DcMotorEx motorTurret;
    public Servo servoExtake;

    public double TURRET_HEADING = 0.0;

    public DeviceExtake(Alliance alliance) {
        super(alliance);
    }

    @Override
    public void init(HardwareMap hardwareMap, GamepadBindings gamepad, Telemetry telemetry) {
        super.init(hardwareMap, gamepad, telemetry);

        motorExtake = hardwareMap.get(DcMotorEx.class, "motorExtake");
        motorExtake.setDirection(DcMotorEx.Direction.FORWARD);
        motorExtake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        motorTurret = hardwareMap.get(DcMotorEx.class, "motorTurret");
        motorTurret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        servoExtake = hardwareMap.get(Servo.class, "servoExtake");
    }

    @Override
    public void start() {
        int currentTicks = motorTurret.getCurrentPosition();
        TURRET_HEADING = normalizeHeading(currentTicks * 360.0 / TURRET_REVOLUTION_TICKS);
        motorTurret.setTargetPosition(calculateTurretTarget(TURRET_HEADING, currentTicks));
        motorTurret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorTurret.setPower(TURRET_POSITION_POWER);
    }

    @Override
    public void update() {
        if (gamepad.turretReset()) {
            resetTurret();
        } else {
            if (gamepad.turretHeadingIncrease()) TURRET_HEADING++;
            if (gamepad.turretHeadingDecrease()) TURRET_HEADING--;

            TURRET_HEADING = normalizeHeading(TURRET_HEADING);
            motorTurret.setTargetPosition(calculateTurretTarget(TURRET_HEADING, motorTurret.getCurrentPosition()));
            motorTurret.setPower(TURRET_POSITION_POWER);
        }

        motorExtake.setPower(gamepad.extakePower());

        if (gamepad.extakePollen()) {
            servoExtake.setPosition(SERVO_POLLEN_POS);
        } else if (gamepad.extakeNectar()) {
            servoExtake.setPosition(SERVO_NECTAR_POS);
        }

        telemetry.addData("turret_pos", motorTurret.getCurrentPosition());
        telemetry.addData("turret_target_pos", motorTurret.getTargetPosition());
        telemetry.addData("turret_deg", motorTurret.getCurrentPosition() * 360.0 / TURRET_REVOLUTION_TICKS);
        telemetry.addData("turret_target_deg", TURRET_HEADING);
    }

    @Override
    public void stop() {
        motorExtake.setPower(0);
        motorTurret.setPower(0);
    }

    private static int calculateTurretTarget(double heading, int currentTicks) {
        int lowerTicks = (int) Math.ceil(TURRET_LOWER_BOUND);
        int upperTicks = (int) Math.floor(TURRET_UPPER_BOUND);
        double baseTicks = normalizeHeading(heading) / 360.0 * TURRET_REVOLUTION_TICKS;
        double minTurn = Math.ceil((lowerTicks - baseTicks) / TURRET_REVOLUTION_TICKS);
        double maxTurn = Math.floor((upperTicks - baseTicks) / TURRET_REVOLUTION_TICKS);
        double nearestTurn = Math.round((currentTicks - baseTicks) / TURRET_REVOLUTION_TICKS);
        double turn = Math.max(minTurn, Math.min(maxTurn, nearestTurn));
        long targetTicks = Math.round(baseTicks + turn * TURRET_REVOLUTION_TICKS);
        return (int) Math.max(lowerTicks, Math.min(upperTicks, targetTicks));
    }

    private void resetTurret() {
        motorTurret.setPower(0);
        motorTurret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        TURRET_HEADING = 0;
        motorTurret.setTargetPosition(0);
        motorTurret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorTurret.setPower(TURRET_POSITION_POWER);
    }

    private static double normalizeHeading(double heading) {
        return ((heading % 360.0) + 360.0) % 360.0;
    }
}
