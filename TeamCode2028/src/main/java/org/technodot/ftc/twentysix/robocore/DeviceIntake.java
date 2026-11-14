package org.technodot.ftc.twentysix.robocore;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.technodot.ftc.twentysix.common.Alliance;
import org.technodot.ftc.twentysix.roboctrl.GamepadBindings;

public class DeviceIntake extends Device {

    public static final double TRANSFER_OPEN_POS = 0.04;
    public static final double TRANSFER_CLOSED_POS = 0.17;

    public static final double FLOWER_HOLD_POS = 0.57;
    public static final double FLOWER_DOWN_POS = 0.35;
    public static final double FLOWER_UP_POS = 0.03;
    private static final long INTAKE_REVERSE_DURATION_NANOS = 100_000_000L;
    public static final long SHOOT_INTAKE_DELAY_MS = 100;
    private static final long SHOOT_INTAKE_DELAY_NANOS = SHOOT_INTAKE_DELAY_MS * 1_000_000L;

    public DcMotorEx motorIntakeLeft;
    public DcMotorEx motorIntakeRight;
    public Servo servoTransfer;
    public Servo servoFlower;

    public double transferPos = 0.0;
    public double flowerPos = 0.0;
    private boolean intakeReverseHeld;
    private boolean intakeReversing;
    private long intakeReverseStartedNanos;
    private boolean shootingHeld;
    private long shootingStartedNanos;
    private boolean shootingIntakeActive;

    public DeviceIntake(Alliance alliance) {
        super(alliance);
    }

    @Override
    public void init(HardwareMap hardwareMap, GamepadBindings gamepad, Telemetry telemetry) {
        super.init(hardwareMap, gamepad, telemetry);

        motorIntakeLeft = hardwareMap.get(DcMotorEx.class, "motorIntakeLeft");
        motorIntakeRight = hardwareMap.get(DcMotorEx.class, "motorIntakeRight");
        motorIntakeLeft.setDirection(DcMotorEx.Direction.FORWARD);
        motorIntakeRight.setDirection(DcMotorEx.Direction.REVERSE);
        motorIntakeLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motorIntakeRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        servoTransfer = hardwareMap.get(Servo.class, "servoTransfer");

        servoFlower = hardwareMap.get(Servo.class, "servoFlower");
        flowerPos = FLOWER_HOLD_POS;
//        servoFlower.setPosition(flowerPos);
    }

    @Override
    public void start() {
        intakeReverseHeld = gamepad.intakeReverse();
        intakeReversing = false;
        shootingHeld = false;
        shootingIntakeActive = false;
        flowerPos = FLOWER_DOWN_POS;
        servoFlower.setPosition(flowerPos);
    }

    @Override
    public void update() {
        long now = System.nanoTime();
        boolean reversePressed = gamepad.intakeReverse();
        if (reversePressed && !intakeReverseHeld) {
            intakeReverseStartedNanos = now;
            intakeReversing = true;
        }
        intakeReverseHeld = reversePressed;
        if (intakeReversing && now - intakeReverseStartedNanos >= INTAKE_REVERSE_DURATION_NANOS) {
            intakeReversing = false;
        }

        boolean shooting = gamepad.sequenceShoot();
        transferPos = shooting ? TRANSFER_OPEN_POS : TRANSFER_CLOSED_POS;
        servoTransfer.setPosition(transferPos);
        if (shooting && !shootingHeld) {
            shootingStartedNanos = System.nanoTime();
        }
        shootingHeld = shooting;

        double power = gamepad.intakePower();
        if (shooting) {
            power = System.nanoTime() - shootingStartedNanos >= SHOOT_INTAKE_DELAY_NANOS ? 1.0 : 0.0;
        }
        if (intakeReversing) {
            power = -1.0;
        }
        shootingIntakeActive = shooting && !intakeReversing && power > 0;
        motorIntakeLeft.setPower(power);
        motorIntakeRight.setPower(power);

        flowerPos = gamepad.flowerUp() ? FLOWER_UP_POS : FLOWER_DOWN_POS;
        servoFlower.setPosition(flowerPos);

        telemetry.addData("transfer_pos", transferPos);
        telemetry.addData("flower_pos", flowerPos);
    }

    @Override
    public void stop() {
        intakeReversing = false;
        shootingHeld = false;
        shootingIntakeActive = false;
        motorIntakeLeft.setPower(0);
        motorIntakeRight.setPower(0);
    }

    public boolean isShootingIntakeActive() {
        return shootingIntakeActive;
    }
}
