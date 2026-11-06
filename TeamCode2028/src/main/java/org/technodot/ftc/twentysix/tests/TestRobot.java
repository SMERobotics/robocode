package org.technodot.ftc.twentysix.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TestRobot", group="TechnoCode")
public class TestRobot extends OpMode {

    public static final double SERVO_NECTAR_POS_A = 0.37;
    public static final double SERVO_POLLEN_POS = 0.67;
    public static final double SERVO_NECTAR_POS_B = 0.97;

    public DcMotorEx motorExtake;
    public DcMotorEx motorTurret;
    public DcMotorEx motorIntakeLeft;
    public DcMotorEx motorIntakeRight;
    public Servo servoExtake;

    @Override
    public void init() {
        motorExtake = hardwareMap.get(DcMotorEx.class, "motorExtake");
        motorExtake.setDirection(DcMotorEx.Direction.REVERSE);
        motorExtake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        motorTurret = hardwareMap.get(DcMotorEx.class, "motorTurret");
        motorTurret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        motorIntakeLeft = hardwareMap.get(DcMotorEx.class, "motorIntakeLeft");
        motorIntakeLeft.setDirection(DcMotorEx.Direction.REVERSE);
        motorIntakeLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        motorIntakeRight = hardwareMap.get(DcMotorEx.class, "motorIntakeRight");
        motorIntakeRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        servoExtake = hardwareMap.get(Servo.class, "servoExtake");
    }

    @Override
    public void init_loop() {
        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        telemetry.addData("status", "starting");
        telemetry.update();
    }

    @Override
    public void loop() {
        motorExtake.setPower(gamepad1.right_bumper ? 1.0 : gamepad1.right_trigger);

        if (gamepad1.a) {
            servoExtake.setPosition(SERVO_POLLEN_POS);
        } else if (gamepad1.b) {
            servoExtake.setPosition(nearestNectarPosition(servoExtake.getPosition()));
        } else {
            double servoAdjustment = (gamepad1.dpad_right ? 0.001 : 0.0)
                - (gamepad1.dpad_left ? 0.001 : 0.0);
            servoExtake.setPosition(Math.max(0.0, Math.min(1.0,
                servoExtake.getPosition() + servoAdjustment)));
        }

        double turretVelocity = (gamepad1.dpad_up ? 0.1 : 0.0) + (gamepad1.dpad_down ? -0.1 : 0.0);
        motorTurret.setPower(turretVelocity);

        motorIntakeLeft.setPower(gamepad1.left_bumper ? 1.0 : gamepad1.left_trigger);
        motorIntakeRight.setPower(gamepad1.left_bumper ? 1.0 : gamepad1.left_trigger);

        telemetry.addData("pwr_extake", motorExtake.getPower());
        telemetry.addData("pwr_turret", motorTurret.getPower());
        telemetry.addData("pwr_intake_left", motorIntakeLeft.getPower());
        telemetry.addData("pwr_intake_right", motorIntakeRight.getPower());

        telemetry.addData("pos_servo", servoExtake.getPosition());

        telemetry.addData("status", "running");
        telemetry.update();
    }

    private double nearestNectarPosition(double position) {
        return Math.abs(position - SERVO_NECTAR_POS_A)
            <= Math.abs(position - SERVO_NECTAR_POS_B)
            ? SERVO_NECTAR_POS_A : SERVO_NECTAR_POS_B;
    }

    @Override
    public void stop() {
        telemetry.addData("status", "stopping");
        telemetry.update();
    }
}
