package org.technodot.ftc.twentysix.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.technodot.ftc.twentysix.robocore.DeviceExtake;

@TeleOp(name="TestRobot", group="TechnoCode")
public class TestRobot extends OpMode {

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
            servoExtake.setPosition(DeviceExtake.SERVO_POLLEN_POS);
        } else if (gamepad1.b) {
            servoExtake.setPosition(DeviceExtake.SERVO_NECTAR_POS);
        }

        motorTurret.setPower(0);

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

    @Override
    public void stop() {
        telemetry.addData("status", "stopping");
        telemetry.update();
    }
}
