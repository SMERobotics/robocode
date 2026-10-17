package org.technodot.ftc.twentysix.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TestExtake", group="TechnoCode")
public class TestExtake extends OpMode {

    public DcMotorEx motorExtake;
    public Servo servoExtake;

    public static final double SERVO_NECTAR_POS_A = 0.25;
    public static final double SERVO_POLLEN_POS_A = 0.54;
    public static final double SERVO_NECTAR_POS_B = 0.83;


    @Override
    public void init() {
        motorExtake = hardwareMap.get(DcMotorEx.class, "motorExtake");
        motorExtake.setDirection(DcMotorEx.Direction.REVERSE);
        motorExtake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        servoExtake = hardwareMap.get(Servo.class, "servoExtake");
    }

    @Override
    public void init_loop() {
        telemetry.addData("status", "initialized");
        telemetry.addData("servo_pos", servoExtake.getPosition());
        telemetry.update();
    }

    @Override
    public void start() {
        motorExtake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorExtake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        servoExtake.setPosition(nearestNectarPosition(servoExtake.getPosition()));

        telemetry.addData("status", "starting");
        telemetry.update();
    }

    @Override
    public void loop() {
        motorExtake.setPower(gamepad1.right_bumper ? 1.0 : gamepad1.right_trigger);

        if (gamepad1.a) {
            servoExtake.setPosition(SERVO_POLLEN_POS_A);
        } else if (gamepad1.b) {
            servoExtake.setPosition(nearestNectarPosition(servoExtake.getPosition()));
        } else {
            double servoAdjustment = (gamepad1.dpad_right ? 0.001 : 0.0)
                    - (gamepad1.dpad_left ? 0.001 : 0.0);
            servoExtake.setPosition(Math.max(0.0, Math.min(1.0,
                    servoExtake.getPosition() + servoAdjustment)));
        }

        telemetry.addData("servo_pos", servoExtake.getPosition());
        telemetry.addData("pwr", motorExtake.getPower());
//        telemetry.addData("pos", motorExtake.getCurrentPosition());
//        telemetry.addData("vel", motorExtake.getVelocity());

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
