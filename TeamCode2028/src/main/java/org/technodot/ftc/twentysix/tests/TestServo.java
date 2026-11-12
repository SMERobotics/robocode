package org.technodot.ftc.twentysix.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TestServo", group="TechnoCode")
public class TestServo extends OpMode {

    private static final double POSITION_STEP = 0.01;

    public Servo servoExtake;
    private boolean leftBumperPressed;
    private boolean rightBumperPressed;

    @Override
    public void init() {
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
        telemetry.addData("status", "starting");
        telemetry.update();
    }

    @Override
    public void loop() {
        double adjustment = 0.0;
        if (gamepad1.left_bumper && !leftBumperPressed) {
            adjustment -= POSITION_STEP;
        }
        if (gamepad1.right_bumper && !rightBumperPressed) {
            adjustment += POSITION_STEP;
        }
        leftBumperPressed = gamepad1.left_bumper;
        rightBumperPressed = gamepad1.right_bumper;

        if (adjustment != 0.0) {
            servoExtake.setPosition(Math.max(0.0, Math.min(1.0,
                    servoExtake.getPosition() + adjustment)));
        }

        telemetry.addData("servo_pos", servoExtake.getPosition());
        telemetry.addData("status", "running");
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("status", "stopping");
        telemetry.update();
    }
}
