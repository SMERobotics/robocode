package com.n0tasha4k.ftc.twentyfive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="CalibrateIntake", group="TechnoCode")
public class CalibrateIntake extends OpMode {

    public Servo servoIndex; // perspective of the robot
    public float servoPosition;

    @Override
    public void init() {
        servoIndex = hardwareMap.get(Servo.class, "indexfinger");
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
        if (gamepad1.dpad_up) {
            servoPosition += 0.001f;
        }

        if (gamepad1.dpad_down) {
            servoPosition -= 0.001f;
        }



        servoIndex.setPosition(servoPosition);
        telemetry.addData("index_pos", servoIndex.getPosition());

        telemetry.addData("status", "running");
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("status", "stopping");
        telemetry.update();
    }
}
