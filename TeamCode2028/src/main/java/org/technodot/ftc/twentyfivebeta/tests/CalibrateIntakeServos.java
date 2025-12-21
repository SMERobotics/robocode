package org.technodot.ftc.twentyfivebeta.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.technodot.ftc.twentyfive.common.Controls;

@TeleOp(name="CalibrateIntakeServos", group="TechnoCode")
public class CalibrateIntakeServos extends OpMode {

    public DcMotorEx motorIntake;
    public Servo servoLeft; // perspective of the robot
    public Servo servoRight; // perspective of the robot

    public float leftPosition = 0;
    public float rightPosition = 0;

    @Override
    public void init() {
        motorIntake = hardwareMap.get(DcMotorEx.class, "motorIntake");
        motorIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");
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
        if (Controls.intakeOut(gamepad1)) {
            motorIntake.setPower(-1.0F);
        } else if (Controls.intakeIn(gamepad1)) {
            motorIntake.setPower(1.0F);
        } else {
            motorIntake.setPower(0.0F);
        }

        if (gamepad1.dpad_left) {
            leftPosition += 0.001f;
        }

        if (gamepad1.dpad_down) {
            leftPosition -= 0.001f;
        }

        if (gamepad1.dpad_right) {
            rightPosition += 0.001f;
        }

        if (gamepad1.dpad_up) {
            rightPosition -= 0.001f;
        }

        servoLeft.setPosition(leftPosition);
        servoRight.setPosition(rightPosition);

        telemetry.addData("l_pos", servoLeft.getPosition());
        telemetry.addData("r_pos", servoRight.getPosition());

        telemetry.addData("status", "running");
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("status", "stopping");
        telemetry.update();
    }
}
