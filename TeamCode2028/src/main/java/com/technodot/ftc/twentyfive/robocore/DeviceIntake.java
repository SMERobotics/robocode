package com.technodot.ftc.twentyfive.robocore;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DeviceIntake extends Device {
    public DcMotor motorIntake;
    public Servo servoLeft; // perspective of the robot
    public Servo servoRight; // perspective of the robot
    public ColorSensor colorLeft;
    public ColorSensor colorRight;

    public boolean leftPressed = false;
    public boolean rightPressed = false;
    public boolean leftActivated = false;
    public boolean rightActivated = false;


    @Override
    public void init(HardwareMap hardwareMap) {
        motorIntake = hardwareMap.get(DcMotor.class, "motorIntake");
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");

        colorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
        colorRight = hardwareMap.get(ColorSensor.class, "colorRight");
    }

    @Override
    public void start() {

    }

    @Override
    public void update(Gamepad gamepad) {
        motorIntake.setPower((gamepad.right_bumper ? 1 : 0) + (gamepad.left_bumper ? -1 : 0));

        // TODO: refactor into Toggleable class?
        if (gamepad.dpad_left && !leftPressed) {
            leftActivated = !leftActivated;
            leftPressed = true;
        } else if (!gamepad.dpad_left) {
            leftPressed = false;
        }

        if (gamepad.dpad_right && !rightPressed) {
            rightActivated = !rightActivated;
            rightPressed = true;
        } else if (!gamepad.dpad_right) {
            rightPressed = false;
        }

        if (leftActivated) {
            servoLeft.setPosition(0.0);
        } else {
            servoLeft.setPosition(0.67);
        }

        if (rightActivated) {
            servoRight.setPosition(0);
        } else {
            servoRight.setPosition(-0.67);
        }
    }

    public void update(Telemetry telemetry) {
        telemetry.addData("Left Color", "R: %d, G: %d, B: %d", colorLeft.red(), colorLeft.green(), colorLeft.blue());
        telemetry.addData("Right Color", "R: %d, G: %d, B: %d", colorRight.red(), colorRight.green(), colorRight.blue());
    }

    @Override
    public void stop() {

    }
}
