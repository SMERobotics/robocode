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

//    public float leftPosition = 0;
//    public float rightPosition = 0;


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

        // TODO: refactor into Toggleable class???
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
            servoLeft.setPosition(0.56); // left closed position
        } else {
            servoLeft.setPosition(0.3); // left open position
        }

        if (rightActivated) {
            servoRight.setPosition(0.3); // right closed position
        } else {
            servoRight.setPosition(0.56); // right open position
        }

        // calibration shit below

//        if (gamepad.dpad_left) {
//            leftPosition += 0.001;
//        }
//
//        if (gamepad.dpad_down) {
//            leftPosition -= 0.001;
//        }
//
//        if (gamepad.dpad_right) {
//            rightPosition += 0.001;
//        }
//
//        if (gamepad.dpad_up) {
//            rightPosition -= 0.001;
//        }
//
//        servoLeft.setPosition(leftPosition);
//        servoRight.setPosition(rightPosition);
    }

    public void update(Telemetry telemetry) {
        telemetry.addData("lcol", "R: %d, G: %d, B: %d", colorLeft.red(), colorLeft.green(), colorLeft.blue());
        telemetry.addData("rcol", "R: %d, G: %d, B: %d", colorRight.red(), colorRight.green(), colorRight.blue());
//        telemetry.addData("lpos", servoLeft.getPosition());
//        telemetry.addData("rpos", servoRight.getPosition());
    }

    @Override
    public void stop() {

    }
}
