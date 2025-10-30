package com.technodot.ftc.twentyfive.robocore;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.technodot.ftc.twentyfive.common.Artifact;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DeviceIntake extends Device {
    public DcMotor motorIntake;
    public Servo servoLeft; // perspective of the robot
    public Servo servoRight; // perspective of the robot
    public RevColorSensorV3 colorLeft;
    public RevColorSensorV3 colorRight;

    public boolean leftPressed = false;
    public boolean rightPressed = false;
    public boolean leftActivated = false;
    public boolean rightActivated = false;

//    public float leftPosition = 0;
//    public float rightPosition = 0;

    public Artifact leftArtifact = Artifact.NONE;
    public Artifact rightArtifact = Artifact.NONE;

    @Override
    public void init(HardwareMap hardwareMap) {
        motorIntake = hardwareMap.get(DcMotor.class, "motorIntake");
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");

        colorLeft = hardwareMap.get(RevColorSensorV3.class, "colorLeft");
        colorRight = hardwareMap.get(RevColorSensorV3.class, "colorRight");

        colorLeft.enableLed(true);
        colorRight.enableLed(true);
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

        // TODO: calibrate the color sensors to balls fr

        double leftDistance = colorLeft.getDistance(DistanceUnit.CM);
        NormalizedRGBA leftColor = colorLeft.getNormalizedColors();
        float[] leftHSV = new float[3];
        Color.colorToHSV(leftColor.toColor(), leftHSV);
        float leftHue = leftHSV[0];
        float leftSaturation = leftHSV[1];
        float leftValue = leftHSV[2];
        if (leftDistance <= 10 && leftSaturation <= 30 && leftValue >= 10 && leftValue <= 90) {
            if (leftHue >= 260 && leftHue <= 320) {
                leftArtifact = Artifact.PURPLE;
            } else if (leftHue >= 80 && leftHue <= 160) {
                leftArtifact = Artifact.GREEN;
            } else {
                leftArtifact = Artifact.NONE;
            }
        } else {
            leftArtifact = Artifact.NONE;
        }

        double rightDistance = colorRight.getDistance(DistanceUnit.CM);
        NormalizedRGBA rightColor = colorRight.getNormalizedColors();
        float[] rightHSV = new float[3];
        Color.colorToHSV(rightColor.toColor(), rightHSV);
        float rightHue = rightHSV[0];
        float rightSaturation = rightHSV[1];
        float rightValue = rightHSV[2];
        if (rightDistance <= 10 && rightSaturation <= 30 && rightValue >= 10 && rightValue <= 90) {
            if (rightHue >= 260 && rightHue <= 320) {
                rightArtifact = Artifact.PURPLE;
            } else if (rightHue >= 80 && rightHue <= 160) {
                rightArtifact = Artifact.GREEN;
            } else {
                rightArtifact = Artifact.NONE;
            }
        } else {
            rightArtifact = Artifact.NONE;
        }
    }

    public void update(Telemetry telemetry) {
        telemetry.addData("l_artifact", leftArtifact);
        telemetry.addData("r_artifact", rightArtifact);

//        telemetry.addData("l_pos", servoLeft.getPosition());
//        telemetry.addData("r_pos", servoRight.getPosition());
    }

    @Override
    public void stop() {

    }
}
