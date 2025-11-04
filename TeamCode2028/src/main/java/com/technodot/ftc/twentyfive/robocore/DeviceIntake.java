package com.technodot.ftc.twentyfive.robocore;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.technodot.ftc.twentyfive.common.Artifact;
import com.technodot.ftc.twentyfive.common.Controls;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DeviceIntake extends Device {
    public DcMotorEx motorIntake;
    public Servo servoLeft; // perspective of the robot
    public Servo servoRight; // perspective of the robot
    public RevColorSensorV3 colorLeft;
    public RevColorSensorV3 colorRight;

    public boolean leftPressed = false;
    public boolean rightPressed = false;
    public boolean bothPressed = false;
    public long leftActivated = 0;
    public long rightActivated = 0;

//    public float leftPosition = 0;
//    public float rightPosition = 0;

    public Artifact leftArtifact = Artifact.NONE;
    public Artifact rightArtifact = Artifact.NONE;

    @Override
    public void init(HardwareMap hardwareMap) {
        motorIntake = hardwareMap.get(DcMotorEx.class, "motorIntake");
        motorIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

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
        if (Controls.intakeOut(gamepad)) {
            motorIntake.setPower(-1.0F);
        } else if (Controls.intakeIn(gamepad)) {
            motorIntake.setPower(1.0F);
        } else {
            motorIntake.setPower(0.0F);
        }

        long now = System.currentTimeMillis();

        boolean closeLeft = Controls.intakeServoLeft(gamepad);
        if (closeLeft && !leftPressed) {
            leftActivated = now + 200;
            leftPressed = true;
        } else if (!closeLeft) {
            leftPressed = false;
        }

        boolean closeRight = Controls.intakeServoRight(gamepad);
        if (closeRight && !rightPressed) {
            rightActivated = now + 200;
            rightPressed = true;
        } else if (!closeRight) {
            rightPressed = false;
        }

        if (now < leftActivated) {
            servoLeft.setPosition(0.56); // left closed position
        } else {
            servoLeft.setPosition(0.3); // left open position
        }

        if (now < rightActivated) {
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

//        double leftDistance = colorLeft.getDistance(DistanceUnit.CM);
//        NormalizedRGBA leftColor = colorLeft.getNormalizedColors();
//        float[] leftHSV = new float[3];
//        Color.colorToHSV(leftColor.toColor(), leftHSV);
//        float leftHue = leftHSV[0];
//        if (leftDistance <= 10) {
//            if (leftHue >= 220 && leftHue <= 320) {
//                leftArtifact = Artifact.PURPLE;
//            } else if (leftHue >= 80 && leftHue <= 160) {
//                leftArtifact = Artifact.GREEN;
//            } else {
//                leftArtifact = Artifact.NONE;
//            }
//        } else {
//            leftArtifact = Artifact.NONE;
//        }
//
//        double rightDistance = colorRight.getDistance(DistanceUnit.CM);
//        NormalizedRGBA rightColor = colorRight.getNormalizedColors();
//        float[] rightHSV = new float[3];
//        Color.colorToHSV(rightColor.toColor(), rightHSV);
//        float rightHue = rightHSV[0];
//        if (rightDistance <= 10) {
//            if (rightHue >= 220 && rightHue <= 320) {
//                rightArtifact = Artifact.PURPLE;
//            } else if (rightHue >= 80 && rightHue <= 160) {
//                rightArtifact = Artifact.GREEN;
//            } else {
//                rightArtifact = Artifact.NONE;
//            }
//        } else {
//            rightArtifact = Artifact.NONE;
//        }

        // TODO: fix ts

//        double leftDistance = colorLeft.getDistance(DistanceUnit.CM);
//        float leftR = colorLeft.red();
//        float leftG = colorLeft.green();
//        float leftB = colorLeft.blue();
//        // fix the bum ahh color readings
//        // check chatgpt for ts
//        float[] leftHSV = new float[3];
//        Color.colorToHSV(leftColor.toColor(), leftHSV);
//        float leftHue = leftHSV[0];
//        if (leftDistance <= 10) {
//            if (leftHue >= 220 && leftHue <= 320) {
//                leftArtifact = Artifact.PURPLE;
//            } else if (leftHue >= 80 && leftHue <= 160) {
//                leftArtifact = Artifact.GREEN;
//            } else {
//                leftArtifact = Artifact.NONE;
//            }
//        } else {
//            leftArtifact = Artifact.NONE;
//        }
    }

    public void update(Telemetry telemetry) {
        telemetry.addData("l_r", colorLeft.red());
        telemetry.addData("l_g", colorLeft.green());
        telemetry.addData("l_b", colorLeft.blue());
        telemetry.addData("l_a", colorLeft.alpha());
        NormalizedRGBA leftColor = colorLeft.getNormalizedColors();
        telemetry.addData("l_r", leftColor.red);
        telemetry.addData("l_g", leftColor.green);
        telemetry.addData("l_b", leftColor.blue);
        telemetry.addData("l_a", leftColor.alpha);
        float[] leftHSV = new float[3];
        Color.colorToHSV(leftColor.toColor(), leftHSV);
        telemetry.addData("l_hsv", String.format("H:%.0f S:%.0f V:%.0f", leftHSV[0], leftHSV[1], leftHSV[2]));
        telemetry.addData("l_dist", colorLeft.getDistance(DistanceUnit.CM));
        telemetry.addData("l_artifact", leftArtifact);

        telemetry.addData("r_r", colorRight.red());
        telemetry.addData("r_g", colorRight.green());
        telemetry.addData("r_b", colorRight.blue());
        telemetry.addData("r_a", colorRight.alpha());
        NormalizedRGBA rightColor = colorRight.getNormalizedColors();
        telemetry.addData("r_r", rightColor.red);
        telemetry.addData("r_g", rightColor.green);
        telemetry.addData("r_b", rightColor.blue);
        telemetry.addData("r_a", rightColor.alpha);
        float[] rightHSV = new float[3];
        Color.colorToHSV(rightColor.toColor(), rightHSV);
        telemetry.addData("r_hsv", String.format("H:%.0f S:%.0f V:%.0f", rightHSV[0], rightHSV[1], rightHSV[2]));
        telemetry.addData("r_dist", colorRight.getDistance(DistanceUnit.CM));
        telemetry.addData("r_artifact", rightArtifact);

//        telemetry.addData("l_pos", servoLeft.getPosition());
//        telemetry.addData("r_pos", servoRight.getPosition());
    }

    @Override
    public void stop() {

    }
}
