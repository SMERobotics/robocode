package com.technodot.ftc.twentyfour;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.technodot.ftc.twentyfour.maytag.MayTag;
import com.technodot.ftc.twentyfour.robocore.DeviceClaw;
import com.technodot.ftc.twentyfour.robocore.DeviceDrive;
import com.technodot.ftc.twentyfour.robocore.DeviceSlide;

@TeleOp(name="BaboOS", group="TechnoCode")
public class BaboOS extends LinearOpMode {

    // controllers
    DeviceDrive deviceDrive;
    DeviceClaw deviceClaw;
    DeviceSlide deviceSlide;
    MayTag mayTag;

    // gamepad input
    float leftX;
    float leftY;
    float rightX;
    float rightY;
    boolean clawOpenPressed;
    boolean clawClosePressed;
    boolean armClosePressed;
    boolean armOpenPressed;
    boolean slideExtendPressed;
    boolean slideRetractPressed;
    boolean preciseMode;
    boolean armCloseForcePressed;
    boolean slideExtendForcePressed;
    boolean slideRetractForcePressed;
    boolean cameraVisionPortalStartPressed;
    boolean cameraVisionPortalStopPressed;

    @Override
    public void runOpMode() {
        // initialize devices
        deviceDrive = new DeviceDrive(hardwareMap, Configuration.driveTrainType, Configuration.driveControlType);
        deviceClaw = new DeviceClaw(hardwareMap);
        deviceSlide = new DeviceSlide(hardwareMap);

        // initialize MayTag library
        if (Configuration.ENABLE_TELEOP_MAYTAG) {
            mayTag = new MayTag(hardwareMap, telemetry);
        }


        // finish initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            // read input
            leftX = gamepad1.left_stick_x;
            leftY = gamepad1.left_stick_y;
            rightX = gamepad1.right_stick_x;
            rightY = gamepad1.right_stick_y;
            clawOpenPressed = gamepad1.a;
            clawClosePressed = gamepad1.b;
            armOpenPressed = gamepad1.x;
            armClosePressed = gamepad1.y;
            slideExtendPressed = gamepad1.left_bumper;
            slideRetractPressed = gamepad1.right_bumper;
            preciseMode = gamepad1.left_trigger > 0.5;
            armCloseForcePressed = gamepad1.right_trigger > 0.5;
            slideExtendForcePressed = gamepad1.dpad_up;
            slideRetractForcePressed = gamepad1.dpad_down;
            cameraVisionPortalStartPressed = gamepad1.dpad_left;
            cameraVisionPortalStopPressed = gamepad1.dpad_right;

            deviceDrive.updatePreciseMode(preciseMode);
            deviceClaw.updatePreciseMode(preciseMode);
            deviceSlide.updatePreciseMode(preciseMode);

            deviceDrive.updateDrive(leftX, leftY, rightX, rightY);
            deviceClaw.updateArm(armOpenPressed, armClosePressed);
            deviceClaw.updateArmForce(armCloseForcePressed);
            deviceClaw.updateClaw(clawOpenPressed, clawClosePressed);
            deviceSlide.updateSlide(slideExtendPressed, slideRetractPressed, slideExtendForcePressed, slideRetractForcePressed);
            deviceSlide.updateSlideForce(slideExtendPressed, slideRetractPressed, slideExtendForcePressed, slideRetractForcePressed);

            if (Configuration.ENABLE_TELEOP_MAYTAG) {
                mayTag.updateCamera(cameraVisionPortalStartPressed, cameraVisionPortalStopPressed);
                mayTag.update();
            }

            telemetry.addData("armEncoder", deviceClaw.clawArm.getCurrentPosition());
            telemetry.update();
        }

        if (Configuration.ENABLE_TELEOP_MAYTAG) {
            mayTag.close();
        }

        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
}