package com.technodot.ftc.twentyfour;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
@Disabled
@TeleOp(name="BaboBotz", group="TechnoCode")
public class BaboBotz extends LinearOpMode {

    private DcMotor driveRight;
    private DcMotor driveLeft;
    private DcMotor slideMain;
    private DcMotor clawArm;
    private Servo clawMain;

    public float driveSpeedMultiplier = 1.0F;
    public long slideTimer = 0;
    public static final int driveRetreatTimer = 900;

    @Override
    public void runOpMode() {
        driveRight = hardwareMap.get(DcMotor.class, "driveRight");
        driveLeft = hardwareMap.get(DcMotor.class, "driveLeft");
        slideMain = hardwareMap.get(DcMotor.class, "slideMain");
        clawArm = hardwareMap.get(DcMotor.class, "clawArm");
        clawMain = hardwareMap.get(Servo.class, "clawMain");

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (gamepad1.right_trigger > 0.5) {
                    driveSpeedMultiplier = 0.5F;
                } else {
                    driveSpeedMultiplier = 1.0F;
                }

                driveLeft.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) * driveSpeedMultiplier);
                driveRight.setPower(-(gamepad1.left_stick_y + gamepad1.left_stick_x) * driveSpeedMultiplier);

                if (gamepad1.left_trigger > 0.5 && slideTimer == 0) {
                    slideTimer = System.currentTimeMillis();
                }

                if (System.currentTimeMillis() < slideTimer + driveRetreatTimer) {
                    driveLeft.setPower(0.8);
                    driveRight.setPower(-0.8);
                } else {
                    slideTimer = 0;
                }

                if (gamepad1.left_bumper) {
                    slideMain.setPower(-1);
                } else if (gamepad1.right_bumper) {
                    slideMain.setPower(1);
                } else {
                    slideMain.setPower(0);
                }

                if (gamepad1.y) {
                    clawArm.setPower(0.4);
                } else if (gamepad1.x) {
                    clawArm.setPower(-0.4);
                } else {
                    clawArm.setPower(0);
                }

                if (gamepad1.a) {
                    clawMain.setPosition(0);
                } else if (gamepad1.b) {
                    clawMain.setPosition(1);
                }

                telemetry.update();
            }
        }
    }
}