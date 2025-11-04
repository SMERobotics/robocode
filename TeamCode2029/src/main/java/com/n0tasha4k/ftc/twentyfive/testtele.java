//very stable version but not the best; no PID
package com.n0tasha4k.ftc.twentyfive;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Tank/1.1 a shooter in the building Android Studio")
public class testtele extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotor left = hardwareMap.dcMotor.get("left");
        DcMotor right = hardwareMap.dcMotor.get("right");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "activeShooter");
        DcMotor index = hardwareMap.dcMotor.get("index");
        DcMotor intake = hardwareMap.dcMotor.get("intakey");


        Servo finger = hardwareMap.servo.get("indexfinger");

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        left.setDirection(DcMotor.Direction.REVERSE);

        boolean isShooterOn = false;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
//            shooter.setPower(1);

            if (gamepad1.a) {
                finger.setPosition(1);
                if (shooter.getVelocity() > 1650) {
                    index.setPower(-1);
                } else {
                    index.setPower(0);
                }
            } else {
                finger.setPosition(0);
                index.setPower(0);
            }

            if (gamepad1.yWasPressed()) {
                isShooterOn = !isShooterOn;
            }
            if (isShooterOn) {
                shooter.setVelocity(1800);
            } else {
                shooter.setVelocity(0);
            }

            if (gamepad1.right_bumper) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

            telemetry.addData("Left Pow", left.getPower());
            telemetry.addData("Right Pow", right.getPower());
            telemetry.addData("Shooter Power", shooter.getVelocity());
            telemetry.addData("Intake Power", intake.getPower());
            telemetry.addData("Shooter Status", isShooterOn ? "On" : "Off"); // Shows toggle state on Driver Hub\
            telemetry.update();


            left.setPower(gamepad1.right_stick_y);
            right.setPower(gamepad1.left_stick_y/1.2773475);
        }

    }
}
