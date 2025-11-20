//very stable version but not the best; no PID
package com.n0tasha4k.ftc.twentyfive;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Tank the Shooter")
public class nutele extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotor left = hardwareMap.dcMotor.get("left");
        DcMotor right = hardwareMap.dcMotor.get("right");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "activeShooter");
        DcMotorEx index = hardwareMap.get(DcMotorEx.class, "index");
        DcMotor intake = hardwareMap.dcMotor.get("intakey");


        Servo finger = hardwareMap.servo.get("indexfinger");

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        index.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //try removing if motor resets to a different position after being moved, this sets whatever position it starts on to 0

        left.setDirection(DcMotor.Direction.REVERSE);

        boolean isShooterOn = false;
        boolean isShooterHyper = false;

        double hypervelocity = 0;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
//control point for balls; out when motor isn't fast enough and will reset to position to get out of way when you are done spinning
            if (gamepad1.a) {
                index.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                if (shooter.getVelocity() > 1700) {
                    index.setPower(1);
                } else {
                    index.setPower(-1);
                }
            } else {
                index.setPower(0);
                index.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                index.setTargetPosition(0); //set motor direction in reverse if it returns to position in the unperferred direction(might not work just an idea)
            }

//toggle for hyper mode for far shooting
            if (gamepad1.bWasPressed()) {
                isShooterHyper = !isShooterHyper;
            }
            if (isShooterHyper) {
                hypervelocity = 525;
            } else {
                hypervelocity = 0;
            }

//toggle for the shooter being on in general
            if (gamepad1.yWasPressed()) {
                isShooterOn = !isShooterOn;
            }
            if (isShooterOn) {
                shooter.setVelocity(1775+hypervelocity);
            } else {
                shooter.setVelocity(0);
            }
//intake belt to bring balls up
            if (gamepad1.right_bumper) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }
//data
            telemetry.addData("Left Pow", left.getPower());
            telemetry.addData("Right Pow", right.getPower());
            telemetry.addData("Shooter Power", shooter.getVelocity());
            telemetry.addData("Intake Power", intake.getPower());
            telemetry.addData("Shooter Status", isShooterOn ? "On" : "Off"); // Shows toggle state on Driver Hub\
            telemetry.addData("Hyper Status", isShooterHyper ? "On" : "Off"); // Shows toggle state on Driver Hub\
            telemetry.update();

//tank drive
            left.setPower(gamepad1.right_stick_y);
            right.setPower(gamepad1.left_stick_y/1.2773475);
        }

    }
}