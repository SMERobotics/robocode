//change back to 'package org.firstinspires.ftc.teamcode;' right below if it doesn't work, i changed it to 'package com.n0tasha4k.ftc.twentyfive;'
package com.n0tasha4k.ftc.twentyfive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "ModularAutoMaybe")
public class TESTAUTO extends LinearOpMode {

    private DcMotor left;
    private DcMotor index;
    private DcMotor right;

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        left = hardwareMap.get(DcMotor.class, "left");
        index = hardwareMap.get(DcMotor.class, "index");
        right = hardwareMap.get(DcMotor.class, "right");
        DcMotor index = hardwareMap.dcMotor.get("index");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "activeShooter");


        waitForStart();

        double TARGET_VELOCITY = 1650;

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                left.setPower(-1);
                right.setPower((1 / 1.2773475));
                sleep(1425);
                left.setPower(0);
                right.setPower(0);
                shooter.setVelocity(TARGET_VELOCITY);

                sleep(2000);

                for (int i = 0; i < 3; i++) {

                    waitForShooterSpeed(TARGET_VELOCITY);

                    shootOne();
                }

                shooter.setVelocity(0);

                telemetry.addData("Left Pow", left.getPower());
                telemetry.addData("Right Pow", right.getPower());
                telemetry.addData("Shooter Power", shooter.getVelocity());
                telemetry.update();
            }
        }
    }


    private void waitForShooterSpeed(double targetSpeed) {
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "activeShooter");

        double tolerance = 100;

        while (opModeIsActive()) {
            double currentSpeed = shooter.getVelocity();
            if (Math.abs(currentSpeed - targetSpeed) < tolerance) {
                break;
            }
        }
    }


    //where you would add a if statement to set the code to run this when the sensor is tripped on the placement of the ball
    private void shootOne() {
        index.setPower(1);
        sleep(5000);
        index.setPower(0);
    }
}