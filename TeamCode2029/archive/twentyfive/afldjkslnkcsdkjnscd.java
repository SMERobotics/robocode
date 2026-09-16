//change back to 'package org.firstinspires.ftc.teamcode;' right below if it doesn't work, i changed it to 'package com.n0tasha4k.ftc.twentyfive;'
package com.n0tasha4k.ftc.twentyfive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "BlueCloseAuto")
public class afldjkslnkcsdkjnscd extends LinearOpMode {

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
        DcMotor intakey = hardwareMap.dcMotor.get("intakey");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "activeShooter");

        Servo finger = hardwareMap.servo.get("indexfinger");


        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                left.setPower(-1);
                right.setPower(1);
                sleep(725);
                left.setPower(0);
                right.setPower(0);
                shooter.setVelocity(1700);
                index.setPower(-1);

                sleep(2000);

                for (int i = 0; i < 3 && opModeIsActive(); ) {

                    if (shooter.getVelocity() > 1675) {
                        i++;
                        index.setPower(1);
                        sleep(1900);
                        index.setPower(-1);
                        intakey.setPower(-1);
                        sleep(2250);
                        intakey.setPower(0);
                    } else {
                        sleep(500);
                    }
                }

                left.setPower(-1);
                right.setPower(-1);
                sleep(200);
                left.setPower(1);
                right.setPower(-1);
                sleep(400);
                left.setPower(0);
                right.setPower(0);



                telemetry.addData("Left Pow", left.getPower());
                telemetry.addData("Right Pow", right.getPower());
                telemetry.addData("Shooter Power", shooter.getVelocity());
                telemetry.update();
                return;
            }
        }
    }
}