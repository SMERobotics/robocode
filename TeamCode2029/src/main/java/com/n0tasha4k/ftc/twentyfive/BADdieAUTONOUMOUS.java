package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "BADdieAUTONOMOUS (Blocks to Java)")
public class BADdieAUTONOUMOUS extends LinearOpMode {

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

        Servo finger = hardwareMap.servo.get("indexfinger");


        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                left.setPower(-1);
                right.setPower((1 / 1.2773475));
                sleep(2000);
                left.setPower(0);
                right.setPower(0);
                shooter.setVelocity(1800);

                if (shooter.getVelocity() > 1650) {
                    sleep(1000);
                index.setPower(-1);
                finger.setPosition(1);
                sleep(5000);
                index.setPower(0);
                finger.setPosition(0);
                break;
            } else {
                    sleep(1000);
                }
                if (shooter.getVelocity() > 1650) {
                    sleep(1000);
                    index.setPower(-1);
                    finger.setPosition(1);
                    sleep(5000);
                    index.setPower(0);
                    finger.setPosition(0);
                    break;
                } else {
                    sleep(1000);
                }
                if (shooter.getVelocity() > 1650) {
                    sleep(1000);
                    index.setPower(-1);
                    finger.setPosition(1);
                    sleep(5000);
                    index.setPower(0);
                    finger.setPosition(0);
                    break;
                } else {
                    sleep(1000);
                }


                telemetry.addData("Left Pow", left.getPower());
                telemetry.addData("Right Pow", right.getPower());
                telemetry.addData("Shooter Power", shooter.getVelocity());
                telemetry.update();
                break;
            }
        }
    }
}