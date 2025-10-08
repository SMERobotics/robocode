package com.n0tasha4k.ftc.twentyfive;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Left(Right[another problem for later])MotorTest (Troubleshooting; not a real teleop)")
public class TestLeftMotor extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotor left = hardwareMap.dcMotor.get("left");
        DcMotor right = hardwareMap.dcMotor.get("right");

        left.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
//            shooter.setPower(1);
            float gamepadX;
            float gamepadY;

            if (gamepad1.a) {
                right.setPower(1);
            } else {
                right.setPower(0);

                if (gamepad1.y) {
                    right.setPower(.45);
                } else {
                    right.setPower(0);

                    telemetry.addData("Left Pow", left.getPower());
                    telemetry.addData("Right Pow", right.getPower());
                    telemetry.update();

                    left.setPower(-gamepad1.left_stick_y);
                    right.setPower(-gamepad1.right_stick_y);
                }

            }
        }
    }
}