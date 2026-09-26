package com.cavalry.biobuzz;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(name = "BioBuzz TeleOp")
public class MecanumMain extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        DcMotor FrontRight = hardwareMap.dcMotor.get("FrontRight");
        DcMotor RearLeft = hardwareMap.dcMotor.get("RearLeft");
        DcMotor RearRight = hardwareMap.dcMotor.get("RearRight");
        DcMotor intake = hardwareMap.dcMotor.get("intakey");


        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        RearLeft.setDirection(DcMotor.Direction.REVERSE);
        RearRight.setDirection(DcMotor.Direction.REVERSE);

        boolean isIntakeOn = false;

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {

            if (gamepad1.aWasPressed()) {
                isIntakeOn = !isIntakeOn;
            }
            if (isIntakeOn) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

            // Calculate powers using the MecanumDrive class
            MecanumDrive.WheelPowers powers = MecanumDrive.calculatePowers(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            // Send calculated power to the motor objects
            FrontLeft.setPower(powers.frontLeft);
            RearLeft.setPower(powers.rearLeft);
            FrontRight.setPower(powers.frontRight);
            RearRight.setPower(powers.rearRight);

            telemetry.addData("FrontLeft Pow", FrontLeft.getPower());
            telemetry.addData("FrontRight Pow", FrontRight.getPower());
            telemetry.addData("RearLeft Pow", RearLeft.getPower());
            telemetry.addData("RearRight Pow", RearRight.getPower());
            telemetry.update();
        }
    }
}
