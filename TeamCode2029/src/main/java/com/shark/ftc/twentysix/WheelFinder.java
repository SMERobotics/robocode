package com.shark.ftc.twentysix;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@Autonomous(name = "Wheel Testing")
public class WheelFinder extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        DcMotor FrontRight = hardwareMap.dcMotor.get("FrontRight");
        DcMotor RearLeft = hardwareMap.dcMotor.get("RearLeft");
        DcMotor RearRight = hardwareMap.dcMotor.get("RearRight");

        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        RearLeft.setDirection(DcMotor.Direction.REVERSE);
        RearRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {

            FrontLeft.setPower(1);
            sleep(1000);
            FrontLeft.setPower(0);
            FrontRight.setPower(1);
            sleep(1000);
            FrontRight.setPower(0);
            RearLeft.setPower(1);
            sleep(1000);
            RearLeft.setPower(0);
            RearRight.setPower(1);
            sleep(1000);
            RearRight.setPower(0);


            telemetry.addData("FrontLeft Pow", FrontLeft.getPower());
            telemetry.addData("FrontRight Pow", FrontRight.getPower());
            telemetry.addData("RearLeft Pow", RearLeft.getPower());
            telemetry.addData("RearRight Pow", RearRight.getPower());
            telemetry.update();
        }
    }
}
