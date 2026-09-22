package com.cavalry.biobuzz;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(name = "Mecanum Test 9/22")


public class MecanumTest {

    public void runOpMode() {
        DcMotor FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        DcMotor FrontRight = hardwareMap.dcMotor.get("FrontRight");
        DcMotor RearLeft = hardwareMap.dcMotor.get("RearLeft");
        DcMotor RearRight = hardwareMap.dcMotor.get("RearRight");

        // 1. Read raw joystick values
        double forward = -gamepad1.left_stick_y;
        double strafe  = gamepad1.left_stick_x;
        double turn    = gamepad1.right_stick_x;

        // 2. Combine inputs for each wheel
        double frontLeftPower  = forward + strafe + turn;
        double backLeftPower   = forward - strafe + turn;
        double frontRightPower = forward - strafe - turn;
        double backRightPower  = forward + strafe - turn;

        // 3. Find the maximum absolute value to normalize power
        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))
        ));

        // 4. Divide powers by max if any value exceeds 1.0
        frontLeftPower  /= max;
        backLeftPower   /= max;
        frontRightPower /= max;
        backRightPower  /= max;

        // 5. Send calculated power to the motor objects
        FrontLeft.setPower(frontLeftPower);
        RearLeft.setPower(backLeftPower);
        FrontRight.setPower(frontRightPower);
        RearRight.setPower(backRightPower);

        telemetry.addData("FrontLeft Pow", FrontLeft.getPower());
        telemetry.addData("FrontRight Pow", FrontRight.getPower());
        telemetry.addData("RearLeft Pow", RearLeft.getPower());
        telemetry.addData("RearRight Pow", RearRight.getPower());

    }
}
