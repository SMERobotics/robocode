package com.technodot.ftc.twentyfive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Run Motor Test")
public class testmotor extends LinearOpMode {

    private DcMotor motorExtake;

    @Override
    public void runOpMode() {
        // Get the motor from the hardware map
        motorExtake = hardwareMap.get(DcMotor.class, "motorExtake");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            motorExtake.setPower(1);
        }

        // Stop the motor when the opmode is finished
        motorExtake.setPower(0);
    }
}
