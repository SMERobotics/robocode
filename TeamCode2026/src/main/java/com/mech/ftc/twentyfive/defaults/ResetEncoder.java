package com.mech.ftc.twentyfive.defaults;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="Reset")
public class ResetEncoder extends OpMode {

    public DriveTrain driveTrain;

    @Override
    public void init() {
        driveTrain.init(hardwareMap);
        driveTrain.indexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {

    }
}
