package com.mech.ftc.twentyfive;

import com.mech.ftc.twentyfive.defaults.DriveTrain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//h
@TeleOp(name="DriveCode")
public class DriveCode extends OpMode {

    DriveTrain driveTrain = new DriveTrain();

    @Override
    public void init() {
        driveTrain.init(hardwareMap);
    }
    @Override
    public void loop() {
        driveTrain.drive(gamepad1);
    }
}
