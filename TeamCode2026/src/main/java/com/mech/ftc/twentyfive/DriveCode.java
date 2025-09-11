package com.mech.ftc.twentyfive;

import com.mech.ftc.twentyfive.defaults.Camera;
import com.mech.ftc.twentyfive.defaults.DriveTrain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="DriveCode")
public class DriveCode extends OpMode {

    DriveTrain driveTrain = new DriveTrain();
    Camera camera = new Camera();

    @Override
    public void init() {
        driveTrain.init(hardwareMap);

    }
    @Override
    public void loop() {
        driveTrain.drive(gamepad1);
        camera.start();
    }
}
