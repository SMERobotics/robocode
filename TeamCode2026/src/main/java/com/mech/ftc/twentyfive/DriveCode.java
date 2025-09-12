package com.mech.ftc.twentyfive;

import com.mech.ftc.twentyfive.defaults.Camera;
import com.mech.ftc.twentyfive.defaults.DriveTrain;
import com.mech.ftc.twentyfive.defaults.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="DriveCode")
public class DriveCode extends OpMode {

    DriveTrain driveTrain = new DriveTrain();
    Velocity v = new Velocity(hardwareMap);
    Camera camera = new Camera();

    @Override
    public void init() {
        driveTrain.init(hardwareMap);

    }
    public void init_loop() {
        getTelemetry();
    }
    @Override
    public void loop() {
        driveTrain.drive(gamepad1);
        camera.start();
        getTelemetry();
    }

    public void getTelemetry() {
        double[] velocity = v.getRobotVelocity();
        telemetry.addData("Forward Velocity (m/s)", velocity[0]);
        telemetry.addData("Lateral Velocity (m/s)", velocity[1]);
        telemetry.update();
    }
}
