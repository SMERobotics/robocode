package com.mech.ftc.twentyfive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.mech.ftc.twentyfive.defaults.Camera;
import com.mech.ftc.twentyfive.defaults.DriveTrain;
import com.mech.ftc.twentyfive.defaults.Launcher;
import com.mech.ftc.twentyfive.defaults.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="DriveCode")
public class DriveCode extends OpMode {

    DriveTrain driveTrain = new DriveTrain();
    Velocity v;
    Launcher launcher;
    Camera camera = new Camera();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    com.acmerobotics.dashboard.telemetry.TelemetryPacket packet = new com.acmerobotics.dashboard.telemetry.TelemetryPacket();

    @Override
    public void init() {
        driveTrain.init(hardwareMap);
        camera.init(hardwareMap);
        camera.start(); // start once
        dashboard.startCameraStream(camera.visionPortal, 30);

        v = new Velocity(driveTrain.frontLeft, driveTrain.frontRight);
        launcher = new Launcher(driveTrain.launchMotor, v);
    }

    @Override
    public void init_loop() {
        getTelemetry();
    }

    @Override
    public void loop() {
        driveTrain.drive(gamepad1);

        boolean fire = gamepad1.right_trigger > 0.5;
        boolean tagVisible = camera.TagID() != -1;
        double distanceM = camera.getTagDistance();

        launcher.setEnabled(fire && tagVisible);
        launcher.update(distanceM);

        if (gamepad1.a) {
            driveTrain.intake.setPower(1);
        } else {
            driveTrain.intake.setPower(0);
        }

        getTelemetry();
    }

    public void getTelemetry() {
        telemetry.addData("Forward Velocity (m/s): ", v.getForwardVelocity());
        telemetry.addData("Lateral Velocity (m/s): ", v.getLateralVelocity());
        telemetry.addData("ID", camera.TagID() + " Tag Distance (m) " +  camera.getTagDistance());
        telemetry.addData("Launch Target Power", launcher.getTargetPower());
        telemetry.addData("Launch Applied Power", driveTrain.launchMotor.getPower());
        packet.put("ID", camera.TagID());
        packet.put("Tag Distance (m)", camera.getTagDistance());
        packet.put("Launch Target Power", launcher.getTargetPower());
        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }
}