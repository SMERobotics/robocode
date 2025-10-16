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
    //Velocity v = new Velocity(hardwareMap);
    Camera camera = new Camera();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    com.acmerobotics.dashboard.telemetry.TelemetryPacket packet = new com.acmerobotics.dashboard.telemetry.TelemetryPacket();

    @Override
    public void init() {
        driveTrain.init(hardwareMap);
        camera.init(hardwareMap);
        dashboard.startCameraStream(camera.visionPortal, 30);
    }
    @Override
    public void init_loop() {
        getTelemetry();
    }
    @Override
    public void loop() {
        driveTrain.drive(gamepad1);
        camera.start();
        getTelemetry();
        //launcher.launch(gamepad1);
    }

    public void getTelemetry() {
        //telemetry.addData("Forward Velocity (m/s): ", v.getForwardVelocity());
        //telemetry.addData("Lateral Velocity (m/s): ", v.getLateralVelocity());
        //telemetry.addData("Robot Velocity (m/s): ", Math.sqrt((v.getForwardVelocity() * v.getForwardVelocity()) + v.getLateralVelocity() * v.getLateralVelocity()));
        telemetry.addData("ID", camera.TagID() + " Tag Distance (m) " +  camera.getTagDistance());
        packet.put("ID", camera.TagID());
        packet.put("Tag Distance (m)", camera.getTagDistance());
        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }
}
