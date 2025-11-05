package com.mech.ftc.twentyfive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.mech.ftc.twentyfive.defaults.Camera;
import com.mech.ftc.twentyfive.defaults.DriveTrain;
import com.mech.ftc.twentyfive.defaults.Launcher;
import com.mech.ftc.twentyfive.defaults.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name="DriveCode")
@SuppressWarnings("FieldCanBeLocal")
public class DriveCode extends OpMode {

    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int targetPosition = 0;

    private final double TICKS_PER_REV = 28*3;

    private boolean prevRightBumper = false;
    private boolean prevLeftBumper = false;

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
        camera.start();
        dashboard.startCameraStream(camera.visionPortal, 30);

        v = new Velocity(driveTrain.frontLeft, driveTrain.frontRight);
        launcher = new Launcher(driveTrain.launchMotor, v);
        driveTrain.indexMotor.setTargetPosition(0);

        controller = new PIDController(p, i, d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void init_loop() { getTelemetry(); }

    @Override
    public void loop() {

        driveTrain.drive(gamepad1);

        boolean fire = gamepad1.right_trigger > 0.5;
        boolean tagVisible = camera.TagID() != -1;
        double distanceM = camera.getTagDistance();

        launcher.setEnabled(fire && tagVisible);
        launcher.update(distanceM);

        if (gamepad1.b) {
            driveTrain.intake.setPower(1);
        } else {
            driveTrain.intake.setPower(0);
        }
        if (gamepad1.a) {
            driveTrain.kicker.setPosition(0.6);
        } else {
            driveTrain.kicker.setPosition(0);
        }

        boolean rb = gamepad1.right_bumper;
        boolean lb = gamepad1.left_bumper;
        boolean rbPressedOnce = rb && !prevRightBumper;
        boolean lbPressedOnce = lb && !prevLeftBumper;

        controller.setPID(p,i,d);
        int indexPosition = driveTrain.indexMotor.getCurrentPosition();
        double pid = controller.calculate(indexPosition, targetPosition);
        double ff = Math.cos(Math.toRadians(targetPosition / TICKS_PER_REV * 360)) * f;

        double power = pid + ff;

        driveTrain.indexMotor.setPower(power);

        if (rbPressedOnce) {
            targetPosition += 14;
        } else if (lbPressedOnce) {
            targetPosition -= 14;
        }

        prevRightBumper = rb;
        prevLeftBumper = lb;
        getTelemetry();
    }

    public void getTelemetry() {
        telemetry.addData("Forward Velocity (m/s): ", v.getForwardVelocity());
        telemetry.addData("Lateral Velocity (m/s): ", v.getLateralVelocity());
        telemetry.addData("ID", camera.TagID() + " Tag Distance (m) " +  camera.getTagDistance());
        telemetry.addData("Launch Target Power", launcher.getTargetPower());
        telemetry.addData("Launch Applied Power", driveTrain.launchMotor.getPower());
        telemetry.addData("Index Target Pos", driveTrain.indexMotor.getTargetPosition());
        telemetry.addData("Index Current Pos", driveTrain.indexMotor.getCurrentPosition());
        telemetry.addData("Index Power", driveTrain.indexMotor.getPower());
        telemetry.addData("launch velocity", driveTrain.launchMotor.getVelocity());
        packet.put("ID", camera.TagID());
        packet.put("Tag Distance (m)", camera.getTagDistance());
        packet.put("Launch Target Power", launcher.getTargetPower());
        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }
}