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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(name="DriveCode")
@SuppressWarnings("FieldCanBeLocal")
public class DriveCode extends OpMode {
    private int x = 1;
    private int y = 0;
    private int rotate = 0;

    private boolean active;

    private PIDController controller;

    public static double p = 0.01, i = 0.08, d = 0.000377;
    public static double f = 0.01;

    public static int targetPosition = 0;

    private final double TICKS_PER_REV = 28*27;

    private boolean prevRightBumper = false;
    private boolean prevLeftBumper = false;

    public static double rotateDistance = 4;

    private PIDController headingController;
    public static double headingP = 0.03;
    public static double headingI = 0.0;
    public static double headingD = 0.001;
    public static double headingToleranceDeg = 0;
    public static double headingMaxPower = 0.6;

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
        dashboard.startCameraStream(camera.visionPortal, 30);

        v = new Velocity(driveTrain.frontLeft, driveTrain.frontRight);
        launcher = new Launcher(driveTrain.launchMotor, v);

        controller = new PIDController(p, i, d);

        headingController = new PIDController(headingP, headingI, headingD);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void init_loop() { getTelemetry(); }

    @Override
    public void loop() {
        boolean fire = gamepad1.right_trigger > 0.5;
        boolean tagVisible = camera.TagID() != -1;
        double distanceM = camera.getTagDistance();

        launcher.setEnabled(fire && tagVisible);

        if (gamepad1.b && y == 0) {
            switch (x) {
                case 1:
                    driveTrain.intake.setPower(1);
                    x = 2;
                    y = 1;
                    break;
                case 2:
                    driveTrain.intake.setPower(0);
                    x = 1;
                    y = 1;
                    break;
                default:
                    break;
            }
        }
        else if (!gamepad1.b) {
            y = 0;
        }

        if (gamepad1.a) {
            driveTrain.kicker.setPosition(0.25);
            driveTrain.wall.setPosition(0);
        } else {
            driveTrain.kicker.setPosition(-.5);
            driveTrain.wall.setPosition(.4);
            if (gamepad1.dpad_down) {
                driveTrain.wall.setPosition(0);
            }
        }

        headingController = new PIDController(headingP, headingI, headingD);

        if (gamepad1.right_trigger > 0.5) {
            double bearing = camera.getTagBearing();
            if (Math.abs(bearing) > headingToleranceDeg) {
                double turn = headingController.calculate(bearing, 0);
                turn = Math.max(Math.min(turn, headingMaxPower), -headingMaxPower);
                driveTrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, (float) turn);
            } else {
                driveTrain.drive(gamepad1);
            }
        } else {
            driveTrain.drive(gamepad1);
        }

        if (gamepad1.left_trigger > 0.5) {
            driveTrain.launchMotor.setPower(1);
        }
        else {
            launcher.update(distanceM);
        }
        if (gamepad1.dpad_up) {
            driveTrain.indexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveTrain.indexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (gamepad1.x) {
            driveTrain.fieldOrientedVar(hardwareMap);
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
            targetPosition += 113;
        } else if (lbPressedOnce) {
            targetPosition -= 113;
        }
        if (targetPosition > 566 || targetPosition < -566) {
            if (driveTrain.intake.getPower() == 0) {
                targetPosition = 0;
                rotate = 0;
            }
        }
        if (driveTrain.colorSensor.getDistance(DistanceUnit.CM) < rotateDistance && driveTrain.intake.getPower() > 0 && active && rotate < 3) {
            if (targetPosition >= 0) {
                targetPosition += 226;
            } else {
                targetPosition -= 226;
            }
            rotate++;
            active = false;
        }
        if (driveTrain.colorSensor.getDistance(DistanceUnit.CM) >= 6) {
            active = true;
        }
        prevRightBumper = rb;
        prevLeftBumper = lb;
        getTelemetry();
    }

    public void getTelemetry() {
        telemetry.addData("Forward Velocity (m/s): ", v.getForwardVelocity());
        telemetry.addData("Lateral Velocity (m/s): ", v.getLateralVelocity());
        telemetry.addData("ID", camera.TagID() + " Tag Distance (m) " +  camera.getTagDistance());
        telemetry.addData("Index Current Pos", driveTrain.indexMotor.getCurrentPosition());
        telemetry.addData("Target", targetPosition);
        telemetry.addData("launch velocity", driveTrain.launchMotor.getVelocity());
        telemetry.addData("target Velocity", launcher.getTargetVelocity());
        telemetry.addData("Color", driveTrain.colorSensor.getNormalizedColors().toColor());
        telemetry.addData("Ball Distance from Sensor(cm)", driveTrain.colorSensor.getDistance(DistanceUnit.CM));
        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }
}