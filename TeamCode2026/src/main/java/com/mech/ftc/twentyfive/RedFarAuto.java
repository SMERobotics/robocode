package com.mech.ftc.twentyfive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.arcrobotics.ftclib.controller.PIDController;
import com.mech.ftc.twentyfive.defaults.Camera;
import com.mech.ftc.twentyfive.defaults.Launcher;
import com.mech.ftc.twentyfive.defaults.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "RedFarAuto")
public class RedFarAuto extends LinearOpMode {

    public volatile int ID = -1;
    public volatile static int target;
    private final double TICKS_PER_REV = 28*27;

    public static double launchP = 50;
    public static double launchI = 0;
    public static double launchD = 50;
    public static double launchF = 13.53;

    public static double rotateDistance = 4;
    public static boolean active = true;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        DcMotorEx launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        Servo kicker = hardwareMap.get(Servo.class, "servo");
        Servo wall = hardwareMap.get(Servo.class, "servoTwo");
        Camera camera = new Camera();
        camera.init(hardwareMap);
        launchMotor.setVelocityPIDFCoefficients(launchP, launchI, launchD, launchF);
        ColorRangeSensor colorSensor = hardwareMap.get(ColorRangeSensor.class, "colorSensor");

        double p = 0.01, i = 0.08, d = 0.000277;
        double f = 0;
        PIDController controller = new PIDController(p, i, d);

        DcMotorEx indexMotor = hardwareMap.get(DcMotorEx.class, "indexMotor");
        launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        indexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        Thread cameraThread = new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                int tag = camera.TagID();
                telemetry.addData("Tag", ID);
                telemetry.update();
                if (tag == 21 || tag == 22 || tag == 23) {
                    ID = tag;
                }
                controller.setPID(p,i,d);
                int indexPosition = indexMotor.getCurrentPosition();
                double pid = controller.calculate(indexPosition, target);
                double ff = Math.cos(Math.toRadians(target / TICKS_PER_REV * 360)) * f;

                double power = pid + ff;

                indexMotor.setPower(power);

                if (colorSensor.getDistance(DistanceUnit.CM) < rotateDistance && intakeMotor.getPower() < 0 && active) {
                    target += 226;
                    active = false;
                }
                if (colorSensor.getDistance(DistanceUnit.CM) >= 4.9) {
                    active = true;
                }
            }
        });
        cameraThread.start();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeToLinearHeading(new Vector2d(0,14), Math.toRadians(245))
                        .stopAndAdd(new LaunchZero(launchMotor, 1970))
                        .stopAndAdd(new Index(0))
                        .stopAndAdd(new Wall(wall, 0))
                        .waitSeconds(.05)
                        .stopAndAdd(new Kick(kicker, .25))
                        .waitSeconds(1.3)
                        .stopAndAdd(new Kick(kicker, 0))
                        .waitSeconds(.7)
                        .stopAndAdd(new Index(226))
                        .waitSeconds(.7)
                        .stopAndAdd(new Kick(kicker, .25))
                        .waitSeconds(.7)
                        .stopAndAdd(new Kick(kicker, 0))
                        .waitSeconds(.7)
                        .stopAndAdd(new Index(452))
                        .waitSeconds(.7)
                        .stopAndAdd(new Kick(kicker, .25))
                        .waitSeconds(.7)
                        .stopAndAdd(new Wall(wall, .4))
                        .stopAndAdd(new Kick(kicker, 0))
                        .waitSeconds(1)
                        .stopAndAdd(new LaunchZero(launchMotor, 1970))
                        .stopAndAdd(new IntakePower(intakeMotor, -1))
                        .strafeToLinearHeading(new Vector2d(0, 27), Math.toRadians(0))
                        .stopAndAdd(new Index(113))
                        .strafeToLinearHeading(new Vector2d(48, 27), Math.toRadians(0))
                        .stopAndAdd(new IntakePower(intakeMotor, 0))
                        .stopAndAdd(new Index(0))
                        .strafeToLinearHeading(new Vector2d(0,14), Math.toRadians(245))
                        .stopAndAdd(new Launch(launchMotor, frontLeft, frontRight, camera))
                        .stopAndAdd(new Index(0))
                        .stopAndAdd(new Wall(wall, 0))
                        .waitSeconds(.05)
                        .stopAndAdd(new Kick(kicker, .25))
                        .waitSeconds(1.3)
                        .stopAndAdd(new Kick(kicker, 0))
                        .waitSeconds(.7)
                        .stopAndAdd(new Index(226))
                        .waitSeconds(.7)
                        .stopAndAdd(new Kick(kicker, .25))
                        .waitSeconds(.7)
                        .stopAndAdd(new Kick(kicker, 0))
                        .waitSeconds(.7)
                        .stopAndAdd(new Index(452))
                        .waitSeconds(.7)
                        .stopAndAdd(new Kick(kicker, .25))
                        .waitSeconds(.7)
                        .stopAndAdd(new Wall(wall, .4))
                        .stopAndAdd(new Kick(kicker, 0))
                        .waitSeconds(1)
                        .stopAndAdd(new LaunchZero(launchMotor,0))
                        .strafeToLinearHeading(new Vector2d(0, 25), Math.toRadians(270))
                        .build());

    }

    public class Kick implements Action {
        Servo kick;
        double pos;

        public Kick(Servo kick, double pos) {
            this.kick = kick;
            this.pos = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            kick.setPosition(pos);
            return false;

        }
    }

    public class Wall implements Action {
        Servo wall;
        double pos;

        public Wall(Servo wall, double pos) {
            this.wall = wall;
            this.pos = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wall.setPosition(pos);
            return false;

        }
    }

    public class Launch implements Action {
        DcMotorEx launcherMotor;
        DcMotorEx leftMotor;
        DcMotorEx rightMotor;
        Velocity v;
        Launcher launcher;
        Camera camera;
        boolean fire = true;

        public Launch(DcMotorEx launcherMotor, DcMotorEx leftMotor, DcMotorEx rightMotor, Camera camera) {
            this.launcherMotor = launcherMotor;
            this.leftMotor = leftMotor;
            this.rightMotor = rightMotor;
            v = new Velocity(leftMotor, rightMotor);
            launcher = new Launcher(launcherMotor, v);
            this.camera = camera;


        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            boolean tagVisible = camera.TagID() != -1;
            double distanceM = camera.getTagDistance();
            launcher.setEnabled(fire && tagVisible);
            launcher.update(distanceM);
            launcherMotor.setVelocity(launcher.getTargetVelocity());
            return distanceM == 0;

        }
    }

    public class Index implements Action {
        int targetPosition;

        public Index(int targetPosition) {
            this.targetPosition = targetPosition;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            target = targetPosition;
            return false;

        }
    }
    public class LaunchZero implements Action {
        DcMotorEx launcherMotor;
        double speed;

        public LaunchZero(DcMotorEx launcherMotor, double speed) {
            this.launcherMotor = launcherMotor;
            this.speed = speed;


        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            launcherMotor.setVelocity(speed);
            return false;

        }
    }
    public class IntakePower implements Action {
        DcMotorEx intakeMotor;
        int power;

        public IntakePower(DcMotorEx intakeMotor, int power) {
            this.intakeMotor = intakeMotor;
            this.power = power;


        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeMotor.setPower(power);
            return false;
        }
    }
}