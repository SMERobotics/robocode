package com.mech.ftc.twentyfive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.mech.ftc.twentyfive.defaults.Camera;
import com.mech.ftc.twentyfive.defaults.Launcher;
import com.mech.ftc.twentyfive.defaults.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "BlueAuto")
public class BlueAuto extends LinearOpMode {

    public volatile int ID = -1;
    public volatile static int target;
    private final double TICKS_PER_REV = 28*27;

    public static double launchP = 50;
    public static double launchI = 0;
    public static double launchD = 50;
    public static double launchF = 13.53;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-45, -45, Math.toRadians(225));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        DcMotorEx launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        Servo kicker = hardwareMap.get(Servo.class, "servo");
        Servo wall = hardwareMap.get(Servo.class, "servoTwo");
        Camera camera = new Camera();
        camera.init(hardwareMap);
        camera.start();
        launchMotor.setVelocityPIDFCoefficients(launchP, launchI, launchD, launchF);

        double p = 0.01, i = 0.022, d = 0.000277;
        double f = 0.01;
        PIDController controller = new PIDController(p, i, d);

        DcMotorEx indexMotor = hardwareMap.get(DcMotorEx.class, "indexMotor");
        launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        indexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        Thread cameraThread = new Thread(() -> {
            while (opModeIsActive() && !isStopRequested() && ID == -1) {
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

            }
        });
        cameraThread.start();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeToLinearHeading(new Vector2d(-15, -15), 45)
                        .stopAndAdd(new Launch(launchMotor, frontLeft, frontRight, camera))
                        .stopAndAdd(new ParallelAction(new Index(0)))
                        .waitSeconds(1)
                        .stopAndAdd(new Wall(wall, 0))
                        .waitSeconds(.05)
                        .stopAndAdd(new Kick(kicker, .25))
                        .waitSeconds(1)
                        .stopAndAdd(new Kick(kicker, 0))
                        .waitSeconds(.5)
                        .stopAndAdd(new ParallelAction(new Index(230)))
                        .waitSeconds(.3)
                        .stopAndAdd(new Kick(kicker, .25))
                        .waitSeconds(.5)
                        .stopAndAdd(new Kick(kicker, 0))
                        .waitSeconds(.5)
                        .stopAndAdd(new ParallelAction(new Index(460)))
                        .waitSeconds(.5)
                        .stopAndAdd(new Kick(kicker, .25))
                        .waitSeconds(.5)
                        .stopAndAdd(new Wall(wall, .4))
                        .stopAndAdd(new Kick(kicker, 0))
                        .stopAndAdd(new LaunchZero(launchMotor))
                        .strafeToLinearHeading(new Vector2d(-5, -30), Math.toRadians(270))
                        .waitSeconds(100)
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

        public LaunchZero(DcMotorEx launcherMotor) {
            this.launcherMotor = launcherMotor;


        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            launcherMotor.setVelocity(0);
            return false;

        }
    }
}