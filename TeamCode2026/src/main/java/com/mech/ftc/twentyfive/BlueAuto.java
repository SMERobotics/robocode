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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "BlueAuto")
public class BlueAuto extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-45, -45, Math.toRadians(45));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        DcMotorEx launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        Servo kicker = hardwareMap.get(Servo.class, "servo");
        Servo wall = hardwareMap.get(Servo.class, "servoTwo");

        double p = 0.01, i = 0.022, d = 0.000277;
        double f = 0.01;
        DcMotorEx indexMotor = hardwareMap.get(DcMotorEx.class, "indexMotor");
        launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        indexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeToConstantHeading(new Vector2d(-15,-15))
                        .stopAndAdd(new Launch(launchMotor, frontLeft, frontRight, hardwareMap))
                        .stopAndAdd(new ParallelAction(new Index(indexMotor, p, i, d, f, 0)))
                        .waitSeconds(1.5)
                        .stopAndAdd(new Wall(wall, 0))
                        .waitSeconds(.5)
                        .stopAndAdd(new Kick(kicker, .25))
                        .waitSeconds(1.25)
                        .stopAndAdd(new Wall(wall, .4))
                        .stopAndAdd(new Kick(kicker, 0))
                        .waitSeconds(.5)
                        .stopAndAdd(new ParallelAction(new Index(indexMotor, p, i, d, f, 230)))
                        .waitSeconds(1.5)
                        .stopAndAdd(new Wall(wall, 0))
                        .waitSeconds(.5)
                        .stopAndAdd(new Kick(kicker, .25))
                        .waitSeconds(.5)
                        .stopAndAdd(new Wall(wall, .4))
                        .stopAndAdd(new Kick(kicker, 0))
                        .waitSeconds(.5)
                        .stopAndAdd(new ParallelAction(new Index(indexMotor, p, i, d, f, 450)))
                        .waitSeconds(1.5)
                        .stopAndAdd(new Wall(wall, 0))
                        .waitSeconds(.5)
                        .stopAndAdd(new Kick(kicker, .25))
                        .waitSeconds(.5)
                        .stopAndAdd(new Wall(wall, .4))
                        .stopAndAdd(new Kick(kicker, 0))
                        .waitSeconds(100)
                        .build());

    }
    public class Kick implements Action {
        Servo kick;
        double pos;
        public Kick (Servo kick, double pos) {
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
        public Wall (Servo wall, double pos) {
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
        Camera camera = new Camera();
        boolean fire = true;
        public Launch (DcMotorEx launcherMotor, DcMotorEx leftMotor, DcMotorEx rightMotor, HardwareMap hardwareMap) {
            this.launcherMotor = launcherMotor;
            this.leftMotor = leftMotor;
            this.rightMotor = rightMotor;
            v = new Velocity(leftMotor, rightMotor);
            launcher = new Launcher(launcherMotor, v);
            camera.init(hardwareMap);
            camera.start();


        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            boolean tagVisible = camera.TagID() != -1;
            double distanceM = camera.getTagDistance();

            launcher.setEnabled(fire && tagVisible);
            launcher.update(distanceM);
            telemetry.addData("Visible", tagVisible);
            telemetry.addData("target", launcher.getTargetVelocity());
            telemetry.addData("D", distanceM);
            telemetry.update();
            launcherMotor.setVelocity(launcher.getTargetVelocity());
            return distanceM == 0;

        }
    }
    public class Index implements Action {
        DcMotorEx index;
        double p, i, d, f;
        int targetPosition;
        PIDController controller;
        public Index(DcMotorEx index, double p, double i, double d, double f, int targetPosition) {
            this.p = p;
            this.i = i;
            this.d = d;
            this.f = f;
            this.targetPosition = targetPosition;
            this.index = index;
            controller = new PIDController(p, i, d);

        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            controller.setPID(p,i,d);
            int indexPosition = index.getCurrentPosition();
            double pid = controller.calculate(indexPosition, targetPosition);
            double ff = Math.cos(Math.toRadians(targetPosition / (28.0*27)* 360)) * f;

            double power = pid + ff;

            index.setPower(power);
            if (indexPosition >= targetPosition -1 && indexPosition <= targetPosition +1) {
                index.setPower(0);
                return false;
            }
            return true;

        }
    }
}

