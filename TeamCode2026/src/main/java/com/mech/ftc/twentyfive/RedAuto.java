package com.mech.ftc.twentyfive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.mech.ftc.twentyfive.defaults.Camera;
import com.mech.ftc.twentyfive.defaults.Launcher;
import com.mech.ftc.twentyfive.defaults.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "RedAuto")
public class RedAuto extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        DcMotorEx launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        Servo kicker = hardwareMap.get(Servo.class, "servo");

        double p = 0.01, i = 0.022, d = 0.000277;
        double f = 0.01;
        DcMotorEx indexMotor = hardwareMap.get(DcMotorEx.class, "indexMotor");
        launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .strafeToLinearHeading(new Vector2d(1, 0), Math.toRadians(45))
                            .strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(0))
                            .build());

            }
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


        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            boolean tagVisible = camera.TagID() != -1;
            double distanceM = camera.getTagDistance();

            launcher.setEnabled(fire && tagVisible);
            launcher.update(distanceM);
            telemetry.addData("target", launcher.getTargetVelocity());
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
            telemetryPacket.put("indexPosition", indexPosition);
            telemetryPacket.put("target", targetPosition);
            if (indexPosition >= targetPosition -2 && indexPosition <= targetPosition +2) {
                index.setPower(0);
                return false;
            }
            return true;

        }
    }
}

