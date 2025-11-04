package com.n0tasha4k.ftc.twentyfive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Autonomous(name = "Decode Challenge Auto (Modular PID)")
public class AutoDecodeChallengeModular extends LinearOpMode {

    // Tunable PID coefficients
    public static double Kp = 0.003;
    public static double Ki = 0.0;
    public static double Kd = 0.0004;

    public static double MAX_TICKS_PER_SECOND = 2500;

    // Encoder constants
    public static double TICKS_PER_REV = 537.6; // goBILDA 312 RPM
    public static double WHEEL_DIAMETER_INCHES = 3.78;
    public static double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);

    private DcMotorEx left, right;
    private PIDController leftPID, rightPID;
    private FtcDashboard dashboard;

    // ──────────────────────────────────────────────
    // Simple PID Controller class
    public static class PIDController {
        private double kp, ki, kd;
        private double setpoint;
        private double integral;
        private double lastError;

        public PIDController(double kp, double ki, double kd) {
            this.kp = kp; this.ki = ki; this.kd = kd;
        }

        public void setSetpoint(double setpoint) {
            this.setpoint = setpoint;
        }

        public double calculate(double measuredValue, double dt) {
            double error = setpoint - measuredValue;
            integral += error * dt;
            double derivative = (error - lastError) / dt;
            lastError = error;
            return (kp * error) + (ki * integral) + (kd * derivative);
        }

        public void updateCoefficients(double kp, double ki, double kd) {
            this.kp = kp; this.ki = ki; this.kd = kd;
        }
    }
    // ──────────────────────────────────────────────

    @Override
    public void runOpMode() throws InterruptedException {

        left = hardwareMap.get(DcMotorEx.class, "left");
        right = hardwareMap.get(DcMotorEx.class, "right");

        left.setDirection(DcMotorEx.Direction.REVERSE);

        left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftPID = new PIDController(Kp, Ki, Kd);
        rightPID = new PIDController(Kp, Ki, Kd);
        dashboard = FtcDashboard.getInstance();

        telemetry.addLine("Ready for DECODE Auto");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        // ─────────── Example Autonomous Routine ───────────
        driveForward(36);   // drive forward 36 inches
        turnDegrees(90);    // turn right 90°; left is -90°
        driveBackward(24);   // drive backward 24 inches
        // You can chain these however your DECODE path requires
        // ───────────────────────────────────────────────────

        left.setPower(0);
        right.setPower(0);
        telemetry.addLine("Autonomous Complete!");
        telemetry.update();
    }

    // ──────────────────────────────────────────────
    // DRIVE FORWARD using PID + Encoders
    private void driveForward(double inches) {
        double targetTicks = inches * TICKS_PER_INCH;
        left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftPID.setSetpoint(targetTicks);
        rightPID.setSetpoint(targetTicks);

        double lastTime = getRuntime();

        while (opModeIsActive()) {
            double currentTime = getRuntime();
            double dt = currentTime - lastTime;
            lastTime = currentTime;

            leftPID.updateCoefficients(Kp, Ki, Kd);
            rightPID.updateCoefficients(Kp, Ki, Kd);

            double leftPosition = Math.abs(left.getCurrentPosition());
            double rightPosition = Math.abs(right.getCurrentPosition());

            double leftOutput = leftPID.calculate(leftPosition, dt);
            double rightOutput = rightPID.calculate(rightPosition, dt);

            leftOutput = Math.max(-1, Math.min(1, leftOutput / MAX_TICKS_PER_SECOND));
            rightOutput = Math.max(-1, Math.min(1, rightOutput / MAX_TICKS_PER_SECOND));

            left.setPower(leftOutput);
            right.setPower(rightOutput);

            telemetry.addData("DriveForward → Left Pos", leftPosition);
            telemetry.addData("DriveForward → Right Pos", rightPosition);
            telemetry.update();

            if (Math.abs(targetTicks - leftPosition) < 50 &&
                    Math.abs(targetTicks - rightPosition) < 50) {
                break;
            }
        }

        left.setPower(0);
        right.setPower(0);
        sleep(300);
    }
    // ──────────────────────────────────────────────
    // DRIVE BACKWARD — wrapper for readability
    private void driveBackward(double inches) {
        driveForward(-inches);
    }
    // TURN using relative encoder differences
    private void turnDegrees(double degrees) {
        double ROBOT_TRACK_WIDTH_INCHES = 15.0; // distance between wheels
        double TURN_CIRCUMFERENCE = Math.PI * ROBOT_TRACK_WIDTH_INCHES;
        double inches = (TURN_CIRCUMFERENCE * (degrees / 360.0));
        double targetTicks = inches * TICKS_PER_INCH;

        left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftPID.setSetpoint(targetTicks);
        rightPID.setSetpoint(targetTicks);

        double lastTime = getRuntime();

        while (opModeIsActive()) {
            double currentTime = getRuntime();
            double dt = currentTime - lastTime;
            lastTime = currentTime;

            leftPID.updateCoefficients(Kp, Ki, Kd);
            rightPID.updateCoefficients(Kp, Ki, Kd);

            double leftPosition = Math.abs(left.getCurrentPosition());
            double rightPosition = Math.abs(right.getCurrentPosition());

            double leftOutput = leftPID.calculate(leftPosition, dt);
            double rightOutput = rightPID.calculate(rightPosition, dt);

            // Turn: left forward, right backward
            leftOutput = Math.max(-1, Math.min(1, leftOutput / MAX_TICKS_PER_SECOND));
            rightOutput = Math.max(-1, Math.min(1, rightOutput / MAX_TICKS_PER_SECOND));

            left.setPower(leftOutput);
            right.setPower(-rightOutput);

            telemetry.addData("TurnDegrees → Left Pos", leftPosition);
            telemetry.addData("TurnDegrees → Right Pos", rightPosition);
            telemetry.update();

            if (Math.abs(targetTicks - leftPosition) < 50 &&
                    Math.abs(targetTicks - rightPosition) < 50) {
                break;
            }
        }

        left.setPower(0);
        right.setPower(0);
        sleep(300);
    }
}

//???????AHAHHAHHAHHAHAHAHHAHAHHAHAHAHAHHAHAHAHHAHAHAHAHAHHAHAHHAHHAHHAHAHHHHHHH H HH H HHHHHHH HH HHTHHISI HIS SUCKS SO MCUH ISDOK LA< CANT FUCGIN FD SODMKOALSFTAKE INT ADSKNJFKNJCDSLNKSDVLNSDLNKSDCLNKJCDSLKNJ