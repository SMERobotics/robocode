package com.n0tasha4k.ftc.twentyfive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;
import android.graphics.Color;

@Config
@TeleOp(name = "Tank PID Drivetrain + Shooter + Graphs")
public class testtele67 extends LinearOpMode {

    // ======== Dashboard-tunable PID constants ========
    public static double Kp = 0.002;
    public static double Ki = 0.000002;
    public static double Kd = 0.0004;
    public static double MAX_TICKS_PER_SECOND = 2500;

    // ======== PID Controller ========
    public static class PIDController {
        private double kp, ki, kd;
        private double setpoint;
        private double integral;
        private double lastError;

        public PIDController(double kp, double ki, double kd) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
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
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
        }
    }

    @Override
    public void runOpMode() {

        // ======== Hardware setup ========
        DcMotorEx left = hardwareMap.get(DcMotorEx.class, "left");
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, "right");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "activeShooter");
        DcMotor index = hardwareMap.dcMotor.get("index");
        DcMotor intake = hardwareMap.dcMotor.get("intakey");
        Servo finger = hardwareMap.servo.get("indexfinger");

        left.setDirection(DcMotor.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean isShooterOn = false;

        // PID controllers for both drive motors
        PIDController leftPID = new PIDController(Kp, Ki, Kd);
        PIDController rightPID = new PIDController(Kp, Ki, Kd);

        // FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();
        if (isStopRequested()) return;

        double lastTime = getRuntime();

        while (opModeIsActive()) {
            double currentTime = getRuntime();
            double dt = currentTime - lastTime;
            lastTime = currentTime;

            // Update PID coefficients dynamically
            leftPID.updateCoefficients(Kp, Ki, Kd);
            rightPID.updateCoefficients(Kp, Ki, Kd);

            // Driver control input
            double leftStick = gamepad1.right_stick_y;
            double rightStick = gamepad1.left_stick_y;

            // Convert joystick input to target velocity (ticks/sec)
            double leftTarget = leftStick * MAX_TICKS_PER_SECOND;
            double rightTarget = rightStick * MAX_TICKS_PER_SECOND;

            leftPID.setSetpoint(leftTarget);
            rightPID.setSetpoint(rightTarget);

            double leftVelocity = left.getVelocity();
            double rightVelocity = right.getVelocity();

            double leftOutput = leftPID.calculate(leftVelocity, dt);
            double rightOutput = rightPID.calculate(rightVelocity, dt);

            leftOutput = Math.max(-1, Math.min(1, leftOutput));
            rightOutput = Math.max(-1, Math.min(1, rightOutput));

            left.setPower(leftOutput);
            right.setPower(rightOutput);

            // ======== Shooter and intake logic ========
            if (gamepad1.a) {
                finger.setPosition(1);
                if (shooter.getVelocity() > 1650) {
                    index.setPower(-1);
                } else {
                    index.setPower(0);
                }
            } else {
                finger.setPosition(0);
                index.setPower(0);
            }

            if (gamepad1.yWasPressed()) {
                isShooterOn = !isShooterOn;
            }
            shooter.setVelocity(isShooterOn ? 1800 : 0);

            intake.setPower(gamepad1.right_bumper ? 1 : 0);

            // ======== Telemetry to Driver Hub ========
            telemetry.addData("Left Target", leftTarget);
            telemetry.addData("Right Target", rightTarget);
            telemetry.addData("Left Velocity", leftVelocity);
            telemetry.addData("Right Velocity", rightVelocity);
            telemetry.addData("Left PID Output", leftOutput);
            telemetry.addData("Right PID Output", rightOutput);
            telemetry.addData("Shooter Velocity", shooter.getVelocity());
            telemetry.addData("Shooter Status", isShooterOn ? "On" : "Off");
            telemetry.update();

            // ======== FTC Dashboard Graphing ========
            TelemetryPacket packet = new TelemetryPacket();
            Canvas field = packet.fieldOverlay();

            // Draw left velocity vs target in blue
            field.setStroke("#0000FF"); // blue
            field.strokeLine(0, leftVelocity / MAX_TICKS_PER_SECOND * 100, 100, leftTarget / MAX_TICKS_PER_SECOND * 100);

            // Draw right velocity vs target in red
            field.setStroke("#FF0000"); // red
            field.strokeLine(0, rightVelocity / MAX_TICKS_PER_SECOND * 100, 100, rightTarget / MAX_TICKS_PER_SECOND * 100);

            // Add numeric data to Dashboard
            packet.put("Left Target", leftTarget);
            packet.put("Left Velocity", leftVelocity);
            packet.put("Right Target", rightTarget);
            packet.put("Right Velocity", rightVelocity);
            packet.put("Left Output", leftOutput);
            packet.put("Right Output", rightOutput);
            packet.put("Kp", Kp);
            packet.put("Ki", Ki);
            packet.put("Kd", Kd);

            dashboard.sendTelemetryPacket(packet);
        }
    }
}
