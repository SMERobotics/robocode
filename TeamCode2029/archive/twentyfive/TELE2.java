//prob dont workify but might idk, this is the default drive mode; left stick forward, right stick turn
package com.n0tasha4k.ftc.twentyfive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Config
@TeleOp(name = "PID Drive Dashboard Tuning")
public class TELE2 extends LinearOpMode {

    // === Dashboard-tunable PID coefficients ===
    public static double Kp = 0.002;
    public static double Ki = 0.000002;
    public static double Kd = 0.0004;

    // Max ticks per second (depends on your motor type)
    public static double MAX_TICKS_PER_SECOND = 2500;

    // Reusable PID class
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
        DcMotorEx left = hardwareMap.get(DcMotorEx.class, "left");
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, "right");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "activeShooter");
        DcMotor index = hardwareMap.dcMotor.get("index");
        Servo finger = hardwareMap.servo.get("indexfinger");

        left.setDirection(DcMotor.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // PID controllers for each drive motor
        PIDController leftPID = new PIDController(Kp, Ki, Kd);
        PIDController rightPID = new PIDController(Kp, Ki, Kd);

        // FTC Dashboard instance
        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();
        if (isStopRequested()) return;

        double lastTime = getRuntime();

        while (opModeIsActive()) {
            double currentTime = getRuntime();
            double dt = currentTime - lastTime;
            lastTime = currentTime;

            // Update PID coefficients live from Dashboard
            leftPID.updateCoefficients(Kp, Ki, Kd);
            rightPID.updateCoefficients(Kp, Ki, Kd);

            // Joystick input → target velocities
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double leftTarget = (drive + turn) * MAX_TICKS_PER_SECOND;
            double rightTarget = (drive - turn) * MAX_TICKS_PER_SECOND;

            leftPID.setSetpoint(leftTarget);
            rightPID.setSetpoint(rightTarget);

            double leftVelocity = left.getVelocity();
            double rightVelocity = right.getVelocity();

            double leftOutput = leftPID.calculate(leftVelocity, dt);
            double rightOutput = rightPID.calculate(rightVelocity, dt);

            // Clamp motor powers
            leftOutput = Math.max(-1, Math.min(1, leftOutput));
            rightOutput = Math.max(-1, Math.min(1, rightOutput));

            left.setPower(leftOutput);
            right.setPower(rightOutput);

            // Shooter/index logic (unchanged)
            if (gamepad1.a) {
                finger.setPosition(1);
                if (shooter.getVelocity() > 1900) {
                    index.setPower(-1);
                } else {
                    index.setPower(0);
                }
            } else {
                finger.setPosition(0);
                index.setPower(0);
            }

            if (gamepad1.y) {
                shooter.setVelocity(2000);
            } else {
                shooter.setVelocity(0);
            }

            // Telemetry to RC & Dashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Left Target", leftTarget);
            packet.put("Right Target", rightTarget);
            packet.put("Left Velocity", leftVelocity);
            packet.put("Right Velocity", rightVelocity);
            packet.put("Left Output", leftOutput);
            packet.put("Right Output", rightOutput);
            packet.put("Kp", Kp);
            packet.put("Ki", Ki);
            packet.put("Kd", Kd);
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Left Velocity", leftVelocity);
            telemetry.addData("Right Velocity", rightVelocity);
            telemetry.addData("Left Target", leftTarget);
            telemetry.addData("Right Target", rightTarget);
            telemetry.addData("Kp", Kp);
            telemetry.addData("Ki", Ki);
            telemetry.addData("Kd", Kd);
            telemetry.update();
        }
    }
}
