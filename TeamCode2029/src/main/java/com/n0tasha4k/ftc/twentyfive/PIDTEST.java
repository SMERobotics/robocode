//bad
package com.n0tasha4k.ftc.twentyfive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "babdabdbadbadbabdadbPIDTEST yayaya(updated graph changeability, error correction on right motor)")
public class PIDTEST extends LinearOpMode {

    // ============================
    // 🔧 Dashboard PID Variables
    // ============================
    public static double KP = 0.002;    // proportional
    public static double KI = 0.000;   // integral
    public static double KD = 0.0002;    // derivative

    public static double POWER_LIMIT = 0.6;          // max motor power
    public static double RIGHT_CORRECTION = 0.7828723194;    // tune if robot drifts right

    // ============================
    // ⚙️ Hardware
    // ============================
    private DcMotorEx left, right;
    private FtcDashboard dashboard;

    // ============================
    // 🧮 PID Controller Class
    // ============================
    class PIDController {
        public double kp, ki, kd;
        private double integralSum = 0;
        private double lastError = 0;
        private double lastTime = 0;

        public PIDController(double kp, double ki, double kd) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
        }

        public double calculate(double target, double current) {
            double error = target - current;
            double currentTime = runtime.seconds();
            double dt = currentTime - lastTime;

            if (dt > 0) {
                integralSum += error * dt;
                double derivative = (error - lastError) / dt;
                lastError = error;
                lastTime = currentTime;
                return (kp * error) + (ki * integralSum) + (kd * derivative);
            } else {
                return 0;
            }
        }

        public void setCoefficients(double kp, double ki, double kd) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
        }

        public void reset() {
            integralSum = 0;
            lastError = 0;
            lastTime = 0;
        }
    }

    private PIDController leftPID = new PIDController(KP, KI, KD);
    private PIDController rightPID = new PIDController(KP, KI, KD);
    private ElapsedTime runtime = new ElapsedTime();

    // ============================
    // 📏 Encoder + Drive Utility
    // ============================
    private static final double TICKS_PER_REV = 537.6;
    private static final double WHEEL_DIAMETER_INCHES = 3.5;
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);

    @Override
    public void runOpMode() {
        left = hardwareMap.get(DcMotorEx.class, "left");
        right = hardwareMap.get(DcMotorEx.class, "right");

        left.setDirection(DcMotor.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dashboard = FtcDashboard.getInstance();

        telemetry.addLine("Ready for start. Open Dashboard to tune (FtcDashboard.localhost:8080)");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        // Example usage: Drive back and forth forever for tuning
        while (opModeIsActive()) {
            driveForward(24);
            sleep(500);
            driveForward(-24);// backward
            sleep(500);
        }
    }

    // ============================
    // 🚗 Drive Function
    // ============================
    private void driveForward(double distanceInches) {
        leftPID.reset();
        rightPID.reset();

        double targetTicks = distanceInches * TICKS_PER_INCH;
        double leftStart = left.getCurrentPosition();
        double rightStart = right.getCurrentPosition();

        while (opModeIsActive()) {
            // Live-update PID constants
            leftPID.setCoefficients(KP, KI, KD);
            rightPID.setCoefficients(KP, KI, KD);

            // Compute current positions
            double leftPos = left.getCurrentPosition() - leftStart;
            double rightPos = right.getCurrentPosition() - rightStart;

            double leftOutput = leftPID.calculate(targetTicks, leftPos);
            double rightOutput = rightPID.calculate(targetTicks, rightPos);

            // Limit power and apply drift correction
            leftOutput = clamp(leftOutput, -POWER_LIMIT, POWER_LIMIT);
            rightOutput = clamp(rightOutput * RIGHT_CORRECTION, -POWER_LIMIT, POWER_LIMIT);

            left.setPower(leftOutput);
            right.setPower(rightOutput);

            // Graph + telemetry
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("LeftPos", leftPos);
            packet.put("RightPos", rightPos);
            packet.put("LeftPower", leftOutput);
            packet.put("RightPower", rightOutput);
            packet.put("TargetTicks", targetTicks);

            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Target (ticks)", targetTicks);
            telemetry.addData("Left Pos", leftPos);
            telemetry.addData("Right Pos", rightPos);
            telemetry.addData("Left Power", leftOutput);
            telemetry.addData("Right Power", rightOutput);
            telemetry.update();

            // Break condition when both near target
            if (Math.abs(leftPos - targetTicks) < 15 && Math.abs(rightPos - targetTicks) < 15) {
                break;
            }
        }

        left.setPower(0);
        right.setPower(0);
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}