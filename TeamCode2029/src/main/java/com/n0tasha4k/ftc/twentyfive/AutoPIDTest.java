package com.n0tasha4k.ftc.twentyfive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Autonomous(name = "Modular PID Autonomous with Graphnot main so far, not live updateable", group = "PID")
public class AutoPIDTest extends LinearOpMode {

    // ===== FTC Dashboard Tunables =====
    public static double KP = 0.0008;
    public static double KI = 0.0;
    public static double KD = 0.00005;
    public static double TURN_KP = 0.01; // simpler proportional turn
    public static double ALLOWABLE_ERROR_TICKS = 30;
    public static double TICKS_PER_REV = 537.7; // GoBilda 312 RPM
    public static double WHEEL_DIAMETER_INCHES = 3.77953;
    public static double TRACK_WIDTH_INCHES = 14.0; // distance between wheels

    private static final double TICKS_PER_INCH =
            TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);

    private DcMotorEx left, right;
    private FtcDashboard dashboard;
    private PIDController leftPID, rightPID;

    @Override
    public void runOpMode() {
        // ===== Initialization =====
        left = hardwareMap.get(DcMotorEx.class, "left");
        right = hardwareMap.get(DcMotorEx.class, "right");
        dashboard = FtcDashboard.getInstance();

        left.setDirection(DcMotor.Direction.REVERSE);
        resetEncoders();

        leftPID = new PIDController(KP, KI, KD);
        rightPID = new PIDController(KP, KI, KD);

        telemetry.addLine("Ready — waiting for start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ===== Autonomous Sequence =====
        for (int i = 0; i < 9101 && opModeIsActive(); i++) {
            driveForward(18);  // forward 24 inches
            sleep(250);
            driveBackward(18); // backward 24 inches
            sleep(250);
        }

        turnDegrees(90);
        sleep(500);
        turnDegrees(-90);

        stopMotors();
    }

    // ======== MODULAR DRIVE COMMANDS ========

    private void driveForward(double inches) {
        driveToPosition(inches * TICKS_PER_INCH);
    }

    private void driveBackward(double inches) {
        driveToPosition(-inches * TICKS_PER_INCH);
    }

    private void driveToPosition(double targetTicks) {
        resetEncoders();
        leftPID.setSetpoint(targetTicks);
        rightPID.setSetpoint(targetTicks);

        while (opModeIsActive()) {
            double leftPos = left.getCurrentPosition();
            double rightPos = right.getCurrentPosition();

            double leftPower = leftPID.update(leftPos);
            double rightPower = rightPID.update(rightPos);

            leftPower = clip(leftPower);
            rightPower = clip(rightPower);

            left.setPower(leftPower);
            right.setPower(rightPower);

            sendDashboardTelemetry(leftPos, rightPos, targetTicks, leftPower, rightPower);

            if (Math.abs(leftPos - targetTicks) < ALLOWABLE_ERROR_TICKS &&
                    Math.abs(rightPos - targetTicks) < ALLOWABLE_ERROR_TICKS) {
                break;
            }

            sleep(20);
        }

        stopMotors();
    }

    private void turnDegrees(double degrees) {
        // Simple proportional turning based on encoder difference
        resetEncoders();

        // Estimate ticks for a given rotation
        double arcLengthInches = (Math.PI * TRACK_WIDTH_INCHES) * (degrees / 360.0);
        double targetTicks = arcLengthInches * TICKS_PER_INCH;

        while (opModeIsActive()) {
            double error = targetTicks - (right.getCurrentPosition() - left.getCurrentPosition());
            double turnPower = TURN_KP * error;

            turnPower = clip(turnPower);

            left.setPower(-turnPower);
            right.setPower(turnPower);

            sendDashboardTurnTelemetry(error, turnPower, degrees);

            if (Math.abs(error) < ALLOWABLE_ERROR_TICKS) break;

            sleep(20);
        }

        stopMotors();
    }

    // ======== HELPER FUNCTIONS ========

    private void sendDashboardTelemetry(double leftPos, double rightPos,
                                        double targetTicks, double leftPower, double rightPower) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        packet.put("Left Encoder", leftPos);
        packet.put("Right Encoder", rightPos);
        packet.put("Target Ticks", targetTicks);
        packet.put("Left Power", leftPower);
        packet.put("Right Power", rightPower);

        field.setStroke("blue");
        field.strokeLine(0, 0, targetTicks / 50.0, 0);
        field.setStroke("red");
        field.strokeLine(0, 0, leftPos / 50.0, 5);
        field.setStroke("green");
        field.strokeLine(0, 0, rightPos / 50.0, -5);

        dashboard.sendTelemetryPacket(packet);
    }

    private void sendDashboardTurnTelemetry(double error, double turnPower, double targetDegrees) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Turn Error", error);
        packet.put("Turn Power", turnPower);
        packet.put("Target Degrees", targetDegrees);
        dashboard.sendTelemetryPacket(packet);
    }

    private void resetEncoders() {
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void stopMotors() {
        left.setPower(0);
        right.setPower(0);
    }

    private double clip(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }

    // ======== INTERNAL PID CLASS ========

    public static class PIDController {
        private double kp, ki, kd;
        private double setpoint;
        private double lastError;
        private double integral;

        public PIDController(double kp, double ki, double kd) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
        }

        public void setSetpoint(double setpoint) {
            this.setpoint = setpoint;
            this.integral = 0;
            this.lastError = 0;
        }

        public double update(double current) {
            double error = setpoint - current;
            integral += error;
            double derivative = error - lastError;
            lastError = error;
            return (kp * error) + (ki * integral) + (kd * derivative);
        }

        public double getSetpoint() {
            return setpoint;
        }
    }
}