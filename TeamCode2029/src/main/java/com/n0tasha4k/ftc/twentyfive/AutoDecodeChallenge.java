//no clue what this'll do
package com.n0tasha4k.ftc.twentyfive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Autonomous(name = "Decode Challenge Auto (PID)")
public class AutoDecodeChallenge extends LinearOpMode {

    // Dashboard tunable PID coefficients
    public static double Kp = 0.003;
    public static double Ki = 0.0;
    public static double Kd = 0.0004;

    // Motor max ticks/sec (estimate for your drivetrain)
    public static double MAX_TICKS_PER_SECOND = 2500;

    // Distance constants (change to match your robot)
    public static double TICKS_PER_REV = 537.6; // e.g. goBILDA 312 RPM
    public static double WHEEL_DIAMETER_INCHES = 3.78;
    public static double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);

    // Simple PID controller
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

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx left = hardwareMap.get(DcMotorEx.class, "left");
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, "right");

        left.setDirection(DcMotorEx.Direction.REVERSE);
        left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        PIDController leftPID = new PIDController(Kp, Ki, Kd);
        PIDController rightPID = new PIDController(Kp, Ki, Kd);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry.addLine("Ready to start autonomous");
        telemetry.update();
        waitForStart();

        double targetDistanceInches = 36;  // drive 36 inches forward
        double targetTicks = targetDistanceInches * TICKS_PER_INCH;

        leftPID.setSetpoint(targetTicks);
        rightPID.setSetpoint(targetTicks);

        double lastTime = getRuntime();

        while (opModeIsActive()) {
            double currentTime = getRuntime();
            double dt = currentTime - lastTime;
            lastTime = currentTime;

            // Update PID coefficients live from Dashboard
            leftPID.updateCoefficients(Kp, Ki, Kd);
            rightPID.updateCoefficients(Kp, Ki, Kd);

            double leftPosition = Math.abs(left.getCurrentPosition());
            double rightPosition = Math.abs(right.getCurrentPosition());

            double leftOutput = leftPID.calculate(leftPosition, dt);
            double rightOutput = rightPID.calculate(rightPosition, dt);

            // Convert position PID output to power (-1 to 1)
            leftOutput = Math.max(-1, Math.min(1, leftOutput / MAX_TICKS_PER_SECOND));
            rightOutput = Math.max(-1, Math.min(1, rightOutput / MAX_TICKS_PER_SECOND));

            left.setPower(leftOutput);
            right.setPower(rightOutput);

            telemetry.addData("Left Pos", leftPosition);
            telemetry.addData("Right Pos", rightPosition);
            telemetry.addData("Left Power", leftOutput);
            telemetry.addData("Right Power", rightOutput);
            telemetry.addData("Target (ticks)", targetTicks);
            telemetry.update();

            // Stop when we reach target within tolerance
            if (Math.abs(targetTicks - leftPosition) < 50 &&
                    Math.abs(targetTicks - rightPosition) < 50) {
                left.setPower(0);
                right.setPower(0);
                break;
            }
        }

        // Done!
        telemetry.addLine("Reached target distance!");
        telemetry.update();
        sleep(1000);
    }
}
