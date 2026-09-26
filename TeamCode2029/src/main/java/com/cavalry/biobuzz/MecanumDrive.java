package com.cavalry.biobuzz;

import com.acmerobotics.dashboard.config.Config;

@Config
public class MecanumDrive {

    // Drivetrain tuning variables (editable via code or FTC Dashboard)
    public static double driveSpeed = 1.0;
    public static double strafeSpeed = 1.0;
    public static double turnSpeed = 1.0;

    // Data structure to hold the calculated powers for each wheel
    public static class WheelPowers {
        public double frontLeft;
        public double frontRight;
        public double rearLeft;
        public double rearRight;

        public WheelPowers(double frontLeft, double frontRight, double rearLeft, double rearRight) {
            this.frontLeft = frontLeft;
            this.frontRight = frontRight;
            this.rearLeft = rearLeft;
            this.rearRight = rearRight;
        }
    }

    /**
     * Calculates normalized wheel powers based on joystick inputs and tuning multipliers.
     *
     * @param forward Forward/backward joystick value (-1.0 to 1.0)
     * @param strafe  Left/right joystick value (-1.0 to 1.0)
     * @param turn    Rotation joystick value (-1.0 to 1.0)
     * @return WheelPowers object with calculated power for each motor
     */
    public static WheelPowers calculatePowers(double forward, double strafe, double turn) {
        // 1. Apply tuning multipliers
        double f = forward * driveSpeed;
        double s = strafe * strafeSpeed;
        double t = turn * turnSpeed;

        // 2. Combine inputs for each wheel
        double frontLeftPower = f + s + t;
        double backLeftPower = f - s + t;
        double frontRightPower = f - s - t;
        double backRightPower = f + s - t;

        // 3. Find the maximum absolute value to normalize power
        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))
        ));

        // 4. Divide powers by max if any value exceeds 1.0
        return new WheelPowers(
                frontLeftPower / max,
                frontRightPower / max,
                backLeftPower / max,
                backRightPower / max
        );
    }
}

