package com.technodot.ftc.twentyfive.common;

import com.qualcomm.robotcore.util.Range;

public class PIDController {
    private double kP;  // Proportional gain
    private double kI;  // Integral gain
    private double kD;  // Derivative gain

    private double setpoint;
    private double lastError;
    private double integral;
    private double lastTime;

    private double minOutput;
    private double maxOutput;
    private boolean outputLimited;

    private double integralMin;
    private double integralMax;
    private boolean integralLimited;

    /**
     * Creates a new PID controller with the specified gains.
     *
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        this.setpoint = 0;
        this.lastError = 0;
        this.integral = 0;
        this.lastTime = -1;

        this.outputLimited = false;
        this.integralLimited = false;
    }

    /**
     * Calculates the PID output based on the current measurement.
     *
     * @param measurement Current measured value
     * @param currentTime Current time in seconds
     * @return Control output
     */
    public double calculate(double measurement, double currentTime) {
        double error = setpoint - measurement;

        // Calculate time delta
        double dt;
        if (lastTime < 0) {
            dt = 0;
            lastTime = currentTime;
        } else {
            dt = currentTime - lastTime;
            lastTime = currentTime;
        }

        // Proportional term
        double p = kP * error;

        // Integral term
        if (dt > 0) {
            integral += error * dt;

            // Apply integral limits if enabled
            if (integralLimited) {
                integral = Range.clip(integral, integralMin, integralMax);
            }
        }
        double i = kI * integral;

        // Derivative term
        double d = 0;
        if (dt > 0) {
            d = kD * (error - lastError) / dt;
        }
        lastError = error;

        // Calculate total output
        double output = p + i + d;

        // Apply output limits if enabled
        if (outputLimited) {
            output = Range.clip(output, minOutput, maxOutput);
        }

        return output;
    }

    /**
     * Calculates the PID output without explicit time (uses auto-incrementing time).
     * Useful for fixed-rate control loops.
     *
     * @param measurement Current measured value
     * @return Control output
     */
    public double calculate(double measurement) {
        if (lastTime < 0) {
            lastTime = 0;
        }
        return calculate(measurement, lastTime + 0.02); // Assume 20ms default
    }

    /**
     * Sets the target setpoint.
     *
     * @param setpoint Target value
     */
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    /**
     * Gets the current setpoint.
     *
     * @return Current setpoint
     */
    public double getSetpoint() {
        return setpoint;
    }

    /**
     * Sets the PID gains.
     *
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     * Sets the proportional gain.
     *
     * @param kP Proportional gain
     */
    public void setP(double kP) {
        this.kP = kP;
    }

    /**
     * Sets the integral gain.
     *
     * @param kI Integral gain
     */
    public void setI(double kI) {
        this.kI = kI;
    }

    /**
     * Sets the derivative gain.
     *
     * @param kD Derivative gain
     */
    public void setD(double kD) {
        this.kD = kD;
    }

    /**
     * Gets the proportional gain.
     *
     * @return Proportional gain
     */
    public double getP() {
        return kP;
    }

    /**
     * Gets the integral gain.
     *
     * @return Integral gain
     */
    public double getI() {
        return kI;
    }

    /**
     * Gets the derivative gain.
     *
     * @return Derivative gain
     */
    public double getD() {
        return kD;
    }

    /**
     * Sets output limits to prevent output from exceeding bounds.
     *
     * @param min Minimum output value
     * @param max Maximum output value
     */
    public void setOutputLimits(double min, double max) {
        this.minOutput = min;
        this.maxOutput = max;
        this.outputLimited = true;
    }

    /**
     * Removes output limits.
     */
    public void removeOutputLimits() {
        this.outputLimited = false;
    }

    /**
     * Sets integral limits to prevent integral windup.
     *
     * @param min Minimum integral value
     * @param max Maximum integral value
     */
    public void setIntegralLimits(double min, double max) {
        this.integralMin = min;
        this.integralMax = max;
        this.integralLimited = true;
    }

    /**
     * Removes integral limits.
     */
    public void removeIntegralLimits() {
        this.integralLimited = false;
    }

    /**
     * Resets the controller state (integral and derivative terms).
     */
    public void reset() {
        this.integral = 0;
        this.lastError = 0;
        this.lastTime = -1;
    }

    /**
     * Gets the current integral value.
     *
     * @return Current integral accumulation
     */
    public double getIntegral() {
        return integral;
    }

    /**
     * Gets the last error value.
     *
     * @return Last error
     */
    public double getLastError() {
        return lastError;
    }

    /**
     * Utility method to clamp a value between min and max.
     *
     * @param value Value to clamp
     * @param min Minimum value
     * @param max Maximum value
     * @return Clamped value
     */
    private double clamp(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }
}
