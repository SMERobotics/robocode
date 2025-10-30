package com.technodot.ftc.twentyfive.common;

public class PIDController {
    // Gains
    private double kP;
    private double kI;
    private double kD;

    // State
    private double setpoint;
    private double integral;
    private double prevError;
    private boolean hasPrevError;

    // Track last error rate for tolerance checks
    private double lastErrorRate; // units per second
    private boolean hasLastRate;

    // Time tracking (for implicit dt)
    private long lastTimeNanos;
    private boolean hasLastTime;

    // Limits
    private double minOutput = -Double.MAX_VALUE;
    private double maxOutput = Double.MAX_VALUE;
    private double minIntegral = -Double.MAX_VALUE;
    private double maxIntegral = Double.MAX_VALUE;

    // Tolerances
    private double positionTolerance = 0.0; // in same units as measurement
    private double velocityTolerance = Double.MAX_VALUE; // units per second (MAX => disabled)

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     * Calculate output using the elapsed time since last call.
     * If this is the first call, derivative and integral are not applied.
     */
    public double calculate(double measurement) {
        final long now = System.nanoTime();
        double output;

        if (!hasLastTime) {
            // First run: compute P only, initialize time
            output = calculateInternal(measurement, 0.0);
            lastTimeNanos = now;
            hasLastTime = true;
        } else {
            double dt = (now - lastTimeNanos) / 1_000_000_000.0; // seconds
            if (dt < 0) dt = 0; // guard (shouldn't happen)
            output = calculateInternal(measurement, dt);
            lastTimeNanos = now;
        }
        return output;
    }

    /**
     * Calculate output using an explicit dt in seconds.
     * dt <= 0 disables I and D for this step but still returns the P term.
     */
    public double calculate(double measurement, double dtSeconds) {
        return calculateInternal(measurement, dtSeconds);
    }

    // Core computation with optional dt (seconds). dt <= 0 => only P is applied.
    private double calculateInternal(double measurement, double dt) {
        final double error = setpoint - measurement;

        // Proportional
        double p = kP * error;

        // Integral (with anti-windup via clamping)
        double i = 0.0;
        if (kI != 0.0 && dt > 0.0) {
            integral += error * dt;
            // Clamp integral accumulator
            if (integral > maxIntegral) integral = maxIntegral;
            if (integral < minIntegral) integral = minIntegral;
            i = kI * integral;
        }

        // Derivative (on error) – uses previous error when available
        double d = 0.0;
        if (dt > 0.0 && hasPrevError) {
            double errorRate = (error - prevError) / dt;
            lastErrorRate = errorRate;
            hasLastRate = true;
            if (kD != 0.0) {
                d = kD * errorRate;
            }
        } else if (dt <= 0.0) {
            // no rate update when dt is invalid
            hasLastRate = false;
        }

        // Save state for next step
        prevError = error;
        hasPrevError = true;

        // Sum and clamp output
        double output = p + i + d;
        if (output > maxOutput) output = maxOutput;
        if (output < minOutput) output = minOutput;
        return output;
    }

    // Configuration API

    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setP(double kP) { this.kP = kP; }
    public void setI(double kI) { this.kI = kI; }
    public void setD(double kD) { this.kD = kD; }

    public double getP() { return kP; }
    public double getI() { return kI; }
    public double getD() { return kD; }

    public void setSetpoint(double setpoint) { this.setpoint = setpoint; }
    public double getSetpoint() { return setpoint; }

    /**
     * Clamp output to [min, max]. Defaults to unbounded.
     */
    public void setOutputRange(double min, double max) {
        if (min > max) {
            // swap to be forgiving
            double tmp = min; min = max; max = tmp;
        }
        this.minOutput = min;
        this.maxOutput = max;
    }

    /**
     * Clamp the integral accumulator contribution. Defaults to unbounded.
     */
    public void setIntegralRange(double min, double max) {
        if (min > max) {
            double tmp = min; min = max; max = tmp;
        }
        this.minIntegral = min;
        this.maxIntegral = max;
        // also clamp current integral to respect new bounds
        if (integral > maxIntegral) integral = maxIntegral;
        if (integral < minIntegral) integral = minIntegral;
    }

    /**
     * Set tolerances used by atSetpoint().
     * @param positionTolerance allowable absolute error (>= 0)
     * @param velocityTolerance allowable absolute error rate in units/sec (>= 0)
     *                           Use Double.MAX_VALUE to disable velocity check.
     */
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        if (positionTolerance < 0 || velocityTolerance < 0) {
            throw new IllegalArgumentException("Tolerances must be non-negative");
        }
        this.positionTolerance = positionTolerance;
        this.velocityTolerance = velocityTolerance;
    }

    /**
     * True when the absolute error is within position tolerance and, if enabled,
     * the absolute error rate is within velocity tolerance.
     * Requires at least one prior calculate() call.
     */
    public boolean atSetpoint() {
        if (!hasPrevError) {
            return false; // need at least one measurement step
        }

        boolean positionOK = Math.abs(prevError) <= positionTolerance;

        // If velocity tolerance is disabled (Double.MAX_VALUE), only check position
        if (velocityTolerance == Double.MAX_VALUE) {
            return positionOK;
        }

        boolean velocityOK = hasLastRate && Math.abs(lastErrorRate) <= velocityTolerance;
        return positionOK && velocityOK;
    }

    /** Reset integrator, previous error, and internal timers. */
    public void reset() {
        integral = 0.0;
        prevError = 0.0;
        hasPrevError = false;
        lastErrorRate = 0.0;
        hasLastRate = false;
        lastTimeNanos = 0L;
        hasLastTime = false;
    }
}
