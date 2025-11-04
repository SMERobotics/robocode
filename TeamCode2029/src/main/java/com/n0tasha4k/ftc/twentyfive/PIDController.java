//ts useless fr twin
package com.n0tasha4k.ftc.twentyfive;

public class PIDController {
    private double kp; // Proportional gain
    private double ki; // Integral gain
    private double kd; // Derivative gain

    private double integral;
    private double previousError;
    private double dt; // Time step (seconds)

    public PIDController(double kp, double ki, double kd, double dt) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.dt = dt;
        this.integral = 0;
        this.previousError = 0;
    }

    public double update(double setpoint, double actualValue) {
        double error = setpoint - actualValue;

        // --- Proportional term ---
        double Pout = kp * error;

        // --- Integral term ---
        integral += error * dt;
        double Iout = ki * integral;

        // --- Derivative term ---
        double derivative = (error - previousError) / dt;
        double Dout = kd * derivative;

        // --- Save state for next update ---
        previousError = error;

        // --- Compute total output ---
        double output = Pout + Iout + Dout;

        return output;
    }

    public void reset() {
        integral = 0;
        previousError = 0;
    }
}
