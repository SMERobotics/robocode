package com.technodot.ftc.twentyfive.robocore;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technodot.ftc.twentyfive.common.Controls;
import com.technodot.ftc.twentyfive.common.PIDController;

public class DeviceExtake extends Device {
    public DcMotorEx motorExtake;
    public ExtakeState currentState = ExtakeState.IDLE;

    private boolean pressingShootLow;
    private boolean pressingShootHigh;

    public final float velocityHigh = 1780; // target ticks/sec (tune as needed)
    public final float velocityLow = 1350;  // target ticks/sec (tune as needed)

    // Custom PID controller for velocity regulation
    private final PIDController velocityPID = new PIDController(0.001, 0.0000, 0.0002); // Initial gains; tune!
    private long lastTimeNs = -1;
    private int lastPosition = 0;
    private double measuredVelocity = 0; // ticks/sec

    // Optional feedforward term (kF) ~ power per ticks/sec; tune based on motor characteristics
    private static final double KF = 1.0 / 2500.0; // Assuming ~2500 ticks/sec at full power; adjust after testing

    public enum ExtakeState {
        IDLE,
        SHOOTING_LOW,
        SHOOTING_HIGH,
        REVERSING
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        motorExtake = hardwareMap.get(DcMotorEx.class, "motorExtake");
        motorExtake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void start() {
        motorExtake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // Use RUN_WITHOUT_ENCODER so built-in velocity PID does not interfere
        motorExtake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lastTimeNs = -1;
        lastPosition = 0;
        measuredVelocity = 0;
        velocityPID.reset();
    }

    @Override
    public void update(Gamepad gamepad) {
        // Determine desired state from controls (toggle logic preserved)
        boolean shootLow = Controls.extakeShootLow(gamepad);
        boolean shootHigh = Controls.extakeShootHigh(gamepad);

        ExtakeState prevState = currentState;

        if (Controls.extakeShootReverse(gamepad)) {
            currentState = ExtakeState.REVERSING;
        } else {
            if (shootLow && !pressingShootLow) {
                if (currentState.equals(ExtakeState.SHOOTING_LOW)) {
                    currentState = ExtakeState.IDLE;
                } else {
                    currentState = ExtakeState.SHOOTING_LOW;
                }
            } else if (shootHigh && !pressingShootHigh) {
                if (currentState.equals(ExtakeState.SHOOTING_HIGH)) {
                    currentState = ExtakeState.IDLE;
                } else {
                    currentState = ExtakeState.SHOOTING_HIGH;
                }
            } else if (currentState.equals(ExtakeState.REVERSING)) {
                currentState = ExtakeState.IDLE;
            }
        }

        // Edge: if we changed shooting mode or exited shooting, reset PID for cleaner response
        if (prevState != currentState && (currentState == ExtakeState.SHOOTING_LOW || currentState == ExtakeState.SHOOTING_HIGH)) {
            velocityPID.reset();
        }
        if (prevState != currentState && currentState == ExtakeState.IDLE) {
            velocityPID.reset();
        }

        pressingShootLow = shootLow;
        pressingShootHigh = shootHigh;

        // Update measured velocity (manual derivative of encoder position)
        long now = System.nanoTime();
        int currentPosition = motorExtake.getCurrentPosition();
        if (lastTimeNs >= 0) {
            long dtNs = now - lastTimeNs;
            if (dtNs > 0) {
                double dtSec = dtNs / 1e9;
                int deltaPos = currentPosition - lastPosition;
                measuredVelocity = deltaPos / dtSec; // ticks/sec
            }
        }
        lastTimeNs = now;
        lastPosition = currentPosition;

        switch (currentState) {
            case REVERSING: {
                motorExtake.setPower(-1.0); // Full reverse purge
                break;
            }
            case SHOOTING_LOW: {
                velocityPID.setSetpoint(velocityLow);
                double pidOut = velocityPID.calculate(measuredVelocity, (now / 1e9)); // seconds
                double ff = velocityLow * KF; // simple feedforward
                double power = ff + pidOut;
                // Clamp power
                if (power > 1.0) power = 1.0;
                if (power < 0.0) power = 0.0; // don't allow negative during forward shoot
                motorExtake.setPower(power);
                break;
            }
            case SHOOTING_HIGH: {
                velocityPID.setSetpoint(velocityHigh);
                double pidOut = velocityPID.calculate(measuredVelocity, (now / 1e9));
                double ff = velocityHigh * KF;
                double power = ff + pidOut;
                if (power > 1.0) power = 1.0;
                if (power < 0.0) power = 0.0;
                motorExtake.setPower(power);
                break;
            }
            case IDLE: {
                // Minimal power or full stop; ensure PID not driving
                motorExtake.setPower(0.0);
                break;
            }
        }
    }

    // Direct manual override (still raw power)
    public void update(double power) {
        motorExtake.setPower(power);
    }

    @Override
    public void stop() {
        motorExtake.setPower(0.0);
    }

    // === Helper methods for telemetry/tuning ===
    public double getMeasuredVelocity() { return measuredVelocity; }

    public double getTargetVelocity() {
        switch (currentState) {
            case SHOOTING_LOW: return velocityLow;
            case SHOOTING_HIGH: return velocityHigh;
            default: return 0.0;
        }
    }

    public void setVelocityPID(double kP, double kI, double kD) { velocityPID.setPID(kP, kI, kD); }
}
