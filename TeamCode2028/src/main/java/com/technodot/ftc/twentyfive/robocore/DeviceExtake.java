package com.technodot.ftc.twentyfive.robocore;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technodot.ftc.twentyfive.common.Controls;

public class DeviceExtake extends Device {
    public DcMotorEx motorExtake;
    public ExtakeState currentState = ExtakeState.IDLE;

    private boolean pressingShootLow;
    private boolean pressingShootHigh;

    public final double velocityHigh = 1420; // target ticks/sec (tune as needed)
    public final double velocityLow = 1200;  // target ticks/sec (tune as needed)

    // Autonomous can request a specific target velocity, but we pass it straight to the SDK.
    private boolean velocityOverride = false;
    private double overrideVelocity = 0.0; // ticks/sec

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
        // Use built-in velocity control so setVelocity(1160) behaves as expected.
        motorExtake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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

        pressingShootLow = shootLow;
        pressingShootHigh = shootHigh;

        update();
    }

    public void update() {
        switch (currentState) {
            case REVERSING: {
                motorExtake.setPower(-1.0); // Full reverse purge
                break;
            }
            case SHOOTING_LOW: {
                double target = velocityOverride ? overrideVelocity : velocityLow;
                // SDK expects ticks/second in RUN_USING_ENCODER mode
                motorExtake.setVelocity(target);
                break;
            }
            case SHOOTING_HIGH: {
                double target = velocityOverride ? overrideVelocity : velocityHigh;
                motorExtake.setVelocity(target);
                break;
            }
            case IDLE: {
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

    // === Autonomous helper ===
    // Set extake state directly (e.g., from autonomous routines) bypassing gamepad toggle logic.
    // Call before or between update() loops; update() will maintain the state unless gamepad buttons are pressed.
    public void setState(ExtakeState state) {
        currentState = state;
        // Clear toggle latch booleans so manual toggle logic won't immediately flip state.
        pressingShootLow = false;
        pressingShootHigh = false;
    }

    // === Velocity override API ===
    public void setVelocityOverride(double ticksPerSec) {
        velocityOverride = true;
        overrideVelocity = ticksPerSec;
    }
    public void clearVelocityOverride() {
        velocityOverride = false;
    }
}
