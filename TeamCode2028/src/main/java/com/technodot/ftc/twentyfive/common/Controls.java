package com.technodot.ftc.twentyfive.common;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Centralized control map for the driver station gamepads.
 *
 * <p>All button/axis bindings should be defined here so they can be
 * rebound from one location without touching the subsystems.</p>
 */
public final class Controls {

    private static final float DRIVE_FORWARD_SCALE = -1.0f;
    private static final float DRIVE_STRAFE_SCALE = 1.0f;
    private static final float DRIVE_ROTATE_SCALE = 1.0f;

    private Controls() {
        // no instances
    }

    public static float driveForward(Gamepad gamepad) {
        return gamepad == null ? 0f : DRIVE_FORWARD_SCALE * gamepad.left_stick_y;
    }

    public static float driveStrafe(Gamepad gamepad) {
        return gamepad == null ? 0f : DRIVE_STRAFE_SCALE * gamepad.left_stick_x;
    }

    public static float driveRotate(Gamepad gamepad) {
        return gamepad == null ? 0f : DRIVE_ROTATE_SCALE * gamepad.right_stick_x;
    }

    public static boolean drivePrecise(Gamepad gamepad) {
        return gamepad != null && gamepad.y;
    }

    public static boolean intakeIn(Gamepad gamepad) {
        return gamepad != null && gamepad.right_trigger > 0.1;
    }

    public static boolean intakeOut(Gamepad gamepad) {
        return gamepad != null && gamepad.left_trigger > 0.1;
    }

    public static boolean intakeServoLeft(Gamepad gamepad) {
        return gamepad != null && gamepad.dpad_left;
    }

    public static boolean intakeServoRight(Gamepad gamepad) {
        return gamepad != null && gamepad.dpad_right;
    }

    public static boolean intakeServoGreen(Gamepad gamepad) {
        return gamepad != null && gamepad.left_bumper;
    }

    public static boolean intakeServoPurple(Gamepad gamepad) {
        return gamepad != null && gamepad.right_bumper;
    }

    public static boolean extakeShootLow(Gamepad gamepad) {
        return gamepad != null && (gamepad.a || gamepad.dpad_up);
    }

    public static boolean extakeShootHigh(Gamepad gamepad) {
        return gamepad != null && gamepad.y;
    }

    public static boolean extakeShootReverse(Gamepad gamepad) {
        return gamepad != null && gamepad.dpad_down;
    }
}
