package org.technodot.ftc.twentyfivebeta;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Configuration {
    public static volatile boolean DEBUG = true; // all targets override debug to false (should)

    public static volatile double DRIVE_CONTROLLER_DEADZONE = 0.02;
    public static volatile int DRIVE_MOTOR_FORWARD_TILE_TICKS = 0; // TEMP
    public static volatile int DRIVE_MOTOR_STRAFE_TILE_TICKS = 0; // TEMP
    public static volatile int DRIVE_MOTOR_ROTATE_CIRCLE_TICKS = 0; // TEMP
    public static volatile double DRIVE_AUTO_MAX_VELOCITY = 670;
    public static volatile double DRIVE_MOTOR_ACTIVATION = 0.2;
    public static volatile double DRIVE_SPEED_MULTIPLIER = 1.0;
    public static volatile double DRIVE_STRAFE_MULTIPLIER = 1.1; // 1.10 +/- 0.01 idfk? Y TF NOT
//    public static volatile double DRIVE_STRAFE_MULTIPLIER = 1.0; // TODO: fuck it just test it
    public static volatile double DRIVE_AIM_KP = 0.025; // prev 0.0267
    public static volatile double DRIVE_AIM_KI = 0.05;
    public static volatile double DRIVE_AIM_KD = 0.0;
    public static volatile double DRIVE_AIM_KF = 0.0;
//    public static volatile double DRIVE_AIM_KD = 0.000067;
    public static volatile double DRIVE_AIM_TOLERANCE = 1.0;
    public static volatile double DRIVE_ROTATE_KP = 0.03;
    public static volatile double DRIVE_ROTATE_KI = 0.0;
    public static volatile double DRIVE_ROTATE_KD = 0.0001;
    public static volatile double DRIVE_ROTATE_KF = 0.0;
//    public static volatile double DRIVE_ROTATE_TOLERANCE = 1.0;
    public static volatile long DRIVE_ROTATE_SNAPSHOT_DELAY_NS = 100_000000L;

    public static volatile double INTAKE_MOTOR_NUDGE_POWER = 0.67;
    public static volatile int INTAKE_MOTOR_NUDGE_TICKS = 670;
    public static volatile double INTAKE_LEFT_ACTIVATION = 0.46;
    public static volatile double INTAKE_LEFT_HOLD = 0.30;
    public static volatile double INTAKE_LEFT_DEACTIVATION = 0.24;
    public static volatile double INTAKE_RIGHT_ACTIVATION = 0.50;
    public static volatile double INTAKE_RIGHT_HOLD = 0.66;
    public static volatile double INTAKE_RIGHT_DEACTIVATION = 0.72;
    public static volatile long INTAKE_SERVO_INTERVAL_MS = 200;
//    public static volatile long INTAKE_SERVO_DELAY_MS = 500; // global delay for now, PREVIOUSLY WAS 670MS
    public static volatile long INTAKE_SERVO_SHORT_DELAY_MS = 500;
    public static volatile long INTAKE_SERVO_LONG_DELAY_MS = 670;

    public static volatile double EXTAKE_MOTOR_KP = 300.0;
    public static volatile double EXTAKE_MOTOR_KI = 0.0;
    public static volatile double EXTAKE_MOTOR_KD = 1.0; // TODO: *maybe* increase?
    public static volatile double EXTAKE_MOTOR_KF = 17.767;
    public static volatile double EXTAKE_MOTOR_SPEED_SHORT = 1200; // TODO: adjust
    public static volatile double EXTAKE_MOTOR_SPEED_LONG = 1460; // TODO: adjust
    public static volatile double EXTAKE_MOTOR_SPEED_TOLERANCE = 20;
    public static volatile double EXTAKE_MOTOR_SUPER_FEEDFORWARD_THRESHOLD = 60;
//    public static volatile int EXTAKE_STABILIZATION_CYCLES = 4;
    public static volatile int EXTAKE_STABILIZATION_CYCLES = 1; // TODO: test zero because human and intake latency should be enough time
    // y (ticks/sec) = 340.130269893 x (m/sec) - 783.609122293 @ r = 0.9969, R^2 = 0.9937
    public static volatile double EXTAKE_MODEL_VELOCITY_M = 340.130269893;
    public static volatile double EXTAKE_MODEL_VELOCITY_B = -783.609122293;
    // y (deg) = -9.04083 x (m/sec) + 109.38517 @ r = -0.9975, R^2 = 0.9950
    public static volatile double EXTAKE_MODEL_ANGLE_M = -9.04083;
    public static volatile double EXTAKE_MODEL_ANGLE_B = 109.38517;

    public static volatile int GAMEPAD_RUMBLE_DURATION_MS = 200; // rev ps4 gamepad is the worst gamepad ever, negative build quality, need the dualshock 5
}
