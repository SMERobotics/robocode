package org.technodot.ftc.twentyfivebeta;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Configuration {
    public static volatile double DRIVE_CONTROLLER_DEADZONE = 0.02;
    public static volatile double DRIVE_MOTOR_ACTIVATION = 0.2;
    public static volatile double DRIVE_SPEED_MULTIPLIER = 1.0;
    public static volatile double DRIVE_STRAFE_MULTIPLIER = 1.0;
    public static volatile double DRIVE_AIM_KP = 0.013;
    public static volatile double DRIVE_AIM_KI = 0.0;
    public static volatile double DRIVE_AIM_KD = 0.006;
    public static volatile double DRIVE_AIM_TOLERANCE = 1.0;

    public static volatile double INTAKE_LEFT_ACTIVATION = 0.40;
    public static volatile double INTAKE_LEFT_DEACTIVATION = 0.26;
    public static volatile double INTAKE_RIGHT_ACTIVATION = 0.40;
    public static volatile double INTAKE_RIGHT_DEACTIVATION = 0.6;
    public static volatile long INTAKE_SERVO_INTERVAL_MS = 200;

    public static volatile double EXTAKE_MOTOR_KP = 0.01;
    public static volatile double EXTAKE_MOTOR_KI = 0.0;
    public static volatile double EXTAKE_MOTOR_KD = 0.0005;
    public static volatile double EXTAKE_MOTOR_KF = 0.12;
    public static volatile double EXTAKE_MOTOR_SPEED_SHORT = 1200;
    public static volatile double EXTAKE_MOTOR_SPEED_LONG = 1460;
}
