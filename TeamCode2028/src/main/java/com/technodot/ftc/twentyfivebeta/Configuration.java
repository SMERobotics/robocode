package com.technodot.ftc.twentyfivebeta;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Configuration {
    public static volatile double DRIVE_CONTROLLER_DEADZONE = 0.02f;
    public static volatile double DRIVE_MOTOR_ACTIVATION = 0.2f;
    public static volatile double DRIVE_SPEED_MULTIPLIER = 1.0f;
    public static volatile double DRIVE_STRAFE_MULTIPLIER = 1.0f;

    public static volatile double EXTAKE_MOTOR_SPEED_SHORT = 1200;
    public static volatile double EXTAKE_MOTOR_SPEED_LONG = 1460;
}
