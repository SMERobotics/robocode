package com.technodot.ftc.twentyfive.robocore;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class Device {
    public abstract void init(HardwareMap hardwareMap);

    public abstract void start();

    public abstract void update(Gamepad gamepad);

    public abstract void stop();
}
