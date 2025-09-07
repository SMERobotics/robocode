package com.technodot.ftc.twentyfour.autotask;

import com.technodot.ftc.twentyfour.robocore.Device;

public class AutoObjective {
    ObjectiveType type;
    Device device;
    boolean status;

    public AutoObjective(ObjectiveType objectiveType, Device targetDevice) {
        type = objectiveType;
        device = targetDevice;
    }

    public AutoObjective(ObjectiveType objectiveType, Device targetDevice, boolean status) {
        type = objectiveType;
        device = targetDevice;
    }
}
