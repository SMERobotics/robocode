package com.technodot.ftc.twentyfivebeta.robocore;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technodot.ftc.twentyfivebeta.common.Alliance;
import com.technodot.ftc.twentyfivebeta.roboctrl.InputController;

public abstract class Device {

    protected InputController inputController;

    protected final Alliance alliance;

    public Device(Alliance alliance) {
        this.alliance = alliance;
    }

    public abstract void init(HardwareMap hardwareMap, InputController inputController);

    public abstract void start();

    public abstract void update();

    public abstract void stop();
}
