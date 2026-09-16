package org.technodot.ftc.twentyfivebeta.robocore;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.technodot.ftc.twentyfivebeta.common.Alliance;
import org.technodot.ftc.twentyfivebeta.roboctrl.InputController;

public abstract class Device {

    protected InputController inputController;

    protected static Alliance alliance = Alliance.BLUE;

    public Device(Alliance alliance) {
        Device.alliance = alliance;
    }

    public abstract void init(HardwareMap hardwareMap, InputController inputController);

    public abstract void start();

    public abstract void update();

    public abstract void stop();
}
