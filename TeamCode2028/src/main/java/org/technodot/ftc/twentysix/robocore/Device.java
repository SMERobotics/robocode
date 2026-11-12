package org.technodot.ftc.twentysix.robocore;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.technodot.ftc.twentysix.common.Alliance;
import org.technodot.ftc.twentysix.roboctrl.GamepadBindings;

public abstract class Device {

    protected GamepadBindings gamepad;
    protected Telemetry telemetry;

    protected static Alliance alliance = Alliance.BLUE;

    public Device(Alliance alliance) {
        Device.alliance = alliance;
    }

    public void init(HardwareMap hardwareMap, GamepadBindings gamepad, Telemetry telemetry) {
        this.gamepad = gamepad;
        this.telemetry = telemetry;
    }

    public abstract void start();

    public abstract void update();

    public abstract void stop();
}
