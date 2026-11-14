package org.technodot.ftc.twentysix.robocore;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.technodot.ftc.twentysix.common.Alliance;
import org.technodot.ftc.twentysix.roboctrl.GamepadBindings;

public abstract class Device {

    protected GamepadBindings gamepad;
    protected Telemetry telemetry;

    protected final Alliance alliance;

    public Device(Alliance alliance) {
        this.alliance = alliance;
    }

    public void init(HardwareMap hardwareMap, GamepadBindings gamepad, Telemetry telemetry) {
        this.gamepad = gamepad;
        this.telemetry = telemetry;
    }

    public abstract void start();

    public abstract void update();

    public abstract void stop();
}
