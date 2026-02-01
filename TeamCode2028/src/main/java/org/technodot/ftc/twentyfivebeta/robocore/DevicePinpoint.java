package org.technodot.ftc.twentyfivebeta.robocore;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.technodot.ftc.twentyfivebeta.Configuration;
import org.technodot.ftc.twentyfivebeta.common.Alliance;
import org.technodot.ftc.twentyfivebeta.pedro.Follower;
import org.technodot.ftc.twentyfivebeta.roboctrl.InputController;

public class DevicePinpoint extends Device {

    public Follower follower;

    public DevicePinpoint(Alliance alliance) {
        super(alliance);
    }

    @Override
    public void init(HardwareMap hardwareMap, InputController inputController) {
        follower = (Follower) Configuration.createFollower(hardwareMap);
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        follower.update();
    }

    @Override
    public void stop() {

    }
}
