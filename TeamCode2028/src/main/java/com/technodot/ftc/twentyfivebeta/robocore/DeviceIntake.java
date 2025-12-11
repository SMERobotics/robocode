package com.technodot.ftc.twentyfivebeta.robocore;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technodot.ftc.twentyfivebeta.common.Alliance;
import com.technodot.ftc.twentyfivebeta.roboctrl.InputController;

public class DeviceIntake extends Device {

    public RevColorSensorV3 colorLeft1;
    public RevColorSensorV3 colorLeft2;
    public RevColorSensorV3 colorRight1;
    public RevColorSensorV3 colorRight2;

    public DeviceIntake(Alliance alliance) {
        super(alliance);
    }

    @Override
    public void init(HardwareMap hardwareMap, InputController inputController) {
        colorLeft1 = hardwareMap.get(RevColorSensorV3.class, "colorLeft1");
        colorLeft2 = hardwareMap.get(RevColorSensorV3.class, "colorLeft2");
//        colorRight1 = hardwareMap.get(RevColorSensorV3.class, "colorRight1");
//        colorRight2 = hardwareMap.get(RevColorSensorV3.class, "colorRight2");
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        // TODO
    }

    @Override
    public void stop() {

    }
}
