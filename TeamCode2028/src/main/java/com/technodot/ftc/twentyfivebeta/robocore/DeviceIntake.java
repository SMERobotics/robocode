package com.technodot.ftc.twentyfivebeta.robocore;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.technodot.ftc.twentyfive.common.Artifact;
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
        colorRight1 = hardwareMap.get(RevColorSensorV3.class, "colorRight1");
        colorRight2 = hardwareMap.get(RevColorSensorV3.class, "colorRight2");
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

    public static Artifact getArtifactColor(NormalizedRGBA color, double distanceCM) {
        // TODO: ask ts gpt-5.2 to write ts algorithm
        return Artifact.NONE;
    }

    public static Artifact combineArtifactColors(Artifact a, Artifact b) {
        // PURPLE over everything.
        // GREEN over NONE.
        // NONE only with NONE.

        if (a == Artifact.PURPLE || b == Artifact.PURPLE) {
            return Artifact.PURPLE;
        } else if (a == Artifact.GREEN || b == Artifact.GREEN) {
            return Artifact.GREEN;
        } else {
            return Artifact.NONE;
        }
    }
}
