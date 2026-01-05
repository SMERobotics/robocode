package org.technodot.ftc.twentyfivebeta.targets;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.technodot.ftc.twentyfivebeta.BaboAuto;
import org.technodot.ftc.twentyfivebeta.Configuration;
import org.technodot.ftc.twentyfivebeta.common.Alliance;

@Autonomous(name="BaboAuto/RED", group="_")
public class BaboAutoRed extends BaboAuto {
    @Override
    public void config() {
        alliance = Alliance.RED;
        Configuration.DEBUG = false;
    }
}
