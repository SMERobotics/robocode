package org.technodot.ftc.twentyfivebeta.targets;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.technodot.ftc.twentyfivebeta.BaboAuto;
import org.technodot.ftc.twentyfivebeta.Configuration;
import org.technodot.ftc.twentyfivebeta.common.Alliance;

@Autonomous(name="BaboAuto/CLOSE/BLUE", group="_")
public class BaboAutoCloseBlue extends BaboAuto {
    @Override
    public void config() {
        alliance = Alliance.BLUE;
        autoType = AutoType.CLOSE;
        Configuration.DEBUG = false;
    }
}
