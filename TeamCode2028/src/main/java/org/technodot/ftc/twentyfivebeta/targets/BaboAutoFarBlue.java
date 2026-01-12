package org.technodot.ftc.twentyfivebeta.targets;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.technodot.ftc.twentyfivebeta.BaboAuto;
import org.technodot.ftc.twentyfivebeta.Configuration;
import org.technodot.ftc.twentyfivebeta.common.Alliance;

@Autonomous(name="BaboAuto/FAR/BLUE", group="_", preselectTeleOp="BaboOS/BLUE")
public class BaboAutoFarBlue extends BaboAuto {
    @Override
    public void config() {
        alliance = Alliance.BLUE;
        autoType = AutoType.FAR;
        Configuration.DEBUG = false;
    }
}
