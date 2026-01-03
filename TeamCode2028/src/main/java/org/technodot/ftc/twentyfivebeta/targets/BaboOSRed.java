package org.technodot.ftc.twentyfivebeta.targets;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.technodot.ftc.twentyfivebeta.BaboOS;
import org.technodot.ftc.twentyfivebeta.Configuration;
import org.technodot.ftc.twentyfivebeta.common.Alliance;

@TeleOp(name="BaboOS/RED", group="prod")
public class BaboOSRed extends BaboOS {
    @Override
    public void config() {
        alliance = Alliance.RED;
        Configuration.DEBUG = false;
    }
}
