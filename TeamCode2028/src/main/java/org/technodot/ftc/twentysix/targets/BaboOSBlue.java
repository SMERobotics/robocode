package org.technodot.ftc.twentysix.targets;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.technodot.ftc.twentysix.BaboOS;
import org.technodot.ftc.twentysix.common.Alliance;

@TeleOp(name="BaboOS/BLUE", group="TechnoCode")
public class BaboOSBlue extends BaboOS {
    @Override
    protected void config() { alliance = Alliance.BLUE; }
}
