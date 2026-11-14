package org.technodot.ftc.twentysix.targets;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.technodot.ftc.twentysix.BaboOS;
import org.technodot.ftc.twentysix.common.Alliance;

@TeleOp(name="BaboOS/RED", group="TechnoCode")
public class BaboOSRed extends BaboOS {
    @Override
    protected void config() { alliance = Alliance.RED; }
}
