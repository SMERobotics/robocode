package org.technodot.ftc.twentysix.targets;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.technodot.ftc.twentysix.BaboAuto;
import org.technodot.ftc.twentysix.common.Alliance;

@Autonomous(name="BaboAuto/RED", group="TechnoCode", preselectTeleOp="BaboOS/RED")
public class BaboAutoRed extends BaboAuto {
    @Override
    protected void config() { alliance = Alliance.RED; }
}
