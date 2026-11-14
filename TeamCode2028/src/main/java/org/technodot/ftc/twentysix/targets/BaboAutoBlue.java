package org.technodot.ftc.twentysix.targets;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.technodot.ftc.twentysix.BaboAuto;
import org.technodot.ftc.twentysix.common.Alliance;

@Autonomous(name="BaboAuto/BLUE", group="TechnoCode", preselectTeleOp="BaboOS/BLUE")
public class BaboAutoBlue extends BaboAuto {
    @Override
    protected void config() { alliance = Alliance.BLUE; }
}
