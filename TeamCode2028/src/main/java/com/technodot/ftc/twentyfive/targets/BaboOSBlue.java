package com.technodot.ftc.twentyfive.targets;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.technodot.ftc.twentyfive.BaboOS;
import com.technodot.ftc.twentyfive.common.Team;

@TeleOp(name = "BaboOS Blue", group = "TechnoCode")
public class BaboOSBlue extends BaboOS {
    @Override
    public void config() {
        team = Team.BLUE;
    }
}
