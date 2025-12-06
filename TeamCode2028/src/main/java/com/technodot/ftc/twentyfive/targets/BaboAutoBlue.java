package com.technodot.ftc.twentyfive.targets;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.technodot.ftc.twentyfive.BaboAuto;
import com.technodot.ftc.twentyfive.common.Team;

@Disabled
@Autonomous(name = "BaboAuto Blue", group = "TechnoCode")
public class BaboAutoBlue extends BaboAuto {
    @Override
    public void config() {
        team = Team.BLUE;
    }
}
