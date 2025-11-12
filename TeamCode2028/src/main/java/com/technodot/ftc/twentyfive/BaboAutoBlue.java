package com.technodot.ftc.twentyfive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.technodot.ftc.twentyfive.common.Team;

@Autonomous(name = "BaboAuto Blue", group = "TechnoCode")
public class BaboAutoBlue extends BaboAuto {
    @Override
    public void config() {
        team = Team.BLUE;
    }
}