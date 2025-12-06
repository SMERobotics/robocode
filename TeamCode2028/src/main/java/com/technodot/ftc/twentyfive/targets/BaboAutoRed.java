package com.technodot.ftc.twentyfive.targets;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.technodot.ftc.twentyfive.BaboAuto;
import com.technodot.ftc.twentyfive.common.Team;

@Disabled
@Autonomous(name = "BaboAuto Red", group = "TechnoCode")
public class BaboAutoRed extends BaboAuto {
    @Override
    public void config() {
        team = Team.RED;
    }
}
