package com.technodot.ftc.twentyfive.targets;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.technodot.ftc.twentyfive.BaboOS;
import com.technodot.ftc.twentyfive.common.Team;

@Disabled
@TeleOp(name = "BaboOS Red", group = "TechnoCode")
public class BaboOSRed extends BaboOS {
    @Override
    public void config() {
        team = Team.RED;
    }
}
