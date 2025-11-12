package com.technodot.ftc.twentyfive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.technodot.ftc.twentyfive.common.Team;

@TeleOp(name = "BaboOS Red", group = "TechnoCode")
public class BaboOSRed extends BaboOS {
    @Override
    public void config() {
        team = Team.RED;
    }
}

