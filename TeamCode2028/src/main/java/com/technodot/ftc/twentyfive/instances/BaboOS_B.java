package com.technodot.ftc.twentyfive.instances;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.technodot.ftc.twentyfive.BaboOS;
import com.technodot.ftc.twentyfive.Team;

@TeleOp(name="BaboOS_B", group="_prod")
public class BaboOS_B extends BaboOS {
    public BaboOS_B() {
        this.team = Team.BLUE;
    }
}
