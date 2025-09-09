package com.technodot.ftc.twentyfive.instances;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.technodot.ftc.twentyfive.BaboOS;
import com.technodot.ftc.twentyfive.Team;

@TeleOp(name="BaboOS_R", group="_prod")
public class BaboOS_R extends BaboOS {
    public BaboOS_R() {
        this.team = Team.RED;
    }
}
