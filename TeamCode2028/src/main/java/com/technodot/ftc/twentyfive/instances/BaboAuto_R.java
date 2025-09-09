package com.technodot.ftc.twentyfive.instances;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.technodot.ftc.twentyfive.BaboAuto;
import com.technodot.ftc.twentyfive.Team;

@Autonomous(name="BaboAuto_R", group="_prod")
public class BaboAuto_R extends BaboAuto {
    public BaboAuto_R() {
        this.team = Team.RED;
    }
}
