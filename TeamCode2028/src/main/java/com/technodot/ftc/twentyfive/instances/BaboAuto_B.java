package com.technodot.ftc.twentyfive.instances;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.technodot.ftc.twentyfive.BaboAuto;
import com.technodot.ftc.twentyfive.Team;

@Autonomous(name="BaboAuto_B", group="_prod")
public class BaboAuto_B extends BaboAuto {
    public BaboAuto_B() {
        this.team = Team.BLUE;
    }
}
