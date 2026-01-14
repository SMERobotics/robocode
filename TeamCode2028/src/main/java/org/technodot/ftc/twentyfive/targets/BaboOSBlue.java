package org.technodot.ftc.twentyfive.targets;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.technodot.ftc.twentyfive.BaboOS;
import org.technodot.ftc.twentyfive.common.Team;

@Disabled
@TeleOp(name = "BaboOS Blue", group = "TechnoCode")
public class BaboOSBlue extends BaboOS {
    @Override
    public void config() {
        team = Team.BLUE;
    }
}
