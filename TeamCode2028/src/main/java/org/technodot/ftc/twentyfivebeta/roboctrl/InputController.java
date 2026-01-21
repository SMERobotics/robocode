package org.technodot.ftc.twentyfivebeta.roboctrl;

import com.bylazar.gamepad.PanelsGamepad;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.technodot.ftc.twentyfivebeta.Configuration;

public class InputController {

    protected final Gamepad gamepad1;
    protected final Gamepad gamepad2;

    public InputController(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = Configuration.DEBUG ? PanelsGamepad.INSTANCE.getFirstManager().asCombinedFTCGamepad(gamepad1) : gamepad1;
        this.gamepad2 = Configuration.DEBUG ? PanelsGamepad.INSTANCE.getSecondManager().asCombinedFTCGamepad(gamepad2) : gamepad2;
    }

    public boolean ready() {
        return gamepad1 != null && gamepad2 != null;
    }
}
