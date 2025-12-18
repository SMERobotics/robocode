package org.technodot.ftc.twentyfivebeta.roboctrl;

import com.qualcomm.robotcore.hardware.Gamepad;

public class InputController {

    protected final Gamepad gamepad1;
    protected final Gamepad gamepad2;

    public InputController(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public boolean ready() {
        return gamepad1 != null && gamepad2 != null;
    }
}
