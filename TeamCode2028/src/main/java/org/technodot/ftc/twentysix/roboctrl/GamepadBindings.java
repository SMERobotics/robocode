package org.technodot.ftc.twentysix.roboctrl;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadBindings {

    protected Gamepad gamepad1;
    protected Gamepad gamepad2;

    public GamepadBindings(Gamepad gamepad1, Gamepad gamepad2){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public boolean ready() {
        return gamepad1 != null && gamepad2 != null;
    }

    public float driveForward() {
        return ready() ? -gamepad1.left_stick_y : 0.0f;
    }

    public float driveStrafe() {
        return ready() ? gamepad1.left_stick_x : 0.0f;
    }

    public float driveRotate() {
        return ready() ? gamepad1.right_stick_x : 0.0f;
    }

    public float intakePower() {
        return ready() ? gamepad1.left_trigger : 0.0f;
    }

    public float extakePower() {
        return ready() ? gamepad1.right_trigger : 0.0f;
    }

    public boolean transferOpen() {
        return ready() && gamepad1.right_bumper;
    }

    public boolean transferClose() {
        return ready() && gamepad1.left_bumper;
    }

    public boolean turretHeadingIncrease() {
        return ready() && gamepad1.dpad_up;
    }

    public boolean turretHeadingDecrease() {
        return ready() && gamepad1.dpad_down;
    }

    public boolean extakePollen() {
        return ready() && gamepad1.a;
    }

    public boolean extakeNectar() {
        return ready() && gamepad1.b;
    }

    public boolean turretReset() {
        return ready() && gamepad1.start;
    }
}
