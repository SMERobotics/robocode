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
        return ready() ? gamepad1.right_trigger : 0.0f;
    }

    public boolean intakeReverse() {
        return ready() && gamepad1.x;
    }

    public boolean extakeVelocityIncrease() {
        return ready() && gamepad1.dpad_up;
    }

    public boolean extakeVelocityDecrease() {
        return ready() && gamepad1.dpad_down;
    }

    public boolean extakeSpinUp() {
        return ready() && gamepad1.y;
    }

    public void vibrateExtakeReady(int durationMillis) {
        if (gamepad1 != null) gamepad1.rumble(durationMillis);
    }

    public boolean extakePreset1520() {
        return ready() && gamepad2.a;
    }

    public boolean extakePreset1560() {
        return ready() && gamepad2.b;
    }

    public boolean extakePreset1600() {
        return ready() && gamepad2.x;
    }

    public boolean extakePreset1780() {
        return ready() && gamepad2.y;
    }

    public boolean extakePresetCancel() {
        return ready() && (gamepad2.dpad_up || gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.dpad_down);
    }

    public boolean sequenceShoot() {
        return ready() && gamepad1.right_bumper;
    }

    public boolean flowerUp() {
        return ready() && gamepad1.left_trigger > 0.0f;
    }

    public boolean extakePollen() {
        return ready() && gamepad1.a;
    }

    public boolean extakeNectar() {
        return ready() && gamepad1.b;
    }

}
