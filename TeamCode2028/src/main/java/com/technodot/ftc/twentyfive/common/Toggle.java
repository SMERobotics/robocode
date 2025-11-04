package com.technodot.ftc.twentyfive.common;

public class Toggle {
    private boolean state;
    private boolean lastInput;

    public Toggle() {
        this.state = false;
        this.lastInput = false;
    }

    public void update(boolean input) {
        if (input && !lastInput) {
            state = !state;
        }
        lastInput = input;
    }

    public boolean getState() {
        return state;
    }
}
