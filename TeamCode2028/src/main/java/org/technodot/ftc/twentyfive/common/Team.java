package org.technodot.ftc.twentyfive.common;

public enum Team {
    BLUE,
    RED;

    public int apply(int value) {
        return this == BLUE ? value : -value;
    }

    public double apply(double value) {
        return this == BLUE ? value : -value;
    }

    public float apply(float value) {
        return this == BLUE ? value : -value;
    }
}
