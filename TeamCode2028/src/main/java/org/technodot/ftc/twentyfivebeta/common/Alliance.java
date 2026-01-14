package org.technodot.ftc.twentyfivebeta.common;

public enum Alliance {
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
