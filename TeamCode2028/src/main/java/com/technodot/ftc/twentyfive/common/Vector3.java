package com.technodot.ftc.twentyfive.common;

public class Vector3 {
    public final double x;
    public final double y;
    public final double z;

    public Vector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    @Override
    public String toString() {
        return String.format("(%.5f, %.5f, %.5f)", x, y, z);
    }
}
