package org.technodot.ftc.twentyfive.common;

public class Vector3 {
    public double x;
    public double y;
    public double z;

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
