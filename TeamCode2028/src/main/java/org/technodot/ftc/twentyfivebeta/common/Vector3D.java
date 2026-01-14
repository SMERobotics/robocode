package org.technodot.ftc.twentyfivebeta.common;

public class Vector3D {

    // X+: right
    // Y+: forward
    // Z+: up

    public double x;
    public double y;
    public double z;

    public Vector3D(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vector3D(Vector3D vector) {
        this.x = vector.x;
        this.y = vector.y;
        this.z = vector.z;
    }

    public double magnitude() {
        return Math.sqrt(x * x + y * y + z * z);
    }
}
