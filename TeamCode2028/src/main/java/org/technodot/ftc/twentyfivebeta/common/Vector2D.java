package org.technodot.ftc.twentyfivebeta.common;

public class Vector2D {
    public double x;
    public double y;

    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2D(Vector2D vector) {
        this.x = vector.x;
        this.y = vector.y;
    }

    public double magnitude() {
        return Math.hypot(x, y);
    }

    public Vector2D difference(Vector2D vector) {
        return new Vector2D(this.x - vector.x, this.y - vector.y);
    }

    public Vector2D rotate(double thetaRad) {
        double skibidi = this.x;
        this.x = x * Math.cos(thetaRad) - y * Math.sin(thetaRad);
        this.y = skibidi * Math.sin(thetaRad) + y * Math.cos(thetaRad);
        return this;
    }

}
