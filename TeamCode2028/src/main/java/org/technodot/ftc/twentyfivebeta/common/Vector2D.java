package org.technodot.ftc.twentyfivebeta.common;

public class Vector2D {
    public double x;
    public double y;

    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double magnitude() {
        return Math.sqrt(x * x + y * y);
    }

    public Vector2D rotate(double thetaRad) {
        double skibidi = this.x;
        this.x = x * Math.cos(thetaRad) - y * Math.sin(thetaRad);
        this.y = skibidi * Math.sin(thetaRad) + y * Math.cos(thetaRad);
        return this;
    }

}
