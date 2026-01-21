package org.technodot.ftc.twentyfivebeta.common;

public class Movement {
    public double forward;
    public double strafe;
    public double rotate;
    public double velocity;

    public Movement(double forward, double strafe, double rotate) {
        this(forward, strafe, rotate, Double.NaN);
    }

    public Movement(double forward, double strafe, double rotate, double velocity) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotate = rotate;
        this.velocity = velocity;
    }
}
