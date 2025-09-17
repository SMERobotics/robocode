package com.technodot.ftc.twentyfive.common;

public class TimedVector3 {
    public double x;
    public double y;
    public double z;
    public long time;

    public TimedVector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.time = System.nanoTime();
    }

    public TimedVector3(double x, double y, double z, long time) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.time = time;
    }

    @Override
    public String toString() {
        return String.format("(%.5f, %.5f, %.5f) @ %d", x, y, z, time);
    }
}
