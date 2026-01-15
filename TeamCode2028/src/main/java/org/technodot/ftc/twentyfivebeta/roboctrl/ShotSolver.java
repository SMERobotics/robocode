package org.technodot.ftc.twentyfivebeta.roboctrl;

import org.technodot.ftc.twentyfivebeta.Configuration;
import org.technodot.ftc.twentyfivebeta.common.Vector3D;

import java.util.ArrayDeque;
import java.util.Queue;

public class ShotSolver {

    private static final Queue<Double> ranger = new ArrayDeque<>();
    private static double sum;

    // X+: right
    // Y+: forward
    // Z+: up

    public static final Vector3D GOAL_OFFSET = new Vector3D(0, 12, 9.25); // (x, y, z) inches
    public static final Vector3D EXTAKE_OFFSET = traceRBE(new Vector3D(0, -11, 0)); // (x, y, z) inches

    /**
     * Trace from range, bearing, elevation to x, y, z
     * @param rangeIn range, in inches
     * @param bearingRad bearing, in radians
     * @param pitchRad elevation, in radians
     * @return Vector3D (x, y, z) in inches
     */
    public static Vector3D traceXYZ(double rangeIn, double bearingRad, double pitchRad) {
        double distXY = rangeIn * Math.cos(pitchRad);
        double distZ = rangeIn * Math.sin(pitchRad);
        double distX = distXY * Math.sin(bearingRad);
        double distY = distXY * Math.cos(bearingRad);
        return new Vector3D(distX, distY, distZ);
    }

    /**
     * Trace from x, y, z to range, bearing, elevation
     * @param dist (x, y, z) in inches
     * @return Vector3D (range, bearing, elevation)
     */
    public static Vector3D traceRBE(Vector3D dist) {
        double distX = dist.x;
        double distY = dist.y;
        double distZ = dist.z;

        double rangeIn = Math.sqrt(distX * distX + distY * distY + distZ * distZ);
        double bearingRad = Math.atan2(distX, distY);
        double pitchRad = Math.asin(distZ / rangeIn);

        return new Vector3D(rangeIn, bearingRad, pitchRad);
    }

    /**
     * Project the goal offset and the extake offset onto the tag position and get the bearing angle.
     * @param tagPosition position of the tag in (x, y, z) inches
     * @param yawDeg robot yaw in degrees
     * @return bearing angle in degrees
     */
    public static double projectGoal(Vector3D tagPosition, double yawDeg) {
        Vector3D goalRBE = new Vector3D(GOAL_OFFSET);
        goalRBE.y += Math.toRadians(yawDeg); // y is bearing in (range, bearing, elevation) // MIGHT BE -= INSTEAD IDK TEST!!!
        Vector3D goalXYZ = traceXYZ(goalRBE.x, goalRBE.y, goalRBE.z);

        return Math.toDegrees(traceRBE(new Vector3D(
                tagPosition.x + goalXYZ.x - EXTAKE_OFFSET.x,
                tagPosition.y + goalXYZ.y - EXTAKE_OFFSET.y,
                tagPosition.z + goalXYZ.z - EXTAKE_OFFSET.z
        )).y);
    }

    public static double calculateLaunchVelocity(double rangeIn) {
        sum += rangeIn;
        ranger.add(rangeIn);
        if (ranger.size() > 10) sum -= ranger.remove();
        double range = sum / ranger.size();

        return Configuration.EXTAKE_MODEL_VELOCITY_SIMPLE_A * range * range
                + Configuration.EXTAKE_MODEL_VELOCITY_SIMPLE_B * range
                + Configuration.EXTAKE_MODEL_VELOCITY_SIMPLE_C;
    }

    public static void clearVelocityQueue() {
        ranger.clear();
        sum = 0;
    }
}
