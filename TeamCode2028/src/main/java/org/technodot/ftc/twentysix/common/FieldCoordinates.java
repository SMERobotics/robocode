package org.technodot.ftc.twentysix.common;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;

/** Inches from the audience-left corner; +X toward BLUE, +Y toward the rear wall. */
public final class FieldCoordinates {
    public static final double FIELD_SIZE = 144;

    private FieldCoordinates() { }

    /** Robot rear touches its starting wall, facing into the field. BLUE is a 180-degree rotation of RED. */
    public static Pose startPose(Alliance alliance, double allianceWallOffset, double startingWallOffset) {
        if (alliance == Alliance.RED) {
            return new Pose(allianceWallOffset, startingWallOffset, Math.PI / 2);
        }
        return new Pose(FIELD_SIZE - allianceWallOffset, FIELD_SIZE - startingWallOffset, 3 * Math.PI / 2);
    }

    public static Pose moveRight(Pose pose, double inches) {
        return new Pose(pose.x() + inches * Math.sin(pose.heading()),
                pose.y() - inches * Math.cos(pose.heading()), pose.heading());
    }

    /** Result: X = robot forward power, Y = robot right power (the manual drivetrain convention). */
    public static Vector2D driverToRobot(Alliance alliance, double heading, double forward, double right) {
        double driverHeading = alliance == Alliance.RED ? 0 : Math.PI;
        double relativeHeading = heading - driverHeading;
        double c = Math.cos(relativeHeading), s = Math.sin(relativeHeading);
        return Vector2D.cartesian(forward * c - right * s, forward * s + right * c);
    }
}
