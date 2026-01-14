package org.technodot.ftc.twentyfive.shotsolver;

import org.technodot.ftc.twentyfive.common.Team;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class ShotSolver {
    // Offsets of the desired goal point in the AprilTag's local (tag) frame, in inches.
    // Tag frame T is defined such that the tag lies on the XZ plane and its center is at the origin.
    private static final double GOAL_OFFSET_X_T = 0.740128657074; // inches
    private static final double GOAL_OFFSET_Y_T = 12.9696232669;  // inches
    private static final double GOAL_OFFSET_Z_T = 9.25;           // inches

    // Camera offset in the robot frame (inches). These locate the camera origin relative to
    // the same reference point used for range/bearing/elevation (typically the robot center).
    // +X_R: forward, +Y_R: left, +Z_R: up
    // Adjust these values to match your actual camera mounting.
    private static final double CAMERA_OFFSET_X_R = 0.0;
    private static final double CAMERA_OFFSET_Y_R = 0.0;
    private static final double CAMERA_OFFSET_Z_R = 0.0;

    public static RelativeRBE projectGoal(AprilTagDetection detection, Team team) {
        // angles are negated because cameraFront is upside down!
        return projectGoal(
                detection.ftcPose.range,
                -detection.ftcPose.bearing,
                -detection.ftcPose.elevation,
                -detection.ftcPose.yaw,
                team
        );
    }

    /**
     * Projects from a measured AprilTag polar observation to a goal point offset from the tag,
     * then converts that goal point back into range / bearing / elevation relative to the
     * original reference point.
     *
     * All angles are in degrees, distances in inches.
     *
     * Coordinate conventions (robot frame R):
     *  - +X_R: forward
     *  - +Y_R: left (bearing increases CCW when looking down on +Z_R)
     *  - +Z_R: up
     *
     * The AprilTag's own coordinate frame T is defined such that the tag lies on the XZ plane,
     * centered at the origin, and its orientation relative to the robot frame is given purely by
     * yaw (rotation about Z).
     *
     * @param range        distance from camera to the tag center (inches)
     * @param bearing      azimuth angle from camera +X_R toward +Y_R (degrees)
     * @param elevation    elevation angle from the camera X-Y plane toward +Z_R (degrees)
     * @param yaw          yaw of the tag about +Z_R (degrees). This must be used so the tag lies
     *                     flat on the XZ plane in its own frame.
     * @param team         what team are we on chat
     * @return RelativeRBE to the goal point, in inches/degrees.
     */
    public static RelativeRBE projectGoal(double range, double bearing, double elevation, double yaw, Team team) {
        // ---- Step 1: polar (range, bearing, elevation) -> Cartesian in camera frame (aligned with R) ----
        double bRad = Math.toRadians(bearing);
        double eRad = Math.toRadians(elevation);

        double rXY = range * Math.cos(eRad); // projection onto X-Y plane
        double zCam = range * Math.sin(eRad);  // vertical component

        double xCam = rXY * Math.cos(bRad);
        double yCam = rXY * Math.sin(bRad);

        // ---- Step 1b: optionally shift from camera origin to robot reference origin in R ----
        double xR = xCam;
        double yR = yCam;
        double zR = zCam;
        xR += CAMERA_OFFSET_X_R;
        yR += CAMERA_OFFSET_Y_R;
        zR += CAMERA_OFFSET_Z_R;

        // ---- Step 2: rotate into tag frame T using yaw so the tag lies on XZ plane ----
        double yawRad = Math.toRadians(yaw);
        double cosY = Math.cos(yawRad);
        double sinY = Math.sin(yawRad);

        // Robot -> Tag: R_R^T = Rz(-yaw)
        double xT =  cosY * xR + sinY * yR;
        double yT = -sinY * xR + cosY * yR;
        double zT =  zR; // yaw is about Z, so Z component is unchanged

        // ---- Step 3: add fixed offsets in tag frame to reach the goal point ----
        double xGoalT = xT + team.apply(GOAL_OFFSET_X_T);
        double yGoalT = yT + GOAL_OFFSET_Y_T;
        double zGoalT = zT + GOAL_OFFSET_Z_T;

        // ---- Step 4: transform goal point back to robot frame R ----
        // Tag -> Robot: R_T^R = Rz(+yaw)
        double xGoalR =  cosY * xGoalT - sinY * yGoalT;
        double yGoalR =  sinY * xGoalT + cosY * yGoalT;
        double zGoalR =  zGoalT;

        // ---- Step 5: convert Cartesian back to polar (range, bearing, elevation) ----
        double rangeOut = Math.sqrt(xGoalR * xGoalR + yGoalR * yGoalR + zGoalR * zGoalR);

        double rXYGoal = Math.sqrt(xGoalR * xGoalR + yGoalR * yGoalR);
        double eGoalRad = Math.atan2(zGoalR, rXYGoal);   // elevation from horizontal
        double bGoalRad = Math.atan2(yGoalR, xGoalR);    // bearing from +X_R toward +Y_R

        double bearingOut = Math.toDegrees(bGoalRad);
        double elevationOut = Math.toDegrees(eGoalRad);

        return new RelativeRBE(rangeOut, bearingOut, elevationOut);
    }
}
