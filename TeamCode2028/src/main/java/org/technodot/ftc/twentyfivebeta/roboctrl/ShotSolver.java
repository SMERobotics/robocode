package org.technodot.ftc.twentyfivebeta.roboctrl;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.technodot.ftc.twentyfivebeta.Configuration;
import org.technodot.ftc.twentyfivebeta.common.Alliance;
import org.technodot.ftc.twentyfivebeta.common.Vector2D;
import org.technodot.ftc.twentyfivebeta.robocore.DevicePinpoint;

import java.util.ArrayDeque;
import java.util.Queue;

public class ShotSolver {

    private static final Queue<Double> ranger = new ArrayDeque<>();
    private static double sum;
    private static final Queue<Vector2D> poseWindow = new ArrayDeque<>();
    private static Vector2D filteredPose;
    private static Alliance filteredAlliance;
    private static int filteredTagId = -1;
    private static Double filteredYawErrorDeg;

    private static final double GOAL_X_OFFSET = 12.8; // in, relative to top left corner for blue goal
    private static final double GOAL_Y_OFFSET = 14.4; // in, relative to top left corner for blue goal
    private static final double GOAL_HEIGHT = 29.5; // in
    // Base field-heading for BLUE goal in the team's heading convention.
    // Team heading convention is not math-angle; we convert before using cos/sin.
    private static final double GOAL_HEADINIG = Math.PI / 2 + Math.atan(4.0 / 3.0); // radians
    private static final double CAMERA_HEIGHT = 17.5; // in

    /*
    This coordinate system is going to take some explaining. It is dependent upon which alliance you are on, and I promise you there's a compelling reason.

    Heading of 0 is the direction you and your team faces when you are on that alliance's people zone idfk what ts is called
    i.e. opposite the side of your alliance's goal and facing towards the side with your alliance's goal

    Your loading zone, regardless of what alliance you are on, will always be (0, 0).
    Because you are on that side, the side of the field opposite you with your goal will be y=144.
    This is where the alliance specific stuff really shines.
    The blue goal is to the north and east relative to your loading zone, giving it coordinates of (**-144**, 144).
    The red goal is to the north and west relative to your loading zone, giving it coordinates of (**144**, 144).

    Think about it as if you isolated the stuff important as the red alliance, put it on the left, and then the stuff important to the blue alliance, and put it on the right.
    To the left would be +X, to the right would be -X. **This is how the GoBilda Pinpoint works, *do not ask me.***

    I believe this system is better because you're going to face the direction of shit you're trying to understand/do.

    G=Goal L=Loading Zone
                      (0, 144)
    (144, 144) GGG--------+--------GGG (-144, 144)
               GG         |         GG
               |    RED   |   BLUE   |
               |        LL|LL        |
      (144, 0) +-------LLL+LLL-------+ (-144, 0)
                 ^-YOU  (0, 0) YOU-^
     */

    public static Vector2D calculateAbsolutePosition(AprilTagDetection tag, Alliance alliance) {
        if (isInvalidDetection(tag) || alliance == null) return null;
        if (tag.id != 20 && tag.id != 24) return null;

        Vector2D goalPos = getTagPos(tag, alliance);

        // Build goal heading using team field-heading convention from the user-provided mapping.
        double goalHeadingField = (tag.id == 20 ? GOAL_HEADINIG : Math.PI - GOAL_HEADINIG) - (alliance == Alliance.BLUE ? 0 : Math.PI);
        // Convert to math-angle for trig (0 = +X, CCW positive).
        double goalHeadingMath = Math.PI / 2.0 - goalHeadingField;
        // Range is the primary signal (distance in the camera XY plane). This avoids noisy angle-only depth solves.
        double planarRange = Math.abs(tag.ftcPose.range);
        // Fallback depth solve: intersect observed ray with z = CAMERA_HEIGHT using known tag height.

        // Kept guarded because elevation/pitch/roll are noisier under camera vibration.
        double tiltMagnitude = Math.hypot(tag.ftcPose.pitch, tag.ftcPose.roll);
        if ((!Double.isFinite(planarRange) || planarRange <= 0.0) && tiltMagnitude < 12.0) {
            double dz = GOAL_HEIGHT - CAMERA_HEIGHT;
            double elevationRad = Math.toRadians(tag.ftcPose.elevation);
            double bearingRad = Math.toRadians(tag.ftcPose.bearing);
            double tanElevation = Math.tan(elevationRad);
            double cosBearing = Math.cos(bearingRad);

            if (Math.abs(tanElevation) > 1e-3 && Math.abs(cosBearing) > 1e-3) {
                // FTC definitions: y = dz / tan(elevation), range = |y| / cos(bearing)
                planarRange = Math.abs(dz / tanElevation / cosBearing);
            }
        }
        if (!Double.isFinite(planarRange) || planarRange < 0.0) planarRange = 0.0;

        // yaw-bearing is the most stable horizontal offset from the tag centerline
        // (camera yaw jitter tends to affect both terms similarly and cancel here).
        double centerlineOffsetRad = Math.toRadians(tag.ftcPose.yaw - tag.ftcPose.bearing);
        if (!Double.isFinite(centerlineOffsetRad)) centerlineOffsetRad = 0.0;

        double tagToCameraHeading = goalHeadingMath + centerlineOffsetRad;
        double solvedX = goalPos.x + planarRange * Math.cos(tagToCameraHeading);
        double solvedY = goalPos.y + planarRange * Math.sin(tagToCameraHeading);

        // Keep camera estimate inside alliance-relative field bounds.
        double minX = alliance == Alliance.BLUE ? -144.0 : 0.0;
        double maxX = alliance == Alliance.BLUE ? 0.0 : 144.0;
        solvedX = clip(solvedX, minX, maxX);
        solvedY = clip(solvedY, 0.0, 144.0);

        return smoothPose(new Vector2D(solvedX, solvedY), alliance, tag.id);
    }

    public static Vector2D getTagPos(AprilTagDetection tag, Alliance alliance) {
        if (isInvalidDetection(tag) || alliance == null) return null;
        if (tag.id != 20 && tag.id != 24) return null;

        return new Vector2D(
                alliance == Alliance.BLUE ? -(144 - GOAL_X_OFFSET) : (144 - GOAL_X_OFFSET),
                tag.id == (alliance == Alliance.BLUE ? 20 : 24) ? 144 - GOAL_Y_OFFSET : GOAL_Y_OFFSET
        );
    }

    public static Vector2D getGoalPos(AprilTagDetection tag, Alliance alliance) {
        if (isInvalidDetection(tag) || alliance == null) return null;
        if (tag.id != 20 && tag.id != 24) return null;

        return new Vector2D(
                alliance == Alliance.BLUE ? -144 : 144,
                tag.id == (alliance == Alliance.BLUE ? 20 : 24) ? 144 : 0
        );
    }

    public static double getGoalDistance(AprilTagDetection tag, Alliance alliance) {
        Vector2D goalPos = getGoalPos(tag, alliance);
        Vector2D robotPos = calculateAbsolutePosition(tag, alliance);
        if (goalPos == null || robotPos == null) return Double.NaN;

        double dx = goalPos.x - robotPos.x;
        double dy = goalPos.y - robotPos.y;
        return Math.hypot(dx, dy);
    }

    public static double getGoalYawError(AprilTagDetection tag, Alliance alliance) {
        if (isInvalidDetection(tag) || alliance == null) return filteredYawErrorDeg != null ? filteredYawErrorDeg : Double.NaN;
        if (DevicePinpoint.pinpoint == null) return filteredYawErrorDeg != null ? filteredYawErrorDeg : Double.NaN;

        Vector2D goalPos = getGoalPos(tag, alliance);
        Vector2D robotPos = calculateAbsolutePosition(tag, alliance);
        if (goalPos == null || robotPos == null) return filteredYawErrorDeg != null ? filteredYawErrorDeg : Double.NaN;

        // Calculate the vector from robot to goal
        double dx = goalPos.x - robotPos.x;
        double dy = goalPos.y - robotPos.y;

        // Calculate the desired heading (angle from robot to goal) in ShotSolver's convention:
        // Heading 0 = facing +Y direction (towards your goal), +X is left for blue, right for red
        // In this system, heading is measured as rotation from +Y axis
        // Using atan2: angle from +X axis (math convention), then convert to field heading
        double desiredHeadingMath = Math.atan2(dy, dx); // math angle: 0 = +X, CCW positive
        // Convert math-angle to field-heading: field heading 0 = +Y, so subtract PI/2
        // Field heading = PI/2 - math angle (since +Y corresponds to heading 0)
        double desiredHeadingField = Math.toDegrees(Math.PI / 2.0 - desiredHeadingMath);

        // Get the robot's current heading from the Pinpoint (in degrees)
        // Pinpoint uses GoBilda convention which aligns with ShotSolver's field heading system
        double currentHeading = DevicePinpoint.pinpoint.getHeading(AngleUnit.DEGREES);
        if (!Double.isFinite(currentHeading)) return filteredYawErrorDeg != null ? filteredYawErrorDeg : Double.NaN;

        // Calculate error: how much the robot needs to rotate to face the goal
        double error = normalizeDegrees(desiredHeadingField - currentHeading);

        if (filteredYawErrorDeg == null || !Double.isFinite(filteredYawErrorDeg)) {
            filteredYawErrorDeg = error;
        } else {
            double delta = normalizeDegrees(error - filteredYawErrorDeg);
            filteredYawErrorDeg = normalizeDegrees(filteredYawErrorDeg + Configuration.SHOTSOLVER_YAW_ERROR_EMA_ALPHA * delta);
        }

        return filteredYawErrorDeg;
    }

    private static boolean isInvalidDetection(AprilTagDetection tag) {
        return tag == null || tag.ftcPose == null;
    }

    private static double normalizeDegrees(double angleDeg) {
        double normalized = angleDeg;
        while (normalized > 180.0) normalized -= 360.0;
        while (normalized < -180.0) normalized += 360.0;
        return normalized;
    }

    public static double calculateLaunchVelocity(double rangeIn) {
        sum += rangeIn;
        ranger.add(rangeIn);
        if (ranger.size() > 10) sum -= ranger.remove();
        double range = sum / ranger.size();

        return Configuration.EXTAKE_MODEL_VELOCITY_SIMPLE_A * (range + Configuration.EXTAKE_MODEL_VELOCITY_SIMPLE_RANGE_SHIFT) * (range + Configuration.EXTAKE_MODEL_VELOCITY_SIMPLE_RANGE_SHIFT)
                + Configuration.EXTAKE_MODEL_VELOCITY_SIMPLE_B * (range + Configuration.EXTAKE_MODEL_VELOCITY_SIMPLE_RANGE_SHIFT)
                + Configuration.EXTAKE_MODEL_VELOCITY_SIMPLE_C;
    }

    public static void clearVelocityQueue() {
        ranger.clear();
        sum = 0;
        clearPoseFilter();
    }

    public static void clearPoseFilter() {
        poseWindow.clear();
        filteredPose = null;
        filteredAlliance = null;
        filteredTagId = -1;
        filteredYawErrorDeg = null;
    }

    private static Vector2D smoothPose(Vector2D measurement, Alliance alliance, int tagId) {
        if (measurement == null) return null;
        if (filteredPose == null || filteredAlliance != alliance || filteredTagId != tagId) {
            clearPoseFilter();
            filteredPose = new Vector2D(measurement);
            filteredAlliance = alliance;
            filteredTagId = tagId;
            poseWindow.add(new Vector2D(measurement));
            return new Vector2D(filteredPose);
        }

        // Reject single-frame spikes from vibration before they enter the window.
        Vector2D windowSample = measurement;
        if (measurement.difference(filteredPose).magnitude() > Configuration.SHOTSOLVER_POSE_OUTLIER_REJECT_IN) {
            windowSample = new Vector2D(filteredPose);
        }

        poseWindow.add(new Vector2D(windowSample));
        while (poseWindow.size() > Configuration.SHOTSOLVER_POSE_WINDOW_SIZE) poseWindow.remove();

        double medianX = medianWindowAxis(true);
        double medianY = medianWindowAxis(false);

        double nextX = filteredPose.x + Configuration.SHOTSOLVER_POSE_EMA_ALPHA * (medianX - filteredPose.x);
        double nextY = filteredPose.y + Configuration.SHOTSOLVER_POSE_EMA_ALPHA * (medianY - filteredPose.y);

        double stepDx = nextX - filteredPose.x;
        double stepDy = nextY - filteredPose.y;
        double stepMag = Math.hypot(stepDx, stepDy);
        if (stepMag > Configuration.SHOTSOLVER_POSE_MAX_STEP_IN && stepMag > 1e-6) {
            double scale = Configuration.SHOTSOLVER_POSE_MAX_STEP_IN / stepMag;
            stepDx *= scale;
            stepDy *= scale;
        }

        filteredPose = new Vector2D(filteredPose.x + stepDx, filteredPose.y + stepDy);
        return new Vector2D(filteredPose);
    }

    private static double medianWindowAxis(boolean xAxis) {
        int n = poseWindow.size();
        if (n == 0) return 0.0;
        double[] values = new double[n];
        int i = 0;
        for (Vector2D sample : poseWindow) {
            values[i++] = xAxis ? sample.x : sample.y;
        }
        java.util.Arrays.sort(values);
        int mid = n / 2;
        if ((n & 1) == 1) return values[mid];
        return (values[mid - 1] + values[mid]) * 0.5;
    }

    private static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
