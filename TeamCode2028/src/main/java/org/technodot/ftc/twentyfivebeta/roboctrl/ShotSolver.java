package org.technodot.ftc.twentyfivebeta.roboctrl;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.technodot.ftc.twentyfivebeta.Configuration;
import org.technodot.ftc.twentyfivebeta.common.Alliance;
import org.technodot.ftc.twentyfivebeta.common.Vector2D;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceExtake;
import org.technodot.ftc.twentyfivebeta.robocore.DeviceIntake;
import org.technodot.ftc.twentyfivebeta.robocore.DevicePinpoint;

import java.util.ArrayDeque;
import java.util.Queue;

public class ShotSolver {

    private static final Queue<Double> ranger = new ArrayDeque<>();
    private static double sum;
    private static final Queue<Vector2D> poseWindow = new ArrayDeque<>();
    private static double poseWindowSumX;
    private static double poseWindowSumY;
    private static Vector2D lastPose;
    private static boolean cameraPosStateInitialized;
    private static boolean lastIdleState;
    private static long stateEnteredNs;
    private static Vector2D filteredPose;
    private static Alliance filteredAlliance;
    private static int filteredTagId = -1;
    private static Double filteredYawErrorDeg;
    private static final int CAMERA_POSE_WINDOW_SIZE = 10;
    private static final double CAMERA_OUTLIER_DISTANCE_IN = 24.0;
    private static final long CAMERA_STATE_SWITCH_NS = 250_000_000L;
    // Tuned to suppress ~13 Hz heading vibration while preserving teleop-scale heading motion.
    private static final double BEARING_FILTER_CUTOFF_HZ = 2.5;
    private static final double BEARING_FILTER_MAX_RATE_DEG_PER_SEC = 360.0;
    private static final double BEARING_FILTER_MIN_STEP_DEG = 0.5;
    private static final double BEARING_FILTER_RESET_GAP_S = 0.25;
    private static final double BEARING_FILTER_DT_MIN_S = 1.0 / 240.0;
    private static final double BEARING_FILTER_DT_MAX_S = 1.0 / 20.0;
    private static final double BUTTERWORTH_Q = Math.sqrt(0.5);

    private static final double GOAL_X_OFFSET = 12.8; // in, relative to top left corner for blue goal
    private static final double GOAL_Y_OFFSET = 14.4; // in, relative to top left corner for blue goal
    private static final double GOAL_HEIGHT = 29.5; // in
    private static final double CAMERA_HEIGHT = 17.5; // in
    // Base field-heading for BLUE goal in the team's heading convention.
    // Team heading convention is not math-angle; we convert before using cos/sin.
    private static final double GOAL_HEADINIG = Math.PI / 2 + Math.atan(4.0 / 3.0); // radians
    private static boolean bearingFilterInitialized;
    private static long bearingFilterLastNs;
    private static double bearingRawUnwrappedDeg;
    private static double bearingFilterX1;
    private static double bearingFilterX2;
    private static double bearingFilterY1;
    private static double bearingFilterY2;
    private static Alliance bearingFilterAlliance;
    private static int bearingFilterTagId = -1;

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

    public static Vector2D getCameraPos(AprilTagDetection tag, Alliance alliance) {
        boolean idleNow = isExtakeIdleSafe();
        long nowNs = System.nanoTime();

        if (!cameraPosStateInitialized) {
            cameraPosStateInitialized = true;
            lastIdleState = idleNow;
            stateEnteredNs = nowNs;
        }

        boolean stateChanged = idleNow != lastIdleState;
        long previousStateDurationNs = stateChanged ? nowNs - stateEnteredNs : 0L;
        if (stateChanged) {
            lastIdleState = idleNow;
            stateEnteredNs = nowNs;
        }

        if (idleNow) {
            // If we are re-entering idle after running for >=250ms, start a fresh camera average window.
            if (stateChanged && previousStateDurationNs >= CAMERA_STATE_SWITCH_NS) {
                clearPoseWindow();
            }

            // Keep a 10-sample running average of solved camera positions.
            Vector2D solvedPose = calculateAbsolutePosition(tag, alliance);
            if (solvedPose == null || !isFinitePose(solvedPose)) {
                return getPoseWindowAverage();
            }

            // Reject outliers from averaging if they jump too far from last pose, but still track last pose.
            boolean outlier = consumeAndCheckOutlier(solvedPose);

            if (!outlier) {
                pushPoseSample(solvedPose);
            }

            Vector2D averaged = getPoseWindowAverage();
            Vector2D outputPose = averaged != null ? averaged : solvedPose;

            if (outputPose != null && isFinitePose(outputPose)) {
                FtcDashboard.getInstance().getTelemetry().addData("rx", outputPose.x);
                FtcDashboard.getInstance().getTelemetry().addData("ry", outputPose.y);
            }

            return outputPose;
        } else {
            // If we are entering non-idle after being idle for >=250ms, align odometry once from camera.
            if (stateChanged && previousStateDurationNs >= CAMERA_STATE_SWITCH_NS) {
                Vector2D absolutePose = calculateAbsolutePosition(tag, alliance);
                boolean outlier = absolutePose != null && isFinitePose(absolutePose) && consumeAndCheckOutlier(absolutePose);
                if (absolutePose != null && isFinitePose(absolutePose) && !outlier && DevicePinpoint.pinpoint != null) {
                    Vector2D blendedForPinpoint = getTransitionSetPose(absolutePose);
                    if (blendedForPinpoint != null && isFinitePose(blendedForPinpoint)) {
                        DevicePinpoint.setPos(blendedForPinpoint);
//                        DevicePinpoint.setHeading(calculateBearing(tag, alliance));
                    } else {
                        DevicePinpoint.setPos(absolutePose);
//                        DevicePinpoint.setHeading(calculateBearing(tag, alliance));
                    }
                }
            }

            // While extake is running, trust localizer pose.
            if (DevicePinpoint.pinpoint == null) return null;
            Vector2D odoPose = DevicePinpoint.getPos();

            if (odoPose != null && isFinitePose(odoPose)) {
                FtcDashboard.getInstance().getTelemetry().addData("rx", odoPose.x);
                FtcDashboard.getInstance().getTelemetry().addData("ry", odoPose.y);
            }

            return odoPose != null && isFinitePose(odoPose) ? odoPose : null;
        }
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
        Vector2D robotPos = getCameraPos(tag, alliance);
        if (goalPos == null || robotPos == null) return Double.NaN;

        double dx = goalPos.x - robotPos.x;
        double dy = goalPos.y - robotPos.y;
        return Math.hypot(dx, dy);
    }

    public static double getGoalYawError(AprilTagDetection tag, Alliance alliance) {
        if (isInvalidDetection(tag) || alliance == null) return filteredYawErrorDeg != null ? filteredYawErrorDeg : Double.NaN;
        if (DevicePinpoint.pinpoint == null) return filteredYawErrorDeg != null ? filteredYawErrorDeg : Double.NaN;

        Vector2D goalPos = getGoalPos(tag, alliance);
        Vector2D robotPos = getCameraPos(tag, alliance);
        if (goalPos == null || robotPos == null) return filteredYawErrorDeg != null ? filteredYawErrorDeg : Double.NaN;

        // Calculate the vector from robot to goal
        double dx = goalPos.x - robotPos.x;
        double dy = goalPos.y - robotPos.y;

        // Get the robot's current heading from the Pinpoint (in degrees)
        // Pinpoint uses GoBilda convention which aligns with ShotSolver's field heading system
        // TODO: instead of getting the pinpoint's heading, calculate the heading based on the tag.ftcPose and the previously defined tag data.
         double currentHeading = DevicePinpoint.pinpoint.getHeading(AngleUnit.DEGREES);
//        double currentHeading = calculateBearing(tag, alliance);

        if (!Double.isFinite(currentHeading)) return filteredYawErrorDeg != null ? filteredYawErrorDeg : Double.NaN;

        if (DeviceExtake.extakeState != DeviceExtake.ExtakeState.DUAL_SHORT) {
            // Shift amount in inches
            double shiftInches = 3;
            // Convert heading to radians for trig functions
            double headingRad = Math.toRadians(currentHeading);
            // Robot's right direction in field coords: (cos(heading), -sin(heading))
            // Robot's left direction in field coords: (-cos(heading), sin(heading))
            double rightX = Math.cos(headingRad);
            double rightY = -Math.sin(headingRad);

            switch (DeviceIntake.targetSide) {
                case LEFT:
                    // Shift the target position 3 inches to the RIGHT (robot's perspective)
                    // WARNING: this was previously broken??? don't trust ANYTHING
                    dx -= shiftInches * rightX;
                    dy -= shiftInches * rightY;
                    break;
                case RIGHT:
                    // Shift the target position 3 inches to the LEFT (robot's perspective)
                    // WARNING: this was previously broken??? don't trust ANYTHING
                    dx += shiftInches * rightX;
                    dy += shiftInches * rightY;
                    break;
                default:
                    break;
            }
        }

        // Calculate the desired heading (angle from robot to goal) in ShotSolver's convention:
        // Heading 0 = facing +Y direction (towards your goal), +X is left for blue, right for red
        // In this system, heading is measured as rotation from +Y axis
        // Using atan2: angle from +X axis (math convention), then convert to field heading
        double desiredHeadingMath = Math.atan2(dy, dx); // math angle: 0 = +X, CCW positive
        // Convert math-angle to field-heading: field heading 0 = +Y, so subtract PI/2
        // Field heading = PI/2 - math angle (since +Y corresponds to heading 0)
        double desiredHeadingField = Math.toDegrees(Math.PI / 2.0 - desiredHeadingMath);

        // Calculate error: how much the robot needs to rotate to face the goal
        double error = normalizeDegrees(desiredHeadingField - currentHeading);

        if (filteredYawErrorDeg == null || !Double.isFinite(filteredYawErrorDeg)) {
            filteredYawErrorDeg = error;
        } else {
            double delta = normalizeDegrees(error - filteredYawErrorDeg);
            filteredYawErrorDeg = normalizeDegrees(filteredYawErrorDeg + Configuration.SHOTSOLVER_YAW_ERROR_EMA_ALPHA * delta);
        }

//        return filteredYawErrorDeg;
        return error;

        /*
        BEGIN SECOND SOLUTION
         */

//        if (isInvalidDetection(tag) || alliance == null) return Double.NaN;
//
//        Vector2D goalPos = getGoalPos(tag, alliance);
//        Vector2D tagPos = getTagPos(tag, alliance);
//        if (goalPos == null || tagPos == null) return Double.NaN;
//
//        double rangeIn = tag.ftcPose.range;
//        double bearingDeg = tag.ftcPose.bearing;
//        double elevationDeg = tag.ftcPose.elevation;
//        double yawDeg = tag.ftcPose.yaw;
//        if (!Double.isFinite(rangeIn) || !Double.isFinite(bearingDeg) || !Double.isFinite(elevationDeg) || !Double.isFinite(yawDeg)) {
//            return Double.NaN;
//        }
//
//        // Project 3D ray into horizontal plane using known camera height.
//        double elevationRad = Math.toRadians(elevationDeg);
//        double verticalOffsetIn = GOAL_HEIGHT - CAMERA_HEIGHT;
//        double absRangeIn = Math.abs(rangeIn);
//        double planarRangeSq = absRangeIn * absRangeIn - verticalOffsetIn * verticalOffsetIn;
//
//        // Fallback to elevation projection if noisy range momentarily violates geometry.
//        // TODO: determine if fixed heights or dyanmic elevation is less noisy & impacted by vibrations.
//        double planarRange = planarRangeSq >= 0.0
//                ? Math.sqrt(planarRangeSq)
//                : Math.abs(absRangeIn * Math.cos(elevationRad));
//        if (!Double.isFinite(planarRange)) return Double.NaN;
//
//        // Build heading from goal -> tag in field-math angle, then offset by observed centerline
//        // shift (yaw - bearing) to recover tag -> camera direction.
//        double goalToTagHeadingMath = Math.atan2(tagPos.y - goalPos.y, tagPos.x - goalPos.x);
//        double centerlineOffsetRad = normalizeRadians(Math.toRadians(yawDeg - bearingDeg));
//        double tagToCameraHeadingMath = normalizeRadians(goalToTagHeadingMath + centerlineOffsetRad);
//        double cameraToTagHeadingMath = normalizeRadians(tagToCameraHeadingMath + Math.PI);
//
//        // Recover camera heading from camera->tag heading and measured camera-frame bearing.
//        double cameraHeadingMath = normalizeRadians(cameraToTagHeadingMath - Math.toRadians(bearingDeg));
//        double currentHeadingFieldRad = normalizeRadians(Math.PI / 2.0 - cameraHeadingMath);
//
//        // Build camera->goal vector without solving absolute camera field position.
//        double cameraToTagX = planarRange * Math.cos(cameraToTagHeadingMath);
//        double cameraToTagY = planarRange * Math.sin(cameraToTagHeadingMath);
//        double tagToGoalX = goalPos.x - tagPos.x;
//        double tagToGoalY = goalPos.y - tagPos.y;
//        double dx = cameraToTagX + tagToGoalX;
//        double dy = cameraToTagY + tagToGoalY;
//        if (!Double.isFinite(dx) || !Double.isFinite(dy)) return Double.NaN;
//
//        if (DeviceExtake.extakeState != DeviceExtake.ExtakeState.DUAL_SHORT) {
//            double shiftInches = 3.0;
//            double rightX = Math.cos(currentHeadingFieldRad);
//            double rightY = -Math.sin(currentHeadingFieldRad);
//
//            switch (DeviceIntake.targetSide) {
//                case LEFT:
//                    dx -= shiftInches * rightX;
//                    dy -= shiftInches * rightY;
//                    break;
//                case RIGHT:
//                    dx += shiftInches * rightX;
//                    dy += shiftInches * rightY;
//                    break;
//                default:
//                    break;
//            }
//        }
//
//        double desiredHeadingMath = Math.atan2(dy, dx);
//        double desiredHeadingFieldRad = normalizeRadians(Math.PI / 2.0 - desiredHeadingMath);
//        double errorDeg = Math.toDegrees(normalizeRadians(desiredHeadingFieldRad - currentHeadingFieldRad));
//        if (!Double.isFinite(errorDeg)) return Double.NaN;
//
//        return filterBearingHeading(-normalizeDegrees(errorDeg), alliance, tag.id);
    }

    public static double calculateBearing(AprilTagDetection tag, Alliance alliance) {
        if (isInvalidDetection(tag) || alliance == null) return Double.NaN;

        Vector2D tagPos = getTagPos(tag, alliance);
//        Vector2D cameraPos = getCameraPos(tag, alliance); // getCameraPos smoothes discrepancies out w/ DevicePinpoint. Significantly more smooth, at cost of drifting and accuracy. PRECISION OVER ACCURACY.
        Vector2D cameraPos = calculateAbsolutePosition(tag, alliance); // calculateAbsolutePosition outputs raw pos without any smoothing. More rough. ACCURACY OVER PRECISION.
        if (tagPos == null || cameraPos == null || !isFinitePose(cameraPos)) return Double.NaN;

        double bearingDeg = tag.ftcPose.bearing;
        if (!Double.isFinite(bearingDeg)) return Double.NaN;

        // Match the old Pinpoint "currentHeading" semantics used by ShotSolver:
        // 1) camera->tag absolute direction in field math-angle
        // 2) subtract camera-frame bearing to recover camera heading in math-angle
        // 3) convert math-angle to ShotSolver field-heading (0 = +Y)
        double cameraToTagHeadingMath = Math.atan2(tagPos.y - cameraPos.y, tagPos.x - cameraPos.x);
        double cameraHeadingMath = normalizeRadians(cameraToTagHeadingMath - Math.toRadians(bearingDeg));
        double headingFieldDeg = Math.toDegrees(Math.PI / 2.0 - cameraHeadingMath);
        return filterBearingHeading(headingFieldDeg, alliance, tag.id);
    }

    private static double filterBearingHeading(double rawHeadingDeg, Alliance alliance, int tagId) {
        if (!Double.isFinite(rawHeadingDeg)) return Double.NaN;

        long nowNs = System.nanoTime();
        boolean reset = !bearingFilterInitialized || alliance != bearingFilterAlliance || tagId != bearingFilterTagId;
        double dt = 0.0;

        if (!reset && bearingFilterLastNs > 0L) {
            dt = (nowNs - bearingFilterLastNs) * 1e-9;
            if (!Double.isFinite(dt) || dt <= 0.0 || dt > BEARING_FILTER_RESET_GAP_S) {
                reset = true;
            }
        }

        double rawNormDeg = normalizeDegrees(rawHeadingDeg);

        if (reset) {
            bearingFilterInitialized = true;
            bearingFilterAlliance = alliance;
            bearingFilterTagId = tagId;
            bearingFilterLastNs = nowNs;
            bearingRawUnwrappedDeg = rawNormDeg;
            bearingFilterX1 = rawNormDeg;
            bearingFilterX2 = rawNormDeg;
            bearingFilterY1 = rawNormDeg;
            bearingFilterY2 = rawNormDeg;
            return rawNormDeg;
        }

        // Unwrap around +/-180 so filtering remains continuous.
        double rawDeltaDeg = normalizeDegrees(rawNormDeg - normalizeDegrees(bearingRawUnwrappedDeg));
        bearingRawUnwrappedDeg += rawDeltaDeg;

        // Reject very sharp spikes beyond physically plausible turn rate.
        double maxStepDeg = BEARING_FILTER_MIN_STEP_DEG + BEARING_FILTER_MAX_RATE_DEG_PER_SEC * dt;
        double residualDeg = bearingRawUnwrappedDeg - bearingFilterY1;
        double clampedInputDeg = bearingFilterY1 + clip(residualDeg, -maxStepDeg, maxStepDeg);

        // 2nd-order Butterworth low-pass biquad.
        double dtCoeff = clip(dt, BEARING_FILTER_DT_MIN_S, BEARING_FILTER_DT_MAX_S);
        double fs = 1.0 / dtCoeff;
        double fc = Math.min(BEARING_FILTER_CUTOFF_HZ, 0.45 * fs);
        double omega = 2.0 * Math.PI * fc / fs;
        double sinW = Math.sin(omega);
        double cosW = Math.cos(omega);
        double alpha = sinW / (2.0 * BUTTERWORTH_Q);

        double b0 = (1.0 - cosW) * 0.5;
        double b1 = 1.0 - cosW;
        double b2 = (1.0 - cosW) * 0.5;
        double a0 = 1.0 + alpha;
        double a1 = -2.0 * cosW;
        double a2 = 1.0 - alpha;

        b0 /= a0;
        b1 /= a0;
        b2 /= a0;
        a1 /= a0;
        a2 /= a0;

        double y = b0 * clampedInputDeg + b1 * bearingFilterX1 + b2 * bearingFilterX2 - a1 * bearingFilterY1 - a2 * bearingFilterY2;
        if (!Double.isFinite(y)) {
            bearingFilterInitialized = false;
            return rawNormDeg;
        }

        bearingFilterX2 = bearingFilterX1;
        bearingFilterX1 = clampedInputDeg;
        bearingFilterY2 = bearingFilterY1;
        bearingFilterY1 = y;
        bearingFilterLastNs = nowNs;

        return normalizeDegrees(y);
    }

    private static void resetBearingFilter() {
        bearingFilterInitialized = false;
        bearingFilterLastNs = 0L;
        bearingRawUnwrappedDeg = 0.0;
        bearingFilterX1 = 0.0;
        bearingFilterX2 = 0.0;
        bearingFilterY1 = 0.0;
        bearingFilterY2 = 0.0;
        bearingFilterAlliance = null;
        bearingFilterTagId = -1;
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

    private static double normalizeRadians(double angleRad) {
        double normalized = angleRad;
        while (normalized > Math.PI) normalized -= 2.0 * Math.PI;
        while (normalized < -Math.PI) normalized += 2.0 * Math.PI;
        return normalized;
    }

    public static double calculateLaunchVelocity(double rangeIn) {
        sum += rangeIn;
        ranger.add(rangeIn);
        if (ranger.size() > 10) sum -= ranger.remove();
        double range = sum / ranger.size();

        return DeviceExtake.extakeState == DeviceExtake.ExtakeState.DYNAMIC ?
                Configuration.EXTAKE_MODEL_VELOCITY_SIMPLE_A * (range + Configuration.EXTAKE_MODEL_VELOCITY_SIMPLE_RANGE_SHIFT) * (range + Configuration.EXTAKE_MODEL_VELOCITY_SIMPLE_RANGE_SHIFT)
                + Configuration.EXTAKE_MODEL_VELOCITY_SIMPLE_B * (range + Configuration.EXTAKE_MODEL_VELOCITY_SIMPLE_RANGE_SHIFT)
                + Configuration.EXTAKE_MODEL_VELOCITY_SIMPLE_C
                :
                Range.clip(
                Configuration.EXTAKE_MODEL_VELOCITY_DOUBLE_A * (range + Configuration.EXTAKE_MODEL_VELOCITY_SIMPLE_RANGE_SHIFT) * (range + Configuration.EXTAKE_MODEL_VELOCITY_SIMPLE_RANGE_SHIFT)
                + Configuration.EXTAKE_MODEL_VELOCITY_DOUBLE_B * (range + Configuration.EXTAKE_MODEL_VELOCITY_SIMPLE_RANGE_SHIFT)
                + Configuration.EXTAKE_MODEL_VELOCITY_DOUBLE_C,
                0, 1500);
    }

    private static Vector2D calculateAbsolutePosition(AprilTagDetection tag, Alliance alliance) {
        if (isInvalidDetection(tag) || alliance == null) return null;
        if (tag.id != 20 && tag.id != 24) return null;

        Vector2D goalPos = getTagPos(tag, alliance);

        // Build goal heading using team field-heading convention from the user-provided mapping.
        double goalHeadingField = (tag.id == 20 ? GOAL_HEADINIG : Math.PI - GOAL_HEADINIG) - (alliance == Alliance.BLUE ? 0 : Math.PI);
        // Convert to math-angle for trig (0 = +X, CCW positive).
        double goalHeadingMath = Math.PI / 2.0 - goalHeadingField;

        // Secondary solve (no forced camera-height intersection): use full ftcPose directly and
        // project into a top-down pose by ignoring Z instead of solving for a fixed Z plane.
        double bearingRad = Math.toRadians(tag.ftcPose.bearing);
        double elevationRad = Math.toRadians(tag.ftcPose.elevation);
        double yawRad = Math.toRadians(tag.ftcPose.yaw);
        double pitchRad = Math.toRadians(tag.ftcPose.pitch);
        double rollRad = Math.toRadians(tag.ftcPose.roll);

        // Horizontal distance from full 3D range ray, ignoring the vertical component.
        double planarRange = Math.abs(tag.ftcPose.range * Math.cos(elevationRad));
        if (!Double.isFinite(planarRange) || planarRange < 0.0) planarRange = planarRange;

        // Use pitch/roll as a tilt attenuation factor on the horizontal centerline offset.
        double tiltScale = Math.cos(pitchRad) * Math.cos(rollRad);
        if (!Double.isFinite(tiltScale)) tiltScale = 1.0;
        tiltScale = clip(tiltScale, -1.0, 1.0);

        double centerlineOffsetRad = normalizeRadians((yawRad - bearingRad) * tiltScale);
        double tagToCameraHeading = goalHeadingMath + centerlineOffsetRad;

        double solvedX = goalPos.x + planarRange * Math.cos(tagToCameraHeading);
        double solvedY = goalPos.y + planarRange * Math.sin(tagToCameraHeading);

        // Keep camera estimate inside alliance-relative field bounds.
        double minX = alliance == Alliance.BLUE ? -144.0 : 0.0;
        double maxX = alliance == Alliance.BLUE ? 0.0 : 144.0;
//        solvedX = clip(solvedX, minX, maxX);
//        solvedY = clip(solvedY, 0.0, 144.0);

        if (!(minX < solvedX && solvedX < maxX && 0 < solvedY && solvedY < 144)) return null;

        FtcDashboard.getInstance().getTelemetry().addData("rx", solvedX);
        FtcDashboard.getInstance().getTelemetry().addData("ry", solvedY);

        return new Vector2D(solvedX, solvedY);
    }

    private static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static boolean isExtakeIdleSafe() {
        try {
            return DeviceExtake.isIdle();
        } catch (RuntimeException ignored) {
            return true;
        }
    }

    private static boolean isFinitePose(Vector2D pose) {
        return pose != null && Double.isFinite(pose.x) && Double.isFinite(pose.y);
    }

    private static boolean consumeAndCheckOutlier(Vector2D pose) {
        if (!isFinitePose(pose)) return true;
        boolean outlier = false;
        if (lastPose != null && isFinitePose(lastPose)) {
            double dist = Math.hypot(pose.x - lastPose.x, pose.y - lastPose.y);
            outlier = Double.isFinite(dist) && dist > CAMERA_OUTLIER_DISTANCE_IN;
        }
        lastPose = new Vector2D(pose);
        return outlier;
    }

    private static void pushPoseSample(Vector2D pose) {
        Vector2D sample = new Vector2D(pose);
        poseWindow.add(sample);
        poseWindowSumX += sample.x;
        poseWindowSumY += sample.y;

        while (poseWindow.size() > CAMERA_POSE_WINDOW_SIZE) {
            Vector2D removed = poseWindow.remove();
            poseWindowSumX -= removed.x;
            poseWindowSumY -= removed.y;
        }

        filteredPose = getPoseWindowAverage();
    }

    private static Vector2D getPoseWindowAverage() {
        if (poseWindow.isEmpty()) return null;
        double n = poseWindow.size();
        return new Vector2D(poseWindowSumX / n, poseWindowSumY / n);
    }

    private static Vector2D getTransitionSetPose(Vector2D currentPose) {
        if (!isFinitePose(currentPose)) return null;

        Vector2D previousAvg = getPoseWindowAverage();
        Vector2D currentAvg = getPoseWindowAverageWithCandidate(currentPose);

        if (!isFinitePose(previousAvg) && !isFinitePose(currentAvg)) return currentPose;
        if (!isFinitePose(previousAvg)) return currentAvg;
        if (!isFinitePose(currentAvg)) return previousAvg;

        // Required behavior: overall average of previous average and current average (50/50).
        return new Vector2D(
                0.5 * previousAvg.x + 0.5 * currentAvg.x,
                0.5 * previousAvg.y + 0.5 * currentAvg.y
        );
    }

    private static Vector2D getPoseWindowAverageWithCandidate(Vector2D candidate) {
        if (!isFinitePose(candidate)) return getPoseWindowAverage();
        if (poseWindow.isEmpty()) return new Vector2D(candidate);

        double sumX = poseWindowSumX + candidate.x;
        double sumY = poseWindowSumY + candidate.y;
        int count = poseWindow.size() + 1;

        if (count > CAMERA_POSE_WINDOW_SIZE) {
            Vector2D oldest = poseWindow.peek();
            if (oldest != null) {
                sumX -= oldest.x;
                sumY -= oldest.y;
            }
            count = CAMERA_POSE_WINDOW_SIZE;
        }

        if (count <= 0) return null;
        return new Vector2D(sumX / count, sumY / count);
    }

    private static void clearPoseWindow() {
        poseWindow.clear();
        poseWindowSumX = 0.0;
        poseWindowSumY = 0.0;
        filteredPose = null;
        lastPose = null;
        resetBearingFilter();
    }
}
