package org.technodot.ftc.twentyfivebeta.common;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;

/**
 * This is the Drawing class. It handles the drawing of stuff on Panels Dashboard, like the robot.
 *
 * @author Lazar - 19234, TechnoDot @ 26855
 * @version 1.2, 2/7/2026
 */
public class Drawing {
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style(
            "", "#00ff00", 0.75
    );
    private static final Style historyLook = new Style(
            "", "#5f00ff", 0.75
    );

    public static final double ROBOT_LENGTH = 18; // front to back distance, in inches
    public static final double ROBOT_WIDTH = 18; // left to right distance, in inches
    public static final double LOCALIZER_LENGTH_OFFSET = ROBOT_LENGTH - 6.7; // front-left to localizer distance along robot length axis, in inches
    public static final double LOCALIZER_WIDTH_OFFSET = 9; // front-left to localizer distance along robot width axis, in inches

    /**
     * This prepares Panels Field for using Pedro Offsets
     */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashbaordDrawingHandler class is not needed.
     *
     * @param follower Pedro Follower instance.
     */
    public static void draw(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook);
        drawRobot(follower.getPose(), historyLook);

        sendPacket();
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose  the localizer Pose used to draw the robot body
     * @param style the parameters used to draw the robot with
     */
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading();

        double forwardX = Math.cos(heading);
        double forwardY = Math.sin(heading);
        double rightX = forwardY;
        double rightY = -forwardX;

        double frontLeftX = x + (forwardX * LOCALIZER_LENGTH_OFFSET) - (rightX * LOCALIZER_WIDTH_OFFSET);
        double frontLeftY = y + (forwardY * LOCALIZER_LENGTH_OFFSET) - (rightY * LOCALIZER_WIDTH_OFFSET);

        double frontRightX = frontLeftX + (rightX * ROBOT_WIDTH);
        double frontRightY = frontLeftY + (rightY * ROBOT_WIDTH);
        double backLeftX = frontLeftX - (forwardX * ROBOT_LENGTH);
        double backLeftY = frontLeftY - (forwardY * ROBOT_LENGTH);
        double backRightX = backLeftX + (rightX * ROBOT_WIDTH);
        double backRightY = backLeftY + (rightY * ROBOT_WIDTH);

        double frontCenterX = (frontLeftX + frontRightX) / 2.0;
        double frontCenterY = (frontLeftY + frontRightY) / 2.0;

        panelsField.setStyle(style);
        panelsField.moveCursor(frontLeftX, frontLeftY);
        panelsField.line(frontRightX, frontRightY);
        panelsField.line(backRightX, backRightY);
        panelsField.line(backLeftX, backLeftY);
        panelsField.line(frontLeftX, frontLeftY);

        panelsField.moveCursor(x, y);
        panelsField.line(frontCenterX, frontCenterY);
    }

    /**
     * This draws a robot at a specified Pose. The heading is represented as a line.
     *
     * @param pose the localizer Pose used to draw the robot body
     */
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    /**
     * This draws a Path with a specified look.
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    public static void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();

        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }

    /**
     * This draws all the Paths in a PathChain with a
     * specified look.
     *
     * @param pathChain the PathChain to draw
     * @param style     the parameters used to draw the PathChain with
     */
    public static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     * @param style       the parameters used to draw the pose history with
     */
    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
        panelsField.setStyle(style);

        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {

            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     */
    public static void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, historyLook);
    }

    /**
     * This tries to send the current packet to FTControl Panels.
     */
    public static void sendPacket() {
        panelsField.update();
    }
}
