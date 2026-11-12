package org.technodot.ftc.twentyfivebeta.common;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import java.util.ArrayDeque;
import java.util.Map;
import java.util.WeakHashMap;


import org.technodot.ftc.twentyfivebeta.Configuration;

/**
 * This is the Drawing class. It handles the drawing of stuff on Panels Dashboard, like the robot.
 *
 * @author Lazar - 19234, TechnoDot @ 26855
 * @version 1.2, 2/7/2026
 */
public class Drawing {
    private static final Map<Follower, ArrayDeque<Pose>> histories = new WeakHashMap<>();
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style(
            "", "#00ff00", 0.75
    );
    private static final Style historyLook = new Style(
            "", "#5f00ff", 0.75
    );

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
        if (follower.currentPath() != null) {
            drawPath(follower.currentPath(), robotLook);
            drawRobot(follower.closestPose(), robotLook);
        }
        ArrayDeque<Pose> history = histories.computeIfAbsent(follower, f -> new ArrayDeque<>());
        history.addLast(follower.pose());
        while (history.size() > 100) history.removeFirst();
        panelsField.setStyle(historyLook);
        Pose previous = null;
        for (Pose pose : history) {
            if (previous != null) {
                panelsField.moveCursor(previous.x(), previous.y());
                panelsField.line(pose.x(), pose.y());
            }
            previous = pose;
        }
        drawRobot(follower.pose(), historyLook);

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
        if (pose == null || Double.isNaN(pose.x()) || Double.isNaN(pose.y()) || Double.isNaN(pose.heading())) {
            return;
        }

        double x = pose.x();
        double y = pose.y();
        double heading = pose.heading();

        double forwardX = Math.cos(heading);
        double forwardY = Math.sin(heading);
        double rightX = forwardY;
        double rightY = -forwardX;

        double frontLeftX = x + (forwardX * Configuration.LOCALIZER_LENGTH_FRONT_OFFSET) - (rightX * Configuration.LOCALIZER_WIDTH_LEFT_OFFSET);
        double frontLeftY = y + (forwardY * Configuration.LOCALIZER_LENGTH_FRONT_OFFSET) - (rightY * Configuration.LOCALIZER_WIDTH_LEFT_OFFSET);

        double frontRightX = frontLeftX + (rightX * Configuration.ROBOT_WIDTH);
        double frontRightY = frontLeftY + (rightY * Configuration.ROBOT_WIDTH);
        double backLeftX = frontLeftX - (forwardX * Configuration.ROBOT_LENGTH);
        double backLeftY = frontLeftY - (forwardY * Configuration.ROBOT_LENGTH);
        double backRightX = backLeftX + (rightX * Configuration.ROBOT_WIDTH);
        double backRightY = backLeftY + (rightY * Configuration.ROBOT_WIDTH);

        double frontCenterX = (frontLeftX + frontRightX) / 2.0;
        double frontCenterY = (frontLeftY + frontRightY) / 2.0;

        panelsField.setStyle(style);
        panelsField.moveCursor(frontLeftX, frontLeftY);
        panelsField.line(frontRightX, frontRightY);
        panelsField.moveCursor(frontRightX, frontRightY);
        panelsField.line(backRightX, backRightY);
        panelsField.moveCursor(backRightX, backRightY);
        panelsField.line(backLeftX, backLeftY);
        panelsField.moveCursor(backLeftX, backLeftY);
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
        panelsField.setStyle(style);
        Pose previous = path.get(0);
        for (int i = 1; i <= 100; i++) {
            Pose point = path.get(i / 100.0);
            panelsField.moveCursor(previous.x(), previous.y());
            panelsField.line(point.x(), point.y());
            previous = point;
        }
    }

    public static void sendPacket() {
        panelsField.update();
    }
}
