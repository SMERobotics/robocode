package com.technodot.ftc.twentyfour.maytag;

import com.technodot.ftc.twentyfour.Configuration;

import java.util.Dictionary;
import java.util.Hashtable;

/**
 * This class tracks the position and heading of the robot relative to multiple AprilTags around the field.
 */
public class MayTagTracker {
    public double x;
    public double y;
    public double h;

    public final Dictionary<Integer, double[]> tagPositions = new Hashtable<>();

    public static final int unitLength = 24;

    public MayTagTracker() {
        // initialize tag positions
        tagPositions.put(11, new double[]{1.0D, 0.0D, 0.0D});
        tagPositions.put(12, new double[]{0.0D, 3.0D, 90.0D});
        tagPositions.put(13, new double[]{1.0D, 6.0D, 180.0D});
        tagPositions.put(14, new double[]{5.0D, 6.0D, 180.0D});
        tagPositions.put(15, new double[]{6.0D, 3.0D, 270.0D});
        tagPositions.put(16, new double[]{5.0D, 6.0D, 0.0D});
    }

    /**
     * Sets the robot's position.
     *
     * @param x the x coordinate position of the robot
     * @param y the y coordinate position of the robot
     * @param h the heading of the robot in degrees
     */
    public void setRobotPosition(double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
    }

    /**
     * Calculates the robot's position based on the detected AprilTag.
     *
     * @param tagId   the ID of the detected AprilTag
     * @param range   the distance from the AprilTag to the robot in inches
     * @param yaw     the angle from the AprilTag to the robot in degrees
     * @param bearing the angle from the robot to the AprilTag in degrees
     * @return a double[] containing the x position, y position, and heading in degrees
     */
    public double[] calculateRobotPosition(int tagId, double range, double yaw, double bearing) {
        double[] tagData = tagPositions.get(tagId);

        double tagX = tagData[0];
        double tagY = tagData[1];
        double tagH = tagData[2];

        double posX = tagX + (range + Configuration.robotCameraCenterOffset) / unitLength * Math.sin(Math.toRadians(tagH + yaw));
        double posY = tagY + (range + Configuration.robotCameraCenterOffset) / unitLength * Math.cos(Math.toRadians(tagH + yaw));
        double posH = (tagH + yaw - bearing + 180 - Configuration.robotCameraAngleOffset) % 360;

        return new double[]{posX, posY, posH};
    }

    /**
     * Updates the robot's position based on the detected AprilTag.
     *
     * @param tagId   the ID of the detected AprilTag
     * @param range   the distance from the robot to the AprilTag in inches
     * @param yaw     the angle from the AprilTag to the robot in degrees
     * @param bearing the angle from the robot to the AprilTag in degrees
     */
    public void updateRobotPosition(int tagId, double range, double yaw, double bearing) {
        if (tagPositions.get(tagId) != null) {
            return;
        }
        double[] pos = calculateRobotPosition(tagId, range, yaw, bearing);
        setRobotPosition(pos[0], pos[1], pos[2]);
    }

    /**
     * Updates the robot's position based on multiple detected AprilTags and averages them out.
     *
     * @param tagDataArray a 2D array where each row contains the tag ID, range, and bearing
     */
    public void updateRobotPositions(double[][] tagDataArray) {
        double sumX = 0;
        double sumY = 0;
        double sumH = 0;
        int count = 0;

        for (double[] tagData : tagDataArray) {
            int tagId = (int) tagData[0];
            double range = tagData[1];
            double yaw = tagData[2];
            double bearing = tagData[3];

            if (tagPositions.get(tagId) != null) {
                double[] pos = calculateRobotPosition(tagId, range, yaw, bearing);
                sumX += pos[0];
                sumY += pos[1];
                sumH += pos[2];
                count++;
            }
        }

        if (count > 0) {
            setRobotPosition(sumX / count, sumY / count, sumH / count);
        }
    }
}
