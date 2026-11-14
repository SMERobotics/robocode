package org.technodot.ftc.twentysix.common;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import org.junit.Test;

import static org.junit.Assert.*;

public class FieldCoordinatesTest {
    @Test public void startAndShootingPosesAreMirroredAndTouchTheirOwnWalls() {
        Pose red = FieldCoordinates.startPose(Alliance.RED, 24 + 8, 8);
        Pose blue = FieldCoordinates.startPose(Alliance.BLUE, 24 + 8, 8);
        assertEquals(32, red.x(), 1e-9);
        assertEquals(8, red.y(), 1e-9);
        assertEquals(Math.PI / 2, red.heading(), 1e-9);
        assertEquals(112, blue.x(), 1e-9);
        assertEquals(136, blue.y(), 1e-9);
        assertEquals(3 * Math.PI / 2, blue.heading(), 1e-9);
        Pose redShoot = FieldCoordinates.moveRight(red, 3);
        Pose blueShoot = FieldCoordinates.moveRight(blue, 3);
        assertEquals(35, redShoot.x(), 1e-9);
        assertEquals(109, blueShoot.x(), 1e-9);
        assertEquals(red.y(), redShoot.y(), 1e-9);
        assertEquals(blue.y(), blueShoot.y(), 1e-9);
        assertEquals(3, red.distance(redShoot), 1e-9);
        assertEquals(3, blue.distance(blueShoot), 1e-9);
        // A 16-inch chassis stays inside the field and fully on its own alliance half.
        assertEquals(0, red.y() - 8, 1e-9);
        assertEquals(144, blue.y() + 8, 1e-9);
        assertTrue(redShoot.x() + 8 < 72);
        assertTrue(blueShoot.x() - 8 > 72);
    }

    @Test public void forwardAlwaysMovesAwayFromTheSelectedAlliancesDrivers() {
        for (Alliance alliance : Alliance.values()) {
            double driverHeading = alliance == Alliance.RED ? 0 : Math.PI;
            for (double robotHeading : new double[] {0, Math.PI / 2, Math.PI, 3 * Math.PI / 2, 7.3}) {
                Vector2D robot = FieldCoordinates.driverToRobot(alliance, robotHeading, 1, 0);
                double fieldX = robot.x() * Math.cos(robotHeading) + robot.y() * Math.sin(robotHeading);
                double fieldY = robot.x() * Math.sin(robotHeading) - robot.y() * Math.cos(robotHeading);
                assertEquals(Math.cos(driverHeading), fieldX, 1e-9);
                assertEquals(Math.sin(driverHeading), fieldY, 1e-9);
            }
        }
    }

    @Test public void rightAlwaysMovesToTheSelectedDriversRight() {
        for (Alliance alliance : Alliance.values()) {
            double driverHeading = alliance == Alliance.RED ? 0 : Math.PI;
            for (double robotHeading : new double[] {0, Math.PI / 2, Math.PI, -8.1}) {
                Vector2D robot = FieldCoordinates.driverToRobot(alliance, robotHeading, 0, 1);
                double fieldX = robot.x() * Math.cos(robotHeading) + robot.y() * Math.sin(robotHeading);
                double fieldY = robot.x() * Math.sin(robotHeading) - robot.y() * Math.cos(robotHeading);
                assertEquals(Math.sin(driverHeading), fieldX, 1e-9);
                assertEquals(-Math.cos(driverHeading), fieldY, 1e-9);
            }
        }
    }

    @Test public void transformationPreservesDiagonalInputMagnitude() {
        Vector2D power = FieldCoordinates.driverToRobot(Alliance.BLUE, 0.73, 0.6, -0.4);
        assertEquals(Math.hypot(0.6, -0.4), power.magnitude(), 1e-9);
    }
}
