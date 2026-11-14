package org.technodot.ftc.twentysix.roboctrl;

import org.junit.Test;
import org.technodot.ftc.twentysix.common.Alliance;
import org.technodot.ftc.twentysix.common.Hive;

import java.util.Collections;

import static org.junit.Assert.*;

public class HiveTurretAimTest {
    private static final long TIME = 10_000_000_000L;

    private static SimpleHiveSolver.Config camera(double pitch, double roll) {
        SimpleHiveSolver.Config c = new SimpleHiveSolver.Config();
        c.cameraPitchDegrees = pitch; c.cameraRollDegrees = roll;
        c.maxAgeMillis = 200;
        return c;
    }

    private static HiveObservation observation(Alliance alliance, Hive state, long capture, double... focusInches) {
        HiveGeometry.Cell cell = HiveGeometry.cellForSide(alliance, -1);
        double[] meters = focusInches.clone();
        for (int i = 0; i < meters.length; i++) meters[i] *= HiveGeometry.INCH;
        return new HiveObservation(alliance, "", cell, state, capture,
                Collections.singletonList(cell.firstTagId), 1, 0, Double.POSITIVE_INFINITY, 25, 25, meters);
    }

    private static HiveObservation horizontal(Alliance alliance, Hive state, long capture, double rightDegrees) {
        return observation(alliance, state, capture, 60 * Math.sin(Math.toRadians(rightDegrees)),
                0, 60 * Math.cos(Math.toRadians(rightDegrees)));
    }

    @Test public void allHiveStatesAimInTheClockwiseEncoderDirectionForEitherAlliance() {
        for (Alliance alliance : Alliance.values()) for (Hive state : Hive.values()) {
            HiveTurretAim aim = new HiveTurretAim(alliance, camera(0, 0), new HiveTurretAim.Config());
            aim.start(40, TIME);
            assertEquals(55, aim.update(40, horizontal(alliance, state, TIME, 15), TIME, TIME), 1e-9);
            assertEquals("tracking", aim.getStatus());
            assertEquals(20, aim.update(40, horizontal(alliance, state, TIME + 10_000_000, -20),
                    TIME + 10_000_000, TIME + 10_000_000), 1e-9);
        }
    }

    @Test public void wrongAllianceAndInvalidPosesCannotChangeTheAimTarget() {
        HiveTurretAim aim = new HiveTurretAim(Alliance.BLUE, camera(0, 0), new HiveTurretAim.Config());
        aim.start(20, TIME);
        assertEquals(35, aim.update(20, horizontal(Alliance.BLUE, Hive.READY, TIME, 15), TIME, TIME), 1e-9);
        long now = TIME + 20_000_000;
        assertEquals(35, aim.update(22, horizontal(Alliance.RED, Hive.READY, now, 70), 0, now), 1e-9);
        now += 20_000_000;
        assertEquals(35, aim.update(23, HiveObservation.invalid(Alliance.BLUE, "ambiguous planar pose", now),
                now, now), 1e-9);
    }

    @Test public void cameraPitchAndRollAreRemovedBeforeComputingHorizontalYaw() {
        double pitch = Math.toRadians(70), roll = Math.toRadians(13), yaw = Math.toRadians(24);
        double x = 60 * Math.sin(yaw), y = -15, z = 60 * Math.cos(yaw);
        double pitchedY = Math.cos(pitch) * y + Math.sin(pitch) * z;
        double pitchedZ = -Math.sin(pitch) * y + Math.cos(pitch) * z;
        double opticalX = Math.cos(roll) * x + Math.sin(roll) * pitchedY;
        double opticalY = -Math.sin(roll) * x + Math.cos(roll) * pitchedY;
        HiveObservation observation = observation(Alliance.BLUE, Hive.READY, TIME, opticalX, opticalY, pitchedZ);
        HiveTurretAim aim = new HiveTurretAim(Alliance.BLUE, camera(70, 13), new HiveTurretAim.Config());
        aim.start(35, TIME);
        assertEquals(59, aim.update(35, observation, TIME, TIME), 1e-9);
    }

    @Test public void cameraYawAndOpticalCenterOffsetsAreIncluded() {
        HiveTurretAim.Config mount = new HiveTurretAim.Config();
        mount.cameraYawDegrees = 10;
        mount.cameraRightInches = 4; mount.cameraForwardInches = 3;
        HiveTurretAim aim = new HiveTurretAim(Alliance.BLUE, camera(0, 0), mount);
        aim.start(0, TIME);
        HiveObservation observation = observation(Alliance.BLUE, Hive.READY, TIME, 0, 0, 60);
        double expected = Math.toDegrees(Math.atan2(60 * Math.sin(Math.toRadians(10)) + 4,
                60 * Math.cos(Math.toRadians(10)) + 3));
        assertEquals(expected, aim.update(0, observation, TIME, TIME), 1e-9);
    }

    @Test public void captureTimeHeadingIsInterpolatedAndRepeatedFramesDoNotAccumulateCorrections() {
        HiveTurretAim aim = new HiveTurretAim(Alliance.BLUE, camera(0, 0), new HiveTurretAim.Config());
        aim.start(10, TIME);
        long capture = TIME + 50_000_000;
        HiveObservation observation = horizontal(Alliance.BLUE, Hive.READY, capture, 20);
        assertEquals(40, aim.update(30, observation, capture, TIME + 100_000_000), 1e-9);
        assertEquals(40, aim.update(35, observation, capture, TIME + 120_000_000), 1e-9);
        assertEquals(40, aim.update(39, observation, capture, TIME + 140_000_000), 1e-9);
    }

    @Test public void continuousEncoderAnglesCrossingZeroAreNotInterpolatedThroughTheOppositeDirection() {
        HiveTurretAim aim = new HiveTurretAim(Alliance.BLUE, camera(0, 0), new HiveTurretAim.Config());
        aim.start(-5, TIME);
        long capture = TIME + 50_000_000;
        assertEquals(10, aim.update(5, horizontal(Alliance.BLUE, Hive.READY, capture, 10),
                capture, TIME + 100_000_000), 1e-9);
    }

    @Test public void frozenFramesCannotExtendTheThreeSecondTagTimeout() {
        HiveTurretAim aim = new HiveTurretAim(Alliance.BLUE, camera(0, 0), new HiveTurretAim.Config());
        aim.start(0, TIME);
        HiveObservation observation = horizontal(Alliance.BLUE, Hive.READY, TIME, 30);
        assertEquals(30, aim.update(0, observation, TIME, TIME), 1e-9);
        assertEquals(30, aim.update(15, observation, TIME, TIME + 2_999_000_000L), 1e-9);
        assertEquals(0, aim.update(25, observation, TIME, TIME + 3_000_000_000L), 0);
        assertTrue(aim.getStatus().contains("forward"));
        long now = TIME + 3_020_000_000L;
        assertEquals(15, aim.update(20, horizontal(Alliance.BLUE, Hive.TIPPING, now, -5), now, now), 1e-9);
    }

    @Test public void visibleAllianceTagsRefreshTheTimeoutEvenWhenTheirPoseCannotBeUsed() {
        HiveTurretAim aim = new HiveTurretAim(Alliance.BLUE, camera(0, 0), new HiveTurretAim.Config());
        aim.start(0, TIME);
        aim.update(0, horizontal(Alliance.BLUE, Hive.READY, TIME, 30), TIME, TIME);
        long seen = TIME + 2_000_000_000L;
        assertEquals(30, aim.update(30, HiveObservation.invalid(Alliance.BLUE, "no physical pose", seen), seen, seen), 1e-9);
        long tipped = TIME + 4_000_000_000L;
        assertEquals(110, aim.update(30, horizontal(Alliance.BLUE, Hive.TIPPED, tipped, 80), tipped, tipped), 1e-9);
        assertEquals(110, aim.update(30, null, tipped, TIME + 6_999_000_000L), 1e-9);
        assertEquals(0, aim.update(30, null, tipped, TIME + 7_000_000_000L), 0);
    }

    @Test public void opponentTagsDoNotKeepTheLastAimAlive() {
        HiveTurretAim aim = new HiveTurretAim(Alliance.BLUE, camera(0, 0), new HiveTurretAim.Config());
        aim.start(0, TIME);
        aim.update(0, horizontal(Alliance.BLUE, Hive.READY, TIME, 30), TIME, TIME);
        long now = TIME + 3_000_000_000L;
        assertEquals(0, aim.update(30, horizontal(Alliance.RED, Hive.READY, now, 60), 0, now), 0);
    }

    @Test public void startupHoldsPositionUntilTimeoutAndPreStartOrStaleImagesCannotSteer() {
        HiveTurretAim aim = new HiveTurretAim(Alliance.BLUE, camera(0, 0), new HiveTurretAim.Config());
        aim.start(45, TIME);
        assertEquals(45, aim.update(45, horizontal(Alliance.BLUE, Hive.READY, TIME - 20_000_000, 60),
                TIME - 20_000_000, TIME), 0);
        assertEquals(45, aim.update(45, horizontal(Alliance.BLUE, Hive.READY, TIME, 60), TIME,
                TIME + 500_000_000), 0);
        assertEquals(0, aim.update(45, null, 0, TIME + 3_000_000_000L), 0);
    }
}
