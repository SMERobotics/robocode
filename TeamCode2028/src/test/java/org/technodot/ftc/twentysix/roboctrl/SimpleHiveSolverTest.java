package org.technodot.ftc.twentysix.roboctrl;

import org.junit.Test;
import org.technodot.ftc.twentysix.common.Alliance;
import org.technodot.ftc.twentysix.common.Hive;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static org.junit.Assert.*;

public class SimpleHiveSolverTest {
    private static final long TIME = 1_000_000_000L;
    private static final HiveSolver.Calibration CAL = new HiveSolver.Calibration(640, 480,
            610.7225, 611.699, 318.613, 251.2745, new double[5]);

    private static SimpleHiveSolver.Config config() {
        SimpleHiveSolver.Config c = new SimpleHiveSolver.Config();
        c.cameraPitchDegrees = 45; c.cameraRollDegrees = 13;
        c.aimRightInches = 0; c.aimDownInches = 0; c.aimIntoCellInches = 0;
        return c;
    }

    private static List<HiveSolver.TagObservation> image(HiveSolver.Pose pose, int... ids) {
        List<HiveSolver.TagObservation> tags = new ArrayList<>();
        for (int id : ids) {
            double[][] model = HiveGeometry.corners(id), pixels = new double[4][];
            for (int i = 0; i < 4; i++) {
                double[] point = HiveSolverTest.apply(pose.rotation, model[i]);
                for (int k = 0; k < 3; k++) point[k] += pose.translation[k];
                pixels[i] = CAL.project(point);
            }
            tags.add(new HiveSolver.TagObservation(id, pixels));
        }
        return tags;
    }

    @Test public void directGeometryRecoversFocusAndStateForBothSidesAndAllCells() {
        for (HiveGeometry.Cell printed : HiveGeometry.Cell.values()) for (int side : new int[] {-1, 1}) {
            for (double tilt : new double[] {-25, 0, 25}) {
                SimpleHiveSolver.Config c = config();
                double yaw = side == -1 ? 18 : 162;
                HiveSolver.Pose pose = HiveSolverTest.pose(45, 13, yaw, tilt, .1, 1.1, 1.5);
                SimpleHiveSolver solver = new SimpleHiveSolver(c);
                solver.update(image(pose, printed.firstTagId, printed.firstTagId + 3), CAL, TIME, TIME);
                HiveObservation o = solver.getObservation(printed.alliance, TIME);
                assertTrue(o.reason, o.valid);
                assertEquals(side, o.cell.side);
                assertEquals(tilt, o.hingeDegrees, 1e-8);
                assertEquals(tilt * side >= 15 ? Hive.READY : tilt * side <= -15 ? Hive.TIPPED : Hive.TIPPING, o.state);
                double[] expected = HiveSolverTest.apply(pose.rotation, o.cell.focus(0, 0, 0));
                for (int k = 0; k < 3; k++) expected[k] += pose.translation[k];
                assertEquals(expected[0] / HiveGeometry.INCH, o.focusRightInches, 1e-7);
                assertEquals(expected[1] / HiveGeometry.INCH, o.focusDownInches, 1e-7);
                assertEquals(expected[2] / HiveGeometry.INCH, o.focusForwardInches, 1e-7);
            }
        }
    }

    @Test public void aSingleTagAndPartiallyOffscreenCornersAreAccepted() {
        SimpleHiveSolver.Config c = config();
        HiveSolver.Pose pose = HiveSolverTest.pose(45, 13, 0, -25, .8, 1.1, 1);
        SimpleHiveSolver solver = new SimpleHiveSolver(c);
        List<HiveSolver.TagObservation> tags = image(pose, 41);
        assertTrue(tags.get(0).corners()[0][0] > 640);
        solver.update(tags, CAL, TIME, TIME);
        HiveObservation o = solver.getObservation(Alliance.BLUE, TIME);
        assertTrue(o.reason, o.valid);
        assertEquals(Hive.READY, o.state);
        assertTrue(o.singleTagFallback);
    }

    @Test public void everyTagContributesWithoutReprojectionOrAmbiguityRejection() {
        SimpleHiveSolver.Config c = config();
        HiveSolver.Pose pose = HiveSolverTest.pose(45, 13, 0, -25, 0, 1.1, 1);
        List<HiveSolver.TagObservation> tags = image(pose, 38, 39, 40, 41);
        double[][] noisy = tags.get(0).corners();
        for (double[] pixel : noisy) { pixel[0] += 30; pixel[1] -= 10; }
        tags.set(0, new HiveSolver.TagObservation(38, noisy));
        SimpleHiveSolver solver = new SimpleHiveSolver(c);
        solver.update(tags, CAL, TIME, TIME);
        HiveObservation o = solver.getObservation(Alliance.BLUE, TIME);
        assertTrue(o.reason, o.valid);
        assertEquals(4, o.tagIds.size());
        assertTrue(Double.isNaN(o.reprojectionErrorPixels));
    }

    @Test public void statesChangeImmediatelyAndFocusOffsetsFollowTheCell() {
        SimpleHiveSolver.Config c = config();
        c.aimRightInches = 1; c.aimDownInches = 2; c.aimIntoCellInches = 3;
        SimpleHiveSolver solver = new SimpleHiveSolver(c);
        long time = TIME;
        for (double tilt : new double[] {-25, 0, 25}) {
            HiveSolver.Pose pose = HiveSolverTest.pose(45, 13, 0, tilt, 0, 1.1, 1);
            solver.update(image(pose, 38, 41), CAL, time, time);
            HiveObservation o = solver.getObservation(Alliance.BLUE, time);
            assertEquals(tilt < 0 ? Hive.READY : tilt > 0 ? Hive.TIPPED : Hive.TIPPING, o.state);
            double[] expected = HiveSolverTest.apply(pose.rotation, o.cell.focus(1, 2, 3));
            for (int k = 0; k < 3; k++) expected[k] += pose.translation[k];
            assertEquals(HiveMath.norm(expected) / HiveGeometry.INCH, o.rangeInches, 1e-7);
            time += 10_000_000;
        }
    }

    @Test public void alliancesAreIndependentAndOnlyMissingOrOldFramesLoseTheResult() {
        SimpleHiveSolver.Config c = config();
        HiveSolver.Pose pose = HiveSolverTest.pose(45, 13, 0, -25, 0, 1.1, 1);
        SimpleHiveSolver solver = new SimpleHiveSolver(c);
        solver.update(image(pose, 34, 38), CAL, TIME, TIME);
        assertTrue(solver.getObservation(Alliance.RED, TIME).valid);
        assertTrue(solver.getObservation(Alliance.BLUE, TIME).valid);
        assertTrue(solver.getObservation(Alliance.BLUE, TIME + 2_999_000_000L).valid);
        assertFalse(solver.getObservation(Alliance.BLUE, TIME + 3_001_000_000L).valid);
        solver.update(Collections.emptyList(), CAL, TIME + 3_010_000_000L, TIME + 3_010_000_000L);
        assertEquals("no tags", solver.getObservation(Alliance.RED, TIME + 3_010_000_000L).reason);
    }
}
