package org.technodot.ftc.twentysix.roboctrl;

import org.junit.Test;
import org.technodot.ftc.twentysix.common.Alliance;
import org.technodot.ftc.twentysix.common.Hive;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static org.junit.Assert.*;

public class HiveSolverTest {
    static final long TIME = 1_000_000_000L;
    static final HiveSolver.Calibration CAL = new HiveSolver.Calibration(1280, 960,
            900, 900, 640, 480, new double[] {-.04, .01, .001, -.002, 0});

    static HiveSolver.Config config() {
        HiveSolver.Config config = new HiveSolver.Config();
        config.cameraOrientationConfigured = true;
        config.cameraPitchDegrees = 15;
        config.cameraRollDegrees = 0;
        config.aimRightInches = 0;
        config.aimDownInches = 0;
        config.aimIntoCellInches = 0;
        config.settleMillis = 0;
        config.tipConfirmMillis = 0;
        config.stateFilterFrames = 1;
        return config;
    }

    /** Independent axis-angle construction, also used by the native integration tests. */
    static HiveSolver.Pose pose(double pitch, double roll, double yaw, double tilt, double x, double height, double distance) {
        double p = Math.toRadians(pitch), r = Math.toRadians(roll), y = Math.toRadians(yaw), a = Math.toRadians(90 + tilt);
        double[][] mount = multiply(new double[][] {{Math.cos(r), Math.sin(r), 0}, {-Math.sin(r), Math.cos(r), 0}, {0, 0, 1}},
                new double[][] {{1, 0, 0}, {0, Math.cos(p), Math.sin(p)}, {0, -Math.sin(p), Math.cos(p)}});
        double[][] rotation = multiply(mount, multiply(
                new double[][] {{Math.cos(y), 0, Math.sin(y)}, {0, 1, 0}, {-Math.sin(y), 0, Math.cos(y)}},
                new double[][] {{1, 0, 0}, {0, Math.cos(a), -Math.sin(a)}, {0, Math.sin(a), Math.cos(a)}}));
        double[] t = apply(mount, new double[] {x, -height, distance});
        return new HiveSolver.Pose(rotation, t);
    }

    static double[][] multiply(double[][] a, double[][] b) {
        double[][] out = new double[3][3];
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) for (int k = 0; k < 3; k++) out[i][j] += a[i][k] * b[k][j];
        return out;
    }

    static double[] apply(double[][] r, double[] p) {
        double[] out = new double[3];
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) out[i] += r[i][j] * p[j];
        return out;
    }

    static List<HiveSolver.TagObservation> image(HiveSolver.Pose pose, int... ids) {
        List<HiveSolver.TagObservation> tags = new ArrayList<>();
        for (int id : ids) {
            double[][] pixels = new double[4][];
            double[][] object = HiveGeometry.corners(id);
            for (int i = 0; i < 4; i++) {
                double[] p = apply(pose.rotation, object[i]);
                for (int k = 0; k < 3; k++) p[k] += pose.translation[k];
                pixels[i] = CAL.project(p);
            }
            tags.add(new HiveSolver.TagObservation(id, pixels));
        }
        return tags;
    }

    static HiveSolver fixed(HiveSolver.Config config, HiveSolver.Pose pose) {
        return new HiveSolver(config, (tags, calibration) -> Collections.singletonList(pose));
    }

    @Test public void cadAndSdkGeometryAgreeAtBothCells() {
        assertEquals(.08255, HiveGeometry.TAG_SIZE, 1e-12);
        for (HiveGeometry.Cell cell : HiveGeometry.Cell.values()) {
            double[][] c = HiveGeometry.corners(cell.firstTagId);
            double[] focus = cell.focus(0, 0, 0);
            double centerX = (c[0][0] + c[1][0]) / 2;
            assertEquals(-6.5, (centerX - focus[0]) / (-cell.side * HiveGeometry.INCH), 1e-9);
            assertEquals(7.1874, (c[0][1] - HiveGeometry.TAG_SIZE / 2 * -cell.side - focus[1]) / (-cell.side * HiveGeometry.INCH), 1e-9);
            assertEquals(-5.622, -focus[2] / HiveGeometry.INCH, 1e-9);
            assertEquals(0, c[0][2], 0);
        }
    }

    @Test public void projectionIncludesDistortionAndScalesCalibration() {
        HiveSolver.Calibration c = new HiveSolver.Calibration(640, 480, 500, 510, 320, 240, new double[] {.1, 0, 0, 0, 0});
        assertArrayEquals(new double[] {320 + 500 * .2 * 1.005, 240 + 510 * .1 * 1.005}, c.project(new double[] {.2, .1, 1}), 1e-9);
        double[] scaled = c.scaledTo(1280, 960).project(new double[] {.2, .1, 1});
        assertEquals(c.project(new double[] {.2, .1, 1})[0] * 2, scaled[0], 1e-9);
    }

    @Test(expected = IllegalArgumentException.class) public void rejectCalibrationCropOrAspectChange() {
        CAL.scaledTo(1280, 800);
    }

    @Test public void audienceReadyAndFocusAnglesHaveOpticalSigns() {
        HiveSolver.Config c = config();
        HiveSolver.Pose p = pose(15, 0, 0, -30, .1, .7, 2.5);
        HiveSolver solver = fixed(c, p);
        solver.update(image(p, 38, 39, 40, 41), CAL, TIME, TIME);
        HiveObservation o = solver.getObservation(Alliance.BLUE, TIME);
        assertTrue(o.reason, o.valid);
        assertEquals(Hive.READY, o.state);
        assertEquals(HiveGeometry.Cell.BLUE_AUDIENCE, o.cell);
        assertTrue(o.bearingDegrees < 0); // target is to the camera's right
        assertEquals(4, o.tagIds.size());
        double[] target = apply(p.rotation, HiveGeometry.Cell.BLUE_AUDIENCE.focus(0, 0, 0));
        for (int i = 0; i < 3; i++) target[i] += p.translation[i];
        assertEquals(HiveMath.norm(target) / HiveGeometry.INCH, o.rangeInches, 1e-5);
        assertEquals(Math.toDegrees(Math.atan2(-target[1], Math.hypot(target[0], target[2]))), o.elevationDegrees, 1e-5);
    }

    @Test public void upsideDownOppositeRowStillIdentifiesCameraSide() {
        HiveSolver.Pose p = pose(15, 0, 0, -30, 0, .7, 2.5);
        HiveSolver solver = fixed(config(), p);
        solver.update(image(p, 42, 43, 44, 45), CAL, TIME, TIME);
        HiveObservation o = solver.getObservation(Alliance.BLUE, TIME);
        assertTrue(o.reason, o.valid);
        assertEquals(HiveGeometry.Cell.BLUE_AUDIENCE, o.cell);
        assertEquals(Hive.READY, o.state);
    }

    @Test public void scoringSideAndBothAlliancesAreIndependent() {
        HiveSolver.Config c = config();
        HiveSolver.Pose p = pose(15, 0, 180, 30, 0, .7, 2.5);
        HiveSolver solver = fixed(c, p);
        List<HiveSolver.TagObservation> tags = image(p, 30, 31, 32, 33, 42, 43, 44, 45);
        solver.update(tags, CAL, TIME, TIME);
        for (Alliance alliance : Alliance.values()) {
            HiveObservation o = solver.getObservation(alliance, TIME);
            assertTrue(o.reason, o.valid);
            assertEquals(Hive.READY, o.state);
            assertEquals(1, o.cell.side);
            assertEquals(4, o.tagIds.size());
        }
    }

    @Test public void tippedAndTransitionUseGeometryNotDetectionLoss() {
        HiveSolver.Config c = config(); c.cameraPitchDegrees = 45;
        HiveSolver.Pose down = pose(45, 0, 0, 30, 0, 1.1, 1);
        HiveSolver solver = fixed(c, down);
        solver.update(image(down, 38, 41), CAL, TIME, TIME);
        assertEquals(Hive.TIPPED, solver.getObservation(Alliance.BLUE, TIME).state);
        HiveSolver.Pose mid = pose(45, 0, 0, 0, 0, 1.1, 1);
        solver = fixed(c, mid);
        solver.update(image(mid, 38, 41), CAL, TIME, TIME);
        assertEquals(Hive.TIPPING, solver.getObservation(Alliance.BLUE, TIME).state);
        solver.update(Collections.emptyList(), CAL, TIME + 1, TIME + 1);
        HiveObservation missing = solver.getObservation(Alliance.BLUE, TIME + 1);
        assertFalse(missing.valid); assertNull(missing.state); assertTrue(Double.isNaN(missing.rangeInches));
    }

    @Test public void settlingRequiresDistinctFramesAndStaleDataExpires() {
        HiveSolver.Config c = config(); c.settleMillis = 100;
        HiveSolver.Pose p = pose(15, 0, 0, -30, 0, .7, 2.5);
        HiveSolver solver = fixed(c, p);
        List<HiveSolver.TagObservation> tags = image(p, 38, 41);
        solver.update(tags, CAL, TIME, TIME);
        assertEquals("settling at stop", solver.getObservation(Alliance.BLUE, TIME).reason);
        solver.update(tags, CAL, TIME + 60_000_000, TIME + 60_000_000);
        assertFalse(solver.getObservation(Alliance.BLUE, TIME + 60_000_000).valid);
        solver.update(tags, CAL, TIME + 60_000_000, TIME + 120_000_000);
        assertFalse(solver.getObservation(Alliance.BLUE, TIME + 120_000_000).valid);
        solver.update(tags, CAL, TIME + 120_000_000, TIME + 120_000_000);
        assertEquals(Hive.READY, solver.getObservation(Alliance.BLUE, TIME + 120_000_000).state);
        HiveObservation stale = solver.getObservation(Alliance.BLUE, TIME + 330_000_000);
        assertFalse(stale.valid); assertEquals("stale image", stale.reason);
    }

    @Test public void unconfiguredTiltAndDuplicateIdsFailClosed() {
        HiveSolver.Pose p = pose(15, 0, 0, -30, 0, .7, 2.5);
        HiveSolver.Config c = new HiveSolver.Config();
        c.cameraOrientationConfigured = false;
        HiveSolver solver = fixed(c, p);
        solver.update(image(p, 38, 41), CAL, TIME, TIME);
        assertEquals("configure camera pitch and roll", solver.getObservation(Alliance.BLUE, TIME).reason);
        solver = fixed(config(), p);
        solver.update(image(p, 38, 38), CAL, TIME, TIME);
        assertEquals("duplicate tag IDs", solver.getObservation(Alliance.BLUE, TIME).reason);
    }

    @Test public void initializationFailureIsNotMaskedByUnconfiguredCamera() {
        HiveSolver.Config c = new HiveSolver.Config();
        c.cameraOrientationConfigured = false;
        HiveSolver solver = new HiveSolver(c, (tags, calibration) -> {
            throw new AssertionError("Initialization failure must not attempt a solve");
        });
        solver.reset("OpenCV native initialization failed");
        for (Alliance alliance : Alliance.values()) {
            HiveObservation observation = solver.getObservation(alliance, TIME);
            assertFalse(observation.valid);
            assertNull(observation.state);
            assertTrue(Double.isNaN(observation.rangeInches));
            assertEquals("OpenCV native initialization failed", observation.reason);
        }
    }

    @Test public void disablingCameraConfigurationImmediatelyInvalidatesAnExistingPose() {
        HiveSolver.Config c = config();
        HiveSolver.Pose p = pose(15, 0, 0, -30, 0, .7, 2.5);
        HiveSolver solver = fixed(c, p);
        solver.update(image(p, 38, 41), CAL, TIME, TIME);
        assertTrue(solver.getObservation(Alliance.BLUE, TIME).valid);
        c.cameraOrientationConfigured = false;
        HiveObservation observation = solver.getObservation(Alliance.BLUE, TIME);
        assertFalse(observation.valid);
        assertEquals("configure camera pitch and roll", observation.reason);
        c.cameraOrientationConfigured = true;
        assertFalse(solver.getObservation(Alliance.BLUE, TIME).valid);
        solver.update(image(p, 38, 41), CAL, TIME + 1, TIME + 1);
        assertTrue(solver.getObservation(Alliance.BLUE, TIME + 1).valid);
    }

    @Test public void optionalSingleTagAndAimOffsetAreExplicit() {
        HiveSolver.Config c = config(); c.aimRightInches = 2;
        HiveSolver.Pose p = pose(15, 0, 0, -30, 0, .7, 2.5);
        HiveSolver solver = fixed(c, p);
        solver.update(image(p, 38), CAL, TIME, TIME);
        HiveObservation o = solver.getObservation(Alliance.BLUE, TIME);
        assertTrue(o.reason, o.valid); assertTrue(o.singleTagFallback);
        assertEquals(2 + HiveGeometry.ROW_CENTER_X / HiveGeometry.INCH, o.focusRightInches, 1e-5);
        c.allowSingleTag = false;
        solver.update(image(p, 38), CAL, TIME + 1, TIME + 1);
        assertEquals("multiple tags required", solver.getObservation(Alliance.BLUE, TIME + 1).reason);
    }

    @Test public void frameWithNoUsableCornersCannotReusePose() {
        HiveSolver.Pose p = pose(15, 0, 0, -30, 0, .7, 2.5);
        HiveSolver solver = fixed(config(), p);
        solver.update(image(p, 38, 41), CAL, TIME, TIME);
        double[][] invalid = {{-1, 0}, {2, 0}, {2, 2}, {-1, 2}};
        solver.update(Arrays.asList(new HiveSolver.TagObservation(38, invalid)), CAL, TIME + 1, TIME + 1);
        assertFalse(solver.getObservation(Alliance.BLUE, TIME + 1).valid);
    }

    @Test public void motionRequiresResettlingAndStopHysteresisDoesNotLatchEarly() {
        HiveSolver.Config c = config();
        final HiveSolver.Pose[] current = {pose(15, 0, 0, -30, 0, .7, 2.5)};
        HiveSolver solver = new HiveSolver(c, (tags, calibration) -> Collections.singletonList(current[0]));
        solver.update(image(current[0], 38, 41), CAL, TIME, TIME);
        assertEquals(Hive.READY, solver.getObservation(Alliance.BLUE, TIME).state);
        current[0] = pose(15, 0, 0, -10, 0, .7, 2.5);
        solver.update(image(current[0], 38, 41), CAL, TIME + 20_000_000, TIME + 20_000_000);
        assertEquals(Hive.TIPPING, solver.getObservation(Alliance.BLUE, TIME + 20_000_000).state);
        // Twelve degrees from the stop is within the hold band, outside the enter band.
        current[0] = pose(15, 0, 0, -18, 0, .7, 2.5);
        for (int i = 1; i <= 20; i++) {
            long t = TIME + 20_000_000 + i * 30_000_000L;
            solver.update(image(current[0], 38, 41), CAL, t, t);
            assertEquals(Hive.TIPPING, solver.getObservation(Alliance.BLUE, t).state);
        }
    }

    @Test public void settledStopsAllowExtraTravelAndFastWobbleBeforeTipping() {
        for (HiveGeometry.Cell cell : HiveGeometry.Cell.values()) for (Hive endpoint : new Hive[] {Hive.READY, Hive.TIPPED}) {
            HiveSolver.Config c = config(); c.cameraPitchDegrees = 45;
            double yaw = cell.side == -1 ? 0 : 180;
            double stop = (endpoint == Hive.READY ? 30 : -30) * cell.side;
            final HiveSolver.Pose[] current = {pose(45, 0, yaw, stop, 0, 1.1, 1)};
            HiveSolver solver = new HiveSolver(c, (tags, calibration) -> Collections.singletonList(current[0]));
            long time = TIME;
            solver.update(image(current[0], cell.firstTagId, cell.firstTagId + 3), CAL, time, time);
            assertEquals(cell + " " + endpoint, endpoint, solver.getObservation(cell.alliance, time).state);
            for (double excursion : new double[] {6, 12, 9, 12, 7}) {
                // Rapid 20 ms steps exercise the rate trigger as well as position tolerance.
                current[0] = pose(45, 0, yaw, stop - Math.signum(stop) * excursion, 0, 1.1, 1);
                time += 20_000_000;
                solver.update(image(current[0], cell.firstTagId, cell.firstTagId + 3), CAL, time, time);
                assertEquals(cell + " wobble " + excursion, endpoint, solver.getObservation(cell.alliance, time).state);
            }
            current[0] = pose(45, 0, yaw, stop - Math.signum(stop) * 15, 0, 1.1, 1);
            time += 20_000_000;
            solver.update(image(current[0], cell.firstTagId, cell.firstTagId + 3), CAL, time, time);
            assertEquals(cell + " leaves stop", Hive.TIPPING, solver.getObservation(cell.alliance, time).state);
        }
    }

    @Test public void noisyTwentyFiveDegreeStopsAreRecognizedWithoutAFalseMotionState() {
        for (HiveGeometry.Cell cell : HiveGeometry.Cell.values()) for (Hive endpoint : new Hive[] {Hive.READY, Hive.TIPPED}) {
            HiveSolver.Config c = config(); c.cameraPitchDegrees = 45;
            c.stateFilterFrames = 5; c.tipConfirmMillis = 150; c.settleMillis = 100;
            double yaw = cell.side == -1 ? 0 : 180;
            double center = (endpoint == Hive.READY ? 25 : -25) * cell.side;
            final HiveSolver.Pose[] current = {pose(45, 0, yaw, center, 0, 1.1, 1)};
            HiveSolver solver = new HiveSolver(c, (tags, calibration) -> Collections.singletonList(current[0]));
            for (int i = 0; i < 40; i++) {
                // Pose noise at 100 Hz used to keep the derivative's motion flag active forever.
                current[0] = pose(45, 0, yaw, center + (i % 2 == 0 ? -2 : 2), 0, 1.1, 1);
                long time = TIME + i * 10_000_000L;
                solver.update(image(current[0], cell.firstTagId, cell.firstTagId + 3), CAL, time, time);
                HiveObservation o = solver.getObservation(cell.alliance, time);
                if (i >= 12) {
                    assertTrue(cell + ": " + o.reason, o.valid);
                    assertEquals(cell + " at " + center, endpoint, o.state);
                } else {
                    assertNotEquals(Hive.TIPPING, o.state);
                }
            }
        }
    }

    @Test public void briefAngleOutliersDoNotTipButSustainedDepartureAndArrivalAreConfirmed() {
        for (Hive endpoint : new Hive[] {Hive.READY, Hive.TIPPED}) {
            HiveSolver.Config c = config(); c.cameraPitchDegrees = 45;
            c.stateFilterFrames = 5; c.tipConfirmMillis = 150; c.settleMillis = 100;
            double stop = endpoint == Hive.READY ? -25 : 25;
            final HiveSolver.Pose[] current = {pose(45, 0, 0, stop, 0, 1.1, 1)};
            HiveSolver solver = new HiveSolver(c, (tags, calibration) -> Collections.singletonList(current[0]));
            long time = TIME;
            for (int i = 0; i < 12; i++) {
                time += 20_000_000;
                solver.update(image(current[0], 38, 41), CAL, time, time);
            }
            assertEquals(endpoint, solver.getObservation(Alliance.BLUE, time).state);
            // A short burst can move even the median outside the hold band, but
            // must not accumulate evidence across separate bursts.
            for (int i = 0; i < 40; i++) {
                double angle = i % 10 < 3 ? 0 : stop;
                current[0] = pose(45, 0, 0, angle, 0, 1.1, 1);
                time += 20_000_000;
                solver.update(image(current[0], 38, 41), CAL, time, time);
                HiveObservation o = solver.getObservation(Alliance.BLUE, time);
                assertEquals(endpoint, o.state);
                if (i % 10 == 0) {
                    assertEquals(0, o.hingeDegrees, 1e-6); // pose diagnostics stay current
                    assertEquals(stop, o.stateHingeDegrees, 1e-6); // state rejects one-frame noise
                }
            }
            current[0] = pose(45, 0, 0, 0, 0, 1.1, 1);
            for (int i = 0; i < 18; i++) {
                time += 20_000_000;
                solver.update(image(current[0], 38, 41), CAL, time, time);
            }
            assertEquals(Hive.TIPPING, solver.getObservation(Alliance.BLUE, time).state);
            current[0] = pose(45, 0, 0, -stop, 0, 1.1, 1);
            for (int i = 0; i < 14; i++) {
                time += 20_000_000;
                solver.update(image(current[0], 38, 41), CAL, time, time);
            }
            assertEquals(endpoint == Hive.READY ? Hive.TIPPED : Hive.READY,
                    solver.getObservation(Alliance.BLUE, time).state);
        }
    }

    @Test public void aRepeatedImageCannotConfirmDepartureFromAStop() {
        HiveSolver.Config c = config(); c.cameraPitchDegrees = 45;
        c.tipConfirmMillis = 150;
        final HiveSolver.Pose[] current = {pose(45, 0, 0, -25, 0, 1.1, 1)};
        HiveSolver solver = new HiveSolver(c, (tags, calibration) -> Collections.singletonList(current[0]));
        solver.update(image(current[0], 38, 41), CAL, TIME, TIME);
        current[0] = pose(45, 0, 0, 0, 0, 1.1, 1);
        List<HiveSolver.TagObservation> middle = image(current[0], 38, 41);
        long departure = TIME + 20_000_000;
        solver.update(middle, CAL, departure, departure);
        for (int i = 1; i <= 8; i++) {
            long now = departure + i * 20_000_000;
            solver.update(middle, CAL, departure, now);
            assertEquals(Hive.READY, solver.getObservation(Alliance.BLUE, now).state);
        }
        solver.update(middle, CAL, departure + 170_000_000, departure + 170_000_000);
        assertEquals(Hive.READY, solver.getObservation(Alliance.BLUE, departure + 170_000_000).state);
        solver.update(middle, CAL, departure + 190_000_000, departure + 190_000_000);
        assertEquals(Hive.TIPPING, solver.getObservation(Alliance.BLUE, departure + 190_000_000).state);
    }

    @Test public void changedCameraTiltAndRestartClearSettlingHistory() {
        HiveSolver.Config c = config(); c.settleMillis = 100;
        final HiveSolver.Pose[] current = {pose(15, 0, 0, -30, 0, .7, 2.5)};
        HiveSolver solver = new HiveSolver(c, (tags, calibration) -> Collections.singletonList(current[0]));
        solver.update(image(current[0], 38, 41), CAL, TIME, TIME);
        solver.update(image(current[0], 38, 41), CAL, TIME + 60_000_000, TIME + 60_000_000);
        solver.update(image(current[0], 38, 41), CAL, TIME + 120_000_000, TIME + 120_000_000);
        assertTrue(solver.getObservation(Alliance.BLUE, TIME + 120_000_000).valid);
        c.cameraPitchDegrees = 16;
        current[0] = pose(16, 0, 0, -30, 0, .7, 2.5);
        solver.update(image(current[0], 38, 41), CAL, TIME + 130_000_000, TIME + 130_000_000);
        assertEquals("settling at stop", solver.getObservation(Alliance.BLUE, TIME + 130_000_000).reason);
        solver.reset("camera restarted");
        solver.update(image(current[0], 38, 41), CAL, TIME + 150_000_000, TIME + 150_000_000);
        assertEquals("settling at stop", solver.getObservation(Alliance.BLUE, TIME + 150_000_000).reason);
    }
}
