package org.technodot.ftc.twentysix.roboctrl;

import org.junit.Assume;
import org.junit.BeforeClass;
import org.junit.Test;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.technodot.ftc.twentysix.common.Alliance;
import org.technodot.ftc.twentysix.common.Hive;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

/** Run tools/test_hive_solver.py for real desktop OpenCV and JSON (not Android's stubs). */
public class HiveSolverNativeTest {
    // Captured from the connected LL3A's /cal-default response.
    private static final String CAMERA_CALIBRATION = "{\"CALIBRATION_RESULT\":{\"DISPLAY_NAME\":\"\","
            + "\"DISTORTION_COEFFICIENTS\":[0.17716814022120847,-0.4573406421907695,"
            + "0.002752733239126054,0.00036002841308005,0.17825941966394934],"
            + "\"INTRINSICS_MATRIX\":[1221.445,0,637.226,0,1223.398,502.549,0,0,1],"
            + "\"REPROJECTION_ERROR\":0,\"RES_X\":1280,\"RES_Y\":960}}";

    @BeforeClass public static void loadNativeRuntime() throws Exception {
        Assume.assumeTrue("Native suite uses tools/test_hive_solver.py", Boolean.getBoolean("hive.nativeTests"));
        Class.forName("nu.pattern.OpenCV").getMethod("loadLocally").invoke(null);
    }

    private static List<HiveSolver.TagObservation> nativeImage(HiveSolver.Pose pose, Random random, double noise, int... ids) {
        Mat rotation = new Mat(3, 3, CvType.CV_64F), rvec = new Mat(), tvec = new Mat(3, 1, CvType.CV_64F);
        Mat k = Mat.eye(3, 3, CvType.CV_64F);
        MatOfDouble d = new MatOfDouble(HiveSolverTest.CAL.distortion());
        List<HiveSolver.TagObservation> result = new ArrayList<>();
        try {
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) rotation.put(i, j, pose.rotation[i][j]);
                tvec.put(i, 0, pose.translation[i]);
            }
            Calib3d.Rodrigues(rotation, rvec);
            k.put(0, 0, 900); k.put(1, 1, 900); k.put(0, 2, 640); k.put(1, 2, 480);
            for (int id : ids) {
                double[][] corners = HiveGeometry.corners(id);
                Point3[] points = new Point3[4];
                for (int i = 0; i < 4; i++) points[i] = new Point3(corners[i][0], corners[i][1], corners[i][2]);
                MatOfPoint3f object = new MatOfPoint3f(points);
                MatOfPoint2f projected = new MatOfPoint2f();
                try {
                    Calib3d.projectPoints(object, rvec, tvec, k, d, projected);
                    Point[] image = projected.toArray();
                    double[][] pixels = new double[4][2];
                    for (int i = 0; i < 4; i++) {
                        pixels[i][0] = image[i].x + noise * random.nextGaussian();
                        pixels[i][1] = image[i].y + noise * random.nextGaussian();
                    }
                    result.add(new HiveSolver.TagObservation(id, pixels));
                } finally { object.release(); projected.release(); }
            }
        } finally { rotation.release(); rvec.release(); tvec.release(); k.release(); d.release(); }
        return result;
    }

    private static HiveObservation solve(HiveSolver.Config config, List<HiveSolver.TagObservation> tags, Alliance alliance) {
        HiveSolver solver = new HiveSolver(config, new OpenCvHivePoseSeeds());
        solver.update(tags, HiveSolverTest.CAL, HiveSolverTest.TIME, HiveSolverTest.TIME);
        return solver.getObservation(alliance, HiveSolverTest.TIME);
    }

    @Test public void realIppeFitsOneThroughEightTagsAndBothCells() {
        int[][] masks = {{38}, {38, 41}, {38, 40, 41}, {38, 39, 40, 41}, {42, 43, 44, 45},
                {38, 39, 40, 41, 42}, {38, 39, 40, 41, 42, 43}, {38, 39, 40, 41, 42, 43, 44},
                {38, 39, 40, 41, 42, 43, 44, 45}};
        for (int[] mask : masks) {
            HiveSolver.Config config = HiveSolverTest.config(); config.cameraRollDegrees = 3;
            HiveSolver.Pose truth = HiveSolverTest.pose(15, 3, 12, -30, .15, .7, 2.5);
            HiveObservation observation = solve(config, nativeImage(truth, new Random(1), 0, mask), Alliance.BLUE);
            assertTrue("mask " + mask.length + ": " + observation.reason, observation.valid);
            assertEquals(Hive.READY, observation.state);
            assertEquals(mask.length, observation.tagIds.size());
            assertEquals(-30, observation.hingeDegrees, .01);
            assertTrue(observation.reprojectionErrorPixels < .001);
        }
    }

    @Test public void bothAlliancesAndViewingDirectionsWorkWithCameraRoll() {
        for (int first : new int[] {30, 34, 38, 42}) {
            HiveGeometry.Cell cell = HiveGeometry.cellForTag(first);
            double yaw = cell.side == -1 ? 0 : 180;
            HiveSolver.Config config = HiveSolverTest.config(); config.cameraRollDegrees = -7;
            HiveSolver.Pose truth = HiveSolverTest.pose(15, -7, yaw, cell.side * 30, 0, .7, 2.5);
            HiveObservation observation = solve(config, nativeImage(truth, new Random(2), .1, first, first + 1, first + 2, first + 3), cell.alliance);
            assertTrue(cell + ": " + observation.reason, observation.valid);
            assertEquals(cell, observation.cell);
            assertEquals(Hive.READY, observation.state);
            assertEquals(cell.side * 30, observation.hingeDegrees, 1);
        }
    }

    @Test public void realIppeRecognizesTippedAndMidTravelWhenTagsAreVisible() {
        HiveSolver.Config config = HiveSolverTest.config(); config.cameraPitchDegrees = 45;
        for (double tilt : new double[] {-30, -15, 0, 15, 30}) {
            HiveSolver.Pose truth = HiveSolverTest.pose(45, 0, 0, tilt, 0, 1.1, 1);
            HiveObservation o = solve(config, nativeImage(truth, new Random(1), 0, 38, 41, 42, 45), Alliance.BLUE);
            assertTrue("tilt " + tilt + ": " + o.reason, o.valid);
            assertEquals(tilt == -30 ? Hive.READY : tilt == 30 ? Hive.TIPPED : Hive.TIPPING, o.state);
            assertEquals(tilt, o.hingeDegrees, .01);
        }
    }

    @Test public void shootingDistanceAndObliqueViewsRemainAccurate() {
        for (double distance : new double[] {.9, 1.5, 2.5, 3.65}) for (double yaw : new double[] {-25, 0, 25}) {
            HiveSolver.Config config = HiveSolverTest.config(); config.cameraPitchDegrees = 30;
            HiveSolver.Pose truth = HiveSolverTest.pose(30, 0, yaw, -30, 0, .5, distance);
            HiveObservation o = solve(config, nativeImage(truth, new Random(3), .1, 38, 39, 40, 41), Alliance.BLUE);
            assertTrue("distance " + distance + " yaw " + yaw + ": " + o.reason, o.valid);
            assertEquals(-30, o.hingeDegrees, 1);
            double[] f = HiveSolverTest.apply(truth.rotation, o.cell.focus(0, 0, 0));
            for (int i = 0; i < 3; i++) f[i] += truth.translation[i];
            assertEquals(HiveMath.norm(f) / HiveGeometry.INCH, o.rangeInches, 1);
        }
    }

    @Test public void yawingCameraDoesNotLookLikeHiveMotion() {
        HiveSolver.Config config = HiveSolverTest.config(); config.cameraRollDegrees = 4;
        HiveSolver solver = new HiveSolver(config, new OpenCvHivePoseSeeds());
        HiveSolver.Pose initial = HiveSolverTest.pose(15, 4, 0, -30, 0, .7, 2.5);
        // Camera yaw about gravity changes both optical translation and rotation.
        double[][] mount = HiveSolverTest.pose(15, 4, 0, -90, 0, 0, 0).rotation;
        double[][] inverseMount = new double[3][3];
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) inverseMount[i][j] = mount[j][i];
        for (int step = 0; step < 11; step++) {
            double a = Math.toRadians(step * 2 - 10);
            double[][] yaw = {{Math.cos(a), 0, Math.sin(a)}, {0, 1, 0}, {-Math.sin(a), 0, Math.cos(a)}};
            double[][] change = HiveSolverTest.multiply(mount, HiveSolverTest.multiply(yaw, inverseMount));
            HiveSolver.Pose view = new HiveSolver.Pose(HiveSolverTest.multiply(change, initial.rotation),
                    HiveSolverTest.apply(change, initial.translation));
            long time = HiveSolverTest.TIME + step * 20_000_000L;
            solver.update(nativeImage(view, new Random(step), 0, 38, 39, 40, 41), HiveSolverTest.CAL, time, time);
            HiveObservation o = solver.getObservation(Alliance.BLUE, time);
            assertTrue(o.reason, o.valid); assertEquals(Hive.READY, o.state);
        }
    }

    @Test public void noisy640PixelStopsStayConfirmedWithRealIppeAtHighFrameRate() {
        HiveSolver.Calibration calibration = HiveSolverTest.CAL.scaledTo(640, 480);
        for (double tilt : new double[] {-25, 25}) {
            HiveSolver.Config config = HiveSolverTest.config(); config.cameraPitchDegrees = 45;
            config.stateFilterFrames = 5; config.tipConfirmMillis = 150; config.settleMillis = 100;
            HiveSolver.Pose truth = HiveSolverTest.pose(45, 0, 0, tilt, 0, 1.1, 1);
            HiveSolver solver = new HiveSolver(config, new OpenCvHivePoseSeeds());
            Random random = new Random(218);
            int confirmed = 0;
            for (int i = 0; i < 70; i++) {
                List<HiveSolver.TagObservation> tags = new ArrayList<>();
                for (HiveSolver.TagObservation tag : nativeImage(truth, random, .7, 38, 41, 42, 45)) {
                    double[][] pixels = tag.corners();
                    for (double[] point : pixels) { point[0] /= 2; point[1] /= 2; }
                    tags.add(new HiveSolver.TagObservation(tag.id, pixels));
                }
                long time = HiveSolverTest.TIME + i * 10_000_000L;
                solver.update(tags, calibration, time, time);
                HiveObservation observation = solver.getObservation(Alliance.BLUE, time);
                assertNotEquals("stationary stop at " + tilt, Hive.TIPPING, observation.state);
                if (observation.valid) {
                    assertEquals(tilt < 0 ? Hive.READY : Hive.TIPPED, observation.state);
                    confirmed++;
                }
            }
            assertTrue("stop must be usable after settling: " + confirmed, confirmed >= 50);
        }
    }

    @Test public void inconsistentTagIsRejectedAsAWhole() {
        HiveSolver.Config config = HiveSolverTest.config();
        HiveSolver.Pose truth = HiveSolverTest.pose(15, 0, 0, -30, 0, .7, 2.5);
        List<HiveSolver.TagObservation> tags = nativeImage(truth, new Random(4), .15, 38, 39, 40, 41);
        double[][] bad = tags.get(1).corners();
        for (double[] p : bad) { p[0] += 45; p[1] -= 20; }
        tags.set(1, new HiveSolver.TagObservation(39, bad));
        HiveObservation o = solve(config, tags, Alliance.BLUE);
        assertTrue(o.reason, o.valid);
        assertEquals(3, o.tagIds.size()); assertFalse(o.tagIds.contains(39));
        assertEquals(-30, o.hingeDegrees, 1);
    }

    @Test public void multiTagNoiseErrorBeatsSingleTagBaseline() {
        HiveSolver.Config config = HiveSolverTest.config();
        HiveSolver.Pose truth = HiveSolverTest.pose(15, 0, 8, -30, .1, .7, 3);
        double[] focus = HiveSolverTest.apply(truth.rotation, HiveGeometry.Cell.BLUE_AUDIENCE.focus(0, 0, 0));
        for (int k = 0; k < 3; k++) focus[k] += truth.translation[k];
        double expected = HiveMath.norm(focus) / HiveGeometry.INCH;
        double singleError = 0, multiError = 0;
        int singles = 0, multis = 0;
        Random random = new Random(117);
        for (int i = 0; i < 30; i++) {
            List<HiveSolver.TagObservation> tags = nativeImage(truth, random, .35, 38, 39, 40, 41);
            HiveObservation single = solve(config, tags.subList(0, 1), Alliance.BLUE);
            HiveObservation multi = solve(config, tags, Alliance.BLUE);
            if (single.valid) { singleError += Math.pow(single.rangeInches - expected, 2); singles++; }
            if (multi.valid) { multiError += Math.pow(multi.rangeInches - expected, 2); multis++; }
        }
        assertTrue("single baseline availability " + singles, singles >= 15);
        assertTrue("multi-tag availability " + multis, multis >= 27);
        double singleRms = Math.sqrt(singleError / singles), multiRms = Math.sqrt(multiError / multis);
        System.out.printf("Range RMS with 0.35 px noise: single %.3f in (%d), joint %.3f in (%d)%n", singleRms, singles, multiRms, multis);
        assertTrue("joint fit should improve range", multiRms < singleRms * .75);
    }

    @Test public void ambiguityIsRejectedInsteadOfSelectingFirstIppePose() {
        HiveSolver.Config config = HiveSolverTest.config(); config.cameraPitchDegrees = 90;
        Random random = new Random(58);
        int rejected = 0;
        for (int i = 0; i < 40; i++) {
            HiveSolver.Pose truth = HiveSolverTest.pose(90, 0, 0, -10, .01, 3, .25);
            HiveObservation o = solve(config, nativeImage(truth, random, .25, 38), Alliance.BLUE);
            if (o.reason.equals("ambiguous planar pose")) {
                rejected++; assertNull(o.state); assertTrue(Double.isNaN(o.rangeInches));
            }
        }
        assertTrue("near-normal single-tag views should expose a competing pose", rejected > 0);
    }

    @Test public void inputAcceptsNestedAndFlatCornersAndPreservesDecodedOrder() throws Exception {
        String header = "{\"fidx\":7,\"ts\":42,\"pID\":0,\"v\":1,\"cl\":10,\"tl\":5,\"Fiducial\":[{\"fID\":38,\"pts\":";
        String tail = "}]}";
        LimelightHiveFrame a = new LimelightHiveFrame(header + "[[10,20],[30,20],[30,10],[10,10]]" + tail, 0, false);
        LimelightHiveFrame b = new LimelightHiveFrame(header + "[10,20,30,20,30,10,10,10]" + tail, 0, false);
        assertEquals(7, a.counter); assertEquals(42000, a.timestampMicros, 0); assertEquals(15, a.latencyMillis, 0);
        assertArrayEquals(a.tags.get(0).corners()[3], b.tags.get(0).corners()[3], 0);
        LimelightHiveFrame rotated = new LimelightHiveFrame(header + "[[30,10],[10,10],[10,20],[30,20]]" + tail, 2, false);
        assertArrayEquals(a.tags.get(0).corners()[0], rotated.tags.get(0).corners()[0], 0);
        LimelightHiveFrame missing = new LimelightHiveFrame(header + "[]" + tail, 0, false);
        assertTrue(missing.tags.isEmpty());
    }

    @Test public void tagSightingsAreAllianceSpecificAndDoNotRequireAUsablePose() throws Exception {
        String json = "{\"fidx\":8,\"pID\":0,\"v\":1,\"Fiducial\":["
                + "{\"fID\":30,\"pts\":[]},{\"fID\":45},{\"fID\":1}]}";
        LimelightHiveFrame frame = new LimelightHiveFrame(json, 0, false);
        assertTrue(frame.tags.isEmpty());
        assertEquals(2, frame.visibleHives.size());
        assertTrue(frame.visibleHives.contains(Alliance.RED));
        assertTrue(frame.visibleHives.contains(Alliance.BLUE));
        assertTrue(new LimelightHiveFrame(json.replace("\"v\":1", "\"v\":0"), 0, false).visibleHives.isEmpty());
        assertTrue(new LimelightHiveFrame("{\"fidx\":9,\"v\":1,\"Fiducial\":[{\"fID\":1}]}",
                0, false).visibleHives.isEmpty());
    }

    @Test public void cameraCalibrationEnvelopePreservesMeasuredIntrinsicsAndDistortion() throws Exception {
        HiveSolver.Calibration c = LimelightCalibration.parse(CAMERA_CALIBRATION, 1280, 960, false);
        assertEquals(1280, c.width); assertEquals(960, c.height);
        assertEquals(1221.445, c.fx, 1e-9); assertEquals(1223.398, c.fy, 1e-9);
        assertEquals(637.226, c.cx, 1e-9); assertEquals(502.549, c.cy, 1e-9);
        assertArrayEquals(new double[] {.17716814022120847, -.4573406421907695,
                .002752733239126054, .00036002841308005, .17825941966394934}, c.distortion(), 1e-15);
    }

    @Test public void flatCalibrationRemainsSupportedAndScalesToImageSize() throws Exception {
        String flat = new JSONObject(CAMERA_CALIBRATION).getJSONObject("CALIBRATION_RESULT").toString();
        HiveSolver.Calibration c = LimelightCalibration.parse(flat, 640, 480, false);
        assertEquals(1221.445 / 2, c.fx, 1e-9); assertEquals(502.549 / 2, c.cy, 1e-9);
        assertEquals(.17716814022120847, c.distortion()[0], 1e-15);
        assertThrows(IllegalArgumentException.class, () -> LimelightCalibration.parse(flat, 1280, 800, false));
    }

    @Test public void calibrationDistortionIsDisabledOnlyForExplicitlyUndistortedPixels() throws Exception {
        HiveSolver.Calibration c = LimelightCalibration.parse(CAMERA_CALIBRATION, 1280, 960, true);
        assertArrayEquals(new double[5], c.distortion(), 0);
        assertEquals(1221.445, c.fx, 1e-9);
    }

    @Test public void malformedCalibrationCannotBecomeAnApparentlyValidCamera() throws Exception {
        assertThrows(JSONException.class, () -> LimelightCalibration.parse("{}", 1280, 960, false));
        assertThrows(JSONException.class, () -> LimelightCalibration.parse(
                "{\"CALIBRATION_RESULT\":null}", 1280, 960, false));
        JSONObject data = new JSONObject(CAMERA_CALIBRATION).getJSONObject("CALIBRATION_RESULT");
        data.put("DISTORTION_COEFFICIENTS", new JSONArray());
        String missingCoefficients = data.toString();
        assertThrows(IllegalArgumentException.class, () -> LimelightCalibration.parse(missingCoefficients, 1280, 960, true));
        data = new JSONObject(CAMERA_CALIBRATION).getJSONObject("CALIBRATION_RESULT");
        data.put("RES_X", 1280.5);
        String fractionalSize = data.toString();
        assertThrows(IllegalArgumentException.class, () -> LimelightCalibration.parse(fractionalSize, 1280, 960, false));
    }
}
