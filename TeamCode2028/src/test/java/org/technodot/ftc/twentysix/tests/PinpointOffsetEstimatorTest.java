package org.technodot.ftc.twentysix.tests;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class PinpointOffsetEstimatorTest {
    private static final double[] WHEEL_MM = {0.460, 0.452, 0.444, 0.457};

    @Test
    public void recoversOffsetsWithEllipticalNonclosingTurnsAndUnequalWheels() {
        Simulation sim = new Simulation(-26, -161, 13.26291192);
        List<PinpointOffsetEstimator.Sample> samples = sim.generate();
        PinpointOffsetEstimator.Result result = PinpointOffsetEstimator.estimate(samples, true);
        assertEquals(sim.x, result.xMm, 0.3);
        assertEquals(sim.y, result.yMm, 0.3);
        assertTrue(result.issues.toString(), result.validated());
        for (int turns : result.acceptedTurns) assertEquals(3, turns);
        double mean = 0;
        for (int i = 1; i < 4; i++) mean += WHEEL_MM[i] / 3;
        assertEquals(0, result.wheelScales[0], 0);
        for (int i = 1; i < 4; i++) assertEquals(WHEEL_MM[i] / mean, result.wheelScales[i], 0.003);

        // A raw-distance/full-angle estimator is biased by the elliptical path,
        // even when it repeats perfectly. Our synthetic slow turn also ends
        // 24 mm forward and 12 mm sideways from where it started.
        double rawLeft = 0, angle = 0;
        for (PinpointOffsetEstimator.Sample s : samples) {
            if (s.run == 0) { rawLeft += s.podLeftMm; angle += s.heading; }
        }
        assertTrue(Math.abs(rawLeft / angle - sim.y) > 15);
    }

    @Test
    public void supportsOtherSignsAndPodResolution() {
        Simulation sim = new Simulation(92, 37, 19.89436789);
        PinpointOffsetEstimator.Result result = PinpointOffsetEstimator.estimate(sim.generate(), true);
        assertEquals(sim.x, result.xMm, 0.3);
        assertEquals(sim.y, result.yMm, 0.3);
        assertTrue(result.issues.toString(), result.validated());
    }

    @Test
    public void correctsWorldDriftMissingFromDriveEncoders() {
        Simulation sim = new Simulation(-26, -161, 13.26291192);
        sim.unmeasuredDrift = true;
        PinpointOffsetEstimator.Result result = PinpointOffsetEstimator.estimate(sim.generate(), true);
        assertEquals(sim.x, result.xMm, 0.4);
        assertEquals(sim.y, result.yMm, 0.4);
        assertTrue(result.issues.toString(), result.validated());
    }

    @Test
    public void resistsIntermittentPodSlipAndMotorOutliers() {
        Simulation sim = new Simulation(-26, -161, 13.26291192);
        sim.outliers = true;
        PinpointOffsetEstimator.Result result = PinpointOffsetEstimator.estimate(sim.generate(), true);
        assertEquals(sim.x, result.xMm, 1.0);
        assertEquals(sim.y, result.yMm, 1.0);
        assertTrue("Should reject slip intervals", result.inlierFraction < 0.99);
        assertTrue("Most measurements should remain useful", result.inlierFraction > 0.8);
    }

    @Test
    public void detectsDirectionDependentBias() {
        Simulation sim = new Simulation(-26, -161, 13.26291192);
        sim.directionBias = true;
        PinpointOffsetEstimator.Result result = PinpointOffsetEstimator.estimate(sim.generate(), true);
        assertFalse(result.validated());
        assertTrue(result.directionGapMm > 5);
        assertTrue(result.issues.toString(), result.issues.stream().anyMatch(s -> s.contains("CW/CCW")));
    }

    @Test
    public void detectsSpeedDependentBias() {
        Simulation sim = new Simulation(-26, -161, 13.26291192);
        sim.speedBias = true;
        PinpointOffsetEstimator.Result result = PinpointOffsetEstimator.estimate(sim.generate(), true);
        assertFalse(result.validated());
        assertTrue(result.speedGapMm > 5);
        assertTrue(result.issues.toString(), result.issues.stream().anyMatch(s -> s.contains("Slow/fast")));
    }

    @Test
    public void heldOutDataCannotPullTrainingEstimateTowardItself() {
        Simulation sim = new Simulation(-26, -161, 13.26291192);
        List<PinpointOffsetEstimator.Sample> samples = sim.generate();
        PinpointOffsetEstimator.Result before = PinpointOffsetEstimator.estimate(samples, false);
        List<PinpointOffsetEstimator.Sample> altered = new ArrayList<>();
        for (PinpointOffsetEstimator.Sample s : samples) {
            altered.add(new PinpointOffsetEstimator.Sample(s.ticks,
                    s.podForwardMm - (s.run >= 4 ? 10 * s.heading : 0), s.podLeftMm,
                    s.heading, s.seconds, s.cosSeconds, s.sinSeconds, s.run, s.turn));
        }
        PinpointOffsetEstimator.Result result = PinpointOffsetEstimator.estimate(altered, true);
        assertEquals(before.xMm, result.xMm, 1e-10);
        assertEquals(before.yMm, result.yMm, 1e-10);
        assertTrue(result.validationGapMm > 8);
        assertFalse(result.validated());
    }

    @Test
    public void rejectsHeldOutWheelInconsistencyEvenWithoutApparentTranslation() {
        Simulation sim = new Simulation(-26, -161, 13.26291192);
        List<PinpointOffsetEstimator.Sample> altered = new ArrayList<>();
        for (PinpointOffsetEstimator.Sample s : sim.generate()) {
            double[] ticks = s.ticks.clone();
            if (s.run >= 4) {
                // FR/BR increasing together and BL decreasing is pure wheel yaw:
                // forward/left translation cancels, but the IMU did not rotate.
                ticks[1] += 8; ticks[2] -= 8; ticks[3] += 8;
            }
            altered.add(new PinpointOffsetEstimator.Sample(ticks, s.podForwardMm, s.podLeftMm,
                    s.heading, s.seconds, s.cosSeconds, s.sinSeconds, s.run, s.turn));
        }
        PinpointOffsetEstimator.Result result = PinpointOffsetEstimator.estimate(altered, true);
        assertFalse(result.validated());
        assertTrue(result.issues.toString(), result.issues.stream().anyMatch(s -> s.contains("held-out slip")));
    }

    @Test
    public void uncertaintyIncludesCorrelatedTurnErrors() {
        Simulation clean = new Simulation(-26, -161, 13.26291192);
        PinpointOffsetEstimator.Result baseline = PinpointOffsetEstimator.estimate(clean.generate(), true);
        Simulation noisy = new Simulation(-26, -161, 13.26291192);
        noisy.turnBias = true;
        PinpointOffsetEstimator.Result result = PinpointOffsetEstimator.estimate(noisy.generate(), true);
        assertTrue(result.xIntervalMm > baseline.xIntervalMm + 1);
        assertFalse(result.validated());
    }

    @Test
    public void requiresTranslationExcitationAndBothRotationDirections() {
        Simulation sim = new Simulation(-26, -161, 13.26291192);
        List<PinpointOffsetEstimator.Sample> samples = sim.generate();
        List<PinpointOffsetEstimator.Sample> rotations = new ArrayList<>(), oneRun = new ArrayList<>();
        for (PinpointOffsetEstimator.Sample s : samples) {
            if (s.run >= 0) rotations.add(s);
            if (s.run < 0 || s.run == 0) oneRun.add(s);
        }
        assertThrows(IllegalArgumentException.class, () -> PinpointOffsetEstimator.estimate(rotations, false));
        PinpointOffsetEstimator.Result result = PinpointOffsetEstimator.estimate(oneRun, true);
        assertFalse(result.validated());
        assertEquals(0, result.acceptedTurns[1]);
    }

    @Test
    public void handlesCounterRolloverAndHeadingWrap() {
        assertEquals(9, PinpointOffsetEstimator.counterDelta(Integer.MIN_VALUE + 4, Integer.MAX_VALUE - 4));
        assertEquals(-9, PinpointOffsetEstimator.counterDelta(Integer.MAX_VALUE - 4, Integer.MIN_VALUE + 4));
        assertEquals(Math.toRadians(2), PinpointOffsetEstimator.headingDelta(Math.toRadians(-179), Math.toRadians(179)), 1e-12);
        assertEquals(Math.toRadians(-2), PinpointOffsetEstimator.headingDelta(Math.toRadians(179), Math.toRadians(-179)), 1e-12);
        assertEquals(0.02, PinpointOffsetEstimator.headingDelta(20 * Math.PI + .02, 20 * Math.PI), 1e-12);
        Simulation sim = new Simulation(-26, -161, 13.26291192);
        sim.counterStart = Integer.MAX_VALUE - 100;
        PinpointOffsetEstimator.Result result = PinpointOffsetEstimator.estimate(sim.generate(), false);
        assertEquals(sim.x, result.xMm, .3);
        assertEquals(sim.y, result.yMm, .3);
    }

    @Test
    public void learnsEncoderCountingPolarityWithoutChangingRobotCoordinates() {
        Simulation sim = new Simulation(-26, -161, 13.26291192);
        List<PinpointOffsetEstimator.Sample> original = sim.generate();
        // Cover all counting polarities of the three working encoders. Motor
        // commands and pod/IMU readings remain unchanged.
        for (int mask = 0; mask < 8; mask++) {
            List<PinpointOffsetEstimator.Sample> altered = new ArrayList<>();
            for (PinpointOffsetEstimator.Sample s : original) {
                double[] ticks = s.ticks.clone();
                for (int i = 1; i < 4; i++) if ((mask & (1 << (i - 1))) != 0) ticks[i] = -ticks[i];
                altered.add(new PinpointOffsetEstimator.Sample(ticks, s.podForwardMm, s.podLeftMm,
                        s.heading, s.seconds, s.cosSeconds, s.sinSeconds, s.run, s.turn));
            }
            PinpointOffsetEstimator.Result result = PinpointOffsetEstimator.estimate(altered, false);
            assertEquals("Polarity mask " + mask, sim.x, result.xMm, .3);
            assertEquals("Polarity mask " + mask, sim.y, result.yMm, .3);
            for (int i = 1; i < 4; i++) {
                assertEquals((mask & (1 << (i - 1))) == 0 ? 1 : -1, (int) Math.signum(result.wheelScales[i]));
            }
        }
    }

    @Test
    public void identifiesIndividualMissingEncodersInsteadOfBlamingWheelTraction() {
        String[] names = {"FL", "FR", "BL", "BR"};
        Simulation sim = new Simulation(-26, -161, 13.26291192);
        List<PinpointOffsetEstimator.Sample> original = sim.generate();
        for (int wheel = 1; wheel < 4; wheel++) {
            List<PinpointOffsetEstimator.Sample> altered = new ArrayList<>();
            for (PinpointOffsetEstimator.Sample s : original) {
                double[] ticks = s.ticks.clone();
                ticks[wheel] = 0;
                altered.add(new PinpointOffsetEstimator.Sample(ticks, s.podForwardMm, s.podLeftMm,
                        s.heading, s.seconds, s.cosSeconds, s.sinSeconds, s.run, s.turn));
            }
            IllegalArgumentException failure = assertThrows(IllegalArgumentException.class,
                    () -> PinpointOffsetEstimator.estimate(altered, false));
            assertTrue(failure.getMessage(), failure.getMessage().contains(names[wheel] + " encoder"));
        }
    }

    @Test
    public void ignoresDeadWildAndNonfiniteFrontLeftCountsThroughoutCalibration() {
        Simulation sim = new Simulation(-26, -161, 13.26291192);
        List<PinpointOffsetEstimator.Sample> clean = sim.generate();
        List<PinpointOffsetEstimator.Sample> corrupted = new ArrayList<>();
        double[] garbage = {0, 1e30, -1e30, Double.NaN, Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY, Integer.MAX_VALUE, Integer.MIN_VALUE};
        int index = 0;
        for (PinpointOffsetEstimator.Sample s : clean) {
            double[] ticks = s.ticks.clone();
            ticks[0] = garbage[index++ % garbage.length];
            PinpointOffsetEstimator.Sample ignored = new PinpointOffsetEstimator.Sample(ticks,
                    s.podForwardMm, s.podLeftMm, s.heading, s.seconds, s.cosSeconds, s.sinSeconds, s.run, s.turn);
            assertEquals(0, ignored.ticks[0], 0);
            corrupted.add(ignored);
        }
        PinpointOffsetEstimator.Result baseline = PinpointOffsetEstimator.estimate(clean, true);
        PinpointOffsetEstimator.Result result = PinpointOffsetEstimator.estimate(corrupted, true);
        assertTrue(result.issues.toString(), result.validated());
        assertEquals(baseline.xMm, result.xMm, 0);
        assertEquals(baseline.yMm, result.yMm, 0);
        assertEquals(baseline.xIntervalMm, result.xIntervalMm, 0);
        assertEquals(baseline.yIntervalMm, result.yIntervalMm, 0);
        assertArrayEquals(baseline.wheelScales, result.wheelScales, 0);
        assertArrayEquals(baseline.acceptedTurns, result.acceptedTurns);
    }

    @Test
    public void frontLeftCannotAffectSamplingOrEncoderHealthChecks() {
        PinpointOffsetEstimator.Accumulator accumulator = new PinpointOffsetEstimator.Accumulator();
        accumulator.add(new double[]{Double.NaN, 40, 39, 41}, 20, 0, 0, .1, 0);
        PinpointOffsetEstimator.Sample sample = accumulator.take(-1, -1);
        assertEquals(0, sample.ticks[0], 0);
        assertArrayEquals(new int[]{0, 1, 1, 1},
                PinpointOffsetEstimator.encoderPolarities(new double[]{Double.NaN, 400, 390, 410}, 0));
        PinpointOffsetEstimator.checkEncoderPolarities(new int[]{-1, 1, 1, 1}, new int[]{1, 1, 1, 1});
        assertFalse(PinpointOffsetEstimator.formatCounts(new double[]{Double.NaN, 1, 2, 3}).contains("NaN"));
    }

    @Test
    public void rejectsEncoderPolarityThatChangesBetweenMovementDirections() {
        Simulation sim = new Simulation(-26, -161, 13.26291192);
        List<PinpointOffsetEstimator.Sample> altered = new ArrayList<>();
        for (PinpointOffsetEstimator.Sample s : sim.generate()) {
            double[] ticks = s.ticks.clone();
            if (s.run == -3) ticks[1] = -ticks[1];
            altered.add(new PinpointOffsetEstimator.Sample(ticks, s.podForwardMm, s.podLeftMm,
                    s.heading, s.seconds, s.cosSeconds, s.sinSeconds, s.run, s.turn));
        }
        IllegalArgumentException failure = assertThrows(IllegalArgumentException.class,
                () -> PinpointOffsetEstimator.estimate(altered, false));
        assertTrue(failure.getMessage(), failure.getMessage().contains("FR encoder response changes"));
    }

    @Test
    public void repeatingOneSpinDirectionCannotDominateTheWheelScaleCalibration() {
        Simulation sim = new Simulation(-26, -161, 13.26291192);
        List<PinpointOffsetEstimator.Sample> once = new ArrayList<>(), repeated = new ArrayList<>();
        for (PinpointOffsetEstimator.Sample s : sim.generate()) {
            if (s.run > 0) continue;
            once.add(s);
            repeated.add(s);
        }
        for (int repeat = 1; repeat <= 2; repeat++) {
            for (PinpointOffsetEstimator.Sample s : once) {
                if (s.run != 0) continue;
                repeated.add(new PinpointOffsetEstimator.Sample(s.ticks, s.podForwardMm, s.podLeftMm,
                        s.heading, s.seconds, s.cosSeconds, s.sinSeconds, s.run, s.turn + repeat * 100));
            }
        }
        PinpointOffsetEstimator.Result first = PinpointOffsetEstimator.estimate(once, false);
        PinpointOffsetEstimator.Result many = PinpointOffsetEstimator.estimate(repeated, false);
        assertArrayEquals(first.wheelScales, many.wheelScales, 1e-9);
    }

    @Test
    public void collectsAllDirectionsAndSpeedsBeforeJudgingIncompleteFits() {
        assertEquals(1, PinpointOffsetEstimator.nextTrainingRun(new int[]{3, 0, 0, 0}, null));
        assertEquals(2, PinpointOffsetEstimator.nextTrainingRun(new int[]{3, 3, 0, 0}, new int[]{0, 0, 0, 0}));
        assertEquals(3, PinpointOffsetEstimator.nextTrainingRun(new int[]{3, 3, 3, 0}, null));
        assertEquals(4, PinpointOffsetEstimator.nextTrainingRun(new int[]{3, 3, 3, 3}, new int[]{3, 3, 3, 3}));
        assertEquals(-1, PinpointOffsetEstimator.nextTrainingRun(new int[]{3, 3, 3, 3}, null));
    }

    @Test
    public void balancesRetriesAndStopsWhenAnUnreliableRunExhaustsItsAttempts() {
        assertEquals(0, PinpointOffsetEstimator.nextTrainingRun(new int[]{3, 3, 3, 3}, new int[]{2, 2, 3, 3}));
        assertEquals(1, PinpointOffsetEstimator.nextTrainingRun(new int[]{4, 3, 3, 3}, new int[]{2, 2, 3, 3}));
        assertEquals(-1, PinpointOffsetEstimator.nextTrainingRun(new int[]{5, 3, 3, 3}, new int[]{2, 3, 3, 3}));
    }

    @Test
    public void rejectsNonfiniteOrDegenerateData() {
        assertThrows(IllegalArgumentException.class, () -> new PinpointOffsetEstimator.Sample(
                new double[4], Double.NaN, 0, 0, .1, .1, 0, 0, 0));
        List<PinpointOffsetEstimator.Sample> still = new ArrayList<>();
        for (int run = -4; run <= 0; run++) still.add(new PinpointOffsetEstimator.Sample(
                new double[4], 0, 0, 0, .1, .1, 0, run, 0));
        assertThrows(IllegalArgumentException.class, () -> PinpointOffsetEstimator.estimate(still, false));
    }

    @Test
    public void repeatabilityCannotCertifyUnobservableSystematicSlip() {
        // A body-frame velocity proportional to yaw is indistinguishable from
        // an offset change when drive encoders miss it consistently. Document
        // that limit rather than asserting that statistics can detect it.
        Simulation sim = new Simulation(-26, -161, 13.26291192);
        sim.systematicSlip = true;
        PinpointOffsetEstimator.Result result = PinpointOffsetEstimator.estimate(sim.generate(), true);
        assertEquals(sim.x - 6, result.xMm, .3);
        assertTrue(result.issues.toString(), result.validated());
    }

    @Test
    public void positionFeedbackWithIncorrectSeedsDoesNotBiasTheFittedOffsets() {
        Simulation sim = new Simulation(-26, -161, 13.26291192);
        sim.positionHold = new PinpointPositionHold(sim.x + 75, sim.y - 45);
        PinpointOffsetEstimator.Result result = PinpointOffsetEstimator.estimate(sim.generate(), true);
        assertTrue("Held travel " + sim.maxHeldDistance, sim.maxHeldDistance < 250);
        assertEquals(sim.x, result.xMm, .3);
        assertEquals(sim.y, result.yMm, .3);
        assertTrue(result.issues.toString(), result.validated());
    }

    private static final class Simulation {
        final double x, y, ticksPerMm;
        final double[] counters = new double[6];
        final int[] previous = new int[6];
        final List<PinpointOffsetEstimator.Sample> samples = new ArrayList<>();
        final PinpointOffsetEstimator.Accumulator bin = new PinpointOffsetEstimator.Accumulator();
        final Random random = new Random(226);
        boolean unmeasuredDrift, outliers, directionBias, speedBias, turnBias, systematicSlip;
        int counterStart, sampleNumber;
        double worldHeading;
        PinpointPositionHold positionHold;
        double maxHeldDistance;

        Simulation(double x, double y, double ticksPerMm) {
            this.x = x;
            this.y = y;
            this.ticksPerMm = ticksPerMm;
        }

        List<PinpointOffsetEstimator.Sample> generate() {
            for (int i = 0; i < 6; i++) { counters[i] = counterStart; previous[i] = counterStart; }
            for (int leg = 0; leg < 4; leg++) {
                for (int n = 0; n < 40; n++) {
                    double sign = leg % 2 == 0 ? 1 : -1;
                    double angle = .0002 * Math.sin(n);
                    add(leg < 2 ? sign * 5 : 0, leg >= 2 ? sign * 5 : 0,
                            angle, .05, -1 - leg, -1, 0);
                    if ((n + 1) % 4 == 0) samples.add(bin.take(-1 - leg, -1));
                }
            }
            int id = 0;
            int[] rates = {30, -30, -60, 60, 45, -45};
            for (int run = 0; run < 6; run++) {
                double sign = Math.signum(rates[run]);
                for (int turn = 0; turn < (run < 4 ? 3 : 1); turn++, id++) {
                    double bias = turnBias && run < 4 ? (turn - 1) * 6 : 0;
                    for (int n = 0; n < 360; n++) {
                        double a = worldHeading, dh = sign * Math.toRadians(1), b = a + dh;
                        double mid = (a + b) / 2;
                        double rate = Math.toRadians(Math.abs(rates[run])) * (1 + .07 * Math.sin(2 * mid));
                        double dt = Math.abs(dh) / rate;
                        double dx = 28 * (Math.cos(b) - Math.cos(a)) + (positionHold == null ? 2 : 20) * dt;
                        double dy = 14 * (Math.sin(b) - Math.sin(a)) - dt;
                        double f = Math.cos(mid) * dx + Math.sin(mid) * dy;
                        double l = -Math.sin(mid) * dx + Math.cos(mid) * dy;
                        if (positionHold != null) {
                            positionHold.control(worldHeading, dt, .35);
                            f += 420 * positionHold.forwardPower() * dt;
                            l += 340 * positionHold.leftPower() * dt;
                        }
                        add(f, l, dh, dt, run, id, bias);
                        if ((n + 1) % 5 == 0) samples.add(bin.take(run, id));
                    }
                }
            }
            return samples;
        }

        void add(double f, double l, double dh, double dt, int run, int turn, double turnOffset) {
            double mid = worldHeading + dh / 2;
            // The 1.12 lateral scale models the different effective travel of
            // mecanum rollers during sideways movement, without wheel geometry.
            double[] travel = {f - l / 1.12 - 250 * dh, f + l / 1.12 + 250 * dh,
                    f + l / 1.12 - 250 * dh, f - l / 1.12 + 250 * dh};
            double observedX = x + turnOffset + (directionBias && (run == 1 || run == 2) ? 8 : 0)
                    + (speedBias && (run == 2 || run == 3) ? 8 : 0);
            double qx = f - observedX * dh, qy = l + y * dh;
            if (unmeasuredDrift && run >= 0) {
                qx += (1.2 * Math.cos(mid) - .6 * Math.sin(mid)) * dt;
                qy += (-1.2 * Math.sin(mid) - .6 * Math.cos(mid)) * dt;
            }
            if (systematicSlip && run >= 0) qx += 6 * dh;
            if (outliers && run >= 0 && sampleNumber % 173 == 0) {
                qx += random.nextBoolean() ? 5 : -5;
                qy += random.nextBoolean() ? 5 : -5;
            }
            double[] ticks = new double[4];
            double podX = 0, podY = 0;
            for (int i = 0; i < 6; i++) {
                double distance = i < 4 ? travel[i] : i == 4 ? qx : qy;
                if (outliers && run >= 0 && sampleNumber % 239 == 0 && i == 1) distance += 3;
                counters[i] += i < 4 ? distance / WHEEL_MM[i] : distance * ticksPerMm;
                int count = (int) Math.round(counters[i]);
                int delta = PinpointOffsetEstimator.counterDelta(count, previous[i]);
                previous[i] = count;
                if (i < 4) ticks[i] = delta;
                else if (i == 4) podX = delta / ticksPerMm;
                else podY = delta / ticksPerMm;
            }
            bin.add(ticks, podX, podY, dh, dt, mid);
            if (positionHold != null) {
                positionHold.observe(podX, podY, dh, mid, dt);
                maxHeldDistance = Math.max(maxHeldDistance, positionHold.distanceMm());
            }
            worldHeading += dh;
            sampleNumber++;
        }
    }
}
