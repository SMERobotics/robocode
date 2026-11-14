package org.technodot.ftc.twentysix.tests;

import org.junit.Test;
import java.util.ArrayList;
import java.util.List;
import java.io.File;
import java.nio.file.Files;
import static org.junit.Assert.*;

public class PinpointRotatingOffsetEstimatorTest {
    private PinpointOffsetEstimatorTest.Simulation simulation() {
        PinpointOffsetEstimatorTest.Simulation s = new PinpointOffsetEstimatorTest.Simulation(-26, -161, 13.26291192);
        s.rotatingExcitation = true;
        s.turningScaleChanges = true;
        return s;
    }

    @Test public void learnsWheelScalesUnderEachTurningLoadWithoutTransferringStraightScales() {
        PinpointOffsetEstimator.Result r = PinpointRotatingOffsetEstimator.estimate(simulation().generate(), true);
        assertTrue(r.issues.toString(), r.validated());
        assertTrue(r.rotatingCalibration);
        assertEquals(-26, r.xMm, .3);
        assertEquals(-161, r.yMm, .3);
        for (int n : r.acceptedTurns) assertEquals(3, n);
    }

    @Test public void resistsWorldDriftPodSlipAndIntermittentMotorOutliers() {
        PinpointOffsetEstimatorTest.Simulation sim = simulation();
        sim.unmeasuredDrift = true; sim.outliers = true;
        PinpointOffsetEstimator.Result r = PinpointRotatingOffsetEstimator.estimate(sim.generate(), true);
        assertEquals(-26, r.xMm, 1);
        assertEquals(-161, r.yMm, 1);
        assertTrue(r.inlierFraction > .8 && r.inlierFraction < .99);
    }

    @Test public void doesNotHideDirectionDependentOffsetsInSeparateWheelModels() {
        PinpointOffsetEstimatorTest.Simulation sim = simulation(); sim.directionBias = true;
        PinpointOffsetEstimator.Result r = PinpointRotatingOffsetEstimator.estimate(sim.generate(), true);
        assertFalse(r.validated()); assertTrue(r.directionGapMm > 5);
    }

    @Test public void stillRejectsSpeedDependentOffsets() {
        PinpointOffsetEstimatorTest.Simulation sim = simulation(); sim.speedBias = true;
        PinpointOffsetEstimator.Result r = PinpointRotatingOffsetEstimator.estimate(sim.generate(), true);
        assertFalse(r.validated()); assertTrue(r.speedGapMm > 5);
    }

    @Test public void phaseBiasCannotCancelOutOfBothDirectionAndSpeedAverages() {
        List<PinpointOffsetEstimator.Sample> altered = new ArrayList<>();
        for (PinpointOffsetEstimator.Sample s : simulation().generate()) {
            double bias = s.run >= 0 && s.run < 4 ? (s.run % 2 == 0 ? 6 : -6) : 0;
            altered.add(new PinpointOffsetEstimator.Sample(s.ticks, s.podForwardMm - bias * s.heading,
                    s.podLeftMm, s.heading, s.seconds, s.cosSeconds, s.sinSeconds, s.run, s.turn));
        }
        PinpointOffsetEstimator.Result r = PinpointRotatingOffsetEstimator.estimate(altered, true);
        assertTrue(r.directionGapMm < 2 && r.speedGapMm < 2 && r.validationGapMm < 2);
        assertFalse(r.validated());
        assertTrue(r.issues.toString(), r.issues.stream().anyMatch(i -> i.contains("Independent turning phases")));
    }

    @Test public void heldOutMeasurementsCannotPullTheTrainingEstimate() {
        List<PinpointOffsetEstimator.Sample> original = simulation().generate(), altered = new ArrayList<>();
        PinpointOffsetEstimator.Result before = PinpointRotatingOffsetEstimator.estimate(original, false);
        for (PinpointOffsetEstimator.Sample s : original) altered.add(new PinpointOffsetEstimator.Sample(s.ticks,
                s.podForwardMm - (s.run >= 4 ? 10 * s.heading : 0), s.podLeftMm, s.heading,
                s.seconds, s.cosSeconds, s.sinSeconds, s.run, s.turn));
        PinpointOffsetEstimator.Result after = PinpointRotatingOffsetEstimator.estimate(altered, true);
        assertEquals(before.xMm, after.xMm, 0); assertEquals(before.yMm, after.yMm, 0);
        assertFalse(after.validated()); assertTrue(after.validationGapMm > 8);
    }

    @Test public void heldOutWheelYawFaultCannotBeHiddenByItsOwnWheelModel() {
        List<PinpointOffsetEstimator.Sample> altered = new ArrayList<>();
        for (PinpointOffsetEstimator.Sample s : simulation().generate()) {
            double[] t = s.ticks.clone();
            if (s.run >= 4) { t[1] += 8; t[2] -= 8; t[3] += 8; }
            altered.add(new PinpointOffsetEstimator.Sample(t, s.podForwardMm, s.podLeftMm,
                    s.heading, s.seconds, s.cosSeconds, s.sinSeconds, s.run, s.turn));
        }
        PinpointOffsetEstimator.Result r = PinpointRotatingOffsetEstimator.estimate(altered, true);
        assertFalse(r.validated());
        assertTrue(r.issues.toString(), r.issues.stream().anyMatch(i -> i.contains("Held-out slip")));
    }

    @Test public void savedProtocolIdentifiesTheTurningEstimatorAndPreservesTheResult() throws Exception {
        List<PinpointOffsetEstimator.Sample> original = simulation().generate();
        File csv = File.createTempFile("pinpoint-turning-", ".csv");
        try {
            PinpointOffsetCapture.writeSamples(csv, original, true);
            PinpointOffsetCapture.Capture loaded = PinpointOffsetCapture.read(csv);
            assertTrue(loaded.rotatingCalibration);
            PinpointOffsetEstimator.Result a = PinpointRotatingOffsetEstimator.estimate(original, true);
            PinpointOffsetEstimator.Result b = PinpointRotatingOffsetEstimator.estimate(loaded.samples, true);
            assertEquals(a.xMm, b.xMm, 0); assertEquals(a.yMm, b.yMm, 0);
            assertTrue(b.issues.toString(), b.validated());
        } finally { Files.deleteIfExists(csv.toPath()); }
    }

    @Test public void uncertaintyIncludesRelearningWheelScalesAfterOmittingWholeTurns() {
        PinpointOffsetEstimatorTest.Simulation sim = simulation(); sim.turnBias = true;
        PinpointOffsetEstimator.Result r = PinpointRotatingOffsetEstimator.estimate(sim.generate(), true);
        assertFalse(r.validated()); assertTrue(r.xIntervalMm > 1);
    }

    @Test public void refusesPureYawWithoutIndependentTranslation() {
        PinpointOffsetEstimatorTest.Simulation sim = simulation(); sim.pureRotation = true;
        assertThrows(IllegalArgumentException.class, () -> PinpointRotatingOffsetEstimator.estimate(sim.generate(), true));
    }

    @Test public void frontLeftRemainsIrrelevantAndEncoderPolarityIsStillLearned() {
        List<PinpointOffsetEstimator.Sample> original = simulation().generate(), altered = new ArrayList<>();
        for (PinpointOffsetEstimator.Sample s : original) {
            double[] t = s.ticks.clone(); t[0] = Double.NaN; t[1] = -t[1]; t[3] = -t[3];
            altered.add(new PinpointOffsetEstimator.Sample(t, s.podForwardMm, s.podLeftMm,
                    s.heading, s.seconds, s.cosSeconds, s.sinSeconds, s.run, s.turn));
        }
        PinpointOffsetEstimator.Result r = PinpointRotatingOffsetEstimator.estimate(altered, true);
        assertEquals(-26, r.xMm, .3); assertEquals(-161, r.yMm, .3);
        assertTrue(r.issues.toString(), r.validated());
    }

    @Test public void actualPositionAndTurnControllersProduceIdentifiableDataWithIncorrectSeeds() {
        PinpointOffsetEstimatorTest.Simulation s = simulation();
        s.translationSet(0);
        PinpointPositionHold hold = new PinpointPositionHold(s.x + 45, s.y - 20);
        double rate = 0, vx = 0, vy = 0, x = 0, y = 0;
        int turnId = 0;
        int[] rates = {30, -30, -60, 60, 45, -45};
        for (int run = 0; run < 6; run++) {
            PinpointTurnController turn = new PinpointTurnController(Math.toRadians(rates[run]), .7);
            int turns = 0;
            double time = 0, turnStart = s.worldHeading;
            boolean collecting = false;
            while (turns < (run < 4 ? 3 : 1)) {
                double dt = .02;
                turn.update(rate, dt);
                PinpointCalibrationTrajectory target = new PinpointCalibrationTrajectory(time);
                hold.controlTarget(s.worldHeading, dt, .7 - Math.abs(turn.power()), target.x, target.y, target.vx, target.vy);
                double fPower = hold.forwardPower(), lPower = hold.leftPower(), rPower = turn.power();
                for (double motor : new double[]{fPower - lPower - rPower, fPower + lPower + rPower,
                        fPower + lPower - rPower, fPower - lPower + rPower}) assertTrue(Math.abs(motor) <= .7 + 1e-12);
                double desiredRate = Math.signum(rPower) * Math.max(0, Math.abs(rPower) - .135) * Math.toRadians(400);
                rate += dt / (.12 + dt) * (desiredRate - rate);
                double angle = rate * dt, mid = s.worldHeading + angle / 2;
                double cos = Math.cos(mid), sin = Math.sin(mid);
                vx += dt / (.12 + dt) * (cos * 420 * fPower - sin * 340 * lPower + 20 - vx);
                vy += dt / (.12 + dt) * (sin * 420 * fPower + cos * 340 * lPower - 9 - vy);
                double dx = vx * dt, dy = vy * dt;
                x += dx; y += dy;
                int oldX = s.previous[4], oldY = s.previous[5];
                s.add(cos * dx + sin * dy, -sin * dx + cos * dy, angle, dt, run, turnId, 0);
                hold.observe((s.previous[4] - oldX) / s.ticksPerMm, (s.previous[5] - oldY) / s.ticksPerMm, angle, mid, dt);
                if (!collecting) {
                    s.bin.clear();
                    if (turn.settled()) { collecting = true; turnStart = s.worldHeading; }
                    assertFalse("Rate settling failed", turn.timedOut());
                } else {
                    if (Math.abs(s.bin.heading) >= Math.toRadians(5)) s.samples.add(s.bin.take(run, turnId));
                    if ((s.worldHeading - turnStart) * Math.signum(rates[run]) >= 2 * Math.PI) {
                        if (s.bin.seconds > 0) s.samples.add(s.bin.take(run, turnId));
                        turns++; turnId++; turnStart = s.worldHeading;
                    }
                }
                time += dt;
                assertTrue("Run did not finish", time < 60);
                assertTrue("Physical travel " + Math.hypot(x,y), Math.hypot(x,y) < 500);
                assertTrue(hold.distanceMm() < 500);
            }
            hold.pause();
            for (int n = 0; n < 50; n++) {
                rate *= .85; vx *= .85; vy *= .85;
                double angle = rate * .02, mid = s.worldHeading + angle / 2;
                double cos = Math.cos(mid), sin = Math.sin(mid), dx = vx * .02, dy = vy * .02;
                int oldX = s.previous[4], oldY = s.previous[5];
                s.add(cos * dx + sin * dy, -sin * dx + cos * dy, angle, .02, run, -1, 0);
                hold.observe((s.previous[4] - oldX) / s.ticksPerMm, (s.previous[5] - oldY) / s.ticksPerMm, angle, mid, .02);
                s.bin.clear(); x += dx; y += dy;
            }
        }
        PinpointOffsetEstimator.Result r = PinpointRotatingOffsetEstimator.estimate(s.samples, true);
        assertTrue(r.issues.toString(), r.validated());
        assertEquals(s.x, r.xMm, .5); assertEquals(s.y, r.yMm, .5);
    }
}
