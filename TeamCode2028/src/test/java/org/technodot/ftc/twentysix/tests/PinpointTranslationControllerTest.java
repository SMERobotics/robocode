package org.technodot.ftc.twentysix.tests;

import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class PinpointTranslationControllerTest {
    @Test
    public void finishesLegThatOldPowerTaperLeavesShortOfTarget() {
        Plant old = new Plant(.17, .11, 900);
        for (double t = 0; t < 8; t += .02) {
            double power = Math.min(.18, .3 * t) * Math.max(.4, Math.min(1, (200 - old.position) / 60));
            old.advance(power, .02);
        }
        assertTrue("Old controller should reproduce stopping near the end", old.position > 150 && old.position < 195);
        assertTrue("Old controller should have stopped", old.velocity < 1);

        PinpointTranslationController controller = new PinpointTranslationController(200, .35);
        Plant fixed = new Plant(.17, .11, 900);
        run(controller, fixed, false);
        assertEquals(PinpointTranslationController.Status.COMPLETE, controller.status());
        assertTrue(fixed.position >= 197.9 && fixed.position < 205);
        assertEquals(0, controller.power(), 0);
    }

    @Test
    public void overcomesDifferentForwardAndSidewaysBreakawayFriction() {
        for (double[] friction : new double[][]{{.17, .11}, {.28, .22}, {.33, .29}}) {
            PinpointTranslationController controller = new PinpointTranslationController(200, .35);
            Plant plant = new Plant(friction[0], friction[1], 900);
            run(controller, plant, false);
            assertEquals("Friction " + friction[0], PinpointTranslationController.Status.COMPLETE, controller.status());
            assertTrue(controller.elapsedSeconds() < PinpointTranslationController.MAX_SECONDS);
            assertTrue(controller.peakPower() <= .35 + 1e-12);
            assertTrue("Must build enough power to start", controller.peakPower() >= friction[0]);
        }
    }

    @Test
    public void handlesQuantizedPositionAndVariableLoopTiming() {
        PinpointTranslationController controller = new PinpointTranslationController(200, .35);
        Plant plant = new Plant(.27, .20, 1100);
        run(controller, plant, true);
        assertEquals(PinpointTranslationController.Status.COMPLETE, controller.status());
        assertTrue(plant.position < 206);
    }

    @Test
    public void reportsJamSeparatelyFromWheelsTurningWithoutPodTravel() {
        PinpointTranslationController jam = new PinpointTranslationController(200, .35);
        PinpointTranslationController noPodTravel = new PinpointTranslationController(200, .35);
        for (int i = 0; i < 300; i++) {
            jam.update(0, 0, .02);
            noPodTravel.update(0, i * 10, .02);
        }
        assertEquals(PinpointTranslationController.Status.DRIVE_STALLED, jam.status());
        assertEquals(PinpointTranslationController.Status.WHEELS_WITHOUT_POD_TRAVEL, noPodTravel.status());
        assertTrue(jam.elapsedSeconds() > 2.5 && jam.elapsedSeconds() < 5);
        assertTrue(jam.peakPower() >= .33);
        assertEquals(0, jam.power(), 0);
        assertEquals(0, noPodTravel.power(), 0);
    }

    @Test
    public void slowContinuingProgressDoesNotCountAsAStall() {
        PinpointTranslationController controller = new PinpointTranslationController(200, .35);
        for (int i = 1; i <= 400; i++) controller.update(i * .2, i * .5, .02);
        assertEquals(PinpointTranslationController.Status.RUNNING, controller.status());
        assertEquals(80, controller.progressMm(), 1e-9);
    }

    @Test
    public void stillBoundsLegDurationWhenContinuingProgressIsTooSlow() {
        PinpointTranslationController controller = new PinpointTranslationController(200, .35);
        for (int i = 1; i <= 1000; i++) controller.update(i * .1, i * .25, .02);
        assertEquals(PinpointTranslationController.Status.TIMED_OUT, controller.status());
        assertTrue(controller.elapsedSeconds() >= PinpointTranslationController.MAX_SECONDS);
        assertEquals(0, controller.power(), 0);
    }

    @Test
    public void detectsReversedTravelAndMissingDriveEncoderFeedback() {
        PinpointTranslationController reversed = new PinpointTranslationController(200, .35);
        reversed.update(-26, 60, .02);
        assertEquals(PinpointTranslationController.Status.WRONG_DIRECTION, reversed.status());
        assertEquals(0, reversed.power(), 0);

        PinpointTranslationController noEncoder = new PinpointTranslationController(200, .35);
        for (int i = 1; i < 100; i++) noEncoder.update(i, 0, .02);
        assertEquals(PinpointTranslationController.Status.DRIVE_ENCODERS_NOT_TRACKING, noEncoder.status());
        assertEquals(0, noEncoder.power(), 0);
    }

    @Test
    public void completedOrFailedLegCannotRestartFromMoreFeedback() {
        PinpointTranslationController complete = new PinpointTranslationController(200, .35);
        complete.update(200, 400, .02);
        complete.update(0, 500, .02);
        assertEquals(PinpointTranslationController.Status.COMPLETE, complete.status());
        assertEquals(0, complete.power(), 0);
    }

    private static void run(PinpointTranslationController controller, Plant plant, boolean jitter) {
        Random random = new Random(226);
        for (int i = 0; i < 2000 && controller.status() == PinpointTranslationController.Status.RUNNING; i++) {
            double dt = jitter ? .01 + .03 * random.nextDouble() : .02;
            double measured = jitter ? Math.round(plant.position / .0754) * .0754 : plant.position;
            controller.update(measured, plant.position / .455, dt);
            assertTrue(controller.power() >= 0 && controller.power() <= .35 + 1e-12);
            plant.advance(controller.power(), dt);
        }
    }

    /** Simple motor/traction model with static friction, rolling friction, and inertia. */
    private static final class Plant {
        final double breakaway, rolling, gain;
        double position, velocity;

        Plant(double breakaway, double rolling, double gain) {
            this.breakaway = breakaway;
            this.rolling = rolling;
            this.gain = gain;
        }

        void advance(double power, double dt) {
            double target = velocity < .5 && power < breakaway ? 0 : Math.max(0, power - rolling) * gain;
            velocity += (1 - Math.exp(-dt / .08)) * (target - velocity);
            position += velocity * dt;
        }
    }
}
