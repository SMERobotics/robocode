package org.technodot.ftc.twentysix.tests;

import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class PinpointTurnControllerTest {
    @Test
    public void reachesFastRateWhereTheOldIntegralCapStallsAround44DegreesPerSecond() {
        for (int sign : new int[]{-1, 1}) {
            Plant old = new Plant(.18, .135, 400);
            double target = sign * Math.toRadians(60), command = 0, integral = 0;
            for (int n = 0; n < 400; n++) {
                command += clip(target - command, -Math.toRadians(60) * .02, Math.toRadians(60) * .02);
                double error = command - old.rate;
                integral = clip(integral + .12 * error * .02, -.12, .12);
                old.step(.08 * Math.signum(command) + .16 * error + integral, .02);
            }
            assertEquals(44, sign * Math.toDegrees(old.rate), 1);

            PinpointTurnController fixed = new PinpointTurnController(target, .7);
            Plant plant = new Plant(.18, .135, 400);
            double firstSettled = run(fixed, plant, 25);
            assertTrue("Must settle before timeout: " + firstSettled, firstSettled < PinpointTurnController.SETTLE_TIMEOUT_SECONDS);
            assertEquals(sign * 60, Math.toDegrees(plant.rate), .3);
            assertTrue("Must exceed the old artificial power ceiling", fixed.peakPower() > .26);
        }
    }

    @Test
    public void handlesBothDirectionsAndAllCalibrationSpeedsAcrossDifferentTraction() {
        for (double[] properties : new double[][]{{.17, .11, 900}, {.32, .25, 180}, {.5, .42, 240}}) {
            for (double target : new double[]{30, -30, -60, 60, 45, -45}) {
                Plant plant = new Plant(properties[0], properties[1], properties[2]);
                PinpointTurnController controller = new PinpointTurnController(Math.toRadians(target), .7);
                double settled = run(controller, plant, 30);
                assertTrue("Target " + target + ", rolling friction " + properties[1] + ": " + settled,
                        settled < PinpointTurnController.SETTLE_TIMEOUT_SECONDS);
                assertEquals(target, Math.toDegrees(plant.rate), .6);
                assertTrue(controller.peakPower() <= .7);
            }
        }
    }

    @Test
    public void settlesWithLoopJitterGyroNoiseAndUnevenRotation() {
        Random random = new Random(226);
        for (double target : new double[]{30, -60}) {
            Plant plant = new Plant(.21, .15, 350);
            PinpointTurnController controller = new PinpointTurnController(Math.toRadians(target), .7);
            double time = 0, settled = Double.POSITIVE_INFINITY;
            while (time < 25) {
                double dt = .01 + .03 * random.nextDouble();
                plant.rolling = .15 + .015 * Math.sin(plant.heading * 2);
                double measured = plant.rate + Math.toRadians(2 * random.nextGaussian() + 8 * Math.sin(40 * time));
                controller.update(measured, dt);
                plant.step(controller.power(), dt);
                time += dt;
                if (controller.settled()) settled = Math.min(settled, time);
                assertTrue(Math.abs(controller.power()) <= .7);
            }
            assertTrue("Settling time " + settled, settled < PinpointTurnController.SETTLE_TIMEOUT_SECONDS);
            assertEquals(target, Math.toDegrees(controller.filteredRate()), 4);
        }
    }

    @Test
    public void reportsAnUnreachableRateOnlyAfterActuallyUsingTheAllowedPower() {
        Plant plant = new Plant(.27, .23, 240);
        PinpointTurnController controller = new PinpointTurnController(Math.toRadians(-60), .35);
        while (!controller.timedOut()) {
            controller.update(plant.rate, .02);
            plant.step(controller.power(), .02);
            assertTrue(controller.elapsedSeconds() < 13);
        }
        assertTrue(controller.powerLimited());
        assertFalse(controller.settled());
        assertEquals(.35, Math.abs(controller.power()), .005);
        assertTrue(Math.abs(Math.toDegrees(plant.rate)) < 35);
    }

    @Test
    public void unstableFeedbackCanTimeOutWithoutFalselyReportingThePowerLimit() {
        PinpointTurnController controller = new PinpointTurnController(Math.toRadians(60), .7);
        while (!controller.timedOut()) {
            double t = controller.elapsedSeconds();
            controller.update(Math.toRadians(60 + 40 * Math.sin(2 * Math.PI * t / 1.2)), .02);
            assertFalse(controller.settled());
            assertTrue(controller.elapsedSeconds() < 13);
        }
        assertFalse(controller.powerLimited());
        assertTrue(controller.peakPower() < .7);
    }

    @Test
    public void recoversAfterSaturationWithoutKeepingTheIntegratorWoundUp() {
        PinpointTurnController controller = new PinpointTurnController(Math.toRadians(60), .7);
        for (int n = 0; n < 400; n++) controller.update(0, .02);
        assertTrue(controller.powerLimited());
        Plant plant = new Plant(.18, .135, 400);
        double peakRate = 0;
        for (int n = 0; n < 800; n++) {
            controller.update(plant.rate, .02);
            plant.step(controller.power(), .02);
            peakRate = Math.max(peakRate, plant.rate);
        }
        assertTrue("Release overspeed " + Math.toDegrees(peakRate), Math.toDegrees(peakRate) < 125);
        assertTrue(controller.settled());
        assertFalse(controller.powerLimited());
        assertEquals(60, Math.toDegrees(plant.rate), .5);
    }

    @Test
    public void cannotStartCollectingWhileStillRampingToTheRequestedRate() {
        PinpointTurnController controller = new PinpointTurnController(Math.toRadians(60), .7);
        for (int n = 0; n < 40; n++) {
            controller.update(Math.toRadians(60 * n * .02), .02);
            assertFalse(controller.settled());
        }
        for (int n = 0; n < 60; n++) controller.update(Math.toRadians(60), .02);
        assertTrue(controller.settled());
    }

    @Test
    public void reachesFastRateWhilePositionFeedbackContainsDrift() {
        Plant yaw = new Plant(.32, .25, 180);
        PinpointTurnController turn = new PinpointTurnController(Math.toRadians(-60), .7);
        PinpointPositionHold hold = new PinpointPositionHold(-26, -161);
        double x = 0, y = 0, maxTravel = 0, firstSettled = Double.POSITIVE_INFINITY;
        for (int n = 0; n < 3000; n++) {
            double dt = .02;
            turn.update(yaw.rate, dt);
            hold.control(yaw.heading, dt, .7 - Math.abs(turn.power()));
            double f = hold.forwardPower(), l = hold.leftPower(), r = turn.power();
            for (double motor : new double[]{f - l - r, f + l + r, f + l - r, f - l + r}) {
                assertTrue(Math.abs(motor) <= .7 + 1e-12);
            }
            double oldHeading = yaw.heading;
            yaw.step(r, dt);
            double angle = yaw.heading - oldHeading, mid = (oldHeading + yaw.heading) / 2;
            double cos = Math.cos(mid), sin = Math.sin(mid);
            double dx = (cos * 420 * f - sin * 340 * l + 20) * dt;
            double dy = (sin * 420 * f + cos * 340 * l - 9) * dt;
            x += dx;
            y += dy;
            hold.observe(cos * dx + sin * dy + 26 * angle, -sin * dx + cos * dy - 161 * angle, angle, mid, dt);
            maxTravel = Math.max(maxTravel, Math.hypot(x, y));
            if (turn.settled()) firstSettled = Math.min(firstSettled, turn.elapsedSeconds());
        }
        assertTrue(firstSettled < PinpointTurnController.SETTLE_TIMEOUT_SECONDS);
        assertEquals(-60, Math.toDegrees(yaw.rate), .5);
        assertTrue("Physical drift " + maxTravel, maxTravel < 120);
    }

    private static double run(PinpointTurnController controller, Plant plant, double seconds) {
        double firstSettled = Double.POSITIVE_INFINITY;
        while (controller.elapsedSeconds() < seconds) {
            controller.update(plant.rate, .02);
            plant.step(controller.power(), .02);
            if (controller.settled()) firstSettled = Math.min(firstSettled, controller.elapsedSeconds());
        }
        return firstSettled;
    }

    private static final class Plant {
        final double breakaway, gain;
        double rolling, rate, heading;

        Plant(double breakaway, double rolling, double degreesPerSecondPerPower) {
            this.breakaway = breakaway;
            this.rolling = rolling;
            gain = Math.toRadians(degreesPerSecondPerPower);
        }

        void step(double power, double dt) {
            double target = Math.abs(rate) < .01 && Math.abs(power) < breakaway ? 0
                    : Math.signum(power) * Math.max(0, Math.abs(power) - rolling) * gain;
            rate += (1 - Math.exp(-dt / .12)) * (target - rate);
            heading += rate * dt;
        }
    }

    private static double clip(double value, double min, double max) { return Math.max(min, Math.min(max, value)); }
}
