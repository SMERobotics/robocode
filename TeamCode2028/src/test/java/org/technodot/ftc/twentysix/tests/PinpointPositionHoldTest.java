package org.technodot.ftc.twentysix.tests;

import org.junit.Test;

import static org.junit.Assert.*;

public class PinpointPositionHoldTest {
    @Test
    public void containsSustainedDriftThroughBothDirectionsAndSpeeds() {
        Plant plant = new Plant(-26, -161);
        double maxDistance = 0;
        double[] rates = {30, -30, -60, 60, 45, -45};
        for (double rate : rates) {
            for (int n = 0; n < 1200; n++) {
                // Loop jitter, quantized pods, inertial response and a 20/-9
                // mm/s disturbance which would travel over 3 m unchecked.
                plant.step(.014 + .004 * (n % 4), Math.toRadians(rate), .35);
                maxDistance = Math.max(maxDistance, Math.hypot(plant.x, plant.y));
            }
            plant.hold.pause();
        }
        assertTrue("Peak physical travel " + maxDistance, maxDistance < 160);
        assertTrue("Final physical travel", Math.hypot(plant.x, plant.y) < 70);
        assertTrue("Final monitored travel", plant.hold.distanceMm() < 70);
    }

    @Test
    public void containsDriftWithImperfectSeedOffsetsAndAnEllipticalOrbit() {
        Plant plant = new Plant(49, -206);
        double maxDistance = 0;
        for (int n = 0; n < 8000; n++) {
            plant.step(.02, n < 4000 ? Math.toRadians(30) : Math.toRadians(-60), .35);
            maxDistance = Math.max(maxDistance, Math.hypot(plant.x, plant.y));
        }
        assertTrue("Peak physical travel " + maxDistance, maxDistance < 260);
        assertTrue("Held point is bounded", plant.hold.distanceMm() < 160);
    }

    @Test
    public void incompleteYawWithWrongSeedsDoesNotBecomeUnboundedTravel() {
        PinpointPositionHold hold = new PinpointPositionHold(49, -206);
        double heading = 0;
        double angle = Math.toRadians(1);
        for (int turn = 0; turn < 20; turn++) {
            for (int n = 0; n < 360; n++) {
                hold.observe(26 * angle, -161 * angle, angle, heading + angle / 2, .02);
                heading += angle;
                assertTrue(hold.distanceMm() < 176);
            }
            assertEquals(0, hold.distanceMm(), 1e-8);
        }
    }

    @Test
    public void brakingResetsCommandsButRetainsDisplacementFromTheOriginalStart() {
        PinpointPositionHold hold = new PinpointPositionHold(0, 0);
        hold.observe(80, -40, 0, 0, .1);
        hold.control(0, .02, .3);
        assertTrue(hold.forwardPower() < 0);
        assertTrue(hold.leftPower() > 0);
        hold.pause();
        assertEquals(0, hold.forwardPower(), 0);
        assertEquals(0, hold.leftPower(), 0);
        assertEquals(80, hold.xMm(), 0);
        assertEquals(-40, hold.yMm(), 0);
        hold.observe(5, 0, 0, 0, .1);
        assertEquals(85, hold.xMm(), 0);
    }

    @Test
    public void restoresTheSameFieldPositionAtEveryRobotHeading() {
        for (double heading : new double[]{0, Math.PI / 2, Math.PI, -Math.PI / 2}) {
            PinpointPositionHold hold = new PinpointPositionHold(0, 0);
            hold.observe(100, 0, 0, 0, .1);
            hold.control(heading, .02, .3);
            double fieldX = Math.cos(heading) * hold.forwardPower() - Math.sin(heading) * hold.leftPower();
            double fieldY = Math.sin(heading) * hold.forwardPower() + Math.cos(heading) * hold.leftPower();
            assertTrue(fieldX < 0);
            assertEquals(0, fieldY, 1e-12);
        }
    }

    @Test
    public void reservesTurnPowerEvenWhenHeadingAndAvailablePowerChangeAbruptly() {
        PinpointPositionHold hold = new PinpointPositionHold(0, 0);
        hold.observe(600, 250, 0, 0, .1);
        double maxPower = .7;
        for (int n = 0; n < 1200; n++) {
            double turn = new double[]{.25, -.55, .699, -.7, 0}[n % 5];
            hold.control(n * .15, .02, maxPower - Math.abs(turn));
            double f = hold.forwardPower(), l = hold.leftPower();
            double[] wheels = {f - l - turn, f + l + turn, f + l - turn, f - l + turn};
            for (double wheel : wheels) assertTrue(Math.abs(wheel) <= maxPower + 1e-12);
            assertEquals(turn, (-wheels[0] + wheels[1] - wheels[2] + wheels[3]) / 4, 1e-12);
            assertTrue(Math.abs(f) + Math.abs(l) <= .28 + 1e-12);
        }
    }

    @Test
    public void noAvailablePowerCannotWindUpThePositionIntegrator() {
        PinpointPositionHold saturated = new PinpointPositionHold(0, 0);
        PinpointPositionHold fresh = new PinpointPositionHold(0, 0);
        for (PinpointPositionHold hold : new PinpointPositionHold[]{saturated, fresh}) {
            hold.observe(100, 0, 0, 0, .02);
            for (int n = 0; n < 200; n++) hold.observe(0, 0, 0, 0, .02);
        }
        for (int n = 0; n < 2000; n++) saturated.control(0, .02, 0);
        for (int n = 0; n < 30; n++) {
            saturated.control(0, .02, .25);
            fresh.control(0, .02, .25);
            assertEquals(fresh.forwardPower(), saturated.forwardPower(), 1e-12);
        }
    }

    @Test
    public void uncontainedPhysicalDriftStillReachesTheTravelLimit() {
        Plant plant = new Plant(-26, -161);
        plant.driftX = 250;
        for (int n = 0; n < 300; n++) plant.step(.02, Math.toRadians(30), .02);
        assertTrue(Math.hypot(plant.x, plant.y) > 750);
        assertTrue(plant.hold.distanceMm() > 750);
        plant.hold.pause();
        assertTrue(plant.hold.distanceMm() > 750);
    }

    private static final class Plant {
        final PinpointPositionHold hold;
        double x, y, vx, vy, heading;
        double podX, podY;
        int countX, countY;
        double driftX = 20;

        Plant(double seedX, double seedY) { hold = new PinpointPositionHold(seedX, seedY); }

        void step(double dt, double rate, double budget) {
            double angle = rate * dt, mid = heading + angle / 2;
            double cos = Math.cos(mid), sin = Math.sin(mid);
            double f = 420 * hold.forwardPower(), l = 340 * hold.leftPower();
            double wantX = cos * f - sin * l + driftX - 25 * rate * Math.sin(mid);
            double wantY = sin * f + cos * l - 9 + 12 * rate * Math.cos(mid);
            vx += dt / (.12 + dt) * (wantX - vx);
            vy += dt / (.12 + dt) * (wantY - vy);
            double dx = vx * dt, dy = vy * dt;
            x += dx;
            y += dy;
            podX += (cos * dx + sin * dy + 26 * angle) * 13.26291192;
            podY += (-sin * dx + cos * dy - 161 * angle) * 13.26291192;
            int nextX = (int) Math.round(podX), nextY = (int) Math.round(podY);
            hold.observe((nextX - countX) / 13.26291192, (nextY - countY) / 13.26291192, angle, mid, dt);
            countX = nextX;
            countY = nextY;
            heading += angle;
            hold.control(heading, dt, budget);
        }
    }
}
