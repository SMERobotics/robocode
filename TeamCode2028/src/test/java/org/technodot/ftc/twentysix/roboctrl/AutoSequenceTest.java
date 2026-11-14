package org.technodot.ftc.twentysix.roboctrl;

import org.junit.Test;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.junit.Assert.*;

public class AutoSequenceTest {
    private static long ms(long millis) { return millis * 1_000_000L; }

    @Test public void readinessEndsAWaitEarlyAndStartsTheNextDelayFromThatMoment() {
        boolean[] ready = {false};
        List<String> calls = new ArrayList<>();
        AutoSequence sequence = new AutoSequence()
                .then("spin up", () -> calls.add("spin"))
                .await("speed", 1000, () -> ready[0])
                .then("open", () -> calls.add("open"))
                .delay("clearance", 100)
                .then("feed", () -> calls.add("feed"));
        sequence.update(ms(0));
        assertEquals(Arrays.asList("spin"), calls);
        ready[0] = true;
        sequence.update(ms(40));
        assertEquals(Arrays.asList("spin", "open"), calls);
        sequence.update(ms(139));
        assertEquals(2, calls.size());
        sequence.update(ms(140));
        assertEquals(Arrays.asList("spin", "open", "feed"), calls);
        assertTrue(sequence.isFinished());
        sequence.update(ms(10000));
        assertEquals(3, calls.size());
    }

    @Test public void timeoutPreventsAllFollowingShootingActions() {
        List<String> calls = new ArrayList<>();
        AutoSequence sequence = new AutoSequence()
                .await("aim", 100, () -> false)
                .then("shoot", () -> calls.add("shoot"));
        sequence.update(ms(10));
        sequence.update(ms(110));
        sequence.update(ms(10000));
        assertTrue(sequence.isFinished());
        assertEquals("Timed out: aim", sequence.getStatus());
        assertTrue(calls.isEmpty());
    }

    @Test public void slowLoopsCannotSkipAnyOfTheFourSeparateFeedPulses() {
        List<String> calls = new ArrayList<>();
        AutoSequence sequence = new AutoSequence();
        for (int i = 1; i <= 4; i++) {
            final int shot = i;
            sequence.then("open", () -> calls.add("open" + shot))
                    .delay("feed", 150)
                    .then("close", () -> calls.add("close" + shot))
                    .delay("separate", 400);
        }
        sequence.update(ms(0));
        sequence.update(ms(5000));
        // Even a five-second scheduling gap must not consume the next shot's dwell time.
        assertEquals(Arrays.asList("open1", "close1"), calls);
        long now = 5000;
        while (!sequence.isFinished()) sequence.update(ms(now += 5000));
        assertEquals(Arrays.asList("open1", "close1", "open2", "close2",
                "open3", "close3", "open4", "close4"), calls);
    }

    @Test public void cancelPreventsAnyNewActions() {
        List<String> calls = new ArrayList<>();
        AutoSequence sequence = new AutoSequence()
                .delay("wait", 100)
                .then("shoot", () -> calls.add("shoot"));
        sequence.update(ms(0));
        sequence.cancel();
        sequence.update(ms(1000));
        assertEquals("stopped", sequence.getStatus());
        assertTrue(calls.isEmpty());
    }
}
