package com.technodot.ftc.twentyfive.batch;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class Batch {

    public boolean running = false;
    public long startNs = 0;
    public long currentNs = 0;

    private static final class Action {
        final long startOffsetNs;
        final long durationNs;
        final long endNs;
        final Runnable callback;
        boolean completed;

        Action(long startOffsetNs, long durationNs, Runnable callback) {
            this.startOffsetNs = startOffsetNs;
            this.durationNs = durationNs;
            this.endNs = startOffsetNs + durationNs;
            this.callback = callback;
        }

        boolean isActive(long elapsedNs) {
            if (durationNs == 0) {
                if (completed) {
                    return false;
                }
                if (elapsedNs >= startOffsetNs) {
                    completed = true;
                    return true;
                }
                return false;
            }
            return elapsedNs >= startOffsetNs && elapsedNs < endNs;
        }

        void reset() {
            completed = false;
        }
    }

    private final List<Action> actions = new ArrayList<>();

    public Batch plan(long startMs, long durationMs, Runnable action) {
        if (action == null) {
            throw new IllegalArgumentException("action must not be null");
        }
        long clampedStartMs = Math.max(0L, startMs);
        long clampedDurationMs = Math.max(0L, durationMs);
        long startOffsetNs = TimeUnit.MILLISECONDS.toNanos(clampedStartMs);
        long durationNs = TimeUnit.MILLISECONDS.toNanos(clampedDurationMs);
        actions.add(new Action(startOffsetNs, durationNs, action));
        return this;
    }

    public void run() {
        currentNs = System.nanoTime();
        if (!running) {
            running = true;
            startNs = currentNs;
        }

        long elapsedNs = currentNs - startNs;

        for (Action action : actions) {
            if (action.isActive(elapsedNs)) {
                action.callback.run();
            }
        }
    }

    public void reset() {
        running = false;
        startNs = 0;
        currentNs = 0;
        for (Action action : actions) {
            action.reset();
        }
    }
}
