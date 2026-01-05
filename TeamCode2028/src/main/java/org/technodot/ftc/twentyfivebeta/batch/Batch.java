package org.technodot.ftc.twentyfivebeta.batch;

import java.util.ArrayList;
import java.util.List;

/**
 * Schedules and runs Actions relative to a shared start time.
 */
public class Batch {

    public boolean running = false;
    public long startNs = 0;
    public long currentNs = 0;

    private final List<Action> actions = new ArrayList<>();

    /**
     * Creates an empty Batch.
     */
    public Batch() {

    }

    /**
     * Adds a timed action that runs within a fixed window.
     */
    public Batch plan(long startMs, long durationMs, CallbackInterface callback) {
        if (callback == null) {
            throw new IllegalArgumentException("action is cooked gng \uD83E\uDD40");
        }
        // 1s = 1_000ms = 1_000_000us = 1_000_000_000ns
        long startNs = Math.max(0L, startMs) * 1_000_000L;
        long durationNs = Math.max(0L, durationMs) * 1_000_000L;
        actions.add(new Action(startNs, durationNs, callback));
        return this;
    }

    /**
     * Adds an instantaneous action triggered once.
     */
    public Batch plan(long startMs, CallbackInterface callback) {
        if (callback == null) {
            throw new IllegalArgumentException("action is cooked gng \uD83E\uDD40");
        }
        // 1s = 1_000ms = 1_000_000us = 1_000_000_000ns
        long startNs = Math.max(0L, startMs) * 1_000_000L;
        actions.add(new Action(startNs, callback));
        return this;
    }

    /**
     * Adds a fully configured action.
     */
    public Batch plan(Action action) {
        actions.add(action);
        return this;
    }

    /**
     * Invokes all actions that are active at the current time.
     */
    public void run() {
        currentNs = System.nanoTime();
        if (!running) {
            running = true;
            startNs = currentNs;
        }

        long elapsedNs = currentNs - startNs;

        for (Action action : actions) {
            action.invoke(elapsedNs);
        }
    }

    /**
     * Clears runtime state so the batch can run again.
     */
    public void reset() {
        running = false;
        startNs = 0;
        currentNs = 0;
        for (Action action : actions) {
            action.reset();
        }
    }
}
