package com.technodot.ftc.twentyfive.batch;

import java.util.ArrayList;
import java.util.List;

public class Batch {

    public boolean running = false;
    public long startNs = 0;
    public long currentNs = 0;

    private final List<Action> actions = new ArrayList<>();

    public Batch plan(long startMs, long durationMs, Runnable callback) {
        if (callback == null) {
            throw new IllegalArgumentException("action is cooked gng \uD83E\uDD40");
        }
        // 1s = 1_000ms = 1_000_000us = 1_000_000_000ns
        long startNs = Math.max(0L, startMs) * 1_000_000L;
        long durationNs = Math.max(0L, durationMs) * 1_000_000L;
        actions.add(new Action(startNs, durationNs, callback));
        return this;
    }

    public Batch plan(long startMs, Runnable callback) {
        if (callback == null) {
            throw new IllegalArgumentException("action is cooked gng \uD83E\uDD40");
        }
        // 1s = 1_000ms = 1_000_000us = 1_000_000_000ns
        long startNs = Math.max(0L, startMs) * 1_000_000L;
        actions.add(new Action(startNs, callback));
        return this;
    }

    public Batch plan(Action action) {
        actions.add(action);
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
