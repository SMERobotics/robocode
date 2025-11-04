package com.technodot.ftc.twentyfive.batch;

public class Action {
    public final long startOffsetNs;
    public final long durationNs;
    public final long endNs;
    public final Runnable callback;
    public boolean completed;

    public Action(long startOffsetNs, long durationNs, Runnable callback) {
        this.startOffsetNs = startOffsetNs;
        this.durationNs = durationNs;
        this.endNs = startOffsetNs + durationNs;
        this.callback = callback;
    }

    public Action(long startOffsetNs, Runnable callback) {
        this.startOffsetNs = startOffsetNs;
        this.durationNs = 0;
        this.endNs = startOffsetNs;
        this.callback = callback;
    }

    public boolean isActive(long elapsedNs) {
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

    public void reset() {
        completed = false;
    }
}
