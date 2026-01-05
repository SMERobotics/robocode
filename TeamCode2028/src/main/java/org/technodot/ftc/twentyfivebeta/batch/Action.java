package org.technodot.ftc.twentyfivebeta.batch;

public class Action {

    // relative to batch start time
    public final long startNs;
    public final long durationNs;
    public final long endNs;
    public final CallbackInterface callback;
    public boolean completed;

    public Action(long startNs, long durationNs, CallbackInterface callback) {
        this.startNs = startNs;
        this.durationNs = durationNs;
        this.endNs = startNs + durationNs;
        this.callback = callback;
    }

    public Action(long startNs, CallbackInterface callback) {
        this.startNs = startNs;
        this.durationNs = 0;
        this.endNs = startNs;
        this.callback = callback;
    }

    public boolean isActive(long elapsedNs) {
        // relative to batch start time
        if (completed) return false;
        if (durationNs == 0) return elapsedNs >= startNs;
        return elapsedNs >= startNs && elapsedNs < endNs;
    }

    public void invoke(long elapsedNs) {
        if (completed) return;

        if (this.isActive(elapsedNs)) {
            if (callback instanceof Callback) {
                ((Callback) callback).execute();
            } else if (callback instanceof InterruptibleCallback) {
                boolean interrupt = ((InterruptibleCallback) callback).execute();
                if (interrupt) completed = true;
            } else if (callback instanceof ExecutionCallback) {
                // relative to batch start time
                ((ExecutionCallback) callback).execute((elapsedNs - startNs) / 1_000_000L);
            } else if (callback instanceof InterruptibleExecutionCallback) {
                // relative to batch start time
                boolean interrupt = ((InterruptibleExecutionCallback) callback).execute((elapsedNs - startNs) / 1_000_000L);
                if (interrupt) completed = true;
            }
            // what goofy ahh kinda callback is ts then?

            if (durationNs == 0) completed = true;
        }
    }

    public void reset() {
        completed = false;
    }
}
