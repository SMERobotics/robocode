package org.technodot.ftc.twentyfivebeta.batch;

/**
 * Sequence that executes entries back-to-back, filling gaps automatically.
 */
public class ContiguousSequence extends Sequence {

    private long cursorNs;

    /**
     * Creates a ContiguousSequence that starts immediately.
     */
    public ContiguousSequence() {
        super();
    }

    /**
     * Creates a ContiguousSequence that starts after a delay.
     */
    public ContiguousSequence(long startMs) {
        super(startMs);
    }

    /**
     * Appends a timed action directly after the previous entry.
     */
    public ContiguousSequence then(long durationMs, CallbackInterface callback) {
        long startNs = nextStartNs();
        long durationNs = msToNs(durationMs);
        Action action = new Action(startNs, durationNs, requireCallback(callback));
        addAction(action);
        cursorNs = scheduledEndNs(action);
        return this;
    }

    /**
     * Appends an instantaneous action after the previous entry.
     */
    public ContiguousSequence then(CallbackInterface callback) {
        long startNs = nextStartNs();
        Action action = new Action(startNs, requireCallback(callback));
        addAction(action);
        cursorNs = scheduledEndNs(action);
        return this;
    }

    /**
     * Appends a delay block after the previous entry.
     */
    public ContiguousSequence delay(long durationMs) {
        long startNs = nextStartNs();
        long durationNs = msToNs(durationMs);
        Action delay = createDelayAction(startNs, durationNs);
        addAction(delay);
        cursorNs = scheduledEndNs(delay);
        return this;
    }

    /**
     * Shifts the timeline forward when an action ends early.
     */
    @Override
    protected void handleActionCompletion(Action action, long effectiveElapsedNs) {
        long scheduledEnd = scheduledEndNs(action);
        if (scheduledEnd > action.startNs && effectiveElapsedNs < scheduledEnd) {
            timeShiftNs += scheduledEnd - effectiveElapsedNs;
        }
    }

    /**
     * Returns the next contiguous start time.
     */
    private long nextStartNs() {
        cursorNs = Math.max(cursorNs, getPlannedDurationNs());
        return cursorNs;
    }
}
