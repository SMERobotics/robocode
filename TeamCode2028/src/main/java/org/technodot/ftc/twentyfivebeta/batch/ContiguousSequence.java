package org.technodot.ftc.twentyfivebeta.batch;

public class ContiguousSequence extends Sequence {

    private long cursorNs;

    public ContiguousSequence() {
        super();
    }

    public ContiguousSequence(long startMs) {
        super(startMs);
    }

    public ContiguousSequence then(long durationMs, CallbackInterface callback) {
        long startNs = nextStartNs();
        long durationNs = msToNs(durationMs);
        Action action = new Action(startNs, durationNs, requireCallback(callback));
        addAction(action);
        cursorNs = scheduledEndNs(action);
        return this;
    }

    public ContiguousSequence then(CallbackInterface callback) {
        long startNs = nextStartNs();
        Action action = new Action(startNs, requireCallback(callback));
        addAction(action);
        cursorNs = scheduledEndNs(action);
        return this;
    }

    public ContiguousSequence delay(long durationMs) {
        long startNs = nextStartNs();
        long durationNs = msToNs(durationMs);
        Action delay = createDelayAction(startNs, durationNs);
        addAction(delay);
        cursorNs = scheduledEndNs(delay);
        return this;
    }

    @Override
    protected void handleActionCompletion(Action action, long effectiveElapsedNs) {
        long scheduledEnd = scheduledEndNs(action);
        if (scheduledEnd > action.startNs && effectiveElapsedNs < scheduledEnd) {
            timeShiftNs += scheduledEnd - effectiveElapsedNs;
        }
    }

    private long nextStartNs() {
        cursorNs = Math.max(cursorNs, getPlannedDurationNs());
        return cursorNs;
    }
}
