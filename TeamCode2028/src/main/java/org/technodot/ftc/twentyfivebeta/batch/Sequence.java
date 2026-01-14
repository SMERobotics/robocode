package org.technodot.ftc.twentyfivebeta.batch;

import java.util.ArrayList;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Set;

/**
 * Base Action that manages nested actions on a shared timeline.
 */
public class Sequence extends Action {

    private static final CallbackInterface NO_SEQUENCE_CALLBACK = new CallbackInterface() {};
    private static final Callback DELAY_CALLBACK = () -> {};
    private static final String COOKED_ACTION = "action is cooked gng \uD83E\uDD40";

    protected final List<Action> actions = new ArrayList<>();
    private final Set<Action> finishedActions = Collections.newSetFromMap(new IdentityHashMap<>());

    private boolean running;
    private long sequenceStartNs;
    protected long timeShiftNs;
    private long plannedDurationNs;

    /**
     * Creates a sequence that starts immediately.
     */
    public Sequence() {
        this(0);
    }

    /**
     * Creates a sequence that starts at the given delay.
     */
    public Sequence(long startMs) {
        super(msToNs(startMs), 0, NO_SEQUENCE_CALLBACK);
    }

    /**
     * Adds a timed child action.
     */
    public Sequence plan(long startMs, long durationMs, CallbackInterface callback) {
        long startNs = msToNs(startMs);
        long durationNs = msToNs(durationMs);
        addAction(new Action(startNs, durationNs, requireCallback(callback)));
        return this;
    }

    /**
     * Adds an instantaneous child action.
     */
    public Sequence plan(long startMs, CallbackInterface callback) {
        long startNs = msToNs(startMs);
        addAction(new Action(startNs, requireCallback(callback)));
        return this;
    }

    /**
     * Adds a prebuilt child action.
     */
    public Sequence plan(Action action) {
        addAction(action);
        return this;
    }

    /**
     * Adds a delay placeholder.
     */
    public Sequence delay(long startMs, long durationMs) {
        long startNs = msToNs(startMs);
        long durationNs = msToNs(durationMs);
        addAction(createDelayAction(startNs, durationNs));
        return this;
    }

    /**
     * Hook for subclasses to validate actions before storing them.
     */
    protected void addAction(Action action) {
        addActionInternal(requireAction(action));
    }

    /**
     * Appends an action and updates duration bookkeeping.
     */
    protected final void addActionInternal(Action action) {
        actions.add(action);
        updatePlannedDuration(action);
    }

    /**
     * Builds a no-op delay Action.
     */
    protected Action createDelayAction(long startNs, long durationNs) {
        return new Action(startNs, durationNs, DELAY_CALLBACK);
    }

    /**
     * Returns the scheduled finish time of an action.
     */
    protected long scheduledEndNs(Action action) {
        if (action instanceof Sequence) {
            long nestedLength = ((Sequence) action).getPlannedDurationNs();
            return action.startNs + Math.max(0L, nestedLength);
        }
        if (action.durationNs == 0) {
            return action.startNs;
        }
        return action.endNs;
    }

    /**
     * Validates an action reference.
     */
    protected final Action requireAction(Action action) {
        if (action == null) {
            throw new IllegalArgumentException(COOKED_ACTION);
        }
        return action;
    }

    /**
     * Validates a callback reference.
     */
    protected final CallbackInterface requireCallback(CallbackInterface callback) {
        if (callback == null) {
            throw new IllegalArgumentException(COOKED_ACTION);
        }
        return callback;
    }

    /**
     * Expands the cached duration using the action span.
     */
    protected void updatePlannedDuration(Action action) {
        plannedDurationNs = Math.max(plannedDurationNs, scheduledEndNs(action));
    }

    /**
     * @return total scheduled length in nanoseconds.
     */
    public long getPlannedDurationNs() {
        return plannedDurationNs;
    }

    /**
     * Sequences stay active once their start time is reached.
     */
    @Override
    public boolean isActive(long elapsedNs) {
        if (completed) return false;
        return elapsedNs >= startNs;
    }

    /**
     * Updates children using elapsed time relative to the sequence.
     */
    @Override
    public void invoke(long elapsedNs) {
        if (completed) return;
        if (elapsedNs < startNs) return;

        if (!running) {
            running = true;
            sequenceStartNs = elapsedNs;
        }

        if (actions.isEmpty()) {
            completed = true;
            return;
        }

        long sequenceElapsedNs = elapsedNs - sequenceStartNs;
        long effectiveElapsedNs = mapElapsed(sequenceElapsedNs);

        for (Action action : actions) {
            action.invoke(effectiveElapsedNs);

            if (markActionFinished(action, effectiveElapsedNs)) {
                effectiveElapsedNs = mapElapsed(sequenceElapsedNs);
            }
        }

        if (finishedActions.size() == actions.size()) {
            completed = true;
        }
    }

    /**
     * Maps elapsed sequence time to child times.
     */
    protected long mapElapsed(long sequenceElapsedNs) {
        return sequenceElapsedNs + timeShiftNs;
    }

    /**
     * Tracks completion state and allows subclasses to react.
     */
    private boolean markActionFinished(Action action, long effectiveElapsedNs) {
        if (finishedActions.contains(action)) return false;

        if (hasFinished(action, effectiveElapsedNs)) {
            finishedActions.add(action);
            handleActionCompletion(action, effectiveElapsedNs);
            return true;
        }
        return false;
    }

    /**
     * Determines when an action is fully done.
     */
    protected boolean hasFinished(Action action, long effectiveElapsedNs) {
        if (action instanceof Sequence) {
            return action.completed;
        }
        return action.completed || effectiveElapsedNs >= action.endNs;
    }

    /**
     * Optional hook invoked after a child ends.
     */
    protected void handleActionCompletion(Action action, long effectiveElapsedNs) {
    }

    /**
     * Resets runtime state across the sequence tree.
     */
    @Override
    public void reset() {
        super.reset();
        running = false;
        sequenceStartNs = 0;
        timeShiftNs = 0;
        finishedActions.clear();
        for (Action action : actions) {
            action.reset();
        }
    }

    /**
     * Converts milliseconds to bounded nanoseconds.
     */
    protected static long msToNs(long ms) {
        return Math.max(0L, ms) * 1_000_000L;
    }
}
