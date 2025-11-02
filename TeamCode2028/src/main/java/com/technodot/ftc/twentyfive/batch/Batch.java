package com.technodot.ftc.twentyfive.batch;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;
import java.util.Objects;

/**
 * Scheduler that orchestrates time-based actions for an autonomous routine.
 *
 * <p>Create a batch, register actions with {@link #plan(long, long, BatchAction)}
 * or {@link #planOpenEnded(long, BatchOpenEndedAction)}, and call {@link #run()}
121  * (or {@link #update(long)}) repeatedly inside the OpMode loop. The first call
 * starts the schedule; afterwards actions begin once their start offset has
 * been reached.</p>
 */
public final class Batch {

    private final List<ScheduledAction> allActions = new ArrayList<>();
    private final List<ScheduledAction> pendingActions = new ArrayList<>();
    private final List<ScheduledAction> activeActions = new ArrayList<>();

    private long startTimestampMillis = -1L;
    private boolean started;

    /**
     * Register a timed action to be executed.
     *
     * @param offsetMillis delay from batch start before the action begins
     * @param durationMillis how long the action should remain active
     * @param action logic to execute while the action is active
     * @return this batch for chaining
     */
    public Batch plan(long offsetMillis, long durationMillis, BatchAction action) {
        ensureNotStarted();

        if (offsetMillis < 0) {
            throw new IllegalArgumentException("offsetMillis must be >= 0");
        }
        if (durationMillis < 0) {
            throw new IllegalArgumentException("durationMillis must be >= 0");
        }

        ScheduledAction scheduledAction = ScheduledAction.timed(offsetMillis, durationMillis, action);
        addScheduledAction(scheduledAction);
        return this;
    }

    /**
     * Register an action that is allowed to decide when it has finished.
     *
     * @param offsetMillis delay from batch start before the action begins
     * @param action logic that updates itself until {@link BatchOpenEndedAction#shouldFinish(BatchRuntime, long)} returns true
     * @return this batch for chaining
     */
    public Batch planOpenEnded(long offsetMillis, BatchOpenEndedAction action) {
        ensureNotStarted();

        if (offsetMillis < 0) {
            throw new IllegalArgumentException("offsetMillis must be >= 0");
        }

        ScheduledAction scheduledAction = ScheduledAction.openEnded(offsetMillis, action);
        addScheduledAction(scheduledAction);
        return this;
    }

    /**
     * Clears runtime state and allows the batch to be started again with the same plan.
     */
    public void reset() {
        started = false;
        startTimestampMillis = -1L;

        for (ScheduledAction action : allActions) {
            action.reset();
        }

        pendingActions.clear();
        pendingActions.addAll(allActions);
        activeActions.clear();

        sortPendingByOffset();
    }

    /**
     * @return {@code true} once the batch has been started at least once
     */
    public boolean hasStarted() {
        return started;
    }

    /**
     * @return {@code true} when every scheduled action has finished
     */
    public boolean isComplete() {
        return started && pendingActions.isEmpty() && activeActions.isEmpty();
    }

    /**
     * Convenience helper that advances the batch using the current wall-clock time.
     */
    public void run() {
        update(System.currentTimeMillis());
    }

    /**
     * Main update loop. Call this method repeatedly with an increasing timestamp.
     *
     * @param nowMillis current time in milliseconds
     */
    public void update(long nowMillis) {
        if (allActions.isEmpty()) {
            return;
        }

        if (!started) {
            started = true;
            startTimestampMillis = nowMillis;
        }

        long batchElapsedMillis = nowMillis - startTimestampMillis;
        BatchRuntime runtime = new BatchRuntime(this, nowMillis, batchElapsedMillis);

        startReadyActions(runtime, batchElapsedMillis);
        updateActiveActions(runtime, batchElapsedMillis);
    }

    private void ensureNotStarted() {
        if (started) {
            throw new IllegalStateException("Cannot modify a batch after it has started");
        }
    }

    private void addScheduledAction(ScheduledAction scheduledAction) {
        Objects.requireNonNull(scheduledAction.action, "action cannot be null");

        allActions.add(scheduledAction);
        pendingActions.add(scheduledAction);
        sortPendingByOffset();
    }

    private void sortPendingByOffset() {
        pendingActions.sort(Comparator.comparingLong(action -> action.offsetMillis));
    }

    private void startReadyActions(BatchRuntime runtime, long batchElapsedMillis) {
        if (pendingActions.isEmpty()) {
            return;
        }

        Iterator<ScheduledAction> iterator = pendingActions.iterator();
        while (iterator.hasNext()) {
            ScheduledAction action = iterator.next();
            if (batchElapsedMillis >= action.offsetMillis) {
                iterator.remove();
                action.start(runtime, batchElapsedMillis);
                if (!action.finished) {
                    activeActions.add(action);
                }
            } else {
                // List is sorted, so we can bail out early
                break;
            }
        }
    }

    private void updateActiveActions(BatchRuntime runtime, long batchElapsedMillis) {
        if (activeActions.isEmpty()) {
            return;
        }

        Iterator<ScheduledAction> iterator = activeActions.iterator();
        while (iterator.hasNext()) {
            ScheduledAction action = iterator.next();
            long elapsedMillis = Math.max(0L, batchElapsedMillis - action.startedAtBatchMillis);

            action.action.onUpdate(runtime, elapsedMillis);

            if (action.shouldFinish(runtime, elapsedMillis)) {
                action.finish(runtime);
                iterator.remove();
            }
        }
    }

    private static final class ScheduledAction {
        final long offsetMillis;
        final BatchAction action;
        final Long durationMillis; // null for open-ended
        final BatchOpenEndedAction openEndedAction;

        boolean started;
        boolean finished;
        long startedAtBatchMillis;

        private ScheduledAction(long offsetMillis,
                                Long durationMillis,
                                BatchAction action,
                                BatchOpenEndedAction openEndedAction) {
            this.offsetMillis = offsetMillis;
            this.durationMillis = durationMillis;
            this.action = action;
            this.openEndedAction = openEndedAction;

            this.started = false;
            this.finished = false;
            this.startedAtBatchMillis = offsetMillis;
        }

        static ScheduledAction timed(long offsetMillis, long durationMillis, BatchAction action) {
            return new ScheduledAction(offsetMillis, durationMillis, action, null);
        }

        static ScheduledAction openEnded(long offsetMillis, BatchOpenEndedAction action) {
            return new ScheduledAction(offsetMillis, null, action, action);
        }

        void start(BatchRuntime runtime, long batchElapsedMillis) {
            if (started) {
                return;
            }

            started = true;
            startedAtBatchMillis = offsetMillis;

            action.onStart(runtime);

            // Ensure we run at least once even if we are significantly behind schedule.
            if (durationMillis != null && batchElapsedMillis - offsetMillis >= durationMillis) {
                finished = true;
                action.onUpdate(runtime, batchElapsedMillis - offsetMillis);
                action.onFinish(runtime);
            }
        }

        void finish(BatchRuntime runtime) {
            if (finished) {
                return;
            }

            finished = true;
            action.onFinish(runtime);
        }

        boolean shouldFinish(BatchRuntime runtime, long elapsedMillis) {
            if (finished) {
                return true;
            }

            if (durationMillis != null) {
                return elapsedMillis >= durationMillis;
            }

            if (openEndedAction != null) {
                return openEndedAction.shouldFinish(runtime, elapsedMillis);
            }

            return false;
        }

        void reset() {
            started = false;
            finished = false;
            startedAtBatchMillis = offsetMillis;
        }
    }
}
