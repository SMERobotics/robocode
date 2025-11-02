package com.technodot.ftc.twentyfive.batch;

/**
 * Represents a time-bound action that can be scheduled inside a {@link Batch}.
 *
 * <p>An implementation may choose to leave {@link #onStart(BatchRuntime)} and
 * {@link #onFinish(BatchRuntime)} empty if setup or teardown is unnecessary.
 * The {@link #onUpdate(BatchRuntime, long)} hook will be called on every loop
 * iteration while the action is active.</p>
 */
public interface BatchAction {

    /**
     * Called once when the action first becomes active.
     *
     * @param runtime shared timing information for the batch
     */
    default void onStart(BatchRuntime runtime) { }

    /**
     * Called on each loop while the action is active.
     *
     * @param runtime shared timing information for the batch
     * @param elapsedMillis how long this action has been running
     */
    void onUpdate(BatchRuntime runtime, long elapsedMillis);

    /**
     * Called exactly once when the action finishes.
     *
     * @param runtime shared timing information for the batch
     */
    default void onFinish(BatchRuntime runtime) { }
}
