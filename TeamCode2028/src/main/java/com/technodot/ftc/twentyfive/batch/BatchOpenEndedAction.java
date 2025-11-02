package com.technodot.ftc.twentyfive.batch;

/**
 * An action that decides for itself when it has completed, instead of relying
 * on a pre-planned duration.
 */
public interface BatchOpenEndedAction extends BatchAction {

    /**
     * Called after {@link #onUpdate(BatchRuntime, long)} every loop in order to
     * decide whether the action has finished.
     *
     * @param runtime shared timing information for the batch
     * @param elapsedMillis how long this action has been running
     * @return {@code true} when the action is complete and should be removed
     */
    boolean shouldFinish(BatchRuntime runtime, long elapsedMillis);
}
