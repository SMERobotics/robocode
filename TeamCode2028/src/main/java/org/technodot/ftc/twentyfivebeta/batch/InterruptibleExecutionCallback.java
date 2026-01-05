package org.technodot.ftc.twentyfivebeta.batch;

@FunctionalInterface
public interface InterruptibleExecutionCallback extends CallbackInterface {
    /**
     * Executes the callback.
     * @param currentMs The current time in milliseconds, since the beginning of the callback execution.
     * @return false to continue execution, true to interrupt.
     */
    boolean execute(long currentMs);
}
