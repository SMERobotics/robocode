package org.technodot.ftc.twentyfivebeta.batch;

@FunctionalInterface
public interface ExecutionCallback extends CallbackInterface {
    /**
     * Executes the callback.
     * @param currentMs The current time in milliseconds, since the beginning of the callback execution.
     */
    void execute(long currentMs);
}
