package org.technodot.ftc.twentyfivebeta.batch;

@FunctionalInterface
public interface InterruptibleCallback extends CallbackInterface {
    /**
     * Executes the callback.
     * @return false to continue execution, true to interrupt.
     */
    boolean execute();
}
