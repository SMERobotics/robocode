package org.technodot.ftc.twentyfivebeta.batch;

@FunctionalInterface
public interface Callback extends CallbackInterface {
    /**
     * Executes the callback.
     */
    void execute();
}
