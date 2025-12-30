package org.technodot.ftc.twentyfivebeta.batch;

@FunctionalInterface
public interface InterruptibleExecutionCallback {
    boolean invoke(long currentMs, long durationMs);
}
