package org.technodot.ftc.twentyfive.batch;

@FunctionalInterface
public interface BatchCallback {
    boolean invoke(long startMs, long durationMs, long currentMs);
}
