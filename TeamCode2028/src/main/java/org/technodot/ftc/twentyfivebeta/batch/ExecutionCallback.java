package org.technodot.ftc.twentyfivebeta.batch;

@FunctionalInterface
public interface ExecutionCallback {
    void invoke(long currentMs, long durationMs);
}
