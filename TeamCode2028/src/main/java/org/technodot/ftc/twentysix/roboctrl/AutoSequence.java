package org.technodot.ftc.twentysix.roboctrl;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

/** Non-blocking callbacks, delays and bounded readiness waits, following the beta autos' sequence style. */
public final class AutoSequence {
    private static final class Step {
        final String name;
        final Runnable action;
        final BooleanSupplier ready;
        final long durationNanos;

        Step(String name, Runnable action, BooleanSupplier ready, long durationMillis) {
            this.name = name;
            this.action = action;
            this.ready = ready;
            this.durationNanos = Math.max(0, durationMillis) * 1_000_000L;
        }
    }

    private final List<Step> steps = new ArrayList<>();
    private int index;
    private boolean entered;
    private boolean cancelled;
    private long stepStarted;
    private String failure;

    public AutoSequence then(String name, Runnable action) {
        steps.add(new Step(name, action, null, 0));
        return this;
    }

    public AutoSequence delay(String name, long millis) {
        steps.add(new Step(name, null, null, millis));
        return this;
    }

    public AutoSequence await(String name, long timeoutMillis, BooleanSupplier ready) {
        steps.add(new Step(name, null, ready, timeoutMillis));
        return this;
    }

    public void update(long now) {
        while (!isFinished()) {
            Step step = steps.get(index);
            if (!entered) {
                entered = true;
                stepStarted = now;
                if (step.action != null) step.action.run();
            }
            long elapsed = now - stepStarted;
            if (step.ready != null) {
                if (!step.ready.getAsBoolean()) {
                    if (elapsed >= step.durationNanos) failure = "Timed out: " + step.name;
                    return;
                }
            } else if (elapsed < step.durationNanos) {
                return;
            }
            index++;
            entered = false;
        }
    }

    public boolean isFinished() {
        return cancelled || failure != null || index >= steps.size();
    }

    public String getStatus() {
        if (failure != null) return failure;
        if (cancelled) return "stopped";
        return index >= steps.size() ? "complete" : steps.get(index).name;
    }

    public void cancel() {
        cancelled = true;
    }
}
