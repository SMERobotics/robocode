package org.technodot.ftc.twentyfivebeta.batch;

/**
 * Sequence variant that forbids overlapping actions.
 */
public class LinearSequence extends Sequence {

    /**
     * Creates a LinearSequence that starts immediately.
     */
    public LinearSequence() {
        super();
    }

    /**
     * Creates a LinearSequence that starts after a delay.
     */
    public LinearSequence(long startMs) {
        super(startMs);
    }

    /**
     * Adds a timed action and returns this sequence.
     */
    @Override
    public LinearSequence plan(long startMs, long durationMs, CallbackInterface callback) {
        super.plan(startMs, durationMs, callback);
        return this;
    }

    /**
     * Adds an instantaneous action and returns this sequence.
     */
    @Override
    public LinearSequence plan(long startMs, CallbackInterface callback) {
        super.plan(startMs, callback);
        return this;
    }

    /**
     * Adds a prebuilt action and returns this sequence.
     */
    @Override
    public LinearSequence plan(Action action) {
        super.plan(action);
        return this;
    }

    /**
     * Adds a delay marker and returns this sequence.
     */
    @Override
    public LinearSequence delay(long startMs, long durationMs) {
        super.delay(startMs, durationMs);
        return this;
    }

    /**
     * Validates each action before adding it to guarantee no overlap.
     */
    @Override
    protected void addAction(Action action) {
        Action candidate = requireAction(action);
        ensureNoOverlap(candidate);
        super.addActionInternal(candidate);
    }

    /**
     * Throws if an action intersects any previously scheduled span.
     */
    private void ensureNoOverlap(Action candidate) {
        long candidateStart = candidate.startNs;
        long candidateEnd = scheduledEndNs(candidate);

        for (Action existing : actions) {
            long existingStart = existing.startNs;
            long existingEnd = scheduledEndNs(existing);

            if (conflicts(candidateStart, candidateEnd, existingStart, existingEnd)) {
                throw new IllegalStateException("LinearSequence cannot run multiple Actions simultaneously.");
            }
        }
    }

    /**
     * Returns true when two intervals intersect.
     */
    private boolean conflicts(long startA, long endA, long startB, long endB) {
        boolean instantA = startA == endA;
        boolean instantB = startB == endB;

        if (instantA && instantB) {
            return startA == startB;
        }

        if (instantA) {
            return startA >= startB && startA < endB;
        }

        if (instantB) {
            return startB >= startA && startB < endA;
        }

        return startA < endB && startB < endA;
    }
}
