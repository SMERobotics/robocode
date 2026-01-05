package org.technodot.ftc.twentyfivebeta.batch;

public class LinearSequence extends Sequence {

    public LinearSequence() {
        super();
    }

    public LinearSequence(long startMs) {
        super(startMs);
    }

    @Override
    public LinearSequence plan(long startMs, long durationMs, CallbackInterface callback) {
        super.plan(startMs, durationMs, callback);
        return this;
    }

    @Override
    public LinearSequence plan(long startMs, CallbackInterface callback) {
        super.plan(startMs, callback);
        return this;
    }

    @Override
    public LinearSequence plan(Action action) {
        super.plan(action);
        return this;
    }

    public LinearSequence delay(long startMs, long durationMs) {
        super.delay(startMs, durationMs);
        return this;
    }

    @Override
    protected void addAction(Action action) {
        Action candidate = requireAction(action);
        ensureNoOverlap(candidate);
        super.addActionInternal(candidate);
    }

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
