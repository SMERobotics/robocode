package com.technodot.ftc.twentyfive.batch;

/**
 * Lightweight context shared with actions while a {@link Batch} is running.
 */
public final class BatchRuntime {

    private final Batch batch;
    private final long nowMillis;
    private final long batchElapsedMillis;

    BatchRuntime(Batch batch, long nowMillis, long batchElapsedMillis) {
        this.batch = batch;
        this.nowMillis = nowMillis;
        this.batchElapsedMillis = batchElapsedMillis;
    }

    /**
     * @return the batch driving this runtime
     */
    public Batch getBatch() {
        return batch;
    }

    /**
     * @return absolute wall time (ms) when this runtime snapshot was created
     */
    public long getNowMillis() {
        return nowMillis;
    }

    /**
     * @return how long (ms) the batch has been running
     */
    public long getBatchElapsedMillis() {
        return batchElapsedMillis;
    }
}
