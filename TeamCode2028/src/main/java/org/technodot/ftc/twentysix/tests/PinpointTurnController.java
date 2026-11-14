package org.technodot.ftc.twentysix.tests;

/** IMU-only turn-rate feedback; independent of every motor encoder. */
final class PinpointTurnController {
    static final double SETTLE_TIMEOUT_SECONDS = 12;
    private static final double RATE_ACCELERATION = Math.toRadians(60);
    private static final double RATE_FILTER_SECONDS = 0.1;
    private static final double RATE_TOLERANCE_FRACTION = 0.15;
    private static final double REQUIRED_STABLE_SECONDS = 0.3;
    private static final double STATIC_FEEDFORWARD = 0.08;
    private static final double VELOCITY_FEEDFORWARD = 0.12;
    private static final double PROPORTIONAL_GAIN = 0.16;
    private static final double INTEGRAL_GAIN = 0.12;

    final double targetRate, maxPower;
    private double elapsed, commandedRate, filteredRate, integral;
    private double power, peakPower, stableSeconds, limitedSeconds;
    private boolean observed;

    PinpointTurnController(double targetRate, double maxPower) {
        if (!finite(targetRate) || targetRate == 0 || !finite(maxPower) || maxPower <= 0 || maxPower > 1) {
            throw new IllegalArgumentException("Invalid turn target or power limit");
        }
        this.targetRate = targetRate;
        this.maxPower = maxPower;
    }

    void update(double measuredRate, double dt) {
        if (!finite(measuredRate) || !finite(dt) || dt <= 0) {
            throw new IllegalArgumentException("Invalid turn-rate feedback");
        }
        elapsed += dt;
        if (!observed) {
            filteredRate = measuredRate;
            observed = true;
        } else filteredRate += dt / (RATE_FILTER_SECONDS + dt) * (measuredRate - filteredRate);
        commandedRate += clip(targetRate - commandedRate, -RATE_ACCELERATION * dt, RATE_ACCELERATION * dt);
        // Use the current gyro rate for motor feedback so braking responds
        // promptly if traction improves. The slower filter is for settling only.
        double error = commandedRate - measuredRate;
        double base = STATIC_FEEDFORWARD * Math.signum(commandedRate)
                + VELOCITY_FEEDFORWARD * commandedRate + PROPORTIONAL_GAIN * error;
        // The integral must cover the power the robot actually needs. Capping
        // it at 0.12 previously stranded a fast turn around 44 deg/s even with
        // a 0.7 motor-power allowance. Limit against the real actuator range.
        double nextIntegral = clip(integral + INTEGRAL_GAIN * error * dt, -maxPower, maxPower);
        double wanted = base + nextIntegral;
        double clipped = clip(wanted, -maxPower, maxPower);
        if (wanted == clipped || (wanted - clipped) * error < 0) integral = nextIntegral;
        power = clip(base + integral, -maxPower, maxPower);
        peakPower = Math.max(peakPower, Math.abs(power));
        if (Math.abs(power) >= 0.98 * maxPower) limitedSeconds += dt;
        else limitedSeconds = 0;

        // Actual angular increments still drive the fit. Filtering here avoids
        // restarting settling for every small instantaneous gyro-rate fluctuation.
        if (Math.abs(commandedRate - targetRate) < 1e-9
                && Math.abs(filteredRate - targetRate) <= RATE_TOLERANCE_FRACTION * Math.abs(targetRate)) {
            stableSeconds += dt;
        } else stableSeconds = 0;
    }

    boolean settled() { return stableSeconds >= REQUIRED_STABLE_SECONDS; }
    boolean timedOut() { return elapsed >= SETTLE_TIMEOUT_SECONDS && !settled(); }
    boolean powerLimited() { return limitedSeconds >= 0.5; }
    double power() { return power; }
    double peakPower() { return peakPower; }
    double filteredRate() { return filteredRate; }
    double elapsedSeconds() { return elapsed; }
    double stableSeconds() { return stableSeconds; }

    private static double clip(double value, double min, double max) { return Math.max(min, Math.min(max, value)); }
    private static boolean finite(double value) { return !Double.isNaN(value) && !Double.isInfinite(value); }
}
