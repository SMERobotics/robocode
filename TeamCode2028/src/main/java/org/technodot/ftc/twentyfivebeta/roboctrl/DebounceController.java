package org.technodot.ftc.twentyfivebeta.roboctrl;

/**
 * DebounceController helps to determine if a system has been within a specified tolerance
 * of a setpoint for a certain duration (debounce time) before confirming stability.
 */
public class DebounceController {
    public double setpoint;
    public long lastErrorTimeNs;

    public final double tolerance;
    public final long debounceTimeNs; // 67ms debounce time default

    public DebounceController(double tolerance) {
        this.tolerance = tolerance;
        this.debounceTimeNs = 67_000_000L;
        this.lastErrorTimeNs = System.nanoTime();
    }

    public DebounceController(double tolerance, long debounceTimeNs) {
        this.tolerance = tolerance;
        this.debounceTimeNs = debounceTimeNs;
        this.lastErrorTimeNs = System.nanoTime();
    }

    public boolean update(double current) {
        double error = Math.abs(setpoint - current);
        long currentTimeNs = System.nanoTime();

        if (error > tolerance) {
            lastErrorTimeNs = currentTimeNs;
            return false;
        } else {
            return (currentTimeNs - lastErrorTimeNs) >= debounceTimeNs;
        }
    }

    public boolean update(double current, long currentNs) {
        double error = Math.abs(setpoint - current);

        if (error > tolerance) {
            lastErrorTimeNs = currentNs;
            return false;
        } else {
            return (currentNs - lastErrorTimeNs) >= debounceTimeNs;
        }
    }
}
