package org.technodot.ftc.twentysix.tests;

/**
 * Holds a fixed point near the calibration's starting position during turns.
 * Pod/IMU feedback detects real drift even when drive wheels slip. The seed
 * offsets define which rigid point is held; they are never calibration priors.
 * With imperfect seeds the drive center can trace a small orbit, which the
 * independent drive-encoder regression still measures as translation.
 */
final class PinpointPositionHold {
    private static final double POSITION_GAIN = 0.7;
    private static final double MAX_SPEED_MM_PER_SECOND = 70;
    private static final double MAX_CORRECTION_POWER = 0.28;
    private static final double VELOCITY_FILTER_SECONDS = 0.15;
    private static final double VELOCITY_FEEDFORWARD = 0.001;
    private static final double VELOCITY_GAIN = 0.0015;
    private static final double INTEGRAL_GAIN = 0.0015;
    private static final double POWER_SLEW_PER_SECOND = 0.5;

    private final double seedXOffset, seedYOffset;
    private double x, y, velocityX, velocityY;
    // Integrate velocity error in the fixed frame, not a rotating robot frame.
    private double integralX, integralY, powerX, powerY;
    private double forwardPower, leftPower;

    PinpointPositionHold(double seedXOffset, double seedYOffset) {
        if (!finite(seedXOffset) || !finite(seedYOffset)) {
            throw new IllegalArgumentException("Invalid position-hold seed offsets");
        }
        this.seedXOffset = seedXOffset;
        this.seedYOffset = seedYOffset;
    }

    /** Observe all motion, including ramps and braking; never re-zero between turns. */
    void observe(double podForward, double podLeft, double angle, double middleHeading, double dt) {
        if (!finite(podForward) || !finite(podLeft) || !finite(angle)
                || !finite(middleHeading) || !finite(dt) || dt <= 0) {
            throw new IllegalArgumentException("Invalid position-hold observation");
        }
        double forward = podForward + seedXOffset * angle;
        double left = podLeft - seedYOffset * angle;
        double cos = Math.cos(middleHeading), sin = Math.sin(middleHeading);
        double dx = cos * forward - sin * left;
        double dy = sin * forward + cos * left;
        x += dx;
        y += dy;
        double alpha = dt / (VELOCITY_FILTER_SECONDS + dt);
        velocityX += alpha * (dx / dt - velocityX);
        velocityY += alpha * (dy / dt - velocityY);
    }

    /**
     * Returns body-frame translation power. availablePower reserves the yaw
     * controller's wheel power first, so correcting position cannot scale down
     * the requested rotation. No drive encoder, including FL, is used here.
     */
    void control(double heading, double dt, double availablePower) {
        if (!finite(heading) || !finite(dt) || dt <= 0
                || !finite(availablePower) || availablePower < 0 || availablePower > 1) {
            throw new IllegalArgumentException("Invalid position-hold command");
        }
        double desiredX = -POSITION_GAIN * x, desiredY = -POSITION_GAIN * y;
        double speedScale = Math.max(1, Math.hypot(desiredX, desiredY) / MAX_SPEED_MM_PER_SECOND);
        desiredX /= speedScale;
        desiredY /= speedScale;
        double errorX = desiredX - velocityX, errorY = desiredY - velocityY;
        double nextIntegralX = integralX + INTEGRAL_GAIN * errorX * dt;
        double nextIntegralY = integralY + INTEGRAL_GAIN * errorY * dt;
        double baseX = VELOCITY_FEEDFORWARD * desiredX + VELOCITY_GAIN * errorX;
        double baseY = VELOCITY_FEEDFORWARD * desiredY + VELOCITY_GAIN * errorY;
        double wantedX = baseX + nextIntegralX, wantedY = baseY + nextIntegralY;
        double cos = Math.cos(heading), sin = Math.sin(heading);
        double limit = Math.min(MAX_CORRECTION_POWER, availablePower);
        // Conditional integration prevents windup when either the correction
        // cap or reserved rotation power prevents the requested translation.
        if (wheelPower(wantedX, wantedY, cos, sin) <= limit
                || wantedX * errorX + wantedY * errorY < 0) {
            integralX = nextIntegralX;
            integralY = nextIntegralY;
        }
        wantedX = baseX + integralX;
        wantedY = baseY + integralY;
        double scale = powerScale(wantedX, wantedY, cos, sin, limit);
        wantedX *= scale;
        wantedY *= scale;
        double deltaX = wantedX - powerX, deltaY = wantedY - powerY;
        double slew = Math.max(1, Math.hypot(deltaX, deltaY) / (POWER_SLEW_PER_SECOND * dt));
        powerX += deltaX / slew;
        powerY += deltaY / slew;
        // A changing heading/power budget can tighten the wheel-space cap even
        // when the field-frame correction has barely changed.
        scale = powerScale(powerX, powerY, cos, sin, limit);
        powerX *= scale;
        powerY *= scale;
        forwardPower = cos * powerX + sin * powerY;
        leftPower = -sin * powerX + cos * powerY;
    }

    /** Reset only controller output when braking; retain displacement from the start. */
    void pause() {
        integralX = integralY = powerX = powerY = forwardPower = leftPower = 0;
    }

    double xMm() { return x; }
    double yMm() { return y; }
    double distanceMm() { return Math.hypot(x, y); }
    double forwardPower() { return forwardPower; }
    double leftPower() { return leftPower; }

    private static double wheelPower(double px, double py, double cos, double sin) {
        return Math.abs(cos * px + sin * py) + Math.abs(-sin * px + cos * py);
    }

    private static double powerScale(double px, double py, double cos, double sin, double limit) {
        double required = wheelPower(px, py, cos, sin);
        return required > limit ? limit / required : 1;
    }

    private static boolean finite(double value) { return !Double.isNaN(value) && !Double.isInfinite(value); }
}
