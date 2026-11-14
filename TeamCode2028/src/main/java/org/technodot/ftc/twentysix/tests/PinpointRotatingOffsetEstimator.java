package org.technodot.ftc.twentysix.tests;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

/**
 * Learns encoder-to-ground scales under the same rotational load as the offset
 * measurements. Each direction/speed has its own wheel model. Independent
 * forward/sideways excitation is essential: pure turning cannot identify these
 * scales separately from pod offsets. Straight moves learn counting signs only.
 */
final class PinpointRotatingOffsetEstimator {
    private PinpointRotatingOffsetEstimator() { }

    private static final class RunFit {
        final List<PinpointOffsetEstimator.Sample> samples;
        final PinpointOffsetEstimator.Regression regression;
        final double[] gains;
        final double forward, left, x, y, lever;

        RunFit(List<PinpointOffsetEstimator.Sample> samples, int[] polarity) {
            this.samples = samples;
            List<PinpointOffsetEstimator.Equation> equations = new ArrayList<>();
            for (PinpointOffsetEstimator.Sample s : samples) {
                equations.add(new PinpointOffsetEstimator.Equation(new double[]{polarity[2] * s.ticks[2],
                        polarity[3] * s.ticks[3], 0, 0, -s.heading, 0, s.cosSeconds, s.sinSeconds}, s.podForwardMm, 1));
                equations.add(new PinpointOffsetEstimator.Equation(new double[]{0, 0, polarity[1] * s.ticks[1],
                        -polarity[3] * s.ticks[3], 0, s.heading, -s.sinSeconds, s.cosSeconds}, s.podLeftMm, 1));
            }
            regression = PinpointOffsetEstimator.robustFit(equations, .08);
            checkExcitation();
            double[] p = regression.coefficients;
            for (int i = 0; i < 4; i++) if (p[i] <= 0) {
                throw new IllegalArgumentException("Insufficient independent translation during turning, or inconsistent working encoders");
            }
            double lateral = p[3] / p[1];
            double fr = 2 * p[2] / lateral, bl = 2 * p[0], br = 2 * p[1];
            forward = (fr + bl + br) / 3;
            left = lateral * forward;
            gains = new double[]{0, polarity[1] * fr / forward, polarity[2] * bl / forward, polarity[3] * br / forward};
            for (int i = 1; i < 4; i++) if (Math.abs(gains[i]) < .5 || Math.abs(gains[i]) > 2) {
                throw new IllegalArgumentException("Turning wheel-scale fit unstable; need independent forward/sideways motion");
            }
            x = p[4]; y = p[5];
            if (Math.abs(x) > 1000 || Math.abs(y) > 1000) throw new IllegalArgumentException("Offsets exceed 1 meter");
            List<PinpointOffsetEstimator.Equation> yaw = new ArrayList<>();
            for (PinpointOffsetEstimator.Sample s : samples) yaw.add(new PinpointOffsetEstimator.Equation(
                    new double[]{s.heading}, yawDistance(s), 1));
            lever = PinpointOffsetEstimator.robustFit(yaw, .1).coefficients[0];
            if (lever <= 0) throw new IllegalArgumentException("Working encoder yaw opposes IMU");
        }

        private void checkExcitation() {
            List<PinpointOffsetEstimator.Equation> f = new ArrayList<>(), l = new ArrayList<>();
            for (PinpointOffsetEstimator.Sample s : samples) {
                double[] nuisance = {s.heading, s.cosSeconds, s.sinSeconds};
                f.add(new PinpointOffsetEstimator.Equation(nuisance, s.podForwardMm, 1));
                l.add(new PinpointOffsetEstimator.Equation(nuisance, s.podLeftMm, 1));
            }
            double[] u = PinpointOffsetEstimator.robustFit(f, .08).residuals;
            double[] v = PinpointOffsetEstimator.robustFit(l, .08).residuals;
            double threshold = Math.max(.5, 3.5 * regression.scale);
            double su = 0, sv = 0, uu = 0, vv = 0, uv = 0;
            int count = 0;
            for (int i = 0; i < samples.size(); i++) {
                if (Math.abs(regression.residuals[2 * i]) > threshold || Math.abs(regression.residuals[2 * i + 1]) > threshold) continue;
                su += u[i]; sv += v[i]; uu += u[i] * u[i]; vv += v[i] * v[i]; uv += u[i] * v[i]; count++;
            }
            if (count < 10) throw new IllegalArgumentException("Too few consistent translation measurements during turns");
            uu = uu / count - Math.pow(su / count, 2);
            vv = vv / count - Math.pow(sv / count, 2);
            uv = uv / count - su * sv / (count * (double) count);
            double minimumRms = Math.sqrt(Math.max(0, (uu + vv - Math.hypot(uu - vv, 2 * uv)) / 2));
            if (minimumRms < Math.max(.3, 3 * regression.scale)) {
                throw new IllegalArgumentException("Insufficient independent forward/sideways excitation above sensor noise during turns");
            }
        }

        double yawDistance(PinpointOffsetEstimator.Sample s) {
            return forward * (gains[1] * s.ticks[1] - gains[2] * s.ticks[2]) / 2;
        }

        boolean accepted(int index, double referenceLever) {
            PinpointOffsetEstimator.Sample s = samples.get(index);
            double threshold = Math.max(.5, 3.5 * regression.scale);
            return Math.abs(regression.residuals[2 * index]) <= threshold
                    && Math.abs(regression.residuals[2 * index + 1]) <= threshold
                    && Math.abs(yawDistance(s) - referenceLever * s.heading) <= 1.5;
        }
    }

    static PinpointOffsetEstimator.Result estimate(List<PinpointOffsetEstimator.Sample> samples, boolean finalAnalysis) {
        int[] polarity = PinpointOffsetEstimator.encoderPolarities(samples);
        RunFit[] fits = new RunFit[6];
        List<String> issues = new ArrayList<>();
        double[][] offsets = new double[6][2];
        for (double[] xy : offsets) Arrays.fill(xy, Double.NaN);
        for (int run = 0; run < (finalAnalysis ? 6 : 4); run++) {
            List<PinpointOffsetEstimator.Sample> phase = new ArrayList<>();
            for (PinpointOffsetEstimator.Sample s : samples) if (s.run == run) phase.add(s);
            if (phase.isEmpty()) continue;
            try {
                fits[run] = new RunFit(phase, polarity);
                offsets[run] = new double[]{fits[run].x, fits[run].y};
            } catch (IllegalArgumentException e) { issues.add("Run " + (run + 1) + ": " + e.getMessage()); }
        }
        double[] coefficients = new double[4], gains = new double[4];
        int phases = 0;
        double lever = 0, noise = 0;
        for (int run = 0; run < 4; run++) if (fits[run] != null) {
            RunFit fit = fits[run]; phases++;
            coefficients[0] += fit.forward; coefficients[1] += fit.left;
            coefficients[2] += fit.x; coefficients[3] += fit.y;
            for (int i = 1; i < 4; i++) gains[i] += fit.gains[i];
            lever += fit.lever; noise += fit.regression.scale;
        }
        if (phases == 0) throw new IllegalArgumentException("No identifiable turning data: " + String.join("; ", issues));
        for (int i = 0; i < 4; i++) { coefficients[i] /= phases; gains[i] /= phases; }
        lever /= phases; noise /= phases;

        int[] acceptedTurns = new int[4];
        double totalAngle = 0, acceptedAngle = 0;
        for (int run = 0; run < 4; run++) {
            RunFit fit = fits[run];
            if (fit != null) {
                Set<Integer> turns = new LinkedHashSet<>();
                for (PinpointOffsetEstimator.Sample s : fit.samples) turns.add(s.turn);
                for (int turn : turns) {
                    double angle = 0, good = 0;
                    for (int n = 0; n < fit.samples.size(); n++) {
                        PinpointOffsetEstimator.Sample s = fit.samples.get(n);
                        if (s.turn != turn) continue;
                        angle += Math.abs(s.heading);
                        if (fit.accepted(n, lever)) good += Math.abs(s.heading);
                    }
                    totalAngle += angle; acceptedAngle += good;
                    if (angle >= .98 * PinpointOffsetEstimator.FULL_TURN && good / angle >= .8) acceptedTurns[run]++;
                }
            }
            if (acceptedTurns[run] < 3) issues.add("Run " + (run + 1) + " needs 3 accepted turns");
        }
        double fraction = totalAngle > 0 ? acceptedAngle / totalAngle : 0;
        if (fraction < .8) issues.add("Fewer than 80% of rotation measurements agree with the model");
        double[] intervals = {Double.NaN, Double.NaN}, shift = {Double.NaN, Double.NaN};
        double[] gaps = {Double.NaN, Double.NaN, Double.NaN};
        if (finalAnalysis && phases == 4) {
            try {
                List<double[]> leaveOneOut = new ArrayList<>();
                Arrays.fill(shift, 0);
                for (int run = 0; run < 4; run++) {
                    Set<Integer> turns = new LinkedHashSet<>();
                    for (PinpointOffsetEstimator.Sample s : fits[run].samples) turns.add(s.turn);
                    for (int turn : turns) {
                        List<PinpointOffsetEstimator.Sample> subset = new ArrayList<>();
                        for (PinpointOffsetEstimator.Sample s : fits[run].samples) if (s.turn != turn) subset.add(s);
                        RunFit fold = new RunFit(subset, polarity);
                        double[] xy = {coefficients[2] + (fold.x - fits[run].x) / 4, coefficients[3] + (fold.y - fits[run].y) / 4};
                        for (int axis = 0; axis < 2; axis++) shift[axis] = Math.max(shift[axis], Math.abs(xy[axis] - coefficients[axis + 2]));
                        leaveOneOut.add(xy);
                    }
                }
                if (leaveOneOut.size() < 10) throw new IllegalArgumentException("Too few independent turns");
                for (int axis = 0; axis < 2; axis++) {
                    double mean = 0, sum = 0;
                    for (double[] xy : leaveOneOut) mean += xy[axis] / leaveOneOut.size();
                    for (double[] xy : leaveOneOut) sum += Math.pow(xy[axis] - mean, 2);
                    intervals[axis] = 2.3 * Math.sqrt((leaveOneOut.size() - 1.0) / leaveOneOut.size() * sum);
                }
            } catch (IllegalArgumentException e) { issues.add("Repeatability incomplete: " + e.getMessage()); }
            gaps[0] = gap(fits[0], fits[3], fits[1], fits[2]);
            gaps[1] = gap(fits[0], fits[1], fits[2], fits[3]);
            double minX = Double.POSITIVE_INFINITY, maxX = Double.NEGATIVE_INFINITY;
            double minY = Double.POSITIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY;
            for (int run = 0; run < 4; run++) {
                minX = Math.min(minX, fits[run].x); maxX = Math.max(maxX, fits[run].x);
                minY = Math.min(minY, fits[run].y); maxY = Math.max(maxY, fits[run].y);
            }
            // Direction/speed averages can both cancel an interaction between
            // the two. Independent phase offsets must agree as well.
            if (Math.max(maxX - minX, maxY - minY) > 2) issues.add("Independent turning phases differ by over 2 mm");
            gaps[2] = 0;
            for (int run = 4; run < 6; run++) {
                RunFit fit = fits[run];
                if (fit == null) { issues.add("Missing identifiable held-out run " + (run + 1)); gaps[2] = Double.NaN; continue; }
                double angle = 0, good = 0;
                for (int n = 0; n < fit.samples.size(); n++) {
                    double a = Math.abs(fit.samples.get(n).heading); angle += a;
                    if (fit.accepted(n, lever)) good += a;
                }
                if (angle < .98 * PinpointOffsetEstimator.FULL_TURN || good / angle < .8) issues.add("Held-out slip/coverage check failed in run " + (run + 1));
                gaps[2] = Math.max(gaps[2], Math.max(Math.abs(fit.x - coefficients[2]), Math.abs(fit.y - coefficients[3])));
            }
            if (intervals[0] > 1 || intervals[1] > 1) issues.add("95% repeatability half-width exceeds 1 mm");
            if (gaps[0] > 2) issues.add("CW/CCW estimates differ by over 2 mm");
            if (gaps[1] > 2) issues.add("Slow/fast estimates differ by over 2 mm");
            if (gaps[2] > 2) issues.add("Held-out estimates differ by over 2 mm");
        }
        return new PinpointOffsetEstimator.Result(coefficients, gains, acceptedTurns, fraction, noise, intervals, gaps,
                finalAnalysis, issues, new double[]{Double.NaN, Double.NaN}, shift, offsets, Double.NaN, true);
    }

    private static double gap(RunFit a, RunFit b, RunFit c, RunFit d) {
        return Math.max(Math.abs((a.x + b.x - c.x - d.x) / 2), Math.abs((a.y + b.y - c.y - d.y) / 2));
    }
}
