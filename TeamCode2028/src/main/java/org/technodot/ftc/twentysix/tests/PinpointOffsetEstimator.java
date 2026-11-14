package org.technodot.ftc.twentysix.tests;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

/**
 * Calibration math, deliberately independent of the FTC/Android runtime.
 *
 * Robot coordinates are forward/left/CCW. In those coordinates a forward pod at
 * lateral offset x measures dForward - x*dHeading, and a sideways pod at
 * longitudinal offset y measures dLeft + y*dHeading. Neither equation assumes
 * that a turn closes, or that translation is zero.
 *
 * FR, BL, and BR provide the independent translation reference; FL is never read
 * or used. For corrected wheel distances, forward=(BL+BR)/2 and
 * left=(FR-BR)/2 times the fitted lateral factor. This assumes a
 * symmetric mecanum drive and reasonably repeatable wheel traction. Systematic
 * slip that satisfies the mecanum constraints is unobservable with these sensors;
 * the reported interval describes repeatability, not guaranteed absolute error.
 */
final class PinpointOffsetEstimator {
    static final double FULL_TURN = 2.0 * Math.PI;
    static final double MAX_INTERVAL_MM = 1.0;
    static final double MAX_DISAGREEMENT_MM = 2.0;
    static final double MIN_INLIER_FRACTION = 0.8;
    static final int REQUIRED_TURNS = 3;
    static final int MAX_TURNS_PER_RUN = 5;
    static final int FIRST_ENCODER = 1; // FL is unusable. Keep its array slot zero.
    static final int ENCODER_COUNT = 3;
    private static final double MAX_WHEEL_RESIDUAL_MM = 1.5;
    private static final String[] WHEEL_NAMES = {"FL", "FR", "BL", "BR"};
    // Encoder signs for forward, backward, left, right, after counting polarity
    // is corrected. A motor's power direction need not identify encoder polarity.
    private static final int[][] TRANSLATION_SIGNS = {
            {1, 1, 1, 1}, {-1, -1, -1, -1}, {-1, 1, 1, -1}, {1, -1, -1, 1}
    };

    private PinpointOffsetEstimator() { }

    static int counterDelta(int current, int previous) {
        // Subtract as int BEFORE conversion to double/long, preserving wraparound.
        return current - previous;
    }

    static double headingDelta(double current, double previous) {
        // Works with both wrapped and unnormalized IMU output at normal sample rates.
        double delta = current - previous;
        return Math.atan2(Math.sin(delta), Math.cos(delta));
    }

    /** 0..3 selects a run, 4 starts validation, -1 means no useful retry remains. */
    static int nextTrainingRun(int[] collected, int[] accepted) {
        // A provisional fit is not a reason to prevent collecting the opposite
        // direction and second speed needed to judge the complete experiment.
        for (int run = 0; run < 4; run++) if (collected[run] < REQUIRED_TURNS) return run;
        if (accepted == null) return -1;
        int retry = -1;
        boolean complete = true;
        for (int run = 0; run < 4; run++) {
            if (accepted[run] >= REQUIRED_TURNS) continue;
            complete = false;
            if (collected[run] < MAX_TURNS_PER_RUN
                    && (retry < 0 || collected[run] < collected[retry])) retry = run;
        }
        return complete ? 4 : retry;
    }

    static final class Sample {
        final double[] ticks; // Unused FL=0, FR, BL, BR; learned counting polarity
        final double podForwardMm;
        final double podLeftMm;
        final double heading;
        final double seconds;
        final double cosSeconds;
        final double sinSeconds;
        // -1..-4: translation legs; 0..3: training spins; 4..5: held-out spins
        final int run;
        final int turn;

        Sample(double[] ticks, double podForwardMm, double podLeftMm,
               double heading, double seconds, double cosSeconds, double sinSeconds,
               int run, int turn) {
            if (ticks.length != 4 || seconds <= 0 || !finite(seconds)
                    || !finite(podForwardMm) || !finite(podLeftMm) || !finite(heading)
                    || !finite(cosSeconds) || !finite(sinSeconds)) {
                throw new IllegalArgumentException("Invalid calibration sample");
            }
            this.ticks = new double[4];
            for (int i = FIRST_ENCODER; i < 4; i++) {
                if (!finite(ticks[i])) throw new IllegalArgumentException("Invalid motor count");
                this.ticks[i] = ticks[i];
            }
            this.podForwardMm = podForwardMm;
            this.podLeftMm = podLeftMm;
            this.heading = heading;
            this.seconds = seconds;
            this.cosSeconds = cosSeconds;
            this.sinSeconds = sinSeconds;
            this.run = run;
            this.turn = turn;
        }
    }

    static final class Accumulator {
        final double[] ticks = new double[4];
        double forward, left, heading, seconds, cosSeconds, sinSeconds;

        void add(double[] motorTicks, double podForward, double podLeft,
                 double angle, double dt, double middleHeading) {
            for (int i = FIRST_ENCODER; i < 4; i++) ticks[i] += motorTicks[i];
            forward += podForward;
            left += podLeft;
            heading += angle;
            seconds += dt;
            cosSeconds += Math.cos(middleHeading) * dt;
            sinSeconds += Math.sin(middleHeading) * dt;
        }

        void add(Sample s) {
            for (int i = FIRST_ENCODER; i < 4; i++) ticks[i] += s.ticks[i];
            forward += s.podForwardMm;
            left += s.podLeftMm;
            heading += s.heading;
            seconds += s.seconds;
            cosSeconds += s.cosSeconds;
            sinSeconds += s.sinSeconds;
        }

        Sample take(int run, int turn) {
            Sample result = new Sample(ticks, forward, left, heading, seconds,
                    cosSeconds, sinSeconds, run, turn);
            clear();
            return result;
        }

        void clear() {
            Arrays.fill(ticks, 0);
            forward = left = heading = seconds = cosSeconds = sinSeconds = 0;
        }
    }

    static final class Result {
        final double xMm, yMm;
        final double forwardMmPerTick, leftMmPerTick;
        final double[] wheelScales;
        final int[] acceptedTurns;
        final double inlierFraction, residualMm;
        final double xIntervalMm, yIntervalMm;
        final double directionGapMm, speedGapMm, validationGapMm;
        final boolean finalAnalysis;
        final List<String> issues;

        Result(Fit fit, Diagnostics diagnostics, double[] intervals,
               double directionGap, double speedGap, double validationGap,
               boolean finalAnalysis, List<String> issues) {
            xMm = fit.coefficients[2];
            yMm = fit.coefficients[3];
            forwardMmPerTick = fit.coefficients[0];
            leftMmPerTick = fit.coefficients[1];
            wheelScales = fit.wheelScales.clone();
            acceptedTurns = diagnostics.acceptedTurns;
            inlierFraction = diagnostics.inlierFraction;
            residualMm = fit.regression.scale;
            xIntervalMm = intervals[0];
            yIntervalMm = intervals[1];
            directionGapMm = directionGap;
            speedGapMm = speedGap;
            validationGapMm = validationGap;
            this.finalAnalysis = finalAnalysis;
            this.issues = issues;
        }

        boolean validated() {
            return finalAnalysis && issues.isEmpty();
        }
    }

    static Result estimate(List<Sample> samples, boolean finalAnalysis) {
        List<Sample> training = new ArrayList<>();
        for (Sample s : samples) if (s.run < 4) training.add(s);
        Fit fit = fit(training);
        Diagnostics diagnostics = diagnose(fit);
        List<String> issues = new ArrayList<>();
        for (int run = 0; run < 4; run++) {
            if (diagnostics.acceptedTurns[run] < REQUIRED_TURNS) {
                issues.add("Run " + (run + 1) + " needs " + REQUIRED_TURNS + " accepted turns");
            }
        }
        if (diagnostics.inlierFraction < MIN_INLIER_FRACTION) {
            issues.add("Fewer than 80% of rotation measurements agree with the model");
        }

        double[] intervals = {Double.NaN, Double.NaN};
        double directionGap = Double.NaN, speedGap = Double.NaN, validationGap = Double.NaN;
        if (finalAnalysis) {
            try {
                intervals = jackknife(training);
                directionGap = compareSubsets(training, true);
                speedGap = compareSubsets(training, false);
                validationGap = validate(fit, samples);
                if (intervals[0] > MAX_INTERVAL_MM || intervals[1] > MAX_INTERVAL_MM) {
                    issues.add("95% repeatability half-width exceeds 1 mm");
                }
                if (directionGap > MAX_DISAGREEMENT_MM) issues.add("CW/CCW estimates differ by over 2 mm");
                if (speedGap > MAX_DISAGREEMENT_MM) issues.add("Slow/fast estimates differ by over 2 mm");
                if (validationGap > MAX_DISAGREEMENT_MM) issues.add("Held-out estimates differ by over 2 mm");
            } catch (IllegalArgumentException e) {
                issues.add("Validation incomplete: " + e.getMessage());
            }
        }
        return new Result(fit, diagnostics, intervals, directionGap, speedGap,
                validationGap, finalAnalysis, issues);
    }

    private static final class Fit {
        final List<Sample> samples;
        final double[] wheelScales, coefficients;
        final Regression regression;
        final double rotationMmPerRad, rotationLimitMm;

        Fit(List<Sample> samples, int[] polarity, Regression regression, Map<Integer, Integer> counts) {
            this.samples = samples;
            this.regression = regression;
            double[] p = regression.coefficients;
            for (int i = 0; i < 4; i++) {
                if (p[i] <= 0) throw new IllegalArgumentException("Inconsistent FR/BL/BR translation scales");
            }
            // qForward=a*BL+b*BR-x*dHeading;
            // qLeft=c*FR-d*BR+y*dHeading. Fitting all four coefficients with
            // offsets avoids assuming that straight calibration legs have zero yaw.
            double lateralFactor = p[3] / p[1];
            double fr = 2 * p[2] / lateralFactor, bl = 2 * p[0], br = 2 * p[1];
            double mean = (fr + bl + br) / ENCODER_COUNT;
            wheelScales = new double[]{0, polarity[1] * fr / mean,
                    polarity[2] * bl / mean, polarity[3] * br / mean};
            coefficients = new double[]{mean, lateralFactor * mean, p[4], p[5]};
            for (int i = FIRST_ENCODER; i < 4; i++) {
                if (!finite(wheelScales[i]) || Math.abs(wheelScales[i]) < 0.5 || Math.abs(wheelScales[i]) > 2) {
                    throw new IllegalArgumentException("Wheel-scale fit unstable (FL ignored; FR/BL/BR="
                            + formatCounts(wheelScales) + ")");
                }
            }
            // Three encoders have no fourth-wheel null constraint. Their angular
            // component can instead be checked against the independent IMU, using
            // a lever arm fitted on training data and frozen for validation.
            List<Equation> angular = new ArrayList<>();
            for (Sample s : samples) {
                angular.add(new Equation(new double[]{s.heading},
                        mean * driveTicks(s, wheelScales)[2], 1.0 / counts.get(s.run)));
            }
            Regression yaw = robustFit(angular, 0.1);
            rotationMmPerRad = yaw.coefficients[0];
            rotationLimitMm = Math.max(MAX_WHEEL_RESIDUAL_MM, 3.5 * yaw.scale);
            if (rotationMmPerRad <= 0) throw new IllegalArgumentException("FR/BL encoder yaw opposes the IMU");
        }

        boolean rotationConsistent(Sample s) {
            return Math.abs(coefficients[0] * driveTicks(s, wheelScales)[2]
                    - rotationMmPerRad * s.heading) <= rotationLimitMm;
        }
    }

    private static Fit fit(List<Sample> samples) {
        checkInterrupted();
        Set<Integer> translationRuns = new LinkedHashSet<>();
        Map<Integer, Integer> driftColumns = new LinkedHashMap<>();
        Map<Integer, Integer> counts = new LinkedHashMap<>();
        for (Sample s : samples) {
            counts.put(s.run, counts.containsKey(s.run) ? counts.get(s.run) + 1 : 1);
            if (s.run < 0) translationRuns.add(s.run);
            else if (!driftColumns.containsKey(s.run)) driftColumns.put(s.run, 6 + 2 * driftColumns.size());
        }
        if (translationRuns.size() < 3 || driftColumns.isEmpty()) {
            throw new IllegalArgumentException("Need forward, sideways, and rotation measurements");
        }
        int[] polarity = encoderPolarities(samples);
        int columns = 6 + 2 * driftColumns.size();
        List<Equation> equations = new ArrayList<>();
        for (Sample s : samples) {
            double[] x = new double[columns], y = new double[columns];
            x[0] = polarity[2] * s.ticks[2];
            x[1] = polarity[3] * s.ticks[3];
            y[2] = polarity[1] * s.ticks[1];
            y[3] = -polarity[3] * s.ticks[3];
            x[4] = -s.heading;
            y[5] = s.heading;
            if (s.run >= 0) {
                int col = driftColumns.get(s.run);
                x[col] = s.cosSeconds;
                x[col + 1] = s.sinSeconds;
                y[col] = -s.sinSeconds;
                y[col + 1] = s.cosSeconds;
            }
            // Equal total influence per movement keeps repeated spins from
            // overwhelming the straight/sideways scale-calibration excitation.
            double weight = 1.0 / counts.get(s.run);
            equations.add(new Equation(x, s.podForwardMm, weight));
            equations.add(new Equation(y, s.podLeftMm, weight));
        }
        Regression regression = robustFit(equations, 0.08);
        if (Math.abs(regression.coefficients[4]) > 1000 || Math.abs(regression.coefficients[5]) > 1000) {
            throw new IllegalArgumentException("Estimated offsets exceed 1 meter; check pod and motor configuration");
        }
        return new Fit(samples, polarity, regression, counts);
    }

    /** Learn counting polarity from completed movements; never change motor direction. */
    static int[] encoderPolarities(double[] ticks, int translationLeg) {
        if (ticks.length != 4 || translationLeg < 0 || translationLeg >= 4) {
            throw new IllegalArgumentException("Invalid encoder response measurement");
        }
        double[] magnitudes = new double[ENCODER_COUNT];
        for (int i = FIRST_ENCODER; i < 4; i++) {
            if (!finite(ticks[i])) throw new IllegalArgumentException("Non-finite " + WHEEL_NAMES[i] + " encoder response");
            magnitudes[i - FIRST_ENCODER] = Math.abs(ticks[i]);
        }
        double minimum = Math.max(5, 0.05 * median(magnitudes));
        int[] polarity = new int[4];
        for (int i = FIRST_ENCODER; i < 4; i++) {
            if (magnitudes[i - FIRST_ENCODER] < minimum) {
                throw new IllegalArgumentException(WHEEL_NAMES[i] + " encoder has insufficient movement ("
                        + formatCounts(ticks) + " ticks; FL ignored); check that motor's encoder connection");
            }
            polarity[i] = ticks[i] * TRANSLATION_SIGNS[translationLeg][i] > 0 ? 1 : -1;
        }
        return polarity;
    }

    private static int[] encoderPolarities(List<Sample> samples) {
        double[][] translations = new double[4][4];
        boolean[] present = new boolean[4];
        for (Sample s : samples) {
            if (s.run >= 0) continue;
            int leg = -1 - s.run;
            if (leg >= 4) throw new IllegalArgumentException("Invalid translation leg");
            present[leg] = true;
            for (int i = FIRST_ENCODER; i < 4; i++) translations[leg][i] += s.ticks[i];
        }
        int[] result = null;
        for (int leg = 0; leg < 4; leg++) {
            if (!present[leg]) continue;
            int[] response = encoderPolarities(translations[leg], leg);
            if (result == null) result = response;
            else checkEncoderPolarities(result, response);
        }
        if (result == null) throw new IllegalArgumentException("Need straight-movement encoder measurements");
        return result;
    }

    static void checkEncoderPolarities(int[] expected, int[] measured) {
        for (int i = FIRST_ENCODER; i < 4; i++) {
            if (expected[i] != measured[i]) {
                throw new IllegalArgumentException(WHEEL_NAMES[i] + " encoder response changes between movement directions; "
                        + "check motor names, wheel arrangement, or an intermittent encoder");
            }
        }
    }

    static String formatCounts(double[] values) {
        return String.format(Locale.US, "[FL ignored; FR %.3f, BL %.3f, BR %.3f]", values[1], values[2], values[3]);
    }

    private static double[] driveTicks(Sample s, double[] gain) {
        double fr = gain[1] * s.ticks[1];
        double bl = gain[2] * s.ticks[2], br = gain[3] * s.ticks[3];
        return new double[]{(bl + br) / 2, (fr - br) / 2, (fr - bl) / 2};
    }

    private static final class Diagnostics {
        final int[] acceptedTurns = new int[4];
        double inlierFraction;
    }

    private static Diagnostics diagnose(Fit fit) {
        Diagnostics result = new Diagnostics();
        Map<Integer, double[]> turns = new LinkedHashMap<>();
        double totalAngle = 0, acceptedAngle = 0;
        double threshold = Math.max(0.5, 3.5 * fit.regression.scale);
        for (int i = 0; i < fit.samples.size(); i++) {
            Sample s = fit.samples.get(i);
            if (s.run < 0) continue;
            double angle = Math.abs(s.heading);
            boolean accepted = Math.abs(fit.regression.residuals[2 * i]) <= threshold
                    && Math.abs(fit.regression.residuals[2 * i + 1]) <= threshold
                    && fit.rotationConsistent(s);
            double[] turn = turns.get(s.turn);
            if (turn == null) {
                turn = new double[]{s.run, 0, 0};
                turns.put(s.turn, turn);
            }
            turn[1] += angle;
            totalAngle += angle;
            if (accepted) {
                turn[2] += angle;
                acceptedAngle += angle;
            }
        }
        for (double[] turn : turns.values()) {
            if (turn[1] >= 0.98 * FULL_TURN && turn[2] / turn[1] >= MIN_INLIER_FRACTION) {
                result.acceptedTurns[(int) turn[0]]++;
            }
        }
        result.inlierFraction = totalAngle > 0 ? acceptedAngle / totalAngle : 0;
        return result;
    }

    private static double[] jackknife(List<Sample> samples) {
        Set<Integer> groups = new LinkedHashSet<>();
        for (Sample s : samples) groups.add(s.run < 0 ? s.run : s.turn);
        if (groups.size() < 10) throw new IllegalArgumentException("Too few independent measurement groups");
        double[][] estimates = new double[groups.size()][2];
        int index = 0;
        for (int group : groups) {
            checkInterrupted();
            List<Sample> subset = new ArrayList<>();
            for (Sample s : samples) if ((s.run < 0 ? s.run : s.turn) != group) subset.add(s);
            Fit leaveOneOut = fit(subset);
            estimates[index++] = new double[]{leaveOneOut.coefficients[2], leaveOneOut.coefficients[3]};
        }
        double[] intervals = new double[2];
        for (int axis = 0; axis < 2; axis++) {
            double mean = 0, sum = 0;
            for (double[] estimate : estimates) mean += estimate[axis] / estimates.length;
            for (double[] estimate : estimates) sum += square(estimate[axis] - mean);
            // Conservative Student-t factor for >=10 groups. Refit wheel gains
            // and distance scales too, so their uncertainty is not treated as zero.
            intervals[axis] = 2.3 * Math.sqrt((estimates.length - 1.0) / estimates.length * sum);
        }
        return intervals;
    }

    private static double compareSubsets(List<Sample> samples, boolean byDirection) {
        List<Sample> a = new ArrayList<>(), b = new ArrayList<>();
        for (Sample s : samples) {
            if (s.run < 0) {
                a.add(s);
                b.add(s);
            } else if (byDirection ? (s.run == 0 || s.run == 3) : s.run < 2) a.add(s);
            else b.add(s);
        }
        Fit first = fit(a), second = fit(b);
        return Math.max(Math.abs(first.coefficients[2] - second.coefficients[2]),
                Math.abs(first.coefficients[3] - second.coefficients[3]));
    }

    private static double validate(Fit training, List<Sample> samples) {
        double gap = 0;
        for (int run = 4; run <= 5; run++) {
            List<Equation> equations = new ArrayList<>();
            List<Sample> heldOut = new ArrayList<>();
            double angle = 0;
            for (Sample s : samples) {
                if (s.run != run) continue;
                heldOut.add(s);
                double[] motion = driveTicks(s, training.wheelScales);
                equations.add(new Equation(new double[]{-s.heading, 0, s.cosSeconds, s.sinSeconds},
                        s.podForwardMm - training.coefficients[0] * motion[0], 1));
                equations.add(new Equation(new double[]{0, s.heading, -s.sinSeconds, s.cosSeconds},
                        s.podLeftMm - training.coefficients[1] * motion[1], 1));
                angle += Math.abs(s.heading);
            }
            if (angle < 0.98 * FULL_TURN) throw new IllegalArgumentException("Missing held-out revolution");
            Regression fit = robustFit(equations, 0.08);
            double threshold = Math.max(0.5, 3.5 * fit.scale);
            double acceptedAngle = 0;
            for (int i = 0; i < equations.size(); i += 2) {
                Sample s = heldOut.get(i / 2);
                if (Math.abs(fit.residuals[i]) <= threshold && Math.abs(fit.residuals[i + 1]) <= threshold
                        && training.rotationConsistent(s)) {
                    acceptedAngle += Math.abs(s.heading);
                }
            }
            if (acceptedAngle / angle < MIN_INLIER_FRACTION) {
                throw new IllegalArgumentException("Too many held-out slip outliers");
            }
            gap = Math.max(gap, Math.max(Math.abs(fit.coefficients[0] - training.coefficients[2]),
                    Math.abs(fit.coefficients[1] - training.coefficients[3])));
        }
        return gap;
    }

    private static final class Equation {
        final double[] x;
        final double y, weight;

        Equation(double[] x, double y, double weight) {
            this.x = x;
            this.y = y;
            this.weight = weight;
        }
    }

    private static final class Regression {
        final double[] coefficients, residuals;
        final double scale;

        Regression(double[] coefficients, double[] residuals, double scale) {
            this.coefficients = coefficients;
            this.residuals = residuals;
            this.scale = scale;
        }
    }

    private static Regression robustFit(List<Equation> equations, double noiseFloor) {
        if (equations.isEmpty() || equations.size() < equations.get(0).x.length) {
            throw new IllegalArgumentException("Insufficient independent measurements");
        }
        double[] weights = new double[equations.size()], residuals = new double[equations.size()];
        for (int i = 0; i < weights.length; i++) weights[i] = equations.get(i).weight;
        double[] balancingWeights = weights.clone();
        double[] coefficients = null;
        double scale = noiseFloor;
        for (int iteration = 0; iteration < 10; iteration++) {
            checkInterrupted();
            coefficients = leastSquares(equations, weights);
            double[] standardized = new double[weights.length];
            for (int i = 0; i < weights.length; i++) {
                Equation e = equations.get(i);
                residuals[i] = e.y - dot(e.x, coefficients);
                // These weights balance movements; they are not noise precision.
                // Keep the robust noise estimate in millimeters for every phase.
                standardized[i] = residuals[i];
            }
            double center = weightedMedian(standardized, balancingWeights);
            double[] deviations = new double[weights.length];
            for (int i = 0; i < weights.length; i++) deviations[i] = Math.abs(standardized[i] - center);
            scale = Math.max(noiseFloor, 1.4826 * weightedMedian(deviations, balancingWeights));
            for (int i = 0; i < weights.length; i++) {
                weights[i] = equations.get(i).weight
                        * Math.min(1, 1.345 * scale / Math.max(1e-12, Math.abs(standardized[i])));
            }
        }
        return new Regression(coefficients, residuals, scale);
    }

    /** Column-scaled, pivoted Householder QR; normal equations magnify degeneracy. */
    private static double[] leastSquares(List<Equation> equations, double[] weights) {
        int rows = equations.size(), columns = equations.get(0).x.length;
        double[][] a = new double[rows][columns];
        double[] b = new double[rows], scales = new double[columns];
        int[] permutation = new int[columns];
        for (int j = 0; j < columns; j++) {
            permutation[j] = j;
            for (int i = 0; i < rows; i++) scales[j] = Math.hypot(scales[j], equations.get(i).x[j] * Math.sqrt(weights[i]));
            if (scales[j] < 1e-10) throw new IllegalArgumentException("Unexcited calibration parameter");
        }
        for (int i = 0; i < rows; i++) {
            double weight = Math.sqrt(weights[i]);
            for (int j = 0; j < columns; j++) a[i][j] = equations.get(i).x[j] * weight / scales[j];
            b[i] = equations.get(i).y * weight;
        }
        double maxDiagonal = 0, minDiagonal = Double.POSITIVE_INFINITY;
        for (int k = 0; k < columns; k++) {
            int pivot = k;
            double largest = -1;
            for (int j = k; j < columns; j++) {
                double norm = 0;
                for (int i = k; i < rows; i++) norm = Math.hypot(norm, a[i][j]);
                if (norm > largest) { largest = norm; pivot = j; }
            }
            if (largest < 1e-7) throw new IllegalArgumentException("Ill-conditioned calibration; need independent motion");
            for (int i = 0; i < rows; i++) {
                double temp = a[i][k]; a[i][k] = a[i][pivot]; a[i][pivot] = temp;
            }
            int temp = permutation[k]; permutation[k] = permutation[pivot]; permutation[pivot] = temp;
            double alpha = -Math.copySign(largest, a[k][k]);
            double[] v = new double[rows - k];
            double norm = 0;
            for (int i = k; i < rows; i++) {
                v[i - k] = a[i][k] - (i == k ? alpha : 0);
                norm = Math.hypot(norm, v[i - k]);
            }
            for (int i = 0; i < v.length; i++) v[i] /= norm;
            for (int j = k; j < columns; j++) {
                double projection = 0;
                for (int i = k; i < rows; i++) projection += v[i - k] * a[i][j];
                for (int i = k; i < rows; i++) a[i][j] -= 2 * v[i - k] * projection;
            }
            double projection = 0;
            for (int i = k; i < rows; i++) projection += v[i - k] * b[i];
            for (int i = k; i < rows; i++) b[i] -= 2 * v[i - k] * projection;
            maxDiagonal = Math.max(maxDiagonal, Math.abs(alpha));
            minDiagonal = Math.min(minDiagonal, Math.abs(alpha));
        }
        if (maxDiagonal / minDiagonal > 1e7) throw new IllegalArgumentException("Ill-conditioned calibration");
        double[] z = new double[columns], result = new double[columns];
        for (int i = columns - 1; i >= 0; i--) {
            double value = b[i];
            for (int j = i + 1; j < columns; j++) value -= a[i][j] * z[j];
            z[i] = value / a[i][i];
            result[permutation[i]] = z[i] / scales[permutation[i]];
            if (!finite(result[permutation[i]])) throw new IllegalArgumentException("Non-finite calibration result");
        }
        return result;
    }

    private static double median(double[] values) {
        double[] sorted = values.clone();
        Arrays.sort(sorted);
        int middle = sorted.length / 2;
        return sorted.length % 2 == 0 ? (sorted[middle - 1] + sorted[middle]) / 2 : sorted[middle];
    }

    private static double weightedMedian(double[] values, double[] weights) {
        Integer[] order = new Integer[values.length];
        double total = 0;
        for (int i = 0; i < order.length; i++) { order[i] = i; total += weights[i]; }
        Arrays.sort(order, (a, b) -> Double.compare(values[a], values[b]));
        double cumulative = 0;
        for (int index : order) {
            cumulative += weights[index];
            if (cumulative >= total / 2) return values[index];
        }
        return values[order[order.length - 1]];
    }

    private static double dot(double[] a, double[] b) {
        double sum = 0;
        for (int i = 0; i < a.length; i++) sum += a[i] * b[i];
        return sum;
    }

    private static double square(double value) { return value * value; }

    static boolean finite(double value) { return !Double.isNaN(value) && !Double.isInfinite(value); }

    private static void checkInterrupted() {
        if (Thread.currentThread().isInterrupted()) throw new IllegalArgumentException("Analysis cancelled");
    }
}
