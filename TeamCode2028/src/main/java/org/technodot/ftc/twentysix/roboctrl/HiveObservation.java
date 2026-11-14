package org.technodot.ftc.twentysix.roboctrl;

import org.technodot.ftc.twentysix.common.Alliance;
import org.technodot.ftc.twentysix.common.Hive;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** An immutable measurement at image capture time, not an extrapolated aim command. */
public final class HiveObservation {
    public final Alliance alliance;
    public final boolean valid;
    public final String reason;
    public final HiveGeometry.Cell cell;
    /** Null when invalid. Missing vision never means TIPPING. */
    public final Hive state;
    public final long captureTimeNanos;
    public final List<Integer> tagIds;
    public final int visibleTagCount;
    public final boolean singleTagFallback;
    public final double reprojectionErrorPixels;
    public final double ambiguityMarginPixels;
    public final double hingeDegrees;
    /** Angle used for classification; equals hingeDegrees for the simple trig solver. */
    public final double stateHingeDegrees;
    /** Optical camera axes: right, down, forward. */
    public final double focusRightInches, focusDownInches, focusForwardInches;
    public final double rangeInches, bearingDegrees, elevationDegrees;

    HiveObservation(Alliance alliance, String reason, HiveGeometry.Cell cell, Hive state,
                    long captureTimeNanos, List<Integer> ids, int visibleTagCount,
                    double rms, double ambiguityMargin, double hingeDegrees, double stateHingeDegrees, double[] focus) {
        this.alliance = alliance;
        this.valid = reason.isEmpty();
        this.reason = reason;
        this.cell = cell;
        this.state = valid ? state : null;
        this.captureTimeNanos = captureTimeNanos;
        this.tagIds = Collections.unmodifiableList(new ArrayList<>(ids));
        this.visibleTagCount = visibleTagCount;
        this.singleTagFallback = ids.size() == 1;
        this.reprojectionErrorPixels = rms;
        this.ambiguityMarginPixels = ambiguityMargin;
        this.hingeDegrees = hingeDegrees;
        this.stateHingeDegrees = stateHingeDegrees;
        focusRightInches = valid ? focus[0] / HiveGeometry.INCH : Double.NaN;
        focusDownInches = valid ? focus[1] / HiveGeometry.INCH : Double.NaN;
        focusForwardInches = valid ? focus[2] / HiveGeometry.INCH : Double.NaN;
        rangeInches = valid ? HiveMath.norm(focus) / HiveGeometry.INCH : Double.NaN;
        bearingDegrees = valid ? Math.toDegrees(Math.atan2(-focus[0], focus[2])) : Double.NaN;
        elevationDegrees = valid ? Math.toDegrees(Math.atan2(-focus[1], Math.hypot(focus[0], focus[2]))) : Double.NaN;
    }

    public double imageAgeMillis(long nowNanos) {
        return captureTimeNanos == 0 ? Double.POSITIVE_INFINITY : Math.max(0, (nowNanos - captureTimeNanos) / 1e6);
    }

    static HiveObservation invalid(Alliance alliance, String reason, long captureTimeNanos) {
        return new HiveObservation(alliance, reason, null, null, captureTimeNanos,
                Collections.emptyList(), 0, Double.NaN, Double.NaN, Double.NaN, Double.NaN, null);
    }
}
