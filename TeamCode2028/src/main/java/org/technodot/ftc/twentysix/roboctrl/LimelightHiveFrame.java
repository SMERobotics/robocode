package org.technodot.ftc.twentysix.roboctrl;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import org.technodot.ftc.twentysix.common.Alliance;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;

/** Reads the JSON directly: SDK 12's corner parser only accepts nested pts arrays. */
public final class LimelightHiveFrame {
    public final long counter;
    public final double timestampMicros;
    public final int pipeline;
    public final double latencyMillis;
    public final List<HiveSolver.TagObservation> tags;
    /** Decoded hive IDs count as seen even if corners cannot produce a usable pose. */
    public final Set<Alliance> visibleHives;

    public LimelightHiveFrame(String json, int bottomLeftCorner, boolean reverse) throws JSONException {
        if (bottomLeftCorner < 0 || bottomLeftCorner > 3) throw new IllegalArgumentException("Corner index must be 0..3");
        JSONObject data = new JSONObject(json);
        counter = data.optLong("fidx", -1);
        timestampMicros = data.has("ts_us") ? data.optDouble("ts_us", Double.NaN) : data.optDouble("ts", Double.NaN) * 1000;
        if (counter < 0 && (!Double.isFinite(timestampMicros) || timestampMicros <= 0)) {
            throw new IllegalArgumentException("No image counter or camera timestamp");
        }
        pipeline = data.optInt("pID", -1);
        latencyMillis = data.optDouble("cl", 0) + data.optDouble("tl", 0);
        if (!Double.isFinite(latencyMillis) || latencyMillis < 0) throw new IllegalArgumentException("Invalid image latency");
        List<HiveSolver.TagObservation> found = new ArrayList<>();
        Set<Alliance> seen = EnumSet.noneOf(Alliance.class);
        JSONArray fiducials = data.optJSONArray("Fiducial");
        if (data.optInt("v", 0) == 1 && fiducials != null) for (int i = 0; i < fiducials.length(); i++) {
            JSONObject tag = fiducials.optJSONObject(i);
            if (tag == null) continue;
            int id = tag.optInt("fID", -1);
            HiveGeometry.Cell cell = HiveGeometry.cellForTag(id);
            if (cell == null) continue;
            seen.add(cell.alliance);
            double[][] points = decodeCorners(tag.optJSONArray("pts"), bottomLeftCorner, reverse);
            if (points != null) found.add(new HiveSolver.TagObservation(id, points));
        }
        tags = Collections.unmodifiableList(found);
        visibleHives = Collections.unmodifiableSet(seen);
    }

    private static double[][] decodeCorners(JSONArray pts, int first, boolean reverse) {
        if (pts == null || (pts.length() != 4 && pts.length() != 8)) return null;
        double[][] decoded = new double[4][2];
        for (int i = 0; i < 4; i++) {
            int k = (first + (reverse ? -i : i) + 4) % 4;
            JSONArray nested = pts.optJSONArray(k);
            if (pts.length() == 4) {
                if (nested == null || nested.length() != 2) return null;
                decoded[i][0] = nested.optDouble(0, Double.NaN);
                decoded[i][1] = nested.optDouble(1, Double.NaN);
            } else {
                decoded[i][0] = pts.optDouble(2 * k, Double.NaN);
                decoded[i][1] = pts.optDouble(2 * k + 1, Double.NaN);
            }
            if (!Double.isFinite(decoded[i][0]) || !Double.isFinite(decoded[i][1])) return null;
        }
        return decoded;
    }
}
