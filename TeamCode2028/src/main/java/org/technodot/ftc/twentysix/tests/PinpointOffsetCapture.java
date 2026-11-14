package org.technodot.ftc.twentysix.tests;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

/** Replayable observations and a human-readable report; never stores an FL count. */
final class PinpointOffsetCapture {
    private PinpointOffsetCapture() { }

    static void writeSamples(File file, List<PinpointOffsetEstimator.Sample> samples) throws IOException {
        File parent = file.getParentFile();
        if (parent != null && !parent.isDirectory() && !parent.mkdirs()) {
            throw new IOException("Cannot create " + parent);
        }
        try (BufferedWriter writer = writer(file)) {
            writer.write("run,turn,fr_ticks,bl_ticks,br_ticks,pod_forward_mm,pod_left_mm,angle_rad,dt_s,cos_s,sin_s\n");
            for (PinpointOffsetEstimator.Sample s : samples) {
                writer.write(String.format(Locale.US, "%d,%d,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g%n",
                        s.run, s.turn, s.ticks[1], s.ticks[2], s.ticks[3], s.podForwardMm,
                        s.podLeftMm, s.heading, s.seconds, s.cosSeconds, s.sinSeconds));
            }
        }
    }

    static void writeSummary(File csv, PinpointOffsetEstimator.Result result, String status) throws IOException {
        try (BufferedWriter writer = writer(new File(csv.getPath() + ".txt"))) {
            writer.write(status + "\nFR/BL/BR only; FL ignored. Robot axes: forward/left/CCW.\n");
            writer.write("Negative run IDs are successive F/B/L/R translation legs. Runs 0..3 are CCW slow, CW slow, CW fast, CCW fast; 4..5 are held-out CCW/CW.\n");
            if (result == null) return;
            writer.write(String.format(Locale.US,
                    "X %.9f mm; Y %.9f mm\n95%% repeatability +/-: X %.9f; Y %.9f mm\nDirection/speed/validation gaps: %.9f / %.9f / %.9f mm\nBefore/after translation check: %.9f mm\n",
                    result.xMm, result.yMm, result.xIntervalMm, result.yIntervalMm,
                    result.directionGapMm, result.speedGapMm, result.validationGapMm, result.translationCheckGapMm));
            writer.write("Leave-one-translation maximum X/Y shifts (mm): " + Arrays.toString(result.translationSensitivityMm) + "\n");
            writer.write("Leave-one-turn maximum X/Y shifts (mm): " + Arrays.toString(result.turnSensitivityMm) + "\n");
            writer.write("Accepted turns: " + Arrays.toString(result.acceptedTurns) + "\n");
            writer.write(String.format(Locale.US, "Drive F/L mm/tick %.9f / %.9f; gains %s\n",
                    result.forwardMmPerTick, result.leftMmPerTick, PinpointOffsetEstimator.formatCounts(result.wheelScales)));
            for (int run = 0; run < result.runOffsetsMm.length; run++) {
                writer.write("Run " + (run + 1) + " offsets with fixed training wheel model (mm): "
                        + Arrays.toString(result.runOffsetsMm[run]) + "\n");
            }
            for (String issue : result.issues) writer.write(issue + "\n");
            writer.write("Repeatability does not measure systematic drive-wheel slip.\n");
        }
    }

    private static BufferedWriter writer(File file) throws IOException {
        return new BufferedWriter(new OutputStreamWriter(new FileOutputStream(file), StandardCharsets.UTF_8));
    }
}
