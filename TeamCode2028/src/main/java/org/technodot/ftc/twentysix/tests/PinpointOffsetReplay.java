package org.technodot.ftc.twentysix.tests;

import java.io.File;
import java.util.Locale;

/** Desktop entry point: CSV path, then optional output report base path. */
public final class PinpointOffsetReplay {
    private PinpointOffsetReplay() { }

    public static void main(String[] args) throws Exception {
        if (args.length < 1 || args.length > 2) throw new IllegalArgumentException("Usage: PinpointOffsetReplay capture.csv [output-base]");
        File source = new File(args[0]);
        PinpointOffsetCapture.Capture capture = PinpointOffsetCapture.read(source);
        PinpointOffsetEstimator.Result result = capture.rotatingCalibration
                ? PinpointRotatingOffsetEstimator.estimate(capture.samples, true)
                : PinpointOffsetEstimator.estimate(capture.samples, true);
        File output = args.length == 2 ? new File(args[1]) : source;
        PinpointOffsetCapture.writeSummary(output, result, result.validated() ? "VALIDATED" : "INCONCLUSIVE");
        System.out.printf(Locale.US, "%s: X %.6f +/- %.6f mm; Y %.6f +/- %.6f mm%n",
                result.validated() ? "VALIDATED" : "INCONCLUSIVE", result.xMm, result.xIntervalMm, result.yMm, result.yIntervalMm);
        for (String issue : result.issues) System.out.println(issue);
        System.out.println("Report: " + output.getPath() + ".txt");
    }
}
