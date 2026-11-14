package org.technodot.ftc.twentysix.robocore;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.technodot.ftc.twentysix.HiveVisionConfig;
import org.technodot.ftc.twentysix.common.Alliance;
import org.technodot.ftc.twentysix.roboctrl.GamepadBindings;
import org.technodot.ftc.twentysix.roboctrl.HiveObservation;
import org.technodot.ftc.twentysix.roboctrl.HiveSolver;
import org.technodot.ftc.twentysix.roboctrl.LimelightCalibration;
import org.technodot.ftc.twentysix.roboctrl.LimelightHiveFrame;
import org.technodot.ftc.twentysix.roboctrl.SimpleHiveSolver;

import java.io.IOException;
import java.net.URL;
import java.util.EnumMap;

public class DeviceLimelight extends Device {
    public Limelight3A limelight;
    public SimpleHiveSolver hiveSolver;
    private HiveSolver.Calibration calibration;
    private boolean running;
    private int pipeline;
    private int firstCorner;
    private boolean reverseCorners;
    private long lastCounter = -1;
    private double lastCameraTime = Double.NaN;
    private String calibrationStatus = "not loaded";
    private double solveMillis;
    private final EnumMap<Alliance, Long> lastHiveTagSeen = new EnumMap<>(Alliance.class);

    public DeviceLimelight(Alliance alliance) {
        super(alliance);
    }

    @Override
    public void init(HardwareMap hardwareMap, GamepadBindings gamepad, Telemetry telemetry) {
        super.init(hardwareMap, gamepad, telemetry);
        lastHiveTagSeen.clear();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        hiveSolver = new SimpleHiveSolver(HiveVisionConfig.solver);
        pipeline = HiveVisionConfig.PIPELINE;
        firstCorner = HiveVisionConfig.BOTTOM_LEFT_CORNER;
        reverseCorners = HiveVisionConfig.REVERSE_CORNERS;
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(pipeline);
        try {
            loadCalibration();
        } catch (IOException | RuntimeException error) {
            calibrationStatus = HiveVisionConfig.CALIBRATION_SOURCE + ": "
                    + error.getClass().getSimpleName() + ": " + error.getMessage();
            hiveSolver.reset(calibrationStatus);
        }
        start();
    }

    private void loadCalibration() throws IOException {
        String endpoint;
        switch (HiveVisionConfig.CALIBRATION_SOURCE) {
            case FILE: endpoint = "/cal-file"; break;
            case EEPROM: endpoint = "/cal-eeprom"; break;
            default: endpoint = "/cal-default"; break;
        }
        calibration = LimelightCalibration.fetch(new URL("http", HiveVisionConfig.LIMELIGHT_HOST, 5807, endpoint),
                HiveVisionConfig.WIDTH, HiveVisionConfig.HEIGHT, false);
        calibrationStatus = HiveVisionConfig.CALIBRATION_SOURCE + " " + calibration.width + "x" + calibration.height;
    }

    @Override
    public void start() {
        if (!running) { limelight.start(); running = true; }
    }

    @Override
    public void update() {
        long now = System.nanoTime();
        telemetry.addData("limelight connected", limelight.isConnected());
        LLResult result = limelight.getLatestResult();
        try {
            if (result != null) {
                LimelightHiveFrame frame = new LimelightHiveFrame(result.toString(), firstCorner, reverseCorners);
                if (frame.pipeline != pipeline) {
                    hiveSolver.reset("waiting for Hive pipeline " + pipeline);
                    lastCounter = -1; lastCameraTime = Double.NaN;
                } else {
                    boolean repeated = frame.counter >= 0 ? frame.counter == lastCounter
                            : Double.compare(frame.timestampMicros, lastCameraTime) == 0;
                    if (!repeated) {
                        if ((frame.counter >= 0 && lastCounter >= 0 && frame.counter < lastCounter)
                                || (Double.isFinite(frame.timestampMicros) && Double.isFinite(lastCameraTime)
                                && frame.timestampMicros < lastCameraTime)) hiveSolver.reset("camera restarted");
                        lastCounter = frame.counter; lastCameraTime = frame.timestampMicros;
                        double age = frame.latencyMillis + result.getParseLatency() + Math.max(0, result.getStaleness());
                        if (!Double.isFinite(age) || age < 0) throw new IllegalArgumentException("Invalid image age");
                        long capture = now - (long) (age * 1e6);
                        for (Alliance visible : frame.visibleHives) {
                            lastHiveTagSeen.put(visible, Math.max(lastHiveTagSeen.getOrDefault(visible, 0L), capture));
                        }
                        if (calibration != null) {
                            long begin = System.nanoTime();
                            hiveSolver.update(frame.tags, calibration, capture, now);
                            solveMillis = (System.nanoTime() - begin) / 1e6;
                        }
                    }
                }
            } else {
                hiveSolver.reset("no camera result");
            }
        } catch (Exception error) {
            hiveSolver.reset("vision error: " + error.getClass().getSimpleName() + ": " + error.getMessage());
        }
        if (calibration == null) hiveSolver.reset("calibration unavailable: " + calibrationStatus);
        hiveSolver.expire(System.nanoTime());
        telemetry.addData("hive calibration", calibrationStatus);
        telemetry.addData("hive solver", "simple trig");
        telemetry.addData("hive solve (ms)", "%.1f", solveMillis);
        addHiveTelemetry(Alliance.RED);
        addHiveTelemetry(Alliance.BLUE);
    }

    public HiveObservation getHiveObservation(Alliance alliance) {
        return hiveSolver.getObservation(alliance, System.nanoTime());
    }

    public long getLastHiveTagSeenNanos(Alliance alliance) {
        return lastHiveTagSeen.getOrDefault(alliance, 0L);
    }

    private void addHiveTelemetry(Alliance alliance) {
        HiveObservation observation = getHiveObservation(alliance);
        String prefix = alliance + " hive";
        telemetry.addData(prefix, observation.valid ? observation.state : observation.reason);
        telemetry.addData(prefix + " cell", observation.cell);
        telemetry.addData(prefix + " tags", observation.tagIds);
        telemetry.addData(prefix + " age (ms)", "%.0f", observation.imageAgeMillis(System.nanoTime()));
        telemetry.addData(prefix + " tilt (deg)", "%.1f", observation.hingeDegrees);
        if (observation.valid) {
            telemetry.addData(prefix + " RBE (in,deg,deg)", "%.1f, %.1f, %.1f", observation.rangeInches,
                    observation.bearingDegrees, observation.elevationDegrees);
            telemetry.addData(prefix + " single tag", observation.singleTagFallback);
        }
    }

    @Override
    public void stop() {
        limelight.stop();
        running = false;
        hiveSolver.reset("stopped");
    }
}
