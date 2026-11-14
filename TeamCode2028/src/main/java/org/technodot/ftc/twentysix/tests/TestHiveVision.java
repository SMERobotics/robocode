package org.technodot.ftc.twentysix.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.technodot.ftc.twentysix.common.Alliance;
import org.technodot.ftc.twentysix.common.Hive;
import org.technodot.ftc.twentysix.robocore.DeviceLimelight;
import org.technodot.ftc.twentysix.roboctrl.GamepadBindings;
import org.technodot.ftc.twentysix.roboctrl.HiveGeometry;
import org.technodot.ftc.twentysix.roboctrl.HiveObservation;
import org.technodot.ftc.twentysix.roboctrl.HiveSolver;
import org.technodot.ftc.twentysix.roboctrl.SimpleHiveSolver;

import java.util.ArrayList;
import java.util.List;

/** Sensor-only diagnostic: no drive, turret, intake, or launcher is initialized. */
@TeleOp(name="TestVision", group="TechnoCode")
public class TestHiveVision extends OpMode {
    private DeviceLimelight camera;
    private String solverCheck;
    private boolean snapshotPressed;

    @Override
    public void init() {
        camera = new DeviceLimelight(Alliance.BLUE);
        camera.init(hardwareMap, new GamepadBindings(gamepad1, gamepad2), telemetry);
        solverCheck = checkSimpleSolver();
    }

    @Override
    public void init_loop() { showVision(); }

    @Override
    public void start() { camera.start(); }

    @Override
    public void loop() { showVision(); }

    @Override
    public void stop() { if (camera != null) camera.stop(); }

    private void showVision() {
        camera.update();
        telemetry.addData("simple solver check", solverCheck);
        telemetry.addData("capture raw JSON", "press A to write one frame to the RC log");
        if (gamepad1.a && !snapshotPressed) {
            String json = camera.limelight.getLatestResult().toString();
            int parts = (json.length() + 2999) / 3000;
            // Android log entries have a size limit; preserve the complete frame.
            for (int part = 0; part < parts; part++) {
                com.qualcomm.robotcore.util.RobotLog.ii("HiveVisionFrame", "part %d/%d %s", part + 1, parts,
                        json.substring(part * 3000, Math.min(json.length(), (part + 1) * 3000)));
            }
        }
        snapshotPressed = gamepad1.a;
        telemetry.update();
    }

    /** Exercises the active trig solver without needing a visible target. */
    private static String checkSimpleSolver() {
        try {
            SimpleHiveSolver.Config config = new SimpleHiveSolver.Config();
            config.cameraPitchDegrees = 0;
            config.cameraRollDegrees = 0;
            config.aimRightInches = 0;
            config.aimDownInches = 0;
            config.aimIntoCellInches = 0;
            HiveSolver.Calibration calibration = new HiveSolver.Calibration(1280, 960,
                    900, 900, 640, 480, new double[5]);
            List<HiveSolver.TagObservation> tags = new ArrayList<>();
            // Known -30 degree tilt, level camera: R = Rx(60 degrees).
            for (int id = 38; id <= 41; id++) {
                double[][] object = HiveGeometry.corners(id), image = new double[4][2];
                for (int i = 0; i < 4; i++) {
                    double y = .5 * object[i][1] - .7;
                    double z = Math.sqrt(.75) * object[i][1] + 2.5;
                    image[i][0] = 900 * object[i][0] / z + 640;
                    image[i][1] = 900 * y / z + 480;
                }
                tags.add(new HiveSolver.TagObservation(id, image));
            }
            SimpleHiveSolver solver = new SimpleHiveSolver(config);
            long time = System.nanoTime();
            solver.update(tags, calibration, time, time);
            double elapsed = (System.nanoTime() - time) / 1e6;
            HiveObservation observation = solver.getObservation(Alliance.BLUE, time);
            if (!observation.valid || observation.state != Hive.READY || Math.abs(observation.hingeDegrees + 30) > .1) {
                return "FAIL: " + observation.reason;
            }
            return String.format("PASS (%.1f ms)", elapsed);
        } catch (RuntimeException error) {
            return "FAIL: " + error.getClass().getSimpleName() + ": " + error.getMessage();
        }
    }
}
