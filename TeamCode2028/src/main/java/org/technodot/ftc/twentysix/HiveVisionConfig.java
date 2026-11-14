package org.technodot.ftc.twentysix;

import com.acmerobotics.dashboard.config.Config;

import org.technodot.ftc.twentysix.roboctrl.SimpleHiveSolver;
import org.technodot.ftc.twentysix.roboctrl.HiveTurretAim;

@Config
public final class HiveVisionConfig {
    public enum CalibrationSource { DEFAULT, FILE, EEPROM }

    // Pipeline/calibration/corner-convention changes require STOP then INIT.
    /** Limelight USB address from the robot hardware configuration. */
    public static String LIMELIGHT_HOST = "172.29.0.1";
    public static int PIPELINE = 0;
    public static int WIDTH = 640;
    public static int HEIGHT = 480;
    public static CalibrationSource CALIBRATION_SOURCE = CalibrationSource.DEFAULT;
    /** Index of printed bottom-left in the firmware's four decoded corners. */
    public static int BOTTOM_LEFT_CORNER = 0;
    public static boolean REVERSE_CORNERS = false;
    public static SimpleHiveSolver.Config solver = new SimpleHiveSolver.Config();
    public static HiveTurretAim.Config turretAim = new HiveTurretAim.Config();

    private HiveVisionConfig() { }
}
