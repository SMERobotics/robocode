package com.technodot.ftc.twentyfour.maytag;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * MayTag class for handling camera and tracking functionalities.
 */
public class MayTag {
    public MayTagDetector detector = new MayTagDetector();
    public MayTagTracker tracker = new MayTagTracker();

    private Telemetry telemetry;

    public MayTag(HardwareMap map, Telemetry telem) {
        detector.initAprilTag(map, telem);
        telemetry = telem;
    }

    /**
     * Sets the status of the camera.
     *
     * @param active true to activate the camera, false to deactivate
     */
    public void setCameraStatus(Boolean active) {
        detector.setCameraStatus(active);
    }

    /**
     * Sets the status of the camera.
     *
     * @param startPressed startPressed button
     * @param stopPressed stopPressed button
     */
    public void updateCamera(Boolean startPressed, Boolean stopPressed) {
        if (startPressed) {
            setCameraStatus(true);
        } else if (stopPressed) {
            setCameraStatus(false);
        }
    }

    /**
     * Updates the robot's position and logs it to telemetry.
     */
    @SuppressLint("DefaultLocale")
    public void update() {
        double[][] data = detector.detectAprilTag();
        tracker.updateRobotPositions(data);

        telemetry.addLine(String.format("Position: (%.2f, %.2f, %.2f)", tracker.x, tracker.y, tracker.h));
    }

    /**
     * Closes the MayTag system and releases resources.
     */
    public void close() {
        detector.closeAprilTag();
    }
}
