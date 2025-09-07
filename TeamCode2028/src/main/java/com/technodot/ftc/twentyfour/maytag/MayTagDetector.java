package com.technodot.ftc.twentyfour.maytag;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

/**
 * This class is responsible for detecting and processing AprilTags.
 */
@SuppressWarnings("FieldCanBeLocal")
public class MayTagDetector {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private WebcamName cameraFront;

    /**
     * Sets the camera status to active or inactive.
     *
     * @param active true to activate the camera, false to deactivate
     */
    public void setCameraStatus(Boolean active) {
        if (!active) {
            visionPortal.stopStreaming();
        } else {
            visionPortal.resumeStreaming();
        }
    }

    /**
     * Initializes the AprilTag detection system.
     *
     * @param map   the hardware map to use for initializing hardware devices
     * @param telem the telemetry object for sending data to the driver station
     */
    public void initAprilTag(HardwareMap map, Telemetry telem) {
        hardwareMap = map;
        telemetry = telem;

        // initialize AprilTag processor
        cameraFront = hardwareMap.get(WebcamName.class, "cameraFront");
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true) // draw axis on tag
                .setDrawCubeProjection(true) // draw cube projection on tag
                .setDrawTagOutline(true) // draw outline of tag
                .build();

        /*
        Adjust Image Decimation to trade-off detection-range for detection-rate.
        eg: Some typical detection data using a Logitech C920 WebCam
        Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        Note: Decimation can be changed on-the-fly to adapt during a match.
        */

        // aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder().setLiveViewContainerId(0);
        builder.setCamera(cameraFront);
        builder.enableLiveView(true); // enable live view of camera

        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(aprilTag, true); // disable or re-enable the aprilTag processor
    }

    /**
     * Detects AprilTags and returns their data.
     *
     * @return a 2D array where each row contains the tag ID, range, yaw, and bearing
     */
    @SuppressLint("DefaultLocale")
    public double[][] detectAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        List<AprilTagDetection> validDetections = new ArrayList<AprilTagDetection>();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("ID %d %s RYB %6.1f %6.1f %6.1f", detection.id, detection.metadata.name, detection.ftcPose.range, detection.ftcPose.yaw, detection.ftcPose.bearing));
                validDetections.add(detection);
            }
        }

        double[][] tagDataArray = new double[validDetections.size()][4];
        for (int i = 0; i < validDetections.size(); i++) {
            AprilTagDetection detection = validDetections.get(i);
            tagDataArray[i] = new double[]{detection.id, detection.ftcPose.range, detection.ftcPose.yaw, detection.ftcPose.bearing};
        }
        return tagDataArray;
    }

    /**
     * Closes the AprilTag detection system and releases resources.
     */
    public void closeAprilTag() {
        visionPortal.close();
    }
}
