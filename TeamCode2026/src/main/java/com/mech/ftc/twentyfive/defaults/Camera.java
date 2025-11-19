package com.mech.ftc.twentyfive.defaults;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class Camera {

    public AprilTagProcessor aprilTagProcessor;
    public VisionPortal visionPortal;
    public int aprilTagIdCode;
    public WebcamName webcam;

    public void init(HardwareMap hardwareMap) {
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .enableLiveView(true)
                .build();

        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
//        visionPortal.setProcessorEnabled(aprilTagProcessor, false);
//        visionPortal.stopStreaming();
//        visionPortal.stopLiveView();
    }
    public void start() {
//        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
//        visionPortal.resumeLiveView();
//        visionPortal.resumeStreaming();

    }
    public int TagID() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection tagDetected : detections) {
            switch (tagDetected.id) {
                case 20:
                    aprilTagIdCode = 20;
                    break;
                case 21:
                    aprilTagIdCode = 21;
                    break;
                case 22:
                    aprilTagIdCode = 22;
                    break;
                case 23:
                    aprilTagIdCode = 23;
                    break;
                case 24:
                    aprilTagIdCode = 24;
                    break;
                default:
                    aprilTagIdCode = -1;
                    break;
            }
        }
        return aprilTagIdCode;
    }
    public double getTagDistance() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        double tagDistance = 0;
        for (AprilTagDetection tagDetected : detections) {
            if (tagDetected.metadata == null) {
                return 0;
            }
            tagDistance = tagDetected.ftcPose.range;
        }
        return tagDistance*0.0254;
    }
    public double getTagDistanceY() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        double tagDistance = 0;
        for (AprilTagDetection tagDetected : detections) {
            if (tagDetected.metadata == null) {
                return 0;
            }
            tagDistance = tagDetected.ftcPose.y;
        }
        return tagDistance*0.0254;
    }
    public double getTagDistanceX() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        double tagDistance = 0;
        for (AprilTagDetection tagDetected : detections) {
            if (tagDetected.metadata == null) {
                return 0;
            }
            tagDistance = tagDetected.ftcPose.x;
        }
        return tagDistance*0.0254;
    }
    public double getTagBearing() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        double bearing = 0;
        for (AprilTagDetection tagDetected : detections) {
            if (tagDetected.metadata == null) {
                return 0;
            }
            bearing = tagDetected.ftcPose.bearing;
        }
        return bearing;
    }



}
