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

    public void init(HardwareMap hardwareMap) {
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
                .build();

        visionPortal.setProcessorEnabled(aprilTagProcessor, false);
        visionPortal.stopStreaming();
        visionPortal.stopLiveView();
    }
    public void start() {
        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
        visionPortal.resumeLiveView();
        visionPortal.resumeStreaming();

    }
    public int idDetection() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        for (AprilTagDetection tagDetected : detections) {
            switch (tagDetected.id) {
                case 21:
                    aprilTagIdCode = 21;
                    break;
                case 22:
                    aprilTagIdCode = 22;
                    break;
                case 23:
                    aprilTagIdCode = 23;
                    break;
                default:
                    break;
            }
        }
        return aprilTagIdCode;
    }


}
