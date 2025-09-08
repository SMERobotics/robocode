package com.technodot.ftc.twentyfive.robocore;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technodot.ftc.twentyfive.ObeliskType;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class DeviceCamera extends Device {

    public AprilTagProcessor aprilTagProcessor;
    public VisionPortal visionPortal;

    public ObeliskType obeliskType;

    @Override
    public void init(HardwareMap hardwareMap) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "camera"));
        builder.addProcessor(aprilTagProcessor);
        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(aprilTagProcessor, false);
        visionPortal.stopLiveView();
        visionPortal.stopStreaming();
    }

    @Override
    public void start() {
        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
        visionPortal.resumeLiveView();
        visionPortal.resumeStreaming();
    }

    @Override
    public void update(Gamepad gamepad) {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : detections) {
            switch (detection.id) {
                case 21:
                    obeliskType = ObeliskType.GPP;
                    break;
                case 22:
                    obeliskType = ObeliskType.PGP;
                    break;
                case 23:
                    obeliskType = ObeliskType.PPG;
                    break;
                default:
                    break;
            }
        }
    }

    @Override
    public void stop() {
        visionPortal.setProcessorEnabled(aprilTagProcessor, false);
        visionPortal.stopLiveView();
        visionPortal.stopStreaming();
    }
}
