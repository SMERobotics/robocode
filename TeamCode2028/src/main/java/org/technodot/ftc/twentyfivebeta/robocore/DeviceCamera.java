package org.technodot.ftc.twentyfivebeta.robocore;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.technodot.ftc.twentyfivebeta.Configuration;
import org.technodot.ftc.twentyfivebeta.common.Alliance;
import org.technodot.ftc.twentyfivebeta.common.Obelisk;
import org.technodot.ftc.twentyfivebeta.roboctrl.InputController;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Optional;
import java.util.concurrent.TimeUnit;

public class DeviceCamera extends Device {

    public AprilTagProcessor aprilTagProcessorFront;
    public VisionPortal visionPortalFront;

    public Obelisk obelisk; // last seen obelisk state
    public static AprilTagDetection goalTagDetection;
    public static long goalTagTimestamp;
    public static Optional<Double> fieldOffset;

    public int allianceTag;
    public boolean configured;

    public static final double GOAL_DEG = Math.toDegrees(Math.atan((double) 4 / 3));

    public DeviceCamera(Alliance alliance) {
        super(alliance);
    }

    @Override
    public void init(HardwareMap hardwareMap, InputController inputController) {
        allianceTag = this.alliance == Alliance.RED ? 24 : 20;

        aprilTagProcessorFront = new AprilTagProcessor.Builder()
                .setDrawAxes(Configuration.DEBUG)
                .setDrawCubeProjection(Configuration.DEBUG)
                .setDrawTagOutline(Configuration.DEBUG)
                .build();

        visionPortalFront = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "cameraFront"))
//                .setCameraResolution(new Size(1280, 720))
//                .setCameraResolution(new Size(640, 360))
                .setCameraResolution(new Size(800, 448))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTagProcessorFront)
//                .enableLiveView(Configuration.DEBUG)
//                .setAutoStartStreamOnBuild(true)
                .build();

        if (Configuration.DEBUG) FtcDashboard.getInstance().startCameraStream(visionPortalFront, 0);
    }

    @Override
    public void start() {
        configured = false;
    }

    @Override
    public void update() {
        configureCamera();

        goalTagDetection = null;
        if (aprilTagProcessorFront != null) {
            List<AprilTagDetection> frontDetections = aprilTagProcessorFront.getDetections();
            for (AprilTagDetection tag : frontDetections) {
                switch (tag.id) {
                    case 21:
                        obelisk = Obelisk.GPP;
                        continue;
                    case 22:
                        obelisk = Obelisk.PGP;
                        continue;
                    case 23:
                        obelisk = Obelisk.PPG;
                        continue;
                    case 20:
                    case 24:
                        fieldOffset = Optional.of((tag.id == 20 ? -GOAL_DEG : GOAL_DEG) + tag.ftcPose.yaw - tag.ftcPose.bearing + (this.alliance == Alliance.RED ? -90 : 90));
                        if (tag.id == allianceTag) {
                            goalTagDetection = tag;
                            goalTagTimestamp = System.nanoTime(); // TODO: i REALLY don't like this call when im literally calling ts in main update loop
                        }
                        continue;
                    default:
                }
            }
        }
    }

    @Override
    public void stop() {

    }

    public Optional<Double> getFieldOffset() {
        try {
            return fieldOffset;
        } finally {
            fieldOffset = Optional.empty();
        }
    }

    public AprilTagDetection getGoalDetection() {
        return goalTagDetection;
    }

    private void configureCamera() {
        if (configured || visionPortalFront.getCameraState() != VisionPortal.CameraState.STREAMING || visionPortalFront == null) return;

        ExposureControl exposureControl = visionPortalFront.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure(2L, TimeUnit.MILLISECONDS); // 100% increase!!!
        GainControl gainControl = visionPortalFront.getCameraControl(GainControl.class);
        gainControl.setGain(255);

        configured = true;
    }
}
