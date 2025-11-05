package com.technodot.ftc.twentyfive.robocore;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technodot.ftc.twentyfive.common.Obelisk;
import com.technodot.ftc.twentyfive.common.Team;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class DeviceCamera extends Device {

    public AprilTagProcessor aprilTagProcessor;
    public VisionPortal visionPortal;

    public Obelisk obelisk;
    public int teamID;

    @Override
    @Deprecated // make sure dumbass future me remembers that ts is the wrong one
    public void init(HardwareMap hardwareMap) {
        // default to BLUE team
        init(hardwareMap, Team.BLUE);
    }

    public void init(HardwareMap hardwareMap, Team team) {
        teamID = team.equals(Team.RED) ? 24 : 20; // defaults to Team.BLUE with tag ID 20
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "camera"));
        builder.addProcessor(aprilTagProcessor);
        visionPortal = builder.build();

        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);

        // TODO: get the dynamic enablement shit working
        // (for laters! if it ain't broke, don't touch it)

//        visionPortal.setProcessorEnabled(aprilTagProcessor, false);
//        visionPortal.stopLiveView();
//        visionPortal.stopStreaming();
    }

    @Override
    public void start() {
//        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
//        visionPortal.resumeLiveView();
//        visionPortal.resumeStreaming();

        obelisk = null;
    }

    @Override
    public void update(Gamepad gamepad) {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : detections) {
            switch (detection.id) {
                case 21:
                    obelisk = Obelisk.GPP;
                    break;
                case 22:
                    obelisk = Obelisk.PGP;
                    break;
                case 23:
                    obelisk = Obelisk.PPG;
                    break;
                default:
                    break;
            }
        }
    }

    public AprilTagDetection update() {
        AprilTagDetection target = null;
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : detections) {
            switch (detection.id) {
                case 21:
                    obelisk = Obelisk.GPP;
                case 22:
                    obelisk = Obelisk.PGP;
                case 23:
                    obelisk = Obelisk.PPG;
                case 20: // team BLUE
                case 24: // team RED
                    if (detection.id == teamID) target = detection;
                    break;
                default:
                    break;
            }
        }
        return target;
    }

    @Override
    public void stop() {
//        visionPortal.setProcessorEnabled(aprilTagProcessor, false);
//        visionPortal.stopLiveView();
//        visionPortal.stopStreaming();
    }

    public void setTeam(Team team) {
        teamID = team.equals(Team.RED) ? 24 : 20;
    }

    public void getTeam(Team team) {
        teamID = team.equals(Team.RED) ? 24 : 20;
    }
}
