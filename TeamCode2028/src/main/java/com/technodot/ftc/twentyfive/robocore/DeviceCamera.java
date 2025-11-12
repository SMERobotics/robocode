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

/**
 * Dual-camera AprilTag device.
 * FRONT camera streams to Dashboard & DS. BACK camera is processing-only.
 * Once an Obelisk tag (21,22,23) is seen by either camera, the BACK camera is fully released.
 */
public class DeviceCamera extends Device {

    // FRONT (legacy fields retained for compatibility)
    public AprilTagProcessor aprilTagProcessor; // FRONT
    public VisionPortal visionPortal; // FRONT

    // BACK (no stream)
    public AprilTagProcessor aprilTagProcessorBack;
    public VisionPortal visionPortalBack;

    public Obelisk obelisk; // Last seen obelisk
    public int teamID;      // 20 BLUE / 24 RED

    private boolean backDisabled = false; // Set true once back camera is shut down

    @Override
    @Deprecated
    public void init(HardwareMap hardwareMap) { // default to BLUE team
        init(hardwareMap, Team.BLUE);
    }

    public void init(HardwareMap hardwareMap, Team team) {
        teamID = team.equals(Team.RED) ? 24 : 20;

        // Create a MultiPortal view so SDK is happy with multiple portals.
        // We'll assign FRONT to the first view, BACK to the second.
        int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
        int frontViewId = viewIds[0];
        int backViewId = viewIds[1];

        // FRONT: streamed (both DS preview and Dashboard)
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();
        VisionPortal.Builder frontBuilder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "cameraFront"))
                .setLiveViewContainerId(frontViewId)
                .addProcessor(aprilTagProcessor);
        visionPortal = frontBuilder.build();

        // BACK: processing only; must still assign a view ID when multiple portals are present
        aprilTagProcessorBack = new AprilTagProcessor.Builder().build();
        VisionPortal.Builder backBuilder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "cameraBack"))
                .setLiveViewContainerId(backViewId)
                .addProcessor(aprilTagProcessorBack);
        visionPortalBack = backBuilder.build();

        // Stream only FRONT to Dashboard (frame rate 0 means default)
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
    }

    @Override
    public void start() {
        obelisk = null;
        backDisabled = false;
    }

    @Override
    public void update(Gamepad gamepad) { // periodic poll
        detectTags(false);
    }

    /**
     * Detect AprilTags on both cameras. BACK camera will be disabled when an Obelisk is found.
     * @param returnTeamTag if true will return detection of current team tag (20/24) else null
     */
    public AprilTagDetection detectTags(boolean returnTeamTag) {
        AprilTagDetection teamTagDetection = null;

        // BACK first (may disable itself)
        if (!backDisabled && visionPortalBack != null && aprilTagProcessorBack != null) {
            List<AprilTagDetection> backDetections = aprilTagProcessorBack.getDetections();
            for (AprilTagDetection d : backDetections) {
                if (isObelisk(d.id)) {
                    assignObelisk(d.id);
                    disableBackCamera();
                    // After disabling we keep evaluating FRONT but stop looping back.
                } else if (returnTeamTag && isTeamTag(d.id) && d.id == teamID && teamTagDetection == null) {
                    teamTagDetection = d;
                }
                if (backDisabled) break;
            }
        }

        // FRONT detections
        if (aprilTagProcessor != null) {
            List<AprilTagDetection> frontDetections = aprilTagProcessor.getDetections();
            for (AprilTagDetection d : frontDetections) {
                if (isObelisk(d.id)) {
                    assignObelisk(d.id); // FRONT never disables itself
                } else if (returnTeamTag && isTeamTag(d.id) && d.id == teamID && teamTagDetection == null) {
                    teamTagDetection = d;
                }
            }
        }

        return teamTagDetection;
    }

    /** Return team tag detection (20/24) if visible. */
    public AprilTagDetection update() { // convenience method if caller wants team tag
        return detectTags(true);
    }

    private boolean isTeamTag(int id) { return id == 20 || id == 24; }
    private boolean isObelisk(int id) { return id == 21 || id == 22 || id == 23; }

    private void assignObelisk(int id) {
        switch (id) {
            case 21: obelisk = Obelisk.GPP; break;
            case 22: obelisk = Obelisk.PGP; break;
            case 23: obelisk = Obelisk.PPG; break;
        }
    }

    private void disableBackCamera() {
        if (backDisabled) return;
        backDisabled = true;
        if (visionPortalBack != null) {
            try {
                if (aprilTagProcessorBack != null) {
                    visionPortalBack.setProcessorEnabled(aprilTagProcessorBack, false);
                }
            } catch (Exception ignored) {}
            try { visionPortalBack.close(); } catch (Exception ignored) {}
        }
        aprilTagProcessorBack = null;
        visionPortalBack = null;
    }

    @Override
    public void stop() {
//        // Cleanly close portals.
//        if (visionPortal != null) {
//            try { visionPortal.close(); } catch (Exception ignored) {}
//        }
//        if (visionPortalBack != null) {
//            try { visionPortalBack.close(); } catch (Exception ignored) {}
//        }
//        visionPortal = null;
//        visionPortalBack = null;
//        aprilTagProcessor = null;
//        aprilTagProcessorBack = null;
    }

    public void getTeam(Team team) {
        setTeam(team);
    }

    public void setTeam(Team team) {
        teamID = team.equals(Team.RED) ? 24 : 20;
    }

    public Obelisk getObelisk() {
        return obelisk;
    }
}
