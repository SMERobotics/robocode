package com.technodot.ftc.twentyfive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.technodot.ftc.twentyfive.batch.Batch;
import com.technodot.ftc.twentyfive.common.Obelisk;
import com.technodot.ftc.twentyfive.common.Team;
import com.technodot.ftc.twentyfive.robocore.DeviceCamera;
import com.technodot.ftc.twentyfive.robocore.DeviceDrive;
import com.technodot.ftc.twentyfive.robocore.DeviceExtake;
import com.technodot.ftc.twentyfive.robocore.DeviceIntake;

@Autonomous(name="BaboAuto", group="TechnoCode")
public class BaboAuto extends OpMode {

    public DeviceCamera deviceCamera = new DeviceCamera();
    public DeviceDrive deviceDrive = new DeviceDrive();
    public DeviceIntake deviceIntake = new DeviceIntake();
    public DeviceExtake deviceExtake = new DeviceExtake();

    public MultipleTelemetry t;

    public Batch runtime = new Batch();

    public Team team = Team.BLUE;
    public Obelisk obelisk = Obelisk.PPG;

    @Override
    public void init() {
    // Track what artifacts we currently hold (left/right)
    public ArtifactInventory inventory = new ArtifactInventory();

    // Queue of sides to shoot, ordered according to obelisk layout
    private java.util.Queue<ArtifactInventory.Side> shootQueue = new java.util.ArrayDeque<>();
        t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        deviceCamera.init(hardwareMap, team);
        deviceDrive.init(hardwareMap);
        deviceIntake.init(hardwareMap, team);
        deviceExtake.init(hardwareMap);


        runtime.plan(0, (long startMs, long durationMs, long executionMs) -> {
            deviceDrive.applyMovement(-3.8f, -0.1f, 0.0f);
            return false;
        });

        // Aim / rotate sequence (example retained)
        runtime.plan(3000, (long startMs, long durationMs, long executionMs) -> {
            deviceDrive.applyMovement(0.0f, 0.0f, team.equals(Team.BLUE) ? 54.0f : -54.0f);
        runtime.plan(3000, (long startMs, long durationMs, long executionMs) -> {
        });

        runtime.plan(3000, (long startMs, long durationMs, long executionMs) -> {
            // Instead of using SHOOTING_LOW, override the target velocity directly
        runtime.plan(4000, (long startMs, long durationMs, long executionMs) -> {
            deviceExtake.setVelocityOverride(1160.0);
            deviceExtake.setState(DeviceExtake.ExtakeState.SHOOTING_LOW);
        runtime.plan(4000, (long startMs, long durationMs, long executionMs) -> {
            deviceDrive.applyMovement(0.0f, 0.0f, team.equals(Team.BLUE) ? -54.0f : 54.0f);
            return false;
        });
            deviceDrive.updatePose(deviceCamera.update(), executionMs * 1_000_000L);
            // Determine obelisk and precompute shooting order.
    public void loop() {
        // This executes the actions planned in init(), which will call applyMovement().
            // init ts sequence
        runtime.plan(4000, 20000, (long startMs, long durationMs, long executionMs) -> {
        runtime.plan(4000, 20000, (long startMs, long durationMs, long executionMs) -> {
        deviceDrive.flushMovement();
        deviceExtake.update();
        runtime.plan(4000, 20000, (long startMs, long durationMs, long executionMs) -> {
        t.addData("fl", deviceDrive.motorFrontLeft.getCurrentPosition());
        // At 8000ms: inspect inventory, choose next artifact (ball) to shoot, and fire it.
        // At 8000ms: pop next artifact from queue, choose side via intake inventory, and fire.
        runtime.plan(8000, (long startMs, long durationMs, long executionMs) -> {
            // Simple policy: shoot PURPLE if we have it, otherwise GREEN.
