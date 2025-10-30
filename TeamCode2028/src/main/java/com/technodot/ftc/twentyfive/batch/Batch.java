package com.technodot.ftc.twentyfive.batch;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technodot.ftc.twentyfive.robocore.Device;
import com.technodot.ftc.twentyfive.robocore.DeviceDrive;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Batch {
    private Map<Device, List<DevicePlan>> devicePlans = new HashMap<>();

    public void init(HardwareMap hardwareMap) {
        for (Device device : devicePlans.keySet()) {
            device.init(hardwareMap);
        }
    }

    public void start() {
        for (Device device : devicePlans.keySet()) {
            device.start();
        }
    }

    public void stop() {
        for (Device device : devicePlans.keySet()) {
            device.stop();
        }
    }

    public void plan(Device device, long time, double... in) {
        if (!devicePlans.containsKey(device)) {
            devicePlans.put(device, new ArrayList<>());
        }
        devicePlans.get(device).add(new DevicePlan(time, in));
    }

    public void run() {
        if (devicePlans.isEmpty()) {
            return;
        }

        // Track state for each device
        Map<Device, DeviceState> deviceStates = new HashMap<>();
        for (Device device : devicePlans.keySet()) {
            deviceStates.put(device, new DeviceState());
        }

        long startTime = System.currentTimeMillis();

        while (true) {
            long currentTime = System.currentTimeMillis();

            // Process each device according to its plan
            for (Map.Entry<Device, List<DevicePlan>> entry : devicePlans.entrySet()) {
                Device device = entry.getKey();
                List<DevicePlan> plans = entry.getValue();
                DeviceState state = deviceStates.get(device);

                if (plans == null || plans.isEmpty()) {
                    continue;
                }

                // Check if it's time to move to the next plan
                if (currentTime - state.planStartTime >= plans.get(state.currentPlanIndex).time) {
                    state.currentPlanIndex++;
                    if (state.currentPlanIndex >= plans.size()) {
                        state.currentPlanIndex = 0; // Loop back to the first plan
                    }
                    state.planStartTime = currentTime;
                }

                // Apply the current plan's inputs to the device
                DevicePlan currentPlan = plans.get(state.currentPlanIndex);
                applyInput(device, currentPlan.inputs);
            }
        }
    }

    private void applyInput(Device device, double[] inputs) {
        // This method will need to be implemented based on how each device handles inputs
        // For now, devices can override a method to accept control inputs
        // Or we can cast to specific device types and call their update methods
        if (device instanceof DeviceDrive) {
            DeviceDrive drive = (DeviceDrive) device;
            if (inputs.length >= 3) {
                drive.update((float) inputs[0], (float) inputs[1], (float) inputs[2]);
            }
        }
        // Add more device types as needed
    }

    private static class DeviceState {
        int currentPlanIndex = 0;
        long planStartTime = System.currentTimeMillis();
    }

    private static class DevicePlan {
        final long time;
        final double[] inputs;

        DevicePlan(long time, double[] inputs) {
            this.time = time;
            this.inputs = inputs;
        }
    }
}
