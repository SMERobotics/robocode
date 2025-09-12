package com.technodot.ftc.twentyfive.robocore;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

/*
fucking resources
which dumbfuck thought it would be a good idea to make the datasheet and register map separate documents
bro just put them in one pdf like a normal person
- copilot literally autocompleted those two comments it literally understands how fucking ridiculous this is

https://components101.com/sites/default/files/component_datasheet/MPU6050-DataSheet.pdf
https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Writing-an-I2C-Driver
 */

@I2cDeviceType
@DeviceProperties(name="MPU6050 6-Axis IMU", xmlTag="MPU6050")
public class DriverMPU6050 extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    public DriverMPU6050(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(0x68));
        this.deviceClient.engage();
    }

    public enum Register {
        ACCEL_OUT(0x3B),
        GYRO_OUT(0x43),
        TEMP_OUT(0x41),
        SIGNAL_PATH_RESET(0x68);

        public int address;

        Register (int address) {
            this.address = address;
        }
    }

    @Override
    protected synchronized boolean doInitialize() {
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Unknown;
    }

    @Override
    public String getDeviceName() {
        return "MPU6050 6-Axis Accelerometer and Gyroscope";
    }
}
