package com.technodot.ftc.twentyfive.robocore;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.technodot.ftc.twentyfive.common.TimedVector3;

/*
fucking resources
which dumbfuck thought it would be a good idea to make the datasheet and register map separate documents
bro just put them in one pdf like a normal person
- copilot literally autocompleted those two comments it literally understands how fucking ridiculous this is

https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Writing-an-I2C-Driver
 */

@I2cDeviceType
@DeviceProperties(name="MPU6050 6-Axis IMU", xmlTag="MPU6050")
public class MPU6050 extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    public static final I2cAddr I2C_DEVICE_ADDRESS = I2cAddr.create7bit(0x68);

    public static final int DEFAULT_SMPLRT_DIV = 0x00; // Sample rate divider
    public static final int DEFAULT_CONFIG = 0x00; // No external sync, 260Hz DLPF
    public static final int DEFAULT_GYRO_CONFIG = 0x00; // ±250°/s
    public static final int DEFAULT_ACCEL_CONFIG = 0x00; // ±2g
    public static final int DEFAULT_FIFO_EN = 0x00; // FIFO disabled
    public static final int DEFAULT_INT_ENABLE = 0x00; // Interrupts disabled
    public static final int DEFAULT_USER_CTRL = 0x00; // FIFO & interrupts disabled
    public static final int DEFAULT_SIGNAL_PATH_RESET = 0x00; // No reset
    public static final int DEFAULT_PWR_MGMT_1 = 0x00; // Awake, no sleep
    public static final int DEFAULT_PWR_MGMT_2 = 0x00; // All sensors enabled

    private static final double GRAVITY = 9.80665;

    public MPU6050(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);
        this.deviceClient.setI2cAddress(I2C_DEVICE_ADDRESS);
        this.deviceClient.engage();
    }

    public enum Register {
        SMPLRT_DIV(0x19),
        CONFIG(0x1A),
        GYRO_CONFIG(0x1B),
        ACCEL_CONFIG(0x1C),
        FIFO_EN(0x23),
        INT_ENABLE(0x38),
        INT_STATUS(0x3A),
        ACCEL_OUT(0x3B),
        TEMP_OUT(0x41),
        GYRO_OUT(0x43),
        USER_CTRL(0x6A),
        SIGNAL_PATH_RESET(0x68),
        PWR_MGMT_1(0x6B),
        PWR_MGMT_2(0x6C),
        WHO_AM_I(0x75);

        public int address;

        Register(int address) {
            this.address = address;
        }
    }

    @Override
    protected synchronized boolean doInitialize() {
        int whoAmI = deviceClient.read8(Register.WHO_AM_I.address) & 0xFF;
        if (whoAmI != 0x68) return false;

        deviceClient.write8(Register.PWR_MGMT_1.address, DEFAULT_PWR_MGMT_1);
        deviceClient.write8(Register.PWR_MGMT_2.address, DEFAULT_PWR_MGMT_2);

        assertDefault(Register.SMPLRT_DIV, DEFAULT_SMPLRT_DIV);
        assertDefault(Register.CONFIG, DEFAULT_CONFIG);
        assertDefault(Register.GYRO_CONFIG, DEFAULT_GYRO_CONFIG);
        assertDefault(Register.ACCEL_CONFIG, DEFAULT_ACCEL_CONFIG);
        assertDefault(Register.FIFO_EN, DEFAULT_FIFO_EN);
        assertDefault(Register.INT_ENABLE, DEFAULT_INT_ENABLE);
        assertDefault(Register.USER_CTRL, DEFAULT_USER_CTRL);
        assertDefault(Register.SIGNAL_PATH_RESET, DEFAULT_SIGNAL_PATH_RESET);

        return true;
    }

    protected void assertDefault(Register register, int value) {
        if ((deviceClient.read8(register.address) & 0xFF) != value) {
            deviceClient.write8(register.address, value);
        }
    }

    protected static int bytesToSigned16(byte msb, byte lsb) {
        return (short)(((msb & 0xFF) << 8) | (lsb & 0xFF));
    }

    protected double accelLsbPerG() {
        int cfg = deviceClient.read8(Register.ACCEL_CONFIG.address) & 0xFF;
        int sel = (cfg >> 3) & 0x03; // AFS_SEL
        switch (sel) {
            case 0: return 16384.0; // ±2g
            case 1: return 8192.0; // ±4g
            case 2: return 4096.0; // ±8g
            case 3: return 2048.0; // ±16g
            default: return 16384.0;
        }
    }

    protected double gyroLsbPerDegPerSec() {
        int cfg = deviceClient.read8(Register.GYRO_CONFIG.address) & 0xFF;
        int sel = (cfg >> 3) & 0x03; // FS_SEL
        switch (sel) {
            case 0: return 131.072; // ±250
            case 1: return 65.536; // ±500
            case 2: return 32.768; // ±1000
            case 3: return 16.384; // ±2000
            default: return 131.072;
        }
    }

    public synchronized TimedVector3 getAcceleration() {
        byte[] buf = deviceClient.read(Register.ACCEL_OUT.address, 6);
        // on device init, we hardcoded accel to ±2g, lsbPerG is always 16384.0
        // int rawX = bytesToSigned16(buf[0], buf[1]);
        // int rawY = bytesToSigned16(buf[2], buf[3]);
        // int rawZ = bytesToSigned16(buf[4], buf[5]);
        // double lsbPerG = accelLsbPerG();
        // double x = (rawX / lsbPerG) * GRAVITY;
        // double y = (rawY / lsbPerG) * GRAVITY;
        // double z = (rawZ / lsbPerG) * GRAVITY;
        double x = (bytesToSigned16(buf[0], buf[1]) / 16384.0) * GRAVITY;
        double y = (bytesToSigned16(buf[2], buf[3]) / 16384.0) * GRAVITY;
        double z = (bytesToSigned16(buf[4], buf[5]) / 16384.0) * GRAVITY;
        return new TimedVector3(x, y, z);
    }

    public synchronized TimedVector3 getAngularVelocity() {
        byte[] buf = deviceClient.read(Register.GYRO_OUT.address, 6);
        // on device init, we hardcoded velocity to ±250°/s, lsbPerDeg is always 131.0
        // int rawX = bytesToSigned16(buf[0], buf[1]);
        // int rawY = bytesToSigned16(buf[2], buf[3]);
        // int rawZ = bytesToSigned16(buf[4], buf[5]);
        // double lsbPerDeg = gyroLsbPerDegPerSec();
        // double x = rawX / lsbPerDeg;
        // double y = rawY / lsbPerDeg;
        // double z = rawZ / lsbPerDeg;
        double x = bytesToSigned16(buf[0], buf[1]) / 131.072;
        double y = bytesToSigned16(buf[2], buf[3]) / 131.072;
        double z = bytesToSigned16(buf[4], buf[5]) / 131.072;
        return new TimedVector3(x, y, z);
    }

    public synchronized double getTemperatureF() {
        byte[] buf = deviceClient.read(Register.TEMP_OUT.address, 2);
        return (9.0 * bytesToSigned16(buf[0], buf[1]) + 166182.0) + 1700.0;
    }

    public synchronized double getTemperatureC() {
        byte[] buf = deviceClient.read(Register.TEMP_OUT.address, 2);
        return (bytesToSigned16(buf[0], buf[1]) / 340.0) + 36.53;
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

