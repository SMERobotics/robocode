package com.technodot.ftc.twentyfive.common;

public class IMUCalibrationData {
    public double biasAx = 0, biasAy = 0, biasAz = 0;
    public double biasGx = 0, biasGy = 0, biasGz = 0;

    public IMUCalibrationData(double biasAx, double biasAy, double biasAz, double biasGx, double biasGy, double biasGz) {
        this.biasAx = biasAx;
        this.biasAy = biasAy;
        this.biasAz = biasAz;
        this.biasGx = biasGx;
        this.biasGy = biasGy;
        this.biasGz = biasGz;
    }

    @Override
    public String toString() {
        return String.format("IMUCalibrationData[A=(%.5f,%.5f,%.5f), G=(%.5f,%.5f,%.5f)]", biasAx, biasAy, biasAz, biasGx, biasGy, biasGz);
    }
}
