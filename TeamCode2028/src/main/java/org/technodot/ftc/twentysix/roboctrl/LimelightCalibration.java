package org.technodot.ftc.twentysix.roboctrl;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;

/** Reads the selected camera calibration, including firmware's CALIBRATION_RESULT envelope. */
public final class LimelightCalibration {
    private LimelightCalibration() { }

    public static HiveSolver.Calibration fetch(URL url, int width, int height, boolean undistorted) throws IOException {
        HttpURLConnection connection = (HttpURLConnection) url.openConnection();
        connection.setConnectTimeout(500);
        connection.setReadTimeout(1000);
        connection.setUseCaches(false);
        try {
            int status = connection.getResponseCode();
            if (status != HttpURLConnection.HTTP_OK) {
                throw new IOException(url.getPath() + " returned HTTP " + status);
            }
            StringBuilder json = new StringBuilder();
            try (BufferedReader reader = new BufferedReader(new InputStreamReader(
                    connection.getInputStream(), StandardCharsets.UTF_8))) {
                String line;
                while ((line = reader.readLine()) != null) json.append(line);
            }
            if (json.toString().trim().isEmpty()) {
                throw new IOException(url.getPath() + " is empty; selected calibration is not saved on the camera");
            }
            try {
                return parse(json.toString(), width, height, undistorted);
            } catch (JSONException | IllegalArgumentException error) {
                throw new IOException(url.getPath() + ": " + error.getMessage(), error);
            }
        } finally {
            connection.disconnect();
        }
    }

    public static HiveSolver.Calibration parse(String json, int width, int height, boolean undistorted) throws JSONException {
        JSONObject data = new JSONObject(json);
        // SDK 12's CalibrationResult parser expects only the inner object.
        if (data.has("CALIBRATION_RESULT")) data = data.getJSONObject("CALIBRATION_RESULT");
        int sourceWidth = resolution(data, "RES_X"), sourceHeight = resolution(data, "RES_Y");
        double[] k = vector(data, "INTRINSICS_MATRIX", 9);
        double[] distortion = vector(data, "DISTORTION_COEFFICIENTS", 5);
        if (Math.abs(k[1]) > 1e-6 || Math.abs(k[3]) > 1e-6
                || Math.abs(k[6]) > 1e-6 || Math.abs(k[7]) > 1e-6 || Math.abs(k[8] - 1) > 1e-6) {
            throw new IllegalArgumentException("Unsupported calibration matrix");
        }
        return new HiveSolver.Calibration(sourceWidth, sourceHeight, k[0], k[4], k[2], k[5],
                undistorted ? new double[5] : distortion).scaledTo(width, height);
    }

    private static int resolution(JSONObject data, String key) throws JSONException {
        double value = data.getDouble(key);
        if (!Double.isFinite(value) || value <= 0 || value > Integer.MAX_VALUE || value != Math.rint(value)) {
            throw new IllegalArgumentException("Invalid " + key);
        }
        return (int) value;
    }

    private static double[] vector(JSONObject data, String key, int length) throws JSONException {
        JSONArray array = data.getJSONArray(key);
        if (array.length() != length) throw new IllegalArgumentException(key + " needs " + length + " values");
        double[] values = new double[length];
        for (int i = 0; i < length; i++) {
            values[i] = array.getDouble(i);
            if (!Double.isFinite(values[i])) throw new IllegalArgumentException("Invalid " + key);
        }
        return values;
    }
}
