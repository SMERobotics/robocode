package org.technodot.ftc.twentysix.roboctrl;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;

import java.util.ArrayList;
import java.util.List;

/** Requires the SDK's opencv_java4 native library to have been loaded first. */
public final class OpenCvHivePoseSeeds implements HiveSolver.PoseSeeds {
    @Override
    public List<HiveSolver.Pose> solve(List<HiveSolver.TagObservation> tags, HiveSolver.Calibration calibration) {
        List<Point3> object = new ArrayList<>();
        List<Point> pixels = new ArrayList<>();
        for (HiveSolver.TagObservation tag : tags) {
            double[][] corners = HiveGeometry.corners(tag.id), image = tag.corners();
            for (int i = 0; i < 4; i++) {
                object.add(new Point3(corners[i][0], corners[i][1], corners[i][2]));
                pixels.add(new Point(image[i][0], image[i][1]));
            }
        }
        MatOfPoint3f points = new MatOfPoint3f();
        MatOfPoint2f imagePoints = new MatOfPoint2f();
        Mat camera = Mat.eye(3, 3, CvType.CV_64F);
        MatOfDouble distortion = new MatOfDouble(calibration.distortion());
        List<Mat> rotations = new ArrayList<>(), translations = new ArrayList<>();
        List<HiveSolver.Pose> result = new ArrayList<>();
        try {
            points.fromList(object); imagePoints.fromList(pixels);
            camera.put(0, 0, calibration.fx); camera.put(1, 1, calibration.fy);
            camera.put(0, 2, calibration.cx); camera.put(1, 2, calibration.cy);
            Calib3d.solvePnPGeneric(points, imagePoints, camera, distortion, rotations, translations,
                    false, Calib3d.SOLVEPNP_IPPE);
            for (int i = 0; i < rotations.size(); i++) {
                Mat matrix = new Mat();
                try {
                    Calib3d.Rodrigues(rotations.get(i), matrix);
                    double[][] r = new double[3][3];
                    double[] t = new double[3];
                    for (int row = 0; row < 3; row++) {
                        for (int col = 0; col < 3; col++) r[row][col] = matrix.get(row, col)[0];
                        t[row] = translations.get(i).get(row, 0)[0];
                    }
                    result.add(new HiveSolver.Pose(r, t));
                } finally { matrix.release(); }
            }
        } finally {
            points.release(); imagePoints.release(); camera.release(); distortion.release();
            for (Mat r : rotations) r.release();
            for (Mat t : translations) t.release();
        }
        return result;
    }
}
