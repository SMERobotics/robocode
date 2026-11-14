package org.technodot.ftc.twentysix.roboctrl;

import org.technodot.ftc.twentysix.common.Alliance;

/** BIOBUZZ v26-27.2 CAD, production artwork V1, and FTC SDK 12 cluster origins.
 * All solver lengths are meters. The neutral hive frame has X along the axle
 * toward blue, Y toward the scoring table, and Z up. Its origin is on the tag
 * plane below the axle. See docs/hive-vision.md for the derivation and sources.
 */
public final class HiveGeometry {
    public static final double INCH = 0.0254;
    public static final double TAG_SIZE = 3.25 * INCH;
    public static final double STOP_RADIANS = Math.toRadians(30);
    public static final double PIVOT_HEIGHT = 1.498167842 * INCH;
    public static final double ROW_DISTANCE = 14.269059480 * INCH;
    public static final double ROW_CENTER_X = 0.000944882 * INCH;
    public static final double FOCUS_DOWN = -7.1874 * INCH;
    public static final double FOCUS_INTO_CELL = 5.622 * INCH;
    private static final double[] TAG_X = {-6.50, -2.75, 2.75, 6.50};

    public enum Cell {
        RED_SCORING(Alliance.RED, 30, 1),
        RED_AUDIENCE(Alliance.RED, 34, -1),
        BLUE_AUDIENCE(Alliance.BLUE, 38, -1),
        BLUE_SCORING(Alliance.BLUE, 42, 1);

        public final Alliance alliance;
        public final int firstTagId;
        /** -1 = audience, +1 = scoring table. */
        public final int side;

        Cell(Alliance alliance, int firstTagId, int side) {
            this.alliance = alliance;
            this.firstTagId = firstTagId;
            this.side = side;
        }

        public double[] focus(double rightInches, double downInches, double intoCellInches) {
            double printedDirection = -side;
            return new double[] {
                    ROW_CENTER_X + printedDirection * rightInches * INCH,
                    side * ROW_DISTANCE + printedDirection * (FOCUS_DOWN + downInches * INCH),
                    FOCUS_INTO_CELL + intoCellInches * INCH
            };
        }
    }

    public static Cell cellForTag(int id) {
        for (Cell cell : Cell.values()) {
            if (id >= cell.firstTagId && id < cell.firstTagId + 4) return cell;
        }
        return null;
    }

    public static Cell cellForSide(Alliance alliance, int side) {
        for (Cell cell : Cell.values()) {
            if (cell.alliance == alliance && cell.side == side) return cell;
        }
        throw new IllegalArgumentException("Unknown hive side");
    }

    /** AprilTag decoded corner order: printed BL, BR, TR, TL (never screen sort). */
    public static double[][] corners(int id) {
        Cell cell = cellForTag(id);
        if (cell == null) throw new IllegalArgumentException("Not a BIOBUZZ tag: " + id);
        double d = -cell.side;
        double x = ROW_CENTER_X + d * TAG_X[id - cell.firstTagId] * INCH;
        double y = cell.side * ROW_DISTANCE;
        double h = TAG_SIZE / 2;
        return new double[][] {
                {x - d * h, y + d * h, 0}, {x + d * h, y + d * h, 0},
                {x + d * h, y - d * h, 0}, {x - d * h, y - d * h, 0}
        };
    }

    private HiveGeometry() { }
}
