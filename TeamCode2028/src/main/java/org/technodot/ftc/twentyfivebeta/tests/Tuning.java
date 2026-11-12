package org.technodot.ftc.twentyfivebeta.tests;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.technodot.ftc.twentyfivebeta.Configuration;
import org.technodot.ftc.twentyfivebeta.common.Drawing;
import org.technodot.ftc.twentyfivebeta.pedro.Follower;
import java.util.List;
import static com.pedropathing.api.Paths.*;

/** Pedro 3 localization, motion measurements, and Foresight validation. */
@Config
@Disabled
@TeleOp(name = "Tuning", group = "Pedro Pathing")
public class Tuning extends SelectableOpMode {
    public static double DISTANCE = 40;
    public static double POWER = 0.5;
    private static Follower follower;

    public Tuning() {
        super("Select a Tuning OpMode", s -> {
            s.folder("Localization", l -> {
                l.add("Localization Test", () -> new ManualTest(0));
                l.add("Forward Tuner", () -> new ManualTest(1));
                l.add("Lateral Tuner", () -> new ManualTest(2));
                l.add("Turn Tuner", () -> new ManualTest(3));
            });
            s.folder("Measurements", l -> {
                l.add("Forward Velocity", () -> new Measurement(false, false));
                l.add("Lateral Velocity", () -> new Measurement(true, false));
                l.add("Forward Deceleration", () -> new Measurement(false, true));
                l.add("Lateral Deceleration", () -> new Measurement(true, true));
            });
            s.folder("Foresight", l -> {
                l.add("Translational Hold", () -> new HoldTest(false));
                l.add("Heading Hold", () -> new HoldTest(true));
                l.add("Line", () -> new PathTest(0));
                l.add("Triangle", () -> new PathTest(1));
                l.add("Circle", () -> new PathTest(2));
            });
        });
    }

    @Override public void onSelect() {
        follower = Configuration.createFollower(hardwareMap);
        follower.setPose(new Pose(72, 72, 0));
        Drawing.init();
    }

    @Override public void onLog(List<String> lines) { }

    private static ForesightConfig config() {
        return ((Foresight) follower.algorithm()).config;
    }

    private static abstract class Test extends OpMode {
        @Override public void init() {
            follower.stop();
            follower.setPose(new Pose(72, 72, 0));
        }
        void display() {
            telemetry.addData("Pose", follower.pose());
            telemetry.addData("Velocity", follower.velocity());
            telemetry.addData("Foresight", ((Foresight) follower.algorithm()).debugData());
            Drawing.draw(follower);
        }
        @Override public void stop() { follower.stop(); }
    }

    private static class ManualTest extends Test {
        private final int axis;
        ManualTest(int axis) { this.axis = axis; }
        @Override public void loop() {
            follower.manual(ManualDrive.fieldCentric(-gamepad1.left_stick_y,
                    -gamepad1.left_stick_x, -gamepad1.right_stick_x, follower.pose().heading()));
            follower.update();
            telemetry.addData("Instructions", "Drive manually; compare measured travel/turn with the reported pose.");
            if (axis == 1) telemetry.addData("Forward travel (in)", follower.pose().x() - 72);
            if (axis == 2) telemetry.addData("Lateral travel (in)", follower.pose().y() - 72);
            if (axis == 3) telemetry.addData("Heading (rad)", follower.pose().heading());
            display();
        }
    }

    /** Hold A to drive, release to coast. B applies the measured value to this session. */
    private static class Measurement extends Test {
        private final boolean lateral;
        private final boolean deceleration;
        private boolean driving;
        private boolean sampling;
        private boolean previousB;
        private double lastSpeed;
        private long lastTime;
        private double total;
        private int samples;

        Measurement(boolean lateral, boolean deceleration) {
            this.lateral = lateral;
            this.deceleration = deceleration;
        }
        @Override public void init() {
            super.init();
            driving = sampling = previousB = false;
            total = 0;
            samples = 0;
        }
        @Override public void loop() {
            boolean pressed = gamepad1.a;
            if (pressed && !driving) {
                total = 0;
                samples = 0;
                sampling = false;
            }
            if (!pressed && driving && deceleration) sampling = true;
            follower.manual(pressed && !lateral ? POWER : 0, pressed && lateral ? POWER : 0, 0);
            // Float only while measuring natural deceleration; stop() restores braking.
            follower.update();
            if (!pressed && deceleration) follower.drivetrain.stop(false);
            long now = System.nanoTime();
            Vector2D body = follower.velocity().toVector2D().toBodyFrame(follower.pose().heading());
            double speed = Math.abs(lateral ? body.y() : body.x());
            double dt = (now - lastTime) / 1e9;
            if (!deceleration && pressed && speed > 0) {
                total += speed;
                samples++;
            } else if (sampling && dt > 0 && dt < 0.25 && lastSpeed > speed && speed > 1) {
                total += (lastSpeed - speed) / dt;
                samples++;
            }
            if (speed <= 1) sampling = false;
            double average = samples == 0 ? 0 : total / samples;
            if (gamepad1.b && !previousB && average > 0) {
                if (deceleration) {
                    (lateral ? config().naturalStrafeDeceleration : config().naturalForwardDeceleration).set(average);
                } else {
                    // Velocity measurements must use full power to characterize maximum speed.
                    if (POWER == 1) {
                        (lateral ? config().maxAchievableStrafeVelocity : config().maxAchievableForwardVelocity).set(average);
                    }
                }
            }
            telemetry.addData("Instructions", "Hold A to drive; release to coast. B applies to this session.");
            telemetry.addData("Power", POWER);
            telemetry.addData("Measurement", deceleration ? "in/s^2" : "in/s (set POWER=1 for max velocity)");
            telemetry.addData("Average", average);
            telemetry.addData("Samples", samples);
            display();
            driving = pressed;
            previousB = gamepad1.b;
            lastSpeed = speed;
            lastTime = now;
        }
    }

    private static class HoldTest extends Test {
        private final boolean heading;
        HoldTest(boolean heading) { this.heading = heading; }
        @Override public void start() { follower.hold(new Pose(72, 72, 0)); }
        @Override public void loop() {
            if (gamepad1.a) follower.hold(new Pose(72, 72, heading ? Math.PI / 2 : 0));
            if (gamepad1.b) follower.hold(new Pose(72, 72, 0));
            follower.update();
            telemetry.addData("Instructions", heading ? "A: hold 90 degrees; B: hold zero" : "Displace the robot to check translational feedback");
            display();
        }
    }

    private static class PathTest extends Test {
        private final int shape;
        private Path testPath;
        private boolean previousA;
        PathTest(int shape) { this.shape = shape; }
        @Override public void init() {
            super.init();
            Pose origin = new Pose(72, 72, 0);
            if (shape == 0) {
                testPath = path(line(origin, new Pose(72 + DISTANCE, 72)).constant(0),
                        line(new Pose(72 + DISTANCE, 72), origin).constant(0));
            } else if (shape == 1) {
                Pose first = new Pose(96, 48, Math.PI / 2);
                Pose second = new Pose(96, 96, Math.PI / 4);
                testPath = path(line(origin, first).linear(origin, first),
                        line(first, second).linear(first, second), line(second, origin).linear(second, origin));
            } else {
                double r = DISTANCE / 2;
                testPath = path(
                        curve(origin, new Pose(72 + r, 72), new Pose(72 + r, 72 + r)),
                        curve(new Pose(72 + r, 72 + r), new Pose(72 + r, 72 + 2*r), new Pose(72, 72 + 2*r)),
                        curve(new Pose(72, 72 + 2*r), new Pose(72 - r, 72 + 2*r), new Pose(72 - r, 72 + r)),
                        curve(new Pose(72 - r, 72 + r), new Pose(72 - r, 72), origin)).tangent();
            }
            previousA = false;
        }
        @Override public void start() { follower.followPath(testPath, POWER, true); }
        @Override public void loop() {
            if (gamepad1.a && !previousA) follower.followPath(testPath, POWER, true);
            if (gamepad1.b) follower.stop();
            follower.update();
            telemetry.addData("Instructions", "A: repeat path; B: stop");
            telemetry.addData("Busy", follower.isBusy());
            display();
            previousA = gamepad1.a;
        }
    }
}
