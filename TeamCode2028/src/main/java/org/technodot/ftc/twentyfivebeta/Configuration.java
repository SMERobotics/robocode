package org.technodot.ftc.twentyfivebeta;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// FUCK

/*
1969-12-31 18:04:05.944  1956-1956  AnnotatedH...lassFilter com.qualcomm.ftcrobotcontroller      E  failure while calling OnCreateMethods annotated method class com.acmerobotics.dashboard.FtcDashboard.start
1969-12-31 18:04:05.945  1956-1956  AnnotatedH...lassFilter com.qualcomm.ftcrobotcontroller      E  java.lang.reflect.InvocationTargetException
1969-12-31 18:04:05.946  1956-1956  AnnotatedH...lassFilter com.qualcomm.ftcrobotcontroller      E  	at java.lang.reflect.Method.invoke(Native Method)
1969-12-31 18:04:05.947  1956-1956  AnnotatedH...lassFilter com.qualcomm.ftcrobotcontroller      E  	at org.firstinspires.ftc.ftccommon.internal.AnnotatedHooksClassFilter.callOnCreateMethods(AnnotatedHooksClassFilter.java:133)
1969-12-31 18:04:05.947  1956-1956  AnnotatedH...lassFilter com.qualcomm.ftcrobotcontroller      E  	at org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.onCreate(FtcRobotControllerActivity.java:411)
1969-12-31 18:04:05.947  1956-1956  AnnotatedH...lassFilter com.qualcomm.ftcrobotcontroller      E  	at android.app.Activity.performCreate(Activity.java:6709)
1969-12-31 18:04:05.948  1956-1956  AnnotatedH...lassFilter com.qualcomm.ftcrobotcontroller      E  	at android.app.Instrumentation.callActivityOnCreate(Instrumentation.java:1118)
1969-12-31 18:04:05.948  1956-1956  AnnotatedH...lassFilter com.qualcomm.ftcrobotcontroller      E  	at android.app.ActivityThread.performLaunchActivity(ActivityThread.java:2619)
1969-12-31 18:04:05.948  1956-1956  AnnotatedH...lassFilter com.qualcomm.ftcrobotcontroller      E  	at android.app.ActivityThread.handleLaunchActivity(ActivityThread.java:2727)
1969-12-31 18:04:05.949  1956-1956  AnnotatedH...lassFilter com.qualcomm.ftcrobotcontroller      E  	at android.app.ActivityThread.-wrap12(ActivityThread.java)
1969-12-31 18:04:05.949  1956-1956  AnnotatedH...lassFilter com.qualcomm.ftcrobotcontroller      E  	at android.app.ActivityThread$H.handleMessage(ActivityThread.java:1478)
1969-12-31 18:04:05.949  1956-1956  AnnotatedH...lassFilter com.qualcomm.ftcrobotcontroller      E  	at android.os.Handler.dispatchMessage(Handler.java:102)
1969-12-31 18:04:05.950  1956-1956  AnnotatedH...lassFilter com.qualcomm.ftcrobotcontroller      E  	at android.os.Looper.loop(Looper.java:154)
1969-12-31 18:04:05.950  1956-1956  AnnotatedH...lassFilter com.qualcomm.ftcrobotcontroller      E  	at android.app.ActivityThread.main(ActivityThread.java:6121)
1969-12-31 18:04:05.950  1956-1956  AnnotatedH...lassFilter com.qualcomm.ftcrobotcontroller      E  	at java.lang.reflect.Method.invoke(Native Method)
1969-12-31 18:04:05.950  1956-1956  AnnotatedH...lassFilter com.qualcomm.ftcrobotcontroller      E  	at com.android.internal.os.ZygoteInit$MethodAndArgsCaller.run(ZygoteInit.java:905)
1969-12-31 18:04:05.951  1956-1956  AnnotatedH...lassFilter com.qualcomm.ftcrobotcontroller      E  	at com.android.internal.os.ZygoteInit.main(ZygoteInit.java:795)
1969-12-31 18:04:06.183  1956-1956  AnnotatedH...lassFilter com.qualcomm.ftcrobotcontroller      E  Caused by: java.lang.StackOverflowError: stack size 8MB
1969-12-31 18:04:06.183  1956-1956  AnnotatedH...lassFilter com.qualcomm.ftcrobotcontroller      E  	at java.util.ArrayList.ensureExplicitCapacity(ArrayList.java:223)
1969-12-31 18:04:06.184  1956-1956  AnnotatedH...lassFilter com.qualcomm.ftcrobotcontroller      E  	at java.util.ArrayList.ensureCapacityInternal(ArrayList.java:215)
1969-12-31 18:04:06.184  1956-1956  AnnotatedH...lassFilter com.qualcomm.ftcrobotcontroller      E  	at java.util.ArrayList.add(ArrayList.java:441)
1969-12-31 18:04:06.184  1956-1956  AnnotatedH...lassFilter com.qualcomm.ftcrobotcontroller      E  	at java.util.Collections.addAll(Collections.java:4600)
1969-12-31 18:04:06.185  1956-1956  AnnotatedH...lassFilter com.qualcomm.ftcrobotcontroller      E  	at java.lang.Class.getPublicFieldsRecursive(Class.java:1360)
1969-12-31 18:04:06.185  1956-1956  AnnotatedH...lassFilter com.qualcomm.ftcrobotcontroller      E  	at java.lang.Class.getFields(Class.java:1349)
1969-12-31 18:04:06.185  1956-1956  AnnotatedH...lassFilter com.qualcomm.ftcrobotcontroller      E  	at com.acmerobotics.dashboard.config.reflection.ReflectionConfig.createVariableFromField(ReflectionConfig.java:114)
1969-12-31 18:04:06.186  1956-1956  AnnotatedH...lassFilter com.qualcomm.ftcrobotcontroller      E  	at com.acmerobotics.dashboard.config.reflection.ReflectionConfig.createVariableFromField(ReflectionConfig.java:121)
1969-12-31 18:04:06.187  1956-1956  AnnotatedH...lassFilter com.qualcomm.ftcrobotcontroller      E  	at com.acmerobotics.dashboard.config.reflection.ReflectionConfig.createVariableFromField(ReflectionConfig.java:121)
1969-12-31 18:04:06.188  1956-1956  AnnotatedH...lassFilter com.qualcomm.ftcrobotcontroller      E  	at com.acmerobotics.dashboard.config.reflection.ReflectionConfig.createVariableFromField(ReflectionConfig.java:121)
 */

// @Config
@Configurable
public class Configuration {
    public static volatile boolean DEBUG = true; // all targets override debug to false (should)

    public static volatile double DRIVE_CONTROLLER_DEADZONE = 0.02;
    public static volatile int DRIVE_MOTOR_FORWARD_TILE_TICKS = 1337; // as of 01/09/2026
    public static volatile int DRIVE_MOTOR_STRAFE_TILE_TICKS = 1521; // as of 01/09/2026
    public static volatile int DRIVE_MOTOR_ROTATE_CIRCLE_TICKS = 3939; // as of 01/09/2026
    public static volatile int DRIVE_MOTOR_CONTROL_TOLERANCE_TICKS = 20;
    public static volatile long DRIVE_MOTOR_CONTROL_DEBOUNCE_MS = 100;
    public static volatile double DRIVE_AUTO_MAX_VELOCITY = 1600;
    public static volatile double DRIVE_MOTOR_ACTIVATION = 0.2;
    public static volatile double DRIVE_SPEED_MULTIPLIER = 1.0;
    public static volatile double DRIVE_STRAFE_MULTIPLIER = 1.3; // 1.10 +/- 0.01 idfk? Y TF NOT
//    public static volatile double DRIVE_STRAFE_MULTIPLIER = 1.0; // TODO: fuck it just test it
    public static volatile double DRIVE_AIM_OFFSET = 2.0; // degrees, blue goal to the left, red goal to the right. // THIS VALUE ALSO NEEDS TO BE CHANGED IN TELEOP AND AUTO
//    public static volatile double DRIVE_AIM_INTAKE_OFFSET = 1.3639;
    public static volatile double DRIVE_AIM_INTAKE_OFFSET = 3.0;
    public static volatile double DRIVE_AIM_KP = 0.023;
    public static volatile double DRIVE_AIM_KI = 0.067;
    public static volatile double DRIVE_AIM_KD = 0.002;
    public static volatile double DRIVE_AIM_KF = 0.0;
    public static volatile double DRIVE_AIM_INTEGRATION_BOUNDS = 1.0;
//    public static volatile double DRIVE_AIM_KD = 0.000067;
    public static volatile double DRIVE_AIM_TOLERANCE = 1.0;
    public static volatile double DRIVE_ROTATE_KP = 0.03;
    public static volatile double DRIVE_ROTATE_KI = 0.0;
    public static volatile double DRIVE_ROTATE_KD = 0.0001;
    public static volatile double DRIVE_ROTATE_KF = 0.0;
    public static volatile double DRIVE_ROTATE_TOLERANCE = 1.0;
    public static volatile double DRIVE_FORWARD_KP = -0.2;
    public static volatile double DRIVE_FORWARD_KI = 0.0;
    public static volatile double DRIVE_FORWARD_KD = 0.0;
    public static volatile double DRIVE_FORWARD_KF = 0.0;
    public static volatile double DRIVE_STRAFE_KP = 0.0;
    public static volatile double DRIVE_STRAFE_KI = 0.0;
    public static volatile double DRIVE_STRAFE_KD = 0.0;
    public static volatile double DRIVE_STRAFE_KF = 0.0;
    public static volatile long DRIVE_ROTATE_SNAPSHOT_DELAY_NS = 100_000000L;

    public static volatile double INTAKE_MOTOR_NUDGE_POWER = -0.67;
    public static volatile int INTAKE_MOTOR_NUDGE_TICKS = -80;
    public static volatile double INTAKE_LEFT_DEACTIVATION = 0.57;
    public static volatile double INTAKE_LEFT_HOLD = INTAKE_LEFT_DEACTIVATION + 0.03;
    public static volatile double INTAKE_LEFT_ACTIVATION = INTAKE_LEFT_DEACTIVATION + 0.22;
    public static volatile double INTAKE_RIGHT_DEACTIVATION = 0.57;
    public static volatile double INTAKE_RIGHT_HOLD = INTAKE_RIGHT_DEACTIVATION - 0.03;
    public static volatile double INTAKE_RIGHT_ACTIVATION = INTAKE_RIGHT_DEACTIVATION - 0.22;
    public static volatile long INTAKE_SERVO_INTERVAL_MS = 200;
//    public static volatile long INTAKE_SERVO_DELAY_MS = 500; // global delay for now, PREVIOUSLY WAS 670MS
    public static volatile long INTAKE_SERVO_SHORT_DELAY_MS = 400;
//    public static volatile long INTAKE_SERVO_LONG_DELAY_MS = 670;
    public static volatile long INTAKE_SERVO_LONG_DELAY_MS = 670;
    public static volatile long INTAKE_AUTO_RELOAD_DELAY_MS = 500;
//    public static volatile long INTAKE_AUTO_RELOAD_DURATION_MS = 1000;
    public static volatile long INTAKE_AUTO_RELOAD_DURATION_MS = 1400;

    public static volatile double EXTAKE_MOTOR_KP = 150.0;
    public static volatile double EXTAKE_MOTOR_KI = 0.0;
    public static volatile double EXTAKE_MOTOR_KD = 0.0;
    public static volatile double EXTAKE_MOTOR_KF = 16.9;
    public static volatile double EXTAKE_MOTOR_SPEED_SHORT = 1200; // TODO: adjust
    public static volatile double EXTAKE_MOTOR_SPEED_DUAL_SHORT = 1400; // TODO: adjust
    public static volatile double EXTAKE_MOTOR_SPEED_LONG = 1520; // TODO: adjust
    public static volatile double EXTAKE_MOTOR_SPEED_TOLERANCE = 60;
    public static volatile double EXTAKE_MOTOR_SUPER_FEEDFORWARD_THRESHOLD = 60;
    public static volatile double EXTAKE_DUAL_TRANSITION_MS = 200;
    public static volatile double EXTAKE_RANGE_STABILITY_THRESHOLD = 3.0; // inches
    public static volatile long EXTAKE_RANGE_STABILITY_DURATION_MS = 150;
//    public static volatile int EXTAKE_STABILIZATION_CYCLES = 4;
    public static volatile int EXTAKE_STABILIZATION_CYCLES = 1; // TODO: test zero because human and intake latency should be enough time
    // y (ticks/sec) = -0.0306x^2 + 12.8479 x + 415.9836

    // ORIGINAL CALIB
    public static volatile double EXTAKE_MODEL_VELOCITY_SIMPLE_A = -0.00135941391;
    public static volatile double EXTAKE_MODEL_VELOCITY_SIMPLE_B = 5.06119602;
    public static volatile double EXTAKE_MODEL_VELOCITY_SIMPLE_C = 874.146043;

    // UPDATED QUADRATIC REGRESSION
//    public static volatile double EXTAKE_MODEL_VELOCITY_SIMPLE_A = 0.0482869;
//    public static volatile double EXTAKE_MODEL_VELOCITY_SIMPLE_B = 3.82284;
//    public static volatile double EXTAKE_MODEL_VELOCITY_SIMPLE_C = 1179.6397;

    // LINEAR REGRESSION
//    public static volatile double EXTAKE_MODEL_VELOCITY_SIMPLE_A = 0.0;
//    public static volatile double EXTAKE_MODEL_VELOCITY_SIMPLE_B = 4.88586;
//    public static volatile double EXTAKE_MODEL_VELOCITY_SIMPLE_C = 837.86722;

    // DOUBLE FIRST CALIB
    public static volatile double EXTAKE_MODEL_VELOCITY_DOUBLE_A = -0.0147856;
    public static volatile double EXTAKE_MODEL_VELOCITY_DOUBLE_B = 6.12754;
    public static volatile double EXTAKE_MODEL_VELOCITY_DOUBLE_C = 1038.92235;

    public static volatile double EXTAKE_MODEL_VELOCITY_SIMPLE_RANGE_SHIFT = 0.0; // + forward, - backwards // THIS VALUE NEEDS TO ALSO BE CHANGED IN TELEOP AND AUTO
    public static volatile double EXTAKE_MODEL_VELOCITY_PINPOINT_A = 0.0460686;
    public static volatile double EXTAKE_MODEL_VELOCITY_PINPOINT_B = -4.1204;
    public static volatile double EXTAKE_MODEL_VELOCITY_PINPOINT_C = 1166.41746;
    public static volatile double EXTAKE_MODEL_VELOCITY_PINPOINT_RANGE_SHIFT = 0.0;
    // y (ticks/sec) = 340.130269893 x (m/sec) - 783.609122293 @ r = 0.9969, R^2 = 0.9937
    public static volatile double EXTAKE_MODEL_VELOCITY_M = 340.130269893;
    public static volatile double EXTAKE_MODEL_VELOCITY_B = -783.609122293;
    // y (deg) = -9.04083 x (m/sec) + 109.38517 @ r = -0.9975, R^2 = 0.9950
    public static volatile double EXTAKE_MODEL_ANGLE_M = -9.04083;
    public static volatile double EXTAKE_MODEL_ANGLE_B = 109.38517;

    public static volatile int GAMEPAD_RUMBLE_DURATION_MS = 200; // rev ps4 gamepad is the worst gamepad ever, negative build quality, need the dualshock 5
    public static volatile int GAMEPAD_RUMBLE_STRONG_MS = 500;
    public static volatile int GAMEPAD_RUMBLE_WEAK_MS = 100;
    public static volatile int GAMEPAD_RUMBLE_FINALE_MS = 6767;

    public static volatile double PINPOINT_HEADING_P = 0.01;
    public static volatile double PINPOINT_HEADING_I = 0;
    public static volatile double PINPOINT_HEADING_D = 0;
    public static volatile double PINPOINT_HEADING_F = 0.04;
    public static volatile double PINPOINT_AIM_P = 0.03;
    public static volatile double PINPOINT_AIM_I = 0;
    public static volatile double PINPOINT_AIM_D = 0.003;
    public static volatile double PINPOINT_AIM_F = 0.08;
    public static volatile double PINPOINT_BEARING_P = 0.01;
    public static volatile double PINPOINT_BEARING_I = 0;
    public static volatile double PINPOINT_BEARING_D = 0.0015;
    public static volatile double PINPOINT_BEARING_F = 0.12;
    public static volatile double PINPOINT_ANGLE_OFFSET = -2;
    public static volatile double PINPOINT_ANGLE_SIDE_OFFSET = 2;

    public static volatile int SHOTSOLVER_POSE_WINDOW_SIZE = 9;
    public static volatile double SHOTSOLVER_POSE_EMA_ALPHA = 0.67;
    public static volatile double SHOTSOLVER_POSE_MAX_STEP_IN = 1;
    public static volatile double SHOTSOLVER_POSE_OUTLIER_REJECT_IN = 6.7;
    public static volatile double SHOTSOLVER_YAW_ERROR_EMA_ALPHA = 1;

    // FUCK MANUAL MEASUREMENTS
//    public static volatile double PINPOINT_OFFSET_FORWARD_Y = -2.1673433165342573; // gemini-3.0-flash-preview (thinking)
//    public static volatile double PINPOINT_OFFSET_STRAFE_X = -1.5354035333572047; // gemini-3.0-flash-preview (thinking)
    public static volatile double PINPOINT_OFFSET_FORWARD_Y = -1.9974842264556827; // gpt-5.2-thinking
    public static volatile double PINPOINT_OFFSET_STRAFE_X = -1.6029825263449966; // gpt-5.2-thinking
//    public static volatile double PINPOINT_OFFSET_FORWARD_Y = 2.784; // 1.92+(1.728/2), measured as of 2/3/2026
//    public static volatile double PINPOINT_OFFSET_STRAFE_X = -0.4715; // (1.971/2)-0.514, measured as of 2/8/2026
    public static volatile GoBildaPinpointDriver.EncoderDirection PINPOINT_DIRECTION_FORWARD = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static volatile GoBildaPinpointDriver.EncoderDirection PINPOINT_DIRECTION_STRAFE = GoBildaPinpointDriver.EncoderDirection.REVERSED;

    public static volatile double ROBOT_LENGTH = 17.13; // front to back distance, in inches // TODO: guesstimated
    public static volatile double ROBOT_WIDTH = 16.81; // left to right distance, in inches // TODO: guesstimated
//    public static volatile double LOCALIZER_LENGTH_FRONT_OFFSET = ROBOT_LENGTH - 6.9255; // (1.971/2)+5.94, measured as of 2/8/2026 // front-left to localizer distance along robot length axis, in inches // (1.971/2)+5.94
//    public static volatile double LOCALIZER_WIDTH_LEFT_OFFSET = 8.3835; // (1.728/2)+5.6715+1.848, measured as of 2/8/2026 // front-left to localizer distance along robot width axis, in inches
//    public static volatile double LOCALIZER_LENGTH_BACK_OFFSET = ROBOT_LENGTH - LOCALIZER_LENGTH_FRONT_OFFSET;
//    public static volatile double LOCALIZER_WIDTH_RIGHT_OFFSET = ROBOT_WIDTH - LOCALIZER_WIDTH_LEFT_OFFSET;
    public static volatile double LOCALIZER_LENGTH_FRONT_OFFSET = 11.60;
    public static volatile double LOCALIZER_WIDTH_LEFT_OFFSET = ROBOT_WIDTH / 2;
    public static volatile double LOCALIZER_LENGTH_BACK_OFFSET = 5.54;
    public static volatile double LOCALIZER_WIDTH_RIGHT_OFFSET = ROBOT_WIDTH / 2;

    public static final PinpointConfig ODOMETRY_LOCALIZER_PINPOINT_CONSTANTS = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.xPodOffset.set(PINPOINT_OFFSET_FORWARD_Y);
        c.yPodOffset.set(PINPOINT_OFFSET_STRAFE_X);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        c.xPodDirection.set(PINPOINT_DIRECTION_FORWARD);
        c.yPodDirection.set(PINPOINT_DIRECTION_STRAFE);
        c.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
    });

    public static final MecanumConfig ODOMETRY_DRIVE_MECANUM_CONSTANTS = new MecanumConfig(c -> {
        c.frontLeftName.set("motorFrontLeft");
        c.frontRightName.set("motorFrontRight");
        c.backLeftName.set("motorBackLeft");
        c.backRightName.set("motorBackRight");
        c.frontLeftDirection.set(DcMotorEx.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorEx.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorEx.Direction.REVERSE);
        c.backRightDirection.set(DcMotorEx.Direction.FORWARD);
    });

    // Live feedback/feedforward tuning through Dashboard and Panels.
    public static volatile double ODOMETRY_HEADING_P = 1;
    public static volatile double ODOMETRY_HEADING_D = 0.067;
    public static volatile double ODOMETRY_HEADING_STATIC_FF = 0.04;
    public static volatile double ODOMETRY_TRANSLATIONAL_P = 0.067;
    public static volatile double ODOMETRY_TRANSLATIONAL_D = 0.01;
    public static volatile double ODOMETRY_TRANSLATIONAL_STATIC_FF = 0.03;
    public static volatile double ODOMETRY_BRAKE_VELOCITY_FF = 1 / 72.367958;
    public static volatile double ODOMETRY_COAST_VELOCITY_FF = 1 / 72.367958;

    // Pedro 3 uses Foresight instead of the Pedro 2 mass/centripetal/drive PID model.
    // These controller gains are starting values; validate Foresight on the robot.
    public static ForesightConfig createFollowerConfig() {
        return new ForesightConfig(c -> {
            c.maxAchievableForwardVelocity.set(72.367958);
            c.maxAchievableStrafeVelocity.set(55.709416);
            c.naturalForwardDeceleration.set(55.351116);
            c.naturalStrafeDeceleration.set(80.316819);
            c.headingFeedback.set(Controller.proportional(() -> ODOMETRY_HEADING_P)
                    .plus(Controller.derivative(() -> ODOMETRY_HEADING_D)));
            c.headingStaticFF.set(Controller.staticFeedforward(() -> ODOMETRY_HEADING_STATIC_FF));
            c.forwardTranslational.set(Controller.proportional(() -> ODOMETRY_TRANSLATIONAL_P)
                    .plus(Controller.derivative(() -> ODOMETRY_TRANSLATIONAL_D))
                    .plus(Controller.staticFeedforward(() -> ODOMETRY_TRANSLATIONAL_STATIC_FF)));
            c.strafeTranslational.set(Controller.proportional(() -> ODOMETRY_TRANSLATIONAL_P)
                    .plus(Controller.derivative(() -> ODOMETRY_TRANSLATIONAL_D))
                    .plus(Controller.staticFeedforward(() -> ODOMETRY_TRANSLATIONAL_STATIC_FF)));
            // Foresight brake/coast controllers receive target speed, not the old drive PID error.
            c.brake.set(Controller.proportionalFeedforward(() -> ODOMETRY_BRAKE_VELOCITY_FF));
            c.coast.set(Controller.proportionalFeedforward(() -> ODOMETRY_COAST_VELOCITY_FF));
            // Constant-deceleration estimate: stopping distance = v^2 / (2a).
            // Replace with measured Foresight brake model coefficients after tuning.
            c.linearBrakeCoefficients.set(Matrix.zero(2));
            c.quadraticBrakeCoefficients.set(Matrix.diag(1 / (2 * 55.351116), 1 / (2 * 80.316819)));
            c.headingBrakeCoefficients.set(Vector2D.zero());
            c.parametricTConstraint.set(0.01);
            c.timeoutConstraint.set(100.0);
        });
    }

    public static org.technodot.ftc.twentyfivebeta.pedro.Follower createFollower(HardwareMap hardwareMap) {
        return new org.technodot.ftc.twentyfivebeta.pedro.Follower(
                new PinpointLocalizer(hardwareMap, ODOMETRY_LOCALIZER_PINPOINT_CONSTANTS),
                new Mecanum(hardwareMap, ODOMETRY_DRIVE_MECANUM_CONSTANTS),
                new Foresight(createFollowerConfig()));
    }
}
