package org.technodot.ftc.twentyfivebeta;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.paths.PathConstraints;
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

    public static volatile double INTAKE_MOTOR_NUDGE_POWER = 0.67;
    public static volatile int INTAKE_MOTOR_NUDGE_TICKS = -80;
    public static volatile double INTAKE_LEFT_DEACTIVATION = 0.59;
    public static volatile double INTAKE_LEFT_HOLD = INTAKE_LEFT_DEACTIVATION + 0.06;
    public static volatile double INTAKE_LEFT_ACTIVATION = INTAKE_LEFT_DEACTIVATION + 0.22;
    public static volatile double INTAKE_RIGHT_DEACTIVATION = 0.23;
    public static volatile double INTAKE_RIGHT_HOLD = INTAKE_RIGHT_DEACTIVATION - 0.06;
    public static volatile double INTAKE_RIGHT_ACTIVATION = INTAKE_RIGHT_DEACTIVATION - 0.22;
    public static volatile long INTAKE_SERVO_INTERVAL_MS = 150;
//    public static volatile long INTAKE_SERVO_DELAY_MS = 500; // global delay for now, PREVIOUSLY WAS 670MS
    public static volatile long INTAKE_SERVO_SHORT_DELAY_MS = 400;
//    public static volatile long INTAKE_SERVO_LONG_DELAY_MS = 670;
    public static volatile long INTAKE_SERVO_LONG_DELAY_MS = 800;

    public static volatile double EXTAKE_MOTOR_KP = 100.0;
    public static volatile double EXTAKE_MOTOR_KI = 0.0;
    public static volatile double EXTAKE_MOTOR_KD = 0.1;
    public static volatile double EXTAKE_MOTOR_KF = 17.8;
    public static volatile double EXTAKE_MOTOR_SPEED_SHORT = 1200; // TODO: adjust
    public static volatile double EXTAKE_MOTOR_SPEED_DUAL_SHORT = 1400; // TODO: adjust
    public static volatile double EXTAKE_MOTOR_SPEED_LONG = 1520; // TODO: adjust
    public static volatile double EXTAKE_MOTOR_SPEED_TOLERANCE = 20;
    public static volatile double EXTAKE_MOTOR_SUPER_FEEDFORWARD_THRESHOLD = 60;
    public static volatile double EXTAKE_DUAL_TRANSITION_MS = 200;
    public static volatile double EXTAKE_RANGE_STABILITY_THRESHOLD = 3.0; // inches
    public static volatile long EXTAKE_RANGE_STABILITY_DURATION_MS = 150;
//    public static volatile int EXTAKE_STABILIZATION_CYCLES = 4;
    public static volatile int EXTAKE_STABILIZATION_CYCLES = 4; // TODO: test zero because human and intake latency should be enough time
    // y (ticks/sec) = -0.0306x^2 + 12.8479 x + 415.9836
    public static volatile double EXTAKE_MODEL_VELOCITY_SIMPLE_A = -0.00135941391;
    public static volatile double EXTAKE_MODEL_VELOCITY_SIMPLE_B = 5.06119602;
    public static volatile double EXTAKE_MODEL_VELOCITY_SIMPLE_C = 874.146043;
    public static volatile double EXTAKE_MODEL_VELOCITY_SIMPLE_RANGE_SHIFT = 0.0; // + forward, - backwards // THIS VALUE NEEDS TO ALSO BE CHANGED IN TELEOP AND AUTO
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

    public static volatile double PINPOINT_HEADING_P = 0.03;
    public static volatile double PINPOINT_HEADING_I = 0;
    public static volatile double PINPOINT_HEADING_D = 0.0001;
    public static volatile double PINPOINT_HEADING_F = 0;

    public static volatile double PINPOINT_OFFSET_FORWARD_Y = -2.784; // 1.92+(1.728/2), measured as of 2/3/2026
    public static volatile double PINPOINT_OFFSET_STRAFE_X = -0.4715; // (1.971/2)-0.514, measured as of 2/8/2026
    public static volatile GoBildaPinpointDriver.EncoderDirection PINPOINT_DIRECTION_FORWARD = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static volatile GoBildaPinpointDriver.EncoderDirection PINPOINT_DIRECTION_STRAFE = GoBildaPinpointDriver.EncoderDirection.REVERSED;

    public static volatile double ROBOT_LENGTH = 17.1; // front to back distance, in inches // TODO: guesstimated
    public static volatile double ROBOT_WIDTH = 16.767; // left to right distance, in inches // TODO: guesstimated
    public static volatile double LOCALIZER_LENGTH_FRONT_OFFSET = ROBOT_LENGTH - 6.9255; // (1.971/2)+5.94, measured as of 2/8/2026 // front-left to localizer distance along robot length axis, in inches // (1.971/2)+5.94
    public static volatile double LOCALIZER_WIDTH_LEFT_OFFSET = 8.3835; // (1.728/2)+5.6715+1.848, measured as of 2/8/2026 // front-left to localizer distance along robot width axis, in inches
    public static volatile double LOCALIZER_LENGTH_BACK_OFFSET = ROBOT_LENGTH - LOCALIZER_LENGTH_FRONT_OFFSET;
    public static volatile double LOCALIZER_WIDTH_RIGHT_OFFSET = ROBOT_WIDTH - LOCALIZER_WIDTH_LEFT_OFFSET;

    public static volatile PinpointConstants ODOMETRY_LOCALIZER_PINPOINT_CONSTANTS = new PinpointConstants()
            .forwardPodY(PINPOINT_OFFSET_FORWARD_Y)
            .strafePodX(PINPOINT_OFFSET_STRAFE_X)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD) // LEO YOU MF IM GONNA UNALIVE YOU
            .forwardEncoderDirection(PINPOINT_DIRECTION_FORWARD)
            .strafeEncoderDirection(PINPOINT_DIRECTION_STRAFE);

    public static volatile MecanumConstants ODOMETRY_DRIVE_MECANUM_CONSTANTS = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("motorFrontLeft")
            .rightFrontMotorName("motorFrontRight")
            .leftRearMotorName("motorBackLeft")
            .rightRearMotorName("motorBackRight")
            .leftFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorEx.Direction.FORWARD)
            .xVelocity(72.367958) // measured 2/6/2026
            .yVelocity(55.709416); // measured 2/6/2026

    public static volatile FollowerConstants ODOMETRY_FOLLOWER_CONSTANTS = new FollowerConstants()
            .mass(13.608) // TODO: NOT MEASURED, ONLY GUESSTIMATED
            .forwardZeroPowerAcceleration(-55.351116) // measured 2/6/2026
            .lateralZeroPowerAcceleration(-80.316819) // measured 2/6/2026
            .translationalPIDFCoefficients(new PIDFCoefficients(0.067, 0, 0.01, 0.03)) // calibrated 2/6/2026
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.067, 0.04)) // calibrated 2/6/2026
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.008, 0, 0.0005, 0.6, 0.025)) // calibrated 2/6/2026
            .centripetalScaling(0.000175); // calibrated 2/6/2026

    public static volatile PathConstraints ODOMETRY_PATH_CONSTANTS = new PathConstraints(0.99, 100, 1, 1);

    public static org.technodot.ftc.twentyfivebeta.pedro.Follower createFollower(HardwareMap hardwareMap) {
//        return new FollowerBuilder(ODOMETRY_FOLLOWER_CONSTANTS, hardwareMap)
//                .pinpointLocalizer(ODOMETRY_LOCALIZER_PINPOINT_CONSTANTS)
//                .mecanumDrivetrain(ODOMETRY_DRIVE_MECANUM_CONSTANTS)
//                .pathConstraints(ODOMETRY_PATH_CONSTANTS)
//                .build();

        PathConstraints.setDefaultConstraints(ODOMETRY_PATH_CONSTANTS);

        // NOTICE: we return our better extended follower!
        return new org.technodot.ftc.twentyfivebeta.pedro.Follower(
                ODOMETRY_FOLLOWER_CONSTANTS,
                new PinpointLocalizer(hardwareMap, ODOMETRY_LOCALIZER_PINPOINT_CONSTANTS),
                new Mecanum(hardwareMap, ODOMETRY_DRIVE_MECANUM_CONSTANTS),
                ODOMETRY_PATH_CONSTANTS
        );
    }
}
