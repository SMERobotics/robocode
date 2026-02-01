package org.technodot.ftc.twentyfivebeta;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
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
    public static volatile double DRIVE_STRAFE_MULTIPLIER = 1.1; // 1.10 +/- 0.01 idfk? Y TF NOT
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

    public static final PinpointConstants ODOMETRY_LOCALIZER_PINPOINT_CONSTANTS = new PinpointConstants()
            .forwardPodY(2.735)
            .strafePodX(-0.500) // basically 0.500 inches, measured as of 1/29/26
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD) // LEO YOU MF IM GONNA UNALIVE YOU
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static final MecanumConstants ODOMETRY_DRIVE_MECANUM_CONSTANTS = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("motorFrontLeft")
            .rightFrontMotorName("motorFrontRight")
            .leftRearMotorName("motorBackLeft")
            .rightRearMotorName("motorBackRight")
            .leftFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorEx.Direction.FORWARD)
            .xVelocity(63.90)
            .yVelocity(50.50);

    public static final FollowerConstants ODOMETRY_FOLLOWER_CONSTANTS = new FollowerConstants()
            .mass(13.608) // TODO: NOT MEASURED, ONLY GUESSTIMATED
            .forwardZeroPowerAcceleration(-51.653)
            .lateralZeroPowerAcceleration(-89.833)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.065, 0, 0.003, 0.025)) // TODO: NOT MEASURED
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.01, 0.025)) // TODO: NOT MEASURED
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0, 0.0001, 0.6, 0.025)) // TODO: NOT MEASURED
            .centripetalScaling(0.0003); // TODO: NOT MEASURED

    public static final PathConstraints ODOMETRY_PATH_CONSTANTS = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(ODOMETRY_FOLLOWER_CONSTANTS, hardwareMap)
                .pinpointLocalizer(ODOMETRY_LOCALIZER_PINPOINT_CONSTANTS)
                .mecanumDrivetrain(ODOMETRY_DRIVE_MECANUM_CONSTANTS)
                .pathConstraints(ODOMETRY_PATH_CONSTANTS)
                .build();
    }
}
