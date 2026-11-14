# BIOBUZZ BaboAuto and alliance TeleOps

Select `BaboAuto/RED` or `BaboAuto/BLUE`. Each preselects the corresponding
`BaboOS/RED` or `BaboOS/BLUE` TeleOp, following the alliance-target subclass pattern
in `twentyfivebeta`. The shared auto builds a non-blocking sequence of callbacks,
readiness waits, and delays.

## Placement and coordinates

Coordinates are inches from the audience-left corner. +X goes from RED toward
BLUE, +Y goes from the audience wall toward the rear wall, and heading is
counterclockwise from +X. Coordinates refer to the robot's localization center.

| Alliance | Starting wall | Start `(x, y, heading)` | After moving 3 inches robot-right |
| --- | --- | --- | --- |
| RED | Audience | `(32, 8, 90 degrees)` | `(35, 8, 90 degrees)` |
| BLUE | Rear | `(112, 136, 270 degrees)` | `(109, 136, 270 degrees)` |

This interprets the requested 24+8 inches as 32 inches from the alliance's driver
wall, with the robot's rear 8 inches behind its localization center. Physically
place the robot against its starting wall with its front pointing into the field.
The field offsets and rightward travel are in `Configuration.AUTO_*`.

The [BIOBUZZ TU09 manual, G304 and Figure 9-2](https://ftc-resources.firstinspires.org/ftc/archive/2027/game/cm-html/BIOBUZZ%20Competition%20Manual%20-%20TU09.htm)
requires perimeter contact, the robot fully on its alliance half, and no overlap
with the loading zone or a flower. These default poses assume a 16-inch chassis
centered on the localization origin. Confirm the actual chassis footprint and
origin when placing the robot; software sets the pose but does not physically
place the robot.

## Sequence and tuning

At INIT, configure the devices and set Pinpoint's field pose. At START, set the
same start pose again, select the pollen servo position, spin the shooter at
`AUTO_SHOOT_VELOCITY` (default 1650 ticks/second), and follow the short constant
heading path to the shooting pose. Pedro owns the drive motors and Pinpoint
during auto; manual gamepad input is ignored.

Hold the shooting pose and wait for turret alignment and shooter velocity within
tolerance for `AUTO_READY_SETTLE_MS`. Vision continues steering the turret, but
missing frames or the hive's classified state do not prevent feeding. Set
`AUTO_REQUIRE_HIVE_VISION` to true to additionally require a fresh valid alliance
hive observation. This defaults to false so camera availability is not a firing
prerequisite.
The drive-arrival check runs once before the first shot; odometry fluctuations
while the follower holds position do not restart the shot-readiness timer.
For each of four pollen, open the transfer, use the existing 100 ms servo
clearance delay, wait until the intake actually starts feeding at 0.6 power,
feed for `AUTO_FEED_MS` (default 150 ms), close the transfer and stop intake,
then wait `AUTO_BETWEEN_SHOTS_MS` (default 400 ms). Recheck aim and speed before
the next pulse. There is no pollen sensor: the telemetry counts feed pulses,
and pulse timing must be tuned on the robot to release one pollen each time.

The position/heading, speed, and turret tolerances and wait timeouts are exposed
in Panels under `Configuration`. Placement, path and delay/wait durations are
read at INIT; shooter velocity is read at START, and readiness tolerances are
read every loop. A readiness timeout ends the sequence and
reports the failed wait. The `shot blocked by`, speed/target, turret error,
position error, drive speed, and hive telemetry retain the last readiness check
after a timeout, before the motors were stopped. Completion, STOP, or the 29-second limit stops the
drivetrain, intake, shooter, turret, and camera, and closes the transfer.

## TeleOp

After auto, start its preselected alliance TeleOp. Pinpoint's field pose is
preserved; TeleOp does not zero heading or recalibrate the IMU. From either
alliance's driver area, left-stick up always moves away from the players,
left-stick right moves toward their right, and right-stick rotation stays
robot-relative. The RED and BLUE transforms differ by 180 degrees. If running
TeleOp without auto, first establish the same field heading convention in
Pinpoint; an arbitrary zero heading will rotate field-centric controls.

The existing `BaboOS` entry remains available as the BLUE default. Use the named
alliance entries for matches.
