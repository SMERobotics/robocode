# Hive vision setup and verification

`DeviceLimelight` runs **SimpleHiveSolver**. It uses calibrated image rays, the
known 3.25-inch tag width, and the horizontal hive axle to calculate each tag's
position and orientation directly. It averages the tag estimates for each alliance
and applies the CAD focus offset. Either cell's tags can contribute.

The runtime solver uses ordinary arithmetic, square roots, and trigonometry.
It accepts each complete detection immediately. The previous iterative
`HiveSolver` and OpenCV seed provider remain available for offline comparisons.

## Configure the Limelight

Use one runtime pipeline. No cell/alliance pipeline switching is needed.

| Setting | Value |
| --- | --- |
| Pipeline | **0**, name `Hive`, type AprilTag/Fiducial |
| Family | **36h11** |
| Marker size | **82.55 mm** (3.25 in, black square; exclude white margin) |
| Image | **640 x 480**, normal orientation, no hardware zoom |
| Downscale | **1x** to start |
| IDs | `30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45` |
| Crop/grouping | Full frame, no grouping or priority ID needed |
| Output | **Send raw corners/contours**, JSON enabled; four `pts` per fiducial |
| 3D | Optional; only decoded corners are used |
| Exposure | Manual; start at **1 ms** (10 if the UI uses 0.1 ms units) |
| Gain / black level | Start at **15 / 0**, tune for crisp moving edges |

Verify `pts` actually contains four points (or eight scalar coordinates) in the
JSON viewer. `tx/ty` alone cannot drive this solver. Polling at 100 Hz does not
increase the camera's capture rate. Keep exposure short enough to limit motion
blur and rolling-shutter errors during turret motion.

In `HiveVisionConfig` (also available in FTC Dashboard), set:

1. `solver.cameraPitchDegrees`: measured optical-axis tilt above horizontal.
2. `solver.cameraRollDegrees`: measured camera roll, positive clockwise when
   looking forward from behind the camera (right axis rotates downward).
3. `CALIBRATION_SOURCE`: match the calibration actually selected on the camera:
   `DEFAULT`, `FILE`, or `EEPROM`. Missing selected data fails initialization;
   the code does not silently fall back to another calibration.
4. `LIMELIGHT_HOST`: the Limelight's USB address from the robot configuration,
   default `172.29.0.1`. This is the camera's address, not the Control Hub's address.

Calibration is read from the selected REST endpoint. Both the flat JSON schema
and the firmware's `CALIBRATION_RESULT` envelope are supported; SDK 12's built-in
calibration getter does not unwrap that envelope. HTTP failures, empty responses,
and malformed calibration fields are reported with the selected source/endpoint.

For best accuracy, perform a ChArUco calibration, save it, then select the matching
source here. `cal-latest` is not necessarily active and is not used. Intrinsics
are scaled to the configured image size; cropped/zoomed modes are unsupported.
The target size is 640 x 480; a 1280 x 960 source calibration is scaled by 0.5
for focal lengths and principal point. The simple solver uses the pinhole
intrinsics directly and ignores lens distortion. Reinitialize after changing
pipeline, image size, calibration source, or corner order. Pitch, roll, offsets, and solver thresholds can be
tuned in Dashboard. The camera must keep fixed pitch/roll as its turret rotates
in yaw, and the robot must be approximately level for the gravity constraint.

Default corners follow decoded AprilTag **BL, BR, TR, TL**, relative to the
*printed* tag, even when it appears upside down. Never sort corners by screen
position. `BOTTOM_LEFT_CORNER` and `REVERSE_CORNERS` adapt firmware ordering.
Verify these conventions with recorded upright and inverted tag samples on the
actual camera before relying on the output. These hardware samples are not
available in this repository.

## Public result and states

```java
HiveObservation hive = deviceLimelight.getHiveObservation(Alliance.BLUE);
if (hive.valid) {
    // hive.state: READY, TIPPING, or TIPPED
    // hive.rangeInches, hive.bearingDegrees, hive.elevationDegrees
}
```

The camera-side cell is selected from the estimated axle position and horizontal
cell direction. READY means its estimated tilt is at least **+15 degrees**,
TIPPED means at most **-15 degrees**, and TIPPING covers the middle. States change
immediately. `solver.stopThresholdDegrees` adjusts this single threshold.
Far-cell tags also contribute to the camera-side focus estimate.

The optical camera coordinates are **right, down, forward**, in inches. Bearing
is positive left; elevation is positive up; both are in degrees. Range is the
**3D straight-line distance**, not floor distance. Values describe the image's
estimated capture time, not an extrapolated current turret target. The timestamp
uses local monotonic time and the reported capture/processing/parse latency;
unmeasured network transit is not a synchronized exposure timestamp.

The focus defaults to the SDK cluster origin. `aimRightInches`, `aimDownInches`,
and `aimIntoCellInches` add an offset in the selected cell's **printed tag frame**,
so the same offset follows the cell as it tips. Positive into-cell points through
the tag surface toward the inside of the cell.

A detected tag with four distinct corners is trusted, including a single tag,
partial off-screen corners, and noisy geometry. There are no reprojection,
ambiguity, outlier, travel-limit, settling, or motion-rate rejection thresholds.
Missing detections and absent calibration produce an invalid result. Frozen
measurements expire after `solver.maxAgeMillis` (default **3000 ms**). Frame
counters still prevent repeated images from extending the turret's tag timeout.

## Simple calculation

1. Convert the corners into rays using fx/fy/cx/cy, then remove camera pitch/roll.
2. Each horizontal tag edge has equal-height endpoints and a known width. Solve
   the two ray distances directly; on the camera horizon, use apparent tag width.
3. Calculate the tag center and right/down directions from the four endpoints.
4. Subtract the known tag position to get the hive origin, and average all tags.
5. Use `asin` of the vertical component for tilt, apply the CAD focus offset,
   then calculate range and bearing/elevation with square roots and `atan2`.

This intentionally optimistic estimate can be inaccurate with noisy corners,
incorrect mounting angles, or lens distortion. Those estimates are still used.
The active config contains camera pitch/roll, focus offsets, one state threshold,
and the age limit. The `hive solver` telemetry reads `simple trig`; `hive solve
(ms)` measures its runtime. Reprojection and ambiguity scores are unused (NaN).

## Turret tracking in BaboOS

`BaboOS/RED` and `BaboOS/BLUE` select the robot's alliance through `config()`;
the generic `BaboOS` entry defaults to BLUE. A fresh, valid
observation of that alliance updates the turret target toward
the cell focus regardless of hive state. Missing observations hold the previous target. D-pad turret
adjustments and the Start-button encoder reset are removed, including manual
turret movement in TestRobot. Launcher power and mechanism controls remain manual.

Three seconds without a newly captured hive tag from the selected alliance sends
the turret to **encoder heading 0**, facing forward. This turns the turret to its
existing zero; it does not reset its encoder at the current position. Visible
alliance tags refresh this timer even if their pose is TIPPED or unusable. Opponent
tags, repeated results, and frozen frames do not extend the timer. At startup,
without any sightings, the turret holds its current position for three seconds.

Turret encoder angles are positive right. The controller removes camera pitch
and roll from the optical focus before computing horizontal yaw, then adds that
correction to the interpolated encoder angle at image capture time. Each image
updates the target once. The existing nearest-turn selection, cable travel
bounds, RUN_TO_POSITION mode, and position power still apply. Camera-relative
aiming updates as new images arrive; it does not extrapolate robot motion while
tags are absent.

`HiveVisionConfig.turretAim` provides camera yaw alignment (degrees, positive
right), and camera optical-center offsets right/forward from the turret's yaw
axis (inches). These default to zero. `turret aim` telemetry reports tracking,
holding, or returning forward. Test Hive Vision remains sensor-only.

## Geometry provenance


- [Official field CAD and production artwork](https://ftc-resources.firstinspires.org/ftc/field):
  STEP ZIP **v26-27.2**, production artwork **V1**.
- [SDK 12 release information](https://github.com/FIRST-Tech-Challenge/FtcRobotController#release-information),
  `Vision-12.0.0-sources.jar`, `AprilTagGameDatabase.getBioBuzzTagLibrary()` and
  `getBioBuzzCluster()`: tag size 3.25 in; printed X centers -6.50, -2.75,
  +2.75, +6.50 in; centers Y=7.1874, Z=-5.622 in relative to the focus.
- [Limelight corner output](https://docs.limelightvision.io/docs/docs-limelight/apis/json-results-specification)
  and [calibration API](https://docs.limelightvision.io/docs/docs-limelight/apis/rest-http-api).

STEP ZIP SHA-256:
`5e768b731f1ec8dcd14debba53225c43718877923c351ce08504305f68f7fe00`.
Artwork SHA-256:
`be9451596b2a08eda8d22041ae4451feb3946b100fe4ed4d64c5ff24e694cf32`.

`tools/derive_hive_geometry.py` reproduces the baked geometry from that STEP ZIP.
The axle centers in the CAD's meter coordinates are blue
`(0.324038318982907, 1.11632237999612, 0)` and red
`(-0.323661681017093, 1.11632237999612, 0)`. Undo the CAD's blue +30 / red -30
rotation about X, subtract the axle center, then map `(x,y,z)` to `(x,-z,y)`.
The resulting neutral frame is right-handed: X along the axle toward blue,
Y toward the scoring table, Z up.

The PDF's sticker is 17 x 5 in. Its printed black-square centers are 2, 5.75,
11.25, and 15 in from printed left and 2.25 in from printed top. The CAD notches
identify printed bottom at local Y=5 in; printed right is -local X. Transforming
the printed surface (local Z=0.01 in) gives row centers at neutral
`(0.000944882, +/-14.269059480, -1.498167842)` inches relative to the axle.
Shift the model origin down to this common plane so IPPE's corners have Z=0.
The small X offset is preserved from the CAD.

Both rows are on one rigid **coplanar** assembly. Audience IDs increase along
model +X; scoring IDs increase along -X, with printed down also reversed. Thus
both rows can participate in one solve without treating them as independent
objects or assuming a fixed pose on the field. The focus is the SDK origin even
if a preferred shooting location differs; use the configurable offset to tune it.

## Verification

- `gradlew.bat :TeamCode2028:compileDebugJavaWithJavac --offline`
- `gradlew.bat :TeamCode2028:testDebugUnitTest --offline` for JVM math/state tests.
- `python TeamCode2028/tools/test_hive_solver.py` for desktop native OpenCV,
  the simple solver, turret aiming, legacy fitter comparisons, and JSON parsing.
  It downloads pinned test-only Maven jars under `build/`; they do not enter the
  APK. The legacy comparison tests load desktop OpenCV 4.9; the active solver
  uses no native code.

Use **Test Hive Vision** on the robot for sensor-only validation. It initializes
no drive, turret, intake, or launcher motors. BaboOS also shows the same telemetry
during INIT and operation. The test OpMode runs a known synthetic scene through
the active simple solver and reports PASS/FAIL and execution time under `simple
solver check`. This path has no OpenCV/JNI initialization. The synthetic check
uses its own level camera configuration, independent of the real camera's
configured pitch/roll.
Press A to write one raw JSON frame to the Robot Controller log (`HiveVisionFrame`);
join the numbered parts to recover the JSON without Android log truncation.
Check all four cells, both stops, partial occlusion,
real flips, and turret sweeps at your intended ranges. Compare focus location and
RBE to independent physical measurements, record corner JSON samples upright and
inverted, and inspect solve time/error/rejection rates. Launcher controls remain
manual. BaboOS uses the turret tracking behavior described above;
real-camera accuracy remains to be measured on the field.

On Windows, if a quoted PATH entry prevents Gradle's test JVM from launching,
append `-Djava.library.path=.` to the unit-test command. The desktop native suite
uses its own test runtime and does not require changing the system PATH.
