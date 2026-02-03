diff a/README.md b/README.md	(rejected hunks)
@@ -59,6 +59,21 @@ The readme.md file located in the [/TeamCode/src/main/java/org/firstinspires/ftc
 
 # Release Information
 
+## Version 11.1 (20251231-104637)
+
+### Enhancements
+
+* Gamepad triggers can now be accessed as booleans and have edge detection supported.
+* GoBildaPinpointDriver now supports Pinpoint v2 functionality
+* Adds webcam calibrations for goBILDA's USB camera.
+
+### Bug Fixes
+* Fixes issue [1654](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1654) in GoBildaPinpointDriver that caused error if resolution was set in other than MM
+* Fixes issue [1628](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1628) Blocks editor displays incorrect Java code for gamepad edge detection blocks.
+* Fixes possible race condition issue [1884](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1884) on Driver Station startup when Driver Station name doesn't match the Robot Controller name.
+* Fixes issue [1863](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1863) - Incorrect package paths in samples.
+* Fixes an issue where an OnBotJava filename that begins with a lowercase character would fail to properly rename the file if the user tried to rename it so that it begins with an uppercase character.
+
 ## Version 11.0 (20250827-105138)
 
 ### Enhancements
