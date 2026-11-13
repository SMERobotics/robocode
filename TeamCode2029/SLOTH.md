# TeamCode2029 hot reload

This module uses standalone Sloth 0.3.2 and the compatible Dashboard fork,
without Dairy Core. Run Gradle with JDK 17.

Perform a full install, then start the Robot Controller and let initialization finish:

```sh
ANDROID_SERIAL=<hub-serial> ./gradlew :TeamCode2029:installDebug
```

For subsequent Java changes:

```sh
ANDROID_SERIAL=<hub-serial> ./gradlew :TeamCode2029:deploySloth
```

The official Load plugin supplies assembly, upload, and remote cleanup tasks.
Full debug/release installs automatically clear previous Sloth uploads.
`autoconnect = NEVER` uses the existing ADB connection without starting a Wi-Fi
connection. Set `ANDROID_SERIAL` to select the intended hub.

`sloth.gradle` adapts both Sloth package filters to reload
`com.shark.ftc.twentysix` and its subpackages. It also repairs Sloth 0.3.2's
OnBotJava JAR-path conversion. Both adapters fail the build if their expected
bytecode changes; review them when upgrading Sloth.

`HelloBridge` is pinned so JNI continues using its original class loader.
Changes to that bridge, C++ code, dependencies, resources, or pinned annotations
require another full install. Other Java OpModes can be hot reloaded.

If a Dairy Maven mirror is required, `-PdairyMavenUrl=...` applies to both the
Load plugin and runtime dependencies. Offline builds require a warm cache for
that same repository URL.
