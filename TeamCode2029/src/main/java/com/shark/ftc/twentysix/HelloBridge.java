package com.shark.ftc.twentysix;

import dev.frozenmilk.sinister.loading.Pinned;

// JNI libraries remain attached to the class loader that first loads them.
@Pinned
public class HelloBridge {
    static {
        // Loads libhello-native.so, built by CMake and packaged by Gradle.
        System.loadLibrary("hello-native");
    }

    // The implementation lives in hello-world.cpp.
    public native String getHelloWorldMessage();
}
