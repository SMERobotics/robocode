package com.shark.ftc.twentysix;

public class HelloBridge {
    static {
        // Loads libhello-native.so, built by CMake and packaged by Gradle.
        System.loadLibrary("hello-native");
    }

    // The implementation lives in hello-world.cpp.
    public native String getHelloWorldMessage();
}
