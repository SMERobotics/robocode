cmake_minimum_required(VERSION 3.22.1)
project("hello-native" LANGUAGES CXX)

# Build libhello-native.so for Android to load through HelloBridge.
add_library(hello-native SHARED hello-world.cpp)
target_compile_features(hello-native PRIVATE cxx_std_11)
