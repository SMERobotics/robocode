#include <jni.h>
#include <string>

// Matches com.cavalry.biobuzz.HelloBridge.getHelloWorldMessage().
extern "C" JNIEXPORT jstring JNICALL
Java_com_cavalry_biobuzz_HelloBridge_getHelloWorldMessage(JNIEnv* env, jobject /* thiz */) {
    std::string message = "Hello World from C++!";
    return env->NewStringUTF(message.c_str());
}
