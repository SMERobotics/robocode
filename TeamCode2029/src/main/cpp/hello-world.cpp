#include <jni.h>
#include <string>

// Matches com.shark.ftc.twentysix.HelloBridge.getHelloWorldMessage().
extern "C" JNIEXPORT jstring JNICALL
Java_com_shark_ftc_twentysix_HelloBridge_getHelloWorldMessage(JNIEnv* env, jobject /* thiz */) {
    std::string message = "Hello World from C++!";
    return env->NewStringUTF(message.c_str());
}
