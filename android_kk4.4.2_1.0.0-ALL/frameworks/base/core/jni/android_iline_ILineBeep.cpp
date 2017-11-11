/* //device/libs/android_runtime/android_debug_JNITest.cpp
**
** Copyright 2006, The Android Open Source Project
**
** Licensed under the Apache License, Version 2.0 (the "License"); 
** you may not use this file except in compliance with the License. 
** You may obtain a copy of the License at 
**
**     http://www.apache.org/licenses/LICENSE-2.0 
**
** Unless required by applicable law or agreed to in writing, software 
** distributed under the License is distributed on an "AS IS" BASIS, 
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
** See the License for the specific language governing permissions and 
** limitations under the License.
*/

#define LOG_TAG "DebugJNI"

#include "jni.h"
#include "nativehelper/JNIHelp.h"
#include "utils/Log.h"
#include "utils/misc.h"
#include "android_runtime/AndroidRuntime.h"
#include <hardware/imx6q_keysound.h>

#include <pthread.h>
namespace android {


struct keyctl_device_t* keyctl_dev = NULL;



static void keyctl_ctl_open(const struct hw_module_t* module ,struct keyctl_device_t** dev){
	module->methods->open(module,KEYCTL_HARDWARE_MODULE_ID,(struct hw_device_t**)dev);
}
static void android_debug_JNITest_startBeep(JNIEnv* env, jobject object){

	keyctl_module_t* keyctl_module = NULL;
	jboolean isStart  = false;
	if(hw_get_module(KEYCTL_HARDWARE_MODULE_ID,(const hw_module_t**)&keyctl_module) == 0){
		keyctl_ctl_open(&(keyctl_module->common),&keyctl_dev);//////
		ALOGE("SuCESS");
		isStart = true;
	}else{
		 ALOGE("JNI test:");
		 isStart = false;
	}
	

	if(keyctl_dev)
	   keyctl_dev->keysound_enable(keyctl_dev);

	

}

static void android_debug_JNITest_stopBeep(JNIEnv* env, jobject object){
	ALOGE("STOPAD");
	if(keyctl_dev){
		keyctl_dev->keysound_disable(keyctl_dev);
	}
	ALOGE("STOPAD11");
}



static JNINativeMethod gMethods[] = {
    /* name, signature, funcPtr */
    { "startNativeBeep",      "()V",
            (void*) android_debug_JNITest_startBeep },
    { "stopNativeBeep",      "()V",
                     (void*) android_debug_JNITest_stopBeep },

};
int register_android_iline_ILineBeep(JNIEnv* env)
{
    return jniRegisterNativeMethods(env, "android/iline/ILineBeep",
        gMethods, NELEM(gMethods));
}

/*
#if 0
 trampoline into C++ 
extern "C"
int register_android_debug_JNITest_C(JNIEnv* env)
{
    return android::register_android_debug_JNITest(env);
}
#endif*/

}; // namespace android


