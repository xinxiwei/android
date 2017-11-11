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
#include <hardware/imx6q_debug.h>

#include <pthread.h>
namespace android {


struct debug_device_t* debugctl_dev = NULL;

static void debugctl_ctl_open(const struct hw_module_t* module ,struct debug_device_t** dev){
	module->methods->open(module,DEBUG_HARDWARE_MODULE_ID,(struct hw_device_t**)dev);
}
static void android_debug_JNITest_startDebugDevice(JNIEnv* env, jobject object){
	debug_module_t* debug_module = NULL;
	jboolean isStart  = false;
	if(hw_get_module(DEBUG_HARDWARE_MODULE_ID,(const hw_module_t**)&debug_module) == 0){
		debugctl_ctl_open(&(debug_module->common),&debugctl_dev);
		ALOGE("start debugdevice success");
		isStart = true;
	}else{
		 ALOGE("start debugdevice fail");
		 isStart = false;
	}
}

static void android_debug_JNITest_stopDebugDevice(JNIEnv* env, jobject object){
	if(debugctl_dev){
		debugctl_dev->debug_close(debugctl_dev);
	}
	ALOGE("stop debugdevice ");
}

static void android_debug_JNITest_setDebugMode(JNIEnv* env, jobject object,jint mode){
	ALOGE("android_debug_JNITest_setDebugMode_mode = %d",mode);
	if(debugctl_dev){
		debugctl_dev->set_dev_val(debugctl_dev,mode); 
	}
}

static JNINativeMethod gMethods[] = {
    /* name, signature, funcPtr */
    { "startNativeDebugDevice",      "()V",          (void*) android_debug_JNITest_startDebugDevice },
    { "stopNativeDebugDevice",      "()V",           (void*) android_debug_JNITest_stopDebugDevice },
    { "setNativeDebugMode",      "(I)V",             (void*) android_debug_JNITest_setDebugMode },
};

int register_android_iline_ILineDebug(JNIEnv* env)
{
    return jniRegisterNativeMethods(env, "android/iline/ILineDebug", gMethods, NELEM(gMethods));
}

}; 


