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
#include <hardware/imx6q_ctr.h>

#include <pthread.h>
namespace android {


struct gpioctl_device_t* gpioctl_dev = NULL;



static void gpioctl_ctl_open(const struct hw_module_t* module ,struct gpioctl_device_t** dev){
	module->methods->open(module,GPIOCTL_HARDWARE_MODULE_ID,(struct hw_device_t**)dev);
}
static void android_debug_JNITest_startDevice(JNIEnv* env, jobject object){

	gpioctl_module_t* gpio_module = NULL;
	jboolean isStart  = false;
	if(hw_get_module(GPIOCTL_HARDWARE_MODULE_ID,(const hw_module_t**)&gpio_module) == 0){
		gpioctl_ctl_open(&(gpio_module->common),&gpioctl_dev);//////
		ALOGE("SuCESSLight");
		isStart = true;
	}else{
		 ALOGE("JNI testLight:");
		 isStart = false;
	}
}

static void android_debug_JNITest_stopDevice(JNIEnv* env, jobject object){
	ALOGE("STOPADLight");
	if(gpioctl_dev){
		gpioctl_dev->gpioctl_close(gpioctl_dev);
	}
	ALOGE("STOPAD11Light");
}


static void android_debug_JNITest_startLightOn(JNIEnv* env, jobject object,jint light){
	//0 hong 1,huang,2 lv  »Æ 3 lv 4 bhong 5
	if(gpioctl_dev){
		gpioctl_dev->set_val(gpioctl_dev,light,1);
	}
}


static void android_debug_JNITest_startLightOff(JNIEnv* env, jobject object,jint light){

	if(gpioctl_dev){
		 gpioctl_dev->set_val(gpioctl_dev,light,0);
	}
}
static JNINativeMethod gMethods[] = {
    /* name, signature, funcPtr */
    { "startNativeLightDevice",      "()V",
            (void*) android_debug_JNITest_startDevice },
    { "stopNativeLightDevice",      "()V",
                     (void*) android_debug_JNITest_stopDevice },
    { "setNativeLightOpen",      "(I)V",
                     (void*) android_debug_JNITest_startLightOn },
    { "setNativeLightClose",      "(I)V",
                     (void*) android_debug_JNITest_startLightOff },

};
int register_android_iline_ILineLight(JNIEnv* env)
{
    return jniRegisterNativeMethods(env, "android/iline/ILineLight",
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


