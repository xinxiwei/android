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
#include <hardware/imx6q_backlight.h>

#include <pthread.h>
namespace android {


struct gpioctl_device_t* backlightctl_dev = NULL;

static void gpioctl_ctl_open(const struct hw_module_t* module ,struct gpioctl_device_t** dev){
	module->methods->open(module,BACKLIGHTCTL_HARDWARE_MODULE_ID,(struct hw_device_t**)dev);
}
static void android_debug_JNITest_startDevice(JNIEnv* env, jobject object){

	gpioctl_module_t* gpio_module = NULL;
	jboolean isStart  = false;
	if(hw_get_module(BACKLIGHTCTL_HARDWARE_MODULE_ID,(const hw_module_t**)&gpio_module) == 0){
		gpioctl_ctl_open(&(gpio_module->common),&backlightctl_dev);
		//ALOGE("JNI层打开单个背光灯控制的设备文件devbacklightctr 成功");
		isStart = true;
	}else{
		 ALOGE("start backlightdevice fail");
		 isStart = false;
	}
}

static void android_debug_JNITest_stopDevice(JNIEnv* env, jobject object){
	if(backlightctl_dev){
		backlightctl_dev->gpioctl_close(backlightctl_dev);
	}
	//ALOGE("JNI层关闭单个背光灯控制的设备文件devbacklightctr");
}

static void android_debug_JNITest_setLight(JNIEnv* env, jobject object,jint light,jint status){
	//light 是键值，背光  9
	//ALOGE("android_debug_JNITest_setbackLight light = %d, status = %d",light,status);
	if(backlightctl_dev){
		backlightctl_dev->set_backlight_val(backlightctl_dev, light, status); 
	}
}

static JNINativeMethod gMethods[] = {
    /* name, signature, funcPtr */
    { "startNativeBackLightDevice",      "()V",          (void*) android_debug_JNITest_startDevice },
    { "stopNativeBackLightDevice",      "()V",           (void*) android_debug_JNITest_stopDevice },
    { "setNativeBackLight",      "(II)V",             (void*) android_debug_JNITest_setLight },
};

int register_android_iline_ILineBackLight(JNIEnv* env)
{
    return jniRegisterNativeMethods(env, "android/iline/ILineBackLight", gMethods, NELEM(gMethods));
}

}; 


