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
#include <hardware/imx6q_beep.h>

#include <pthread.h>
namespace android {


struct keyctl_device_t* keyctl_dev = NULL;
struct keyctl_device_t* keyctl_dev2 = NULL;


static void keyctl_ctl_open(const struct hw_module_t* module ,struct keyctl_device_t** dev){
	module->methods->open(module,KEYCTL_HARDWARE_MODULE_ID,(struct hw_device_t**)dev);
}

static void android_debug_JNITest_openKey(JNIEnv* env, jobject object){
	keyctl_module_t* keyctl_module = NULL;
	jboolean isStart  = false;
	if(hw_get_module(KEYCTL_HARDWARE_MODULE_ID,(const hw_module_t**)&keyctl_module) == 0){
		keyctl_ctl_open(&(keyctl_module->common),&keyctl_dev);
		ALOGE("startkey SuCESS");
		isStart = true;
	}else{
		 ALOGE("startkey faile");
		 isStart = false;
	}	
}

static void android_debug_JNITest_startKey(JNIEnv* env, jobject object,jint key){
	if(keyctl_dev)
	   keyctl_dev->key_enable(keyctl_dev,key);
}


static void android_debug_JNITest_stopKey(JNIEnv* env, jobject object,jint key){
	if(keyctl_dev){
		keyctl_dev->key_disable(keyctl_dev,key);
	}    
}


static JNINativeMethod gMethods[] = {
    /* name, signature, funcPtr */
    { "startOpenKey",      "()V",            (void*) android_debug_JNITest_openKey },
    { "startNativeKey",      "(I)V",            (void*) android_debug_JNITest_startKey },
    { "stopNativeKey",      "(I)V",             (void*) android_debug_JNITest_stopKey },   
};

int register_android_iline_ILineBeep(JNIEnv* env)
{
    return jniRegisterNativeMethods(env, "android/iline/ILineBeep",    gMethods, NELEM(gMethods));
}

}; 

