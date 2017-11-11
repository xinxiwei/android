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
#include <hardware/imx6q_spi.h>
#include <hardware/hardware.h>
#include <pthread.h>
namespace android {



struct spictl_device_t* spictl_dev = NULL;
static const SpiPressureInterface* sSpiInterface = NULL;
pthread_t th_RcvData;

 jmethodID method_request_java_cb;
 jobject mCallbacksObj;
 static JavaVM *gJavaVm=NULL;
 static void checkAndClearExceptionFromCallback(JNIEnv* env, const char* methodName) {
     if (env->ExceptionCheck()) {
        // ALOGE("An exception was thrown by callback '%s'.", methodName);
        
         env->ExceptionClear();
     }
 }
static void request_java_callback(float data[],jboolean isCollectData){
    for(int i= 0;i<2;i++){
	 ALOGE("jnitest: Gather data[%d] = %f",i,data[i]);
	}

    JNIEnv *lhxEnv;
   	gJavaVm->AttachCurrentThread(&lhxEnv, 0);
    JNIEnv* env = AndroidRuntime::getJNIEnv();
    int length = data[1];
    jfloatArray iarr = env->NewFloatArray(length);//???h??jintArray
    env->SetFloatArrayRegion(iarr, 0, length, data);
    env->CallVoidMethod(mCallbacksObj, method_request_java_cb, iarr,isCollectData);
    checkAndClearExceptionFromCallback(env,__FUNCTION__);	
	
    if(iarr){
    	env->DeleteLocalRef(iarr);
    }
    gJavaVm->DetachCurrentThread();
   
};
static void request_callback(float data[],bool isCollectData){

	 request_java_callback(data,isCollectData);
}

static pthread_t create_thread_callback(const char* name, void (*start)(void *), void* arg)
{
    return (pthread_t)AndroidRuntime::createJavaThread(name, start, arg);
}
SpiPressureCallbacks mSpiCb = {
		request_callback,
		create_thread_callback
	};
static void spictl_ctl_open(const struct hw_module_t* module ,struct spictl_device_t** dev){
	module->methods->open(module,SPICTL_HARDWARE_MODULE_ID,(struct hw_device_t**)dev);
}

static void android_debug_JNITest_startSingleAD(JNIEnv* env, jobject object,float data){  //??????

	mCallbacksObj = env->NewGlobalRef(object);
    jclass clazz = env->GetObjectClass(object);
	method_request_java_cb = env->GetMethodID(clazz,"requestData", "([FZ)V");
    //ALOGE("JNI 11test2222: = %f",data);
	int result = -5;
	//data = -0.000587;
	if(spictl_dev)
		result = spictl_dev->start_press_dial(spictl_dev,data);

}

static void android_debug_JNITest_startGroupAD(JNIEnv* env, jobject object,float data){ //??????

	mCallbacksObj = env->NewGlobalRef(object);
    jclass clazz = env->GetObjectClass(object);
	method_request_java_cb = env->GetMethodID(clazz,"requestData", "([FZ)V");
	//ALOGE("JNI 11test2222: = %f",data);
	int result = -5;
	//data = -20.124567;
	if(spictl_dev)
		result = spictl_dev->start_press_curve(spictl_dev,data);

}

static void android_debug_JNITest_startCaliAD(JNIEnv* env, jobject object){ //??0??

	mCallbacksObj = env->NewGlobalRef(object);
    jclass clazz = env->GetObjectClass(object);
	method_request_java_cb = env->GetMethodID(clazz,"requestData", "([FZ)V");
	int result = -5;
	if(spictl_dev)
		result = spictl_dev->start_press_flag0(spictl_dev);

}

static jboolean android_debug_JNITest_stopAD(JNIEnv* env, jobject object){
	ALOGE("STOPAD");
	if(spictl_dev){

	  jboolean 	 isStop  = (spictl_dev->stop_press_ad(spictl_dev) == 0);
      gJavaVm = NULL;
	  return   isStop;
	}
	return false;
}
static jint android_debug_JNITest_getNativeADFrequency(JNIEnv* env, jobject object){
	
	if(spictl_dev)
		 return  spictl_dev->spi_freq(spictl_dev);
    return -1;
}


static jboolean android_debug_JNITest_startNativeInit(JNIEnv* env, jobject object){
	
	spictl_module_t*  spi_module = NULL;
	jboolean isStart  = true;
	if(hw_get_module(SPICTL_HARDWARE_MODULE_ID,(const hw_module_t**)&spi_module) == 0){
		spictl_ctl_open(&(spi_module->common),&spictl_dev);//////
		isStart = true;
	}else{
		 isStart = false;
	}
	if(isStart){
	   sSpiInterface  = spictl_dev->get_pressure_interface(spictl_dev);
	   sSpiInterface->initPress(&mSpiCb);
	}

	 int tempret = env->GetJavaVM(&gJavaVm); //???›Ô????????JVM
	return isStart;
}



/*
 * JNI registration.
 */
static JNINativeMethod gMethods[] = {
    /* name, signature, funcPtr */
    { "startNativeSingleAD",      "(F)V",
            (void*) android_debug_JNITest_startSingleAD },
    { "startNativeGroupAD",      "(F)V",
                     (void*) android_debug_JNITest_startGroupAD },
	{ "startNativeCaliAD",      "()V",
                     (void*) android_debug_JNITest_startCaliAD },
					 
    {"stopNativeAD",      "()Z",
            (void*) android_debug_JNITest_stopAD },
    {"startNativeInit",      "()Z",
            (void*) android_debug_JNITest_startNativeInit },
    {"getNativeADFrequency",      "()I",
            (void*) android_debug_JNITest_getNativeADFrequency },
};
int register_android_iline_ILineGatherModule(JNIEnv* env)
{
    return jniRegisterNativeMethods(env, "android/iline/ILineGatherModule",
        gMethods, NELEM(gMethods));
}

}; // namespace android


