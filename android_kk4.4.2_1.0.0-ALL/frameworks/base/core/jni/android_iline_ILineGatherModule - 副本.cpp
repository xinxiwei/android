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

jmethodID request_press_data_ID = NULL;
jmethodID request_press_stop_ID = NULL;
jobject mPressCallbacksObj = NULL; 
static JavaVM *gJavaVm=NULL;
jint jni_version = 0;  
static void checkAndClearExceptionFromCallback(JNIEnv* env, const char* methodName) {
     if (env->ExceptionCheck()) {  
	     ALOGE("An exception was thrown by callback '%s'.", methodName);    
         env->ExceptionClear();
     }
 }
 
static void request_java_callback(float data[],jboolean isCollectData){
	ALOGE("jni获取压力回调数据: Gather_data[0] = %f",data[0]);

	bool isAttacked  = false;
	JNIEnv* env = NULL;   //JNI_VERSION_1_6
	int status = gJavaVm->GetEnv((void **)&env, jni_version); //从java 虚拟机获得javaEnv，第一个参数指向JavaEnv， 第二个参数为JNi 版本
	if(status <0) //说明没有得到JavaEnv
	{	
		//ALOGE("jnitest: status = %d",status);
		status = gJavaVm->AttachCurrentThread(&env, NULL); //从当前线程获得javaEnv,链接当前线程到java虚拟机,第二个参数指向线程名， 可以为空
		if(status <0)
		{
			return ;
		}
		isAttacked  = true; 
	}
	
    int length = data[1]; //获取有效数据个数
    jfloatArray iarr = env->NewFloatArray(length);
	
    env->SetFloatArrayRegion(iarr, 0, length, data);
    env->CallVoidMethod(mPressCallbacksObj, request_press_data_ID, iarr,isCollectData);
    checkAndClearExceptionFromCallback(env,__FUNCTION__);	
	
    if(iarr){
    	env->DeleteLocalRef(iarr);
    }
	
	if(isAttacked)
	{
		//ALOGE("jnitest: isAttacked = %d",isAttacked);
		gJavaVm->DetachCurrentThread();  //解除当前线程与java虚拟机之间的链接 
	}
};

static void request_press_stop_callback(jboolean isStop){
	ALOGE("jni收到压力停止指令: request_press_stop_callback enter");
	if (!gJavaVm) {
		return;
	}

	#if 0
		JNIEnv* env = AndroidRuntime::getJNIEnv();
		env->CallVoidMethod(mPressCallbacksObj, request_press_stop_ID, isStop);
		
		gJavaVm = NULL;
		env->DeleteGlobalRef(mPressCallbacksObj);
		request_press_stop_ID = NULL;
		request_press_data_ID = NULL;
		mPressCallbacksObj = NULL;
	#else	
		JNIEnv* env = NULL;
		gJavaVm->GetEnv((void **)&env, jni_version); //从java 虚拟机获得javaEnv，第一个参数指向JavaEnv， 第二个参数为JNi 版本		
		env->CallVoidMethod(mPressCallbacksObj, request_press_stop_ID, isStop);
		
		gJavaVm = NULL;
		env->DeleteGlobalRef(mPressCallbacksObj);
		request_press_stop_ID = NULL;
		request_press_data_ID = NULL;
		mPressCallbacksObj = NULL;	
	#endif
	ALOGE("jnitest: request_press_stop_callback exit");
};

static void request_callback(float data[],bool isCollectData){
	request_java_callback(data,isCollectData);
}

static pthread_t create_thread_callback(const char* name, void (*start)(void *), void* arg)
{
    return (pthread_t)AndroidRuntime::createJavaThread(name, start, arg);
}

static void request_stop_callback(bool isStop){
	request_press_stop_callback(isStop);
}

SpiPressureCallbacks mSpiCb = {
	request_callback,
	create_thread_callback,
	request_stop_callback
};

static void spictl_ctl_open(const struct hw_module_t* module ,struct spictl_device_t** dev){
	module->methods->open(module,SPICTL_HARDWARE_MODULE_ID,(struct hw_device_t**)dev);
}

static void android_debug_JNITest_startSingleAD(JNIEnv* env, jobject object,float data){  
	mPressCallbacksObj = env->NewGlobalRef(object);
    jclass clazz = env->GetObjectClass(object);
	request_press_data_ID = env->GetMethodID(clazz,"requestData", "([FZ)V");
	request_press_stop_ID = env->GetMethodID(clazz,"requestStopPress", "(Z)V");
   
	int result = -5;	
	if(spictl_dev)
    {
	    spictl_dev->start_press_dial(spictl_dev,data);
    }
	env->DeleteLocalRef(clazz);
}

static void android_debug_JNITest_startGroupAD(JNIEnv* env, jobject object,float data){
	mPressCallbacksObj = env->NewGlobalRef(object);
    jclass clazz = env->GetObjectClass(object);
	request_press_data_ID = env->GetMethodID(clazz,"requestData", "([FZ)V");
	request_press_stop_ID = env->GetMethodID(clazz,"requestStopPress", "(Z)V");
	
	int result = -5;	
	if(spictl_dev)
    {
	    spictl_dev->start_press_curve(spictl_dev,data);
	}
	env->DeleteLocalRef(clazz);
}

static void android_debug_JNITest_startCaliAD(JNIEnv* env, jobject object){ 
	mPressCallbacksObj = env->NewGlobalRef(object);
    jclass clazz = env->GetObjectClass(object);
	request_press_data_ID = env->GetMethodID(clazz,"requestData", "([FZ)V");
	request_press_stop_ID = env->GetMethodID(clazz,"requestStopPress", "(Z)V");
	int result = -5;
	if(spictl_dev)
    {
	   spictl_dev->start_press_flag0(spictl_dev);
    }
	env->DeleteLocalRef(clazz);
}

static jboolean android_debug_JNITest_stopAD(JNIEnv* env, jobject object){
	ALOGE("JNI层点击STOPAD，停止压力采集");
	if(spictl_dev){
		jboolean  isStop  = (spictl_dev->stop_press_ad(spictl_dev) == 0);
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
		spictl_ctl_open(&(spi_module->common),&spictl_dev);
		isStart = true;
	}else{
		 isStart = false;
	}
	if(isStart){
	   sSpiInterface  = spictl_dev->get_pressure_interface(spictl_dev);
	   sSpiInterface->initPress(&mSpiCb);
	}

	env->GetJavaVM(&gJavaVm); //获取一个Java虚拟机对象,参数：用来存放获得的虚拟机的指针的指针，成功返回0，失败返回其他
	jni_version = env->GetVersion();//获取当前的JNI版本
	return isStart;
}


/*
 * JNI registration.
 */
static JNINativeMethod gMethods[] = {
    /* name, signature, funcPtr */
    { "startNativeSingleAD",      "(F)V",            (void*) android_debug_JNITest_startSingleAD },
    { "startNativeGroupAD",      "(F)V",             (void*) android_debug_JNITest_startGroupAD },
	{ "startNativeCaliAD",      "()V",               (void*) android_debug_JNITest_startCaliAD },					 
    {"stopNativeAD",      "()Z",                     (void*) android_debug_JNITest_stopAD },
    {"startNativeInit",      "()Z",                  (void*) android_debug_JNITest_startNativeInit },
    {"getNativeADFrequency",      "()I",             (void*) android_debug_JNITest_getNativeADFrequency },
};


int register_android_iline_ILineGatherModule(JNIEnv* env)
{
	//return jniRegisterNativeMethods(env, "android/iline/ILineGatherModule", gMethods, NELEM(gMethods));
    return AndroidRuntime::registerNativeMethods(env, "android/iline/ILineGatherModule", gMethods, NELEM(gMethods));
}

}; 