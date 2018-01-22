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
#include <hardware/imx6q_vibrate_config.h>
#include <hardware/imx6q_calibrate.h>
#include <hardware/hardware.h>
#include <pthread.h>

namespace android{
    
struct spictl_device_t* spictl_dev3 = NULL;
static  const SpiVibrateInterface* sSpiInterface = NULL;

timewave my_timewave3 ={0,0,0.0,0.0,0.0,0.0,0,0,0,0,0,0,0,0,0,0,0};

//默认构造函数
jmethodID calib_data_methodID = NULL; //振动数据ID
jmethodID calib_stop_methodID = NULL;//振动停止ID
//对象名
jobject mCalibCallbacksObj = NULL;

static JavaVM *gJavaVm = NULL;//JavaVM是虚拟机在JNI中的表示，一个JVM中只有一个JavaVM对象，这个对象是线程共享的

struct fields_t {
	jfieldID mPropType;
	jfieldID signalType;
	jfieldID minFreq;
	jfieldID maxFreq;
    jfieldID calibExpecteValue; //校准期望值
	jfieldID waveLength;
	jfieldID spectraNum;
	jfieldID averageNum ;
	jfieldID averageType;
	jfieldID addWinType;
	jfieldID rangeMode;
	jfieldID rangeAccelerationValue;
	jfieldID rangeDistanceValue;
	jfieldID rangeSpeendValue;
	jfieldID rangeGainValue1;
	jfieldID rangeGainValue2;
	jfieldID trigMode;
	jfieldID trigValue;
	jfieldID intervalTime;
	jfieldID totalValueType;
	jfieldID versionMode; //版本模式
    jfieldID voltageRange;//电压量程
    jfieldID chanNum;//电压量程
};

static fields_t fields;
jint version3 = 0;

static void request_vibration_single_callback(float data[],int length, jboolean isCollectData){
    ALOGE("jni收到振动回调的数据====: length=%d,  data[0] = %f",length,data[0]);

	if(!gJavaVm){  //虚拟机为NULL时,直接返回
		return;
	}

	bool isAttacked  = false;
	JNIEnv* env = NULL;   //JNI_VERSION_1_4
	int status = gJavaVm->GetEnv((void **)&env, version3); //从java 虚拟机获得javaEnv，第一个参数指向JavaEnv， 第二个参数为JNi 版本
	if(status <0) //说明没有得到JavaEnv
	{
	    //ALOGE("jnitest: signal_callback status = %d",status);
		status = gJavaVm->AttachCurrentThread(&env, NULL); //从当前线程获得javaEnv,链接当前线程到java虚拟机,第二个参数指向线程名， 可以为空
		if(status <0)
		{
			return ;
		}
		isAttacked  = true;
	}

	jfloatArray farr = env->NewFloatArray(length); //ENV 调用自身创建float数组函数
	env->SetFloatArrayRegion(farr, 0, length, data); //从缓冲区data中复制 length长度数据到 farr数组中，赋值函数
	env->CallVoidMethod(mCalibCallbacksObj, calib_data_methodID, farr, isCollectData); //调用java 对象中的方法。 其实也就是java 与native 通信的方法, 把对应的jobject，jMethodID还有对应的参数传递给java层

	if(farr){
		env->DeleteLocalRef(farr); //删除局部引用
	}
	if(isAttacked)
	{
		//ALOGE("jnitest: signal_callback isAttacked = %d",isAttacked);
		gJavaVm->DetachCurrentThread();  //解除当前线程与java虚拟机之间的链接
	}
};

static void request_vibration_double_callback(float data[],float data2[],int length, jboolean isCollect){
	if(!gJavaVm){
		return;
	}

	bool isAttacked  = false;
	JNIEnv* env = NULL;
	int status = gJavaVm->GetEnv((void **)&env, version3); //从java 虚拟机获得javaEnv，第一个参数指向JavaEnv， 第二个参数为JNi 版本
	if(status <0) //说明没有得到JavaEnv
	{
		status = gJavaVm->AttachCurrentThread(&env, NULL); //从当前线程获得javaEnv,链接当前线程到java虚拟机,第二个参数指向线程名， 可以为空,链接成功返回0，连接失败返回其他
		if(status <0)
		{
			return ;
		}
		isAttacked  = true;
	}

	jfloatArray farr = env->NewFloatArray(length); //构造指定长度的数组
	env->SetFloatArrayRegion(farr, 0, length, data); //给数组赋值

	jfloatArray farr2 = env->NewFloatArray(length);
	env->SetFloatArrayRegion(farr2, 0, length, data2);

	env->CallVoidMethod(mCalibCallbacksObj, calib_data_methodID, farr, farr2, isCollect);

	if(farr){
		env->DeleteLocalRef(farr);
	}
	if(farr2){
	   env->DeleteLocalRef(farr2);
	}

	if(isAttacked)
	{
		gJavaVm->DetachCurrentThread();  //解除当前线程与java虚拟机之间的链接
	}
};

static void request_vibration_stop_callback(jboolean isStop){
    ALOGE("jni层收到振动停止回调的数据==== %d",isStop);
	if (!gJavaVm) {
		return;
	}

	JNIEnv* env = NULL;
	gJavaVm->GetEnv((void **)&env, version3); //从java 虚拟机获得javaEnv，第一个参数指向JavaEnv， 第二个参数为JNi 版本

	env->CallVoidMethod(mCalibCallbacksObj, calib_stop_methodID, isStop);

	gJavaVm = NULL;
	env->DeleteGlobalRef(mCalibCallbacksObj);//删除全局引用
	calib_data_methodID = NULL;
	calib_stop_methodID = NULL;
	mCalibCallbacksObj = NULL;
};

static void request_single_callback(float data[],int length, bool isCollectData)  //单通道正常采集回调数据
{
	 request_vibration_single_callback(data, length, isCollectData);
}

static void request_double_callback(float data[],float data2[],int length,bool isCollectData)//双通道正常采集回调数据
{
	 request_vibration_double_callback(data, data2, length, isCollectData);
}

static void request_stop_callback(bool isStop)//停止回调接口
{
	request_vibration_stop_callback(isStop);
}

SpiVibrateCallbacks mSpiCb3 = { //总的回调接口
	request_single_callback,
	request_double_callback,
	request_stop_callback,
};

static void spictl_ctl_open(const struct hw_module_t* module ,struct spictl_device_t** dev){ //打开设备文件
	module->methods->open(module,CALIBRATECTL_HARDWARE_MODULE_ID,(struct hw_device_t**)dev);
}

static jboolean android_debug_JNITest_startNativeinit(JNIEnv* env, jobject object){ //本地初始化接口
	spictl_module_t*  spi_module = NULL;
	jboolean isStart  = true;

	if(hw_get_module(CALIBRATECTL_HARDWARE_MODULE_ID,(const hw_module_t**)&spi_module) == 0){
		spictl_ctl_open(&(spi_module->common), &spictl_dev3);
		isStart = true;
	}else{
		 isStart = false;
	}

	if(isStart){
	   sSpiInterface  = spictl_dev3->get_vibrate_interface(spictl_dev3); //获取振动回调函数接口
	   sSpiInterface->initVibrate(&mSpiCb3);//初始化振动回调接口
	}

	env->GetJavaVM(&gJavaVm); //获取一个Java虚拟机对象,参数：用来存放获得的虚拟机的指针的指针，成功返回0，失败返回其他
	version3 = env->GetVersion();//获取当前的JNI版本
	return isStart;
}

static jboolean android_debug_JNITest_stopNativeAD(JNIEnv* env, jobject object){ //停止采集
    ALOGE("jni下发启动“振动停止”接口====");
	if(spictl_dev3){
		jboolean isStop  = (spictl_dev3->stop_vibrate_ad(spictl_dev3) == 0);
		return   isStop;
	}
	return false;
}

static void android_debug_JNITest_startNativeCalibrate(JNIEnv* env, jobject object, jobject job){//启动振动校准
    ALOGE("jni下发启动校准接口====");
	mCalibCallbacksObj = env->NewGlobalRef(object);
	jclass clazz = env->GetObjectClass(object);
	calib_data_methodID = env->GetMethodID(clazz,"requestCalibrateData", "([FZ)V");
	calib_stop_methodID = env->GetMethodID(clazz,"requestStopCalibrate", "(Z)V");

	jclass stucls = env->GetObjectClass(job);
	fields.mPropType = env->GetFieldID(stucls,"propType","I");
	fields.signalType = env->GetFieldID(stucls,"signalType","I");
	fields.minFreq = env->GetFieldID(stucls,"minFreq","F");
	fields.maxFreq = env->GetFieldID(stucls,"maxFreq","F");
	fields.calibExpecteValue = env->GetFieldID(stucls,"calibExpecteValue","F");
    
	fields.waveLength = env->GetFieldID(stucls,"waveLength","I");
	fields.rangeMode= env->GetFieldID(stucls,"rangeMode","I");
	fields.rangeAccelerationValue= env->GetFieldID(stucls,"rangeAcceleration","I");
	fields.rangeDistanceValue= env->GetFieldID(stucls,"rangeDistanceValue","I");
	fields.rangeSpeendValue= env->GetFieldID(stucls,"rangeSpeendValue","I");
	fields.trigMode= env->GetFieldID(stucls,"syncMode","I");
	fields.trigValue= env->GetFieldID(stucls,"trigLevel","F");
	fields.versionMode= env->GetFieldID(stucls,"versionMode","I");
	fields.voltageRange= env->GetFieldID(stucls,"voltageRange","I");
    fields.chanNum= env->GetFieldID(stucls,"chanNum","I");

	my_timewave3.data_type = env->GetIntField(job, fields.mPropType);
	my_timewave3.signal_type = env->GetIntField(job, fields.signalType);
	my_timewave3.min_freq = env->GetFloatField(job, fields.minFreq);
	my_timewave3.max_freq = env->GetFloatField(job, fields.maxFreq);
	my_timewave3.calib_expecte_value = env->GetFloatField(job, fields.calibExpecteValue);
    
	my_timewave3.wave_length = env->GetIntField(job, fields.waveLength);
	my_timewave3.range_mode= env->GetIntField(job, fields.rangeMode);
	my_timewave3.range_accel_value = env->GetIntField(job, fields.rangeAccelerationValue);
	my_timewave3.range_speed_value = env->GetIntField(job, fields.rangeDistanceValue);
	my_timewave3.range_disp_value = env->GetIntField(job, fields.rangeSpeendValue);
	my_timewave3.trig_mode = env->GetIntField(job, fields.trigMode);
	my_timewave3.trig_value = env->GetFloatField(job, fields.trigValue);
	my_timewave3.version_mode = env->GetIntField(job, fields.versionMode);
    my_timewave3.voltage_range = env->GetIntField(job, fields.voltageRange);
	my_timewave3.chan_num = env->GetIntField(job, fields.chanNum);
	if(spictl_dev3){
		spictl_dev3->start_vibrate_calibrate(spictl_dev3, my_timewave3);
	}
	env->DeleteLocalRef(clazz);
	env->DeleteLocalRef(stucls);
}

/*
 * JNI registration.
 */
static JNINativeMethod gMethods[] = {
    /* name, signature, funcPtr */
	{"startNativeinit",          "()Z", 	 								(void*) android_debug_JNITest_startNativeinit },
	{"stopNativeAD",             "()Z",	   									(void*) android_debug_JNITest_stopNativeAD },	
	{"startNativeCalibrate",     "(Landroid/iline/SgPropertyBase;)V",	 	(void*) android_debug_JNITest_startNativeCalibrate },
};

int register_android_iline_ILineCalibrateModule(JNIEnv* env)
{
	return AndroidRuntime::registerNativeMethods(env, "android/iline/ILineCalibrateModule", gMethods, NELEM(gMethods));
}
};
