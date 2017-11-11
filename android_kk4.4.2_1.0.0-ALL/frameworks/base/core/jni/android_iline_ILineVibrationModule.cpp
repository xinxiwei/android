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
#include <hardware/imx6q_spi_config.h>
#include <hardware/imx6q_spi.h>
#include <hardware/hardware.h>
#include <pthread.h>

namespace android 
{

struct spictl_device_t* spictl_dev2 = NULL;

static  const SpiVibrateInterface* sSpiInterface = NULL;

timewave my_timewave ={0,0,0,100,1000,0,0,0,0,1.0,1.0,0,0.0};
freqwave my_freqwave ={0,0,0,100,400,1,0,0,0,0,0,0,1.0,1.0,0,0.0};
totalrend my_totalrend ={0,0,0,100,1000,1.00,0,0,0,0,0,1.0,1.0};

static const int VIBRATION_SINGLE_FLAG = 1;
static const int VIBRATION_DOUBLE_FLAG = 2;
static const int VIBRATION_ROTATION_FLAG = 3;


//默认构造函数
jmethodID vib_data_methodID = NULL; //振动数据ID
jmethodID vib_stop_methodID = NULL;//振动停止ID
//对象名
jobject mVibCallbacksObj = NULL;
 
static JavaVM *gJavaVm = NULL;//JavaVM是虚拟机在JNI中的表示，一个JVM中只有一个JavaVM对象，这个对象是线程共享的

struct fields_t {
	jfieldID mPropType;
	jfieldID signalType; 
	jfieldID minFreq;
	jfieldID maxFreq;
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
	jfieldID versionMode;
    
};

static fields_t fields;
jint version = 0;

static void request_vibration_single_callback(float data[],int length, jboolean isCollectData){
    ALOGE("jni收到振动回调数据: length=%d,  data[0] = %f",length,data[0]);

	if(!gJavaVm){  //虚拟机为NULL时,直接返回
		return;
	}
			
	bool isAttacked  = false;
	JNIEnv* env = NULL;   //JNI_VERSION_1_4
	int status = gJavaVm->GetEnv((void **)&env, version); //从java 虚拟机获得javaEnv，第一个参数指向JavaEnv， 第二个参数为JNi 版本
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
	env->CallVoidMethod(mVibCallbacksObj, vib_data_methodID, farr, isCollectData); //调用java 对象中的方法。 其实也就是java 与native 通信的方法, 把对应的jobject，jMethodID还有对应的参数传递给java层

	if(farr){
		env->DeleteLocalRef(farr); //删除局部引用
	}
	
	
	if(isAttacked)
	{
		//ALOGE("jnitest: signal_callback isAttacked = %d",isAttacked);
		gJavaVm->DetachCurrentThread();  //解除当前线程与java虚拟机之间的链接 
	}
	
};

/*
static void request_vibration_error_callback(int data, jboolean isCollectData){
	if(!gJavaVm){  //虚拟机为NULL时,直接返回
		return;
	}	
	bool isAttacked  = false;
	JNIEnv* env = NULL;
    int status = gJavaVm->GetEnv((void **)&env, version); //从java 虚拟机获得javaEnv，第一个参数指向JavaEnv， 第二个参数为JNi 版本
	if(status <0) //说明没有得到JavaEnv
	{	
		status = gJavaVm->AttachCurrentThread(&env, NULL); //从当前线程获得javaEnv,链接当前线程到java虚拟机,第二个参数指向线程名， 可以为空
		if(status <0)
		{
			return ;
		}
		isAttacked  = true; 
	}	
	jint idata = data;
	env->CallVoidMethod(mVibCallbacksObj, vib_data_methodID, idata, isCollectData);

	if(isAttacked)
	{
	    gJavaVm->DetachCurrentThread();  //解除当前线程与java虚拟机之间的链接 
	}
}
*/
static void request_vibration_double_callback(float data[],float data2[],int length, jboolean isCollect){		
	if(!gJavaVm){
		return;
	}	
	
	bool isAttacked  = false;
	JNIEnv* env = NULL;
	int status = gJavaVm->GetEnv((void **)&env, version); //从java 虚拟机获得javaEnv，第一个参数指向JavaEnv， 第二个参数为JNi 版本
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

	env->CallVoidMethod(mVibCallbacksObj, vib_data_methodID, farr, farr2, isCollect);

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
    ALOGE("jni收到振动停止指令: request_vibration_stop_callback enter");	
	if (!gJavaVm) {
		return;
	}
    
	//JNIEnv* env = AndroidRuntime::getJNIEnv();
	JNIEnv* env = NULL;
	gJavaVm->GetEnv((void **)&env, version); //从java 虚拟机获得javaEnv，第一个参数指向JavaEnv， 第二个参数为JNi 版本
		
	env->CallVoidMethod(mVibCallbacksObj, vib_stop_methodID, isStop);

	gJavaVm = NULL;
	env->DeleteGlobalRef(mVibCallbacksObj);//删除全局引用
	vib_data_methodID = NULL;
	vib_stop_methodID = NULL;
	mVibCallbacksObj = NULL;	
	ALOGE("jnitest: request_vibration_stop_callback exit");
};


/* static pthread_t create_thread_callback(const char* name, void (*start)(void *), void* arg) //创建线程
{
	return (pthread_t)AndroidRuntime::createJavaThread(name, start, arg);
} */


static void request_single_callback(float data[],int length, bool isCollectData)  //单通道正常采集回调数据
{
	 request_vibration_single_callback(data, length, isCollectData);
}

//static void request_error_callback(int data, bool isCollectData)  // 错误回调
//{
	// request_vibration_error_callback(data,isCollectData);
//}

static void request_double_callback(float data[],float data2[],int length,bool isCollectData)//双通道正常采集回调数据
{
	 request_vibration_double_callback(data, data2, length, isCollectData);
}

static void request_stop_callback(bool isStop)//停止回调接口
{ 
	request_vibration_stop_callback(isStop);
}


SpiVibrateCallbacks mSpiCb2 = { //总的回调接口
	request_single_callback,
	request_double_callback,
	request_stop_callback,
   // request_error_callback	
};


static void spictl_ctl_open(const struct hw_module_t* module ,struct spictl_device_t** dev){ //打开设备文件
	module->methods->open(module,SPICTL_HARDWARE_MODULE_ID,(struct hw_device_t**)dev);
}

static jboolean android_debug_JNITest_startNativeinit(JNIEnv* env, jobject object){ //本地初始化接口
	spictl_module_t*  spi_module = NULL;
	jboolean isStart  = true;
	
	if(hw_get_module(SPICTL_HARDWARE_MODULE_ID,(const hw_module_t**)&spi_module) == 0){
		spictl_ctl_open(&(spi_module->common), &spictl_dev2);	
		isStart = true;
	}else{		
		 isStart = false;
	}
	
	if(isStart){
	   sSpiInterface  = spictl_dev2->get_vibrate_interface(spictl_dev2); //获取振动回调函数接口
	   sSpiInterface->initVibrate(&mSpiCb2);//初始化振动回调接口
	}
	
	env->GetJavaVM(&gJavaVm); //获取一个Java虚拟机对象,参数：用来存放获得的虚拟机的指针的指针，成功返回0，失败返回其他
	version = env->GetVersion();//获取当前的JNI版本
	return isStart;
}


static jboolean android_debug_JNITest_stopNativeAD(JNIEnv* env, jobject object){ //停止采集
	if(spictl_dev2){
		jboolean isStop  = (spictl_dev2->stop_vibrate_ad(spictl_dev2) == 0);
		return   isStop;
	}
	return false;
}

static void android_debug_JNITest_startNativeRotation(JNIEnv* env, jobject object){ //启动转速
	mVibCallbacksObj = env->NewGlobalRef(object); //C中新建全局引用
	jclass clazz = env->GetObjectClass(object);//从java类的一个引用获得一个jclass 对象
	
	vib_data_methodID = env->GetMethodID(clazz,"requestSingleData", "([FZ)V"); //C中
	
	vib_stop_methodID = env->GetMethodID(clazz,"requestStopCh", "(Z)V");
	if(spictl_dev2){
		spictl_dev2->start_rotation(spictl_dev2);
	}
	 env->DeleteLocalRef(clazz);
}

static void android_debug_JNITest_startNativeTimeWave(JNIEnv* env, jobject object,jint ch_num,jobject job){//启动时域波形
	mVibCallbacksObj = env->NewGlobalRef(object);
	jclass clazz = env->GetObjectClass(object); //从java类的一个引用获得一个jclass 对象	
	if(ch_num == 1)
	{
		vib_data_methodID = env->GetMethodID(clazz,"requestSingleData", "([FZ)V"); //GetMethodID 获取对应类的构造函数
	}else if(ch_num == 2){
		vib_data_methodID = env->GetMethodID(clazz,"requestDoubleChData", "([F[FZ)V");
	}	
	//vib_data_methodID = env->GetMethodID(clazz,"requestErrorData", "(IZ)V");
	vib_stop_methodID = env->GetMethodID(clazz,"requestStopCh", "(Z)V");
	
	
	jclass stucls = env->GetObjectClass(job); //从java类的一个引用获得一个jclass 对象
	fields.mPropType = env->GetFieldID(stucls,"propType","I"); // 获得Java 类中的对应变量的jfieldID, 第二个参数是变量名， 第三个参数是变量的类型
	fields.signalType = env->GetFieldID(stucls,"signalType","I");
	fields.minFreq = env->GetFieldID(stucls,"minFreq","F");
	fields.maxFreq = env->GetFieldID(stucls,"maxFreq","F");
	fields.waveLength = env->GetFieldID(stucls,"waveLength","I");
	fields.rangeMode= env->GetFieldID(stucls,"rangeMode","I");
	fields.rangeAccelerationValue= env->GetFieldID(stucls,"rangeAcceleration","I");
	fields.rangeDistanceValue= env->GetFieldID(stucls,"rangeDistanceValue","I");
	fields.rangeSpeendValue= env->GetFieldID(stucls,"rangeSpeendValue","I");
	fields.trigMode= env->GetFieldID(stucls,"syncMode","I");
	fields.trigValue= env->GetFieldID(stucls,"trigLevel","F");
	fields.versionMode= env->GetFieldID(stucls,"versionMode","I");
	
	my_timewave.data_type = env->GetIntField(job, fields.mPropType);
	my_timewave.signal_type = env->GetIntField(job, fields.signalType);
	my_timewave.min_freq = env->GetFloatField(job, fields.minFreq);
	my_timewave.max_freq = env->GetFloatField(job, fields.maxFreq);
	my_timewave.wave_length = env->GetIntField(job, fields.waveLength);
	my_timewave.range_mode= env->GetIntField(job, fields.rangeMode);
	my_timewave.range_accel_value = env->GetIntField(job, fields.rangeAccelerationValue);
	my_timewave.range_speed_value = env->GetIntField(job, fields.rangeDistanceValue);
	my_timewave.range_disp_value = env->GetIntField(job, fields.rangeSpeendValue);
	my_timewave.trig_mode = env->GetIntField(job, fields.trigMode);
	my_timewave.trig_value = env->GetFloatField(job, fields.trigValue);
    my_timewave.version_mode = env->GetIntField(job, fields.versionMode);
	
    //ALOGE("jnitest: my_timewave.version_mode = %d",my_timewave.version_mode);
	if(spictl_dev2){
		spictl_dev2->start_vibrate_timewave(spictl_dev2,ch_num,my_timewave);
	}
	env->DeleteLocalRef(clazz);
	env->DeleteLocalRef(stucls);
}


static void android_debug_JNITest_startNativeFreqWave(JNIEnv* env, jobject object,jint ch_num,jobject job){//启动频域波形
	mVibCallbacksObj = env->NewGlobalRef(object);
	jclass clazz = env->GetObjectClass(object);
	
	if(ch_num == 1)
	{
		vib_data_methodID = env->GetMethodID(clazz,"requestSingleData", "([FZ)V");
	}else if(ch_num == 2){
		vib_data_methodID = env->GetMethodID(clazz,"requestDoubleChData", "([F[FZ)V");
	}
	//vib_data_methodID = env->GetMethodID(clazz,"requestErrorData", "(IZ)V");
	vib_stop_methodID = env->GetMethodID(clazz,"requestStopCh", "(Z)V");	
	
	
	jclass stucls = env->GetObjectClass(job); 
	fields.mPropType = env->GetFieldID(stucls,"propType","I"); 
	fields.signalType = env->GetFieldID(stucls,"signalType","I");
	fields.minFreq = env->GetFieldID(stucls,"minFreq","F");
	fields.maxFreq = env->GetFieldID(stucls,"maxFreq","F");
	fields.spectraNum = env->GetFieldID(stucls,"spectraNum","I");
	fields.averageNum= env->GetFieldID(stucls,"aveTimes","I"); ;
	fields.averageType= env->GetFieldID(stucls,"aveMode","I");
	fields.addWinType= env->GetFieldID(stucls,"windowType","I");
	fields.rangeMode= env->GetFieldID(stucls,"rangeMode","I");
	fields.rangeAccelerationValue= env->GetFieldID(stucls,"rangeAcceleration","I");
	fields.rangeDistanceValue= env->GetFieldID(stucls,"rangeDistanceValue","I");
	fields.rangeSpeendValue= env->GetFieldID(stucls,"rangeSpeendValue","I");
	fields.trigMode= env->GetFieldID(stucls,"syncMode","I");
	fields.trigValue= env->GetFieldID(stucls,"trigLevel","F");
    fields.versionMode= env->GetFieldID(stucls,"versionMode","I");

	my_freqwave.data_type  = env->GetIntField(job, fields.mPropType);
	my_freqwave.signal_type = env->GetIntField(job, fields.signalType);
	my_freqwave.min_freq = env->GetFloatField(job, fields.minFreq);
	my_freqwave.max_freq = env->GetFloatField(job, fields.maxFreq);
	my_freqwave.spectra_num = env->GetIntField(job, fields.spectraNum);
	my_freqwave.average_num = env->GetIntField(job, fields.averageNum);
	my_freqwave.average_mode = env->GetIntField(job, fields.averageType);
	my_freqwave.window_type = env->GetIntField(job, fields.addWinType);
	my_freqwave.range_mode = env->GetIntField(job, fields.rangeMode);
	my_freqwave.range_accel_value = env->GetIntField(job, fields.rangeAccelerationValue);
	my_freqwave.range_disp_value = env->GetIntField(job, fields.rangeDistanceValue);
	my_freqwave.range_speed_value = env->GetIntField(job, fields.rangeSpeendValue);
	my_freqwave.trig_mode = env->GetIntField(job, fields.trigMode);
	my_freqwave.trig_value = env->GetFloatField(job, fields.trigValue);
    my_freqwave.version_mode = env->GetIntField(job, fields.versionMode);
    
	if(spictl_dev2){
		spictl_dev2->start_vibrate_freqwave(spictl_dev2, ch_num, my_freqwave);
	}
	env->DeleteLocalRef(clazz);
	env->DeleteLocalRef(stucls);
}


static void android_debug_JNITest_startNativeValueWave(JNIEnv* env, jobject object,jint ch_num,jobject job){//启动总值趋势
	mVibCallbacksObj = env->NewGlobalRef(object);
	jclass clazz = env->GetObjectClass(object);
	if(ch_num == 1)
	{
		vib_data_methodID = env->GetMethodID(clazz,"requestSingleData", "([FZ)V");
	}else if(ch_num == 2){
		vib_data_methodID = env->GetMethodID(clazz,"requestDoubleChData", "([F[FZ)V");
	}
	//vib_data_methodID = env->GetMethodID(clazz,"requestErrorData", "(IZ)V");
	vib_stop_methodID = env->GetMethodID(clazz,"requestStopCh", "(Z)V");
	
	jclass stucls = env->GetObjectClass(job); 
	fields.mPropType = env->GetFieldID(stucls,"propType","I"); 
	fields.signalType = env->GetFieldID(stucls,"signalType","I");
	fields.minFreq = env->GetFieldID(stucls,"minFreq","F");
	fields.maxFreq = env->GetFieldID(stucls,"maxFreq","F");
	fields.waveLength = env->GetFieldID(stucls,"waveLength","I");
	fields.intervalTime = env->GetFieldID(stucls,"intervalTimes","F");
	fields.totalValueType= env->GetFieldID(stucls,"tvtMode","I");
	fields.rangeMode= env->GetFieldID(stucls,"rangeMode","I");
	fields.rangeAccelerationValue= env->GetFieldID(stucls,"rangeAcceleration","I");
	fields.rangeDistanceValue= env->GetFieldID(stucls,"rangeDistanceValue","I");
	fields.rangeSpeendValue= env->GetFieldID(stucls,"rangeSpeendValue","I");
    fields.versionMode= env->GetFieldID(stucls,"versionMode","I");

	my_totalrend.data_type = env->GetIntField(job, fields.mPropType);
	my_totalrend.signal_type = env->GetIntField(job, fields.signalType);
	my_totalrend.min_freq = env->GetFloatField(job, fields.minFreq);
	my_totalrend.max_freq  = env->GetFloatField(job, fields.maxFreq);
	my_totalrend.wave_length = env->GetIntField(job, fields.waveLength);
	my_totalrend.interval_time  = env->GetFloatField(job, fields.intervalTime);
	my_totalrend.total_value_type = env->GetIntField(job, fields.totalValueType);
	my_totalrend.range_mode = env->GetIntField(job, fields.rangeMode);
	my_totalrend.range_accel_value = env->GetIntField(job, fields.rangeAccelerationValue);
	my_totalrend.range_disp_value = env->GetIntField(job, fields.rangeDistanceValue);
	my_totalrend.range_speed_value = env->GetIntField(job, fields.rangeSpeendValue);
    my_totalrend.version_mode = env->GetIntField(job, fields.versionMode);

	if(spictl_dev2){
		spictl_dev2->start_vibrate_totalrend(spictl_dev2,ch_num, my_totalrend);
	}
	env->DeleteLocalRef(clazz);
	env->DeleteLocalRef(stucls);
}


static void android_debug_JNITest_startNativeEvalute(JNIEnv* env, jobject object,jobject job){//启动等级评估
	mVibCallbacksObj = env->NewGlobalRef(object);
	jclass clazz = env->GetObjectClass(object);
	
    
	vib_data_methodID = env->GetMethodID(clazz,"requestEvaluteData", "([FZ)V");

	vib_stop_methodID = env->GetMethodID(clazz,"requestStopEvalute", "(Z)V");
	
	jclass stucls = env->GetObjectClass(job); 
	fields.mPropType = env->GetFieldID(stucls,"propType","I"); 
	fields.signalType = env->GetFieldID(stucls,"signalType","I");
	fields.minFreq = env->GetFieldID(stucls,"minFreq","F");
	fields.maxFreq = env->GetFieldID(stucls,"maxFreq","F");
	fields.waveLength = env->GetFieldID(stucls,"waveLength","I");
	fields.rangeMode= env->GetFieldID(stucls,"rangeMode","I");
	fields.rangeAccelerationValue= env->GetFieldID(stucls,"rangeAcceleration","I");
	fields.rangeDistanceValue= env->GetFieldID(stucls,"rangeDistanceValue","I");
	fields.rangeSpeendValue= env->GetFieldID(stucls,"rangeSpeendValue","I");
	fields.trigMode= env->GetFieldID(stucls,"syncMode","I");
	fields.trigValue= env->GetFieldID(stucls,"trigLevel","F");
    fields.versionMode= env->GetFieldID(stucls,"versionMode","I");

	my_timewave.data_type = env->GetIntField(job, fields.mPropType);
	my_timewave.signal_type = env->GetIntField(job, fields.signalType);
	my_timewave.min_freq = env->GetFloatField(job, fields.minFreq);
	my_timewave.max_freq = env->GetFloatField(job, fields.maxFreq);
	my_timewave.wave_length = env->GetIntField(job, fields.waveLength);
	my_timewave.range_mode= env->GetIntField(job, fields.rangeMode);
	my_timewave.range_accel_value = env->GetIntField(job, fields.rangeAccelerationValue);
	my_timewave.range_speed_value = env->GetIntField(job, fields.rangeDistanceValue);
	my_timewave.range_disp_value = env->GetIntField(job, fields.rangeSpeendValue);
	my_timewave.trig_mode = env->GetIntField(job, fields.trigMode);
	my_timewave.trig_value = env->GetFloatField(job, fields.trigValue);
	my_timewave.version_mode = env->GetIntField(job, fields.versionMode);
    
	if(spictl_dev2){
		spictl_dev2->start_vibrate_evalute(spictl_dev2,my_timewave);
	}
	env->DeleteLocalRef(clazz);
	env->DeleteLocalRef(stucls);
}


static jfloatArray  android_debug_JNITest_getNativeFeatureData(JNIEnv* env, jobject object, jfloatArray data, jint length){//获取15个特征值
   jfloat data_temp[length];
   env->GetFloatArrayRegion(data,0,length,data_temp);//将data数组复制到data_temp数组中
   
   jfloat* feature_data = spictl_dev2->spi_feature_value(spictl_dev2,data_temp,length);
   
   jfloatArray farr = env->NewFloatArray(15); //new分配的内存会返回，所以不用delete，java会自动回收
   env->SetFloatArrayRegion(farr, 0, 15, feature_data);
   env->ReleaseFloatArrayElements(farr,feature_data,0); //通知虚拟机释放float数组内存,farr:Java 数组对象,feature_data:指向数组元素的指针, 0: 释放模式
   return farr;
}


static jfloatArray  android_debug_JNITest_getNativeTimeToFreqData(JNIEnv* env, jobject object, jfloatArray data, jint length){//时域转频域
   jfloat data_temp[length];
   env->GetFloatArrayRegion(data,0,length,data_temp);
	   
   jfloat* cb_data = spictl_dev2->spi_timetofreq_value(spictl_dev2,data_temp,length);

   jfloatArray farr = env->NewFloatArray(length/2);
   env->SetFloatArrayRegion(farr, 0, length/2, cb_data);
   env->ReleaseFloatArrayElements(farr,cb_data,0);
   return farr;
}

/*
 * JNI registration.
 */
static JNINativeMethod gMethods[] = {
    /* name, signature, funcPtr */	
	{"startNativeinit",          "()Z", 	 								(void*) android_debug_JNITest_startNativeinit },			 
	{"stopNativeAD",             "()Z",	   									(void*) android_debug_JNITest_stopNativeAD },			   
	{"startNativeTimeWave",      "(ILandroid/iline/SgPropertyTime;)V",		(void*) android_debug_JNITest_startNativeTimeWave },		 
	{"startNativeFreqWave",      "(ILandroid/iline/SgPropertyFreq;)V",	  	(void*) android_debug_JNITest_startNativeFreqWave },					  
	{"startNativeValueWave",     "(ILandroid/iline/SgPropertyTotalTrend;)V",(void*) android_debug_JNITest_startNativeValueWave },					   
	{"startNativeEvalute",       "(Landroid/iline/SgPropertyBase;)V",	 	(void*) android_debug_JNITest_startNativeEvalute },						 
	{"startNativeRotation",      "()V",	   									(void*) android_debug_JNITest_startNativeRotation },					   
	{"getNativeFeatureData",     "([FI)[F",	   								(void*) android_debug_JNITest_getNativeFeatureData},					   
	{"getNativeTimeToFreqData",  "([FI)[F",									(void*) android_debug_JNITest_getNativeTimeToFreqData},
};

int register_android_iline_ILineVibrationModule(JNIEnv* env)
{
	return AndroidRuntime::registerNativeMethods(env, "android/iline/ILineVibrationModule", gMethods, NELEM(gMethods));
}
}; 


