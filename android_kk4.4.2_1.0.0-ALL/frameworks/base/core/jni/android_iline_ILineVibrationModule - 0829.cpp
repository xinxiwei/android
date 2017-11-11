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
namespace android {



struct spictl_device_t* spictl_dev2 = NULL;
static  const SpiVibrateInterface* sSpiInterface = NULL;
timewave my_timewave ={0,0,0,100,1000,0,0,0,0,1.0,1.0,0,0.0};
freqwave my_freqwave ={0,0,0,100,400,1,0,0,0,0,0,0,1.0,1.0,0,0.0};
totalrend my_totalrend ={0,0,0,100,1000,1.00,0,0,0,0,0,1.0,1.0};

static const int VIBRATION_SINGLE_FLAG = 1;
static const int VIBRATION_DOUBLE_FLAG = 2;
static const int VIBRATION_ROTATION_FLAG = 3;
 jmethodID method_request_java_cb2 = NULL;
 jmethodID method_request_java_cb3 = NULL;
 jobject mCallbacksObj2 = NULL;
 
static JavaVM *gJavaVm = NULL;
struct fields_t {

	jfieldID mPropType;
    jfieldID signalType; //??????
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
};

static fields_t fields;


static void request_vibration_single_callback(float data[],int length,jboolean isCollectData){
    for(int i= 0;i<1;i++){
	 ALOGE("jnitest: length=%d,  data[%d] = %lf",length,i,data[i]);
	}

	if(!gJavaVm){
		return;
	}
    JNIEnv *lhxEnv;
   	gJavaVm->AttachCurrentThread(&lhxEnv, 0);
    JNIEnv* env = AndroidRuntime::getJNIEnv();
    if(env){
    	; //ALOGE("jnitest111");
    }else{
    	; //ALOGE("jnitest222");
    }

    jfloatArray iarr = env->NewFloatArray(length);//??????jintArray
    env->SetFloatArrayRegion(iarr, 0, length, data);
    env->CallVoidMethod(mCallbacksObj2, method_request_java_cb2, iarr,isCollectData);

    if(iarr){
    	env->DeleteLocalRef(iarr);
    }
	if(gJavaVm)
       gJavaVm->DetachCurrentThread();    
};


static void request_vibration_double_callback(float data[],float data2[],int length,jboolean isCollect){
    for(int i= 0;i<1;i++){
	 ALOGE("jnitest:length=%d, data[%d] =%f,data2[%d] =%f",length,i,data[i],i,data2[i]);
	}	

	if(!gJavaVm){
		return;
	}
    JNIEnv *lhxEnv;
   	gJavaVm->AttachCurrentThread(&lhxEnv, 0);
    JNIEnv* env = AndroidRuntime::getJNIEnv();
    if(env){
    	;// ALOGE("jnitest111");
    }else{
    	;// ALOGE("jnitest222");
    }

    jfloatArray iarr = env->NewFloatArray(length);//??????jintArray
    env->SetFloatArrayRegion(iarr, 0, length, data);

    jfloatArray iarr2 = env->NewFloatArray(length);//??????jintArray
    env->SetFloatArrayRegion(iarr2, 0, length, data2);

    env->CallVoidMethod(mCallbacksObj2, method_request_java_cb2, iarr,iarr2,isCollect);

    if(iarr){
    	env->DeleteLocalRef(iarr);
    }
    if(iarr2){
       env->DeleteLocalRef(iarr2);
     }
	if(gJavaVm)
       gJavaVm->DetachCurrentThread();    
};



static void request_vibration_stop_callback(jboolean isStop){
	if (!gJavaVm) {
		return;
	}
	/*JNIEnv *lhxEnv;
	gJavaVm->AttachCurrentThread(&lhxEnv, 0);*/
	JNIEnv* env = AndroidRuntime::getJNIEnv();
	env->CallVoidMethod(mCallbacksObj2, method_request_java_cb3, isStop);
/*	if (gJavaVm)
		gJavaVm->DetachCurrentThread();*/

	gJavaVm = NULL;
	// env->DeleteGlobalRef(method_request_java_cb2);
	env->DeleteGlobalRef(mCallbacksObj2);
	method_request_java_cb2 = NULL;
	method_request_java_cb3 = NULL;
	mCallbacksObj2 = NULL;
};
static pthread_t create_thread_callback(const char* name, void (*start)(void *), void* arg)
{
    return (pthread_t)AndroidRuntime::createJavaThread(name, start, arg);
}
static void request_single_callback(float data[],int length,bool isCollectData){

	 request_vibration_single_callback(data,length,isCollectData);
}

static void request_double_callback(float data[],float data2[],int length,bool isCollectData){

	 request_vibration_double_callback(data,data2,length,isCollectData);
}

static void request_stop_callback(bool isStop){

	request_vibration_stop_callback(isStop);
}
SpiVibrateCallbacks mSpiCb2 = {
	request_single_callback,
	request_double_callback,
	request_stop_callback	
};


static void spictl_ctl_open(const struct hw_module_t* module ,struct spictl_device_t** dev){
	module->methods->open(module,SPICTL_HARDWARE_MODULE_ID,(struct hw_device_t**)dev);
}


static jboolean android_debug_JNITest_startNativeinit(JNIEnv* env, jobject object){

	spictl_module_t*  spi_module = NULL;
	jboolean isStart  = true;
	if(hw_get_module(SPICTL_HARDWARE_MODULE_ID,(const hw_module_t**)&spi_module) == 0){
		spictl_ctl_open(&(spi_module->common),&spictl_dev2);//////
		isStart = true;
	}else{
		 isStart = false;
	}
	if(isStart){
	   sSpiInterface  = spictl_dev2->get_vibrate_interface(spictl_dev2);
	   sSpiInterface->initVibrate(&mSpiCb2);
	}
	int tempret = env->GetJavaVM(&gJavaVm);
	return isStart;
}

static jboolean android_debug_JNITest_stopNativeAD(JNIEnv* env, jobject object){
	if(spictl_dev2){
	  jboolean 	 isStop  = (spictl_dev2->stop_vibrate_ad(spictl_dev2) == 0);
	  return   isStop;
	}
	return false;
}



static void android_debug_JNITest_startNativeRotation(JNIEnv* env, jobject object){
	mCallbacksObj2 = env->NewGlobalRef(object);
    jclass clazz = env->GetObjectClass(object);
	method_request_java_cb2 = env->GetMethodID(clazz,"requestSingleData", "([FZ)V");
	
	method_request_java_cb3 = env->GetMethodID(clazz,"requestStopCh", "(Z)V");
	if(spictl_dev2){
		spictl_dev2->start_rotation(spictl_dev2);
	}
	 env->DeleteLocalRef(clazz);


}
static void android_debug_JNITest_startNativeTimeWave(JNIEnv* env, jobject object,jint ch_num,jobject job){

	mCallbacksObj2 = env->NewGlobalRef(object);
    jclass clazz = env->GetObjectClass(object);
    if(ch_num == 1)
    {
	  method_request_java_cb2 = env->GetMethodID(clazz,"requestSingleData", "([FZ)V");
    }else if(ch_num == 2){
      method_request_java_cb2 = env->GetMethodID(clazz,"requestDoubleChData", "([F[FZ)V");
    }
    method_request_java_cb3 = env->GetMethodID(clazz,"requestStopCh", "(Z)V");
	jclass stucls = env->GetObjectClass(job); //???Student??????
    fields.mPropType = env->GetFieldID(stucls,"propType","I"); //????Student???????id
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
    //ALOGE("TimeWave data is = %d = %d = %lf = %f  ",my_timewave.data_type,my_timewave.signal_type, my_timewave.min_freq,my_timewave.max_freq);
    if(spictl_dev2){
    		spictl_dev2->start_vibrate_timewave(spictl_dev2,ch_num,my_timewave);
    	}
    env->DeleteLocalRef(clazz);
    env->DeleteLocalRef(stucls);
}


static void android_debug_JNITest_startNativeFreqWave(JNIEnv* env, jobject object,jint ch_num,jobject job){

	mCallbacksObj2 = env->NewGlobalRef(object);
    jclass clazz = env->GetObjectClass(object);
    if(ch_num == 1)
      {
  	  method_request_java_cb2 = env->GetMethodID(clazz,"requestSingleData", "([FZ)V");
      }else if(ch_num == 2){
        method_request_java_cb2 = env->GetMethodID(clazz,"requestDoubleChData", "([F[FZ)V");
      }
    method_request_java_cb3 = env->GetMethodID(clazz,"requestStopCh", "(Z)V");
	jclass stucls = env->GetObjectClass(job); //???Student??????

	fields.mPropType = env->GetFieldID(stucls,"propType","I"); //????Student???????id
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
    //ALOGE("FreqWave data is = %d = %d = %lf = %f  ", my_freqwave.data_type, my_freqwave.signal_type, my_freqwave.min_freq,my_freqwave.max_freq);

    if(spictl_dev2){
       		spictl_dev2->start_vibrate_freqwave(spictl_dev2,ch_num,my_freqwave);
       	}
    env->DeleteLocalRef(clazz);
    env->DeleteLocalRef(stucls);
}


static void android_debug_JNITest_startNativeValueWave(JNIEnv* env, jobject object,jint ch_num,jobject job){

	mCallbacksObj2 = env->NewGlobalRef(object);
    jclass clazz = env->GetObjectClass(object);
    if(ch_num == 1)
    {
	  method_request_java_cb2 = env->GetMethodID(clazz,"requestSingleData", "([FZ)V");
    }else if(ch_num == 2){
      method_request_java_cb2 = env->GetMethodID(clazz,"requestDoubleChData", "([F[FZ)V");
    }
    method_request_java_cb3 = env->GetMethodID(clazz,"requestStopCh", "(Z)V");
	jclass stucls = env->GetObjectClass(job); //???Student??????
    fields.mPropType = env->GetFieldID(stucls,"propType","I"); //????Student???????id
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

    //ALOGE("ValueWave data is = %d = %d = %lf  = %f",my_totalrend.data_type,my_totalrend.signal_type, my_totalrend.min_freq,	my_totalrend.max_freq );
     if(spictl_dev2){
        		spictl_dev2->start_vibrate_totalrend(spictl_dev2,ch_num,my_totalrend);
        	}
     env->DeleteLocalRef(clazz);
     env->DeleteLocalRef(stucls);
}


static void android_debug_JNITest_startNativeEvalute(JNIEnv* env, jobject object,jobject job){

	mCallbacksObj2 = env->NewGlobalRef(object);
    jclass clazz = env->GetObjectClass(object);

	 method_request_java_cb2 = env->GetMethodID(clazz,"requestEvaluteData", "([FZ)V");
	 method_request_java_cb3 = env->GetMethodID(clazz,"requestStopEvalute", "(Z)V");
    jclass stucls = env->GetObjectClass(job); //???Student??????
    fields.mPropType = env->GetFieldID(stucls,"propType","I"); //????Student???????id
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
     if(spictl_dev2){
        		spictl_dev2->start_vibrate_evalute(spictl_dev2,my_timewave);
        	}
     env->DeleteLocalRef(clazz);
     env->DeleteLocalRef(stucls);
}
static jfloatArray  android_debug_JNITest_getNativeFeatureData(JNIEnv* env, jobject object,jfloatArray data,jint length){


       jfloat data_tmep[length];
       env->GetFloatArrayRegion(data,0,length,data_tmep);
	   jfloat* cb_data = spictl_dev2->spi_feature_value(spictl_dev2,data_tmep,length);
	   jfloat data_cb_tmep[15];
	   for(int i= 0; i<15;i++){
		   data_cb_tmep[i] = cb_data[i];
		   //ALOGE("ValueWave data_cb_tmep[%d] = %f",i, data_cb_tmep[i]); 
	   }

	   jfloatArray iarr = env->NewFloatArray(15);
	   env->SetFloatArrayRegion(iarr, 0, 15, data_cb_tmep);
	   env->ReleaseFloatArrayElements(iarr,cb_data,0);
	   return iarr;
}


static jfloatArray  android_debug_JNITest_getNativeTimeToFreqData(JNIEnv* env, jobject object,jfloatArray data,jint length){


       jfloat data_tmep[length];
       env->GetFloatArrayRegion(data,0,length,data_tmep);
	   
	   
	   jfloat* cb_data = spictl_dev2->spi_timetofreq_value(spictl_dev2,data_tmep,length);
	   jfloat data_cb_tmep[length/2];
	   
	   for(int i= 0; i<length/2;i++){
		   data_cb_tmep[i] = cb_data[i];
		   //ALOGE("ValueWave_time_to_freq[%d] = %f",i, data_cb_tmep[i]);
	   }

	   jfloatArray iarr = env->NewFloatArray(length/2);
	   env->SetFloatArrayRegion(iarr, 0, length/2, data_cb_tmep);
	   
	   env->ReleaseFloatArrayElements(iarr,cb_data,0);
	   return iarr;
}

static JNINativeMethod gMethods[] = {

		{"startNativeinit",      "()Z",
	             (void*) android_debug_JNITest_startNativeinit },
	    {"stopNativeAD",      "()Z",
	               (void*) android_debug_JNITest_stopNativeAD },
		{"startNativeTimeWave",      "(ILandroid/iline/SgPropertyTime;)V",
             (void*) android_debug_JNITest_startNativeTimeWave },
        {"startNativeFreqWave",      "(ILandroid/iline/SgPropertyFreq;)V",
                          (void*) android_debug_JNITest_startNativeFreqWave },
        {"startNativeValueWave",      "(ILandroid/iline/SgPropertyTotalTrend;)V",
                           (void*) android_debug_JNITest_startNativeValueWave },
          {"startNativeEvalute",      "(Landroid/iline/SgPropertyBase;)V",
                             (void*) android_debug_JNITest_startNativeEvalute },
        {"startNativeRotation",      "()V",
                   	       (void*) android_debug_JNITest_startNativeRotation },
        {"getNativeFeatureData",      "([FI)[F",
                   	       (void*) android_debug_JNITest_getNativeFeatureData},
        {"getNativeTimeToFreqData",      "([FI)[F",
                   	        (void*) android_debug_JNITest_getNativeTimeToFreqData},

};
int register_android_iline_ILineVibrationModule(JNIEnv* env)
{
    return jniRegisterNativeMethods(env, "android/iline/ILineVibrationModule",
        gMethods, NELEM(gMethods));
}

}; // namespace android


