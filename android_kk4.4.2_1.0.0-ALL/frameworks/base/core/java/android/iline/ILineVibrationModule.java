/*
 * Copyright(C) 2006 The Android Open Source Project
 *
 * Licensed under the Apache License,  Version 2.0(the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,  software
 * distributed under the License is distributed on an "AS IS" BASIS, 
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,  either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package android.iline;
import android.util.Log;
import android.os.Handler;
import android.os.Message;
import java.util.ArrayList;
/**
 * Simple JNI verification test.
 */
public class ILineVibrationModule {
	private int mPropType = 0;
	private SgPropertyBase mFpgaValue; 
	private VibrationDataCallBack mVibrateCb = null;
	private EvaluteDataCallBack mEvaluteCb = null;

	private static String TAG = "ILineVibrationModule";

	public void setFpgaData(SgPropertyBase value){ //对上APP接口
		mFpgaValue = value;
	}
	
	public SgPropertyBase getFpgaData(){
		return mFpgaValue;  //结构体参数
	}
	
	public float[] getFeatureData(float[] data, int length){		
		return getNativeFeatureData(data, length);
	}	
	
	public float[] getTimeToFreqValue(float[] data, int length){
		float[] data2 = getNativeTimeToFreqData(data, length);		
		return data2;
	}
	
	public void setVibrationCallBack(VibrationDataCallBack cb){
	   mVibrateCb = cb;
	}
	 
	public boolean startAD(){ //启动采集
		boolean isSucess = startNativeinit();
		if(isSucess){
		    if(mFpgaValue instanceof SgPropertyTime){
		    	Log.i(TAG ,"振动采集调用start接口");
			    startNativeTimeWave((SgPropertyTime)mFpgaValue);
		    }else if(mFpgaValue instanceof SgPropertyTotalTrend){
			    startNativeValueWave((SgPropertyTotalTrend)mFpgaValue);
		    }
		}
		return isSucess;
	}

	public boolean startEvalute(SgPropertyBase fgpaValue){ //启动评估
		boolean isSucess = startNativeinit();
		if(isSucess){
			startNativeEvalute((SgPropertyBase)fgpaValue); 
		}
		return isSucess;
	}
	
	public boolean stopEvalute(){
		return stopAD();
	}

	public void setEvaluteCallBack(EvaluteDataCallBack cb){
		mEvaluteCb = cb;
	}
	
	public boolean stopAD(){
	   Log.i(TAG ,"振动采集调用stop接口");
	   return stopNativeAD();
	}
	
    private void requestSingleData(float valuesChA[], boolean isCollectData){
        if(mVibrateCb != null){
        	mVibrateCb.notifyVibrationData(valuesChA, isCollectData);
        }
    }
    
    private void requestDoubleChData(float valuesChA[], float valuesChB[], boolean isCollectData){ 
        if(mVibrateCb != null){
         	mVibrateCb.notifyVibrationDoubleChData(valuesChA, valuesChB, isCollectData);
        }
    }
    
    private void requestStopCh(boolean isStop){		
		if(mVibrateCb != null){
			mVibrateCb.notifyStopAD(isStop);
		}
	}

	private void requestEvaluteData(float evaluteData[], boolean isCollectData){
		if(mEvaluteCb != null){
			mEvaluteCb.notifyEvaluteData(evaluteData, isCollectData);
		}
	}
	
	private void requestStopEvalute(boolean isStop){
		if(mEvaluteCb != null){
			mEvaluteCb.notifyStopAD(isStop);
		}
	}

	public interface VibrationDataCallBack{
		public void notifyVibrationData(float data[], boolean isCollectData);
		public void notifyVibrationDoubleChData(float data[], float data2[], boolean isCollectData);
		//public void notifyVibrationErrorData(int data, boolean isCollectData);
		public void notifyStopAD(boolean isStop);
	}

	public interface EvaluteDataCallBack{
		public void notifyEvaluteData(float data[], boolean isCollectData);
		//public void notifyVibrationErrorData(int data, boolean isCollectData);
		public void notifyStopAD(boolean isStop);
	}
	
	 private native void startNativeTimeWave(SgPropertyTime value);
	 private native void startNativeValueWave(SgPropertyTotalTrend value);
	 private native void  startNativeEvalute(SgPropertyBase fgpaValue);
	 
	 private native float[] getNativeFeatureData(float data[], int length);
	 private native float[] getNativeTimeToFreqData(float data[], int length);
	 private native boolean startNativeinit();
	 private native boolean stopNativeAD();	 
}

