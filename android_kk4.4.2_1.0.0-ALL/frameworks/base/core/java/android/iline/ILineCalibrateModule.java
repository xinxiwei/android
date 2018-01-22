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
public class ILineCalibrateModule {
	private int mPropType = 0;
	private CalibrateDataCallBack mCalibrateCb = null;

	private static String TAG = "ILineCalibrateModule";

	public void startCalibrate(SgPropertyBase property){//启动校准
		Log.i(TAG ,"振动校准调用start接口");
		boolean isSucess = startNativeinit();
		if(isSucess){
			startNativeCalibrate(property); 
		}
	}
	
	public void stopCalibrate(){
		stopAD();
	}
    
    public void setCalibrateCallBack(CalibrateDataCallBack cb){
		mCalibrateCb = cb;
	}
	
	public void stopAD(){
	   Log.i(TAG ,"振动校准调用stop接口");
	   stopNativeAD();
	}  

	private void requestCalibrateData(float calibrateData[], boolean isCollectData){
		Log.i(TAG ,"回调gain = " + calibrateData[0] + " ,offset = " + calibrateData[1]);
		if(mCalibrateCb != null){
			mCalibrateCb.notifyCalibrateData(calibrateData, isCollectData);
		}
	}
	
	private void requestStopCalibrate(boolean isStop){
		if(mCalibrateCb != null){
			mCalibrateCb.notifyStopAD(isStop);
		}
	}	

	public interface CalibrateDataCallBack{
		public void notifyCalibrateData(float data[], boolean isCollectData);
		public void notifyStopAD(boolean isStop);
	}

	private native void  startNativeCalibrate(SgPropertyBase fgpaValue);
	private native boolean startNativeinit();
	private native boolean stopNativeAD();	 	
}

