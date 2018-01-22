/*
 * Copyright (C) 2006 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
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
public class ILineGatherModule {
	private  PressDataCallBack mCallBack;
	private static String TAG = "ILineGatherModule";
	private int mMode = 0;
	public static final int MODE_SINGLE = 0;
	public static final int MODE_GROUP = 1;
	public static final int MODE_CALIBRATE = 2;
	
	private static final int MESSAGE_WHAT = 100;
	private static final int MESSAGE_STAT_AD = 101;

	private  class MyHandler extends Handler{	   
	       @Override
	       public void handleMessage(Message msg) {
	    	   float data = (Float)(msg.obj);
               if(msg.what == MESSAGE_STAT_AD){
	        	  if(mMode == MODE_SINGLE){
	        		  startNativeSingleAD(data);
	        	  }else if(mMode == MODE_GROUP){
	        		  startNativeGroupAD(data);
	        	  }
				  else if (mMode == MODE_CALIBRATE){
	        		  startNativeCaliAD();
	        	  }
	          }
	       }
	};

	private  MyHandler mHandler = new MyHandler();
    public ILineGatherModule( ) {
    
    }
    
    public void setCallBack(PressDataCallBack cb){
    	mCallBack = cb;
    	Log.i(TAG,"mCallBack"+mCallBack);
    }
    
    public PressDataCallBack getCallBack(){
    	return mCallBack;
    }
    public boolean  startAD(float data){    	
       boolean isOpen = startNativeInit();
       if(isOpen){    	   
    	   if(mMode == MODE_SINGLE){
     		  startNativeSingleAD(data);
     	  }else if(mMode == MODE_GROUP){
     		  startNativeGroupAD(data);
     	  }
			  else if (mMode == MODE_CALIBRATE){
     		  startNativeCaliAD();
     	  }
       }
       return isOpen;
    }
    
    public boolean  stopAD(){    
    	//Log.i(TAG,"stop");
    	return stopNativeAD();
    }
    
    public void setTimeInterval(int value){

    }
    
    public void setMode(int mode ){
    	mMode = mode;
    
    }
    public int getMode(){
    	return mMode; 
    }
    public int getADFrequency(){
    	return getNativeADFrequency();
    }
    private void requestData(float data[],boolean isCollectData){
    	if(mMode == MODE_SINGLE  ){
    		mCallBack.notifySingleData(data[0],isCollectData);
    	}else if(mMode == MODE_GROUP){
    		mCallBack.notifyGroupData(data,isCollectData);
    	}else if(mMode == MODE_CALIBRATE){
    		mCallBack.notifyCalibrateData(data[0],isCollectData);
    	}    
    }
    private void requestStopPress( boolean isStop ){
		Log.i(TAG,"framework收到JNI 停止采集回调的状态  " + isStop);
		if( mCallBack != null ){
			mCallBack.notifyStopAD( isStop );
		}
	}
    public interface PressDataCallBack{
        public void notifySingleData(float data,boolean isCollectData);
        public void notifyCalibrateData(float data,boolean isCollectData);
        public void notifyGroupData(float data[],boolean isCollectData);
	    public void notifyStopAD( boolean isStop );
      
     }

    private native void startNativeSingleAD(float data);
    private native void startNativeGroupAD(float data);
	private native void startNativeCaliAD();
	
    private native boolean stopNativeAD();
    private native boolean startNativeInit();
    private native int getNativeADFrequency();
      
}

