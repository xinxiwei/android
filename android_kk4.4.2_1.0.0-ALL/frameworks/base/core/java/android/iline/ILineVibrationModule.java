/*
 * Copyright( C ) 2006 The Android Open Source Project
 *
 * Licensed under the Apache License ,  Version 2.0( the "License" );
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing ,  software
 * distributed under the License is distributed on an "AS IS" BASIS , 
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND ,  either express or implied.
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
	private VibrationDataCallBack mCallBack = null;
	private EvaluteDataCallBack mEvaluteCb = null;
	public static final int VIBRATION_SINGLE_TYPE = 1;
	public static final int VIBRATION_DOUBLE_TYPE = 2;
	public static final int VIBRATION_ROTATION_TYPE = 3;
	private static String TAG = "ILineVibrationModule";

	public void setFpgaData( SgPropertyBase value ){ //对上APP接口
		mFpgaValue = value;
	}
	
	public SgPropertyBase getFpgaData(  ){
		return mFpgaValue;  //结构体参数
	}
	
	public float[] getFeatureData( float[] data , int length ){		
		return getNativeFeatureData( data , length );
	}	
	
	public float[] getTimeToFreqValue( float[] data , int length ){
		float[] data2 = getNativeTimeToFreqData( data , length );		
		return data2;
	}
	
	public void setVibrationCallBack( VibrationDataCallBack cb ){
	   mCallBack = cb;
	}
	 
	public boolean startAD( int ch_num ){ //对上APP接口
		boolean isSucess = startNativeinit(  );
		if( isSucess ){
			if( ch_num == VIBRATION_SINGLE_TYPE  || ch_num == VIBRATION_DOUBLE_TYPE ){
			    if( mFpgaValue instanceof SgPropertyTime ){
			    	Log.i(TAG ,"振动采集调用start接口");
				    startNativeTimeWave( ch_num ,( SgPropertyTime )mFpgaValue );
			    }else if( mFpgaValue instanceof SgPropertyTotalTrend ){
				    startNativeValueWave( ch_num ,( SgPropertyTotalTrend )mFpgaValue );
			    }
		    }
		}
		return isSucess;
	}

	public boolean startEvalute( SgPropertyBase fgpaValue ){
		boolean isSucess = startNativeinit(  );
		if( isSucess ){
			startNativeEvalute(( SgPropertyBase )fgpaValue ); 
		}
		return isSucess;
	}
	
	public boolean stopEvalute(  ){
		return stopAD(  );
	}
	
	public void setEvaluteCallBack( EvaluteDataCallBack cb ){
		mEvaluteCb = cb;
	}
	
	public boolean stopAD(  ){
	   Log.i(TAG ,"振动采集调用stop接口");
	   return stopNativeAD(  );
	}
	
    private void requestSingleData( float valuesChA[] , boolean isCollectData ){
        if( mCallBack != null ){
        	mCallBack.notifyVibrationData( valuesChA , isCollectData );
        }
    }
    
    private void requestDoubleChData( float valuesChA[] , float valuesChB[] , boolean isCollectData ){ 
        if( mCallBack != null ){
         	mCallBack.notifyVibrationDoubleChData( valuesChA , valuesChB , isCollectData );
        }
    }
    
    private void requestStopCh( boolean isStop ){		
		if( mCallBack != null ){
			mCallBack.notifyStopAD( isStop );
		}
	}
/*	
	private void requestErrorData(int data, boolean isCollectData ){		
		if( mCallBack != null ){
			mCallBack.notifyVibrationErrorData(data, isCollectData );
		}
	}
	*/
	private void requestEvaluteData( float evaluteData[] , boolean isCollectData ){
		if( mEvaluteCb != null ){
			mEvaluteCb.notifyEvaluteData( evaluteData , isCollectData );
		}
	}
	
	private void requestStopEvalute( boolean isStop ){
		if( mEvaluteCb != null ){
			mEvaluteCb.notifyStopAD( isStop );
		}
	}

	public interface VibrationDataCallBack{
		public void notifyVibrationData( float data[] , boolean isCollectData );
		public void notifyVibrationDoubleChData( float data[] , float data2[] , boolean isCollectData );
		//public void notifyVibrationErrorData(int data, boolean isCollectData );
		public void notifyStopAD( boolean isStop );
	}

	public interface EvaluteDataCallBack{
		public void notifyEvaluteData( float data[] , boolean isCollectData );
		//public void notifyVibrationErrorData(int data, boolean isCollectData );
		public void notifyStopAD( boolean isStop );
	}
	
	 private native void startNativeTimeWave( int ch_num ,  SgPropertyTime value );
	 private native void startNativeValueWave( int ch_num ,  SgPropertyTotalTrend value );
	 private native void  startNativeEvalute( SgPropertyBase fgpaValue );
	 
	 private native float[] getNativeFeatureData( float data[] , int length );
	 private native float[] getNativeTimeToFreqData( float data[] , int length );
	 private native boolean startNativeinit(  );
	 private native boolean stopNativeAD(  );
	 
}

