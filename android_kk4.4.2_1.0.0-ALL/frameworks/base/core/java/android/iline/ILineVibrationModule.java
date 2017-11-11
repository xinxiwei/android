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

	public void setFpgaData( SgPropertyBase value ){
		mFpgaValue = value;
	}
	
	public SgPropertyBase getFpgaData(  ){
		return mFpgaValue;
	}
	
	public float[] getFeatureData( float[] data , int length ){
		
		return getNativeFeatureData( data , length );
	}
	
	
	public float[] getTimeToFreqValue( float[] data , int length ){
		float[] data2 = getNativeTimeToFreqData( data , length );
		//for( int i= 0;i< data2.length;i++ ){
			//Log.i( "cpzgetTimeToFreqValue" , "i= " + i  +  " value " + data2[i] );
		//}
		return data2;//getNativeTimeToFreqData( data , length );
	}
	public void setVibrationCallBack( VibrationDataCallBack cb ){
	   mCallBack = cb;
	 }
	public boolean startAD( int ch_num ){
		boolean isSucess = startNativeinit(  );
		 //Log.i( "cpz" , "startAD  = " + isSucess + ",ch_num = "+ch_num );
	
		if( isSucess ){
			if( ch_num == VIBRATION_SINGLE_TYPE  || ch_num == VIBRATION_DOUBLE_TYPE ){
			   if( mFpgaValue instanceof SgPropertyTime ){
				  //Log.i( "cpz" , "SgPropertyTime" );
				   //mFpgaValue  = getTime(  );
				   startNativeTimeWave( ch_num ,( SgPropertyTime )mFpgaValue );
			   }else if( mFpgaValue instanceof SgPropertyFreq ){
				   //Log.i( "cpz" , "SgPropertyFreq" );
				   //mFpgaValue = getFreq(  );
				   startNativeFreqWave( ch_num ,( SgPropertyFreq )mFpgaValue );
			   }else if( mFpgaValue instanceof SgPropertyTotalTrend ){
				   //Log.i( "cpz" , "SgPropertyTotalTrend" );
				   //mFpgaValue = getTotal(  );
				   startNativeValueWave( ch_num ,( SgPropertyTotalTrend )mFpgaValue );
			   }
		    }else {
				   startNativeRotation(  );
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
	   return stopNativeAD(  );
	}
	
    private void requestSingleData( float valuesChA[] , boolean isCollectData ){
		//Log.i( "cpz" , "requestDataILineVibrationModule "+valuesChA[0] );
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


//40000    20000     10000	 5000   4000     2500   2000   1000   500
//102.4    51.2      25.6    12.8   10.24    6.4    5.12   2.56   1.28
    public  SgPropertyTime getTime(  ){
    	SgPropertyTime mTime = new SgPropertyTime(  );
    	//mTime.setDistanceRange( 5 );
	    mTime.setMaxFreq( 40000f );//////
	    mTime.setMinFreq( 10f );/////		
	    //mTime.setRangeMode( 6 ); 
	    mTime.setPropType( 4 );///////  4时域   5频域   6总值
	    //mTime.setAccelerationRange( 5 );
	    mTime.setSignalType( 0 );       //0加速度 1速度  2位移
	    //mTime.setSpeendRange( 5 );
	    //mTime.setSyncMode( 5 );
	    //mTime.setTrigLevel( 5.0f );
	    mTime.setWaveLength( 1024 );////// 
		
	    return mTime;
    }
    
    public SgPropertyFreq getFreq(  ){
    	
    	SgPropertyFreq mTime = new SgPropertyFreq(  );
    	mTime.setDistanceRange( 5 );
	    mTime.setMaxFreq( 40000f );/////////
	    mTime.setMinFreq( 5f );//////
	    //mTime.setRangeMode( 6 ); 
	    mTime.setPropType( 5 );/////// 4时域   5频域   6总值
	    //mTime.setAccelerationRange( 5 );
	    mTime.setSignalType( 1 );////         //0加速度  1速度  2位移
	    //mTime.setSpeendRange( 5 );
	    //mTime.setSyncMode( 5 );
	    //mTime.setTrigLevel( 5.0f );
	    mTime.setSpectraNum( 6400 );///
	    mTime.setAveTimes( 1 );///
	    mTime.setAveMode( 0 );///
	    mTime.setWindowType( 1 );///   0不加窗 1汉宁窗
	    return mTime;    	
    }
    
    public SgPropertyTotalTrend getTotal(  ){
    	SgPropertyTotalTrend mTime = new SgPropertyTotalTrend(  );
    	//mTime.setDistanceRange( 5 );
	    mTime.setMaxFreq( 5000f );/////////////
	    mTime.setMinFreq( 1f );///////////
	    //mTime.setRangeMode( 6 ); 
	    mTime.setPropType( 4 );///////4时域   5频域   6总值
	    //mTime.setAccelerationRange( 5 );
	    mTime.setSignalType( 0 );         //0加速度 1速度  2位移
	    //mTime.setSpeendRange( 5 );
	    //mTime.setSyncMode( 5 );
	    //mTime.setTrigLevel( 5.0f );
	    mTime.setWaveLength( 4096 );
	    mTime.setTvtMode( 1 );//////// 总值类型
	    mTime.setIntervalTimes( 0.0f );
	    return mTime;   	
    }
	    public interface VibrationDataCallBack{
	        public void notifyVibrationData( float data[] , boolean isCollectData );
	        public void notifyVibrationDoubleChData( float data[] , float data2[] , boolean isCollectData );
	        public void notifyStopAD( boolean isStop );
	     }

	public interface EvaluteDataCallBack{
		public void notifyEvaluteData( float data[] , boolean isCollectData );
		public void notifyStopAD( boolean isStop );
	}
	 private native void startNativeTimeWave( int ch_num ,  SgPropertyTime value );
	 private native void startNativeFreqWave( int ch_num ,  SgPropertyFreq value );
	 private native void startNativeValueWave( int ch_num ,  SgPropertyTotalTrend value );
	 private native void startNativeRotation(  );
	 private native void  startNativeEvalute( SgPropertyBase fgpaValue );
	 private native float[] getNativeFeatureData( float data[] , int length );
	 private native float[] getNativeTimeToFreqData( float data[] , int length );
	 private native boolean startNativeinit(  );

	 private native boolean stopNativeAD(  );
}

