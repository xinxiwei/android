package com.android.systemui;

import android.app.Service;
import android.content.Intent;
import android.database.ContentObserver;
import android.iline.ILineBeep;
import android.iline.ILineBackLight;
import android.os.Handler;
import android.os.IBinder;
import android.os.Message;
import android.provider.Settings;
import android.util.Log;
import android.widget.Toast;

public class IHipService extends Service{
	private SettingsSoundChangeContentObserver mSoundOb = null;
	private SettingsBackLightChangeContentObserver mBackLightOb = null;
	private ILineBeep  mBeep = null;
    private ILineBackLight mBackLight = null;

	@Override
	public IBinder onBind(Intent arg0) {
		return null;
	}
	
	public void onCreate(){
		super.onCreate();
		Log.i("cpz","IhipServiceStart");
		mBeep = new ILineBeep();
		mBeep.openDevice();
		
		mBackLight = new ILineBackLight();
		
		if(Settings.System.getInt(getContentResolver(), Settings.System.SCREEN_KEY_SOUND, 0) == 1){
			changeBeepState(true);
		}
        if(Settings.System.getInt(getContentResolver(), Settings.System.SCREEN_KEY_SMARTLIGHT, 0) == 1){
        	changeLightState(true);
		}
        if(Settings.System.getInt(getContentResolver(), Settings.System.SCREEN_KEY_SMARTLIGHT, 0) == 2){
        	//changeLightSensor(true);
		}
         mSoundOb = new SettingsSoundChangeContentObserver();
		 getContentResolver().registerContentObserver(Settings.System.getUriFor(Settings.System.SCREEN_KEY_SOUND),true, mSoundOb);
		 
		 mBackLightOb = new SettingsBackLightChangeContentObserver();
		 getContentResolver().registerContentObserver(Settings.System.getUriFor(Settings.System.SCREEN_KEY_SMARTLIGHT),true, mBackLightOb);
	}
		
	
	public void onStart(Intent intent,int flag){
		super.onStart(intent, flag);	
	}
	
	private void changeBeepState(boolean isOpen){//打开beep声音
		if(mBeep == null)
			return;
		if(isOpen){
			Log.i("cpz","UI打开beep声音");
			mBeep.openKey(mBeep.BEEP_TYPE);
		}else{
			Log.i("cpz","UI关闭beep声音");
			mBeep.closeKey(mBeep.BEEP_TYPE);
		}
	}
	
	/*begin add by lianghao */
	private void changeLightSensor(boolean isOpen){//智能感光
		if(mBeep == null)
			return;
		
		if(isOpen){
			Log.i("cpz","UI打开智能感光");
			mBeep.openKey(mBeep.SENSOR_TYPE);
		}else{
			Log.i("cpz","UI关闭智能感光");
			mBeep.closeKey(mBeep.SENSOR_TYPE);
		}
	}
	/*end */
	
	private void changeLightState(boolean isOpen){ //一直打开键盘背光
		if(mBackLight == null)
			return;

		if(isOpen){		   
		   Log.i("cpz","UI一直打开键盘背光");
		   mBackLight.setBackLight(mBackLight.KEY_LIGHT,1);		   
		 }else{
		   mBackLight.setBackLight(mBackLight.KEY_LIGHT,0);
		   Log.i("cpz","UI一直关闭键盘背光");
		} 
	}
	public void onDestroy(){
		super.onDestroy();
		Log.i("cpz","IhipOnDestory");
		getContentResolver().unregisterContentObserver(mSoundOb);
		getContentResolver().unregisterContentObserver(mBackLightOb);
	}

	class SettingsSoundChangeContentObserver extends ContentObserver {
		 
        public SettingsSoundChangeContentObserver() {
            super( new Handler());
        }
 
        @Override
        public void onChange(boolean selfChange) {
            super.onChange(selfChange);
           if(Settings.System.getInt(getContentResolver(), Settings.System.SCREEN_KEY_SOUND, 0) == 0){
        	   changeBeepState(false);
           }else{
        	   changeBeepState(true);
           }           
        }  
    }
	
	
	class SettingsBackLightChangeContentObserver extends ContentObserver {
		 
        public SettingsBackLightChangeContentObserver() {
            super( new Handler());
        }
 
        @Override
        public void onChange(boolean selfChange) {
            super.onChange(selfChange);
          
            if(Settings.System.getInt(getContentResolver(), Settings.System.SCREEN_KEY_SMARTLIGHT, 0) == 0){ //一直关闭
            	changeLightState(false);//mBackLight.setBackLight(mBackLight.KEY_LIGHT,x);		 
				//changeLightSensor(false);//mBeep.openKey(Beep.SENSOR_TYPE);
            }else if(Settings.System.getInt(getContentResolver(), Settings.System.SCREEN_KEY_SMARTLIGHT, 0) == 1){ //一直打开
				//changeLightSensor(false);
            	changeLightState(true);
            }else if(Settings.System.getInt(getContentResolver(), Settings.System.SCREEN_KEY_SMARTLIGHT, 0) == 2){ //智能感光
				changeLightState(false);
            	//changeLightSensor(true);
            }
        }  
    }
}
