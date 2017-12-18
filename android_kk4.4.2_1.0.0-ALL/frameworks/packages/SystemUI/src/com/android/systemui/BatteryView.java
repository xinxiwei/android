package com.android.systemui;

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





import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.BatteryManager;
import android.util.AttributeSet;
import android.util.Log;
import android.widget.ImageView;
import android.graphics.drawable.AnimationDrawable;


/**
 * Digital clock for the status bar.
 */
public class BatteryView extends ImageView {
    private boolean mAttached;


    private Context mContext ;
    private MyReceiver mReceiver = null;
    public BatteryView(Context context) {
        this(context, null);
    }

    public BatteryView(Context context, AttributeSet attrs) {
        this(context, attrs, 0);
    }

    public BatteryView(Context context, AttributeSet attrs, int defStyle) {
        super(context, attrs, defStyle);
        mContext = context;
        mReceiver = new MyReceiver();
    }

    @Override
    protected void onAttachedToWindow() {
        super.onAttachedToWindow();

        if (!mAttached) {
            mAttached = true;
            IntentFilter filter = new IntentFilter();
            filter.addAction(Intent.ACTION_BATTERY_CHANGED);
            mContext.registerReceiver(mReceiver, filter);
        }

        // NOTE: It's safe to do these after registering the receiver since the receiver always runs
        // in the main thread, therefore the receiver can't run before this method returns.

        // The time zone may have changed while the receiver wasn't registered, so update the Time

        // Make sure we update to the current time
    
    }

    @Override
    protected void onDetachedFromWindow() {
        super.onDetachedFromWindow();
        if (mAttached) {
        	Log.i("cpz","onDetachedFromWindow");
        	 getContext().unregisterReceiver(mReceiver);
            mAttached = false;
        }
    }
  private void updatePackageName(String text){
	  
	 
  }
  
  public class MyReceiver extends BroadcastReceiver
  {
      //  当sendbroadcast发送广播时，系统会调用onReceive方法来接收广播
      @Override
      public void onReceive(Context context, Intent intent){
    	     final String action = intent.getAction();
    	        if (action.equals(Intent.ACTION_BATTERY_CHANGED)) {
    	        	
    	            final int level = intent.getIntExtra(BatteryManager.EXTRA_LEVEL, 0);
    	            final int status = intent.getIntExtra(BatteryManager.EXTRA_STATUS,
    	                    BatteryManager.BATTERY_STATUS_UNKNOWN);
    	            Log.i("cpz","level"+level);
    	            Log.i("cpz","level"+status);
    	            switch (status) {
    	        
    	                case BatteryManager.BATTERY_STATUS_FULL:
    	                	BatteryView.this.setImageResource(R.drawable.stat_sys_battery_9);
    	                	break;
    	                case BatteryManager.BATTERY_STATUS_CHARGING:
    	                	//BatteryView.this.setImageResource();
    	                	notifyBattery(level,status);
    	                	break;
    	                case BatteryManager.BATTERY_STATUS_DISCHARGING:
    	                case BatteryManager.BATTERY_STATUS_NOT_CHARGING: 
    	                	BatteryView.this.setImageResource(R.drawable.stat_sys_battery);    
    	                	BatteryView.this.getDrawable().setLevel(level);
    	                    break;
    	            }

    	        }
      }
  }
  
  public void notifyBattery(int level,int status){

      int batteryAnimation;//电量动态增长  
      if(level<= 10){  
          batteryAnimation = R.drawable.stat_sys_battery_charge_anim10;  

      }else if(level <= 20){  
          batteryAnimation = R.drawable.stat_sys_battery_charge_anim20;  

      }else if(level <= 30){  
          batteryAnimation = R.drawable.stat_sys_battery_charge_anim30;  

      }else if(level <= 40){  
          batteryAnimation = R.drawable.stat_sys_battery_charge_anim40;  

      }else if(level <= 50){  
          batteryAnimation = R.drawable.stat_sys_battery_charge_anim50;  

      }else if(level <= 60){  
          batteryAnimation = R.drawable.stat_sys_battery_charge_anim60;  

      }else if(level <= 70){  
          batteryAnimation = R.drawable.stat_sys_battery_charge_anim70;  

      }else if(level <= 80){  
          batteryAnimation = R.drawable.stat_sys_battery_charge_anim80;  

      }else if(level <= 90){
      	 batteryAnimation = R.drawable.stat_sys_battery_charge_anim90;  
      }else {
      	 batteryAnimation = R.drawable.stat_sys_battery_charge_anim90;  
      }
      
      if(status ==BatteryManager.BATTERY_STATUS_CHARGING){  
          this.setImageResource(batteryAnimation);//將電量背景修改為animation即可  
          AnimationDrawable frameAnimation = (AnimationDrawable)this.getDrawable();
          if(!frameAnimation.isRunning()){  
              frameAnimation.stop();  
              frameAnimation.start();  
          }  

      }
  }  
}

