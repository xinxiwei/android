package android.widget;


import java.text.SimpleDateFormat;
import java.util.Date;
import android.graphics.Color;
import android.graphics.drawable.AnimationDrawable;
import android.content.Context;
import android.os.Handler;
import android.os.Message;
import android.util.AttributeSet;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.FrameLayout;
import android.widget.ImageView;
import android.widget.TextView;
import android.content.BroadcastReceiver;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.BatteryManager;
public class SystemTitleView extends FrameLayout{
	private Context mContext = null;
	private TextView mTextLeft = null;
	private TextView mTextCenter = null;
	private TextView mTextRight = null;
	private TextView mTextTime = null;
	private ImageView mImageBattery = null;
	private View mRoot ;
	private boolean mAttached = false;
	private  Date mCurrentTime = new Date();
	private MyReceiver mReceiver = null;
	public static final int SHOW_SINGLE_CONTENT = 0;
	public static final int SHOW_GROUP_CONTENT = 1;
	public static final int TITLE_STYLE_0 = 0;
	public static final int TITLE_STYLE_1 = 1;
	public static final int TITLE_STYLE_2 = 2;
	private  class MyHandler extends Handler{	   
	       @Override
	       public void handleMessage(Message msg) {
	          if(msg.what == 100){
	        	  updateTime();
	        	  mHandler.sendEmptyMessageDelayed(100,1000);
	          }

	       }
	   };

	private  MyHandler mHandler = new MyHandler();
	public SystemTitleView(Context context) {
		super(context);
		// TODO Auto-generated constructor stub
	}
	  public SystemTitleView(Context context, AttributeSet attrs) {
	        this(context, attrs, 0);
	    }

	    public SystemTitleView(Context context, AttributeSet attrs, int defStyle) {
	        super(context, attrs, defStyle);
	        mContext = context;
	        init();
	       
	    }
	    public void init(){
	    	Log.i("cpz","init");
	    	 mReceiver = new MyReceiver();
	    	 LayoutInflater inflate = (LayoutInflater) mContext.getSystemService(Context.LAYOUT_INFLATER_SERVICE);
	         mRoot = inflate.inflate(com.android.internal.R.layout.system_title, null);
	         mTextLeft = (TextView)mRoot.findViewById(com.android.internal.R.id.title_left);
	         mTextCenter = (TextView)mRoot.findViewById(com.android.internal.R.id.title_center);
	         mTextRight = (TextView)mRoot.findViewById(com.android.internal.R.id.title_right);
	         mTextTime = (TextView)mRoot.findViewById(com.android.internal.R.id.system_title_time);
	         mImageBattery = (ImageView)mRoot.findViewById(com.android.internal.R.id.system_title_battery);
	         FrameLayout.LayoutParams frameParams = new FrameLayout.LayoutParams(
	                 ViewGroup.LayoutParams.MATCH_PARENT,
	                 37
	         );
	         this.addView(mRoot, frameParams);
	    }
	    
	    public TextView getLeftText(){
	    	return mTextLeft;
	    }
	    public TextView getRightText(){
	    	return mTextRight;
	    }
	    public TextView getCenterText(){
	    	return mTextCenter;
	    }
		
		 public TextView getTimeText(){
	    	return mTextTime;
	    }
		/**2017-8-26 modify custom system title used to app's system setting begin*/
	    public void setDisplayMode(int mode){
			this.setBackgroundResource(com.android.internal.R.drawable.system_title_group_bg);
			switch(mode){
				case TITLE_STYLE_0:
					mTextCenter.setVisibility(View.GONE);
					mTextRight.setVisibility(View.GONE);
					mTextLeft.setTextColor(0xff000000);
					mTextLeft.setTextSize(26);
					mTextLeft.setBackgroundDrawable(null);
					mTextLeft.setVisibility(View.VISIBLE);
					mTextTime.setVisibility(View.VISIBLE);
				break;
				case TITLE_STYLE_1:
					mTextCenter.setVisibility(View.VISIBLE);
					mTextCenter.setTextSize(26);
					mTextCenter.setTextColor(0xffffffff);
					mTextCenter.setBackgroundResource(com.android.internal.R.drawable.system_title_text2_bg);
					
					mTextRight.setVisibility(View.VISIBLE);
					mTextRight.setTextSize(21);
					mTextRight.setTextColor(0xff878787);
					mTextRight.setBackgroundResource(com.android.internal.R.drawable.system_title_text3_bg);
					mTextTime.setVisibility(View.GONE);
				break;
				case TITLE_STYLE_2:
					mTextCenter.setVisibility(View.VISIBLE);
					mTextCenter.setTextSize(21);
					mTextCenter.setTextColor(0xff878787);
					mTextCenter.setBackgroundResource(com.android.internal.R.drawable.system_title_text3_bg);
					
					mTextRight.setVisibility(View.VISIBLE);
					mTextRight.setTextSize(26);
					mTextRight.setTextColor(0xffffffff);
					mTextRight.setBackgroundResource(com.android.internal.R.drawable.system_title_text2_bg);
					mTextTime.setVisibility(View.GONE);
				break;
			}
						
		}
				
	    public void setSystemTitleText(String text1,String text2,int mode){
			setDisplayMode(mode);
	    	mTextLeft.setVisibility(View.INVISIBLE);
			mTextLeft.setBackgroundResource(com.android.internal.R.drawable.system_title_text1_bg);
    		mTextCenter.setText(text1);
    		mTextRight.setText(text2);
	    }
		/**2017-8-26 modify custom system title used to app's system setting end*/
		
	    
	    public void setSystemTitleText(String text1){
	    	setDisplayMode(0);
	    	mTextLeft.setText(text1);
	    }
	    

	    private void updateTime(){
	    	 SimpleDateFormat format=new SimpleDateFormat("yyyy-MM-dd HH:mm");  
	    	 mCurrentTime.setTime(System.currentTimeMillis());
	    	 final String text = format.format(mCurrentTime);
	    	 mTextTime.setText(text);    
	    	
	    }
	    @Override
	    protected void onAttachedToWindow() {
	    	Log.i("cpz","onAttachEd");
	        super.onAttachedToWindow();

	        if (!mAttached) {
	            mAttached = true;
	            mHandler.sendEmptyMessage(100);
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
	        	mContext.unregisterReceiver(mReceiver);
	        	mHandler.removeMessages(100);
	            mAttached = false;
	        }
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
	
	      	            switch (status) {
	      	                case BatteryManager.BATTERY_STATUS_CHARGING:
	      	                	notifyBattery(level,status);
	      	                	break;
	      	                case BatteryManager.BATTERY_STATUS_FULL:
	      	                case BatteryManager.BATTERY_STATUS_DISCHARGING:
	      	                case BatteryManager.BATTERY_STATUS_NOT_CHARGING: 
	      	                	mImageBattery.setImageResource(com.android.internal.R.drawable.stat_sys_battery);    
	      	                	mImageBattery.getDrawable().setLevel(level);
	      	                    break;
	      	            }

	      	        }
	        }
	    }
	    
	    public void notifyBattery(int level,int status){

	        int batteryAnimation;//电量动态增长  
	        if(level<= 10){  
	            batteryAnimation = com.android.internal.R.drawable.stat_sys_battery_charge_anim10;  

	        }else if(level <= 20){  
	            batteryAnimation = com.android.internal.R.drawable.stat_sys_battery_charge_anim20;  

	        }else if(level <= 30){  
	            batteryAnimation =com.android.internal.R.drawable.stat_sys_battery_charge_anim30;  

	        }else if(level <= 40){  
	            batteryAnimation = com.android.internal.R.drawable.stat_sys_battery_charge_anim40;  

	        }else if(level <= 50){  
	            batteryAnimation = com.android.internal.R.drawable.stat_sys_battery_charge_anim50;  

	        }else if(level <= 60){  
	            batteryAnimation = com.android.internal.R.drawable.stat_sys_battery_charge_anim60;  

	        }else if(level <= 70){  
	            batteryAnimation = com.android.internal.R.drawable.stat_sys_battery_charge_anim70;  

	        }else if(level <= 80){  
	            batteryAnimation = com.android.internal.R.drawable.stat_sys_battery_charge_anim80;  

	        }else if(level <= 90){
	        	 batteryAnimation = com.android.internal.R.drawable.stat_sys_battery_charge_anim90;  
	        }else {
	        	 batteryAnimation = com.android.internal.R.drawable.stat_sys_battery_charge_anim90;  
	        }
	        
	        if(status ==BatteryManager.BATTERY_STATUS_CHARGING){  
	        	mImageBattery.setImageResource(batteryAnimation);//將電量背景修改為animation即可  
	            AnimationDrawable frameAnimation = (AnimationDrawable)mImageBattery.getDrawable();
	            if(!frameAnimation.isRunning()){  
	                frameAnimation.stop();  
	                frameAnimation.start();  
	            }  

	        }
	    }  
}
