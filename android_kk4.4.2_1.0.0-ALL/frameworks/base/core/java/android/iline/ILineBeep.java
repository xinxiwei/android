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
public class ILineBeep {
	public static final int BEEP_TYPE = 1;
	public static final int SENSOR_TYPE = 2;
	public static final int BACK_LIGHT = 3;
	public ILineBeep(){

	}

	public void setOpen(int key, int status){
		startOpenKey();
		if(key == 1)
		{
			setBeepKey(key,status);
		}else if(key == 2)
		{
			setSensorKey(key,status);
		}
	}

	public void setClose(int key,int status){
		if(key == 1)
		{
			setBeepKey(key,status);
		}else if(key == 2)
		{
			setSensorKey(key,status);
		}
		//stopNativeKey();
	}

    private native void startOpenKey();
    private native void setBeepKey(int key,int status);
    private native void setSensorKey(int key,int status);
    private native void stopNativeKey();
}
