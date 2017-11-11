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
 
public class ILineBackLight {
	
	public static final int KEY_LIGHT =9;
	public ILineBackLight(){
		
	}
		
	public void setBackLight(int light,int status){
		startNativeBackLightDevice();
		setNativeBackLight(light,status);
		stopNativeBackLightDevice();
	}
	
    private native void startNativeBackLightDevice();
    private native void stopNativeBackLightDevice();
    private native void setNativeBackLight(int light,int status);
}

