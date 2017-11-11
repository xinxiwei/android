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

public class ILineLight {
	public static final int YELLO =3;
	public static final int BLUE =4;
	public static final int RED =5;
	public static final int KEY_LIGHT =9;
	public ILineLight(){
		
	}
	
	public void openLightDevice(){
		startNativeLightDevice();
	}
	
	public void closeLightDevice(){
		stopNativeLightDevice();
	}
	
	public void setLightOpen(int light){
		setNativeLightOpen(light);
	}
	public void setLightClose(int light){
		setNativeLightClose(light);
	}
    private native void startNativeLightDevice();
    private native void stopNativeLightDevice();
    private native void setNativeLightOpen(int light);
    private native void setNativeLightClose(int light);
    
      
}

