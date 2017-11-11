
package android.collect;

import android.util.Log;
public class CollectPressData{

    private CollectPressDataInterface mDataInterface;
    private float m = 1;
    public CollectPressData(){


    }
    public void setCollectPressDataImpl(CollectPressDataInterface dataInterface){
    	
    	Log.i("cpz","setDataSource");
      mDataInterface = dataInterface;
   }
    private void requestNativeData(float data){
    	m = data;
    	if(mDataInterface != null){
           mDataInterface.notifyCollectData(10);
    	}else{
    		Log.i("cpz","nihao"+m);
    	}
   }
    
    public void part(){
    	Log.i("cpz","m"+m);
     }
   public void startNativeCollectData(){
      startCollectData();
   }
   public void stopNativeCollectData(){
      stopCollectData();
   }
    public interface CollectPressDataInterface {

      public void notifyCollectData(float data);  
    }
   
    private  native void startCollectData();
    private  native void stopCollectData();
}
