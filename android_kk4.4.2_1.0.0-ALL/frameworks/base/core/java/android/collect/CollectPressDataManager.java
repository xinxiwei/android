
package android.collect;




import android.collect.CollectPressData;
public class CollectPressDataManager implements CollectPressData.CollectPressDataInterface{

   
   
    private PressDataCallBack mCallBack;
    private boolean mIsStarted = false;


    private CollectPressData mCollectPressData = null;
    public CollectPressDataManager(){
       mCollectPressData  = new CollectPressData();
       mCollectPressData.setCollectPressDataImpl(this);
    }
   
    public void setPressDataCallBack(PressDataCallBack   callBack ){
      mCallBack =  callBack;
    }
    

    public void startAD(){
      if(mCollectPressData != null){
         mCollectPressData.startNativeCollectData();
      }
    }
    
    public void part4(){
        if(mCollectPressData != null){
           mCollectPressData.part();
        }
      }
    public void stopAD(){
     if(mCollectPressData != null){
         mCollectPressData.stopNativeCollectData();
      }
     mCallBack = null;
   }

  
   public  void notifyCollectData(float data){
         if(mCallBack != null){
            mCallBack.notifyData(data);
     }
  }
  public interface PressDataCallBack{
    public void notifyData(float data);
 }
  

}
