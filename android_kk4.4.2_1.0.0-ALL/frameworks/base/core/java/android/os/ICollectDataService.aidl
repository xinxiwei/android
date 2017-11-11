package android.os;
import android.os.ICollectDataCallBack;  
/**
 * @hide
 */
interface ICollectDataService {
    void startCollectData();
    void stopCollectData();
    void setCallBack(in ICollectDataCallBack listener);
}
