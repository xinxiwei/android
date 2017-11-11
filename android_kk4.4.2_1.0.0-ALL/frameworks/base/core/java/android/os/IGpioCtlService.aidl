package android.os;
import android.os.IGpioCallback;
interface IGpioCtlService {
    void setVal(int val);
    int getVal();
    void setGpioCallBack(IGpioCallback cb);
}
