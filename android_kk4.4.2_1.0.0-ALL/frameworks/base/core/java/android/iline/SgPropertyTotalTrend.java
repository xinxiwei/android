package android.iline;

/**
 * Created by QueKai on 2017/4/28.
 * 閲囬泦瀹氫箟瀹炰綋-鎬诲�艰秼鍔跨被
 */
public class SgPropertyTotalTrend extends SgPropertyBase {
    private float intervalTimes;//闂撮殧鏃堕棿
    private int tvtMode;//鎬诲�肩被鍨�

    public float getIntervalTimes() {
        return intervalTimes;
    }

    public void setIntervalTimes(float intervalTimes) {
        this.intervalTimes = intervalTimes;
    }

    public int getTvtMode() {
        return tvtMode;
    }

    public void setTvtMode(int tvtMode) {
        this.tvtMode = tvtMode;
    }
}
