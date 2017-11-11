package android.iline;

/**
 * Created by QueKai on 2017/4/28.
 * 閲囬泦瀹氫箟瀹炰綋-棰戣氨绫�
 */
public class SgPropertyFreq extends SgPropertyBase {
    private int spectraNum;//璋辩嚎鏁�
    private int aveTimes;//骞冲潎娆℃暟
    private int aveMode;//骞冲潎鏂瑰紡
    private int windowType;//鍔犵獥绫诲瀷

    public int getSpectraNum() {
        return spectraNum;
    }

    public void setSpectraNum(int spectraNum) {
        this.spectraNum = spectraNum;
    }

    public int getAveTimes() {
        return aveTimes;
    }

    public void setAveTimes(int aveTimes) {
        this.aveTimes = aveTimes;
    }

    public int getAveMode() {
        return aveMode;
    }

    public void setAveMode(int aveMode) {
        this.aveMode = aveMode;
    }

    public int getWindowType() {
        return windowType;
    }

    public void setWindowType(int windowType) {
        this.windowType = windowType;
    }
}
