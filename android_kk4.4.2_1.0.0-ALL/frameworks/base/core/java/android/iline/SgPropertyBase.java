package android.iline;

/**
 * Created by QueKai on 2017/4/28.
 * 閲囬泦瀹氫箟瀹炰綋绫�
 */

public class SgPropertyBase {
    private int propID;//閲囬泦瀹氫箟ID
    private String propName;//閲囬泦瀹氫箟鍚�
    private int propType;//閲囬泦瀹氫箟绫诲瀷
    private int chanNum;//閲囬泦閫氶亾鏁�

    private int routeId;//鐐规璁″垝ID

    private float minFreq;//涓嬮檺棰戠巼
    private float maxFreq;//涓婇檺棰戠巼
    private int coupleType;//鑰﹀悎鏂瑰紡
    private int rangeMode;//閲忕▼妯″紡
    private int signalType;//淇″彿绫诲瀷
    private int sensorType;//浼犳劅鍣ㄧ被鍨�
    private int rangeAcceleration;//閲忕▼鍔犻�熷害
    private int rangeDistanceValue;//閲忕▼浣嶇疆
    private int rangeSpeendValue;//閲忕▼閫熷害
    private int syncMode;//鍚屾鏂瑰紡
    private float trigLevel;//瑙﹀彂鐢靛钩
    private float daqFreq;//瀹為檯閲囨牱棰戠巼
    private int waveLength;//娉㈠舰闀垮害
	private int versionMode;

	public int getVersionMode(){
		return versionMode;
	}
	
	public void setVersionMode(int versionMode){
		this.versionMode=versionMode;
	}
    public int getPropID() {
        return propID;
    }

    public void setPropID(int propID) {
        this.propID = propID;
    }

    public String getPropName() {
        return propName;
    }

    public void setPropName(String propName) {
        this.propName = propName;
    }

    public int getPropType() {
        return propType;
    }

    public void setPropType(int propType) {
        this.propType = propType;
    }

    public int getChanNum() {
        return chanNum;
    }

    public void setChanNum(int chanNum) {
        this.chanNum = chanNum;
    }
    
    public int getRouteId() {
        return routeId;
    }

    public void setRouteId(int routeId) {
        this.routeId = routeId;
    }

    public float getMinFreq() {
        return minFreq;
    }

    public void setMinFreq(float minFreq) {
        this.minFreq = minFreq;
    }

    public float getMaxFreq() {
        return maxFreq;
    }

    public void setMaxFreq(float maxFreq) {
        this.maxFreq = maxFreq;
    }

    public int getCoupleType() {
        return coupleType;
    }

    public void setCoupleType(int coupleType) {
        this.coupleType = coupleType;
    }

    public int getRangeMode() {
        return rangeMode;
    }

    public void setRangeMode(int rangeMode) {
        this.rangeMode = rangeMode;
    }

    public int getSignalType() {
        return signalType;
    }

    public void setSignalType(int signalType) {
        this.signalType = signalType;
    }

    public int getSensorType() {
        return sensorType;
    }

    public void setSensorType(int sensorType) {
        this.sensorType = sensorType;
    }

    public float getTrigLevel() {
        return trigLevel;
    }

    public void setTrigLevel(float trigLevel) {
        this.trigLevel = trigLevel;
    }

    public int getDistanceRange() {
        return rangeDistanceValue;
    }

    public void setDistanceRange(int range) {
        this.rangeDistanceValue = range;
    }

    public int getAccelerationRange() {
        return rangeAcceleration;
    }

    public void setAccelerationRange(int range) {
        this.rangeAcceleration = range;
    }
    
    public int getSpeendRange() {
        return rangeSpeendValue;
    }

    public void setSpeendRange(int range) {
        this.rangeSpeendValue = range;
    }
    
    
    public int getSyncMode() {
        return syncMode;
    }

    public void setSyncMode(int syncMode) {
        this.syncMode = syncMode;
    }

    public float getDaqFreq() {
        return daqFreq;
    }

    public void setDaqFreq(float daqFreq) {
        this.daqFreq = daqFreq;
    }

    public int getWaveLength() {
        return waveLength;
    }

    public void setWaveLength(int waveLength) {
        this.waveLength = waveLength;
    }
}
