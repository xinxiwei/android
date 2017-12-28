package android.iline;

/**
 * Created by QueKai on 2017/4/28.
 * 采集定义实体类
 */

public class SgPropertyBase {
    private int propID;//采集定义ID
    private String propName;//采集定义名
    private int propType;//采集定义类型
    private int chanNum;//采集通道数

    private int routeId;//点检计划ID

    private float minFreq;//下限频率
    private float maxFreq;//上限频率
    private int coupleType;//耦合方式
    private int rangeMode;//量程模式
    private int signalType;//信号类型
    private int sensorType;//传感器类型
    private int rangeAcceleration;//
    private int rangeDistanceValue;//
    private int rangeSpeendValue;//
    private int syncMode;//同步方式
    private float trigLevel;//触发电平
    private float daqFreq;//实际采样频率
    private int waveLength;//波形长度
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
