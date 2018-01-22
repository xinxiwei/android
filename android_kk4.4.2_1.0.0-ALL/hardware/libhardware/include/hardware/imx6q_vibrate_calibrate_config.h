
#ifndef ANDROID_IMX6Q_SPI_PRESSURE
#define ANDROID_IMX6Q_SPI_PRESSURE
//DMA
#define  SPIDEV_IOC_RXSTREAMON         (0x06)
#define  SPIDEV_IOC_RXSTREAMOFF        (0x07)
#define  SPIDEV_IOC_QUEUE_STATE        (0X08)
//FPGA
#define StartSampleAddr  0x03000000
#define StartSampleData  0

#define StopSampleAddr   0x04000000
#define StopSampleData   0

#define ResetFpgaRegAddr   0x05000000
#define ResetFpgaRegData   0
//POWER
#define FPGA_3V3_CTR    6
#define FPGA_2V5_CTR    7
#define FPGA_1V2_CTR    8
#define FPGA_RESET_CTR  10
//GPIO
#define GPIO_SET_ON   0X01
#define GPIO_SET_OFF  0X00

#define ADCModeAddr   0

typedef struct time_wave_para
{
     int data_type;//数据类型
     int signal_type; //信号类型
     float min_freq; //下限频率
     float max_freq; //上限频率
     float calib_expecte_value; //校准期望值
     float range_gain_value1;//一级增益
     float range_gain_value2;//二级增益
     float trig_value;//触发电平值

     int wave_length; //波形长度
     int range_mode;//量程方式
     int range_accel_value;//加速度量程
     int range_speed_value; //速度量程
     int range_disp_value; //位移量程
     int trig_mode; //触发方式
     int version_mode; //内部版本模式
	 int voltage_range; //电压量程
     int chan_num;//采集通道

}timewave;

typedef struct freq_wave_para
{
     int data_type;
     int signal_type; //信号类型
     float min_freq;  //
     float max_freq; //上限频率
     float calib_expecte_value; //校准期望值
     float range_gain_value1;//一级增益
     float range_gain_value2;//二级增益
     float trig_value;//触发电平值

     int spectra_num;//频谱线数
     int average_num ;//平均次数
     int average_mode;//平均方式
     int window_type; //加窗类型
     int range_mode;//量程方式
     int range_accel_value;//加速度量程
     int range_speed_value; //速度量程
     int range_disp_value; //位移量程
     int trig_mode; //触发方式
     int version_mode; //内部版本模式
	 int voltage_range; //电压量程
     int chan_num;//采集通道
}freqwave;

typedef struct total_rend_para{
     int data_type;
     int signal_type; //信号类型
     float min_freq;
     float max_freq; //上限频率
     float calib_expecte_value; //校准期望值
     float interval_time; //间隔时间
     float range_gain_value1;//一级增益
     float range_gain_value2;//二级增益

     int wave_length;
     int total_value_type; //总值类型
     int range_mode;       //量程方式
     int range_accel_value;//加速度量程
     int range_speed_value; //速度量程
     int range_disp_value; //位移量程
     int version_mode; //内部版本模式
	 int voltage_range; //电压量程
     int chan_num;//采集通道
}totalrend;

#ifdef __cplusplus
extern "C"
{
#endif

	extern void poweron_spi();
	extern void poweroff_spi();
    extern void set_voltage_range(int ch, int voltageRange);
    extern int enable_fram_write(bool flag);
	extern int  set_singleCH_vibrate_reg(int ch, int signal_type, float maxFreq, float minFreq, int versionMode, int rangeMode);
	extern char *log_time();

#ifdef __cplusplus
};
#endif

#endif
