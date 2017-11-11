
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
//freq
#define SPI_SAMPLE  102400   //Hz

#define ADCModeAddr   0
extern int test_mode; //声明内部测试模式

typedef struct time_wave_para
{
     int data_type;//数据类型
     int signal_type; //信号类型 
     float min_freq; //下限频率
     float max_freq; //上限频率	 
     int wave_length; //波形长度
     int range_mode;//量程方式	
     int range_accel_value;//加速度量程    
     int range_speed_value; //速度量程
	 int range_disp_value; //位移量程
     float range_gain_value1;//一级增益
     float range_gain_value2;//二级增益    
     int trig_mode; //触发方式
     float trig_value;//触发电平值
     int version_mode; //内部版本模式
}timewave;

typedef struct freq_wave_para 
{
     int data_type;
     int signal_type; //信号类型 
     float min_freq;  //
     float max_freq; //上限频率
	 
     int spectra_num;//频谱线数
     int average_num ;//平均次数
     int average_mode;//平均方式
     int window_type; //加窗类型
     int range_mode;//量程方式
     int range_accel_value;//加速度量程    
     int range_speed_value; //速度量程
	 int range_disp_value; //位移量程
     float range_gain_value1;//一级增益
     float range_gain_value2;//二级增益    
     int trig_mode; //触发方式
     float trig_value;//触发电平值
     int version_mode; //内部版本模式
}freqwave;

typedef struct total_rend_para{
     int data_type;
     int signal_type; //信号类型 
     float min_freq;
     float max_freq; //上限频率
	 	 
     int wave_length;
     float interval_time; //间隔时间 
     int total_value_type; //总值类型
     int range_mode;       //量程方式
     int range_accel_value;//加速度量程    
     int range_speed_value; //速度量程
	 int range_disp_value; //位移量程
     float range_gain_value1;//一级增益
     float range_gain_value2;//二级增益 
     int version_mode; //内部版本模式
}totalrend;

#ifdef __cplusplus
extern "C"
{
#endif	  		
    /* extern timewave my_timewave;
	extern freqwave my_freqwave;
	extern totalrend my_totalrend; */
    
	extern int  spi_freq();
	extern void poweron_spi();
	extern void poweroff_spi();		
	extern int  set_press_reg(int smp_rate);
	extern int  set_singleCH_vibrate_reg(int signal_type,float maxFreq, float minFreq);
	extern int  set_doubleCH_vibrate_reg(int signal_type,float maxFreq, float minFreq);
	extern int  set_rotation_reg();		
	extern char *log_time();	 

#ifdef __cplusplus
};
#endif
	
#endif