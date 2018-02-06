

#ifndef ANDROID_IMX6Q_COMMON_CONFIG
#define ANDROID_IMX6Q_COMMON_CONFIG
/////////////////////////振动
#define SIZE_15360     15360
#define SIZE_60K       61440

#define VIB_MAX_SIZE  3284288 //单通道最大（128*1024 + 690000）*4 = 3284288, 双通道最大（128*1024 + 690000）*2*4 = 6568576    //31457280

#define FEATURE_NUM     15  //特征值个数

/* 延时时间定义 */
#define DELAY_50000      50000
#define DELAY_100000     100000
#define DELAY_300000     300000
#define DELAY_500000     500000
#define DELAY_1000000    1000000
#define DELAY_3000000    3000000
#define DELAY_5000000    5000000
    
/* 上下限定义 */
#define FREQ_40000      40000
#define FREQ_20000      20000
#define FREQ_10000      10000
#define FREQ_5000       5000
#define FREQ_4000       4000
#define FREQ_2500       2500
#define FREQ_2000       2000
#define FREQ_1000       1000
#define FREQ_500        500

#define FREQ_100      100
#define FREQ_50       50
#define FREQ_20       20
#define FREQ_10       10
#define FREQ_5        5
#define FREQ_2        2
#define FREQ_1        1
#define FREQ_0.16     0.16
#define FREQ_7        7
    
/* 波形长度 */
#define LENGTH_256K     262144
#define LENGTH_128K     131072
#define LENGTH_64K      65536
#define LENGTH_32K      32768
#define LENGTH_16K      16384
#define LENGTH_8K       8192
#define LENGTH_4K       4096
#define LENGTH_2K       2048
#define LENGTH_1K       1024
    
/* 信号类型     */
#define ACC_TYPE    0
#define VEL_TYPE    1
#define DSP_TYPE    2
/* 电压量程 */
#define VOL_RANGE_V25      0
#define VOL_RANGE_V2P5     1
#define VOL_RANGE_V0P25    2
    
#define EPSINON     0.00000000001  
    
#define MULTIPLE_NUM 1000000

/* FPGA FRAM寄存器 */
#define REG_OFFSET_ADDR1  0x04
#define REG_OFFSET_ADDR2  0x08
    
#define REG_FRAM_ADDR1   0xf000      
#define REG_FRAM_ADDR2   (REG_FRAM_ADDR1 + REG_OFFSET_ADDR2)      
#define REG_FRAM_ADDR3   (REG_FRAM_ADDR2 + REG_OFFSET_ADDR2)    
#define REG_FRAM_ADDR4   (REG_FRAM_ADDR3 + REG_OFFSET_ADDR2)      
#define REG_FRAM_ADDR5   (REG_FRAM_ADDR4 + REG_OFFSET_ADDR2)      
#define REG_FRAM_ADDR6   (REG_FRAM_ADDR5 + REG_OFFSET_ADDR2)      
#define REG_FRAM_ADDR7   (REG_FRAM_ADDR6 + REG_OFFSET_ADDR2)      
#define REG_FRAM_ADDR8   (REG_FRAM_ADDR7 + REG_OFFSET_ADDR2)      
#define REG_FRAM_ADDR9   (REG_FRAM_ADDR8 + REG_OFFSET_ADDR2)      
#define REG_FRAM_ADDR10  (REG_FRAM_ADDR9 + REG_OFFSET_ADDR2)      
#define REG_FRAM_ADDR11  (REG_FRAM_ADDR10 + REG_OFFSET_ADDR2)   
    
/* 版本模式 */
#define TEST_MODE   1
#define NORMAL_MODE 0

/* 特征值类型 */
#define  SG_TIMESIGNAL_HANDLE_RMS   0 //有效值
#define  SG_TIMESIGNAL_HANDLE_PK    1 //峰值
#define  SG_TIMESIGNAL_HANDLE_PP    2 //峰峰值
#define  SG_TIMESIGNAL_HANDLE_DC    3 //平均值
#define  SG_TIMESIGNAL_HANDLE_MAX   4 //最大值

//压力
#define  SIZE_PRESS   32768  //压力采样长度
#define  SMP_RATE     25600   //修改压力采集客户反馈时长不够问题，采样率由102400 改为25600
#define  PRESS_MAX_SIZE    131072 //单通道最大（32K*1024）*4 = 131072

//校准
#define   CALIB_MAX_SIZE  65536 //单通道最大（16*1024）*4 = 65536
	
#endif
