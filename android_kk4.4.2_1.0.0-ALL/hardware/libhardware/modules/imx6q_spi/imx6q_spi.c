#define LOG_TAG "spiStub"
#include <hardware/hardware.h>
#include <hardware/imx6q_spi.h>
#include <hardware/gpio.h>
#include <hardware/log.h>
#include <hardware/masterspi.h>
#include <hardware/slavespi.h>
#include <hardware/spidev.h>
#include <hardware/errorcode.h>
#include <hardware/beep.h>
#include <fcntl.h>
#include <stdbool.h>
#include <math.h>
#include <errno.h>
#include <cutils/log.h>
#include <cutils/atomic.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <sys/timeb.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h> 

#include <hardware/imx6q_spi_config.h>
#include <hardware/imx6q_common_func.h>

#include <hardware/imx6q_spi_pressure.h>
#include <hardware/imx6q_spi_vibrate.h>
#include <hardware/imx6q_iir_filter.h>
#include <hardware/imx6q_fir_filter.h>

#define DEVICE_NAME "/dev/mxc_spidev1" //从设备
#define MODULE_NAME "xxxx"
#define MODULE_AUTHOR "gxl@126.com"

/////////////////////////////////////////////////
#define debug      0
#define SIZE_15360  15360
#define SIZE_60K   61440
#define SIZE_64K   65536
#define SIZE_4K    4096
#define SIZE_16K   16384

#define FEATURE_NUM    15  //特征值个数

int smp_rate = 102400;//采样率HZ
int oflags = 0;
int fd = -1; //从SPI设备文件
int *psample_buf  = NULL; //最终采样的原始数据
int *g_smp_buf = NULL;  //全局采集的数据
float invalid_buf[2] = {0.0}; // power off时回调的假buf,供接口使用，实际无意义
float vib_reg_fail_buf[1] = {10001}; // 振动采集时当寄存器配置失败时，供上面app用
float dma_fail_buf[1] = {10002}; // 振动采集时dma打开异常不成对开关，供上面app用
float press_reg_fail_buf[3] = {40000,40000,40000}; // 压力采集时当寄存器配置失败时，供上面app用
float status_flag[2] ={10000.0,1};	//压力异常回调时的状态数组 ， 1表示数据有效个数为1
float feature_ret_value[FEATURE_NUM] = {0.0}; //15个特征值

unsigned char read_60K_buf[SIZE_60K] = {0};// 读压力数据buf
unsigned char press_buf[SIZE_64K] = {0};//采集到的压力数据

volatile bool thread_finished_flag = 0; //表示当前线程完成标识
volatile bool exit_thread_flag = false; //退出线程标志
volatile int  stop_smp_flag = 0;//停止采集数据 标识
volatile int  restart_power_on_flag = 0;  // spi 重新上电标识
volatile int  power_off_flag = 0; // 点击stop时的下电标识
volatile int  can_start_flag  = 0; //用于快速点击时，dma没有执行完
volatile int  start_enable_flag = 0;// 响应start接口标识
volatile int  press_discard_16K_flag =0; //压力采集丢弃前16K数据，解决波形前面不稳bug
volatile int  press_flag0_flag = 0 ;  //用于压力标0 丢弃前10波数据
volatile float pflag0_value = 0.0; //压力标0 模式值
volatile int  start_num = 0; // 用于判定stop 完成后，才可以响应start
int  test_mode = 0 ; //初始化内部测试模式,会传给寄存器配置


//int old_fd = 0;  //前一次fd
pthread_t  c_id; // 开辟线程 c_id:计算
sem_t   run_sem; //内部信号名  run_sem: 继续运行信号

#define   MAX_SIZE  3284288 //单通道最大（128*1024 + 690000）*4 = 3284288  ,双通道最大（128*1024 + 690000）*2*4 = 6568576    //31457280
unsigned char g_max_char_buf[ MAX_SIZE ] ={0}; 
volatile int g_smpLength = 0; //实际采集的波形长度
int g_chNum = 0;     //振动通道个数
 

//设备打开和关闭接口
static int spi_device_open( const struct hw_module_t* module , const char* name , struct hw_device_t** device );
static int spi_device_close( struct hw_device_t* device );

//设备访问接口
static int start_pressure_dial( struct spictl_device_t* dev , float data ); //表盘模式
static int start_pressure_curve( struct spictl_device_t* dev , float data ); //曲线模式
static int start_pressure_flag0( struct spictl_device_t* dev ); //标0模式

static int start_vibrate_CH_timewave( struct spictl_device_t* dev ,   int ch_num , struct time_wave_para tWave );//启动振动采集 时域波形
static int start_vibrate_CH_freqwave( struct spictl_device_t* dev ,   int ch_num , struct freq_wave_para fWave );//启动振动采集 频域波形
static int start_vibrate_CH_totalrend( struct spictl_device_t* dev ,  int ch_num , struct total_rend_para tRend );//启动振动采集 总值趋势波形
static int start_rotation_CH( struct spictl_device_t* dev );//转速通道
static int start_vibrate_evalute_level( struct spictl_device_t* dev , struct time_wave_para tWave );//启动振动等级评估
static int start_vibrate_calibration( struct spictl_device_t* dev,    struct time_wave_para tWave );//启动振动校准

static int stop_vibrate_sample( struct spictl_device_t* dev ); //停止振动采集
static int stop_press_sample( struct spictl_device_t* dev ); //停止压力采集
static int spi_get_freq( struct spictl_device_t* dev ); //获取采样频率

static float* spi_get_feature_value( struct spictl_device_t* dev , float pData[] , int data_len ); //获取15个特征值接口
static float* spi_time_to_freq_value( struct spictl_device_t* dev , float pData[] , int data_len );  //时域到频域接口

static int spi_set_val( struct spictl_device_t* dev , int val );
static int spi_get_val( struct spictl_device_t* dev , int* val );

struct spictl_device_t* dev;

//模块方法表
static struct hw_module_methods_t spi_module_methods={
   open: spi_device_open
};


//JNI回调接口
static SpiPressureCallbacks    pressure_callback_backup;   //压力回调接口
static SpiVibrateCallbacks     vibrate_callback_backup;  //振动回调接口

//模块实例变量
struct spictl_module_t HAL_MODULE_INFO_SYM ={
    common:{
        tag: HARDWARE_MODULE_TAG , 
        version_major: 1 , 
        version_minor: 0 , 
        id: SPICTL_HARDWARE_MODULE_ID , 
        name: MODULE_NAME , 
        author: MODULE_AUTHOR , 
        methods: &spi_module_methods , 
    }
};

static void initPress( SpiPressureCallbacks* callbacks ){
	pressure_callback_backup = *callbacks;
}

static const SpiPressureInterface  mSpiPressInterface = {
	initPress , 
};

const SpiPressureInterface* spi_press_interface( struct spictl_device_t* dev ){ //压力回调接口
    return &mSpiPressInterface;
}

static void initVibrate( SpiVibrateCallbacks* callbacks ){
	vibrate_callback_backup = *callbacks;
}

static const SpiVibrateInterface  mSpiVibrateInterface = {
	initVibrate , 
};

const SpiVibrateInterface* spi_vibrate_interface( struct spictl_device_t* dev ){//振动回调接口
    return &mSpiVibrateInterface;
}

static int spi_device_open( const struct hw_module_t* module , const char* name , struct hw_device_t** device ){
	//LOGD("\n\nxin: imx6q_spi_default.so-版本信息  = 20170916.0930"); //.so 版本信息
    struct spictl_device_t* dev;
	dev =( struct spictl_device_t* )malloc( sizeof( struct spictl_device_t ) );
	if(dev == NULL)
	{
		LOGD( "imx6q_spi: failed to alloc space" );
		return -EFAULT;
	}
	memset( dev , 0 , sizeof( struct spictl_device_t ) );
    dev->common.tag = HARDWARE_DEVICE_TAG;
    dev->common.version = 0;
    dev->common.module =( hw_module_t* )module;
    dev->common.close = spi_device_close;	
	dev->set_val = spi_set_val;
    dev->get_val = spi_get_val;	
	
    dev->start_press_dial = start_pressure_dial; //表盘模式
	dev->start_press_curve = start_pressure_curve; //曲线模式
	dev->start_press_flag0 = start_pressure_flag0; //标0模式
	
	dev->start_vibrate_timewave = start_vibrate_CH_timewave;  //时域波形
	dev->start_vibrate_freqwave = start_vibrate_CH_freqwave;  //频域波形
	dev->start_vibrate_totalrend = start_vibrate_CH_totalrend;	//总值趋势
	dev->start_vibrate_evalute = start_vibrate_evalute_level; //等级评估
    dev->start_vibrate_calib = start_vibrate_calibration; //振动校准
    
    
	dev->start_rotation = start_rotation_CH; //转速
	
	dev->stop_vibrate_ad = stop_vibrate_sample; //振动停止采集
	dev->stop_press_ad = stop_press_sample; //压力停止采集
	
	dev->spi_freq = spi_get_freq;	//压力采样频率
	dev->spi_feature_value = spi_get_feature_value;	 //15个特征值
	dev->spi_timetofreq_value = spi_time_to_freq_value;	//时域到频域接口
	
    dev->get_pressure_interface = spi_press_interface;   //压力回调接口
	dev->get_vibrate_interface = spi_vibrate_interface; //振动回调接口
	
    *device =&( dev->common );   
    return 0;
}

static int spi_device_close( struct hw_device_t* device ){
    struct spictl_device_t* devspi_device =( struct spictl_device_t* )device;
    if( devspi_device ){
        close( devspi_device->fd );
        free( devspi_device );
    }
    return 0;
}


static int spi_set_val( struct spictl_device_t* dev , int val ){
    LOGD( "imx6q_spi: set value %d to device." , val );
    //write( dev->fd , &val , sizeof( val ) ); 
    return 0;
}


static int spi_get_val( struct spictl_device_t* dev , int* val ){
    if( !val ){
        LOGD( "imx6q_spi: error val pointer" );
        return -EFAULT;
    }
    //read( dev->fd , val , sizeof( *val ) );
    LOGD( "imx6q_spi: get value %d from device" , *val );
    return 0;
}

static int spi_get_freq( struct spictl_device_t* dev ){//获取压力采样率    
	return spi_freq( );
}


///////////////////共用函数如下
inline float adc_data_to_24value( int adc_data )// 将振动采集32位数据 转换为24位有效数据
{
    bool highbit_flag = false;
    float value = 0.0;
    adc_data &= 0x00ffffff;//24位数据
    highbit_flag =( bool )(( adc_data>>23 )&0x1 ); //最高位符号位
    adc_data &= 0x007fffff;
    if( highbit_flag )		
    {	       
        value =(float)(( adc_data-0x800000 )*2.5 )/0x7fffff ;//分段函数        
    }
    else
    {	    
        value =(float)( adc_data*2.5 )/0x7fffff;
    }		
    if(test_mode)//表示内部测试模式, 没有24V激励，电压量程也只是2.5V, 不需要*10
    {
        ;
    }else{
        value = value *10; //乘以10 是因为寄存器配置是25V，硬件电路会衰减10倍，再此*10补上 
    }
	return value;
}

inline float press_adc_data_to_24value( int adc_data )// 将压力采集32位数据 转换为24位有效数据
{
    bool highbit_flag = false;
    float value = 0.0;
    adc_data &= 0x00ffffff;//24位数据
    highbit_flag =( bool )(( adc_data>>23 )&0x1 ); //最高位符号位
    adc_data &= 0x007fffff;
    if( highbit_flag )		
    {	       
        value =(float)(( adc_data-0x800000 )*2.5 )/0x7fffff ;//分段函数        
    }
    else
    {	    
        value =(float)( adc_data*2.5 )/0x7fffff;
    }		
    LOGD("xin:press_adc_data_to_24value——value = %f",value);
    value = value*10;  //乘以10 是因为压力采集寄存器配置是25V，硬件电路会衰减10倍，再此*10补上
	return value;
}

void analyze_CH_data( int adc_data ,  float *value )//解析出各通道的数据
{		
    if(value == NULL)
    {
       exit( EXIT_FAILURE ); 
    }
	int higetst_bit =( adc_data >> 31 )&0x1;
	int channel_bit =( adc_data >> 30 )&0x1;
	
	if( higetst_bit == 0x1 ) //1表示转速数据
	{
		value[0] = adc_data&0x7fffff;			
	}
	else //振动
	{
		if( channel_bit == 0 ) //0表示 通道1 数据
		{
			value[1] = adc_data_to_24value( adc_data );              			
		}
		else if( channel_bit == 1 ) // 1表示 通道2 数据
		{
			value[2] = adc_data_to_24value( adc_data );           					
		}    		
	}		
}

void common_start( )//打开设备和开始fpga采集
{
	int res = -1;    
    if(( fd = open( DEVICE_NAME , O_RDWR ) ) == -1 ){
        LOGD( "xin: 打开从spi设备 /dev/mxc_spidev1 失败 -- %s." , strerror( errno ) );        		
    }else{		
		LOGD( "xin: 打开从spi设备 /dev/mxc_spidev1 成功. fd = %d" , fd );
	}	
    /* int fd_num = 0;
    old_fd = fd; */

    fcntl( fd ,  F_SETOWN ,  getpid( ) );//将当前进程PID设置为fd文件所对应驱动程序将要发送SIGIO,SIGUSR信号进程PID      
    oflags = fcntl( fd ,  F_GETFL );//获取fd的打开方式       
    fcntl( fd ,  F_SETFL ,  oflags | FASYNC );//将fd的打开方式设置为FASYNC --- 即 支持异步通知
    
    res = ioctl( fd ,  SPIDEV_IOC_RXSTREAMON ,  NULL );//启动DMA采集		
	if( res !=0 ) //启动DMA采集失败时，重新启动
	{		
        LOGD( "imx6q_spi: DMA采集-启动失败===== ,fd = %d",fd );
        /* for(fd_num = old_fd-30; fd_num <= fd; fd_num++)
        {
            int res2 = ioctl( fd_num ,  SPIDEV_IOC_RXSTREAMOFF ,  NULL );  //停止DMA搬运 
            if( res2 !=0 ) //启动DMA采集失败时，重新启动
            {		
                LOGD( "stop_fpga_dma: DMA采集-停止失败 ,fd_num = %d",fd_num ); 
            }else{
                LOGD( "stop_fpga_dma: DMA采集-停止成功 ,fd_num = %d",fd_num ); 
                //break;
            }  
        }
        swrite( StopSampleAddr , StopSampleData );
        
        start_num = 0;
        vibrate_callback_backup.single_ch_callback( dma_fail_buf ,1 ,true);//DMA关闭异常 dma_fail_buf vib_reg_fail_buf
        start_enable_flag = 0;
        spi_power_off(  );
        power_off_flag = 0; */
    }else{
        LOGD( "imx6q_spi: DMA采集-启动成功 ,fd = %d",fd );    
    }

	swrite( StartSampleAddr , StartSampleData );// 启动FPGA采集板	
} 

void stop_fpga_dma()//停止FPGA采集, DMA搬运，关闭设备文件
{	
	LOGD( "xin: DMA结束开始 power_off_flag = %d, restart_power_on_flag = %d,is : [%s]\n" ,power_off_flag, restart_power_on_flag,  log_time( ) ); 
  
	stop_smp_flag = 0;	//重要
	press_flag0_flag = 0; // 此时将压力标0要丢弃10波数据清0
   
	int res = ioctl( fd ,  SPIDEV_IOC_RXSTREAMOFF ,  NULL );  //停止DMA搬运 
	if( res !=0 ) //启动DMA采集失败时，重新启动
	{		
        LOGD( "stop_fpga_dma: DMA采集-停止失败===== ,fd = %d",fd ); 
    }else{
        LOGD( "stop_fpga_dma: DMA采集-停止成功 ,fd = %d",fd ); 
    }
	swrite( StopSampleAddr , StopSampleData );//stop fpga		
	LOGD("xin: 停止FPGA采集");	
	
	if( power_off_flag == 1) // 下电标识
	{	
        spi_power_off(); 
		restart_power_on_flag = 0;	// 下次采集重新上电标识 0表示后面采集要重新上电
	}else{
		restart_power_on_flag = 1;	 
	} 
	
	if( close( fd ) == 0)//关闭从设备文件	
	{		
		fd = -1; //关闭成功重新初始fd
		LOGD("xin: 关闭从SPI设备文件成功");
	}    
	LOGD( "xin: DMA结束完成 power_off_flag = %d, restart_power_on_flag = %d,is : [%s]\n" ,power_off_flag, restart_power_on_flag,  log_time( ) );
    start_num--; //此时 数据减1
}

static int stop_press_sample( struct spictl_device_t* dev )//停止压力采集
{	
	LOGD( "xin: 停止压力采集==开始 power_off_flag = %d, restart_power_on_flag = %d,start_enable_flag =%d is : [%s]\n" , power_off_flag, restart_power_on_flag, 
    start_enable_flag, log_time( ) );
	
	if(start_enable_flag == 0)//当可以响应start 线程flag =0时，表示此时处于无任何start线程状态,提前回调
	{		
		spi_power_off( );  //下电
		pressure_callback_backup.stop_press_callback( false );		
		
        start_enable_flag = 0;	
		power_off_flag = 0;
		restart_power_on_flag = 0;
		can_start_flag = 0;
		start_num = 0; //无start线程时，清0
		LOGD("xin: 压力采集此时start线程刚结束，无线程存在,回调false给上层");
        return 0;		
	}	

    start_enable_flag = 0;
    power_off_flag = 1;		
    restart_power_on_flag = 0; //stop时设为0 ，当再响应start时重新上电
    LOGD( "xin: 停止压力采集==结束 power_off_flag = %d, restart_power_on_flag = %d,start_enable_flag =%d is : [%s]\n" , power_off_flag, restart_power_on_flag, 
    start_enable_flag, log_time( ) );
	
	return 0;	
}

static int stop_vibrate_sample( struct spictl_device_t* dev )//停止振动采集，会将power_off_flag 置为1，若为1，stop_fpga_dma时会下电
{ 
    LOGD( "xin: 停止振动采集开始 power_off_flag = %d, restart_power_on_flag = %d,start_enable_flag =%d is : [%s]\n" , power_off_flag, restart_power_on_flag, 
    start_enable_flag, log_time( ) );
	if(start_enable_flag == 0)//当可以响应start 线程flag =0时，表示此时处于无任何start线程状态,提前回调
	{		
		spi_power_off( );  //下电
		vibrate_callback_backup.stop_ch_callback( false); 		
		
        start_enable_flag = 0;	
		power_off_flag = 0;
		restart_power_on_flag = 0;
		can_start_flag = 0;
		start_num = 0;//无start线程时，清0
		LOGD("xin: 振动采集此时start线程刚结束，无线程存在,回调false给上层");
        return 0;		
	}
	
    start_enable_flag = 0;
    power_off_flag = 1;		
    restart_power_on_flag = 0; //stop时设为0 ，当再响应start时重新上电
    LOGD( "xin: 停止振动采集结束 power_off_flag = %d, restart_power_on_flag = %d,start_enable_flag =%d is : [%s]\n" , power_off_flag, restart_power_on_flag, 
    start_enable_flag, log_time( ) );
	return 0;
}


///////////////////压力采集功能如下
void read_press_data( int signo ) // 读取压力adc 采样数据
{	
    int p_loop_num =0;
    //LOGD( "xin: 读取压力数据时stop_smp_flag = %d,p_loop_num = %d, g_smpLength = %d , power_off_flag =%d  , [%s]\n" ,  stop_smp_flag, p_loop_num, g_smpLength , power_off_flag , log_time( ) );
    if(power_off_flag == 1) 
	{	
		LOGD("xin: 读取压力数据时，检测到下电标识，不再读取数据提前stop fpaga DMA,post信号量");
		stop_smp_flag = 1; // 为1时 ，停止继续读数据
		stop_fpga_dma();	
		sem_post( &run_sem ); 
		return ;
	}
	
	if( stop_smp_flag )
	{		 
		return;
	}
	
	if(  read( fd , read_60K_buf , SIZE_60K ) < 0  )
	{
        LOGD( "Error: spi slave device read fail !\n " );
    }
	
	if( p_loop_num == 0 )
    {
        memcpy( &press_buf ,  &read_60K_buf ,  SIZE_60K );	
		p_loop_num ++;	
    }else if( p_loop_num == 1 )
    {
		press_discard_16K_flag ++;
		p_loop_num = 0;
        
		if(press_flag0_flag) //压力标0，丢前6波数据
		{
		   if( press_discard_16K_flag >= 6) 
		   {			
                stop_smp_flag = 1; // flag置1 ，停止继续读数据
                stop_fpga_dma(); 
                
                memcpy( &press_buf[SIZE_60K] ,  &read_60K_buf ,  SIZE_4K ); //SIZE_4K= 4096
                psample_buf =( int* )&( press_buf ) ;
                //LOGD( "xin: post压力信号量 : [%s]\n" ,  log_time( ) ); 
                        
                sem_post( &run_sem );                
                p_loop_num = 0;
                press_discard_16K_flag =0;
		   }
		}else //其它正常采集，只丢前3波数据
        { 	 
			if( press_discard_16K_flag >= 3)
			{			
				stop_smp_flag = 1; // flag置1 ，停止继续读数据
				stop_fpga_dma(); //停止DMA采集
				
				memcpy( &press_buf[SIZE_60K] ,  &read_60K_buf ,  SIZE_4K ); //SIZE_4K= 4096
				psample_buf =( int* )&( press_buf ) ;
				//LOGD( "xin: post压力信号量 : [%s]\n" ,  log_time( ) ); 
				sem_post( &run_sem );				
				p_loop_num = 0;
				press_discard_16K_flag =0;
			}
		}
    }	
}

void *press_flag0_thread( void* arg ) //压力标0模式
{
	//LOGD( "xin: 进入压力标0线程 [%s]\n" ,  log_time( ) );
    int i=0;      	
    float press_flag0_value[2] ={0.0}; //标0模式时，回传的数据 ,  0位：表示数据值，1位：表示有效数据个数
    float f64_sum = 0.0; //64K数据总和
	
	sem_wait( &run_sem );//等待信号量	
	f64_sum = 0.0;	
    
	for( i=0;i<SIZE_16K;i++ )
	{		        
		f64_sum += press_adc_data_to_24value( psample_buf[i] ) ;	
	}	
	press_flag0_value[0] = f64_sum / SIZE_16K;	//总和求平均值
	press_flag0_value[1] = 1; 
	
	LOGD( "xin: 计算出压力标0值 =%f" , press_flag0_value[0] );			
	
    int res = ioctl( fd ,  SPIDEV_IOC_RXSTREAMOFF ,  NULL ); //停止DMA搬运
    if( res !=0 ) //启动DMA采集失败时，重新启动
	{		
        LOGD( "压力标0: DMA采集-停止失败 ,fd = %d",fd ); 
    }else{
        LOGD( "压力标0: DMA采集-停止成功 ,fd = %d",fd ); 
    }
    swrite( StopSampleAddr , StopSampleData );//停止FPGA采集
	
	close( fd ); //关闭设备文件 		
	spi_power_off( );  //下电	  
	sem_destroy( &run_sem );  //销毁信号量
	pressure_callback_backup.mspictl_callback( press_flag0_value , true );//调用JNI 回调方法，向上层传送数据   
	start_num =0;
    //LOGD( "xin: 退出压力标0线程 [%s]\n" ,  log_time( ) );
    return NULL;
}

void *press_curve_thread( void* arg ) //压力曲线模式
{    
    //LOGD( "xin: 进入压力曲线线程 [%s]\n" ,  log_time( ) );
	int i=0;
    float final_buf[SIZE_16K] ={0.0};   //SIZE_16K
	float positive_buf[SIZE_16K] ={0.0}; //压力数据全部转换为正数	
	float ret_value[SIZE_16K] ={0.0};  //回调返回的数据
    float stop_callback_buf[3] = {20000,20000,20000}; //用于压力曲线模式下电时，回调的数据，借用数据通道，后续优化
	
	float value_bar_max2 = 0.0;   
    float value_bar_min2 = 0.0;	
	float tmp1=0.0 , tmp2=0.0 , tmp3=0.0;		
	
	sem_wait( &run_sem );//等待信号量		
	
	if(power_off_flag == 1)
	{
		LOGD("xin: recived post ");
		goto  stop_daq;
	}
	memset( final_buf , 0 , SIZE_16K*sizeof( float ) );
	memset( positive_buf , 0 , SIZE_16K*sizeof( float ) );
	memset( ret_value , 0 , SIZE_16K*sizeof( float ) );		
	
	for( i=0;i<SIZE_16K;i++ )
	{
		final_buf[i] = press_adc_data_to_24value( psample_buf[i] );	           		
		positive_buf[i] =( float )fabs( final_buf[i] );    //采集数据 取绝对值全部转为正数    			
	}
					
	value_bar_max2 = positive_buf[0]; //假设最大最小值都是数组0值 
	value_bar_min2 = positive_buf[0];			
	for( i=1;i<SIZE_16K;i++ )
	{
		if( value_bar_max2 <= positive_buf[i] )            
			value_bar_max2= positive_buf[i];  //对原始采集数据求最大值 ，用于后面0.5bar 判定条件，若最大值小于0.5bar , 则不进行后续算法计算，直接提示异常
		
		if( value_bar_min2 >= positive_buf[i] )            
			value_bar_min2= positive_buf[i];  //对原始采集数据求最小值		    
	}
	//LOGD( "press_curve_thread value_bar_max2 =%f ,  value_bar_min2 =%f\n" ,  value_bar_max2 , value_bar_min2 );			
	tmp1 = fabs(  fabs( pflag0_value ) - value_bar_max2 ); //求标0值和最大最小值差值的绝对值
	tmp2 = fabs(  fabs( pflag0_value ) - value_bar_min2 );		
	tmp3 =( tmp1 > tmp2 )?tmp1:tmp2;	   	//求幅值变化最大值
	
	/*
	if( tmp3  <= 0.01 )  //0.5bar = 0.01V  // 20mv/bar  表示外部环境和气缸环境 相差很小，此时表示空采
	{
		LOGD( "press_curve_thread is <0.5bar status,回调10000 ，1 数据" );   
        can_start_flag = 1;
		spi_power_off();  //下电
		sem_destroy( &run_sem );
		thread_finished_flag = 1;
		usleep(10000); 
			
		pressure_callback_backup.mspictl_callback( status_flag ,  false );
        //LOGD("xin: 退出压力曲线线程  [%s]\n" ,  log_time( ) );			
		return NULL;		
	}
	else if( tmp3 >0.01 )	   
	{ 	*/
		LOGD( "press_curve_thread is >0.5bar status" );				
		press_alg_entry( final_buf , SIZE_16K ,  ret_value );//压力曲线模式算法	
		
		if(power_off_flag == 1)//检测到下电标识为1
		{
			LOGD("xin: 压力曲线线程检测到下电标识为1，下电，销毁信号量，回调false给上层 [%s]\n" ,  log_time( ) );
			can_start_flag = 1;
			spi_power_off(); 
			sem_destroy( &run_sem );
			
			thread_finished_flag = 1;
			usleep(10000);  
			
			pressure_callback_backup.mspictl_callback( stop_callback_buf , false );  //借用数据通道，后续优化  
            LOGD("xin: 退出压力曲线线程  [%s]\n" ,  log_time( ) );			
		    return NULL;
		}else{
			pressure_callback_backup.mspictl_callback( ret_value ,  true );//调用JNI 回调方法，向上层传送数据
			LOGD("xin: 压力曲线数据正常回调数据完成 [%s]\n" ,  log_time( ) );
		}	
		can_start_flag = 1;			
		sem_destroy( &run_sem );
		
	stop_daq:
		if(power_off_flag == 1) 
		{
			LOGD("xin: 压力曲线数据线程检测到下电标识为1");
			thread_finished_flag = 1;
			usleep(10000);                                          
			
			pressure_callback_backup.mspictl_callback( stop_callback_buf , false );
			return NULL;
		}
		thread_finished_flag = 1;		
	//}
    LOGD("xin: 退出压力曲线线程  [%s]\n" ,  log_time( ) );	
    return NULL;
}
 
void *press_dial_thread( void* arg ) //压力表盘模式
{
	//LOGD( "xin: 进入压力表盘线程 [%s]\n" ,  log_time( ) );
    int i=0;	
    float dial_value[2]={0.0}; //表盘模式时，回传的数据 ,  0位：表示数据值，1位：表示有效数据个数
	
	float final_buf[SIZE_16K] ={0.0};   //SIZE_16K
	float positive_buf[SIZE_16K] ={0.0}; 
	float stop_callback_buf[3] = {30000,30000,30000}; //用于压力曲线模式下电时，回调的数据，借用数据通道，后续优化
	
    float f64_sum = 0.0;
	float value_bar_max1 = 0.0;   
    float value_bar_min1 = 0.0;	
	float tmp1=0.0 , tmp2=0.0 , tmp3=0.0;	
			
	sem_wait( &run_sem );//等待信号量
		
    if(power_off_flag == 1)//检测到下电标识为1
	{
		//LOGD("xin: recived post ");
		goto  stop_daq;
	}	
	memset( final_buf , 0 , SIZE_16K*sizeof( float ) );
	memset( positive_buf , 0 , SIZE_16K*sizeof( float ) );		
	f64_sum = 0.0;		
	
	for( i=0;i<SIZE_16K;i++ )
	{			
		final_buf[i] = press_adc_data_to_24value( psample_buf[i] ); 
		f64_sum += final_buf[i];
   		positive_buf[i] =( float )fabs( final_buf[i] );            		
	}	
	
	value_bar_max1 = positive_buf[0]; 
	value_bar_min1 = positive_buf[0];		
	for( i=1;i<SIZE_16K;i++ )
	{
		if( value_bar_max1 <= positive_buf[i] )            
			value_bar_max1= positive_buf[i];  //对原始采集数据求最大值 ，用于后面0.5bar 判定条件，若最大值小于0.5bar , 则不进行后续算法计算，直接提示异常
		
		if( value_bar_min1 >= positive_buf[i] )            
			value_bar_min1= positive_buf[i];  //对原始采集数据求最小值	    
	}		
	
	tmp1 = fabs(  fabs( pflag0_value ) - value_bar_max1 );
	tmp2 = fabs(  fabs( pflag0_value ) - value_bar_min1 );
	tmp3 =( tmp1 > tmp2 )?tmp1:tmp2;  
	
    /*
	if( tmp3  <= 0.01 )  //0.5bar = 0.5*20mv = 10mv = 0.01V  //外部环境和气缸环境 相差很小，此时表示空采
	{
		LOGD( "press_dial_thread is <0.5bar status" );
        can_start_flag = 1;
		spi_power_off(); 
		sem_destroy( &run_sem );
		
		thread_finished_flag = 1;
		usleep(10000);  
		
		pressure_callback_backup.mspictl_callback( status_flag ,  false );
        //LOGD("xin: 退出压力表盘线程  [%s]\n" ,  log_time( ) );			
		return NULL;		
	}
	else if( tmp3 > 0.01 )	   
	{	*/
		LOGD( "press_dial_thread is >0.5bar status" );		
		dial_value[0] = f64_sum / SIZE_16K;   //表盘模式算法是对所采数据求平均
		dial_value[1] = 1;        			
		LOGD( "xin: 压力表盘计算值dial_value =%f\n" ,  dial_value[0] );
		
		if(power_off_flag == 1) 
		{			
			LOGD("xin: 压力表盘线程检测到下电标识为1，下电，销毁信号量，回调false给上层 [%s]\n" ,  log_time( ) );
			can_start_flag = 1;
			spi_power_off(); 
			sem_destroy( &run_sem );
			thread_finished_flag = 1;
			usleep(10000);  
			
			pressure_callback_backup.mspictl_callback( stop_callback_buf , false );
            LOGD("xin: 退出压力表盘线程  [%s]\n" ,  log_time( ) );			
		    return NULL;
		}else{
			pressure_callback_backup.mspictl_callback( dial_value ,  true );//调用JNI 回调方法，向上层传送数据	
		    LOGD("xin: 压力表盘数据正常回调数据完成 [%s]\n" ,  log_time( ) );
		}
		can_start_flag = 1;			
		sem_destroy( &run_sem );
		
	stop_daq:
		if(power_off_flag == 1) 
		{
			LOGD("xin: 压力表盘数据线程检测到下电标识为1");  
			thread_finished_flag = 1;
			usleep(10000);                                          
			
			pressure_callback_backup.mspictl_callback( stop_callback_buf , false );
			return NULL;
		}
		thread_finished_flag = 1;
	//}
    LOGD("xin: 退出压力表盘线程  [%s]\n" ,  log_time( ) );	
    return NULL;
}
 
static int start_pressure_flag0( struct spictl_device_t* dev )// 压力标0模式
{	 
   LOGD("xin: 点击压力标0开始时start_num = %d",start_num);
   if(start_num != 0)
   {
	  return 0; 
   }   
    memset(read_60K_buf,0,SIZE_60K*sizeof(unsigned char));
    memset(press_buf ,0, SIZE_64K*sizeof(unsigned char)); 
	press_flag0_flag = 1;  //只针对标0时，此时置为1
        
    poweron_spi(  );			
    int reg_ret_value = set_press_reg( smp_rate );	//设置压力采集寄存器 
	if( reg_ret_value == -1)
	{	
        LOGD("xin: 寄存器配置失败");
		pressure_callback_backup.mspictl_callback( press_reg_fail_buf ,  false );//表示寄存器配置失败，回调 false给上层
		start_enable_flag = 0;
		spi_power_off(  );			
		return 0;
	}
    
	usleep(2000000); //用于上层时域波形不丢波形数据，在这里直接延时2S,让硬 件预热稳定
	start_num ++;
	
	sem_init( &run_sem ,  0 ,  0 );	
	
	pthread_create( &c_id ,  NULL ,  press_flag0_thread ,  NULL );
	signal( SIGIO ,  read_press_data );// 捕捉异步IO信号，并安装信号处理函数	
    common_start( );	
	//LOGD("xin: 响应压力标0点击结束******************** [%s]\n" ,  log_time( )); 
	return 0;
}
 
static int start_pressure_curve( struct spictl_device_t* dev , float flag0_value )  //压力曲线模式
{	 
   LOGD("xin: 点击压力曲线开始时start_num = %d",start_num);
   if(start_num != 0)
   {
	  return 0; 
   } 
    memset(read_60K_buf,0,SIZE_60K*sizeof(unsigned char)); 
    memset(press_buf ,0, SIZE_64K*sizeof(unsigned char)); 	
	stop_smp_flag = 0;
    pflag0_value = flag0_value;	

	//LOGD("xin: 点击压力曲线开始时******************** [%s]\n" ,  log_time( ));
	LOGD("\nxin: 点击压力曲线开始时start_enable_flag = %d, restart_power_on_flag= %d, can_start_flag = %d , thread_finished_flag = %d",start_enable_flag,restart_power_on_flag,can_start_flag,thread_finished_flag);
		
	if(start_enable_flag == 1) // 1表示前一个start 线程还没有结束，此时不再响应新的start， 为0时表示start线程结束了
	{
		LOGD("xin: start_enable_flag = %d, 前一个start还未执行完，此时不响应新的start接口",start_enable_flag);		
        return 0;
	}else{
	    start_enable_flag = 1;
	}	
	if (can_start_flag == 1)
	{
	    return 0;  		
	}
    
	 
    if(restart_power_on_flag == 1) // 当内部stop DMA，fpga,关闭设备后，flag 置1， 当再启动start接口时不再重新上电，当对外大的停止采集下电后，此flag会置为 0，重新采集时再上电
	{
		;
	}else{
		poweron_spi(  );		
		int reg_ret_value = set_press_reg( smp_rate );	//配置压力采集寄存器 
		if( reg_ret_value == -1)
		{	
            LOGD("xin: 寄存器配置失败");	
			pressure_callback_backup.mspictl_callback( press_reg_fail_buf ,  false );//表示寄存器配置失败，回调 false给上层
			start_enable_flag = 0;
            spi_power_off(  );			
			return 0;
		}
	}
    usleep(2000000); //用于上层时域波形不丢波形数据，在这里直接延时2S,让硬 件预热稳定
	start_num ++;
	sem_init( &run_sem ,  0 ,  0 );   //初始化信号量
	
	pthread_create( &c_id ,  NULL ,  press_curve_thread ,  NULL );
    signal( SIGIO ,  read_press_data );	
    common_start( );
	
	while( thread_finished_flag == 0); // 用于等待算法线程线束时置1，若线程结束时，继续往下运行
	
	can_start_flag = 0;
	thread_finished_flag = 0;
	
	power_off_flag = 0;
	start_enable_flag = 0;
	
	LOGD( "xin: 点击压力曲线结束***** [%s]\n" ,  log_time( ) ); 	
	usleep( 100000 ); //用于底层虚拟地址和物理地址映射出错	
	
	return 0;	
}

static int start_pressure_dial( struct spictl_device_t* dev , float flag0_value )// 压力表盘模式
{	
   LOGD("xin: 点击压力表盘开始时start_num = %d",start_num);
   if(start_num != 0)
   {
	  return 0; 
   } 
    memset(read_60K_buf,0,SIZE_60K*sizeof(unsigned char));   
    memset(press_buf ,0, SIZE_64K*sizeof(unsigned char)); 	
	stop_smp_flag = 0;
    pflag0_value = flag0_value;
	
    //LOGD("xin: 点击压力表盘开始时 [%s]\n" ,  log_time( )); 
	LOGD("\nxin: 点击表盘开始时 start_enable_flag = %d, restart_power_on_flag= %d, can_start_flag = %d , thread_finished_flag = %d",	    start_enable_flag,restart_power_on_flag,can_start_flag,thread_finished_flag);
		
	if(start_enable_flag == 1)
	{
		LOGD("xin: start_enable_flag = %d, 不响应start接口",start_enable_flag);		
        return 0;
	}else{
	    start_enable_flag = 1;
	}
	if (can_start_flag == 1)
	{
	    return 0;  
	}
	
	
    if(restart_power_on_flag == 1) // 当内部stop DMA，fpga,关闭设备后，flag 置1， 当再启动start接口时不再重新上电，当对外大的停止采集下电后，此flag会置为 0，重新采集时再上电
	{
		;
	}else{
		poweron_spi(  );		
		
		int reg_ret_value = set_press_reg( smp_rate );	//设置压力采集寄存器 
		
		if( reg_ret_value == -1)
		{	
            LOGD("xin: 寄存器配置失败");	
			pressure_callback_backup.mspictl_callback( press_reg_fail_buf ,  false );//表示寄存器配置失败，回调 false给上层
			start_enable_flag = 0;
            spi_power_off(  );			
			return 0;
		}
	}
	
	usleep(2000000); //用于上层时域波形不丢波形数据，在这里直接延时2S,让硬 件预热稳定
	start_num++;
	sem_init( &run_sem ,  0 ,  0 );   //初始化信号量
	
	pthread_create( &c_id ,  NULL ,  press_dial_thread ,  NULL );
    signal( SIGIO ,  read_press_data );	
    common_start( );
	
	while( thread_finished_flag == 0); // 用于等待算法线程线束时置1，若线程结束时，继续往下运行
	can_start_flag = 0;
	thread_finished_flag = 0;
	
    power_off_flag = 0;	
	start_enable_flag = 0;
	
	LOGD( "xin: 点击压力表盘结束 [%s]\n" ,  log_time( ) ); 	
	usleep( 100000 ); //用于底层虚拟地址和物理地址映射出错	
	
	return 0; 
}



////////////振动采集功能如下
void read_evalute_data( int signo) //读取振动评估数据    通道2, 下限10HZ, 上限1000 ,长度4096
{	 
     LOGD( "xin: 读取振动评估数据时stop_smp_flag = %d, g_smpLength = %d , power_off_flag =%d  , [%s]\n" ,  stop_smp_flag,  g_smpLength , power_off_flag , log_time( ) );
	if( stop_smp_flag )
	{		 
		return;
	}		
	int total_len = g_smpLength;  //采样总点数
	int shang = total_len/SIZE_15360;  //15360 = 60K 的倍数
	int yu = total_len%SIZE_15360;  // 余数
		
	if( shang < 1 )
	{
		 if( read( fd , g_max_char_buf , yu*sizeof( float ) ) <0 )
		 {			 
			LOGD( "Error: spi slave device read fail !\n " ); 
		 }		
		 stop_smp_flag = 1;			 
         stop_fpga_dma();	  		 
		 g_smp_buf =( int* )&( g_max_char_buf ) ;         
	     sem_post( &run_sem );		
	}		
}

void read_vibrate_data( int signo) //读取振动采集数据
{	 
    int v_loop_num = 0;
    //LOGD( "xin: 读取振动数据时stop_smp_flag = %d,v_loop_num = %d, g_smpLength = %d , power_off_flag =%d  , [%s]\n" ,  stop_smp_flag, v_loop_num, g_smpLength , power_off_flag , log_time( ) );
	if(power_off_flag == 1) 
	{	
		LOGD("xin: 读取振动数据时，检测到下电标识，不再读取数据提前stop fpga DMA,post信号量");
		stop_smp_flag = 1;
		stop_fpga_dma();	
		sem_post( &run_sem ); 
		return ;
	}
    
	if( stop_smp_flag )
	{		 
		return;
	}	
    
	int i = 0;
	int total_len = g_smpLength;  //采样总点数
	int shang = total_len/SIZE_15360;  //15360 的倍数
	int yu = total_len%SIZE_15360;  // 余数	

	if( shang < 1 )
	{			 
		 if( read( fd , g_max_char_buf , yu*sizeof( float ) ) <0 )
		 {			 
			LOGD( "Error: spi slave device read fail !\n " ); 	
		 }	
		 stop_smp_flag = 1;
         stop_fpga_dma();	
		 
		 g_smp_buf =( int* )&( g_max_char_buf ) ;	 	
		 
		 sem_post( &run_sem ); 
		 LOGD( "xin: post振动信号量000 : [%s]\n" ,  log_time( ) );      
	}
		
	if( shang > 0 && yu > 0 )
	{		  
		  if( read( fd , read_60K_buf , SIZE_60K ) <0 )
		  {			  
			 LOGD( "Error: spi slave device read fail !\n " ); 
		  }
		 
		  if( v_loop_num < shang )
		  {			 				 
			 memcpy( &g_max_char_buf[SIZE_60K*v_loop_num] ,  &read_60K_buf ,  SIZE_60K );	
			 v_loop_num++;
		  }
          else if( v_loop_num == shang )
		  {		
	         stop_smp_flag = 1; //flag置1,停止继续读数据
	         stop_fpga_dma(); 
			 
			 memcpy( &g_max_char_buf[SIZE_60K*v_loop_num] ,  &read_60K_buf ,  yu*sizeof( float ) );
			 
			 g_smp_buf =( int* )&( g_max_char_buf ) ; 				 
			 
			 sem_post( &run_sem ); 
             v_loop_num = 0;
			 LOGD( "xin: post振动信号量111 : [%s]\n" ,  log_time( ) );              	 
		  }		
	}	
	if( shang > 0 && yu == 0 )
	{             
		 if( read( fd , read_60K_buf , SIZE_60K ) <0 )
		 {			 
			LOGD( "Error: spi slave device read fail !\n " );
		 }			
		 
		 if( v_loop_num < shang )
		 {			
			 memcpy( &g_max_char_buf[SIZE_60K*v_loop_num] ,  &read_60K_buf ,  SIZE_60K );	
			 v_loop_num++;
		 }
		 else if( v_loop_num == shang )
		 {		
	         stop_smp_flag = 1;
             stop_fpga_dma();	 
			 
			 g_smp_buf =( int* )&( g_max_char_buf ) ;	
			 
			 sem_post( &run_sem ); 
             v_loop_num = 0;
			 LOGD( "xin: post振动信号量222 : [%s]\n" ,  log_time( ) );              			 
		 }		
	}		
}

void read_calibration_data( int signo) //读取振动校准数据
{	
    int c_loop_num = 0;
   // LOGD( "xin: 读取振动校准数据stop_smp_flag = %d,c_loop_num = %d, g_smpLength = %d , power_off_flag =%d  , [%s]\n" ,  stop_smp_flag, c_loop_num, g_smpLength , power_off_flag , log_time( ) ); 
	if(power_off_flag == 1) 
	{	
		LOGD("xin: 读取振动数据时，检测到下电标识，不再读取数据提前stop fpga DMA,post信号量");
		stop_smp_flag = 1;
		stop_fpga_dma();	
		sem_post( &run_sem ); 		
		return ;
	}

	if( stop_smp_flag )
	{		 
		return;
	}	
    
	int i = 0;
	int total_len = g_smpLength;  //采样总点数
	int shang = total_len/SIZE_15360;  //15360 的倍数
	int yu = total_len%SIZE_15360;  // 余数	

	if( shang < 1 )
	{			 
		 if( read( fd , g_max_char_buf , yu*sizeof( float ) ) <0 )
		 {			 
			LOGD( "Error: spi slave device read fail !\n " ); 	
		 }	
		 stop_smp_flag = 1;
         stop_fpga_dma();	
		 
		 g_smp_buf =( int* )&( g_max_char_buf ) ;	 	
		 
		 sem_post( &run_sem ); 
		 LOGD( "xin: post振动信号量000 : [%s]\n" ,  log_time( ) );      
	}
		
	if( shang > 0 && yu > 0 )
	{		  
		  if( read( fd , read_60K_buf , SIZE_60K ) <0 )
		  {			  
			 LOGD( "Error: spi slave device read fail !\n " ); 
		  }
		 
		  if( c_loop_num < shang )
		  {			 				 
			 memcpy( &g_max_char_buf[SIZE_60K*c_loop_num] ,  &read_60K_buf ,  SIZE_60K );	
			 c_loop_num++;
		  }
          else if( c_loop_num == shang )
		  {	
             press_discard_16K_flag ++;
             c_loop_num = 0;
             if(press_discard_16K_flag > 1)//丢一波数据
             {
                 stop_smp_flag = 1; //flag置1,停止继续读数据
                 stop_fpga_dma(); 
                 
                 memcpy( &g_max_char_buf[SIZE_60K*c_loop_num] ,  &read_60K_buf ,  yu*sizeof( float ) );
                 
                 g_smp_buf =( int* )&( g_max_char_buf ) ; 				 
                 
                 sem_post( &run_sem ); 
                 c_loop_num = 0;
                 press_discard_16K_flag = 0;
                 LOGD( "xin: post振动信号量111 : [%s]\n" ,  log_time( ) );
             }
		  }		
	}
}

int t_discard_pnts = 0,t_max_freq = 0,t_min_freq = 0,t_wave_length = 0;
void *time_wave_thread( void* arg ) //时域线程
{	    
	timewave my_timewave;
	my_timewave = *( struct time_wave_para* )arg;
	int i=0 ;
    int t_temp_len =0;		
	float t_CH_data[3]={0.0};	 //各振动通道数据
	
    
	LOGD("时域运算线程maxfreq = %d, minfreq = %d, wavelength = %d, t_discard_pnts = %d",(int)my_timewave.max_freq,(int)my_timewave.min_freq,my_timewave.wave_length,t_discard_pnts);
    
	if( g_chNum == SINGLE_CH )
	{		
		if( t_max_freq == 500 || t_max_freq == 2500 )
		{
			g_smpLength = ( t_wave_length + t_discard_pnts +36 )*4;	//针对 这两个频率，需要1/4 抽点，所以采集长度 *4
		}else if( t_max_freq == 1000 || t_max_freq == 5000 )
		{
			g_smpLength = ( t_wave_length + t_discard_pnts +31 )*2;	//针对 这两个频率，需要1/2 抽点，所以采集长度 *2
		}else
		{
			g_smpLength = ( t_wave_length + t_discard_pnts );	//实际采集的波形长度			
		}

		LOGD( "xin: 时域运算线程单通道t_discard_pnts = %d ,  t_wave_length = %d ,  g_smpLength = %d"  , t_discard_pnts ,  t_wave_length ,  g_smpLength );
				
		//分配需要的内存
		float *time_CH1_smp_buf = NULL; //通道1 采样数据  长度为UI配置长度 + 丢弃的点数
		if( time_CH1_smp_buf == NULL)
		{
			time_CH1_smp_buf =( float* )malloc( g_smpLength*sizeof( float ) );
			if( time_CH1_smp_buf == NULL )
			{
				LOGD( "time_CH1_smp_buf 分配内存失败！" );
				exit( EXIT_FAILURE );		
			}
			memset( time_CH1_smp_buf , 0 , g_smpLength*sizeof( float ) );
		}		  
		 
		 
		sem_wait( &run_sem );//等待信号量 
		
		if(power_off_flag == 1)//接收信号量后第一时间判断是否有下电标识
		{
			LOGD("xin: recived post ");
			goto  stop_daq;
		}
        
        memset(time_CH1_smp_buf , 0 ,g_smpLength*sizeof(float) );			
	    memset(	t_CH_data , 0, 3*sizeof(float));	
		
		for( i=0;i< g_smpLength;i++ )	//转换采集的点数	
		{					
			analyze_CH_data( g_smp_buf[i] , t_CH_data );	
			time_CH1_smp_buf[i] = t_CH_data[2];				
		}		
		
		dis_dc_func(time_CH1_smp_buf, g_smpLength);	 //经过去直流分量算法
		//LOGD( "xin: SINGLE_CH_结束转换数据 is : [%s]\n" ,  log_time( ) );				
		
		t_temp_len = g_smpLength; ////FIR 低通滤波，单通道，采样长度		
		if(t_max_freq == 500 || t_max_freq == 2500 )//上限是5000 2500是1/4抽样 
		{							
			enter_FIR_Filter( time_CH1_smp_buf  , t_temp_len, t_max_freq); //FIR 低通滤波
			t_temp_len = g_smpLength/4 -36; //计算后长度变为1/4	
			
			enter_IIR_Filter( time_CH1_smp_buf , t_temp_len ,t_max_freq ,t_min_freq ); //IIR 高通滤波
		}else if( t_max_freq == 1000 || t_max_freq == 5000)	 //上限是1000 5000是1/2抽样 
		{				
			enter_FIR_Filter( time_CH1_smp_buf  , g_smpLength, t_max_freq); 
			t_temp_len = g_smpLength/2 -31; //计算后长度变为1/2	
						
			enter_IIR_Filter( time_CH1_smp_buf ,t_temp_len ,t_max_freq ,t_min_freq ); 
							
		}else  //其它上限频率只经过IIR
		{	
			enter_IIR_Filter( time_CH1_smp_buf , t_temp_len ,t_max_freq ,t_min_freq ); 
		}
	
		//LOGD("SINGLE_CH_temp_len = %d , t_discard_pnts = %d, wave_length = %d, wave_length+discard_pnts = %d" ,t_temp_len,t_discard_pnts,t_wave_length,t_wave_length + t_discard_pnts  );			
		memcpy( time_CH1_smp_buf ,  &time_CH1_smp_buf[t_discard_pnts] ,  t_wave_length*sizeof( float )  );//经IIR滤波后 去掉丢弃的点							
		//LOGD( "xin: SINGLE_CH_退出算法 is : [%s]\n" ,  log_time( ) );          		
	
    stop_daq:	
        if(power_off_flag == 1) 
		{	
            LOGD("xin: 时域运算线程检测到下电标识为1，释放内存，销毁信号量，回调false给上层 [%s]\n" ,  log_time( ) ); 
			if( time_CH1_smp_buf !=NULL)
			{			
				free( time_CH1_smp_buf );
				time_CH1_smp_buf =NULL; 
			}				
			
			can_start_flag = 1;
			LOGD("xin: free——malloc=====下电时提前释放后, 调用回调函数======");
			sem_destroy( &run_sem );
			thread_finished_flag = 1;			
			
			usleep(10000);                                                   
			
			vibrate_callback_backup.single_ch_callback( invalid_buf , 0,  false );	/////回调时域波形			
		    return NULL;
		}else{           	
            LOGD("xin: 时域运算线程正常回调数据");			
			vibrate_callback_backup.single_ch_callback( time_CH1_smp_buf , t_wave_length ,  true );	/////回调时域波形
		}
		
		LOGD( "xin: SINGLE_CH_回调结束 is : [%s]\n" ,  log_time( ) );	
		if( time_CH1_smp_buf !=NULL)
		{			
			free( time_CH1_smp_buf );
			time_CH1_smp_buf =NULL; 
		}				
		
		can_start_flag = 1;
		LOGD("xin: free——malloc=====正常释放后, 调用回调函数======");
		sem_destroy( &run_sem );
		if(power_off_flag == 1) 
		{
			LOGD("xin: 时域运算线程检测到下电标识为1");
			thread_finished_flag = 1;
			usleep(10000);                                          
			
			vibrate_callback_backup.single_ch_callback( invalid_buf , 0,  false );	
			return NULL;
		}
		thread_finished_flag = 1;		
	}
	
	if( g_chNum == DOUBLE_CH ) //双通道内存为单通道的两倍
	{			
		if( t_max_freq == 500 || t_max_freq == 2500 )
		{
			g_smpLength = ( t_wave_length + t_discard_pnts  )*2*4;	//针对 这两个频率，需要1/4 抽点，所以采集长度 *4
		}else if( t_max_freq == 1000 || t_max_freq == 5000 )
		{
			g_smpLength = ( t_wave_length + t_discard_pnts )*2*2;	//针对 这两个频率，需要1/2 抽点，所以采集长度 *2
		}else
		{
			g_smpLength = ( t_wave_length+ t_discard_pnts )*2;	//实际采集的波形长度
		}
	
		//LOGD( "xin: 时域运算线程双通道t_discard_pnts = %d ,  t_wave_length = %d ,  g_smpLength = %d"  , t_discard_pnts ,  t_wave_length,  g_smpLength );        
	    
        	
        //分配需要的内存			
        float *time_CH1_smp_buf =NULL; //通道1数据  采样数据
		if( time_CH1_smp_buf == NULL)
		{
			time_CH1_smp_buf =( float* )malloc(g_smpLength*sizeof( float ) );
			if( time_CH1_smp_buf == NULL )
			{
				LOGD( "time_CH1_smp_buf 分配内存失败！" );
				exit( EXIT_FAILURE );		
			}
			memset( time_CH1_smp_buf , 0 , g_smpLength*sizeof( float ) );
		}
		
		float *time_CH2_smp_buf =NULL; //通道2数据 采样数据
		if( time_CH2_smp_buf == NULL)
		{
			time_CH2_smp_buf =( float* )malloc(g_smpLength*sizeof( float ) );
			if( time_CH2_smp_buf == NULL )
			{
				LOGD( "time_CH2_smp_buf 分配内存失败！" );
				exit( EXIT_FAILURE );		
			}
			memset( time_CH2_smp_buf , 0 , g_smpLength*sizeof( float ) );	
		}					
			
		sem_wait( &run_sem );//等待信号量
		
        memset(time_CH1_smp_buf , 0 ,g_smpLength*sizeof(float) );	
		memset(time_CH2_smp_buf , 0 ,g_smpLength*sizeof(float) );
		
		 
		for( i=0;i< g_smpLength;i++ )	//转换采集数据	
		{					
			analyze_CH_data( g_smp_buf[i] , t_CH_data );	
			time_CH1_smp_buf[i] = t_CH_data[1];
            time_CH2_smp_buf[i] = t_CH_data[2];				
		}	
        for(i = 0; i<= g_smpLength/2; i++)
		{
			time_CH1_smp_buf[i] = time_CH1_smp_buf[2*i];
			time_CH2_smp_buf[i] = time_CH2_smp_buf[2*i+1];
		}			
		//LOGD( "xin: DOUBLE_CH_结束转换数据 is : [%s]\n" ,  log_time( ) );      	
		
		#if 1				
			t_temp_len = g_smpLength/2; //FIR 低通滤波，原采样长度为双通道 一半			
			if(t_max_freq == 500 || t_max_freq == 2500 )//上限是5000 2500是1/4抽样 
			{					
				enter_FIR_Filter( time_CH1_smp_buf  , t_temp_len, t_max_freq); //FIR 低通滤波
				enter_FIR_Filter( time_CH2_smp_buf  , t_temp_len, t_max_freq); 					
				t_temp_len = g_smpLength/2/4; //FIR 低通滤波,计算后长度变为1/4	
									
				enter_IIR_Filter( time_CH1_smp_buf , t_temp_len , t_max_freq , t_min_freq ); //IIR 高通滤波
				enter_IIR_Filter( time_CH2_smp_buf , t_temp_len , t_max_freq , t_min_freq ); //IIR 高通滤波
			}else if( t_max_freq == 1000 || t_max_freq == 5000)	 //上限是1000 5000是1/2抽样 
			{
				enter_FIR_Filter( time_CH1_smp_buf  , t_temp_len,  t_max_freq); //FIR 低通滤波
				enter_FIR_Filter( time_CH2_smp_buf  , t_temp_len,  t_max_freq); 
				t_temp_len = g_smpLength/2/2; //FIR 低通滤波,计算后长度变为1/2
				
				enter_IIR_Filter( time_CH1_smp_buf , t_temp_len , t_max_freq , t_min_freq ); //IIR 高通滤波
				enter_IIR_Filter( time_CH2_smp_buf , t_temp_len , t_max_freq , t_min_freq ); //IIR 高通滤波
				
			}else  //其它上限频率只经过IIR
			{	
				enter_IIR_Filter( time_CH1_smp_buf , t_temp_len , t_max_freq , t_min_freq );  //IIR 滤波长度为 采集长度的一半
				enter_IIR_Filter( time_CH2_smp_buf , t_temp_len , t_max_freq , t_min_freq );
			}
			LOGD("DOUBLE_CH_temp_len = %d ,t_discard_pnts = %d,wave_length = %d, wave_length+discard_pnts = %d" ,t_temp_len,t_discard_pnts,t_wave_length, t_wave_length + t_discard_pnts );	
			memcpy( time_CH1_smp_buf ,  &time_CH1_smp_buf[t_discard_pnts] ,  t_wave_length*sizeof( float ));// 返回实际需要的点数，去掉丢弃的点
			memcpy( time_CH2_smp_buf ,  &time_CH2_smp_buf[t_discard_pnts] ,  t_wave_length*sizeof( float ));
		
		#endif		
		LOGD( "xin: DOUBLE_CH_退出算法 is : [%s]\n" ,  log_time( ) );
			
        if(power_off_flag == 1)
		{	 
            LOGD("xin: 开始回调时power_off_flag ==1");		
            if(	time_CH1_smp_buf != NULL)
			{
				free( time_CH1_smp_buf ); 
				time_CH1_smp_buf = NULL;	
			}
			if( time_CH2_smp_buf != NULL)
			{
				free( time_CH2_smp_buf ); 
				time_CH2_smp_buf = NULL;
			}					
			
			can_start_flag = 1;	
			LOGD("xin: free——malloc=====下电时提前释放后, 调用回调函数======");
			sem_destroy( &run_sem );
			thread_finished_flag = 1;	
			usleep(10000);  // 用于线程执行后后面flag 执行
			vibrate_callback_backup.double_ch_callback( invalid_buf ,  invalid_buf  , 0 ,  false ); /////回调 UI长度
            return NULL;			
		}else{	
            //LOGD("xin: t_wave_length = %d",t_wave_length);		
			vibrate_callback_backup.double_ch_callback( time_CH1_smp_buf ,  time_CH2_smp_buf  , t_wave_length ,  true ); /////回调 UI长度	
		}
		
		LOGD( "xin: DOUBLE_CH_回调结束 is : [%s]\n" ,  log_time( ));					
		if(	time_CH1_smp_buf != NULL)
		{
			free( time_CH1_smp_buf ); 
			time_CH1_smp_buf = NULL;	
		}
		if( time_CH2_smp_buf != NULL)
		{
			free( time_CH2_smp_buf ); 
			time_CH2_smp_buf = NULL;
		}					
		
		can_start_flag = 1;	
		LOGD("xin: free——malloc=====正常释放后, 调用回调函数======");
		sem_destroy( &run_sem );
		thread_finished_flag = 1;
	}
	return NULL;
}

int r_discard_pnts = 0,r_max_freq = 0,r_min_freq = 0,r_wave_length = 0;
void *total_rend_thread( void* arg ) //总值趋势线程
{
    totalrend my_totalrend;
    my_totalrend = *( struct total_rend_para* )arg;		
	int i=0;		
	int r_temp_len = 0;
	float r_CH1_value[1] ={0.0};
	float r_CH2_value[1] ={0.0};	
	float r_CH_data[3]={0.0};
    
	if( g_chNum == SINGLE_CH  )
	{		        
        if( r_max_freq == 500 || r_max_freq == 2500 )
		{
			g_smpLength = ( r_wave_length + r_discard_pnts +36 )*4;	//针对 这两个频率，需要1/4 抽点，所以采集升度 *4
		}else if( r_max_freq == 1000 || r_max_freq == 5000 )
		{
			g_smpLength = ( r_wave_length + r_discard_pnts +31 )*2;	//针对 这两个频率，需要1/2 抽点，所以采集升度 *2
		}else
		{
			g_smpLength = ( r_wave_length + r_discard_pnts );	//实际采集的波形长度
		}
		
		LOGD( "xin: 总值趋势线程单通道_r_discard_pnts = %d ,  r_wave_length = %d ,  g_smpLength = %d"  , r_discard_pnts ,  r_wave_length ,  g_smpLength );
		
		//分配需要的内存		
		float *rend_CH1_smp_buf =NULL; //通道1 采集数据
		if( rend_CH1_smp_buf == NULL)
		{
			rend_CH1_smp_buf =( float* )malloc( g_smpLength*sizeof( float ) );
			if( rend_CH1_smp_buf == NULL )
			{
				LOGD( "rend_CH1_smp_buf 分配内存失败！" );
				exit( EXIT_FAILURE );		
			}
			memset( rend_CH1_smp_buf , 0 , g_smpLength*sizeof( float ) );			
		}
			
		sem_wait( &run_sem );//等待信号量
		
        if(power_off_flag == 1)
		{
			LOGD("xin: recived post ");
			goto  stop_daq;
		}
		
		memset( rend_CH1_smp_buf , 0 , g_smpLength*sizeof( float ) );
        memset(	r_CH_data , 0, 3*sizeof(float));	
        r_CH1_value[0] = 0.0;
		r_CH2_value[0] = 0.0;			
	    
		
	    for( i=0;i< g_smpLength;i++ )	//转换采集的点数	
		{					
			analyze_CH_data( g_smp_buf[i] , r_CH_data );	
			rend_CH1_smp_buf[i] = r_CH_data[2];				
		}    
		dis_dc_func(rend_CH1_smp_buf,g_smpLength); //去除直流分量算法
		//LOGD( "xin: SINGLE_CH_结束转换数据 is : [%s]\n" ,  log_time( ) );
		
				
		r_temp_len = g_smpLength;		////FIR 低通滤波，单通道，采样长度	
		if(r_max_freq == 500 || r_max_freq == 2500 )//上限5000 2500 是1/4抽样 
		{				
			enter_FIR_Filter( rend_CH1_smp_buf  , r_temp_len, r_max_freq); //FIR 低通滤波
			r_temp_len = g_smpLength/4 -36; //计算后长度变为1/4
			
			enter_IIR_Filter( rend_CH1_smp_buf , r_temp_len , r_max_freq , r_min_freq ); //IIR 高通滤波
		}else if( r_max_freq == 1000 || r_max_freq == 5000)	 //上限1000 5000是1/2抽样 
		{
			enter_FIR_Filter( rend_CH1_smp_buf  , r_temp_len,  r_max_freq); //FIR 低通滤波
			r_temp_len = g_smpLength/2 -31; //计算后长度变为1/2	
			
			enter_IIR_Filter( rend_CH1_smp_buf , r_temp_len , r_max_freq , r_min_freq ); //IIR 高通滤波				
		}else  //其它上限频率只经过IIR
		{	               		
			enter_IIR_Filter( rend_CH1_smp_buf , r_temp_len , r_max_freq , r_min_freq ); //IIR 高通滤波，长度是采样的点
		}			
		
		LOGD("SINGLE_CH_r_temp_len = %d , r_discard_pnts = %d, wave_length = %d, wave_length+discard_pnts = %d" ,r_temp_len,r_discard_pnts,r_wave_length, r_wave_length + r_discard_pnts  );			
		
		memcpy( rend_CH1_smp_buf , &rend_CH1_smp_buf[r_discard_pnts] , r_wave_length*sizeof( float ) );//将IIR滤波后 去掉丢弃的点	
		//LOGD( "xin: SINGLE_CH_退出算法 is : [%s]\n" ,  log_time( ) );
        	
		
		r_CH1_value[0] = rend_value( rend_CH1_smp_buf , r_wave_length , my_totalrend.total_value_type ); //总值趋势算法           			
		LOGD( "xin: SINGLE_CH_CH1_value = %f" , r_CH1_value[0] );	
		
	stop_daq:
	    if(power_off_flag == 1) 
		{	  
            LOGD("xin: 总值数据线程检测到下电标识为1，释放内存，销毁信号量，回调false给上层 [%s]\n" ,  log_time( ) );
            if( rend_CH1_smp_buf != NULL)
			{			
				free( rend_CH1_smp_buf ); 
				rend_CH1_smp_buf =NULL;
			}
						
			can_start_flag = 1;
			LOGD("xin: free——malloc=====下电时提前释放后, 调用回调函数======");
			sem_destroy( &run_sem );
			thread_finished_flag = 1;		
			usleep(10000);  // 用于线程执行后后面flag 执行
			
			vibrate_callback_backup.single_ch_callback( invalid_buf  , 0 ,  false ); 
			return NULL;
		}else{
			LOGD("xin: 总值数据线程正常回调数据");
			vibrate_callback_backup.single_ch_callback( r_CH1_value , sizeof( r_CH1_value )/sizeof( float ) ,  true ); /////回调			
		}		
		LOGD( "xin: SINGLE_CH_回调结束 is : [%s]\n" ,  log_time( ) );	
		
		
		if( rend_CH1_smp_buf != NULL)
		{			
			free( rend_CH1_smp_buf ); 
			rend_CH1_smp_buf =NULL;
		}
					
		can_start_flag = 1;
		LOGD("xin: free——malloc=====正常释放后, 调用回调函数======");
		sem_destroy( &run_sem );
		if(power_off_flag == 1) 
		{
			LOGD("xin: 时域数据线程检测到下电标识为1");
			thread_finished_flag = 1;
			usleep(10000);                                          
			
			vibrate_callback_backup.single_ch_callback( invalid_buf , 0,  false );	
			return NULL;
		}
		thread_finished_flag = 1;			      		
	}

	if( g_chNum == DOUBLE_CH ) //双通道内存为单通道时的两倍
	{	
        if( r_max_freq == 500 || r_max_freq == 2500 )
		{
			g_smpLength = ( r_wave_length + r_discard_pnts  )*2*4;	//针对 这两个频率，需要1/4 抽点，所以采集长度 *4
		}else if( r_max_freq == 1000 || r_max_freq == 5000 )
		{
			g_smpLength = ( r_wave_length + r_discard_pnts  )*2*2;	//针对 这两个频率，需要1/2 抽点，所以采集长度 *2
		}else
		{
			g_smpLength = ( r_wave_length + r_discard_pnts )*2;	//实际采集的波形长度
		}
		
		LOGD( "xin: total_rend_thread_DOUBLE_CH_r_discard_pnts = %d ,  r_wave_length = %d ,  g_smpLength = %d"  , r_discard_pnts ,  r_wave_length ,  g_smpLength );
       
	    //分配需要的内存		
		float *rend_CH1_smp_buf =NULL; //通道1 采样数据
		if( rend_CH1_smp_buf == NULL)
		{
			rend_CH1_smp_buf =( float* )malloc( g_smpLength*sizeof( float ) );
			if( rend_CH1_smp_buf == NULL )
			{
				LOGD( "rend_CH1_smp_buf 分配内存失败！" );
				exit( EXIT_FAILURE );		
			}
			memset( rend_CH1_smp_buf , 0 , g_smpLength*sizeof( float ) );	
		}

		float *rend_CH2_smp_buf =NULL; //通道2 采样数据
		if( rend_CH2_smp_buf == NULL)
		{
			rend_CH2_smp_buf =( float* )malloc( g_smpLength*sizeof( float ) );
			if( rend_CH2_smp_buf == NULL )
			{
				LOGD( "rend_CH2_smp_buf 分配内存失败！" );
				exit( EXIT_FAILURE );		
			}
			memset( rend_CH2_smp_buf , 0 , g_smpLength*sizeof( float ) );
		}		
		
				
		sem_wait( &run_sem );//等待信号量					
		memset( rend_CH1_smp_buf , 0 , g_smpLength*sizeof( float ) );	
		memset( rend_CH2_smp_buf , 0 , g_smpLength*sizeof( float ) );
		
				 		    
		for( i=0;i< g_smpLength;i++ )			
		{					
			analyze_CH_data( g_smp_buf[i] , r_CH_data );  //调用解析通道数据的函数	
			rend_CH1_smp_buf[i] = r_CH_data[1];
			rend_CH2_smp_buf[i] = r_CH_data[2];
		}			
		for( i=0;i<=g_smpLength/2;i++ )
		{					
			rend_CH1_smp_buf[i] = rend_CH1_smp_buf[i*2]; //通道1
			rend_CH2_smp_buf[i] = rend_CH2_smp_buf[i*2+1];//通道2					
		}		
		LOGD( "xin: DOUBLE_CH_数据转换结束 is : [%s]\n" ,  log_time( ) );
		
		#if 1				
			r_temp_len = g_smpLength/2; //FIR 低通滤波，原采样长度为双通道 一半	
			if(r_max_freq == 500 || r_max_freq == 2500 )//上限是5000 2500是1/4抽样 
			{
				enter_FIR_Filter( rend_CH1_smp_buf  , r_temp_len, r_max_freq); //FIR 低通滤波
				enter_FIR_Filter( rend_CH2_smp_buf  , r_temp_len, r_max_freq); 
				r_temp_len = g_smpLength/2/4; //计算后长度变为1/4
				
				enter_IIR_Filter( rend_CH1_smp_buf , r_temp_len ,r_max_freq ,r_min_freq ); //IIR 高通滤波
				enter_IIR_Filter( rend_CH2_smp_buf , r_temp_len ,r_max_freq ,r_min_freq ); //IIR 高通滤波
			}else if( r_max_freq == 1000 || r_max_freq == 5000)	 //上限是1000 5000是1/2抽样 
			{				
				enter_FIR_Filter( rend_CH1_smp_buf  , r_temp_len, r_max_freq); //FIR 低通滤波，原采样长度，计算后长度变为1/2
				enter_FIR_Filter( rend_CH2_smp_buf  , r_temp_len, r_max_freq); //FIR 低通滤波，原采样长度，计算后长度变为1/2
				r_temp_len = g_smpLength/2/2; //计算后长度变为1/2
			
				enter_IIR_Filter( rend_CH1_smp_buf , r_temp_len ,r_max_freq ,r_min_freq ); //IIR 高通滤波
				enter_IIR_Filter( rend_CH2_smp_buf , r_temp_len ,r_max_freq ,r_min_freq ); //IIR 高通滤波
			}else{  //其它上限频率只经过IIR
				enter_IIR_Filter( rend_CH1_smp_buf , r_temp_len ,r_max_freq ,r_min_freq );  //IIR 滤波长度为 采集长度的一半
				enter_IIR_Filter( rend_CH2_smp_buf , r_temp_len ,r_max_freq ,r_min_freq );
			}				
					
			LOGD("DOUBLE_CH_r_temp_len = %d , r_discard_pnts = %d, wave_length = %d, wave_length+discard_pnts = %d" ,r_temp_len,r_discard_pnts,r_wave_length, r_wave_length + r_discard_pnts );	
			memcpy( rend_CH1_smp_buf , &rend_CH1_smp_buf[r_discard_pnts] ,r_wave_length*sizeof( float ) );// 返回实际需要的点数，去掉丢弃的点
			memcpy( rend_CH2_smp_buf , &rend_CH2_smp_buf[r_discard_pnts] ,r_wave_length*sizeof( float ) );			
        #endif	
		LOGD( "xin: DOUBLE_CH_退出算法 is : [%s]\n" ,  log_time( ) );
		
		r_CH1_value[0] = rend_value( rend_CH1_smp_buf , r_wave_length , my_totalrend.total_value_type );	 //计算总值趋势值 CH1 
		r_CH2_value[0] = rend_value( rend_CH2_smp_buf , r_wave_length , my_totalrend.total_value_type );	 //计算总值趋势值 CH2	
		LOGD( "xin: DOUBLE_CH_CH1_value = %f ,  r_CH2_value = %f" , r_CH1_value[0] , r_CH2_value[0] );				
		
        if(power_off_flag == 1)
		{	 
            LOGD("xin: 开始回调时power_off_flag ==1");	
            if( rend_CH1_smp_buf != NULL)
			{
				free( rend_CH1_smp_buf ); 
				rend_CH1_smp_buf =NULL;
			}
			if( rend_CH2_smp_buf != NULL)
			{
				free( rend_CH2_smp_buf );
				rend_CH2_smp_buf =NULL;
			}		
			
			can_start_flag = 1;	
			LOGD("xin: free——malloc=====下电时提前释放后, 调用回调函数======");
			sem_destroy( &run_sem );
			thread_finished_flag = 1;
		    usleep(10000);  // 用于线程执行后后面flag 执行
			vibrate_callback_backup.double_ch_callback( invalid_buf  , invalid_buf, 0 ,  false ); /////回调 UI长度
			return NULL;
		}else{		
			vibrate_callback_backup.double_ch_callback( r_CH1_value , r_CH2_value , sizeof( r_CH1_value )/sizeof( float ) , true );	/////回调
		}		
        
		LOGD( "xin: DOUBLE_CH_回调结束 is : [%s]\n" ,  log_time( ) );		   
	   
		if( rend_CH1_smp_buf != NULL)
		{
			free( rend_CH1_smp_buf ); 
			rend_CH1_smp_buf =NULL;
		}
		if( rend_CH2_smp_buf != NULL)
		{
			free( rend_CH2_smp_buf );
			rend_CH2_smp_buf =NULL;
		}		
		
        can_start_flag = 1;	
		LOGD("xin: free——malloc=====正常释放后, 调用回调函数======");
		sem_destroy( &run_sem );
		thread_finished_flag = 1;		       		
	}	    	
	return NULL;
}


int e_discard_pnts = 0 ,e_max_freq = 0,e_min_freq = 0,e_wave_length = 0;
void *evalute_level_thread( void* arg ) //振动等级评估线程
{    
    timewave my_timewave;
    my_timewave = *( struct time_wave_para* )arg;	
    int i=0;
    int e_temp_len =0;	
	float evalute_value[1] ={0.0};	 //0位:表示速度
	float e_CH_data[3]={0.0};	
    
	if( e_max_freq == 1000 )
	{
		g_smpLength = ( e_wave_length + e_discard_pnts +31 )*2;	//针对 这1000频率，需要1/2 抽点，所以采集升度 *2
	}	
	LOGD( "xin: 振动等级评估线程e_discard_pnts = %d ,  e_wave_length = %d ,  g_smpLength = %d"  , e_discard_pnts ,  e_wave_length ,  g_smpLength );
			
    //分配需要的内存
    float *evalute_CH1_smp_buf =NULL; //振动评估 通道1 采集数据
	if( evalute_CH1_smp_buf == NULL)
	{
		evalute_CH1_smp_buf =( float* )malloc( g_smpLength*sizeof( float ) );
		if( evalute_CH1_smp_buf == NULL )
		{
			LOGD( "evalute_CH1_smp_buf 分配内存失败！" );
			exit( EXIT_FAILURE );		
		}
		memset( evalute_CH1_smp_buf , 0 , g_smpLength*sizeof( float ) ); 
	}  	
	
	sem_wait( &run_sem );//等待信号量	
    memset( evalute_CH1_smp_buf , 0 , g_smpLength*sizeof( float ) );  			
    memset(	e_CH_data , 0, 3*sizeof(float));	
	
	for( i=0;i< g_smpLength;i++ )	//转换采集的点数	
	{					
		analyze_CH_data( g_smp_buf[i] , e_CH_data );	
		evalute_CH1_smp_buf[i] = e_CH_data[2]; //单通道振动采集默认CHB 数据				
	}	
	dis_dc_func(evalute_CH1_smp_buf,g_smpLength);		
	LOGD( "xin: SINGLE_CH_结束转换数据 is : [%s]\n" ,  log_time( ) );	
	
	e_temp_len = g_smpLength; ////FIR 低通滤波，单通道，采样长度	
	if( e_max_freq == 1000 )	 //上限是1000 是1/2抽样 
	{
		enter_FIR_Filter( evalute_CH1_smp_buf  , g_smpLength, e_max_freq); //FIR 低通滤波
		e_temp_len = g_smpLength/2 -31; //计算后长度变为1/2	
		
		enter_IIR_Filter( evalute_CH1_smp_buf ,e_temp_len , e_max_freq , e_min_freq ); //IIR 高通滤波
	}		
	memcpy( evalute_CH1_smp_buf ,  &evalute_CH1_smp_buf[e_discard_pnts] ,  e_wave_length*sizeof( float ) );	
	LOGD( "xin: SINGLE_CH_退出算法 is : [%s]\n" ,  log_time( ) );		
	
	evalute_value[0] = rend_value( evalute_CH1_smp_buf ,  e_wave_length ,  0 );	//速度有效值 0表示有效值
	LOGD( "xin: 计算出速度有效值 = %f" ,  evalute_value[0] );	

    if( evalute_CH1_smp_buf != NULL)
	{
		free( evalute_CH1_smp_buf ); 
		evalute_CH1_smp_buf =NULL;	
	}
	//LOGD("xin: free——malloc=====正常释放后, 调用回调函数======");			
			
	////////自动stop
	start_enable_flag = 0;	
	restart_power_on_flag = 0;  //此时两个flag 置0,防止其它测试时卡住
			
    int res = ioctl( fd ,  SPIDEV_IOC_RXSTREAMOFF ,  NULL ); //停止DMA搬运
    if( res !=0 ) //启动DMA采集失败时，重新启动
	{		
        LOGD( "振动等级评估: DMA采集-停止失败 ,fd = %d",fd ); 
    }else{
        LOGD( "振动等级评估: DMA采集-停止成功 ,fd = %d",fd ); 
    }
	swrite( StopSampleAddr , StopSampleData );//stop fpga
    close( fd );					
	spi_power_off( );  //下电	  
	sem_destroy( &run_sem );  
	
	vibrate_callback_backup.single_ch_callback( evalute_value , sizeof( evalute_value )/sizeof( float ) , true ); /////回调	
	//LOGD( "xin: SINGLE_CH_回调结束 is : [%s]\n" ,  log_time( ) );
		
	return NULL;
}

int v_discard_pnts = 0, v_max_freq = 0, v_min_freq = 0,v_wave_length = 0;
void *vibrate_calib_thread( void* arg ) //振动校准线程
{    
    timewave my_timewave;
    my_timewave = *( struct time_wave_para* )arg;	
    int i=0;
    int v_temp_len =0;	
    float calib_value[2] ={0.0};	 //0位:表示峰峰值，1位：表示平均值
	float v_CH_data[3]={0.0};	

      
    
    g_smpLength = ( v_wave_length + v_discard_pnts );	//实际采集的波形长度
	LOGD( "xin: 振动校准线程v_discard_pnts = %d ,  v_wave_length = %d ,  g_smpLength = %d"  , v_discard_pnts ,  v_wave_length ,  g_smpLength );
			
    //分配需要的内存
    float *calib_CH1_smp_buf =NULL; //振动校准  通道1 采集数据
	if( calib_CH1_smp_buf == NULL)
	{
		calib_CH1_smp_buf =( float* )malloc( g_smpLength*sizeof( float ) );
		if( calib_CH1_smp_buf == NULL )
		{
			LOGD( "calib_CH1_smp_buf 分配内存失败！" );
			exit( EXIT_FAILURE );		
		}
		memset( calib_CH1_smp_buf , 0 , g_smpLength*sizeof( float ) ); 
	}  	
	
	sem_wait( &run_sem );//等待信号量	
    memset( calib_CH1_smp_buf , 0 , g_smpLength*sizeof( float ) );  			
    memset(	v_CH_data , 0, 3*sizeof(float));	
	
	for( i=0;i< g_smpLength;i++ )	//转换采集的点数	
	{					
		analyze_CH_data( g_smp_buf[i] , v_CH_data );	
		calib_CH1_smp_buf[i] = v_CH_data[2]; //单通道振动采集默认CHB 数据				
	}	
	dis_dc_func(calib_CH1_smp_buf,g_smpLength);		
	
	v_temp_len = g_smpLength; ////FIR 低通滤波，单通道，采样长度	
	
    enter_IIR_Filter( calib_CH1_smp_buf , v_temp_len , v_max_freq , v_min_freq ); //IIR 高通滤波，长度是采样的点
	memcpy( calib_CH1_smp_buf ,  &calib_CH1_smp_buf[v_discard_pnts] ,  v_wave_length*sizeof( float ) );	
	
	calib_value[0] = rend_value( calib_CH1_smp_buf ,  v_wave_length ,  2 );	//峰峰值 2
	calib_value[1] = rend_value( calib_CH1_smp_buf ,  v_wave_length ,  3 );	//平均值 3
        
	LOGD( "xin: 计算出振动校准 峰峰值 = %f, 平均值 = %f\n" ,  calib_value[0], calib_value[1]);	

    if( calib_CH1_smp_buf != NULL)
	{
		free( calib_CH1_smp_buf ); 
		calib_CH1_smp_buf =NULL;	
	}
	
	////////自动stop
    int res = ioctl( fd ,  SPIDEV_IOC_RXSTREAMOFF ,  NULL ); //停止DMA搬运
    if( res !=0 ) //启动DMA采集失败时，重新启动
	{		
        LOGD( "振动校准: DMA采集-停止失败 ,fd = %d",fd ); 
    }else{
        LOGD( "振动校准: DMA采集-停止成功 ,fd = %d",fd ); 
    }
	swrite( StopSampleAddr , StopSampleData );//停止FPGA采集
    close( fd );					
	spi_power_off( );  //下电	  
	sem_destroy( &run_sem );  	
	vibrate_callback_backup.single_ch_callback( calib_value , sizeof( calib_value )/sizeof( float ) , true ); /////回调	
		
	return NULL;
}



static int start_vibrate_CH_timewave( struct spictl_device_t* dev ,  int ch_num , struct time_wave_para tWave  )//时域波形
{	
   LOGD("xin: 响应时域点击开始start_num = %d",start_num);
   if(start_num != 0)
   {
      /* LOGD("xin: start_vibrate_CH_timewave stop未完全结束，此时要重新采集");
      start_num =0;
      power_off_flag = 0;
      vibrate_callback_backup.single_ch_callback( dma_fail_buf ,1 ,true);//DMA关闭异常
      start_enable_flag = 0;
      spi_power_off(  ); */	
      
	  return 0; 
   }
    memset(read_60K_buf,0, SIZE_60K*sizeof(unsigned char));
    memset(g_max_char_buf ,0, MAX_SIZE*sizeof(unsigned char)); 
    
	g_chNum = ch_num;	 
	stop_smp_flag = 0;	
	g_smpLength = 0;
    test_mode = tWave.version_mode;   
    
    t_discard_pnts =  Get_InvalidNum((int)tWave.max_freq , (int)tWave.min_freq );  
    t_max_freq = (int)tWave.max_freq;
    t_min_freq = (int)tWave.min_freq;
    t_wave_length = tWave.wave_length;
    
    LOGD("xin: 点击时域开始时 power_off_flag = %d,start_enable_flag = %d, restart_power_on_flag= %d, can_start_flag = %d , thread_finished_flag = %d",power_off_flag,start_enable_flag,restart_power_on_flag,can_start_flag,thread_finished_flag);
	LOGD( "xin: start_vibrate_CH_timewave_ch_num = %d , data_type = %d , signal_type = %d ,  min_freq = %d , max_freq = %d , wave_length = %d, test_mode =%d", ch_num ,  tWave.data_type ,  tWave.signal_type ,  (int)tWave.min_freq ,  (int)tWave.max_freq  , tWave.wave_length, test_mode );
    
	if(start_enable_flag == 1) // 1表示前一个start 线程还没有结束，此时不再响应新的start， 为0时表示start线程结束了
	{
		LOGD("xin: start_enable_flag = %d, 前一个start还未执行完，此时不响应新的start接口",start_enable_flag);		
        return 0;
	}else{
	    start_enable_flag = 1;
	}
    
    if (can_start_flag == 1)
	    return 0;    
    
	if( !is_right_length(tWave.wave_length)) //判断下发的采样长度是否支持
	{
		start_enable_flag = 0; //用于波形长度为0时，防止再手动停止时再采集时 ，start_enable_flag = 0， 直接 回调false给上层，所以此处要置0
		return 0;
	}	
    
    if(restart_power_on_flag == 1) // 当内部stop DMA，fpga,关闭设备后，flag 置1， 当再启动start接口时不再重新上电，当对外大的停止采集下电后，此flag会置为 0，重新采集时再上电
	{
		;
	}else{
		poweron_spi(  );				
		int reg_ret_value =0;
		if( g_chNum == SINGLE_CH )	
			reg_ret_value = set_singleCH_vibrate_reg( tWave.signal_type , (int)tWave.max_freq , (int)tWave.min_freq);//设置单通道采集寄存器	
		if( g_chNum == DOUBLE_CH ) 
			reg_ret_value = set_doubleCH_vibrate_reg( tWave.signal_type , (int)tWave.max_freq , (int)tWave.min_freq );//设置双通道采集寄存器
		
		if( reg_ret_value == -1)
		{
			LOGD("xin: 寄存器配置失败");
			vibrate_callback_backup.single_ch_callback( vib_reg_fail_buf ,1 ,true);
			start_enable_flag = 0;
            spi_power_off(  );			
			return 0;
		}
		usleep(2000000); //用于上层时域波形不丢波形数据，在这里直接延时2S,让硬 件预热稳定        
	}	   
    
	usleep(100000);//用于stop 和start DMA,寄存器稳定
    common_start( );

	start_num++;
	sem_init( &run_sem ,  0 ,  0 );
	
	pthread_create( &c_id ,  NULL ,  time_wave_thread , ( void* )&tWave );
     	
	signal( SIGIO ,  read_vibrate_data ); 

	while( thread_finished_flag == 0); // 用于等待算法线程线束时置1，若线程结束时，继续往下运行
	can_start_flag = 0;
	thread_finished_flag = 0;
		
	power_off_flag = 0;	
	start_enable_flag = 0;
	
	g_smpLength = 0;
	LOGD( "xin: 响应时域点击结束 [%s]\n" ,  log_time( ) ); 
	
	usleep( 100000 ); //用于底层虚拟地址和物理地址映射出错		
	return 0;
}

static int start_vibrate_CH_totalrend( struct spictl_device_t* dev ,  int ch_num , struct total_rend_para tRend  )//总值趋势
{
    LOGD("xin: 响应总值点击开始start_num = %d",start_num);
    if(start_num != 0)
    {
	   return 0; 
    }
	memset(read_60K_buf,0,SIZE_60K*sizeof(unsigned char));
	memset(g_max_char_buf ,0, MAX_SIZE*sizeof(unsigned char));
	
    usleep( tRend.interval_time*1000*1000 ); //下次采集间隔时间  微秒为单位 
	
    g_chNum = ch_num;	
	stop_smp_flag = 0;	
	g_smpLength = 0;	
    test_mode = tRend.version_mode;
    
    r_discard_pnts =  Get_InvalidNum((int)tRend.max_freq , (int)tRend.min_freq );
    r_max_freq = (int)tRend.max_freq;
    r_min_freq = (int)tRend.min_freq;
    r_wave_length = tRend.wave_length;
    
	LOGD("xin: 点击总值开始时 start_enable_flag = %d, restart_power_on_flag= %d, can_start_flag = %d , thread_finished_flag = %d", start_enable_flag,restart_power_on_flag,can_start_flag,thread_finished_flag);
	LOGD( "xin: start_vibrate_CH_totalrend_ch_num = %d , data_type = %d , signal_type = %d ,  min_freq = %d , max_freq = %d , wave_length = %d ,interval_time = %f ,test_mode = %d", ch_num ,  tRend.data_type ,  tRend.signal_type ,  (int)tRend.min_freq ,  (int)tRend.max_freq  , tRend.wave_length, tRend.interval_time,test_mode);	
    
	if(start_enable_flag == 1)
	{
		LOGD("xin: start_enable_flag = %d, 不响应start接口",start_enable_flag);		
        return 0;
	}else{
	    start_enable_flag = 1;
	} 
	if (can_start_flag == 1)
	    return 0;
	
	if( !is_right_length(tRend.wave_length))
	{
		start_enable_flag = 0; //用于波形长度为0时，防止再手动停止时再采集时 ，start_enable_flag = 0， 直接 回调false给上层，所以此处要置0
		return 0;
	}
	
	if(restart_power_on_flag == 1) // 当内部stop DMA，fpga,关闭设备后，flag 置1， 当再启动start接口时不再重新上电，当对外大的停止采集下电后，此flag会置为 0，重新采集时再上电
	{
		;
	}else{
		poweron_spi(  );	
        int reg_ret_value =0;
		if( g_chNum ==SINGLE_CH )	
			reg_ret_value = set_singleCH_vibrate_reg( tRend.signal_type , (int)tRend.max_freq , (int)tRend.min_freq);//设置单通道采集寄存器
		if( g_chNum == DOUBLE_CH ) 
			reg_ret_value = set_doubleCH_vibrate_reg( tRend.signal_type , (int)tRend.max_freq , (int)tRend.min_freq );//设置双通道采集寄存器
				
		if( reg_ret_value == -1)
		{
			LOGD("xin: 寄存器配置失败");
			vibrate_callback_backup.single_ch_callback( vib_reg_fail_buf ,1 ,true);
			start_enable_flag = 0;
            spi_power_off(  );			
			return 0;
		}
		usleep(2000000); //用于上层时域波形不丢波形数据，在这里直接延时2S,让硬 件预热稳定
	}
    
	usleep(100000);//用于stop 和start DMA,寄存器稳定
	common_start( );
	start_num ++;
	sem_init( &run_sem ,  0 ,  0 );
		
	pthread_create( &c_id ,  NULL ,  total_rend_thread , ( void* )&tRend ); 
    
	signal( SIGIO ,  read_vibrate_data ); 
    
	while( thread_finished_flag== 0); // 用于等待算法线程线束时置1，若线程结束时，继续往下运行
	can_start_flag = 0;
	thread_finished_flag = 0;
	
	power_off_flag = 0;
	start_enable_flag = 0;

	g_smpLength = 0;
	LOGD( "xin: 响应时域点击结束 [%s]\n" ,  log_time( ) ); 
	
	usleep( 100000 ); //用于底层虚拟地址和物理地址映射出错	
	return 0;
}

/* 振动等级评估 下限10hz  上限1000Hz  波长4k */
static int start_vibrate_evalute_level( struct spictl_device_t* dev , struct time_wave_para tWave )//振动等级评估
{	
    LOGD("xin: 响应振动等级评估开始start_num = %d",start_num);
    if(start_num != 0)
    {
	   return 0; 
    }
    memset(read_60K_buf,0,SIZE_60K*sizeof(unsigned char));
    memset(g_max_char_buf ,0, MAX_SIZE*sizeof(unsigned char));
	
	stop_smp_flag = 0;	
	g_smpLength = 0;	
	test_mode = tWave.version_mode;
	
    e_discard_pnts = Get_InvalidNum((int)tWave.max_freq , (int)tWave.min_freq ); 
    e_max_freq = (int)tWave.max_freq;
    e_min_freq = (int)tWave.min_freq;
    e_wave_length = tWave.wave_length;
    
	LOGD( "\nxin: 响应振动等级评估开始signal_type = %d ,  min_freq = %d , max_freq = %d , wave_length = %d ,test_mode = %d",	 tWave.signal_type ,  (int)tWave.min_freq ,  (int)tWave.max_freq  , tWave.wave_length, test_mode);	
	 
	if( !is_right_length(tWave.wave_length))
	{
		LOGD("xin: 下发的波形长度 tWave.wave_length = %d",tWave.wave_length);
		return 0;
	}
	
	poweron_spi(  );		
	int reg_ret_value = set_singleCH_vibrate_reg( tWave.signal_type , (int)tWave.max_freq , (int)tWave.min_freq );//设置单通道采集寄存器，数据类型为 速度时域波形
	if( reg_ret_value == -1)
	{
		LOGD("xin: 寄存器配置失败");
		vibrate_callback_backup.single_ch_callback( vib_reg_fail_buf ,1 ,true);
		start_enable_flag = 0;
		spi_power_off(  );			
		return 0;
	}
	
	usleep(2000000); //用于上层时域波形不丢波形数据，在这里直接延时2S,让硬 件预热稳定
    common_start( );

	start_num ++;
	sem_init( &run_sem ,  0 ,  0 );	
			
	pthread_create( &c_id ,  NULL ,  evalute_level_thread , ( void* )&tWave );	
	
	signal( SIGIO ,  read_evalute_data ); 				
	  		 
	//LOGD( "xin: 响应振动等级评估结束 [%s]\n" ,  log_time( ) ); 
	return 0;
}

/* 振动校准,上限40KHZ, 下限分DC耦合5HZ, AC耦合10HZ, 加速度，速度，位移都要校准*/
static int start_vibrate_calibration( struct spictl_device_t* dev ,struct time_wave_para tWave)//振动校准
{	
	LOGD("xin: 响应振动校准开始start_num = %d",start_num);
    
    stop_smp_flag = 0;	
	g_smpLength = 0;	
	test_mode = tWave.version_mode;
    
    v_discard_pnts = Get_InvalidNum((int)tWave.max_freq , (int)tWave.min_freq ); 
    v_max_freq = (int)tWave.max_freq;
    v_min_freq = (int)tWave.min_freq;
    v_wave_length = tWave.wave_length;
    
	LOGD("\nxin: 响应振动校准开始signal_type= %d ,min_freq = %d ,max_freq = %d ,wave_length = %d ",tWave.signal_type,(int)tWave.min_freq,(int)tWave.max_freq,tWave.wave_length);	

	poweron_spi( );	  
    set_singleCH_vibrate_reg( tWave.signal_type , (int)tWave.max_freq , (int)tWave.min_freq);//设置单通道采集寄存器，数据类型为 速度时域波形
       	
	usleep(2000000); //用于上层时域波形不丢波形数据，在这里直接延时2S,让硬件预热稳定
	common_start( );
	sem_init( &run_sem ,0 ,0 );		

	pthread_create( &c_id, NULL, vibrate_calib_thread, ( void* )&tWave );	
	
	signal( SIGIO, read_calibration_data ); 				
	  		 
    
	LOGD( "xin: 响应振动校准结束 [%s]\n" , log_time( ) ); 
	return 0;
}



/* 获取15个特征值 */
static float* spi_get_feature_value( struct spictl_device_t* dev , float pData[] , int data_len )
{	
	memset( feature_ret_value , 0 , FEATURE_NUM*sizeof( float ) ); //每次进来先初始化0
	feature_value( pData ,  data_len ,  feature_ret_value ); // 求15个特征值
	
	/* #if 0
	int i = 0;
	for( i=0;i<FEATURE_NUM;i++ )
	{
		LOGD( "feature_ret_value[%d] = %f" , i , feature_ret_value[i] ); //直接转换为加速度值
	} 
	#endif 	 */
	
	return feature_ret_value;	
}

/* 时域到频域接口 */
static float* spi_time_to_freq_value( struct spictl_device_t* dev , float pData[] , int data_len  )
{
	fft_alg_entry2( pData ,  data_len , 0 , 0 , 0 );	//0默认不加窗，不平均，平均方式不进行
	
	/* #if 0
	int i =0;
	for(i = 0; i< data_len/2; i++)
	{
		 LOGD( "ret_pData[%d] = %f" , i, pData[i] );	
	}
	#endif  */
	
	return pData;    		
}


/////////频率采集功能如下 
int f_discard_pnts = 0,f_max_freq = 0,f_min_freq = 0,f_wave_length = 0;
void *freq_wave_thread( void* arg ) //频域线程
{	
    freqwave my_freqwave ;
	my_freqwave = *( struct freq_wave_para* )arg;		
	int i=0 , j=0 , k=0;	
	float f_CH_data[3]={0.0};
	
    
	if( g_chNum == SINGLE_CH ) 
	{
		g_smpLength = f_wave_length + f_discard_pnts;	//实际采集的波形长度				
				
		LOGD( "xin: freq_wave_thread_SINGLE_CH_f_discard_pnts = %d ,  spectra_num*2.56 = %d ,  g_smpLength = %d"  , f_discard_pnts ,  f_wave_length ,  g_smpLength );	
		if( f_wave_length < 0 )
		{
			return NULL;
		}
		
		float *freq_CH1_smp_buf =NULL; //通道1数据  采集数据
		if( freq_CH1_smp_buf == NULL)
		{
			freq_CH1_smp_buf =( float* )malloc( g_smpLength*sizeof( float ) );
			if( freq_CH1_smp_buf == NULL )
			{
				LOGD( "freq_CH1_smp_buf 分配内存失败！" );
				exit( EXIT_FAILURE );		
			}
			memset( freq_CH1_smp_buf , 0 , g_smpLength*sizeof( float ) );	
		}
		
		float *freq_CH1_IIR_buf =NULL; //通道1 IIR滤波后数据
		if( freq_CH1_IIR_buf == NULL)
		{
			freq_CH1_IIR_buf =( float* )malloc( f_wave_length*sizeof( float ) );
			if( freq_CH1_IIR_buf == NULL )
			{
				LOGD( "freq_CH1_IIR_buf 分配内存失败！" );
				exit( EXIT_FAILURE );		
			}
			memset( freq_CH1_IIR_buf , 0 , f_wave_length*sizeof( float ) );
		}
		
		
		while(  1  )
		{		
			sem_wait( &run_sem );//等待信号量		
			          		
			if( exit_thread_flag )
			{
				#if 1
				if( freq_CH1_smp_buf != NULL)
				{
					free( freq_CH1_smp_buf ); 
					freq_CH1_smp_buf =NULL;	
				}		
                if( freq_CH1_IIR_buf != NULL)
				{					
					free( freq_CH1_IIR_buf ); 
					freq_CH1_IIR_buf =NULL;	
				}				
				#endif
				LOGD( "xin:exit_freq_wave_thread_SINGLE_CH=====" );
				
				return NULL;
			}

			for( i=0;i< g_smpLength;i++ )			
			{					
				analyze_CH_data( g_smp_buf[i] , f_CH_data );  //调用解析通道数据的函数								
				freq_CH1_smp_buf[i] = f_CH_data[2];		 							
			} 
						
			enter_IIR_Filter( freq_CH1_smp_buf , g_smpLength ,( int )my_freqwave.max_freq ,( int )my_freqwave.min_freq ); //IIR 高通滤波，长度是采样长度
			
			memcpy( freq_CH1_IIR_buf , &freq_CH1_smp_buf[f_discard_pnts] , f_wave_length*sizeof( float ) );
			
			fft_alg_entry2( freq_CH1_IIR_buf , f_wave_length , 0 , 0 , 0  ); //0默认不加窗，平均方式，平均次数都为0，不进行
			vibrate_callback_backup.single_ch_callback( freq_CH1_IIR_buf ,  f_wave_length/2 ,  true );	/////频谱回调 长度是实际UI长度一半								    
			stop_smp_flag = 0;		
		}		
	}
		
	if( g_chNum == DOUBLE_CH ) //双通道内存为单通道时的两倍
	{	       
	    f_wave_length = f_wave_length*2; //波形长度 = 频谱线数*2.56
		g_smpLength = f_wave_length + f_discard_pnts *2;	//实际采集的波形长度        
		LOGD( "xin:freq_wave_thread_DOUBLE_CH_f_discard_pnts = %d ,  spectra_num*2.56 = %d ,  g_smpLength = %d"  , f_discard_pnts ,  f_wave_length/2 ,  g_smpLength );
		if( f_wave_length < 0 )
		{
			return NULL;
		}
		
        float *freq_CH1_smp_buf =NULL; //通道1数据  采集数据
		if( freq_CH1_smp_buf == NULL)
		{
			freq_CH1_smp_buf =( float* )malloc( g_smpLength*sizeof( float ) );
			if( freq_CH1_smp_buf == NULL )
			{
				LOGD( "freq_CH1_smp_buf 分配内存失败！" );
				exit( EXIT_FAILURE );		
			}
			memset( freq_CH1_smp_buf , 0 , g_smpLength*sizeof( float ) );
		}		
		
		float *freq_CH2_smp_buf =NULL; //通道2数据 采集数据
		if( freq_CH2_smp_buf == NULL)
		{
			freq_CH2_smp_buf =( float* )malloc( g_smpLength*sizeof( float ) );
			if( freq_CH2_smp_buf == NULL )
			{
				LOGD( "freq_CH2_smp_buf 分配内存失败！" );
				exit( EXIT_FAILURE );		
			}
			memset( freq_CH2_smp_buf , 0 , g_smpLength*sizeof( float ) );
		}

        float *freq_CH1_smp_buf1 =NULL; //通道1数据  采集真实数据
		if( freq_CH1_smp_buf1 == NULL)
		{
			freq_CH1_smp_buf1 =( float* )malloc(( g_smpLength/2 )*sizeof( float ) );
			if( freq_CH1_smp_buf1 == NULL )
			{
				LOGD( "freq_CH1_smp_buf1 分配内存失败！" );
				exit( EXIT_FAILURE );		
			}
			memset( freq_CH1_smp_buf1 , 0 ,( g_smpLength/2 )*sizeof( float ) );	
		}		
		
		float *freq_CH2_smp_buf2 =NULL; //通道2数据 采集真实数据
		if( freq_CH2_smp_buf2 == NULL)
		{
			freq_CH2_smp_buf2 =( float* )malloc(( g_smpLength/2 )*sizeof( float ) );
			if( freq_CH2_smp_buf2 == NULL )
			{
				LOGD( "freq_CH2_smp_buf2 分配内存失败！" );
				exit( EXIT_FAILURE );		
			}
			memset( freq_CH2_smp_buf2 , 0 ,( g_smpLength/2 )*sizeof( float ) );
		}
		
		
		float *freq_CH1_IIR_buf =NULL; //通道1 IIR返回后的数据
		if( freq_CH1_IIR_buf == NULL)
		{
			freq_CH1_IIR_buf =( float* )malloc(( f_wave_length/2 )*sizeof( float ) );
			if( freq_CH1_IIR_buf == NULL )
			{
				LOGD( "freq_CH1_IIR_buf 分配内存失败！" );
				exit( EXIT_FAILURE );		
			}
			memset( freq_CH1_IIR_buf , 0 ,( f_wave_length/2 )*sizeof( float ) );	
		}
		
		float *freq_CH2_IIR_buf =NULL; //通道2 IIR返回后的数据
		if( freq_CH2_IIR_buf == NULL)
		{
			freq_CH2_IIR_buf =( float* )malloc(( f_wave_length/2 )*sizeof( float ) );
			if( freq_CH2_IIR_buf == NULL )
			{
				LOGD( "freq_CH2_IIR_buf 分配内存失败！" );
				exit( EXIT_FAILURE );		
			}
			memset( freq_CH2_IIR_buf , 0 ,( f_wave_length/2 )*sizeof( float ) );		
		}		
		
		while( 1 )
		{		
			sem_wait( &run_sem );//等待信号量			
						
			if( exit_thread_flag )
			{
				#if 1
				if( freq_CH1_smp_buf !=NULL )
				{
					free( freq_CH1_smp_buf ); 
					freq_CH1_smp_buf =NULL;
				}
				if( freq_CH2_smp_buf !=NULL)
				{
					free( freq_CH2_smp_buf ); 
					freq_CH2_smp_buf =NULL;	
				}
				if( freq_CH1_IIR_buf != NULL)
				{
					free( freq_CH1_IIR_buf ); 
					freq_CH1_IIR_buf =NULL;	
				}
				if( freq_CH2_IIR_buf != NULL)
				{
					free( freq_CH2_IIR_buf ); 
					freq_CH2_IIR_buf =NULL; 
				}	
                if( freq_CH1_smp_buf1 != NULL)
				{					
					free( freq_CH1_smp_buf1 ); 
					freq_CH1_smp_buf1 =NULL;
				}
				if( freq_CH2_smp_buf2 != NULL)
				{
					free( freq_CH2_smp_buf2 ); 
					freq_CH2_smp_buf2 =NULL;	
				}
                #endif								
				LOGD( "xin: exit_freq_wave_thread_DOUBLE_CH=====" );
				
				return NULL;
			}
	
			for( i=0;i< g_smpLength;i++ )			
			{					
				analyze_CH_data( g_smp_buf[i] , f_CH_data );  //调用解析通道数据的函数	
				freq_CH1_smp_buf[i] = f_CH_data[1];
				freq_CH2_smp_buf[i] = f_CH_data[2];				
			}			
			for( k=0;k<g_smpLength/2;k++ )
			{					
				freq_CH1_smp_buf1[k] = freq_CH1_smp_buf[k*2+1]; //通道1read_evalute_data
				freq_CH2_smp_buf2[k] = freq_CH2_smp_buf[k*2];//通道2					
			}
					
			enter_IIR_Filter( freq_CH1_smp_buf1 , g_smpLength/2 ,( int )my_freqwave.max_freq ,( int )my_freqwave.min_freq );  //IIR 滤波长度为 采集长度的一半
			enter_IIR_Filter( freq_CH2_smp_buf2 , g_smpLength/2 ,( int )my_freqwave.max_freq ,( int )my_freqwave.min_freq );
         							
            memcpy( freq_CH1_IIR_buf , &freq_CH1_smp_buf1[f_discard_pnts] ,( f_wave_length/2 )*sizeof( float ) );
            memcpy( freq_CH2_IIR_buf , &freq_CH2_smp_buf2[f_discard_pnts] ,( f_wave_length/2 )*sizeof( float ) );			
						
			fft_alg_entry2( freq_CH1_IIR_buf , f_wave_length/2 , 0 , 0 , 0  );//0默认不加窗，平均方式，平均次数都为0，不进行			
			fft_alg_entry2( freq_CH2_IIR_buf , f_wave_length/2 , 0 , 0 , 0  );	//0默认不加窗，平均方式，平均次数都为0，不进行		
			
			vibrate_callback_backup.double_ch_callback( freq_CH1_IIR_buf ,  freq_CH2_IIR_buf ,  f_wave_length/4 ,  true );	/////频谱只回调 前一半数据							
			stop_smp_flag = 0;		
		}		
	}      		
	return NULL;
}

static int start_vibrate_CH_freqwave( struct spictl_device_t* dev , int ch_num , struct freq_wave_para fWave  )// 启动频域波形
{
	g_chNum = ch_num;	 
	LOGD( "xin: start_vibrate_CH_freqwave=====ch_num = %d , data_type = %d , signal_type = %d , min_freq = %f , max_freq = %f , spectra_num = %d , average_num = %d , average_mode = %d , window_type = %d , range_mode = %d , trig_mode = %d , range_accel_value = %d" , 													
	g_chNum ,  fWave.data_type ,  fWave.signal_type ,  fWave.min_freq ,  fWave.max_freq  , fWave.spectra_num ,  fWave.average_num ,  fWave.average_mode ,  fWave.window_type , fWave.range_mode , fWave.trig_mode , fWave.range_accel_value );
	 
	if( g_chNum == SINGLE_CH )	
		set_singleCH_vibrate_reg( fWave.signal_type , (int)fWave.max_freq , (int)fWave.min_freq);//设置单通道采集寄存器	
	if( g_chNum == DOUBLE_CH ) 
		set_doubleCH_vibrate_reg( fWave.signal_type , (int)fWave.max_freq , (int)fWave.min_freq );//设置双通道采集寄存器	

	sem_init( &run_sem ,  0 ,  0 );
   	
    exit_thread_flag = false;	
	stop_smp_flag = 0;	
	
	f_discard_pnts = Get_InvalidNum((int)fWave.max_freq , (int)fWave.min_freq ); 
    f_max_freq = (int)fWave.max_freq;
    f_min_freq = (int)fWave.min_freq;
    f_wave_length = fWave.spectra_num*2.56;//波形长度 = 频谱线数*2.56	
        
	pthread_create( &c_id ,  NULL ,  freq_wave_thread , ( void* )&fWave ); 
    LOGD( "start_vibrate_CH_freqwave time: [%s]\n" ,  log_time( ) ); 	
	signal( SIGIO ,  read_vibrate_data ); 
			 	
    common_start( );
	return 0;
}


////////////转速采集功能如下
void *rotation_CH_thread( void* arg ) //转速通道线程
{
	LOGD( "rotation_CH_thread===========" );	
    int i=0;  
	float CH_data[3]={0.0};	
	
    float rotation_array[2] ={0.0}; //转速数组
	while( 1 )
	{
		sem_wait( &run_sem );//等待信号量
		if( exit_thread_flag )
		{
			return NULL;
		}
        for( i=0;i< 100;i++ )
		{		
			analyze_CH_data( g_smp_buf[i] , CH_data );  //调用解析通道数据的函数							
			rotation_array[0] = CH_data[0];
			LOGD( "rotation_array[%d] = %f" , i , rotation_array[i] );
		}
		stop_smp_flag = 0;
	}	
	return NULL;
}

static int start_rotation_CH( struct spictl_device_t* dev )//启动转速通道采集
{
	LOGD( "start_rotation_CH" );	
	set_rotation_reg( );	//设置转速寄存器
	sem_init( &run_sem ,  0 ,  0 );	
	
	exit_thread_flag = false;      
	stop_smp_flag = 0;	
	
	pthread_create( &c_id ,  NULL ,  rotation_CH_thread ,  NULL );
	signal( SIGIO ,  read_vibrate_data );
    common_start( );
	return 0;
}


