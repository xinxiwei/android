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
#include <hardware/imx6q_spi_pressure.h>
#include <hardware/imx6q_spi_vibrate.h>
#include <hardware/imx6q_iir_filter.h>
#include <hardware/imx6q_fir_filter.h>

#define DEVICE_NAME "/dev/mxc_spidev1"
#define MODULE_NAME "xxxx"
#define MODULE_AUTHOR "gxl@126.com"

/////////////////////////////////////////////////
#define debug      0
#define SIZE_15360  15360
#define SIZE_60K   61440
#define SIZE_64K   65536
#define SIZE_4K    4096
#define SIZE_16K   16384

#define EPSILON    0.000001 
#define FEATURE_NUM    15  //特征值个数

int smp_rate = 102400;//采样率HZ
int oflags = 0;
int fd = -1; //设备文件
int run_sig_res = 0; //内部信号初始化返回值
int *psample_buf  = NULL; //最终采样的原始数据
int *g_smp_buf = NULL;  //全局采集的数据
float invalid_buf[2] = {0.0}; // power off时回调的假buf,供接口使用，实际无意义
float reg_fail_buf[1] = {10001}; // 当寄存器配置失败时，供上面app用

int g_loop_num = 0; //底层反馈的第几波数据
int g_ret_val = 0;  //设备文件关闭返回值

unsigned char read_60K_buf[SIZE_60K] = {0};// 读压力数据buf
unsigned char backup_60K_buf[SIZE_60K] = {0};  //60K
unsigned char press_buf[SIZE_64K] = {0};//采集到的数据

float feature_ret_value[FEATURE_NUM] = {0.0}; //15个特征值

volatile bool thread_finished_flag = 0;
volatile bool post_flag = true;  // 发送post 信号量标识
volatile bool exit_thread_flag = false; //退出线程标志
volatile int  stop_smp_flag = 0;//停止采集数据 标识
volatile int  restart_power_on_flag = 0;  // spi 重新上电标识
volatile int  power_off_flag = 0; //
volatile int  can_start_flag  = 0; //用于快速点击时，dma没有执行完
volatile int  start_enable_flag = 0;// 响应start接口标识

volatile float pflag0_value = 0.0; //压力标0 模式值

pthread_t  c_id; // 开辟线程 c_id:计算
sem_t   run_sem; //内部信号名  run_sem: 继续运行信号


//设备打开和关闭接口
static int spi_device_open( const struct hw_module_t* module , const char* name , struct hw_device_t** device );
static int spi_device_close( struct hw_device_t* device );

//设备访问接口
static int start_pressure_dial( struct spictl_device_t* dev , float data ); //表盘模式
static int start_pressure_curve( struct spictl_device_t* dev , float data ); //曲线模式
static int start_pressure_flag0( struct spictl_device_t* dev ); //标0模式

static int start_vibrate_CH_timewave( struct spictl_device_t* dev ,   int ch_num , struct time_wave_para tWave );//启动振动采集 时域波形
static int start_vibrate_CH_freqwave( struct spictl_device_t* dev ,   int ch_num , struct freq_wave_para fWave );//启动振动采集 频域波形
static int start_vibrate_CH_totalrend( struct spictl_device_t* dev ,   int ch_num , struct total_rend_para tRend );//启动振动采集 总值趋势波形
static int start_rotation_CH( struct spictl_device_t* dev );//转速通道
static int start_vibrate_evalute_level( struct spictl_device_t* dev , struct time_wave_para tWave );//启动振动等级评估

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
	LOGD("imx6q_spi_default.so-version  = 20170831"); //.so 版本信息
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
	
	dev->start_vibrate_timewave = start_vibrate_CH_timewave;  //时域
	dev->start_vibrate_freqwave = start_vibrate_CH_freqwave;  //频域
	dev->start_vibrate_totalrend = start_vibrate_CH_totalrend;	//总值趋势
	dev->start_rotation = start_rotation_CH; //转速
	dev->start_vibrate_evalute = start_vibrate_evalute_level; //等级评估
	
	dev->stop_vibrate_ad = stop_vibrate_sample; //振动停止采集
	dev->stop_press_ad = stop_press_sample; //压力停止采集
	
	dev->spi_freq = spi_get_freq;	
	dev->spi_feature_value = spi_get_feature_value;	 //特征值
	dev->spi_timetofreq_value = spi_time_to_freq_value;	//时域到频域接口
	
    dev->get_pressure_interface = spi_press_interface;   //压力回调接口
	dev->get_vibrate_interface = spi_vibrate_interface; //振动回调接口
	
    *device =&( dev->common );
    //LOGD( "xin: imx6q_spi: open /dev/mxc_spidev1 successfully." );
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

static int spi_get_freq( struct spictl_device_t* dev ){//获取采样率    
	return spi_freq( );
}


static int wdma_num =0; //重新使能DMA次数
///////////////////共用函数如下
void common_start( )//打开设备和开始fpga采集
{
	//LOGD("xin: common_start");
	int res;    
    if(( fd = open( DEVICE_NAME , O_RDWR ) ) == -1 ){
        LOGD( "imx6q_spi: failed to open /dev/mxc_spidev1 -- %s." , strerror( errno ) );        		
    }else{
		LOGD( "imx6q_spi: open /dev/mxc_spidev1 success"); 
	}	
	
    fcntl( fd ,  F_SETOWN ,  getpid( ) );
    oflags = fcntl( fd ,  F_GETFL );
    fcntl( fd ,  F_SETFL ,  oflags | FASYNC );
    res = ioctl( fd ,  SPIDEV_IOC_RXSTREAMON ,  NULL );//启动DMA采集		
	
    if( res !=0 ) {		
        LOGD( "imx6q_spi: can't set spi mode" );
		while((res != 0) || ( wdma_num < 100))
		{
			usleep(100000); //让DMA稳定，给一定延时申请通道
			res = ioctl( fd ,  SPIDEV_IOC_RXSTREAMON ,  NULL );
			
			wdma_num ++;
		}
    }
	if( wdma_num >= 100)
	{
		LOGD("xin: wdma_num >=100");
		wdma_num = 0;
		vibrate_callback_backup.single_ch_callback( reg_fail_buf ,1 ,true);
		start_enable_flag = 0;
		spi_poweroff(  );
	}
	
	wdma_num = 0;
    usleep(50000); ///让DMA稳定，给一定延时申请通道
	LOGD("StartSampleAddr = %d, StartSampleData = %d",StartSampleAddr,StartSampleData);
	swrite( StartSampleAddr , StartSampleData );//start fpga	
} 

void spi_power_off() //spi设备下电
{	   
	spi_poweroff( );  //下电	 	
    LOGD("xin: 设备下电结束");	
}

void stop_dma()//停止FPGA采集, DMA搬运，关闭设备文件
{	
	LOGD( "xin: DMA结束开始 power_off_flag = %d, restart_power_on_flag = %d,is : [%s]\n" ,power_off_flag, restart_power_on_flag,  log_time( ) );   
	
	if(  power_off_flag == 1) //下电标识为1时，开始下电
	{
		spi_power_off(); 
		LOGD("xin: 下电时power_off_flag = %d",power_off_flag);
	}
	int res = -1;	
	
	stop_smp_flag = 0;	
	
	swrite( StopSampleAddr , StopSampleData );//stop fpga
	
	res = ioctl( fd ,  SPIDEV_IOC_RXSTREAMOFF ,  NULL );  //停止DMA采集 
	LOGD("xin: 停止FPGA采集，DMA采集");	
	if(  power_off_flag == 1) // 下电标识
	{	
		restart_power_on_flag = 0;	// 重新上电标识
	}else{
		restart_power_on_flag = 1;	 
	}
		
	g_ret_val = close( fd ); //关闭设备文件	
	if(g_ret_val == 0)
	{		
		fd = -1;
		LOGD("xin: 关闭设备文件成功");
	}
	
	LOGD( "xin: DMA结束完成 power_off_flag = %d, restart_power_on_flag = %d,is : [%s]\n" ,power_off_flag, restart_power_on_flag,  log_time( ) );    	
}

static int stop_press_sample( struct spictl_device_t* dev )//停止压力采集
{	
	LOGD( "xin: 停止采集开始 : [%s]\n" ,  log_time( ) );	
	int res = -1;	
	int error_value = 0;
	
	swrite( StopSampleAddr , StopSampleData );//停止FPGA采集
    res = ioctl( fd ,  SPIDEV_IOC_RXSTREAMOFF ,  NULL ); //停止DMA采集
    if( res == -1 )
    {
        LOGD( "imx6q_spi:SpiSlaveModeOff false." );
    }
	
	exit_thread_flag = true;
	stop_smp_flag = 0;
	sem_post( &run_sem );
	sleep( 2 ); 
	
	g_ret_val=close( fd );
					
	spi_poweroff( );  //下电	  
	sem_destroy( &run_sem );	
	pthread_detach( c_id ); 
	
	if( g_ret_val == 0 )
	{
		LOGD( "xin: stop_press_sample====== success" );
		//pressure_callback_backup.stop_press_callback( true );		
	}else if( g_ret_val < 0 )
	{
		LOGD( "xin: stop_press_sample====== fail" );
		//pressure_callback_backup.stop_press_callback( false );			
	}
    LOGD( "xin: 停止采集结束 : [%s]\n" ,  log_time( ) );
	return 0;
}

static int stop_vibrate_sample( struct spictl_device_t* dev )//停止振动采集
{   
    LOGD( "xin: 停止振动采集开始 power_off_flag = %d, restart_power_on_flag = %d,start_enable_flag =%d is : [%s]\n" , power_off_flag, restart_power_on_flag, start_enable_flag, log_time( ) );
	if(start_enable_flag == 0)
	{	
		vibrate_callback_backup.stop_ch_callback( false); 
		LOGD("xin: 此时start线程刚结束，无线程存在,回调false给上层");
		
        start_enable_flag = 0;	
		power_off_flag = 0;
		restart_power_on_flag = 0;
		can_start_flag = 0;
		
        return 0;		
	}
	
	start_enable_flag = 0;
    power_off_flag = 1;		
	restart_power_on_flag = 0; //防止内部start 响应时重新上电，当为0时，再上电
	
	
	LOGD( "xin: 停止振动采集结束 power_off_flag = %d, restart_power_on_flag = %d,start_enable_flag =%d is : [%s]\n" , power_off_flag, restart_power_on_flag, start_enable_flag, log_time( ) );
	return 0;
}



///////////////////adc数据提取算法
inline float adc_data_to_24value( int adc_data )// 将采集32位数据 转换为24位有效数据
{
    bool highbit_flag = false;
    float value = 0.0;
    adc_data &= 0x00ffffff;//24位数据
    highbit_flag =( bool )(( adc_data>>23 )&0x1 ); //最高位符号位
    adc_data &= 0x007fffff;
    if( highbit_flag )		
    {	       
        value =(float)(( adc_data-0x800000 )*2.5 )/0x7fffff;//分段函数
    }
    else
    {	    
        value =(float)( adc_data*2.5 )/0x7fffff;
    }		
	return value;
}

void analyze_CH_data( int adc_data ,  float *value )//解析出各通道的数据
{		
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

void analyze_single_data(int *src, int length, float *value1)
{
	int i =0;
	for(i =0;i <length; i++)
	{
		int higetst_bit =( src[i] >> 31 )&0x1;
		int channel_bit =( src[i] >> 30 )&0x1;
		if( higetst_bit == 0x1 ) //1表示转速数据
		{
			;
		}
		else //振动
		{
			if( channel_bit == 0 ) //0表示 通道1 数据
			{				
				value1[i] = adc_data_to_24value( src[i] );                				
			}							
		}
	}
}

void analyze_double_data(int *src, int length,float *value1,float *value2)
{	    	
	int i =0;    
	for(i =0;i < length; i++)
	{		
		int higetst_bit =( src[i] >> 31 )&0x1;
		int channel_bit =( src[i] >> 30 )&0x1;
		if( higetst_bit == 0x1 ) //1表示转速数据
		{
			;
		}
		else //振动
		{			
			if( channel_bit == 0 ) //0表示 通道1 数据
			{									
				value1[i] = adc_data_to_24value( src[i] );				 						
			}else if( channel_bit == 1 ) // 1表示 通道2 数据
			{									
				value2[i] = adc_data_to_24value( src[i] );                					
			}						
		}
	}
}



///////////////////压力采集功能如下
void read_press_data( ) // 读取压力adc 采样数据
{	
    LOGD( "read_press_data  ,  time is : [%s]\n" ,  log_time( ) );	
	if(  read( fd , read_60K_buf , SIZE_60K ) < 0  )
	{
        LOGD( "Error: spi slave device read fail !\n " );
    }
    memcpy( &backup_60K_buf , &read_60K_buf ,  SIZE_60K ); //60K	
	
	if( g_loop_num == 0 )
    {
        memcpy( &press_buf ,  &backup_60K_buf ,  SIZE_60K );	
		g_loop_num ++;	
    }else if( g_loop_num == 1 )
    {
        memcpy( &press_buf[SIZE_60K] ,  &backup_60K_buf ,  SIZE_4K ); //SIZE_4K= 4096
        psample_buf =( int* )&( press_buf ) ;        	
        sem_post( &run_sem );		
        g_loop_num = 0;
    }	
}

void *press_flag0_thread( void* arg ) //压力标0模式
{
	LOGD( "press_flag0_thread===========" );
    int i=0;      	
    float press_flag0_value[2] ={0.0}; //标0模式时，回传的数据 ,  0位：表示数据值，1位：表示有效数据个数
    float f64_sum = 0.0; //64K数据总和
	
	sem_wait( &run_sem );//等待信号量	
	f64_sum = 0.0;	
	
	for( i=0;i<SIZE_16K;i++ )
	{		        
		f64_sum += adc_data_to_24value( psample_buf[i] );		
	}	
	press_flag0_value[0] = f64_sum / SIZE_16K;	//总各求平均值
	press_flag0_value[1] = 1; 
	
	LOGD( "xin: press_flag0_value =%f" , press_flag0_value[0] );		
	pressure_callback_backup.mspictl_callback( press_flag0_value , true );//调用JNI 回调方法，向上层传送数据		
    return NULL;
}

void *press_curve_thread( void* arg ) //压力曲线模式
{    
    LOGD( "press_curve_thread===========" );
	int i=0;
    float final_buf[SIZE_16K] ={0.0};   //SIZE_16K
	float positive_buf[SIZE_16K] ={0.0}; //压力数据全部转换为正数	
	float ret_value[SIZE_16K] ={0.0};//回调返回的数据
	float status_flag[2] ={0.0};	
	float value_bar_max2 = 0.0;   
    float value_bar_min2 = 0.0;	
	float tmp1=0.0 , tmp2=0.0 , tmp3=0.0;		
	LOGD( "press_curve_thread fabs( pflag0_value ) =%f\n" ,  fabs( pflag0_value ) ); 		
	while( 1 )
	{
		sem_wait( &run_sem );//等待信号量		
		if( exit_thread_flag )
		{
			LOGD( "xin: exit_press_curve_thread=====" );
			return NULL;
		}
		memset( final_buf , 0 , SIZE_16K*sizeof( float ) );
		memset( positive_buf , 0 , SIZE_16K*sizeof( float ) );
		memset( ret_value , 0 , SIZE_16K*sizeof( float ) );		
		
        for( i=0;i<SIZE_16K;i++ )
        {
            final_buf[i] = adc_data_to_24value( psample_buf[i] );	           		
        }
		#if debug
		for(  i=0;i<100;i++  )  ///test
        {          
            LOGD( "final_buf[%d] = %f" , i , final_buf[i] );			
        }
		#endif		
		
		
		for( i=0;i<SIZE_16K;i++ )
        {	
           	positive_buf[i] =( float )fabs( final_buf[i] );    //采集数据取绝对值       			
        }				
		value_bar_max2 = positive_buf[0]; 
		value_bar_min2 = positive_buf[0];			
		for( i=1;i<SIZE_16K;i++ )
		{
			if( value_bar_max2 <= positive_buf[i] )            
				value_bar_max2= positive_buf[i];  //对原始采集数据求最大值 ，用于后面0.5bar 判定条件，若最大值小于0.5bar , 则不进行后续算法计算，直接提示异常
			
			if( value_bar_min2 >= positive_buf[i] )            
				value_bar_min2= positive_buf[i];  //对原始采集数据求最小值 ，用于后面0.5bar 判定条件，若最大值小于0.5bar , 则不进行后续算法计算，直接提示异常		    
        }		
		//LOGD( "press_curve_thread value_bar_max2 =%f ,  value_bar_min2 =%f\n" ,  value_bar_max2 , value_bar_min2 );			
		
		tmp1 = fabs(  fabs( pflag0_value ) - value_bar_max2 );
		tmp2 = fabs(  fabs( pflag0_value ) - value_bar_min2 );		
	    tmp3 =( tmp1 > tmp2 )?tmp1:tmp2;	   	//求幅值变化最大值
		//LOGD( "tmp1 = %f , tmp2 = %f" , tmp1 , tmp2 );			
		
		if( tmp3  <= 0.01 )  //0.5bar = 0.01V  // 20mv/bar  这波数据  不符合要求
		{
			LOGD( "press_curve_thread is <0.5bar status" );
			status_flag[0] = 10000.0;
			status_flag[1] = 1;        			
			pressure_callback_backup.mspictl_callback( status_flag ,  false );//外部环境和气缸环境 相差很小，此时表示空采
		}
		else if( tmp3 >0.01 )	   
		{		
            LOGD( "press_curve_thread is >0.5bar status" );				
			press_alg_entry( final_buf , SIZE_16K ,  ret_value );		 //压力曲线模式算法	
			for( i=4; i< 1024;i++ )
			{
				LOGD("=====ret_value[%d] = %f",i,ret_value[i]);
			}
			pressure_callback_backup.mspictl_callback( ret_value ,  true );//调用JNI 回调方法，向上层传送数据	
        }		
		stop_smp_flag = 0;
	}
   return NULL;
 }
 
void *press_dial_thread( void* arg ) //压力表盘模式
{
	LOGD( "press_dial_thread===========" );
    int i=0;	
    float dial_value[2]={0.0};
	float status_flag[2]={0.0}; // 标0检测回传值
	float final_buf[SIZE_16K] ={0.0};   //SIZE_16K
	float positive_buf[SIZE_16K] ={0.0};    
    float f64_sum = 0.0;
	float value_bar_max1 = 0.0;   
    float value_bar_min1 = 0.0;	
	float tmp1=0.0 , tmp2=0.0 , tmp3=0.0;	
	LOGD( "press_dial_thread fabs( pflag0_value ) =%f\n" ,  fabs( pflag0_value ) ); 	
	while( 1 )
	{		
		sem_wait( &run_sem );//等待信号量		
		if( exit_thread_flag )
		{
			LOGD( "xin: exit_press_dial_thread=====" );
			return NULL;
		}		
        f64_sum = 0.0;			
		memset( final_buf , 0 , SIZE_16K*sizeof( float ) );
		memset( positive_buf , 0 , SIZE_16K*sizeof( float ) );		
		
		for( i=0;i<SIZE_16K;i++ )
        {			
             final_buf[i] = adc_data_to_24value( psample_buf[i] );	
			 f64_sum += final_buf[i];
	    }
		#if debug
		for( i=0;i<300;i++ )
        {	
	         LOGD( "press_dial_thread_final_buf[%d]= %f" , i , final_buf[i] );	
		}
		#endif	
		
		
		for( i=0;i<SIZE_16K;i++ )
        {	
            positive_buf[i] =( float )fabs( final_buf[i] );            		
        }
		
		value_bar_max1 = positive_buf[0]; 
		value_bar_min1 = positive_buf[0];		
		for( i=1;i<SIZE_16K;i++ )
		{
			if( value_bar_max1 <= positive_buf[i] )            
				value_bar_max1= positive_buf[i];  //对原始采集数据求最大值 ，用于后面0.5bar 判定条件，若最大值小于0.5bar , 则不进行后续算法计算，直接提示异常
			
			if( value_bar_min1 >= positive_buf[i] )            
				value_bar_min1= positive_buf[i];  //对原始采集数据求最小值 ，用于后面0.5bar 判定条件，若最大值小于0.5bar , 则不进行后续算法计算，直接提示异常		    
        }		
		//LOGD( "press_dial_thread value_bar_max1 =%f ,  value_bar_min1 =%f\n" ,  value_bar_max1 , value_bar_min1 );
	   	
		tmp1 = fabs(  fabs( pflag0_value ) - value_bar_max1 );
		tmp2 = fabs(  fabs( pflag0_value ) - value_bar_min1 );
		tmp3 =( tmp1 > tmp2 )?tmp1:tmp2;  
		//LOGD( "tmp1 = %f , tmp2 = %f" , tmp1 , tmp2 );	
		
		if( tmp3  <= 0.01 )  //0.5bar = 0.01V  //这波数据  不符合要求
		{
			LOGD( "press_dial_thread is <0.5bar status" );
		    status_flag[0] = 10000.0;
			status_flag[1] = 1;        			
			pressure_callback_backup.mspictl_callback( status_flag ,  false );//外部环境和气缸环境 相差很小，此时表示空采			
		}
		else if( tmp3 >0.01 )	   
		{	
            LOGD( "press_dial_thread is >0.5bar status" );		
			dial_value[0] = f64_sum / SIZE_16K;   //表盘模式算法是对所采数据求平均
			dial_value[1] = 1;        			
			LOGD( "xin: dial_value =%f\n" ,  dial_value[0] );
			pressure_callback_backup.mspictl_callback( dial_value ,  true );//调用JNI 回调方法，向上层传送数据			
        }						
		stop_smp_flag = 0;
	}
   return NULL;
}
 
static int start_pressure_flag0( struct spictl_device_t* dev )// 压力标0模式
{	    
    LOGD( "start_pressure_flag0=====" );		
    set_press_reg( smp_rate );	//设置压力采集寄存器   
	run_sig_res = sem_init( &run_sem ,  0 ,  0 );
	if( run_sig_res != 0 )
    {
        LOGD( "run_sig_res initialization failed" );
    }	
	exit_thread_flag = false;
	pthread_create( &c_id ,  NULL ,  press_flag0_thread ,  NULL );
	signal( SIGIO ,  read_press_data );// 异步IO事件	
    common_start( );
	return 0;
}
 
static int start_pressure_curve( struct spictl_device_t* dev , float flag0_value )  //压力曲线模式
{	   
    LOGD( "start_pressure_curve=====" );	
    set_press_reg( smp_rate );	//设置压力采集寄存器 
	run_sig_res = sem_init( &run_sem ,  0 ,  0 );   //初始化信号量
	if( run_sig_res != 0 )
    {
        LOGD( "run_sig_res initialization failed" );
    }
	pflag0_value = flag0_value;
	exit_thread_flag = false;
	pthread_create( &c_id ,  NULL ,  press_curve_thread ,  NULL );
    signal( SIGIO ,  read_press_data );// 异步IO事件	
    common_start( );
	return 0;	
}

static int start_pressure_dial( struct spictl_device_t* dev , float flag0_value )// 压力表盘模式
{	    
    LOGD( "start_pressure_dial=====" );	
    set_press_reg( smp_rate );	//设置压力采集寄存器 
   
	run_sig_res = sem_init( &run_sem ,  0 ,  0 );
	if( run_sig_res != 0 )
    {
        LOGD( "run_sig_res initialization failed" );
    }
    pflag0_value = flag0_value;
	exit_thread_flag = false;
	pthread_create( &c_id ,  NULL ,  press_dial_thread ,  NULL );

	signal( SIGIO ,  read_press_data );// 异步IO事件	
    common_start( );
	return 0;
}



////////////振动采集功能如下
#define   MAX_SIZE  31457280
unsigned char g_max_char_buf[ MAX_SIZE ] ={0}; //6.34M  6568576    ,26274304    31457280
int g_discard_pnts = 0; //IIR滤波需要丢弃的点数
int g_smpLength = 0; //实际采集的波形长度
int g_waveLength = 0; //UI下发的波形长度
int g_chNum = 0; //振动通道个数
int g_max_freq = 0; // 上限频率
int g_min_freq = 0; // 下限频率 


void read_evalute_data( ) //读取振动评估数据
{	    
    LOGD( "xin: read_evalute_data_g_smpLength = %d , [%s]\n" ,  g_smpLength , log_time( ) );    	
	if( stop_smp_flag )
	{		 
		return;
	}		
	int total_len = g_smpLength;  //采样总点数
	int shang = total_len/SIZE_15360;  //15360 的倍数
	int yu = total_len%SIZE_15360;  // 余数
		
	if( shang < 1 )
	{
		 if( read( fd , g_max_char_buf , yu*sizeof( float ) ) <0 )
		 {			 
			LOGD( "Error: spi slave device read fail !\n " ); 
		 }			
		 g_smp_buf =( int* )&( g_max_char_buf ) ;
         stop_smp_flag = 1;	
         LOGD( "xin: post振动信号量000 : [%s]\n" ,  log_time( ) );
         stop_dma();	 //停止DMA采集 ，关闭设备文件 		 
	     sem_post( &run_sem );		
	}		
}

void read_vibrate_data( ) //读取振动采集数据
{	    
    //LOGD( "xin: read_vibrate_data_stop_smp_flag = %d,g_loop_num = %d, g_smpLength = %d , g_chNum =%d  , [%s]\n" ,  stop_smp_flag, g_loop_num, g_smpLength , g_chNum , log_time( ) ); 
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
		 //LOGD( "00_g_loop_num = %d" , g_loop_num );
		 
		 if( read( fd , g_max_char_buf , yu*sizeof( float ) ) <0 )
		 {			 
			LOGD( "Error: spi slave device read fail !\n " ); 	
		 }	
		 stop_smp_flag = 1;
         stop_dma();		 
		 g_smp_buf =( int* )&( g_max_char_buf ) ;			 
			 
		 LOGD("000can_start_flag = %d,g_chNum = %d,power_off_flag = %d",can_start_flag,g_chNum,power_off_flag);		 	
		 
		 sem_post( &run_sem ); 
		 LOGD( "xin: post振动信号量000 : [%s]\n" ,  log_time( ) );      
	}
		
	if( shang > 0 && yu > 0 )
	{
		  //LOGD( "11_g_loop_num = %d" , g_loop_num );
		  if( read( fd , read_60K_buf , SIZE_60K ) <0 )
		  {			  
			 LOGD( "Error: spi slave device read fail !\n " ); 
		  }
		  memcpy( &backup_60K_buf , &read_60K_buf ,  SIZE_60K ); //60K
		 
		  if( g_loop_num < shang )
		  {			 				 
			 memcpy( &g_max_char_buf[SIZE_60K*g_loop_num] ,  &backup_60K_buf ,  SIZE_60K );	
			 g_loop_num++;
		  }
          else if( g_loop_num == shang )
		  {		
	         stop_smp_flag = 1; //flag置1,停止继续读数据
	         stop_dma(); //停止DMA采集
			 
			 memcpy( &g_max_char_buf[SIZE_60K*g_loop_num] ,  &backup_60K_buf ,  yu*sizeof( float ) );
			 
			 g_smp_buf =( int* )&( g_max_char_buf ) ;  
			 
			 LOGD("111can_start_flag = %d,g_chNum = %d,power_off_flag = %d",can_start_flag,g_chNum,power_off_flag);
			
			 sem_post( &run_sem ); 
			 LOGD( "xin: post振动信号量111 : [%s]\n" ,  log_time( ) );              	 
		  }		
	}	
	if( shang > 0 && yu == 0 )
	{      
         //LOGD( "22_g_loop_num = %d" , g_loop_num );
		 if( read( fd , read_60K_buf , SIZE_60K ) <0 )
		 {			 
			LOGD( "Error: spi slave device read fail !\n " );
		 }			
		 memcpy( &backup_60K_buf , &read_60K_buf ,  SIZE_60K ); //60K
		 
		 if( g_loop_num < shang )
		 {			
			 memcpy( &g_max_char_buf[SIZE_60K*g_loop_num] ,  &backup_60K_buf ,  SIZE_60K );	
			 g_loop_num++;
		 }
		 else if( g_loop_num == shang )
		 {		
	         stop_smp_flag = 1;
             stop_dma();	 
			 g_smp_buf =( int* )&( g_max_char_buf ) ; 
			 
			 LOGD("222can_start_flag = %d,g_chNum = %d,power_off_flag = %d",can_start_flag,g_chNum,power_off_flag);
			 
			 sem_post( &run_sem ); 
			 LOGD( "xin: post振动信号量222 : [%s]\n" ,  log_time( ) );              			 
		 }		
	}		
}

void *time_wave_thread( void* arg ) //时域线程
{	 
    LOGD("time_wave_thread============");  
	timewave my_timewave ={0};
	my_timewave = *( struct time_wave_para* )arg;
	
	int i=0 ;
    int temp_len =0;		
	float CH_data[3]={0.0};	
	
	if( g_chNum == SINGLE_CH )
	{		
		if( g_max_freq == 500 || g_max_freq == 2500 )
		{
			g_smpLength = ( g_waveLength + g_discard_pnts )*4;	//针对 这两个频率，需要1/4 抽点，所以采集长度 *4
		}else if( g_max_freq == 1000 || g_max_freq == 5000 )
		{
			g_smpLength = ( g_waveLength + g_discard_pnts )*2;	//针对 这两个频率，需要1/2 抽点，所以采集长度 *2
		}else
		{
			g_smpLength = ( g_waveLength + g_discard_pnts );	//实际采集的波形长度			
		}

		LOGD( "xin: time_wave_thread_SINGLE_CH_g_discard_pnts = %d ,  g_waveLength = %d ,  g_smpLength = %d"  , g_discard_pnts ,  g_waveLength ,  g_smpLength );
				
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
        memset(time_CH1_smp_buf , 0 ,g_smpLength*sizeof(float) );			
							
		for( i=0;i< g_smpLength;i++ )	//转换采集的点数	
		{					
			analyze_CH_data( g_smp_buf[i] , CH_data );	
			time_CH1_smp_buf[i] = CH_data[2];				
		}					
		LOGD( "xin: SINGLE_CH_结束转换数据 is : [%s]\n" ,  log_time( ) );				
		
		#if 1		
			temp_len = g_smpLength; ////FIR 低通滤波，单通道，采样长度		
			if(g_max_freq == 500 || g_max_freq == 2500 )//上限是5000 2500是1/4抽样 
			{						
				enter_FIR_Filter( time_CH1_smp_buf  , temp_len, g_max_freq); //FIR 低通滤波
				temp_len = g_smpLength/4; //计算后长度变为1/4	
				
				enter_IIR_Filter( time_CH1_smp_buf , temp_len ,g_max_freq ,g_min_freq ); //IIR 高通滤波
			}else if( g_max_freq == 1000 || g_max_freq == 5000)	 //上限是1000 5000是1/2抽样 
			{
				enter_FIR_Filter( time_CH1_smp_buf  , g_smpLength, g_max_freq); //FIR 低通滤波
				temp_len = g_smpLength/2; //计算后长度变为1/2	
				
				enter_IIR_Filter( time_CH1_smp_buf ,temp_len ,g_max_freq ,g_min_freq ); //IIR 高通滤波
				
			}else  //其它上限频率只经过IIR
			{	
				enter_IIR_Filter( time_CH1_smp_buf , temp_len ,g_max_freq ,g_min_freq ); //IIR 高通滤波
			}
		
			LOGD("SINGLE_CH_temp_len = %d , g_discard_pnts = %d, wave_length = %d, wave_length+discard_pnts = %d" ,temp_len,g_discard_pnts,g_waveLength,g_waveLength + g_discard_pnts  );			
			memcpy( time_CH1_smp_buf ,  &time_CH1_smp_buf[g_discard_pnts] ,  g_waveLength*sizeof( float )  );//经IIR滤波后 去掉丢弃的点							
			LOGD( "xin: SINGLE_CH_退出算法 is : [%s]\n" ,  log_time( ) );
		#endif 		
			
        if(power_off_flag == 1) //置于stop_dma前面，用于先断电，让dma硬buf 为空
		{	
            LOGD("xin: 开始回调时power_off_flag ==1");
 
            LOGD( "xin: SINGLE_CH_回调结束 is : [%s]\n" ,  log_time( ) );      //gxl add 2017.9.6	
			if( time_CH1_smp_buf !=NULL)
			{			
				free( time_CH1_smp_buf );
				time_CH1_smp_buf =NULL; 
			}				
			
			can_start_flag = 1;
			LOGD("xin: free——malloc===========");
			sem_destroy( &run_sem );
			thread_finished_flag = 1;			
			
			usleep(10000);                                                     //gxl add 
			
			vibrate_callback_backup.single_ch_callback( invalid_buf , 0,  false );	/////回调时域波形			
		    return NULL;
		}else{		
			vibrate_callback_backup.single_ch_callback( time_CH1_smp_buf , g_waveLength ,  true );	/////回调时域波形
		}
			
		LOGD( "xin: SINGLE_CH_回调结束 is : [%s]\n" ,  log_time( ) );	
		if( time_CH1_smp_buf !=NULL)
		{			
			free( time_CH1_smp_buf );
			time_CH1_smp_buf =NULL; 
		}				
		
		can_start_flag = 1;
		LOGD("xin: free——malloc===========");
		sem_destroy( &run_sem );
		thread_finished_flag = 1;
	}
	
	if( g_chNum == DOUBLE_CH ) //双通道内存为单通道的两倍
	{			
		if( g_max_freq == 500 || g_max_freq == 2500 )
		{
			g_smpLength = ( g_waveLength + g_discard_pnts  )*2*4;	//针对 这两个频率，需要1/4 抽点，所以采集长度 *4
		}else if( g_max_freq == 1000 || g_max_freq == 5000 )
		{
			g_smpLength = ( g_waveLength + g_discard_pnts )*2*2;	//针对 这两个频率，需要1/2 抽点，所以采集长度 *2
		}else
		{
			g_smpLength = ( g_waveLength+ g_discard_pnts )*2;	//实际采集的波形长度
		}
	
		LOGD( "xin: time_wave_thread_DOUBLE_CH_g_discard_pnts = %d ,  g_waveLength = %d ,  g_smpLength = %d"  , g_discard_pnts ,  g_waveLength,  g_smpLength );        
	    
        	
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
			analyze_CH_data( g_smp_buf[i] , CH_data );	
			time_CH1_smp_buf[i] = CH_data[1];
            time_CH2_smp_buf[i] = CH_data[2];				
		}	
        for(i = 0; i<= g_smpLength/2; i++)
		{
			time_CH1_smp_buf[i] = time_CH1_smp_buf[2*i];
			time_CH2_smp_buf[i] = time_CH2_smp_buf[2*i+1];
		}			
		LOGD( "xin: DOUBLE_CH_结束转换数据 is : [%s]\n" ,  log_time( ) );      	
		
		#if 1				
			temp_len = g_smpLength/2; //FIR 低通滤波，原采样长度为双通道 一半			
			if(g_max_freq == 500 || g_max_freq == 2500 )//上限是5000 2500是1/4抽样 
			{					
				enter_FIR_Filter( time_CH1_smp_buf  , temp_len, g_max_freq); //FIR 低通滤波
				enter_FIR_Filter( time_CH2_smp_buf  , temp_len, g_max_freq); 					
				temp_len = g_smpLength/2/4; //FIR 低通滤波,计算后长度变为1/4	
									
				enter_IIR_Filter( time_CH1_smp_buf , temp_len , g_max_freq , g_min_freq ); //IIR 高通滤波
				enter_IIR_Filter( time_CH2_smp_buf , temp_len , g_max_freq , g_min_freq ); //IIR 高通滤波
			}else if( g_max_freq == 1000 || g_max_freq == 5000)	 //上限是1000 5000是1/2抽样 
			{
				enter_FIR_Filter( time_CH1_smp_buf  , temp_len,  g_max_freq); //FIR 低通滤波
				enter_FIR_Filter( time_CH2_smp_buf  , temp_len,  g_max_freq); 
				temp_len = g_smpLength/2/2; //FIR 低通滤波,计算后长度变为1/2
				
				enter_IIR_Filter( time_CH1_smp_buf , temp_len , g_max_freq , g_min_freq ); //IIR 高通滤波
				enter_IIR_Filter( time_CH2_smp_buf , temp_len , g_max_freq , g_min_freq ); //IIR 高通滤波
				
			}else  //其它上限频率只经过IIR
			{	
				enter_IIR_Filter( time_CH1_smp_buf , temp_len , g_max_freq , g_min_freq );  //IIR 滤波长度为 采集长度的一半
				enter_IIR_Filter( time_CH2_smp_buf , temp_len , g_max_freq , g_min_freq );
			}
			LOGD("DOUBLE_CH_temp_len = %d ,g_discard_pnts = %d,wave_length = %d, wave_length+discard_pnts = %d" ,temp_len,g_discard_pnts,g_waveLength, g_waveLength + g_discard_pnts );	
			memcpy( time_CH1_smp_buf ,  &time_CH1_smp_buf[g_discard_pnts] ,  g_waveLength*sizeof( float ));// 返回实际需要的点数，去掉丢弃的点
			memcpy( time_CH2_smp_buf ,  &time_CH2_smp_buf[g_discard_pnts] ,  g_waveLength*sizeof( float ));
		
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
			LOGD("xin: free——malloc=====下电时提前释放返回======");
			sem_destroy( &run_sem );
			thread_finished_flag = 1;	
			usleep(10000);  // 用于线程执行后后面flag 执行
			vibrate_callback_backup.double_ch_callback( invalid_buf ,  invalid_buf  , 0 ,  false ); /////回调 UI长度
            return NULL;			
		}else{	
            LOGD("xin: g_waveLength = %d",g_waveLength);		
			vibrate_callback_backup.double_ch_callback( time_CH1_smp_buf ,  time_CH2_smp_buf  , g_waveLength ,  true ); /////回调 UI长度	
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
		LOGD("xin: free——malloc=====正常释放返回======");
		sem_destroy( &run_sem );
		thread_finished_flag = 1;
	}
	return NULL;
}

void *total_rend_thread( void* arg ) //总值趋势线程
{
    totalrend my_totalrend ={0};
    my_totalrend = *( struct total_rend_para* )arg;	
	
	int i=0;		
	int temp_len = 0;
	float CH1_value[1] ={0.0};
	float CH2_value[1] ={0.0};	
	float CH_data[3]={0.0};
	
	if( g_chNum == SINGLE_CH  )
	{		        
        if( g_max_freq == 500 || g_max_freq == 2500 )
		{
			g_smpLength = ( g_waveLength + g_discard_pnts )*4;	//针对 这两个频率，需要1/4 抽点，所以采集升度 *4
		}else if( g_max_freq == 1000 || g_max_freq == 5000 )
		{
			g_smpLength = ( g_waveLength + g_discard_pnts )*2;	//针对 这两个频率，需要1/2 抽点，所以采集升度 *2
		}else
		{
			g_smpLength = ( g_waveLength + g_discard_pnts );	//实际采集的波形长度
		}
		
		LOGD( "xin: total_rend_thread_SINGLE_CH_g_discard_pnts = %d ,  g_waveLength = %d ,  g_smpLength = %d"  , g_discard_pnts ,  g_waveLength ,  g_smpLength );
		
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
		memset( rend_CH1_smp_buf , 0 , g_smpLength*sizeof( float ) );			
	    
	    for( i=0;i< g_smpLength;i++ )	//转换采集的点数	
		{					
			analyze_CH_data( g_smp_buf[i] , CH_data );	
			rend_CH1_smp_buf[i] = CH_data[2];				
		}        		
		LOGD( "xin: SINGLE_CH_结束转换数据 is : [%s]\n" ,  log_time( ) );
		
				
		#if 1			
			temp_len = g_smpLength;		////FIR 低通滤波，单通道，采样长度	
			if(g_max_freq == 500 || g_max_freq == 2500 )//上限5000 2500 是1/4抽样 
			{				
				enter_FIR_Filter( rend_CH1_smp_buf  , temp_len, g_max_freq); //FIR 低通滤波
				temp_len = g_smpLength/4; //计算后长度变为1/4
				
				enter_IIR_Filter( rend_CH1_smp_buf , temp_len , g_max_freq , g_min_freq ); //IIR 高通滤波
			}else if( g_max_freq == 1000 || g_max_freq == 5000)	 //上限1000 5000是1/2抽样 
			{
				enter_FIR_Filter( rend_CH1_smp_buf  , temp_len,  g_max_freq); //FIR 低通滤波
				temp_len = g_smpLength/2; //计算后长度变为1/2	
				
				enter_IIR_Filter( rend_CH1_smp_buf , temp_len , g_max_freq , g_min_freq ); //IIR 高通滤波				
			}else  //其它上限频率只经过IIR
			{	               		
				enter_IIR_Filter( rend_CH1_smp_buf , temp_len , g_max_freq , g_min_freq ); //IIR 高通滤波，长度是采样的点
			}			
			
			LOGD("SINGLE_CH_temp_len = %d , g_discard_pnts = %d, wave_length = %d, wave_length+discard_pnts = %d" ,
			     temp_len,g_discard_pnts,g_waveLength, g_waveLength + g_discard_pnts  );			
			
			memcpy( rend_CH1_smp_buf , &rend_CH1_smp_buf[g_discard_pnts] , g_waveLength*sizeof( float ) );//将IIR滤波后 去掉丢弃的点	
		    LOGD( "xin: SINGLE_CH_退出算法 is : [%s]\n" ,  log_time( ) );
		#endif
				
		
		CH1_value[0] = rend_value( rend_CH1_smp_buf , g_waveLength , my_totalrend.total_value_type );            			
		LOGD( "xin: SINGLE_CH_CH1_value = %f" , CH1_value[0] );			
	
	    if(power_off_flag == 1) //置于stop_dma前面，用于先断电，让dma硬buf 为空
		{	  
            LOGD("xin: 开始回调时power_off_flag ==1");	
            if( rend_CH1_smp_buf != NULL)
			{			
				free( rend_CH1_smp_buf ); 
				rend_CH1_smp_buf =NULL;
			}
						
			can_start_flag = 1;
			LOGD("xin: free——malloc===========");
			sem_destroy( &run_sem );
			thread_finished_flag = 1;		
			usleep(10000);  // 用于线程执行后后面flag 执行
			vibrate_callback_backup.single_ch_callback( invalid_buf  , 0 ,  false ); 
			return NULL;
		}else{
			vibrate_callback_backup.single_ch_callback( CH1_value , sizeof( CH1_value )/sizeof( float ) ,  true ); /////回调
		}		
		LOGD( "xin: SINGLE_CH_回调结束 is : [%s]\n" ,  log_time( ) );	
							
		if( rend_CH1_smp_buf != NULL)
		{			
			free( rend_CH1_smp_buf ); 
			rend_CH1_smp_buf =NULL;
		}
					
		can_start_flag = 1;
		LOGD("xin: free——malloc===========");
		sem_destroy( &run_sem );
		thread_finished_flag = 1;			      		
	}

	if( g_chNum == DOUBLE_CH ) //双通道内存为单通道时的两倍
	{	
        if( g_max_freq == 500 || g_max_freq == 2500 )
		{
			g_smpLength = ( g_waveLength + g_discard_pnts  )*2*4;	//针对 这两个频率，需要1/4 抽点，所以采集长度 *4
		}else if( g_max_freq == 1000 || g_max_freq == 5000 )
		{
			g_smpLength = ( g_waveLength + g_discard_pnts  )*2*2;	//针对 这两个频率，需要1/2 抽点，所以采集长度 *2
		}else
		{
			g_smpLength = ( g_waveLength + g_discard_pnts )*2;	//实际采集的波形长度
		}
		
		LOGD( "xin: total_rend_thread_DOUBLE_CH_g_discard_pnts = %d ,  g_waveLength = %d ,  g_smpLength = %d"  , g_discard_pnts ,  g_waveLength ,  g_smpLength );
       
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
			analyze_CH_data( g_smp_buf[i] , CH_data );  //调用解析通道数据的函数	
			rend_CH1_smp_buf[i] = CH_data[1];
			rend_CH2_smp_buf[i] = CH_data[2];
		}			
		for( i=0;i<=g_smpLength/2;i++ )
		{					
			rend_CH1_smp_buf[i] = rend_CH1_smp_buf[i*2]; //通道1
			rend_CH2_smp_buf[i] = rend_CH2_smp_buf[i*2+1];//通道2					
		}		
		LOGD( "xin: DOUBLE_CH_数据转换结束 is : [%s]\n" ,  log_time( ) );
		
		#if 1				
			temp_len = g_smpLength/2; //FIR 低通滤波，原采样长度为双通道 一半	
			if(g_max_freq == 500 || g_max_freq == 2500 )//上限是5000 2500是1/4抽样 
			{
				enter_FIR_Filter( rend_CH1_smp_buf  , temp_len, g_max_freq); //FIR 低通滤波
				enter_FIR_Filter( rend_CH2_smp_buf  , temp_len, g_max_freq); 
				temp_len = g_smpLength/2/4; //计算后长度变为1/4
				
				enter_IIR_Filter( rend_CH1_smp_buf , temp_len ,g_max_freq ,g_min_freq ); //IIR 高通滤波
				enter_IIR_Filter( rend_CH2_smp_buf , temp_len ,g_max_freq ,g_min_freq ); //IIR 高通滤波
			}else if( g_max_freq == 1000 || g_max_freq == 5000)	 //上限是1000 5000是1/2抽样 
			{				
				enter_FIR_Filter( rend_CH1_smp_buf  , temp_len, g_max_freq); //FIR 低通滤波，原采样长度，计算后长度变为1/2
				enter_FIR_Filter( rend_CH2_smp_buf  , temp_len, g_max_freq); //FIR 低通滤波，原采样长度，计算后长度变为1/2
				temp_len = g_smpLength/2/2; //计算后长度变为1/2
			
				enter_IIR_Filter( rend_CH1_smp_buf , temp_len ,g_max_freq ,g_min_freq ); //IIR 高通滤波
				enter_IIR_Filter( rend_CH2_smp_buf , temp_len ,g_max_freq ,g_min_freq ); //IIR 高通滤波
			}else{  //其它上限频率只经过IIR
				enter_IIR_Filter( rend_CH1_smp_buf , temp_len ,g_max_freq ,g_min_freq );  //IIR 滤波长度为 采集长度的一半
				enter_IIR_Filter( rend_CH2_smp_buf , temp_len ,g_max_freq ,g_min_freq );
			}				
					
			LOGD("DOUBLE_CH_temp_len = %d , g_discard_pnts = %d, wave_length = %d, wave_length+discard_pnts = %d" ,temp_len,g_discard_pnts,g_waveLength, g_waveLength + g_discard_pnts );	
			memcpy( rend_CH1_smp_buf , &rend_CH1_smp_buf[g_discard_pnts] ,g_waveLength*sizeof( float ) );// 返回实际需要的点数，去掉丢弃的点
			memcpy( rend_CH2_smp_buf , &rend_CH2_smp_buf[g_discard_pnts] ,g_waveLength*sizeof( float ) );			
        #endif	
		LOGD( "xin: DOUBLE_CH_退出算法 is : [%s]\n" ,  log_time( ) );
		
		CH1_value[0] = rend_value( rend_CH1_smp_buf , g_waveLength , my_totalrend.total_value_type );	 //计算总值趋势值 CH1 
		CH2_value[0] = rend_value( rend_CH2_smp_buf , g_waveLength , my_totalrend.total_value_type );	 //计算总值趋势值 CH2	
		LOGD( "xin: DOUBLE_CH_CH1_value = %f ,  CH2_value = %f" , CH1_value[0] , CH2_value[0] );				
		
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
			LOGD("xin: free——malloc===========");
			sem_destroy( &run_sem );
			thread_finished_flag = 1;
		    usleep(10000);  // 用于线程执行后后面flag 执行
			vibrate_callback_backup.double_ch_callback( invalid_buf  , invalid_buf, 0 ,  false ); /////回调 UI长度
			return NULL;
		}else{		
			vibrate_callback_backup.double_ch_callback( CH1_value , CH2_value , sizeof( CH1_value )/sizeof( float ) , true );	/////回调
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
		LOGD("xin: free——malloc===========");
		sem_destroy( &run_sem );
		thread_finished_flag = 1;		       		
	}	    	
	return NULL;
}

void *evalute_level_thread( void* arg ) //等级评估线程
{    
    timewave my_timewave ={0};
    my_timewave = *( struct time_wave_para* )arg;
	
    int i=0;
    int temp_len =0;	
	float evalute_value[1] ={0.0};	 //0位:表示速度
	float CH_data[1]={0.0};		
	
	if( g_max_freq == 1000 )
	{
		g_smpLength = ( g_waveLength + g_discard_pnts  )*2;	//针对 这1000频率，需要1/2 抽点，所以采集升度 *2
	}	
	LOGD( "xin: evalute_level_thread_SINGLE_CH_g_discard_pnts = %d ,  g_waveLength = %d ,  g_smpLength = %d"  , g_discard_pnts ,  g_waveLength ,  g_smpLength );
			
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
   		
	for( i=0;i< g_smpLength;i++ )	//转换采集的点数	
	{					
		analyze_CH_data( g_smp_buf[i] , CH_data );	
		evalute_CH1_smp_buf[i] = CH_data[2]; //单通道振动采集默认CHB 数据				
	}					
	LOGD( "xin: SINGLE_CH_结束转换数据 is : [%s]\n" ,  log_time( ) );	
	
	#if 1				
		temp_len = g_smpLength; ////FIR 低通滤波，单通道，采样长度	
		if( g_max_freq == 1000 )	 //上限是1000 是1/2抽样 
		{
			enter_FIR_Filter( evalute_CH1_smp_buf  , g_smpLength, g_max_freq); //FIR 低通滤波
			temp_len = g_smpLength/2; //计算后长度变为1/2	
			
			enter_IIR_Filter( evalute_CH1_smp_buf ,temp_len , g_max_freq , g_min_freq ); //IIR 高通滤波
		}		
        memcpy( evalute_CH1_smp_buf ,  &evalute_CH1_smp_buf[g_discard_pnts] ,  g_waveLength*sizeof( float ) );	
        LOGD( "xin: SINGLE_CH_退出算法 is : [%s]\n" ,  log_time( ) );		
	#endif 	
	
	evalute_value[0] = rend_value( evalute_CH1_smp_buf ,  g_waveLength ,  0 );	//速度有效值 0表示有效值
	LOGD( "xin: evalute_value[0]速度有效值 = %f" ,  evalute_value[0] );						
	vibrate_callback_backup.single_ch_callback( evalute_value , sizeof( evalute_value )/sizeof( float ) , true ); /////回调	
	LOGD( "xin: SINGLE_CH_回调结束 is : [%s]\n" ,  log_time( ) );
	
	if( evalute_CH1_smp_buf != NULL)
	{
		free( evalute_CH1_smp_buf ); 
		evalute_CH1_smp_buf =NULL;	
	}
	LOGD("xin: free——malloc===========");			
			
	////////自动stop
	start_enable_flag = 0;	
	restart_power_on_flag = 0;  //此时两个flag 置0,防止其它测试时卡住
	
	int res = -1;	
	int g_ret_val = 0;	
	swrite( StopSampleAddr , StopSampleData );//stop fpga
    res = ioctl( fd ,  SPIDEV_IOC_RXSTREAMOFF ,  NULL ); //停止DMA采集
    g_ret_val=close( fd );					
	spi_poweroff( );  //下电	  
	sem_destroy( &run_sem );  	
	return NULL;
}

static int start_vibrate_CH_timewave( struct spictl_device_t* dev ,  int ch_num , struct time_wave_para tWave  )//时域波形
{	
    memset(g_max_char_buf ,0, MAX_SIZE*sizeof(unsigned char)); 
	int reg_ret_value =0;
	
	if(start_enable_flag == 1) // 1表示前一个start 线程还没有结束，此时不再响应新的start， 为0时表示start线程结束了
	{
		LOGD("xin: start_enable_flag = %d, 前一个start还未执行完，此时不响应新的start接口",start_enable_flag);		
        return 0;
	}else{
	    start_enable_flag = 1;
	}
	
    g_chNum = ch_num;	 
	
	g_discard_pnts = 0;
	g_loop_num =0;	
	g_smpLength = 0;	
	g_waveLength = 0;
	
    g_max_freq = (int)tWave.max_freq;
	g_min_freq = (int)tWave.min_freq;
	g_waveLength = tWave.wave_length;
	
	LOGD("xin: 点出开始时 start_enable_flag = %d, restart_power_on_flag= %d, can_start_flag = %d , thread_finished_flag = %d",
	    start_enable_flag,restart_power_on_flag,can_start_flag,thread_finished_flag);
	LOGD( "xin: start_vibrate_CH_timewave_ch_num = %d , data_type = %d , signal_type = %d ,  min_freq = %d , max_freq = %d , wave_length = %d ", 						
	 ch_num ,  tWave.data_type ,  tWave.signal_type ,  g_min_freq ,  g_max_freq  , g_waveLength);	
	     
    
    LOGD("xin: 响应点击开始 [%s]\n" ,  log_time( )); 
    if(restart_power_on_flag == 1) // 当内部stop_DMA，fpga,关闭设备后，flag 置1， 当再启动start接口时不再重新上电，当对外大的停止采集下电后，此flag会置为 0，重新采集时再上电
	{
		;
	}else{
		spi_poweron(  );
		usleep( 130000 ); //上电后延时		
		
		if( g_chNum == SINGLE_CH )	
			reg_ret_value = set_singleCH_vibrate_reg( tWave.signal_type , g_max_freq , g_min_freq );//设置单通道采集寄存器	
		if( g_chNum == DOUBLE_CH ) 
			reg_ret_value = set_doubleCH_vibrate_reg( tWave.signal_type , g_max_freq , g_min_freq );//设置双通道采集寄存器
		
		#if 1
		if( reg_ret_value == -1)
		{
			vibrate_callback_backup.single_ch_callback( reg_fail_buf ,1 ,true);
			start_enable_flag = 0;
            spi_poweroff(  );			
			return 0;
		}
		#endif
	}	   
    
    if (can_start_flag == 1)
	    return 0;    
    
    if( g_waveLength <= 0)
	{
		start_enable_flag = 0; //用于波形长度为0时，防止再手动停止时再采集时 ，start_enable_flag = 0， 直接 回调false给上层，所以此处要置0
		LOGD("xin: 下发的波形长度 tWave.wave_length = %d",g_waveLength);	
		return 0;
	}		

	usleep(100000);//用于stop 和start DMA,寄存器稳定
	sem_init( &run_sem ,  0 ,  0 );
	
    g_discard_pnts =  Get_InvalidNum(g_max_freq , g_min_freq );  //根据上下限，获取IIR需要丢弃的点数
	   	
	stop_smp_flag = 0;
   	
	pthread_create( &c_id ,  NULL ,  time_wave_thread , ( void* )&tWave );
     	
	signal( SIGIO ,  read_vibrate_data ); 
	
    common_start( );
	
	while( thread_finished_flag == 0); // 用于等待算法线程线束时置1，若线程结束时，继续往下运行
	can_start_flag = 0;
	thread_finished_flag = 0;
	
	//restart_power_on_flag = 0;	//////////////////////////
	
	power_off_flag = 0; 
	start_enable_flag = 0;
	LOGD( "xin: 响应点击结束 [%s]\n" ,  log_time( ) ); 
	
	usleep( 1000000 ); //用于底层虚拟地址和物理地址映射出错		
	return 0;
}

static int start_vibrate_CH_totalrend( struct spictl_device_t* dev ,  int ch_num , struct total_rend_para tRend  )//总值趋势
{
	memset(g_max_char_buf ,0, MAX_SIZE*sizeof(unsigned char));
    usleep( tRend.interval_time*1000*1000 ); //下次采集间隔时间  微秒为单位 
	
	if(start_enable_flag == 1)
	{
		LOGD("xin: start_enable_flag = %d, 不响应start接口",start_enable_flag);		
        return 0;
	}else{
	    start_enable_flag = 1;
	} 
	
	g_chNum = ch_num;	
	g_discard_pnts = 0;
	g_loop_num =0;	
	g_smpLength = 0;
	g_waveLength = 0;
	
    g_max_freq = (int)tRend.max_freq;
    g_min_freq = (int)tRend.min_freq;
	g_waveLength = tRend.wave_length;
	
	LOGD("xin: 点出开始时 start_enable_flag = %d, restart_power_on_flag= %d, can_start_flag = %d , thread_finished_flag = %d",
	    start_enable_flag,restart_power_on_flag,can_start_flag,thread_finished_flag);
	LOGD( "xin: start_vibrate_CH_totalrend_ch_num = %d , data_type = %d , signal_type = %d ,  min_freq = %d , max_freq = %d , wave_length = %d ", 						
	 ch_num ,  tRend.data_type ,  tRend.signal_type ,  g_min_freq ,  g_max_freq  , g_waveLength);	
			
	if(restart_power_on_flag == 1) // 当内部stop_DMA，fpga,关闭设备后，flag 置1， 当再启动start接口时不再重新上电，当对外大的停止采集下电后，此flag会置为 0，重新采集时再上电
	{
		;
	}else{
		spi_poweron(  );//上电后延时
		usleep( 130000 ); 
		if( g_chNum ==SINGLE_CH )	
			set_singleCH_vibrate_reg( tRend.signal_type , g_max_freq , g_min_freq );//设置单通道采集寄存器
		if( g_chNum == DOUBLE_CH ) 
			set_doubleCH_vibrate_reg( tRend.signal_type , g_max_freq , g_min_freq );//设置双通道采集寄存器	
	}
	
	if (can_start_flag == 1)
	    return 0;
	
	if( g_waveLength <= 0)
	{
		start_enable_flag = 0; //用于波形长度为0时，防止再手动停止时再采集时 ，start_enable_flag = 0， 直接 回调false给上层，所以此处要置0
		LOGD("xin: 下发的波形长度 tRend.wave_length = %d",g_waveLength);	
		return 0;
	}
	
	LOGD( "xin: 响应点击开始 [%s]\n" ,  log_time( ) ); 
	
	sem_init( &run_sem ,  0 ,  0 );
	
    g_discard_pnts =  Get_InvalidNum( g_max_freq ,  g_min_freq ); 
       
    	
	stop_smp_flag = 0;			  
		
	pthread_create( &c_id ,  NULL ,  total_rend_thread , ( void* )&tRend ); 
    
	signal( SIGIO ,  read_vibrate_data ); 
		
    common_start( );
	while( thread_finished_flag== 0); // 用于等待算法线程线束时置1，若线程结束时，继续往下运行
	can_start_flag = 0;
	thread_finished_flag = 0;
	
	power_off_flag = 0; 
	start_enable_flag = 0;
	LOGD( "xin: 响应点击结束 [%s]\n" ,  log_time( ) ); 
	return 0;
}

static int start_vibrate_evalute_level( struct spictl_device_t* dev , struct time_wave_para tWave )//振动等级评估
{	
    memset(g_max_char_buf ,0, MAX_SIZE*sizeof(unsigned char));
	g_discard_pnts =0;
	g_loop_num =0;
	g_smpLength = 0;
	g_waveLength = 0;
	
	g_max_freq = (int)tWave.max_freq;
    g_min_freq = (int)tWave.min_freq;
	g_waveLength = tWave.wave_length;
	   
	LOGD( "xin: start_vibrate_evalute_level_signal_type = %d ,  min_freq = %d , max_freq = %d , wave_length = %d ",	 tWave.signal_type ,  g_min_freq ,  g_max_freq  , g_waveLength);	
	 
	if( g_waveLength <= 0)
	{
		LOGD("xin: 下发的波形长度 tWave.wave_length = %d",g_waveLength);
		return 0;
	}
	
	LOGD( "xin: 响应点击开始 [%s]\n" ,  log_time( ) ); 	
	spi_poweron(  );//上电后延时
	usleep( 130000 ); 
	
	set_singleCH_vibrate_reg( tWave.signal_type , g_max_freq , g_min_freq );//设置单通道采集寄存器，数据类型为 速度时域波形
	
	sem_init( &run_sem ,  0 ,  0 );	
	
    g_discard_pnts = Get_InvalidNum( g_max_freq , g_min_freq );    	  
    	
	stop_smp_flag = 0;		
			
	pthread_create( &c_id ,  NULL ,  evalute_level_thread , ( void* )&tWave );	
	
	signal( SIGIO ,  read_evalute_data ); 				
	  		 
    common_start( );
	LOGD( "xin: 响应点击结束 [%s]\n" ,  log_time( ) ); 
	return 0;
}

static float* spi_get_feature_value( struct spictl_device_t* dev , float pData[] , int data_len )//获取15个特征值
{	
	LOGD( "spi_get_feature_value" );
	
	int i=0;	
	memset( feature_ret_value , 0 , FEATURE_NUM*sizeof( float ) );		
	feature_value( pData ,  data_len ,  feature_ret_value ); // 求15个特征值
	
	#if 0
	for( i=0;i<FEATURE_NUM;i++ )
	{
		LOGD( "feature_ret_value[%d] = %f" , i , feature_ret_value[i] ); //直接转换为加速度值
	}
	#endif 	
	
	return feature_ret_value;	
}

static float* spi_time_to_freq_value( struct spictl_device_t* dev , float pData[] , int data_len  )//时域到频域接口
{
	LOGD( "spi_time_to_freq_value" );	   		
	fft_alg_entry2( pData ,  data_len , 0 , 0 , 0 );	//0默认不加窗，不平均，平均方式不进行
	
	#if 0
	int i =0;
	for(i = 0; i< data_len/2; i++)
	{
		 LOGD( "ret_pData[%d] = %f" , i, pData[i] );	
	}
	#endif 
	
	return pData;    		
}


/////////频率采集功能如下 
void *freq_wave_thread( void* arg ) //频域线程
{	
    freqwave my_freqwave ={0};
	my_freqwave = *( struct freq_wave_para* )arg;		
	int i=0 , j=0 , k=0;	
	float CH_data[3]={0.0};
	
	if( g_chNum == SINGLE_CH ) 
	{
	    g_waveLength = my_freqwave.spectra_num*2.56; //波形长度 = 频谱线数*2.56	
		g_smpLength = my_freqwave.spectra_num*2.56 + g_discard_pnts;	//实际采集的波形长度				
				
		LOGD( "xin: freq_wave_thread_SINGLE_CH_g_discard_pnts = %d ,  spectra_num*2.56 = %d ,  g_smpLength = %d"  , g_discard_pnts ,  g_waveLength ,  g_smpLength );	
		if( g_waveLength < 0 )
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
			freq_CH1_IIR_buf =( float* )malloc( g_waveLength*sizeof( float ) );
			if( freq_CH1_IIR_buf == NULL )
			{
				LOGD( "freq_CH1_IIR_buf 分配内存失败！" );
				exit( EXIT_FAILURE );		
			}
			memset( freq_CH1_IIR_buf , 0 , g_waveLength*sizeof( float ) );
		}
		
		
		while(  1  )
		{		
			sem_wait( &run_sem );//等待信号量		
			          		
			if( exit_thread_flag )
			{
				post_flag = false;
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
				analyze_CH_data( g_smp_buf[i] , CH_data );  //调用解析通道数据的函数								
				freq_CH1_smp_buf[i] = CH_data[1];		 							
			} 
			#if debug
			for( j=0;j< 1024;j++ )  //test////////
			{								
				LOGD( "SINGLE_CH_滤波前freq_CH1_smp_buf[%d] = %f" , j , freq_CH1_smp_buf[j] );	
			}
			#endif
			
			enter_IIR_Filter( freq_CH1_smp_buf , g_smpLength ,( int )my_freqwave.max_freq ,( int )my_freqwave.min_freq ); //IIR 高通滤波，长度是采样长度
			
			#if debug
			for( j=0;j< 500;j++ )  //test/////////
			{								
				LOGD( "SINGLE_CH_滤波后freq_CH1_smp_buf[%d] = %f" , j , freq_CH1_smp_buf[j] );	
			}
			#endif
						
			memcpy( freq_CH1_IIR_buf , &freq_CH1_smp_buf[g_discard_pnts] , g_waveLength*sizeof( float ) );
			
			#if debug
			for( j=0;j< 500;j++ )  //test/////////
			{								
				LOGD( "SINGLE_CH_回调数据freq_CH1_IIR_buf[%d] = %f" , j , freq_CH1_IIR_buf[j] );	
			}			
			#endif
			
			fft_alg_entry2( freq_CH1_IIR_buf , g_waveLength , 0 , 0 , 0  ); //0默认不加窗，平均方式，平均次数都为0，不进行
			#if 1
			for( j=0;j< 100;j++ )////test///////// g_waveLength/2
			{								
				LOGD( "xin:SINGLE_CH_freq_CH1_IIR_buf[%d] = %f" , j , freq_CH1_IIR_buf[j] );                				
			}			
            #endif		
			vibrate_callback_backup.single_ch_callback( freq_CH1_IIR_buf ,  g_waveLength/2 ,  true );	/////频谱回调 长度是实际UI长度一半								    
			stop_smp_flag = 0;		
		}		
	}
		
	if( g_chNum == DOUBLE_CH ) //双通道内存为单通道时的两倍
	{	       
	    g_waveLength =( my_freqwave.spectra_num*2.56 )*2; //波形长度 = 频谱线数*2.56
		g_smpLength =( my_freqwave.spectra_num*2.56 + g_discard_pnts )*2;	//实际采集的波形长度        
		LOGD( "xin:freq_wave_thread_DOUBLE_CH_g_discard_pnts = %d ,  spectra_num*2.56 = %d ,  g_smpLength = %d"  , g_discard_pnts ,  g_waveLength/2 ,  g_smpLength );
		if( g_waveLength < 0 )
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
			freq_CH1_IIR_buf =( float* )malloc(( g_waveLength/2 )*sizeof( float ) );
			if( freq_CH1_IIR_buf == NULL )
			{
				LOGD( "freq_CH1_IIR_buf 分配内存失败！" );
				exit( EXIT_FAILURE );		
			}
			memset( freq_CH1_IIR_buf , 0 ,( g_waveLength/2 )*sizeof( float ) );	
		}
		
		float *freq_CH2_IIR_buf =NULL; //通道2 IIR返回后的数据
		if( freq_CH2_IIR_buf == NULL)
		{
			freq_CH2_IIR_buf =( float* )malloc(( g_waveLength/2 )*sizeof( float ) );
			if( freq_CH2_IIR_buf == NULL )
			{
				LOGD( "freq_CH2_IIR_buf 分配内存失败！" );
				exit( EXIT_FAILURE );		
			}
			memset( freq_CH2_IIR_buf , 0 ,( g_waveLength/2 )*sizeof( float ) );		
		}		
		
		while( 1 )
		{		
			sem_wait( &run_sem );//等待信号量			
						
			if( exit_thread_flag )
			{
				post_flag = false;
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
				analyze_CH_data( g_smp_buf[i] , CH_data );  //调用解析通道数据的函数	
				freq_CH1_smp_buf[i] = CH_data[1];
				freq_CH2_smp_buf[i] = CH_data[2];				
			}			
			for( k=0;k<g_smpLength/2;k++ )
			{					
				freq_CH1_smp_buf1[k] = freq_CH1_smp_buf[k*2+1]; //通道1read_evalute_data
				freq_CH2_smp_buf2[k] = freq_CH2_smp_buf[k*2];//通道2					
			}
			#if debug
			for( j=0;j< 300;j++ )//test////////
			{								
				LOGD( "DOUBLE_CH_滤波前freq_CH1_smp_buf1[%d] = %f" , j , freq_CH1_smp_buf1[j] );		
				LOGD( "DOUBLE_CH_滤波前freq_CH2_smp_buf2[%d] = %f" , j , freq_CH2_smp_buf2[j] );
			}
			#endif
			
			enter_IIR_Filter( freq_CH1_smp_buf1 , g_smpLength/2 ,( int )my_freqwave.max_freq ,( int )my_freqwave.min_freq );  //IIR 滤波长度为 采集长度的一半
			enter_IIR_Filter( freq_CH2_smp_buf2 , g_smpLength/2 ,( int )my_freqwave.max_freq ,( int )my_freqwave.min_freq );
			
			#if debug
			for( j=0;j< 300;j++ )//test////////
			{								
				LOGD( "DOUBLE_CH_滤波后freq_CH1_smp_buf1[%d] = %f" , j , freq_CH1_smp_buf1[j] );		
				LOGD( "DOUBLE_CH_滤波后freq_CH2_smp_buf2[%d] = %f" , j , freq_CH2_smp_buf2[j] );
			}
			#endif			
         							
            memcpy( freq_CH1_IIR_buf , &freq_CH1_smp_buf1[g_discard_pnts] ,( g_waveLength/2 )*sizeof( float ) );
            memcpy( freq_CH2_IIR_buf , &freq_CH2_smp_buf2[g_discard_pnts] ,( g_waveLength/2 )*sizeof( float ) );			
						
			fft_alg_entry2( freq_CH1_IIR_buf , g_waveLength/2 , 0 , 0 , 0  );//0默认不加窗，平均方式，平均次数都为0，不进行			
			fft_alg_entry2( freq_CH2_IIR_buf , g_waveLength/2 , 0 , 0 , 0  );	//0默认不加窗，平均方式，平均次数都为0，不进行		
			
			#if 1
			for( j=0;j< 100;j++ )//test//////// /g_waveLength/4
			{	
			    LOGD( "xin: DOUBLE_CH_freq_CH1_IIR_buf[%d] = %f" , j , freq_CH1_IIR_buf[j] );
				LOGD( "xin: DOUBLE_CH_freq_CH2_IIR_buf[%d] = %f" , j , freq_CH2_IIR_buf[j] );				
			}
			#endif
			
			vibrate_callback_backup.double_ch_callback( freq_CH1_IIR_buf ,  freq_CH2_IIR_buf ,  g_waveLength/4 ,  true );	/////频谱只回调 前一半数据							
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
		set_singleCH_vibrate_reg( fWave.signal_type , fWave.max_freq , fWave.min_freq );//设置单通道采集寄存器	
	if( g_chNum == DOUBLE_CH ) 
		set_doubleCH_vibrate_reg( fWave.signal_type , fWave.max_freq , fWave.min_freq );//设置双通道采集寄存器	

	sem_init( &run_sem ,  0 ,  0 );
	
    g_discard_pnts =  Get_InvalidNum(( int )fWave.max_freq , ( int )fWave.min_freq ); 
   	
    exit_thread_flag = false;	
	post_flag = true;
	stop_smp_flag = 0;	
	g_loop_num =0;
		
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
	float CH_data[1]={0.0};	
	
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
	g_loop_num =0;
	
	pthread_create( &c_id ,  NULL ,  rotation_CH_thread ,  NULL );
	signal( SIGIO ,  read_vibrate_data );
    common_start( );
	return 0;
}


