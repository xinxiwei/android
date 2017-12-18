#define LOG_TAG "spiStub"
#include <hardware/hardware.h>
#include <hardware/imx6q_press.h>
#include <hardware/gpio.h>
#include <hardware/log.h>
#include <hardware/masterspi.h>
#include <hardware/slavespi.h>
#include <hardware/spidev.h>
#include <hardware/errorcode.h>
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

#include <hardware/imx6q_press_config.h>


#define DEVICE_NAME "/dev/mxc_spidev1" //从设备
#define MODULE_NAME "xxxx"
#define MODULE_AUTHOR "gxl@126.com"

/////////////////////////////////////////////////
#define debug        0
#define SIZE_15360   15360
#define SIZE_60K     61440
#define SIZE_PRESS   32768  //压力采样长度

#define  smp_rate  25600   //修改压力采集客户反馈时长不够问题，采样率由102400 改为25600

int fd = -1; //从SPI设备文件
int *psample_buf  = NULL; //最终采样的原始数据

float fail_buf[3] = {20000,20000,20000}; // 压力采集时当寄存器配置失败时和停止采集时，供上面app用
float length_err_buf[3] = {40000,40000,40000}; //用于压力曲线 模式下电时，回调的数据长度不够时，借用的数据通道
float bar_err_flag[3] ={10001,10001,10001};	//压力异常回调时的状态数组 ， 1表示数据有效个数为1

unsigned char read_60K_buf[SIZE_60K] = {0};// 读压力数据buf

volatile bool thread_finished_flag = 0; //表示当前线程完成标识
volatile int  restart_power_on_flag = 0;  // spi 重新上电标识, 0标识需要重新上电
volatile int  start_enable_flag = 0;// 响应start接口标识
volatile int  stop_press_smp_flag = 0; // 点击stop时的停止压力采集标识

volatile int  press_smp_discard_flag =0; //压力采集丢弃前16K数据，解决波形前面不稳bug
volatile float pflag0_value = 0.0; //压力标0 模式值
volatile int stop_psmpflag = 0;//压力停止采集标识

pthread_t  c_id; // 开辟线程 c_id:计算
sem_t   run_sem; //内部信号名  run_sem: 继续运行信号

#define   MAX_SIZE  131072 //单通道最大（32K*1024）*4 = 131072
unsigned char g_max_char_buf[ MAX_SIZE ] ={0};


//设备打开和关闭接口
static int spi_device_open( const struct hw_module_t* module , const char* name , struct hw_device_t** device );
static int spi_device_close( struct hw_device_t* device );

//设备访问接口
static int start_pressure_dial( struct spictl_device_t* dev , float data ); //表盘模式
static int start_pressure_curve( struct spictl_device_t* dev , float data ); //曲线模式
static int start_pressure_flag0( struct spictl_device_t* dev ); //标0模式
static int stop_press_sample( struct spictl_device_t* dev ); //停止压力采集
static int spi_get_freq( struct spictl_device_t* dev ); //获取采样频率
static int spi_set_val( struct spictl_device_t* dev , int val );
static int spi_get_val( struct spictl_device_t* dev , int* val );
struct spictl_device_t* dev;

//模块方法表
static struct hw_module_methods_t spi_module_methods={
   open: spi_device_open
};


//JNI回调接口
static SpiPressureCallbacks    pressure_callback_backup;   //压力回调接口

//模块实例变量
struct spictl_module_t HAL_MODULE_INFO_SYM ={
    common:{
        tag: HARDWARE_MODULE_TAG ,
        version_major: 1 ,
        version_minor: 0 ,
        id: PRESSCTL_HARDWARE_MODULE_ID ,
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


static int spi_device_open( const struct hw_module_t* module , const char* name , struct hw_device_t** device ){
    struct spictl_device_t* dev;
	dev =( struct spictl_device_t* )malloc( sizeof( struct spictl_device_t ) );
	if(dev == NULL)
	{
		LOGD( "imx6q_press: failed to alloc space" );
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
	dev->stop_press_ad = stop_press_sample; //压力停止采集
	dev->spi_freq = spi_get_freq;	//压力采样频率
    dev->get_pressure_interface = spi_press_interface;   //压力回调接口
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
    LOGD( "imx6q_press: set value %d to device." , val );
    //write( dev->fd , &val , sizeof( val ) );
    return 0;
}


static int spi_get_val( struct spictl_device_t* dev , int* val ){
    if( !val ){
        LOGD( "imx6q_press: error val pointer" );
        return -EFAULT;
    }
    //read( dev->fd , val , sizeof( *val ) );
    LOGD( "imx6q_press: get value %d from device" , *val );
    return 0;
}

static int spi_get_freq( struct spictl_device_t* dev ){//获取压力采样率
	return spi_freq( );
}


///////////////////共用函数如下
inline float adc_data_to_24value( int adc_data )// 将采集32位数据 转换为24位有效数据
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

    value = value *10; //乘以10 是因为寄存器默认配置都是25V，硬件电路会衰减10倍，再此*10补上, 和24V激励没有关系
	return value;
}

void flag_init() //对所有标识flag 清0
{
    LOGD("压力flag_init====");
    thread_finished_flag = 0;
    restart_power_on_flag = 0;
    stop_press_smp_flag = 0;
    start_enable_flag = 0;
    stop_psmpflag = 0;
}

void common_start( )//打开设备和开始fpga采集
{
    LOGD("common_start stop_press_smp_ flag = %d ,fd = %d, [%s]\n" ,stop_press_smp_flag ,fd, log_time( ));
    if(fd != -1)
    {
        int res1 = ioctl( fd ,  SPIDEV_IOC_RXSTREAMOFF ,  NULL );  //停止DMA搬运
        if( res1 !=0 ) //启动DMA采集失败时，重新启动
        {
            LOGD( "====打开设备前先停一次，防止前一次有DMA没有关闭， DMA采集-停止失败===== ,fd = %d",fd );
        }else{
            LOGD( "====打开设备前先停一次，防止前一次有DMA没有关闭， DMA采集-停止成功 ,fd = %d",fd );
        }
    }

    if( stop_press_smp_flag == 1) // 停止压力采集标识
	{
        LOGD("common_start打开设备时检测到停止压力采集标识,不再继续打开设备文件,直接return出去");
		return ;
    }

    if(( fd = open( DEVICE_NAME , O_RDWR ) ) == -1 ){
        LOGD( "common_start打开从spi设备 /dev/mxc_spidev1 失败 -- %s." , strerror( errno ) );
    }else{
		LOGD( "common_start打开从spi设备 /dev/mxc_spidev1 成功. fd = %d" , fd );
	}

    fcntl( fd ,  F_SETOWN ,  getpid( ) );//将当前进程PID设置为fd文件所对应驱动程序将要发送SIGIO,SIGUSR信号进程PID
    int oflags = fcntl( fd ,  F_GETFL );//获取fd的打开方式
    fcntl( fd ,  F_SETFL ,  oflags | FASYNC );//将fd的打开方式设置为FASYNC --- 即 支持异步通知

    usleep(200000); //用于 stop_press_smp_ flag 和 fd 的在停止时的变化能够生效，防止后面多一次打开DMA ,重要！！！！
    if( stop_press_smp_flag == 1) // 停止压力采集标识
	{
        LOGD("common_start启动DMA时检测到停止压力采集标识,不再继续打开DMA，直接return出去");
        return ;
	}else{
        if( fd != -1)
        {
            int res = ioctl( fd ,  SPIDEV_IOC_RXSTREAMON ,  NULL );//启动DMA采集
            if( res !=0 ) //启动DMA采集失败时，重新启动
            {
                LOGD( "common_startDMA采集-启动失败===== ,fd = %d, [%s]\n",fd ,log_time( ) );
            }else{
                LOGD( "common_startDMA采集-启动成功 ,fd = %d, [%s]\n",fd ,log_time( ) );
            }
        }
        swrite( StartSampleAddr , StartSampleData );// 启动FPGA采集板
    }
}

inline void stop_fpga_dma()//停止FPGA采集, DMA搬运，关闭设备文件
{
    if(fd != -1)
    {
        LOGD( "stop_fpga-dma停止FPGA,DMA stop_press_smp_ flag = %d,  fd = %d, is : [%s]\n" ,stop_press_smp_flag,  fd, log_time( ) );
        int res = ioctl( fd ,  SPIDEV_IOC_RXSTREAMOFF ,  NULL );  //停止DMA搬运
        if( res !=0 ) //启动DMA采集失败时，重新启动
        {
            LOGD( "stop_fpga-dma停止FPGA DMA: DMA采集-停止失败===== ,fd = %d",fd );
        }else{
            LOGD( "stop_fpga-dma停止FPGA DMA: DMA采集-停止成功 ,fd = %d",fd );
        }
        if( close( fd ) == 0)//关闭从设备文件
        {
            fd = -1; //关闭成功重新初始fd
            LOGD("stop_fpga-dma关闭从SPI设备文件成功后  重新初始化fd为 %d",fd);
        }
        swrite( StopSampleAddr , StopSampleData );//stop fpga
    }

	if( stop_press_smp_flag == 1) // 停止压力采集标识
	{
        LOGD("stop_fpga-dma停止FPGA,DMA时测到停止压力采集标识，提前下电并post信号量");
        //poweroff_spi(); //停止时下电，会使后台自启动时寄存器失败，从而快速回调寄存器失败异常，这样会造成上层APP收到寄存器失败后变成开始采集，所以去掉此代码
        sem_post( &run_sem );
        return ;
	}    
}

static int stop_press_sample( struct spictl_device_t* dev )//停止压力采集
{
	LOGD( "停止压力采集==开始 fd = %d, stop_press_smp_ flag = %d, start_enable_ flag = %d is : [%s]\n" ,fd,  stop_press_smp_flag,  start_enable_flag, log_time( ) );
    
    stop_press_smp_flag = 1;
	if(start_enable_flag == 0)
	{
		LOGD("停止压力采集,此时start线程刚结束，无线程存在,直接回调false给上层");
		usleep(500000); //用于等待stop_press_smp_ flag 变量变化，防止在还没有关完前，重新响应start
        if(fd != -1)
        {
            stop_fpga_dma();
            poweroff_spi( );  //下电
        }

        flag_init();
		pressure_callback_backup.stop_press_callback( false );
        return 0;
	}

	if(stop_press_smp_flag == 1)
    {
        LOGD("停止压力采集中stop_press_smp_ flag = 1，立即停止采集，调用stop_fpga_ dma函数");
        stop_psmpflag = 1;
	    stop_fpga_dma();
    }
	return 0;
}

///////////////////压力采集功能如下
int p_loop_num =0;
void read_press_data( int signo ) // 读取压力adc 采样数据
{
    LOGD( "读取压力数据时stop_ psmpflag = %d,p_loop_num = %d,  stop_press_smp_ flag =%d  , [%s]\n" ,  stop_psmpflag, p_loop_num,  stop_press_smp_flag , log_time( ) );
  
    if( stop_psmpflag )
    {
        return;
    }

    int i = 0;
    int total_len = SIZE_PRESS;  //采样总点数
    int shang = total_len/SIZE_15360;  //15360 的倍数
    int yu = total_len%SIZE_15360;  // 余数
    if( shang > 0 && yu > 0 )
    {
        if( read( fd , read_60K_buf , SIZE_60K ) <0 )
        {
            LOGD( "Error: spi slave device read fail !\n " );
        }

        if( p_loop_num < shang )
        {
            memcpy( &g_max_char_buf[SIZE_60K*p_loop_num] ,  &read_60K_buf ,  SIZE_60K );
            p_loop_num++;
        }

        if(p_loop_num == 1)
        {
            if(!press_smp_discard_flag)
            {
                p_loop_num = 0;
                press_smp_discard_flag ++;
            }
        }

        if( p_loop_num == shang )
        {
            if( press_smp_discard_flag >= 1) //表示丢前一波数据
            {
                stop_psmpflag = 1; // flag置1 ，停止继续读数据
                stop_fpga_dma(); //停止DMA采集
                memcpy( &g_max_char_buf[SIZE_60K*shang] ,  &read_60K_buf ,  yu*sizeof( float ) );
                psample_buf =( int* )&( g_max_char_buf ) ;

                LOGD( "post压力信号量 : [%s]\n" ,  log_time( ) );
                sem_post( &run_sem );
                press_smp_discard_flag =0;
            }
        }
    }
}

void *press_flag0_thread( void* arg ) //压力标0模式
{
	LOGD( "进入压力标0线程 [%s]\n" ,  log_time( ) );
    int i=0;
    float press_flag0_value[2] ={0.0}; //标0模式时，回传的数据 ,  0位：表示数据值，1位：表示有效数据个数
    float f64_sum = 0.0; //64K数据总和

	sem_wait( &run_sem );//等待信号量
    
	f64_sum = 0.0;

	for( i=0;i<SIZE_PRESS;i++ )
	{
	    f64_sum += adc_data_to_24value( psample_buf[i] ) ;
	}
	press_flag0_value[0] = f64_sum / SIZE_PRESS;	//总和求平均值
	press_flag0_value[1] = 1;

	LOGD( "计算出压力标0值 =%f" , press_flag0_value[0] );

	poweroff_spi( );  //下电
    thread_finished_flag = 1;
    
    flag_init();   
    LOGD( "压力标0线程正常回调数据 [%s]\n" ,  log_time( ) );
	pressure_callback_backup.mspictl_callback( press_flag0_value , true );//用于标0值回调
    return NULL;
}

void *press_curve_thread( void* arg ) //压力曲线模式
{
    LOGD( "进入压力曲线线程 [%s]\n" ,  log_time( ) );
	int i=0;
    float final_buf[SIZE_PRESS] ={0.0};
	float positive_buf[SIZE_PRESS] ={0.0}; //压力数据全部转换为正数
	float ret_value[SIZE_PRESS+3] ={0.0};  //回调返回的数据

	float value_bar_max2 = 0.0;
    float value_bar_min2 = 0.0;
	float tmp1=0.0 , tmp2=0.0 , tmp3=0.0;

    if(stop_press_smp_flag == 1)
    {
        LOGD("压力曲线线程，进入第一时间，检测到停止压力采集标识，不经过算法，调用stop_DMA_FPGA后，初始化flag变量，回调停止采集数据给上层");
		goto  stop_daq;
    }

	sem_wait( &run_sem );//等待信号量

	if(stop_press_smp_flag == 1)
	{
		LOGD("接收信号量后，压力曲线运算线程第一时间检测到停止压力采集标识,不经过算法，调用stop_DMA_FPGA后，初始化flag变量，回调停止采集数据给上层");
		goto  stop_daq;
	}
	memset( final_buf , 0 , SIZE_PRESS*sizeof( float ) );
	memset( positive_buf , 0 , SIZE_PRESS*sizeof( float ) );
	memset( ret_value , 0 , (SIZE_PRESS+3)*sizeof( float ) );

	for( i=0;i<SIZE_PRESS;i++ )
	{
		final_buf[i] = adc_data_to_24value( psample_buf[i] );
		positive_buf[i] =( float )fabs( final_buf[i] );    //采集数据 取绝对值全部转为正数
	}

	value_bar_max2 = positive_buf[0]; //假设最大最小值都是数组0值
	value_bar_min2 = positive_buf[0];
	for( i=1;i<SIZE_PRESS;i++ )
	{
		if( value_bar_max2 <= positive_buf[i] )
			value_bar_max2= positive_buf[i];  //对原始采集数据求最大值 ，用于后面0.5bar 判定条件，若最大值小于0.5bar , 则不进行后续算法计算，直接提示异常

		if( value_bar_min2 >= positive_buf[i] )
			value_bar_min2= positive_buf[i];  //对原始采集数据求最小值
	}
	tmp1 = fabs(  fabs( pflag0_value ) - value_bar_max2 ); //求标0值和最大最小值差值的绝对值
	tmp2 = fabs(  fabs( pflag0_value ) - value_bar_min2 );
	tmp3 =( tmp1 > tmp2 )?tmp1:tmp2;	   	//求幅值变化最大值

	if( tmp3  <= 0.01 )  //0.5bar = 0.01V  // 20mv/bar  表示外部环境和气缸环境 相差很小，此时表示空采
	{
		LOGD( "press_curve_thread is <0.5bar status,回调10001 ，数据" );
		poweroff_spi();  //下电
		thread_finished_flag = 1;
		
		flag_init();
        LOGD("正常回调压力曲线数据小于0.5bar 数据  [%s]\n" ,  log_time( ) );
		pressure_callback_backup.mspictl_callback( bar_err_flag ,  false );//小于0.5bar
		return NULL;
	}
	else if( tmp3 >0.01 )
	{
		LOGD( "press_curve_thread is >0.5bar status" );
		press_alg_entry( final_buf , SIZE_PRESS ,  ret_value );//压力曲线模式算法

		if(stop_press_smp_flag == 1)//检测到停止压力采集标识为1
		{
			LOGD("压力曲线线程检测到停止压力采集标识为1，下电，销毁信号量，回调false给上层 [%s]\n" ,  log_time( ) );
		    goto stop_daq;
		}else{
            thread_finished_flag = 1;
                     
            flag_init();
            if((int)ret_value[1] < 100)
            {
			    LOGD("压力曲线线程异常回调数据  is : [%s]\n" ,  log_time( ) );
                pressure_callback_backup.mspictl_callback( length_err_buf ,  false );//用于回调长度小于100时
            }else
            {
                restart_power_on_flag =1;
				LOGD("压力曲线线程正常回调数据 restart_power_on_flag =%d  is : [%s]\n" , restart_power_on_flag, log_time( ) );
                pressure_callback_backup.mspictl_callback( ret_value ,  true );//用于压力曲线数据回调
            }
            return NULL;
		}
    }

	stop_daq:
        {
            usleep(100000); 
            if(fd != -1)
            {
                stop_fpga_dma();
            }
            poweroff_spi();
        
            thread_finished_flag = 1;
        
            flag_init();
            LOGD("压力曲线线程异常回调数据  is : [%s]\n" ,  log_time( ) );
            pressure_callback_backup.mspictl_callback( fail_buf , false );//用于停止采集时的异常回调
            return NULL;
        }
    return NULL;
}

void *press_dial_thread( void* arg ) //压力表盘模式
{
	LOGD( "进入压力表盘线程 [%s]\n" ,  log_time( ) );
    int i=0;
    float dial_value[2]={0.0}; //表盘模式时，回传的数据 ,  0位：表示数据值，1位：表示有效数据个数

	float final_buf[SIZE_PRESS] ={0.0};
	float positive_buf[SIZE_PRESS] ={0.0};

    float f64_sum = 0.0;
	float value_bar_max1 = 0.0;
    float value_bar_min1 = 0.0;
	float tmp1=0.0 , tmp2=0.0 , tmp3=0.0;

    if(stop_press_smp_flag == 1)
    {
        LOGD("压力表盘线程，进入第一时间，检测到停止压力采集标识，不经过算法，调用stop_DMA_FPGA后，初始化flag变量，回调停止采集数据给上层");
		goto  stop_daq;
    }

	sem_wait( &run_sem );//等待信号量
        
    if(stop_press_smp_flag == 1)//检测到停止压力采集标识为1
	{
		LOGD("接收信号量后，压力表盘运算线程，第一时间检测到停止压力采集标识,不经过算法，调用stop_DMA_FPGA后，初始化flag变量，回调停止采集数据给上层");
		goto  stop_daq;
	}
	memset( final_buf , 0 , SIZE_PRESS*sizeof( float ) );
	memset( positive_buf , 0 , SIZE_PRESS*sizeof( float ) );
	f64_sum = 0.0;

	for( i=0;i<SIZE_PRESS;i++ )
	{
		final_buf[i] = adc_data_to_24value( psample_buf[i] );
		f64_sum += final_buf[i];
   		positive_buf[i] =( float )fabs( final_buf[i] );
	}

	value_bar_max1 = positive_buf[0];
	value_bar_min1 = positive_buf[0];
	for( i=1;i<SIZE_PRESS;i++ )
	{
		if( value_bar_max1 <= positive_buf[i] )
			value_bar_max1= positive_buf[i];  //对原始采集数据求最大值 ，用于后面0.5bar 判定条件，若最大值小于0.5bar , 则不进行后续算法计算，直接提示异常

		if( value_bar_min1 >= positive_buf[i] )
			value_bar_min1= positive_buf[i];  //对原始采集数据求最小值
	}

	tmp1 = fabs(  fabs( pflag0_value ) - value_bar_max1 );
	tmp2 = fabs(  fabs( pflag0_value ) - value_bar_min1 );
	tmp3 =( tmp1 > tmp2 )?tmp1:tmp2;

	if( tmp3  <= 0.01 )  //0.5bar = 0.5*20mv = 10mv = 0.01V  //外部环境和气缸环境 相差很小，此时表示空采
	{
		LOGD( "press_dial_thread is <0.5bar status" );
		poweroff_spi();
		thread_finished_flag = 1;
		
		flag_init();
        LOGD("正常回调压力表盘数据小于0.5bar 数据  [%s]\n" ,  log_time( ) );
		pressure_callback_backup.mspictl_callback( bar_err_flag ,  false );//小于0.5bar
		return NULL;
	}
	else if( tmp3 > 0.01 )
	{
		LOGD( "press_dial_thread is >0.5bar status" );
		dial_value[0] = f64_sum / SIZE_PRESS;   //表盘模式算法是对所采数据求平均
		dial_value[1] = 1;
		LOGD( "压力表盘计算值dial_value =%f\n" ,  dial_value[0] );

		if(stop_press_smp_flag == 1)
		{
			LOGD("压力表盘线程检测到停止压力采集标识为1，调用stop_DMA_FPGA后，初始化flag变量，回调停止采集数据给上层");
            goto stop_daq;
		}else{
            thread_finished_flag = 1;
            
            flag_init();            
            restart_power_on_flag =1;
			LOGD("压力表盘数据线程正常回调数据  restart_power_on_flag = %d is : [%s]\n" , restart_power_on_flag, log_time( ) );
			pressure_callback_backup.mspictl_callback( dial_value ,  true );//用于表盘值回调
            return NULL;
		}
    }

	stop_daq:
		{
            usleep(100000); 
		    if(fd != -1)
            {
                stop_fpga_dma();
            }
            poweroff_spi();
           
			thread_finished_flag = 1;
            
            flag_init();
            LOGD("压力表盘数据线程异常回调数据  is : [%s]\n" ,  log_time( ) );
			pressure_callback_backup.mspictl_callback( fail_buf , false );//用于停止采集时的异常回调
			return NULL;
		}
    return NULL;
}

static int start_pressure_flag0( struct spictl_device_t* dev )// 压力标0模式
{
    memset(read_60K_buf,0,SIZE_60K*sizeof(unsigned char));
    memset(g_max_char_buf ,0, MAX_SIZE*sizeof(unsigned char));
    p_loop_num = 0;
    stop_psmpflag = 0;

    poweron_spi(  );
    int reg_ret_value = set_press_reg( smp_rate );	//设置压力采集寄存器
	if( reg_ret_value == -1)
	{
        LOGD("寄存器配置失败,回调寄存器失败数据20000");
		flag_init();
		poweroff_spi(  );

		pressure_callback_backup.mspictl_callback( fail_buf ,  false );//表示寄存器配置失败，回调 false给上层
		return 0;
	}

	usleep(2000000); //用于上层时域波形不丢波形数据，在这里直接延时2S,让硬 件预热稳定

	sem_init( &run_sem ,  0 ,  0 );
	pthread_create( &c_id ,  NULL ,  press_flag0_thread ,  NULL );

	signal( SIGIO ,  read_press_data );// 捕捉异步IO信号，并安装信号处理函数
    common_start( );

    while( thread_finished_flag == 0); // 用于等待算法线程线束时置1，若线程结束时，继续往下运行
	
    usleep( 100000 );
	LOGD("响应压力标0点击结束******************** [%s]\n" ,  log_time( ));
	return 0;
}

static int start_pressure_curve( struct spictl_device_t* dev , float flag0_value )  //压力曲线模式
{
	LOGD("点击压力曲线开始时restart_power_on_ flag= %d, thread_finished_ flag = %d",restart_power_on_flag,thread_finished_flag);
    if(start_enable_flag == 1) 
	{
		LOGD("start_enable_ flag = 1, 前一个start还未执行完，此时不响应新的start接口，直接return出去");
        return 0;
	}else{
	    start_enable_flag = 1;
	}

    if(stop_press_smp_flag ==1)
    {
        LOGD("点击压力曲线开始时stop_press_smp_ flag = %d,   [%s]\n",stop_press_smp_flag,log_time( ));
        stop_press_smp_flag = 0;
        return 0;
    }
    memset(read_60K_buf,0,SIZE_60K*sizeof(unsigned char));
	memset(g_max_char_buf ,0, MAX_SIZE*sizeof(unsigned char));
    p_loop_num = 0;
    stop_psmpflag = 0;
    pflag0_value = flag0_value;

    if(restart_power_on_flag == 1) // 当内部stop DMA，fpga,关闭设备后，flag 置1， 当再启动start接口时不再重新上电，当对外大的停止采集下电后，此flag会置为 0，重新采集时再上电
	{
		restart_power_on_flag = 0;
	}else{
		poweron_spi(  );
		int reg_ret_value = set_press_reg( smp_rate );	//配置压力采集寄存器
		if( reg_ret_value == -1)
		{
            LOGD("寄存器配置失败,回调寄存器失败数据20000");
			flag_init();
            poweroff_spi(  );

			pressure_callback_backup.mspictl_callback( fail_buf ,  false );//表示寄存器配置失败，回调 false给上层
			return 0;
		}
        usleep(2000000); //用于每次上电后，上层时域波形不丢波形数据，在这里直接延时2S,让硬 件预热稳定
	}
	sem_init( &run_sem ,  0 ,  0 );   //初始化信号量

	pthread_create( &c_id ,  NULL ,  press_curve_thread ,  NULL );
    signal( SIGIO ,  read_press_data );
    common_start( );

	while( thread_finished_flag == 0); // 用于等待算法线程线束时置1，若线程结束时，继续往下运行
	
	usleep( 100000 );
	LOGD( "点击压力曲线结束***** [%s]\n" ,  log_time( ) );
	return 0;
}

static int start_pressure_dial( struct spictl_device_t* dev , float flag0_value )// 压力表盘模式
{
	LOGD("点击表盘开始时  restart_power_on_ flag= %d,thread_finished_ flag = %d",restart_power_on_flag,thread_finished_flag);
    if(start_enable_flag == 1)
	{
		LOGD("start_enable_ flag = 1, 前一个start还未执行完，此时不响应新的start接口，直接return出去");
        return 0;
	}else{
	    start_enable_flag = 1;
	}
    
    if(stop_press_smp_flag ==1)
    {
        LOGD("点击压力表盘开始时stop_press_smp_ flag = %d,   [%s]\n",stop_press_smp_flag,log_time( ));
        stop_press_smp_flag = 0;
        return 0;
    }

    memset(read_60K_buf,0,SIZE_60K*sizeof(unsigned char));
	memset(g_max_char_buf ,0, MAX_SIZE*sizeof(unsigned char));
    p_loop_num = 0;
    stop_psmpflag = 0;
    pflag0_value = flag0_value;

    if(restart_power_on_flag == 1) // 当内部stop DMA，fpga,关闭设备后，flag 置1， 当再启动start接口时不再重新上电，当对外大的停止采集下电后，此flag会置为 0，重新采集时再上电
	{
		restart_power_on_flag = 0;
	}else{
		poweron_spi(  );
		int reg_ret_value = set_press_reg( smp_rate );	//设置压力采集寄存器
		if( reg_ret_value == -1)
		{
            LOGD("寄存器配置失败,回调寄存器失败数据20000");
			flag_init();

            poweroff_spi(  );
			pressure_callback_backup.mspictl_callback( fail_buf ,  false );//表示寄存器配置失败，回调 false给上层
			return 0;
		}
        usleep(2000000); //用于每次上电后，上层时域波形不丢波形数据，在这里直接延时2S,让硬 件预热稳定
	}
    
	sem_init( &run_sem ,  0 ,  0 );   //初始化信号量

	pthread_create( &c_id ,  NULL ,  press_dial_thread ,  NULL );
    signal( SIGIO ,  read_press_data );
    common_start( );

	while( thread_finished_flag == 0); // 用于等待算法线程线束时置1，若线程结束时，继续往下运行
	
	usleep( 100000 );
	LOGD( "点击压力表盘结束 [%s]\n" ,  log_time( ) );
	return 0;
}
