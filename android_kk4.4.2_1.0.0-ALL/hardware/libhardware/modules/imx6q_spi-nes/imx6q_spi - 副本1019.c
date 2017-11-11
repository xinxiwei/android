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

#define DEVICE_NAME "/dev/mxc_spidev1" //���豸
#define MODULE_NAME "xxxx"
#define MODULE_AUTHOR "gxl@126.com"

/////////////////////////////////////////////////
#define debug      0
#define SIZE_15360  15360
#define SIZE_60K   61440
#define SIZE_64K   65536
#define SIZE_4K    4096
#define SIZE_16K   16384

#define FEATURE_NUM    15  //����ֵ����

int smp_rate = 102400;//������HZ
int oflags = 0;
int fd = -1; //��SPI�豸�ļ�
int *psample_buf  = NULL; //���ղ�����ԭʼ����
int *g_smp_buf = NULL;  //ȫ�ֲɼ�������
float invalid_buf[2] = {0.0}; // power offʱ�ص��ļ�buf,���ӿ�ʹ�ã�ʵ��������

float vib_reg_fail_buf[1] = {10001}; // �񶯲ɼ�ʱ���Ĵ�������ʧ��ʱ��������app��
float press_reg_fail_buf[3] = {40000,40000,40000}; // ѹ���ɼ�ʱ���Ĵ�������ʧ��ʱ��������app��
float status_flag[2] ={10000.0,1};	//ѹ���쳣�ص�ʱ��״̬���� �� 1��ʾ������Ч����Ϊ1

int g_loop_num = 0; //�ײ㷴���ĵڼ�������
int g_ret_val = 0;  //�豸�ļ��رշ���ֵ

unsigned char read_60K_buf[SIZE_60K] = {0};// ��ѹ������buf
unsigned char press_buf[SIZE_64K] = {0};//�ɼ�����ѹ������
float feature_ret_value[FEATURE_NUM] = {0.0}; //15������ֵ

volatile bool thread_finished_flag = 0; //��ʾ��ǰ�߳���ɱ�ʶ
volatile bool post_flag = true;  // ����post �ź�����ʶ
volatile bool exit_thread_flag = false; //�˳��̱߳�־
volatile int  stop_smp_flag = 0;//ֹͣ�ɼ����� ��ʶ
volatile int  restart_power_on_flag = 0;  // spi �����ϵ��ʶ
volatile int  power_off_flag = 0; // ���stopʱ���µ��ʶ
volatile int  can_start_flag  = 0; //���ڿ��ٵ��ʱ��dmaû��ִ����
volatile int  start_enable_flag = 0;// ��Ӧstart�ӿڱ�ʶ
volatile int  press_discard_16K_flag =0; //ѹ���ɼ�����ǰ16K���ݣ��������ǰ�治��bug
volatile int  press_flag0_flag = 0 ;  //����ѹ����0 ����ǰ10������

volatile float pflag0_value = 0.0; //ѹ����0 ģʽֵ

volatile int start_num = 0; // �����ж�stop ��ɺ󣬲ſ�����Ӧstart

pthread_t  c_id; // �����߳� c_id:����
sem_t   run_sem; //�ڲ��ź���  run_sem: ���������ź�


//�豸�򿪺͹رսӿ�
static int spi_device_open( const struct hw_module_t* module , const char* name , struct hw_device_t** device );
static int spi_device_close( struct hw_device_t* device );

//�豸���ʽӿ�
static int start_pressure_dial( struct spictl_device_t* dev , float data ); //����ģʽ
static int start_pressure_curve( struct spictl_device_t* dev , float data ); //����ģʽ
static int start_pressure_flag0( struct spictl_device_t* dev ); //��0ģʽ

static int start_vibrate_CH_timewave( struct spictl_device_t* dev ,   int ch_num , struct time_wave_para tWave );//�����񶯲ɼ� ʱ����
static int start_vibrate_CH_freqwave( struct spictl_device_t* dev ,   int ch_num , struct freq_wave_para fWave );//�����񶯲ɼ� Ƶ����
static int start_vibrate_CH_totalrend( struct spictl_device_t* dev ,   int ch_num , struct total_rend_para tRend );//�����񶯲ɼ� ��ֵ���Ʋ���
static int start_rotation_CH( struct spictl_device_t* dev );//ת��ͨ��
static int start_vibrate_evalute_level( struct spictl_device_t* dev , struct time_wave_para tWave );//�����񶯵ȼ�����

static int stop_vibrate_sample( struct spictl_device_t* dev ); //ֹͣ�񶯲ɼ�
static int stop_press_sample( struct spictl_device_t* dev ); //ֹͣѹ���ɼ�
static int spi_get_freq( struct spictl_device_t* dev ); //��ȡ����Ƶ��

static float* spi_get_feature_value( struct spictl_device_t* dev , float pData[] , int data_len ); //��ȡ15������ֵ�ӿ�
static float* spi_time_to_freq_value( struct spictl_device_t* dev , float pData[] , int data_len );  //ʱ��Ƶ��ӿ�

static int spi_set_val( struct spictl_device_t* dev , int val );
static int spi_get_val( struct spictl_device_t* dev , int* val );

struct spictl_device_t* dev;

//ģ�鷽����
static struct hw_module_methods_t spi_module_methods={
   open: spi_device_open
};

//JNI�ص��ӿ�
static SpiPressureCallbacks    pressure_callback_backup;   //ѹ���ص��ӿ�
static SpiVibrateCallbacks     vibrate_callback_backup;  //�񶯻ص��ӿ�

//ģ��ʵ������
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

const SpiPressureInterface* spi_press_interface( struct spictl_device_t* dev ){ //ѹ���ص��ӿ�
    return &mSpiPressInterface;
}

static void initVibrate( SpiVibrateCallbacks* callbacks ){
	vibrate_callback_backup = *callbacks;
}

static const SpiVibrateInterface  mSpiVibrateInterface = {
	initVibrate , 
};

const SpiVibrateInterface* spi_vibrate_interface( struct spictl_device_t* dev ){//�񶯻ص��ӿ�
    return &mSpiVibrateInterface;
}

static int spi_device_open( const struct hw_module_t* module , const char* name , struct hw_device_t** device ){
	LOGD("\n\nxin: imx6q_spi_default.so-�汾��Ϣ  = 20170916.0930"); //.so �汾��Ϣ
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
	
    dev->start_press_dial = start_pressure_dial; //����ģʽ
	dev->start_press_curve = start_pressure_curve; //����ģʽ
	dev->start_press_flag0 = start_pressure_flag0; //��0ģʽ
	
	dev->start_vibrate_timewave = start_vibrate_CH_timewave;  //ʱ����
	dev->start_vibrate_freqwave = start_vibrate_CH_freqwave;  //Ƶ����
	dev->start_vibrate_totalrend = start_vibrate_CH_totalrend;	//��ֵ����
	dev->start_vibrate_evalute = start_vibrate_evalute_level; //�ȼ�����
	dev->start_rotation = start_rotation_CH; //ת��
	
	dev->stop_vibrate_ad = stop_vibrate_sample; //��ֹͣ�ɼ�
	dev->stop_press_ad = stop_press_sample; //ѹ��ֹͣ�ɼ�
	
	dev->spi_freq = spi_get_freq;	//ѹ������Ƶ��
	dev->spi_feature_value = spi_get_feature_value;	 //15������ֵ
	dev->spi_timetofreq_value = spi_time_to_freq_value;	//ʱ��Ƶ��ӿ�
	
    dev->get_pressure_interface = spi_press_interface;   //ѹ���ص��ӿ�
	dev->get_vibrate_interface = spi_vibrate_interface; //�񶯻ص��ӿ�
	
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
        LOGD( "imx6q_spi:?error?val?pointer" );
        return -EFAULT;
    }
    //read( dev->fd , val , sizeof( *val ) );
    LOGD( "imx6q_spi: get value %d from device" , *val );
    return 0;
}

static int spi_get_freq( struct spictl_device_t* dev ){//��ȡѹ��������    
	return spi_freq( );
}


///////////////////���ú�������
void spi_power_off() //spi�豸�µ�
{	   
	poweroff_spi( );  //spi�µ�	    
}

static int wdma_num =0; //����ʹ��DMA����
void common_start( )//���豸�Ϳ�ʼfpga�ɼ�
{
	int res;    
    if(( fd = open( DEVICE_NAME , O_RDWR ) ) == -1 ){
        LOGD( "xin: �򿪴�spi�豸 /dev/mxc_spidev1 ʧ�� -- %s." , strerror( errno ) );        		
    }else{		
		LOGD( "xin: �򿪴�spi�豸 /dev/mxc_spidev1 �ɹ�. fd = %d" , fd );
	}	
	
    //����ǰ����PID����Ϊfd�ļ�����Ӧ��������Ҫ����SIGIO,SIGUSR�źŽ���PID  
    fcntl( fd ,  F_SETOWN ,  getpid( ) );
    //��ȡfd�Ĵ򿪷�ʽ  
    oflags = fcntl( fd ,  F_GETFL );
    //��fd�Ĵ򿪷�ʽ����ΪFASYNC --- �� ֧���첽֪ͨ 
    fcntl( fd ,  F_SETFL ,  oflags | FASYNC );
    
    res = ioctl( fd ,  SPIDEV_IOC_RXSTREAMON ,  NULL );//����DMA�ɼ�		
	
    if( res !=0 ) //����DMA�ɼ�ʧ��ʱ����������
	{		
        LOGD( "imx6q_spi: can't set spi mode" );
		while((res != 0) || ( wdma_num < 100))
		{
			usleep(100000); //��DMA�ȶ�����һ����ʱ����ͨ��
			res = ioctl( fd ,  SPIDEV_IOC_RXSTREAMON ,  NULL );			
			wdma_num ++;
		}
    }
	if( wdma_num >= 100)
	{		
		wdma_num = 0;
		vibrate_callback_backup.single_ch_callback( vib_reg_fail_buf ,1 ,true);
		start_enable_flag = 0;
		spi_power_off(  );
	}
	
	wdma_num = 0;
    //usleep(100000); ///��DMA�ȶ�����һ����ʱ����ͨ��
	
	swrite( StartSampleAddr , StartSampleData );// ����FPGA�ɼ���	
} 

void stop_fpga_dma()//ֹͣFPGA�ɼ�, DMA���ˣ��ر��豸�ļ�
{	
	LOGD( "xin: DMA������ʼ power_off_flag = %d, restart_power_on_flag = %d,is : [%s]\n" ,power_off_flag, restart_power_on_flag,  log_time( ) );   
	stop_smp_flag = 0;	//��Ҫ
	press_flag0_flag = 0; // ��ʱ��ѹ����0Ҫ����10��������0
	ioctl( fd ,  SPIDEV_IOC_RXSTREAMOFF ,  NULL );  //ֹͣDMA���� 	
	swrite( StopSampleAddr , StopSampleData );//stop fpga		
	
	if(  power_off_flag == 1) //�µ��ʶΪ1ʱ����ʼ�µ�
	{
		spi_power_off(); 	
	}
	
	LOGD("xin: ֹͣFPGA�ɼ�,DMA����");	
	
	if(  power_off_flag == 1) // �µ��ʶ
	{	
		restart_power_on_flag = 0;	// �´βɼ������ϵ��ʶ 0��ʾ����ɼ�Ҫ�����ϵ�
	}else{
		restart_power_on_flag = 1;	 
	}
		
	if( close( fd ) == 0)//�رմ��豸�ļ�	
	{		
		fd = -1; //�رճɹ����³�ʼfd
		LOGD("xin: �رմ�SPI�豸�ļ��ɹ�");
	}
	
	LOGD( "xin: DMA������� power_off_flag = %d, restart_power_on_flag = %d,is : [%s]\n" ,power_off_flag, restart_power_on_flag,  log_time( ) );
    start_num--; //��ʱ ���ݼ�1
}

static int stop_press_sample( struct spictl_device_t* dev )//ֹͣѹ���ɼ�
{	
	LOGD( "xin: ֹͣѹ���ɼ�==��ʼ power_off_flag = %d, restart_power_on_flag = %d,start_enable_flag =%d is : [%s]\n" , power_off_flag, restart_power_on_flag, start_enable_flag, log_time( ) );
	
	if(start_enable_flag == 0)//��������Ӧstart �߳�flag =0ʱ����ʾ��ʱ�������κ�start�߳�״̬,��ǰ�ص�
	{		
		spi_power_off( );  //�µ�
		pressure_callback_backup.stop_press_callback( false );		
		
        start_enable_flag = 0;	
		power_off_flag = 0;
		restart_power_on_flag = 0;
		can_start_flag = 0;
		start_num = 0; //��start�߳�ʱ����0
		LOGD("xin: ѹ���ɼ���ʱstart�̸߳ս��������̴߳���,�ص�false���ϲ�");
        return 0;		
	}	

	    start_enable_flag = 0;
        power_off_flag = 1;		
		restart_power_on_flag = 0; //stopʱ��Ϊ0 ��������Ӧstartʱ�����ϵ�
		
		LOGD( "xin: ֹͣѹ���ɼ�==���� power_off_flag = %d, restart_power_on_flag = %d,start_enable_flag =%d is : [%s]\n" , power_off_flag, restart_power_on_flag, start_enable_flag, log_time( ) );
	
	return 0;	
}

static int stop_vibrate_sample( struct spictl_device_t* dev )//ֹͣ�񶯲ɼ����Ὣpower_off_flag ��Ϊ1����Ϊ1��stop_fpga_dmaʱ���µ�
{   
    LOGD( "xin: ֹͣ�񶯲ɼ���ʼ power_off_flag = %d, restart_power_on_flag = %d,start_enable_flag =%d is : [%s]\n" , power_off_flag, restart_power_on_flag, start_enable_flag, log_time( ) );
	if(start_enable_flag == 0)//��������Ӧstart �߳�flag =0ʱ����ʾ��ʱ�������κ�start�߳�״̬,��ǰ�ص�
	{		
		spi_power_off( );  //�µ�
		vibrate_callback_backup.stop_ch_callback( false); 		
		
        start_enable_flag = 0;	
		power_off_flag = 0;
		restart_power_on_flag = 0;
		can_start_flag = 0;
		start_num = 0;//��start�߳�ʱ����0
		LOGD("xin: �񶯲ɼ���ʱstart�̸߳ս��������̴߳���,�ص�false���ϲ�");
        return 0;		
	}
	
	    start_enable_flag = 0;
        power_off_flag = 1;		
		restart_power_on_flag = 0; //stopʱ��Ϊ0 ��������Ӧstartʱ�����ϵ�
		
		LOGD( "xin: ֹͣ�񶯲ɼ����� power_off_flag = %d, restart_power_on_flag = %d,start_enable_flag =%d is : [%s]\n" , power_off_flag, restart_power_on_flag, start_enable_flag, log_time( ) );
	return 0;
}


///////////////////adc������ȡ�㷨
inline float adc_data_to_24value( int adc_data )// ���ɼ�32λ���� ת��Ϊ24λ��Ч����
{
    bool highbit_flag = false;
    float value = 0.0;
    adc_data &= 0x00ffffff;//24λ����
    highbit_flag =( bool )(( adc_data>>23 )&0x1 ); //���λ����λ
    adc_data &= 0x007fffff;
    if( highbit_flag )		
    {	       
        value =(float)(( adc_data-0x800000 )*2.5 )/0x7fffff ;//�ֶκ���
		value = value *10; //����10 ����Ϊ�Ĵ���������25V��Ӳ����·��˥��10�����ٴ�*10���� 
    }
    else
    {	    
        value =(float)( adc_data*2.5 )/0x7fffff;
		value = value *10; 
    }		
	return value;
}

void analyze_CH_data( int adc_data ,  float *value )//��������ͨ��������
{		
	int higetst_bit =( adc_data >> 31 )&0x1;
	int channel_bit =( adc_data >> 30 )&0x1;
	
	if( higetst_bit == 0x1 ) //1��ʾת������
	{
		value[0] = adc_data&0x7fffff;			
	}
	else //��
	{
		if( channel_bit == 0 ) //0��ʾ ͨ��1 ����
		{
			value[1] = adc_data_to_24value( adc_data );              			
		}
		else if( channel_bit == 1 ) // 1��ʾ ͨ��2 ����
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
		if( higetst_bit == 0x1 ) //1��ʾת������
		{
			;
		}
		else //��
		{
			if( channel_bit == 0 ) //0��ʾ ͨ��1 ����
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
		if( higetst_bit == 0x1 ) //1��ʾת������
		{
			;
		}
		else //��
		{			
			if( channel_bit == 0 ) //0��ʾ ͨ��1 ����
			{									
				value1[i] = adc_data_to_24value( src[i] );				 						
			}else if( channel_bit == 1 ) // 1��ʾ ͨ��2 ����
			{									
				value2[i] = adc_data_to_24value( src[i] );                					
			}						
		}
	}
}

void dis_dc_func(float *src, int length) //�����д��ֳ֣�ȥ��ֱ�������㷨
{
	float sum = 0.0;
	float average_value = 0.0;
	int i =0;
	for(i = 0; i< length; i++)
	{
		 sum += src[i];		 
	}
	average_value = sum/length;  //���ֱ������ƫ��
	for(i =0;i<length;i++)
	{
		 src[i] = src[i] - average_value;
	}
    return ; 		
}


///////////////////ѹ���ɼ���������
void read_press_data( int signo ) // ��ȡѹ��adc ��������
{	
    //LOGD( "xin: ��ȡѹ������  [%s]\n" ,  log_time( ) );
    if(power_off_flag == 1) 
	{		
		LOGD("xin: ��ȡѹ������ʱ����⵽�µ��ʶ�����ٶ�ȡ������ǰstop fpaga DMA,post�ź���");
		stop_smp_flag = 1; // Ϊ1ʱ ��ֹͣ����������
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
	
	if( g_loop_num == 0 )
    {
        memcpy( &press_buf ,  &read_60K_buf ,  SIZE_60K );	
		g_loop_num ++;	
    }else if( g_loop_num == 1 )
    {
		press_discard_16K_flag ++;
		g_loop_num = 0;
		 if(press_flag0_flag) //ѹ����0����ǰ10������
		{
		   if( press_discard_16K_flag >= 10) 
		   {			
			stop_smp_flag = 1; // flag��1 ��ֹͣ����������
			stop_fpga_dma(); 
			
			memcpy( &press_buf[SIZE_60K] ,  &read_60K_buf ,  SIZE_4K ); //SIZE_4K= 4096
			psample_buf =( int* )&( press_buf ) ;
			LOGD( "xin: postѹ���ź��� : [%s]\n" ,  log_time( ) ); 
					
			sem_post( &run_sem );			
			
			g_loop_num = 0;
			press_discard_16K_flag =0;
		   }
		}else{ 	//���������ɼ���ֻ��ǰ3������ 
			if( press_discard_16K_flag >= 3)
			{			
				stop_smp_flag = 1; // flag��1 ��ֹͣ����������
				stop_fpga_dma(); //ֹͣDMA�ɼ�
				
				memcpy( &press_buf[SIZE_60K] ,  &read_60K_buf ,  SIZE_4K ); //SIZE_4K= 4096
				psample_buf =( int* )&( press_buf ) ;
				LOGD( "xin: postѹ���ź��� : [%s]\n" ,  log_time( ) ); 
						
				sem_post( &run_sem );			
				
				g_loop_num = 0;
				press_discard_16K_flag =0;
			}
		}
    }	
}

void *press_flag0_thread( void* arg ) //ѹ����0ģʽ
{
	LOGD( "xin: ����ѹ����0�߳� [%s]\n" ,  log_time( ) );
    int i=0;      	
    float press_flag0_value[2] ={0.0}; //��0ģʽʱ���ش������� ,  0λ����ʾ����ֵ��1λ����ʾ��Ч���ݸ���
    float f64_sum = 0.0; //64K�����ܺ�
	
	sem_wait( &run_sem );//�ȴ��ź���	
	f64_sum = 0.0;	
	
	for( i=0;i<SIZE_16K;i++ )
	{		        
		f64_sum += adc_data_to_24value( psample_buf[i] ) ;	
	}	
	press_flag0_value[0] = f64_sum / SIZE_16K;	//�ܺ���ƽ��ֵ
	press_flag0_value[1] = 1; 
	
	LOGD( "xin: �����ѹ����0ֵ =%f" , press_flag0_value[0] );			
	
    ioctl( fd ,  SPIDEV_IOC_RXSTREAMOFF ,  NULL ); //ֹͣDMA����
    swrite( StopSampleAddr , StopSampleData );//ֹͣFPGA�ɼ�
	
	close( fd ); //�ر��豸�ļ� 		
	spi_power_off( );  //�µ�	  
	sem_destroy( &run_sem );  //�����ź���
	pressure_callback_backup.mspictl_callback( press_flag0_value , true );//����JNI �ص����������ϲ㴫������   
	start_num =0;
	LOGD( "xin: �˳�ѹ����0�߳� [%s]\n" ,  log_time( ) );
    return NULL;
}

void *press_curve_thread( void* arg ) //ѹ������ģʽ
{    
    LOGD( "xin: ����ѹ�������߳� [%s]\n" ,  log_time( ) );
	int i=0;
    float final_buf[SIZE_16K] ={0.0};   //SIZE_16K
	float positive_buf[SIZE_16K] ={0.0}; //ѹ������ȫ��ת��Ϊ����	
	float ret_value[SIZE_16K] ={0.0};  //�ص����ص�����
    float stop_callback_buf[3] = {20000,20000,20000}; //����ѹ������ģʽ�µ�ʱ���ص������ݣ���������ͨ���������Ż�
	
	float value_bar_max2 = 0.0;   
    float value_bar_min2 = 0.0;	
	float tmp1=0.0 , tmp2=0.0 , tmp3=0.0;		
	//LOGD( "press_curve_thread fabs( pflag0_value ) =%f\n" ,  fabs( pflag0_value ) ); 		
	
	sem_wait( &run_sem );//�ȴ��ź���		
	
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
		final_buf[i] = adc_data_to_24value( psample_buf[i] );	           		
		positive_buf[i] =( float )fabs( final_buf[i] );    //�ɼ����� ȡ����ֵȫ��תΪ����    			
	}
					
	value_bar_max2 = positive_buf[0]; //���������Сֵ��������0ֵ 
	value_bar_min2 = positive_buf[0];			
	for( i=1;i<SIZE_16K;i++ )
	{
		if( value_bar_max2 <= positive_buf[i] )            
			value_bar_max2= positive_buf[i];  //��ԭʼ�ɼ����������ֵ �����ں���0.5bar �ж������������ֵС��0.5bar , �򲻽��к����㷨���㣬ֱ����ʾ�쳣
		
		if( value_bar_min2 >= positive_buf[i] )            
			value_bar_min2= positive_buf[i];  //��ԭʼ�ɼ���������Сֵ		    
	}		
	LOGD( "press_curve_thread value_bar_max2 =%f ,  value_bar_min2 =%f\n" ,  value_bar_max2 , value_bar_min2 );			
	
	tmp1 = fabs(  fabs( pflag0_value ) - value_bar_max2 ); //���0ֵ�������Сֵ��ֵ�ľ���ֵ
	tmp2 = fabs(  fabs( pflag0_value ) - value_bar_min2 );		
	tmp3 =( tmp1 > tmp2 )?tmp1:tmp2;	   	//���ֵ�仯���ֵ
	//LOGD( "tmp1 = %f , tmp2 = %f" , tmp1 , tmp2 );			
	
	/* if( tmp3  <= 0.01 )  //0.5bar = 0.01V  // 20mv/bar  ��ʾ�ⲿ���������׻��� ����С����ʱ��ʾ�ղ�
	{
		LOGD( "press_curve_thread is <0.5bar status,�ص�10000 ��1 ����" );   
        can_start_flag = 1;
		spi_power_off();  //�µ�
		sem_destroy( &run_sem );
		thread_finished_flag = 1;
		usleep(10000); 
			
		pressure_callback_backup.mspictl_callback( status_flag ,  false );
        LOGD("xin: �˳�ѹ�������߳�  [%s]\n" ,  log_time( ) );			
		return NULL;		
	}
	else if( tmp3 >0.01 )	   
	{ */	
		LOGD( "press_curve_thread is >0.5bar status" );				
		press_alg_entry( final_buf , SIZE_16K ,  ret_value );//ѹ������ģʽ�㷨	
		
		if(power_off_flag == 1)//��⵽�µ��ʶΪ1
		{
			LOGD("xin: ѹ�������̼߳�⵽�µ��ʶΪ1���µ磬�����ź������ص�false���ϲ� [%s]\n" ,  log_time( ) );
			can_start_flag = 1;
			spi_power_off(); 
			sem_destroy( &run_sem );
			
			thread_finished_flag = 1;
			usleep(10000);  
			
			pressure_callback_backup.mspictl_callback( stop_callback_buf , false );  //��������ͨ���������Ż�   
            LOGD("xin: �˳�ѹ�������߳�  [%s]\n" ,  log_time( ) );			
		    return NULL;
		}else{
			LOGD("xin: ѹ�������߳������ص�����");
			pressure_callback_backup.mspictl_callback( ret_value ,  true );//����JNI �ص����������ϲ㴫������
			LOGD("xin: ѹ���������ݻص���� [%s]\n" ,  log_time( ) );
		}	
		can_start_flag = 1;			
		sem_destroy( &run_sem );
		
	stop_daq:
		if(power_off_flag == 1) 
		{
			LOGD("xin: ѹ�����������̼߳�⵽�µ��ʶΪ1");
			thread_finished_flag = 1;
			usleep(10000);                                          
			
			pressure_callback_backup.mspictl_callback( stop_callback_buf , false );
			return NULL;
		}
		thread_finished_flag = 1;		
	//}
    LOGD("xin: �˳�ѹ�������߳�  [%s]\n" ,  log_time( ) );	
    return NULL;
}
 
void *press_dial_thread( void* arg ) //ѹ������ģʽ
{
	LOGD( "xin: ����ѹ�������߳� [%s]\n" ,  log_time( ) );
    int i=0;	
    float dial_value[2]={0.0}; //����ģʽʱ���ش������� ,  0λ����ʾ����ֵ��1λ����ʾ��Ч���ݸ���
	
	float final_buf[SIZE_16K] ={0.0};   //SIZE_16K
	float positive_buf[SIZE_16K] ={0.0}; 
	float stop_callback_buf[3] = {30000,30000,30000}; //����ѹ������ģʽ�µ�ʱ���ص������ݣ���������ͨ���������Ż�
	
    float f64_sum = 0.0;
	float value_bar_max1 = 0.0;   
    float value_bar_min1 = 0.0;	
	float tmp1=0.0 , tmp2=0.0 , tmp3=0.0;	
	//LOGD( "press_dial_thread fabs( pflag0_value ) =%f\n" ,  fabs( pflag0_value ) ); 	
			
	sem_wait( &run_sem );//�ȴ��ź���
		
    if(power_off_flag == 1)//��⵽�µ��ʶΪ1
	{
		LOGD("xin: recived post ");
		goto  stop_daq;
	}	
	memset( final_buf , 0 , SIZE_16K*sizeof( float ) );
	memset( positive_buf , 0 , SIZE_16K*sizeof( float ) );		
	f64_sum = 0.0;		
	
	
	for( i=0;i<SIZE_16K;i++ )
	{			
		final_buf[i] = adc_data_to_24value( psample_buf[i] ); 
		f64_sum += final_buf[i];
   		positive_buf[i] =( float )fabs( final_buf[i] );            		
	}	
	
	value_bar_max1 = positive_buf[0]; 
	value_bar_min1 = positive_buf[0];		
	for( i=1;i<SIZE_16K;i++ )
	{
		if( value_bar_max1 <= positive_buf[i] )            
			value_bar_max1= positive_buf[i];  //��ԭʼ�ɼ����������ֵ �����ں���0.5bar �ж������������ֵС��0.5bar , �򲻽��к����㷨���㣬ֱ����ʾ�쳣
		
		if( value_bar_min1 >= positive_buf[i] )            
			value_bar_min1= positive_buf[i];  //��ԭʼ�ɼ���������Сֵ	    
	}		
	//LOGD( "press_dial_thread value_bar_max1 =%f ,  value_bar_min1 =%f\n" ,  value_bar_max1 , value_bar_min1 );
	
	tmp1 = fabs(  fabs( pflag0_value ) - value_bar_max1 );
	tmp2 = fabs(  fabs( pflag0_value ) - value_bar_min1 );
	tmp3 =( tmp1 > tmp2 )?tmp1:tmp2;  
	//LOGD( "tmp1 = %f , tmp2 = %f" , tmp1 , tmp2 );	
	
	if( tmp3  <= 0.01 )  //0.5bar = 0.5*20mv = 10mv = 0.01V  //�ⲿ���������׻��� ����С����ʱ��ʾ�ղ�
	{
		LOGD( "press_dial_thread is <0.5bar status" );
        can_start_flag = 1;
		spi_power_off(); 
		sem_destroy( &run_sem );
		
		thread_finished_flag = 1;
		usleep(10000);  
		
		pressure_callback_backup.mspictl_callback( status_flag ,  false );
        LOGD("xin: �˳�ѹ�������߳�  [%s]\n" ,  log_time( ) );			
		return NULL;		
	}
	else if( tmp3 > 0.01 )	   
	{	
		LOGD( "press_dial_thread is >0.5bar status" );		
		dial_value[0] = f64_sum / SIZE_16K;   //����ģʽ�㷨�Ƕ�����������ƽ��
		dial_value[1] = 1;        			
		LOGD( "xin: ѹ�����̼���ֵdial_value =%f\n" ,  dial_value[0] );
		
		if(power_off_flag == 1) 
		{			
			LOGD("xin: ѹ�������̼߳�⵽�µ��ʶΪ1���µ磬�����ź������ص�false���ϲ� [%s]\n" ,  log_time( ) );
			can_start_flag = 1;
			spi_power_off(); 
			sem_destroy( &run_sem );
			thread_finished_flag = 1;
			usleep(10000);  
			
			pressure_callback_backup.mspictl_callback( stop_callback_buf , false );
            LOGD("xin: �˳�ѹ�������߳�  [%s]\n" ,  log_time( ) );			
		    return NULL;
		}else{
			LOGD("xin: ѹ�������߳������ص�����");
			pressure_callback_backup.mspictl_callback( dial_value ,  true );//����JNI �ص����������ϲ㴫������	
		    LOGD("xin: ѹ���������ݻص���� [%s]\n" ,  log_time( ) );
		}
		can_start_flag = 1;			
		sem_destroy( &run_sem );
		
	stop_daq:
		if(power_off_flag == 1) 
		{
			LOGD("xin: ѹ�����������̼߳�⵽�µ��ʶΪ1");  
			thread_finished_flag = 1;
			usleep(10000);                                          
			
			pressure_callback_backup.mspictl_callback( stop_callback_buf , false );
			return NULL;
		}
		thread_finished_flag = 1;
	}
    LOGD("xin: �˳�ѹ�������߳�  [%s]\n" ,  log_time( ) );	
    return NULL;
}
 
static int start_pressure_flag0( struct spictl_device_t* dev )// ѹ����0ģʽ
{	 
   LOGD("xin: ��Ӧѹ����0�����ʼ******************** [%s]\n" ,  log_time( ));  
   LOGD("xin: start_pressure_flag0_start_num = %d",start_num);
   if(start_num != 0)
   {
	  return 0; 
   }   
    memset(read_60K_buf,0,SIZE_60K*sizeof(unsigned char));
    memset(press_buf ,0, SIZE_64K*sizeof(unsigned char)); 
	int reg_ret_value =0;
    g_loop_num = 0;
	   
    poweron_spi(  );	
		
    reg_ret_value = set_press_reg( smp_rate );	//����ѹ���ɼ��Ĵ��� 
	usleep(2000000); //�����ϲ�ʱ���β����������ݣ�������ֱ����ʱ2S,��Ӳ ��Ԥ���ȶ�

	if( reg_ret_value == -1)
	{	
        LOGD("xin: �Ĵ�������ʧ��");
		pressure_callback_backup.mspictl_callback( press_reg_fail_buf ,  false );//��ʾ�Ĵ�������ʧ�ܣ��ص� false���ϲ�
		start_enable_flag = 0;
		spi_power_off(  );			
		return 0;
	}
	
	start_num ++;
	press_flag0_flag = 1;  //ֻ��Ա�0ʱ����ʱ��Ϊ1
	sem_init( &run_sem ,  0 ,  0 );	
	
	pthread_create( &c_id ,  NULL ,  press_flag0_thread ,  NULL );
	signal( SIGIO ,  read_press_data );// ��׽�첽IO�źţ�����װ�źŴ�����	
    common_start( );	
	LOGD("xin: ��Ӧѹ����0�������******************** [%s]\n" ,  log_time( )); 
	return 0;
}
 
static int start_pressure_curve( struct spictl_device_t* dev , float flag0_value )  //ѹ������ģʽ
{	 
   LOGD("xin: start_pressure_curve_start_num = %d",start_num);
   if(start_num != 0)
   {
	  return 0; 
   } 
    memset(read_60K_buf,0,SIZE_60K*sizeof(unsigned char)); 
    memset(press_buf ,0, SIZE_64K*sizeof(unsigned char)); 	
	int reg_ret_value =0;
    g_loop_num = 0;
	
	LOGD("\nxin: ������߿�ʼʱ start_enable_flag = %d, restart_power_on_flag= %d, can_start_flag = %d , thread_finished_flag = %d",
	    start_enable_flag,restart_power_on_flag,can_start_flag,thread_finished_flag);
		
		
	if(start_enable_flag == 1) // 1��ʾǰһ��start �̻߳�û�н�������ʱ������Ӧ�µ�start�� Ϊ0ʱ��ʾstart�߳̽�����
	{
		LOGD("xin: start_enable_flag = %d, ǰһ��start��δִ���꣬��ʱ����Ӧ�µ�start�ӿ�",start_enable_flag);		
        return 0;
	}else{
	    start_enable_flag = 1;
	}	
	
	
	LOGD("xin: ��Ӧѹ�����ߵ����ʼ******************** [%s]\n" ,  log_time( )); 
    if(restart_power_on_flag == 1) // ���ڲ�stop DMA��fpga,�ر��豸��flag ��1�� ��������start�ӿ�ʱ���������ϵ磬��������ֹͣ�ɼ��µ�󣬴�flag����Ϊ 0�����²ɼ�ʱ���ϵ�
	{
		;
	}else{
		poweron_spi(  );		
		
		reg_ret_value = set_press_reg( smp_rate );	//����ѹ���ɼ��Ĵ��� 
		usleep(2000000); //�����ϲ�ʱ���β����������ݣ�������ֱ����ʱ2S,��Ӳ ��Ԥ���ȶ�
		
		if( reg_ret_value == -1)
		{	
            LOGD("xin: �Ĵ�������ʧ��");	
			pressure_callback_backup.mspictl_callback( press_reg_fail_buf ,  false );//��ʾ�Ĵ�������ʧ�ܣ��ص� false���ϲ�
			start_enable_flag = 0;
            spi_power_off(  );			
			return 0;
		}
	}
	
	if (can_start_flag == 1)
	{
	    return 0;  		
	}
	
	start_num ++;
    usleep(100000);//����stop ��start DMA,�Ĵ����ȶ�
	sem_init( &run_sem ,  0 ,  0 );   //��ʼ���ź���
	
	pflag0_value = flag0_value;	

	stop_smp_flag = 0;
	
	pthread_create( &c_id ,  NULL ,  press_curve_thread ,  NULL );
    signal( SIGIO ,  read_press_data );	
    common_start( );
	
	while( thread_finished_flag == 0); // ���ڵȴ��㷨�߳�����ʱ��1�����߳̽���ʱ��������������
	
	can_start_flag = 0;
	thread_finished_flag = 0;
	
	power_off_flag = 0;
	start_enable_flag = 0;
	
	//restart_power_on_flag = 0;
	LOGD( "xin: ��Ӧѹ�����ߵ������***** [%s]\n" ,  log_time( ) ); 	
	usleep( 100000 ); //���ڵײ������ַ�������ַӳ�����	
	
	return 0;	
}

static int start_pressure_dial( struct spictl_device_t* dev , float flag0_value )// ѹ������ģʽ
{	
   LOGD("xin: start_pressure_dial_start_num = %d",start_num);
   if(start_num != 0)
   {
	  return 0; 
   } 
    memset(read_60K_buf,0,SIZE_60K*sizeof(unsigned char));   
    memset(press_buf ,0, SIZE_64K*sizeof(unsigned char)); 	
	int reg_ret_value =0;
    g_loop_num = 0;
	
	LOGD("\nxin: ������̿�ʼʱ start_enable_flag = %d, restart_power_on_flag= %d, can_start_flag = %d , thread_finished_flag = %d",
	    start_enable_flag,restart_power_on_flag,can_start_flag,thread_finished_flag);
		
	if(start_enable_flag == 1)
	{
		LOGD("xin: start_enable_flag = %d, ����Ӧstart�ӿ�",start_enable_flag);		
        return 0;
	}else{
	    start_enable_flag = 1;
	}	
	
	LOGD("xin: ��Ӧѹ�����̵����ʼ [%s]\n" ,  log_time( )); 
    if(restart_power_on_flag == 1) // ���ڲ�stop DMA��fpga,�ر��豸��flag ��1�� ��������start�ӿ�ʱ���������ϵ磬��������ֹͣ�ɼ��µ�󣬴�flag����Ϊ 0�����²ɼ�ʱ���ϵ�
	{
		;
	}else{
		poweron_spi(  );		
		
		reg_ret_value = set_press_reg( smp_rate );	//����ѹ���ɼ��Ĵ��� 
		usleep(2000000); //�����ϲ�ʱ���β����������ݣ�������ֱ����ʱ2S,��Ӳ ��Ԥ���ȶ�
		
		if( reg_ret_value == -1)
		{	
            LOGD("xin: �Ĵ�������ʧ��");	
			pressure_callback_backup.mspictl_callback( press_reg_fail_buf ,  false );//��ʾ�Ĵ�������ʧ�ܣ��ص� false���ϲ�
			start_enable_flag = 0;
            spi_power_off(  );			
			return 0;
		}
		
	}
	
	if (can_start_flag == 1)
	{
	    return 0;  
	}		
	
	start_num++;
    usleep(100000);//����stop ��start DMA,�Ĵ����ȶ�
	sem_init( &run_sem ,  0 ,  0 );   //��ʼ���ź���
	
	pflag0_value = flag0_value;
	
	stop_smp_flag = 0;
	
	pthread_create( &c_id ,  NULL ,  press_dial_thread ,  NULL );
    signal( SIGIO ,  read_press_data );	
    common_start( );
	
	while( thread_finished_flag == 0); // ���ڵȴ��㷨�߳�����ʱ��1�����߳̽���ʱ��������������
	can_start_flag = 0;
	thread_finished_flag = 0;
	
    power_off_flag = 0;	
	start_enable_flag = 0;
	
	//restart_power_on_flag = 0;
	LOGD( "xin: ��Ӧѹ�����̵������ [%s]\n" ,  log_time( ) ); 	
	usleep( 100000 ); //���ڵײ������ַ�������ַӳ�����	
	
	return 0; 
}


////////////�񶯲ɼ���������
#define   MAX_SIZE  31457280

unsigned char g_max_char_buf[ MAX_SIZE ] ={0}; //6.34M  6568576    ,26274304    31457280
int g_discard_pnts = 0; //IIR�˲���Ҫ�����ĵ���
int g_smpLength = 0; //ʵ�ʲɼ��Ĳ��γ���
int g_waveLength = 0; //UI�·��Ĳ��γ���
int g_chNum = 0;     //��ͨ������
int g_max_freq = 0; // ����Ƶ��
int g_min_freq = 0; // ����Ƶ�� 
 
void read_evalute_data( int signo) //��ȡ����������    ͨ��2, ����10HZ, ����1000 ,����4096
{	    
	if( stop_smp_flag )
	{		 
		return;
	}		
	int total_len = g_smpLength;  //�����ܵ���
	int shang = total_len/SIZE_15360;  //15360 = 60K �ı���
	int yu = total_len%SIZE_15360;  // ����
		
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
         //LOGD( "xin: post���ź���000 : [%s]\n" ,  log_time( ) );
	}		
}

void read_vibrate_data( int signo) //��ȡ�񶯲ɼ�����
{	    
    //LOGD( "xin: read_vibrate_data_stop_smp_flag = %d,g_loop_num = %d, g_smpLength = %d , g_chNum =%d  , [%s]\n" ,  stop_smp_flag, g_loop_num, g_smpLength , g_chNum , log_time( ) ); 
	if(power_off_flag == 1) 
	{	
		LOGD("xin: ��ȡ������ʱ����⵽�µ��ʶ�����ٶ�ȡ������ǰstop fpga DMA,post�ź���");
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
	int total_len = g_smpLength;  //�����ܵ���
	int shang = total_len/SIZE_15360;  //15360 �ı���
	int yu = total_len%SIZE_15360;  // ����	

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
		 LOGD( "xin: post���ź���000 : [%s]\n" ,  log_time( ) );      
	}
		
	if( shang > 0 && yu > 0 )
	{		  
		  if( read( fd , read_60K_buf , SIZE_60K ) <0 )
		  {			  
			 LOGD( "Error: spi slave device read fail !\n " ); 
		  }
		 
		  if( g_loop_num < shang )
		  {			 				 
			 memcpy( &g_max_char_buf[SIZE_60K*g_loop_num] ,  &read_60K_buf ,  SIZE_60K );	
			 g_loop_num++;
		  }
          else if( g_loop_num == shang )
		  {		
	         stop_smp_flag = 1; //flag��1,ֹͣ����������
	         stop_fpga_dma(); 
			 
			 memcpy( &g_max_char_buf[SIZE_60K*g_loop_num] ,  &read_60K_buf ,  yu*sizeof( float ) );
			 
			 g_smp_buf =( int* )&( g_max_char_buf ) ; 				 
			 
			 sem_post( &run_sem ); 
			 LOGD( "xin: post���ź���111 : [%s]\n" ,  log_time( ) );              	 
		  }		
	}	
	if( shang > 0 && yu == 0 )
	{             
		 if( read( fd , read_60K_buf , SIZE_60K ) <0 )
		 {			 
			LOGD( "Error: spi slave device read fail !\n " );
		 }			
		 
		 if( g_loop_num < shang )
		 {			
			 memcpy( &g_max_char_buf[SIZE_60K*g_loop_num] ,  &read_60K_buf ,  SIZE_60K );	
			 g_loop_num++;
		 }
		 else if( g_loop_num == shang )
		 {		
	         stop_smp_flag = 1;
             stop_fpga_dma();	 
			 
			 g_smp_buf =( int* )&( g_max_char_buf ) ;	
			 
			 sem_post( &run_sem ); 
			 LOGD( "xin: post���ź���222 : [%s]\n" ,  log_time( ) );              			 
		 }		
	}		
}

void *time_wave_thread( void* arg ) //ʱ���߳�
{	    
	timewave my_timewave ={0};
	my_timewave = *( struct time_wave_para* )arg;
	
	int i=0 ;
    int temp_len =0;		
	float CH_data[3]={0.0};	 //����ͨ������
	
	if( g_chNum == SINGLE_CH )
	{		
		if( g_max_freq == 500 || g_max_freq == 2500 )
		{
			g_smpLength = ( g_waveLength + g_discard_pnts +36 )*4;	//��� ������Ƶ�ʣ���Ҫ1/4 ��㣬���Բɼ����� *4
		}else if( g_max_freq == 1000 || g_max_freq == 5000 )
		{
			g_smpLength = ( g_waveLength + g_discard_pnts +31 )*2;	//��� ������Ƶ�ʣ���Ҫ1/2 ��㣬���Բɼ����� *2
		}else
		{
			g_smpLength = ( g_waveLength + g_discard_pnts );	//ʵ�ʲɼ��Ĳ��γ���			
		}

		LOGD( "xin: time_wave_thread_SINGLE_CH_g_discard_pnts = %d ,  g_waveLength = %d ,  g_smpLength = %d"  , g_discard_pnts ,  g_waveLength ,  g_smpLength );
				
		//������Ҫ���ڴ�
		float *time_CH1_smp_buf = NULL; //ͨ��1 ��������  ����ΪUI���ó��� + �����ĵ���
		if( time_CH1_smp_buf == NULL)
		{
			time_CH1_smp_buf =( float* )malloc( g_smpLength*sizeof( float ) );
			if( time_CH1_smp_buf == NULL )
			{
				LOGD( "time_CH1_smp_buf �����ڴ�ʧ�ܣ�" );
				exit( EXIT_FAILURE );		
			}
			memset( time_CH1_smp_buf , 0 , g_smpLength*sizeof( float ) );
		}		  
		 
		 
		sem_wait( &run_sem );//�ȴ��ź��� 
		
		if(power_off_flag == 1)//�����ź������һʱ���ж��Ƿ����µ��ʶ
		{
			LOGD("xin: recived post ");
			goto  stop_daq;
		}
        memset(time_CH1_smp_buf , 0 ,g_smpLength*sizeof(float) );			
	    memset(	CH_data , 0, 3*sizeof(float));	
		
		for( i=0;i< g_smpLength;i++ )	//ת���ɼ��ĵ���	
		{					
			analyze_CH_data( g_smp_buf[i] , CH_data );	
			time_CH1_smp_buf[i] = CH_data[2];				
		}		
		
		dis_dc_func(time_CH1_smp_buf, g_smpLength);	 //����ȥֱ�������㷨
		LOGD( "xin: SINGLE_CH_����ת������ is : [%s]\n" ,  log_time( ) );				
		
		temp_len = g_smpLength; ////FIR ��ͨ�˲�����ͨ������������		
		if(g_max_freq == 500 || g_max_freq == 2500 )//������5000 2500��1/4���� 
		{							
			enter_FIR_Filter( time_CH1_smp_buf  , temp_len, g_max_freq); //FIR ��ͨ�˲�
			temp_len = g_smpLength/4 -36; //����󳤶ȱ�Ϊ1/4	
			
			enter_IIR_Filter( time_CH1_smp_buf , temp_len ,g_max_freq ,g_min_freq ); //IIR ��ͨ�˲�
		}else if( g_max_freq == 1000 || g_max_freq == 5000)	 //������1000 5000��1/2���� 
		{				
			enter_FIR_Filter( time_CH1_smp_buf  , g_smpLength, g_max_freq); //FIR ��ͨ�˲�
			temp_len = g_smpLength/2 -31; //����󳤶ȱ�Ϊ1/2	
						
			enter_IIR_Filter( time_CH1_smp_buf ,temp_len ,g_max_freq ,g_min_freq ); //IIR ��ͨ�˲�
							
		}else  //��������Ƶ��ֻ����IIR
		{	
			enter_IIR_Filter( time_CH1_smp_buf , temp_len ,g_max_freq ,g_min_freq ); //IIR ��ͨ�˲�
		}
	
		LOGD("SINGLE_CH_temp_len = %d , g_discard_pnts = %d, wave_length = %d, wave_length+discard_pnts = %d" ,temp_len,g_discard_pnts,g_waveLength,g_waveLength + g_discard_pnts  );			
		memcpy( time_CH1_smp_buf ,  &time_CH1_smp_buf[g_discard_pnts] ,  g_waveLength*sizeof( float )  );//��IIR�˲��� ȥ�������ĵ�							
		LOGD( "xin: SINGLE_CH_�˳��㷨 is : [%s]\n" ,  log_time( ) );          		
	
    stop_daq:	
        if(power_off_flag == 1) 
		{	
            LOGD("xin: ʱ�������̼߳�⵽�µ��ʶΪ1���ͷ��ڴ棬�����ź������ص�false���ϲ� [%s]\n" ,  log_time( ) ); 
			if( time_CH1_smp_buf !=NULL)
			{			
				free( time_CH1_smp_buf );
				time_CH1_smp_buf =NULL; 
			}				
			
			can_start_flag = 1;
			LOGD("xin: free����malloc=====�µ�ʱ��ǰ�ͷź�, ���ûص�����======");
			sem_destroy( &run_sem );
			thread_finished_flag = 1;			
			
			usleep(10000);                                                   
			
			vibrate_callback_backup.single_ch_callback( invalid_buf , 0,  false );	/////�ص�ʱ����			
		    return NULL;
		}else{           	
            LOGD("xin: ʱ�������߳������ص�����");			
			vibrate_callback_backup.single_ch_callback( time_CH1_smp_buf , g_waveLength ,  true );	/////�ص�ʱ����
		}
		
		LOGD( "xin: SINGLE_CH_�ص����� is : [%s]\n" ,  log_time( ) );	
		if( time_CH1_smp_buf !=NULL)
		{			
			free( time_CH1_smp_buf );
			time_CH1_smp_buf =NULL; 
		}				
		
		can_start_flag = 1;
		LOGD("xin: free����malloc=====�����ͷź�, ���ûص�����======");
		sem_destroy( &run_sem );
		if(power_off_flag == 1) 
		{
			LOGD("xin: ʱ�������̼߳�⵽�µ��ʶΪ1");
			thread_finished_flag = 1;
			usleep(10000);                                          
			
			vibrate_callback_backup.single_ch_callback( invalid_buf , 0,  false );	
			return NULL;
		}
		thread_finished_flag = 1;		
	}
	
	if( g_chNum == DOUBLE_CH ) //˫ͨ���ڴ�Ϊ��ͨ��������
	{			
		if( g_max_freq == 500 || g_max_freq == 2500 )
		{
			g_smpLength = ( g_waveLength + g_discard_pnts  )*2*4;	//��� ������Ƶ�ʣ���Ҫ1/4 ��㣬���Բɼ����� *4
		}else if( g_max_freq == 1000 || g_max_freq == 5000 )
		{
			g_smpLength = ( g_waveLength + g_discard_pnts )*2*2;	//��� ������Ƶ�ʣ���Ҫ1/2 ��㣬���Բɼ����� *2
		}else
		{
			g_smpLength = ( g_waveLength+ g_discard_pnts )*2;	//ʵ�ʲɼ��Ĳ��γ���
		}
	
		LOGD( "xin: time_wave_thread_DOUBLE_CH_g_discard_pnts = %d ,  g_waveLength = %d ,  g_smpLength = %d"  , g_discard_pnts ,  g_waveLength,  g_smpLength );        
	    
        	
        //������Ҫ���ڴ�			
        float *time_CH1_smp_buf =NULL; //ͨ��1����  ��������
		if( time_CH1_smp_buf == NULL)
		{
			time_CH1_smp_buf =( float* )malloc(g_smpLength*sizeof( float ) );
			if( time_CH1_smp_buf == NULL )
			{
				LOGD( "time_CH1_smp_buf �����ڴ�ʧ�ܣ�" );
				exit( EXIT_FAILURE );		
			}
			memset( time_CH1_smp_buf , 0 , g_smpLength*sizeof( float ) );
		}
		
		float *time_CH2_smp_buf =NULL; //ͨ��2���� ��������
		if( time_CH2_smp_buf == NULL)
		{
			time_CH2_smp_buf =( float* )malloc(g_smpLength*sizeof( float ) );
			if( time_CH2_smp_buf == NULL )
			{
				LOGD( "time_CH2_smp_buf �����ڴ�ʧ�ܣ�" );
				exit( EXIT_FAILURE );		
			}
			memset( time_CH2_smp_buf , 0 , g_smpLength*sizeof( float ) );	
		}					
			
		sem_wait( &run_sem );//�ȴ��ź���
		
        memset(time_CH1_smp_buf , 0 ,g_smpLength*sizeof(float) );	
		memset(time_CH2_smp_buf , 0 ,g_smpLength*sizeof(float) );
		
		 
		for( i=0;i< g_smpLength;i++ )	//ת���ɼ�����	
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
		LOGD( "xin: DOUBLE_CH_����ת������ is : [%s]\n" ,  log_time( ) );      	
		
		#if 1				
			temp_len = g_smpLength/2; //FIR ��ͨ�˲���ԭ��������Ϊ˫ͨ�� һ��			
			if(g_max_freq == 500 || g_max_freq == 2500 )//������5000 2500��1/4���� 
			{					
				enter_FIR_Filter( time_CH1_smp_buf  , temp_len, g_max_freq); //FIR ��ͨ�˲�
				enter_FIR_Filter( time_CH2_smp_buf  , temp_len, g_max_freq); 					
				temp_len = g_smpLength/2/4; //FIR ��ͨ�˲�,����󳤶ȱ�Ϊ1/4	
									
				enter_IIR_Filter( time_CH1_smp_buf , temp_len , g_max_freq , g_min_freq ); //IIR ��ͨ�˲�
				enter_IIR_Filter( time_CH2_smp_buf , temp_len , g_max_freq , g_min_freq ); //IIR ��ͨ�˲�
			}else if( g_max_freq == 1000 || g_max_freq == 5000)	 //������1000 5000��1/2���� 
			{
				enter_FIR_Filter( time_CH1_smp_buf  , temp_len,  g_max_freq); //FIR ��ͨ�˲�
				enter_FIR_Filter( time_CH2_smp_buf  , temp_len,  g_max_freq); 
				temp_len = g_smpLength/2/2; //FIR ��ͨ�˲�,����󳤶ȱ�Ϊ1/2
				
				enter_IIR_Filter( time_CH1_smp_buf , temp_len , g_max_freq , g_min_freq ); //IIR ��ͨ�˲�
				enter_IIR_Filter( time_CH2_smp_buf , temp_len , g_max_freq , g_min_freq ); //IIR ��ͨ�˲�
				
			}else  //��������Ƶ��ֻ����IIR
			{	
				enter_IIR_Filter( time_CH1_smp_buf , temp_len , g_max_freq , g_min_freq );  //IIR �˲�����Ϊ �ɼ����ȵ�һ��
				enter_IIR_Filter( time_CH2_smp_buf , temp_len , g_max_freq , g_min_freq );
			}
			LOGD("DOUBLE_CH_temp_len = %d ,g_discard_pnts = %d,wave_length = %d, wave_length+discard_pnts = %d" ,temp_len,g_discard_pnts,g_waveLength, g_waveLength + g_discard_pnts );	
			memcpy( time_CH1_smp_buf ,  &time_CH1_smp_buf[g_discard_pnts] ,  g_waveLength*sizeof( float ));// ����ʵ����Ҫ�ĵ�����ȥ�������ĵ�
			memcpy( time_CH2_smp_buf ,  &time_CH2_smp_buf[g_discard_pnts] ,  g_waveLength*sizeof( float ));
		
		#endif		
		LOGD( "xin: DOUBLE_CH_�˳��㷨 is : [%s]\n" ,  log_time( ) );
			
        if(power_off_flag == 1)
		{	 
            LOGD("xin: ��ʼ�ص�ʱpower_off_flag ==1");		
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
			LOGD("xin: free����malloc=====�µ�ʱ��ǰ�ͷź�, ���ûص�����======");
			sem_destroy( &run_sem );
			thread_finished_flag = 1;	
			usleep(10000);  // �����߳�ִ�к����flag ִ��
			vibrate_callback_backup.double_ch_callback( invalid_buf ,  invalid_buf  , 0 ,  false ); /////�ص� UI����
            return NULL;			
		}else{	
            LOGD("xin: g_waveLength = %d",g_waveLength);		
			vibrate_callback_backup.double_ch_callback( time_CH1_smp_buf ,  time_CH2_smp_buf  , g_waveLength ,  true ); /////�ص� UI����	
		}
		
		LOGD( "xin: DOUBLE_CH_�ص����� is : [%s]\n" ,  log_time( ));					
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
		LOGD("xin: free����malloc=====�����ͷź�, ���ûص�����======");
		sem_destroy( &run_sem );
		thread_finished_flag = 1;
	}
	return NULL;
}

void *total_rend_thread( void* arg ) //��ֵ�����߳�
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
			g_smpLength = ( g_waveLength + g_discard_pnts +36 )*4;	//��� ������Ƶ�ʣ���Ҫ1/4 ��㣬���Բɼ����� *4
		}else if( g_max_freq == 1000 || g_max_freq == 5000 )
		{
			g_smpLength = ( g_waveLength + g_discard_pnts +31 )*2;	//��� ������Ƶ�ʣ���Ҫ1/2 ��㣬���Բɼ����� *2
		}else
		{
			g_smpLength = ( g_waveLength + g_discard_pnts );	//ʵ�ʲɼ��Ĳ��γ���
		}
		
		LOGD( "xin: total_rend_thread_SINGLE_CH_g_discard_pnts = %d ,  g_waveLength = %d ,  g_smpLength = %d"  , g_discard_pnts ,  g_waveLength ,  g_smpLength );
		
		//������Ҫ���ڴ�		
		float *rend_CH1_smp_buf =NULL; //ͨ��1 �ɼ�����
		if( rend_CH1_smp_buf == NULL)
		{
			rend_CH1_smp_buf =( float* )malloc( g_smpLength*sizeof( float ) );
			if( rend_CH1_smp_buf == NULL )
			{
				LOGD( "rend_CH1_smp_buf �����ڴ�ʧ�ܣ�" );
				exit( EXIT_FAILURE );		
			}
			memset( rend_CH1_smp_buf , 0 , g_smpLength*sizeof( float ) );			
		}
			
		sem_wait( &run_sem );//�ȴ��ź���
		
        if(power_off_flag == 1)
		{
			LOGD("xin: recived post ");
			goto  stop_daq;
		}
		
		memset( rend_CH1_smp_buf , 0 , g_smpLength*sizeof( float ) );
        memset(	CH_data , 0, 3*sizeof(float));	
        CH1_value[0] = 0.0;
		CH2_value[0] = 0.0;			
	    
		
	    for( i=0;i< g_smpLength;i++ )	//ת���ɼ��ĵ���	
		{					
			analyze_CH_data( g_smp_buf[i] , CH_data );	
			rend_CH1_smp_buf[i] = CH_data[2];				
		}    
		dis_dc_func(rend_CH1_smp_buf,g_smpLength); //ȥ��ֱ�������㷨
		LOGD( "xin: SINGLE_CH_����ת������ is : [%s]\n" ,  log_time( ) );
		
				
		temp_len = g_smpLength;		////FIR ��ͨ�˲�����ͨ������������	
		if(g_max_freq == 500 || g_max_freq == 2500 )//����5000 2500 ��1/4���� 
		{				
			enter_FIR_Filter( rend_CH1_smp_buf  , temp_len, g_max_freq); //FIR ��ͨ�˲�
			temp_len = g_smpLength/4 -36; //����󳤶ȱ�Ϊ1/4
			
			enter_IIR_Filter( rend_CH1_smp_buf , temp_len , g_max_freq , g_min_freq ); //IIR ��ͨ�˲�
		}else if( g_max_freq == 1000 || g_max_freq == 5000)	 //����1000 5000��1/2���� 
		{
			enter_FIR_Filter( rend_CH1_smp_buf  , temp_len,  g_max_freq); //FIR ��ͨ�˲�
			temp_len = g_smpLength/2 -31; //����󳤶ȱ�Ϊ1/2	
			
			enter_IIR_Filter( rend_CH1_smp_buf , temp_len , g_max_freq , g_min_freq ); //IIR ��ͨ�˲�				
		}else  //��������Ƶ��ֻ����IIR
		{	               		
			enter_IIR_Filter( rend_CH1_smp_buf , temp_len , g_max_freq , g_min_freq ); //IIR ��ͨ�˲��������ǲ����ĵ�
		}			
		
		LOGD("SINGLE_CH_temp_len = %d , g_discard_pnts = %d, wave_length = %d, wave_length+discard_pnts = %d" ,temp_len,g_discard_pnts,g_waveLength, g_waveLength + g_discard_pnts  );			
		
		memcpy( rend_CH1_smp_buf , &rend_CH1_smp_buf[g_discard_pnts] , g_waveLength*sizeof( float ) );//��IIR�˲��� ȥ�������ĵ�	
		LOGD( "xin: SINGLE_CH_�˳��㷨 is : [%s]\n" ,  log_time( ) );
        	
		
		CH1_value[0] = rend_value( rend_CH1_smp_buf , g_waveLength , my_totalrend.total_value_type ); //��ֵ�����㷨           			
		LOGD( "xin: SINGLE_CH_CH1_value = %f" , CH1_value[0] );	
		
	stop_daq:
	    if(power_off_flag == 1) 
		{	  
            LOGD("xin: ��ֵ�����̼߳�⵽�µ��ʶΪ1���ͷ��ڴ棬�����ź������ص�false���ϲ� [%s]\n" ,  log_time( ) );
            if( rend_CH1_smp_buf != NULL)
			{			
				free( rend_CH1_smp_buf ); 
				rend_CH1_smp_buf =NULL;
			}
						
			can_start_flag = 1;
			LOGD("xin: free����malloc=====�µ�ʱ��ǰ�ͷź�, ���ûص�����======");
			sem_destroy( &run_sem );
			thread_finished_flag = 1;		
			usleep(10000);  // �����߳�ִ�к����flag ִ��
			
			vibrate_callback_backup.single_ch_callback( invalid_buf  , 0 ,  false ); 
			return NULL;
		}else{
			LOGD("xin: ��ֵ�����߳������ص�����");
			vibrate_callback_backup.single_ch_callback( CH1_value , sizeof( CH1_value )/sizeof( float ) ,  true ); /////�ص�			
		}		
		LOGD( "xin: SINGLE_CH_�ص����� is : [%s]\n" ,  log_time( ) );	
		
		
		if( rend_CH1_smp_buf != NULL)
		{			
			free( rend_CH1_smp_buf ); 
			rend_CH1_smp_buf =NULL;
		}
					
		can_start_flag = 1;
		LOGD("xin: free����malloc=====�����ͷź�, ���ûص�����======");
		sem_destroy( &run_sem );
		if(power_off_flag == 1) 
		{
			LOGD("xin: ʱ�������̼߳�⵽�µ��ʶΪ1");
			thread_finished_flag = 1;
			usleep(10000);                                          
			
			vibrate_callback_backup.single_ch_callback( invalid_buf , 0,  false );	
			return NULL;
		}
		thread_finished_flag = 1;			      		
	}

	if( g_chNum == DOUBLE_CH ) //˫ͨ���ڴ�Ϊ��ͨ��ʱ������
	{	
        if( g_max_freq == 500 || g_max_freq == 2500 )
		{
			g_smpLength = ( g_waveLength + g_discard_pnts  )*2*4;	//��� ������Ƶ�ʣ���Ҫ1/4 ��㣬���Բɼ����� *4
		}else if( g_max_freq == 1000 || g_max_freq == 5000 )
		{
			g_smpLength = ( g_waveLength + g_discard_pnts  )*2*2;	//��� ������Ƶ�ʣ���Ҫ1/2 ��㣬���Բɼ����� *2
		}else
		{
			g_smpLength = ( g_waveLength + g_discard_pnts )*2;	//ʵ�ʲɼ��Ĳ��γ���
		}
		
		LOGD( "xin: total_rend_thread_DOUBLE_CH_g_discard_pnts = %d ,  g_waveLength = %d ,  g_smpLength = %d"  , g_discard_pnts ,  g_waveLength ,  g_smpLength );
       
	    //������Ҫ���ڴ�		
		float *rend_CH1_smp_buf =NULL; //ͨ��1 ��������
		if( rend_CH1_smp_buf == NULL)
		{
			rend_CH1_smp_buf =( float* )malloc( g_smpLength*sizeof( float ) );
			if( rend_CH1_smp_buf == NULL )
			{
				LOGD( "rend_CH1_smp_buf �����ڴ�ʧ�ܣ�" );
				exit( EXIT_FAILURE );		
			}
			memset( rend_CH1_smp_buf , 0 , g_smpLength*sizeof( float ) );	
		}

		float *rend_CH2_smp_buf =NULL; //ͨ��2 ��������
		if( rend_CH2_smp_buf == NULL)
		{
			rend_CH2_smp_buf =( float* )malloc( g_smpLength*sizeof( float ) );
			if( rend_CH2_smp_buf == NULL )
			{
				LOGD( "rend_CH2_smp_buf �����ڴ�ʧ�ܣ�" );
				exit( EXIT_FAILURE );		
			}
			memset( rend_CH2_smp_buf , 0 , g_smpLength*sizeof( float ) );
		}		
		
				
		sem_wait( &run_sem );//�ȴ��ź���					
		memset( rend_CH1_smp_buf , 0 , g_smpLength*sizeof( float ) );	
		memset( rend_CH2_smp_buf , 0 , g_smpLength*sizeof( float ) );
		
				 		    
		for( i=0;i< g_smpLength;i++ )			
		{					
			analyze_CH_data( g_smp_buf[i] , CH_data );  //���ý���ͨ�����ݵĺ���	
			rend_CH1_smp_buf[i] = CH_data[1];
			rend_CH2_smp_buf[i] = CH_data[2];
		}			
		for( i=0;i<=g_smpLength/2;i++ )
		{					
			rend_CH1_smp_buf[i] = rend_CH1_smp_buf[i*2]; //ͨ��1
			rend_CH2_smp_buf[i] = rend_CH2_smp_buf[i*2+1];//ͨ��2					
		}		
		LOGD( "xin: DOUBLE_CH_����ת������ is : [%s]\n" ,  log_time( ) );
		
		#if 1				
			temp_len = g_smpLength/2; //FIR ��ͨ�˲���ԭ��������Ϊ˫ͨ�� һ��	
			if(g_max_freq == 500 || g_max_freq == 2500 )//������5000 2500��1/4���� 
			{
				enter_FIR_Filter( rend_CH1_smp_buf  , temp_len, g_max_freq); //FIR ��ͨ�˲�
				enter_FIR_Filter( rend_CH2_smp_buf  , temp_len, g_max_freq); 
				temp_len = g_smpLength/2/4; //����󳤶ȱ�Ϊ1/4
				
				enter_IIR_Filter( rend_CH1_smp_buf , temp_len ,g_max_freq ,g_min_freq ); //IIR ��ͨ�˲�
				enter_IIR_Filter( rend_CH2_smp_buf , temp_len ,g_max_freq ,g_min_freq ); //IIR ��ͨ�˲�
			}else if( g_max_freq == 1000 || g_max_freq == 5000)	 //������1000 5000��1/2���� 
			{				
				enter_FIR_Filter( rend_CH1_smp_buf  , temp_len, g_max_freq); //FIR ��ͨ�˲���ԭ�������ȣ�����󳤶ȱ�Ϊ1/2
				enter_FIR_Filter( rend_CH2_smp_buf  , temp_len, g_max_freq); //FIR ��ͨ�˲���ԭ�������ȣ�����󳤶ȱ�Ϊ1/2
				temp_len = g_smpLength/2/2; //����󳤶ȱ�Ϊ1/2
			
				enter_IIR_Filter( rend_CH1_smp_buf , temp_len ,g_max_freq ,g_min_freq ); //IIR ��ͨ�˲�
				enter_IIR_Filter( rend_CH2_smp_buf , temp_len ,g_max_freq ,g_min_freq ); //IIR ��ͨ�˲�
			}else{  //��������Ƶ��ֻ����IIR
				enter_IIR_Filter( rend_CH1_smp_buf , temp_len ,g_max_freq ,g_min_freq );  //IIR �˲�����Ϊ �ɼ����ȵ�һ��
				enter_IIR_Filter( rend_CH2_smp_buf , temp_len ,g_max_freq ,g_min_freq );
			}				
					
			LOGD("DOUBLE_CH_temp_len = %d , g_discard_pnts = %d, wave_length = %d, wave_length+discard_pnts = %d" ,temp_len,g_discard_pnts,g_waveLength, g_waveLength + g_discard_pnts );	
			memcpy( rend_CH1_smp_buf , &rend_CH1_smp_buf[g_discard_pnts] ,g_waveLength*sizeof( float ) );// ����ʵ����Ҫ�ĵ�����ȥ�������ĵ�
			memcpy( rend_CH2_smp_buf , &rend_CH2_smp_buf[g_discard_pnts] ,g_waveLength*sizeof( float ) );			
        #endif	
		LOGD( "xin: DOUBLE_CH_�˳��㷨 is : [%s]\n" ,  log_time( ) );
		
		CH1_value[0] = rend_value( rend_CH1_smp_buf , g_waveLength , my_totalrend.total_value_type );	 //������ֵ����ֵ CH1 
		CH2_value[0] = rend_value( rend_CH2_smp_buf , g_waveLength , my_totalrend.total_value_type );	 //������ֵ����ֵ CH2	
		LOGD( "xin: DOUBLE_CH_CH1_value = %f ,  CH2_value = %f" , CH1_value[0] , CH2_value[0] );				
		
        if(power_off_flag == 1)
		{	 
            LOGD("xin: ��ʼ�ص�ʱpower_off_flag ==1");	
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
			LOGD("xin: free����malloc=====�µ�ʱ��ǰ�ͷź�, ���ûص�����======");
			sem_destroy( &run_sem );
			thread_finished_flag = 1;
		    usleep(10000);  // �����߳�ִ�к����flag ִ��
			vibrate_callback_backup.double_ch_callback( invalid_buf  , invalid_buf, 0 ,  false ); /////�ص� UI����
			return NULL;
		}else{		
			vibrate_callback_backup.double_ch_callback( CH1_value , CH2_value , sizeof( CH1_value )/sizeof( float ) , true );	/////�ص�
		}		
        
		LOGD( "xin: DOUBLE_CH_�ص����� is : [%s]\n" ,  log_time( ) );		   
	   
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
		LOGD("xin: free����malloc=====�����ͷź�, ���ûص�����======");
		sem_destroy( &run_sem );
		thread_finished_flag = 1;		       		
	}	    	
	return NULL;
}

void *evalute_level_thread( void* arg ) //�ȼ������߳�
{    
    timewave my_timewave ={0};
    my_timewave = *( struct time_wave_para* )arg;
	
    int i=0;
    int temp_len =0;	
	float evalute_value[1] ={0.0};	 //0λ:��ʾ�ٶ�
	float CH_data[3]={0.0};		
	
	if( g_max_freq == 1000 )
	{
		g_smpLength = ( g_waveLength + g_discard_pnts +31 )*2;	//��� ��1000Ƶ�ʣ���Ҫ1/2 ��㣬���Բɼ����� *2
	}	
	LOGD( "xin: evalute_level_thread_SINGLE_CH_g_discard_pnts = %d ,  g_waveLength = %d ,  g_smpLength = %d"  , g_discard_pnts ,  g_waveLength ,  g_smpLength );
			
    //������Ҫ���ڴ�
    float *evalute_CH1_smp_buf =NULL; //������ ͨ��1 �ɼ�����
	if( evalute_CH1_smp_buf == NULL)
	{
		evalute_CH1_smp_buf =( float* )malloc( g_smpLength*sizeof( float ) );
		if( evalute_CH1_smp_buf == NULL )
		{
			LOGD( "evalute_CH1_smp_buf �����ڴ�ʧ�ܣ�" );
			exit( EXIT_FAILURE );		
		}
		memset( evalute_CH1_smp_buf , 0 , g_smpLength*sizeof( float ) ); 
	}  	
	
	sem_wait( &run_sem );//�ȴ��ź���	
    memset( evalute_CH1_smp_buf , 0 , g_smpLength*sizeof( float ) );  			
    memset(	CH_data , 0, 3*sizeof(float));	
	
	for( i=0;i< g_smpLength;i++ )	//ת���ɼ��ĵ���	
	{					
		analyze_CH_data( g_smp_buf[i] , CH_data );	
		evalute_CH1_smp_buf[i] = CH_data[2]; //��ͨ���񶯲ɼ�Ĭ��CHB ����				
	}	
	dis_dc_func(evalute_CH1_smp_buf,g_smpLength);		
	LOGD( "xin: SINGLE_CH_����ת������ is : [%s]\n" ,  log_time( ) );	
	
	temp_len = g_smpLength; ////FIR ��ͨ�˲�����ͨ������������	
	if( g_max_freq == 1000 )	 //������1000 ��1/2���� 
	{
		enter_FIR_Filter( evalute_CH1_smp_buf  , g_smpLength, g_max_freq); //FIR ��ͨ�˲�
		temp_len = g_smpLength/2 -31; //����󳤶ȱ�Ϊ1/2	
		
		enter_IIR_Filter( evalute_CH1_smp_buf ,temp_len , g_max_freq , g_min_freq ); //IIR ��ͨ�˲�
	}		
	memcpy( evalute_CH1_smp_buf ,  &evalute_CH1_smp_buf[g_discard_pnts] ,  g_waveLength*sizeof( float ) );	
	LOGD( "xin: SINGLE_CH_�˳��㷨 is : [%s]\n" ,  log_time( ) );		
	
	evalute_value[0] = rend_value( evalute_CH1_smp_buf ,  g_waveLength ,  0 );	//�ٶ���Чֵ 0��ʾ��Чֵ
	LOGD( "xin: ������ٶ���Чֵ = %f" ,  evalute_value[0] );	

    if( evalute_CH1_smp_buf != NULL)
	{
		free( evalute_CH1_smp_buf ); 
		evalute_CH1_smp_buf =NULL;	
	}
	LOGD("xin: free����malloc=====�����ͷź�, ���ûص�����======");			
			
	////////�Զ�stop
	start_enable_flag = 0;	
	restart_power_on_flag = 0;  //��ʱ����flag ��0,��ֹ��������ʱ��ס
			
    ioctl( fd ,  SPIDEV_IOC_RXSTREAMOFF ,  NULL ); //ֹͣDMA�ɼ�
	swrite( StopSampleAddr , StopSampleData );//stop fpga
    close( fd );					
	spi_power_off( );  //�µ�	  
	sem_destroy( &run_sem );  
	
	vibrate_callback_backup.single_ch_callback( evalute_value , sizeof( evalute_value )/sizeof( float ) , true ); /////�ص�	
	LOGD( "xin: SINGLE_CH_�ص����� is : [%s]\n" ,  log_time( ) );
		
	return NULL;
}


static int start_vibrate_CH_timewave( struct spictl_device_t* dev ,  int ch_num , struct time_wave_para tWave  )//ʱ����
{	
   LOGD("xin: start_vibrate_CH_timewave_start_num = %d",start_num);
   if(start_num != 0)
   {
	  return 0; 
   }
    memset(read_60K_buf,0, SIZE_60K*sizeof(unsigned char));
    memset(g_max_char_buf ,0, MAX_SIZE*sizeof(unsigned char)); 
	int reg_ret_value =0;
	
	if(start_enable_flag == 1) // 1��ʾǰһ��start �̻߳�û�н�������ʱ������Ӧ�µ�start�� Ϊ0ʱ��ʾstart�߳̽�����
	{
		LOGD("xin: start_enable_flag = %d, ǰһ��start��δִ���꣬��ʱ����Ӧ�µ�start�ӿ�",start_enable_flag);		
        return 0;
	}else{
	    start_enable_flag = 1;
	}
	
    g_chNum = ch_num;	 
	
	g_discard_pnts = 0;
	g_loop_num =0;	
	
	g_waveLength = 0;
	
    g_max_freq = (int)tWave.max_freq;
	g_min_freq = (int)tWave.min_freq;
	g_waveLength = tWave.wave_length;
	
	LOGD("xin: ���ʱ��ʼʱ start_enable_flag = %d, restart_power_on_flag= %d, can_start_flag = %d , thread_finished_flag = %d",start_enable_flag,restart_power_on_flag,can_start_flag,thread_finished_flag);
	LOGD( "xin: start_vibrate_CH_timewave_ch_num = %d , data_type = %d , signal_type = %d ,  min_freq = %d , max_freq = %d , wave_length = %d", ch_num ,  tWave.data_type ,  tWave.signal_type ,  g_min_freq ,  g_max_freq  , g_waveLength );	
    
    LOGD("xin: ��Ӧʱ������ʼ [%s]\n" ,  log_time( )); 
    if(restart_power_on_flag == 1) // ���ڲ�stop DMA��fpga,�ر��豸��flag ��1�� ��������start�ӿ�ʱ���������ϵ磬��������ֹͣ�ɼ��µ�󣬴�flag����Ϊ 0�����²ɼ�ʱ���ϵ�
	{
		;
	}else{
		poweron_spi(  );				
		
		if( g_chNum == SINGLE_CH )	
			reg_ret_value = set_singleCH_vibrate_reg( tWave.signal_type , g_max_freq , g_min_freq );//���õ�ͨ���ɼ��Ĵ���	
		if( g_chNum == DOUBLE_CH ) 
			reg_ret_value = set_doubleCH_vibrate_reg( tWave.signal_type , g_max_freq , g_min_freq );//����˫ͨ���ɼ��Ĵ���
		
		usleep(2000000); //�����ϲ�ʱ���β����������ݣ�������ֱ����ʱ2S,��Ӳ ��Ԥ���ȶ�
		
		
		if( reg_ret_value == -1)
		{
			LOGD("xin: �Ĵ�������ʧ��");
			vibrate_callback_backup.single_ch_callback( vib_reg_fail_buf ,1 ,true);
			start_enable_flag = 0;
            spi_power_off(  );			
			return 0;
		}
	}	   
    
    if (can_start_flag == 1)
	    return 0;    
    
    if( g_waveLength <= 0)
	{
		start_enable_flag = 0; //���ڲ��γ���Ϊ0ʱ����ֹ���ֶ�ֹͣʱ�ٲɼ�ʱ ��start_enable_flag = 0�� ֱ�� �ص�false���ϲ㣬���Դ˴�Ҫ��0
		LOGD("xin: �·��Ĳ��γ��� tWave.wave_length = %d",g_waveLength);	
		return 0;
	}		

	start_num++;
	usleep(100000);//����stop ��start DMA,�Ĵ����ȶ�
	sem_init( &run_sem ,  0 ,  0 );
	
    g_discard_pnts =  Get_InvalidNum(g_max_freq , g_min_freq );  //���������ޣ���ȡIIR��Ҫ�����ĵ���
	   	
	stop_smp_flag = 0;
   	
	pthread_create( &c_id ,  NULL ,  time_wave_thread , ( void* )&tWave );
     	
	signal( SIGIO ,  read_vibrate_data ); 
	
    common_start( );
	
	while( thread_finished_flag == 0); // ���ڵȴ��㷨�߳�����ʱ��1�����߳̽���ʱ��������������
	can_start_flag = 0;
	thread_finished_flag = 0;
		
	power_off_flag = 0;	
	start_enable_flag = 0;
	
	g_smpLength = 0;
	LOGD( "xin: ��Ӧʱ�������� [%s]\n" ,  log_time( ) ); 
	
	usleep( 100000 ); //���ڵײ������ַ�������ַӳ�����		
	return 0;
}

static int start_vibrate_CH_totalrend( struct spictl_device_t* dev ,  int ch_num , struct total_rend_para tRend  )//��ֵ����
{
    LOGD("xin: start_vibrate_CH_totalrend_start_num = %d",start_num);
    if(start_num != 0)
    {
	   return 0; 
    }
	memset(read_60K_buf,0,SIZE_60K*sizeof(unsigned char));
	memset(g_max_char_buf ,0, MAX_SIZE*sizeof(unsigned char));
	
    usleep( tRend.interval_time*1000*1000 ); //�´βɼ����ʱ��  ΢��Ϊ��λ 
	int reg_ret_value =0;
	
	if(start_enable_flag == 1)
	{
		LOGD("xin: start_enable_flag = %d, ����Ӧstart�ӿ�",start_enable_flag);		
        return 0;
	}else{
	    start_enable_flag = 1;
	} 
	
	g_chNum = ch_num;	
	g_discard_pnts = 0;
	g_loop_num =0;	
	
	g_waveLength = 0;
	
    g_max_freq = (int)tRend.max_freq;
    g_min_freq = (int)tRend.min_freq;
	g_waveLength = tRend.wave_length;
	
	LOGD("xin: �����ֵ��ʼʱ start_enable_flag = %d, restart_power_on_flag= %d, can_start_flag = %d , thread_finished_flag = %d", start_enable_flag,restart_power_on_flag,can_start_flag,thread_finished_flag);
	LOGD( "xin: start_vibrate_CH_totalrend_ch_num = %d , data_type = %d , signal_type = %d ,  min_freq = %d , max_freq = %d , wave_length = %d ,interval_time = %f ", ch_num ,  tRend.data_type ,  tRend.signal_type ,  g_min_freq ,  g_max_freq  , g_waveLength, tRend.interval_time);	
	
    LOGD("xin: ��Ӧ��ֵ�����ʼ [%s]\n" ,  log_time( )); 	
	if(restart_power_on_flag == 1) // ���ڲ�stop DMA��fpga,�ر��豸��flag ��1�� ��������start�ӿ�ʱ���������ϵ磬��������ֹͣ�ɼ��µ�󣬴�flag����Ϊ 0�����²ɼ�ʱ���ϵ�
	{
		;
	}else{
		poweron_spi(  );		
		if( g_chNum ==SINGLE_CH )	
			reg_ret_value = set_singleCH_vibrate_reg( tRend.signal_type , g_max_freq , g_min_freq );//���õ�ͨ���ɼ��Ĵ���
		if( g_chNum == DOUBLE_CH ) 
			reg_ret_value = set_doubleCH_vibrate_reg( tRend.signal_type , g_max_freq , g_min_freq );//����˫ͨ���ɼ��Ĵ���
		
		usleep(2000000); //�����ϲ�ʱ���β����������ݣ�������ֱ����ʱ2S,��Ӳ ��Ԥ���ȶ�
		
		if( reg_ret_value == -1)
		{
			LOGD("xin: �Ĵ�������ʧ��");
			vibrate_callback_backup.single_ch_callback( vib_reg_fail_buf ,1 ,true);
			start_enable_flag = 0;
            spi_power_off(  );			
			return 0;
		}
	}
	
	if (can_start_flag == 1)
	    return 0;
	
	if( g_waveLength <= 0)
	{
		start_enable_flag = 0; //���ڲ��γ���Ϊ0ʱ����ֹ���ֶ�ֹͣʱ�ٲɼ�ʱ ��start_enable_flag = 0�� ֱ�� �ص�false���ϲ㣬���Դ˴�Ҫ��0
		LOGD("xin: �·��Ĳ��γ��� tRend.wave_length = %d",g_waveLength);	
		return 0;
	}
	start_num ++;
	sem_init( &run_sem ,  0 ,  0 );
	
    g_discard_pnts =  Get_InvalidNum( g_max_freq ,  g_min_freq );        
    	
	stop_smp_flag = 0;			  
		
	pthread_create( &c_id ,  NULL ,  total_rend_thread , ( void* )&tRend ); 
    
	signal( SIGIO ,  read_vibrate_data ); 
		
    common_start( );
	while( thread_finished_flag== 0); // ���ڵȴ��㷨�߳�����ʱ��1�����߳̽���ʱ��������������
	can_start_flag = 0;
	thread_finished_flag = 0;
	
	power_off_flag = 0;
	start_enable_flag = 0;

	g_smpLength = 0;
	LOGD( "xin: ��Ӧʱ�������� [%s]\n" ,  log_time( ) ); 
	
	usleep( 100000 ); //���ڵײ������ַ�������ַӳ�����	
	return 0;
}

static int start_vibrate_evalute_level( struct spictl_device_t* dev , struct time_wave_para tWave )//�񶯵ȼ�����
{	
    LOGD("xin: start_vibrate_evalute_level_start_num = %d",start_num);
    if(start_num != 0)
    {
	   return 0; 
    }
    memset(read_60K_buf,0,SIZE_60K*sizeof(unsigned char));
    memset(g_max_char_buf ,0, MAX_SIZE*sizeof(unsigned char));
	int reg_ret_value = 0;
	g_discard_pnts = 0;
	g_loop_num = 0;
	g_smpLength = 0;
	g_waveLength = 0;
	
	g_max_freq = (int)tWave.max_freq;
    g_min_freq = (int)tWave.min_freq;
	g_waveLength = tWave.wave_length;
	   
	LOGD( "\nxin: start_vibrate_evalute_level_signal_type = %d ,  min_freq = %d , max_freq = %d , wave_length = %d ",	 tWave.signal_type ,  g_min_freq ,  g_max_freq  , g_waveLength);	
	 
	if( g_waveLength <= 0)
	{
		LOGD("xin: �·��Ĳ��γ��� tWave.wave_length = %d",g_waveLength);
		return 0;
	}
	
	LOGD( "xin: ��Ӧ���������ʼ [%s]\n" ,  log_time( ) ); 	
	
	poweron_spi(  );	
	
	reg_ret_value = set_singleCH_vibrate_reg( tWave.signal_type , g_max_freq , g_min_freq );//���õ�ͨ���ɼ��Ĵ�������������Ϊ �ٶ�ʱ����
	
	usleep(2000000); //�����ϲ�ʱ���β����������ݣ�������ֱ����ʱ2S,��Ӳ ��Ԥ���ȶ�
	
	if( reg_ret_value == -1)
	{
		LOGD("xin: �Ĵ�������ʧ��");
		vibrate_callback_backup.single_ch_callback( vib_reg_fail_buf ,1 ,true);
		start_enable_flag = 0;
		spi_power_off(  );			
		return 0;
	}
	start_num ++;
	sem_init( &run_sem ,  0 ,  0 );	
	
    g_discard_pnts = Get_InvalidNum( g_max_freq , g_min_freq );    	  
    	
	stop_smp_flag = 0;		
			
	pthread_create( &c_id ,  NULL ,  evalute_level_thread , ( void* )&tWave );	
	
	signal( SIGIO ,  read_evalute_data ); 				
	  		 
    common_start( );
	LOGD( "xin: ��Ӧ����������� [%s]\n" ,  log_time( ) ); 
	return 0;
}

static float* spi_get_feature_value( struct spictl_device_t* dev , float pData[] , int data_len )//��ȡ15������ֵ
{	
	memset( feature_ret_value , 0 , FEATURE_NUM*sizeof( float ) ); //ÿ�ν����ȳ�ʼ��0
	feature_value( pData ,  data_len ,  feature_ret_value ); // ��15������ֵ
	
	#if 0
	int i = 0;
	for( i=0;i<FEATURE_NUM;i++ )
	{
		LOGD( "feature_ret_value[%d] = %f" , i , feature_ret_value[i] ); //ֱ��ת��Ϊ���ٶ�ֵ
	}
	#endif 	
	
	return feature_ret_value;	
}

static float* spi_time_to_freq_value( struct spictl_device_t* dev , float pData[] , int data_len  )//ʱ��Ƶ��ӿ�
{
	fft_alg_entry2( pData ,  data_len , 0 , 0 , 0 );	//0Ĭ�ϲ��Ӵ�����ƽ����ƽ����ʽ������
	
	#if 0
	int i =0;
	for(i = 0; i< data_len/2; i++)
	{
		 LOGD( "ret_pData[%d] = %f" , i, pData[i] );	
	}
	#endif 
	
	return pData;    		
}


/////////Ƶ�ʲɼ��������� 
void *freq_wave_thread( void* arg ) //Ƶ���߳�
{	
    freqwave my_freqwave ={0};
	my_freqwave = *( struct freq_wave_para* )arg;		
	int i=0 , j=0 , k=0;	
	float CH_data[3]={0.0};
	
	if( g_chNum == SINGLE_CH ) 
	{
	    g_waveLength = my_freqwave.spectra_num*2.56; //���γ��� = Ƶ������*2.56	
		g_smpLength = my_freqwave.spectra_num*2.56 + g_discard_pnts;	//ʵ�ʲɼ��Ĳ��γ���				
				
		LOGD( "xin: freq_wave_thread_SINGLE_CH_g_discard_pnts = %d ,  spectra_num*2.56 = %d ,  g_smpLength = %d"  , g_discard_pnts ,  g_waveLength ,  g_smpLength );	
		if( g_waveLength < 0 )
		{
			return NULL;
		}
		
		float *freq_CH1_smp_buf =NULL; //ͨ��1����  �ɼ�����
		if( freq_CH1_smp_buf == NULL)
		{
			freq_CH1_smp_buf =( float* )malloc( g_smpLength*sizeof( float ) );
			if( freq_CH1_smp_buf == NULL )
			{
				LOGD( "freq_CH1_smp_buf �����ڴ�ʧ�ܣ�" );
				exit( EXIT_FAILURE );		
			}
			memset( freq_CH1_smp_buf , 0 , g_smpLength*sizeof( float ) );	
		}
		
		float *freq_CH1_IIR_buf =NULL; //ͨ��1 IIR�˲�������
		if( freq_CH1_IIR_buf == NULL)
		{
			freq_CH1_IIR_buf =( float* )malloc( g_waveLength*sizeof( float ) );
			if( freq_CH1_IIR_buf == NULL )
			{
				LOGD( "freq_CH1_IIR_buf �����ڴ�ʧ�ܣ�" );
				exit( EXIT_FAILURE );		
			}
			memset( freq_CH1_IIR_buf , 0 , g_waveLength*sizeof( float ) );
		}
		
		
		while(  1  )
		{		
			sem_wait( &run_sem );//�ȴ��ź���		
			          		
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
				analyze_CH_data( g_smp_buf[i] , CH_data );  //���ý���ͨ�����ݵĺ���								
				freq_CH1_smp_buf[i] = CH_data[2];		 							
			} 
						
			enter_IIR_Filter( freq_CH1_smp_buf , g_smpLength ,( int )my_freqwave.max_freq ,( int )my_freqwave.min_freq ); //IIR ��ͨ�˲��������ǲ�������
			
			memcpy( freq_CH1_IIR_buf , &freq_CH1_smp_buf[g_discard_pnts] , g_waveLength*sizeof( float ) );
			
			fft_alg_entry2( freq_CH1_IIR_buf , g_waveLength , 0 , 0 , 0  ); //0Ĭ�ϲ��Ӵ���ƽ����ʽ��ƽ��������Ϊ0��������
			vibrate_callback_backup.single_ch_callback( freq_CH1_IIR_buf ,  g_waveLength/2 ,  true );	/////Ƶ�׻ص� ������ʵ��UI����һ��								    
			stop_smp_flag = 0;		
		}		
	}
		
	if( g_chNum == DOUBLE_CH ) //˫ͨ���ڴ�Ϊ��ͨ��ʱ������
	{	       
	    g_waveLength =( my_freqwave.spectra_num*2.56 )*2; //���γ��� = Ƶ������*2.56
		g_smpLength =( my_freqwave.spectra_num*2.56 + g_discard_pnts )*2;	//ʵ�ʲɼ��Ĳ��γ���        
		LOGD( "xin:freq_wave_thread_DOUBLE_CH_g_discard_pnts = %d ,  spectra_num*2.56 = %d ,  g_smpLength = %d"  , g_discard_pnts ,  g_waveLength/2 ,  g_smpLength );
		if( g_waveLength < 0 )
		{
			return NULL;
		}
		
        float *freq_CH1_smp_buf =NULL; //ͨ��1����  �ɼ�����
		if( freq_CH1_smp_buf == NULL)
		{
			freq_CH1_smp_buf =( float* )malloc( g_smpLength*sizeof( float ) );
			if( freq_CH1_smp_buf == NULL )
			{
				LOGD( "freq_CH1_smp_buf �����ڴ�ʧ�ܣ�" );
				exit( EXIT_FAILURE );		
			}
			memset( freq_CH1_smp_buf , 0 , g_smpLength*sizeof( float ) );
		}		
		
		float *freq_CH2_smp_buf =NULL; //ͨ��2���� �ɼ�����
		if( freq_CH2_smp_buf == NULL)
		{
			freq_CH2_smp_buf =( float* )malloc( g_smpLength*sizeof( float ) );
			if( freq_CH2_smp_buf == NULL )
			{
				LOGD( "freq_CH2_smp_buf �����ڴ�ʧ�ܣ�" );
				exit( EXIT_FAILURE );		
			}
			memset( freq_CH2_smp_buf , 0 , g_smpLength*sizeof( float ) );
		}

        float *freq_CH1_smp_buf1 =NULL; //ͨ��1����  �ɼ���ʵ����
		if( freq_CH1_smp_buf1 == NULL)
		{
			freq_CH1_smp_buf1 =( float* )malloc(( g_smpLength/2 )*sizeof( float ) );
			if( freq_CH1_smp_buf1 == NULL )
			{
				LOGD( "freq_CH1_smp_buf1 �����ڴ�ʧ�ܣ�" );
				exit( EXIT_FAILURE );		
			}
			memset( freq_CH1_smp_buf1 , 0 ,( g_smpLength/2 )*sizeof( float ) );	
		}		
		
		float *freq_CH2_smp_buf2 =NULL; //ͨ��2���� �ɼ���ʵ����
		if( freq_CH2_smp_buf2 == NULL)
		{
			freq_CH2_smp_buf2 =( float* )malloc(( g_smpLength/2 )*sizeof( float ) );
			if( freq_CH2_smp_buf2 == NULL )
			{
				LOGD( "freq_CH2_smp_buf2 �����ڴ�ʧ�ܣ�" );
				exit( EXIT_FAILURE );		
			}
			memset( freq_CH2_smp_buf2 , 0 ,( g_smpLength/2 )*sizeof( float ) );
		}
		
		
		float *freq_CH1_IIR_buf =NULL; //ͨ��1 IIR���غ������
		if( freq_CH1_IIR_buf == NULL)
		{
			freq_CH1_IIR_buf =( float* )malloc(( g_waveLength/2 )*sizeof( float ) );
			if( freq_CH1_IIR_buf == NULL )
			{
				LOGD( "freq_CH1_IIR_buf �����ڴ�ʧ�ܣ�" );
				exit( EXIT_FAILURE );		
			}
			memset( freq_CH1_IIR_buf , 0 ,( g_waveLength/2 )*sizeof( float ) );	
		}
		
		float *freq_CH2_IIR_buf =NULL; //ͨ��2 IIR���غ������
		if( freq_CH2_IIR_buf == NULL)
		{
			freq_CH2_IIR_buf =( float* )malloc(( g_waveLength/2 )*sizeof( float ) );
			if( freq_CH2_IIR_buf == NULL )
			{
				LOGD( "freq_CH2_IIR_buf �����ڴ�ʧ�ܣ�" );
				exit( EXIT_FAILURE );		
			}
			memset( freq_CH2_IIR_buf , 0 ,( g_waveLength/2 )*sizeof( float ) );		
		}		
		
		while( 1 )
		{		
			sem_wait( &run_sem );//�ȴ��ź���			
						
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
				analyze_CH_data( g_smp_buf[i] , CH_data );  //���ý���ͨ�����ݵĺ���	
				freq_CH1_smp_buf[i] = CH_data[1];
				freq_CH2_smp_buf[i] = CH_data[2];				
			}			
			for( k=0;k<g_smpLength/2;k++ )
			{					
				freq_CH1_smp_buf1[k] = freq_CH1_smp_buf[k*2+1]; //ͨ��1read_evalute_data
				freq_CH2_smp_buf2[k] = freq_CH2_smp_buf[k*2];//ͨ��2					
			}
					
			enter_IIR_Filter( freq_CH1_smp_buf1 , g_smpLength/2 ,( int )my_freqwave.max_freq ,( int )my_freqwave.min_freq );  //IIR �˲�����Ϊ �ɼ����ȵ�һ��
			enter_IIR_Filter( freq_CH2_smp_buf2 , g_smpLength/2 ,( int )my_freqwave.max_freq ,( int )my_freqwave.min_freq );
         							
            memcpy( freq_CH1_IIR_buf , &freq_CH1_smp_buf1[g_discard_pnts] ,( g_waveLength/2 )*sizeof( float ) );
            memcpy( freq_CH2_IIR_buf , &freq_CH2_smp_buf2[g_discard_pnts] ,( g_waveLength/2 )*sizeof( float ) );			
						
			fft_alg_entry2( freq_CH1_IIR_buf , g_waveLength/2 , 0 , 0 , 0  );//0Ĭ�ϲ��Ӵ���ƽ����ʽ��ƽ��������Ϊ0��������			
			fft_alg_entry2( freq_CH2_IIR_buf , g_waveLength/2 , 0 , 0 , 0  );	//0Ĭ�ϲ��Ӵ���ƽ����ʽ��ƽ��������Ϊ0��������		
			
			vibrate_callback_backup.double_ch_callback( freq_CH1_IIR_buf ,  freq_CH2_IIR_buf ,  g_waveLength/4 ,  true );	/////Ƶ��ֻ�ص� ǰһ������							
			stop_smp_flag = 0;		
		}		
	}      		
	return NULL;
}

static int start_vibrate_CH_freqwave( struct spictl_device_t* dev , int ch_num , struct freq_wave_para fWave  )// ����Ƶ����
{
	g_chNum = ch_num;	 
	LOGD( "xin: start_vibrate_CH_freqwave=====ch_num = %d , data_type = %d , signal_type = %d , min_freq = %f , max_freq = %f , spectra_num = %d , average_num = %d , average_mode = %d , window_type = %d , range_mode = %d , trig_mode = %d , range_accel_value = %d" , 													
	g_chNum ,  fWave.data_type ,  fWave.signal_type ,  fWave.min_freq ,  fWave.max_freq  , fWave.spectra_num ,  fWave.average_num ,  fWave.average_mode ,  fWave.window_type , fWave.range_mode , fWave.trig_mode , fWave.range_accel_value );
	 
	if( g_chNum == SINGLE_CH )	
		set_singleCH_vibrate_reg( fWave.signal_type , fWave.max_freq , fWave.min_freq );//���õ�ͨ���ɼ��Ĵ���	
	if( g_chNum == DOUBLE_CH ) 
		set_doubleCH_vibrate_reg( fWave.signal_type , fWave.max_freq , fWave.min_freq );//����˫ͨ���ɼ��Ĵ���	

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


////////////ת�ٲɼ���������
void *rotation_CH_thread( void* arg ) //ת��ͨ���߳�
{
	LOGD( "rotation_CH_thread===========" );	
    int i=0;  
	float CH_data[3]={0.0};	
	
    float rotation_array[2] ={0.0}; //ת������
	while( 1 )
	{
		sem_wait( &run_sem );//�ȴ��ź���
		if( exit_thread_flag )
		{
			return NULL;
		}
        for( i=0;i< 100;i++ )
		{		
			analyze_CH_data( g_smp_buf[i] , CH_data );  //���ý���ͨ�����ݵĺ���							
			rotation_array[0] = CH_data[0];
			LOGD( "rotation_array[%d] = %f" , i , rotation_array[i] );
		}
		stop_smp_flag = 0;
	}	
	return NULL;
}

static int start_rotation_CH( struct spictl_device_t* dev )//����ת��ͨ���ɼ�
{
	LOGD( "start_rotation_CH" );	
	set_rotation_reg( );	//����ת�ټĴ���
	sem_init( &run_sem ,  0 ,  0 );	
	
	exit_thread_flag = false;      
	stop_smp_flag = 0;	
	g_loop_num =0;
	
	pthread_create( &c_id ,  NULL ,  rotation_CH_thread ,  NULL );
	signal( SIGIO ,  read_vibrate_data );
    common_start( );
	return 0;
}


