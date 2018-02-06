#define LOG_TAG "spiStub"
#include <hardware/hardware.h>
#include <hardware/imx6q_calibrate.h>
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
#include <hardware/imx6q_vibrate_calibrate_config.h>
#include <hardware/imx6q_vibrate_calibrate.h>
#include <hardware/imx6q_common_config.h>

#define DEVICE_NAME "/dev/mxc_spidev1" //从设备，用于上传采集数据
#define MODULE_NAME "xxxx"
#define MODULE_AUTHOR "gxl@126.com"

/////////////////////////////////////////////////
#define debug          0

pthread_t  c_id; // 开辟线程 c_id:计算
sem_t   run_sem; //内部信号名  run_sem: 继续运行信号

volatile int fd = -1; //从SPI设备文件
volatile int g_smpLength = 0; //实际采集的波形长度
int *g_smp_buf = NULL;  //全局采集的数据
unsigned char read_60K_buf[SIZE_60K] = {0};// 读60K数据buf
unsigned char g_max_char_buf[MAX_SIZE1] = {0};

volatile bool thread_finished_flag = false; //表示当前线程完成标识
volatile bool restart_power_on_flag = false;  // spi 重新上电标识, 0标识需要重新上电
volatile bool start_enable_flag = false;// 响应start接口标识
volatile bool stop_vibrate_smp_flag = false; // 点击stop时的停止振动采集标识
volatile bool stop_vread_flag = false;//停止振动读取数据标识

volatile int  calib_smp_discard_flag = 0; //校准丢弃前16K数据，解决波形前面不稳bug

//设备打开和关闭接口
static int spi_device_open(const struct hw_module_t* module, const char* name, struct hw_device_t** device);
static int spi_device_close(struct hw_device_t* device);
//设备访问接口
static int start_vibrate_calibration(struct spictl_device_t* dev, struct time_wave_para tWave);//启动振动校准
static int stop_vibrate_sample(struct spictl_device_t* dev); //停止振动采集
struct spictl_device_t* dev;

//模块方法表
static struct hw_module_methods_t spi_module_methods = {
   open: spi_device_open
};

//JNI回调接口
static SpiVibrateCallbacks   vibrate_callback_backup;  //振动回调接口

//模块实例变量
struct spictl_module_t HAL_MODULE_INFO_SYM = {
    common:{
        tag: HARDWARE_MODULE_TAG,
        version_major: 1,
        version_minor: 0,
        id: CALIBRATECTL_HARDWARE_MODULE_ID,
        name: MODULE_NAME,
        author: MODULE_AUTHOR,
        methods: &spi_module_methods,
    }
};

static void initVibrate(SpiVibrateCallbacks* callbacks){
	vibrate_callback_backup = *callbacks;
}

static const SpiVibrateInterface  mSpiVibrateInterface = {
	initVibrate,
};

const SpiVibrateInterface* spi_vibrate_interface(struct spictl_device_t* dev){//振动回调接口
    return &mSpiVibrateInterface;
}

static int spi_device_open(const struct hw_module_t* module, const char* name, struct hw_device_t** device){
    struct spictl_device_t* dev;
	dev = (struct spictl_device_t*)malloc(sizeof(struct spictl_device_t));
	if(dev == NULL)
	{
		LOGD("imx6q_vibrate: failed to alloc space");
		return -EFAULT;
	}
	memset(dev, 0, sizeof(struct spictl_device_t));
    dev->common.tag = HARDWARE_DEVICE_TAG;
    dev->common.version = 0;
    dev->common.module = (hw_module_t*)module;
    dev->common.close = spi_device_close;
	dev->start_vibrate_calibrate = start_vibrate_calibration; //振动校准
	dev->stop_vibrate_ad = stop_vibrate_sample; //振动停止采集
	dev->get_vibrate_interface = spi_vibrate_interface; //振动回调接口
    *device = &(dev->common);
    return 0;
}

static int spi_device_close(struct hw_device_t* device){
    struct spictl_device_t* devspi_device = (struct spictl_device_t*)device;
    if(devspi_device){
        close(devspi_device->fd);
        free(devspi_device);
    }
    return 0;
}

void flag_init() //对所有标识flag 清0
{
    LOGD("振动flag_init====");
    thread_finished_flag = false;
    restart_power_on_flag = false;
    stop_vibrate_smp_flag = false;
    start_enable_flag = false;
	stop_vread_flag = false;
}

void common_start()//打开设备和开始fpga采集
{
    LOGD("common_start stop_vibrate_smp_flag = %d, fd = %d, [%s]\n", stop_vibrate_smp_flag, fd, log_time());
    int res = 0;
    if(fd != -1)
    {
        res = ioctl(fd, SPIDEV_IOC_RXSTREAMOFF, NULL);  //停止DMA搬运
        if(res != 0) //启动DMA采集失败时，重新启动
        {
            LOGD("====打开设备前先停一次，防止前一次有DMA没有关闭， DMA采集-停止失败=====, fd = %d", fd);
        }else{
            LOGD("====打开设备前先停一次，防止前一次有DMA没有关闭， DMA采集-停止成功, fd = %d", fd);
        }
    }

    if(stop_vibrate_smp_flag) // 停止振动采集标识
	{
        LOGD("common_start打开设备时检测到停止振动采集标识, 不再继续打开设备文件, 直接return出去");
		return;
    }

    if((fd = open(DEVICE_NAME, O_RDWR)) == -1){
        LOGD("common_start打开从spi设备 /dev/mxc_spidev1 失败 -- %s.", strerror(errno));
    }else{
		LOGD("common_start打开从spi设备 /dev/mxc_spidev1 成功. fd = %d", fd);
	}

    fcntl(fd, F_SETOWN, getpid());//将当前进程PID设置为fd文件所对应驱动程序将要发送SIGIO, SIGUSR信号进程PID
    int oflags = fcntl(fd, F_GETFL);//获取fd的打开方式
    fcntl(fd, F_SETFL, oflags | FASYNC);//将fd的打开方式设置为FASYNC --- 即 支持异步通知

   	usleep(DELAY_300000); //用于 stop_vibrate_smp_ flag 和 fd 的在停止时的变化能够生效，防止后面多一次打开DMA, 重要！！！！
    if(stop_vibrate_smp_flag) // 停止振动采集标识
	{
        LOGD("common_start启动DMA时检测到停止振动采集标识, 不再继续打开DMA，直接return出去");
        return;
	}else{
        if(fd == -1)
        { 
        	LOGD("common_start启动DMA时检测到fd = -1, 不再继续打开DMA，直接return出去0000, [%s]\n", log_time());
        	return;  
        }else{
        	if(fd == -1)
        	{        		
        		LOGD("common_start启动DMA时检测到fd = -1, 不再继续打开DMA，直接return出去1111, [%s]\n", log_time());
        		return;
        	}        
        	LOGD("common_startDMA采集 fd = %d, [%s]\n", fd, log_time());
            res = ioctl(fd, SPIDEV_IOC_RXSTREAMON, NULL);//启动DMA采集
            if(res != 0) //启动DMA采集失败时，重新启动
            {
                LOGD("common_startDMA采集-启动失败=====, fd = %d, [%s]\n", fd, log_time());                
            }else{
                LOGD("common_startDMA采集-启动成功, fd = %d, [%s]\n", fd, log_time());
            	swrite(StartSampleAddr, StartSampleData);// 启动FPGA采集板
            }
        }            
    }
}

inline void stop_fpga_dma()//停止FPGA采集, DMA搬运，关闭设备文件
{
    if(fd != -1)
    {
        LOGD("stop_fpga-dma停止FPGA, DMA stop_vibrate_smp_ flag = %d, fd = %d, is : [%s]\n", stop_vibrate_smp_flag, fd, log_time());
        int res = ioctl(fd, SPIDEV_IOC_RXSTREAMOFF, NULL);  //停止DMA搬运
        if(res != 0) //启动DMA采集失败时，重新启动
        {
            LOGD("stop_fpga-dma停止FPGA DMA: DMA采集-停止失败=====, fd = %d", fd);
        }else{
            LOGD("stop_fpga-dma停止FPGA DMA: DMA采集-停止成功, fd = %d", fd);
        }
        if(close(fd) == 0)//关闭从设备文件
        {
            fd = -1; //关闭成功重新初始fd
            LOGD("stop_fpga-dma关闭从SPI设备文件成功,将fd重新初始化为 %d", fd);
        }
        swrite(StopSampleAddr, StopSampleData);//stop fpga
    }
	if(stop_vibrate_smp_flag) // 停止振动采集标识
	{
        LOGD("stop_fpga-dma停止FPGA, DMA时测到停止振动采集标识，提前下电并post信号量");
        //poweroff_spi(); //停止时下电，会使后台自启动时寄存器失败，从而快速回调寄存器失败异常，这样会造成上层APP收到寄存器失败后变成开始采集，所以去掉此代码
        sem_post(&run_sem);
        return;
	}
}

static int stop_vibrate_sample(struct spictl_device_t* dev)//停止振动采集，会将stop_vibrate_smp_ flag 置为1，若为1，stop_fpga-dma时会下电
{
    LOGD("停止振动采集开始 fd = %d, start_enable_ flag = %d is : [%s]\n", fd, start_enable_flag, log_time());
    stop_vibrate_smp_flag = true;
	if(!start_enable_flag)
	{
		LOGD("停止振动采集, 此时start线程刚结束，无线程存在, 直接回调false给上层");
		usleep(DELAY_500000); //用于等待stop_vibrate_smp_ flag 变量变化，防止在还没有关完前，重新响应start
        if(fd != -1)
        {
            stop_fpga_dma();
        }
        poweroff_spi();  //下电
        masterspi_close();
        
        flag_init();//停止和异常回调 对所有标识flag清0
		vibrate_callback_backup.stop_ch_callback(false);
        return 0;
	}

    //LOGD("停止振动采集stop_vibrate_smp_flag = 1，立即停止读取数据, 调用stop_fpga_ dma函数");
    stop_vread_flag = true; //停止读取数据
    stop_fpga_dma();
	return 0;
}

////////////振动采集功能如下

int c_loop_num = 0;
void read_vibrate_data(int signo) //读取振动采集数据
{
    LOGD("读取振动数据时stop_vread_ flag = %d, c_loop_num = %d, g_smpLength = %d, stop_vibrate_smp_ flag =%d, [%s]\n", stop_vread_flag, c_loop_num, g_smpLength, stop_vibrate_smp_flag, log_time());

	if(stop_vread_flag)
	{
		return;
	}

	int i = 0;
	int total_len = g_smpLength;  //采样总点数
	int shang = total_len/SIZE_15360;  //15360 的倍数
	int yu = total_len%SIZE_15360;  // 余数
	
	if(shang > 0 && yu > 0)
	{
		if(read(fd, read_60K_buf, SIZE_60K) < 0)
		{
			LOGD("Error: spi slave device read fail !\n ");
		}
		if(c_loop_num < shang)
		{
			memcpy(&g_max_char_buf[SIZE_60K*c_loop_num], &read_60K_buf, SIZE_60K);
			c_loop_num++;
           /*  if(c_loop_num == 1)
            {
                if(!calib_smp_discard_flag)
                {
                    c_loop_num = 0; 
                    calib_smp_discard_flag ++; 
                }
            } */
		}else if(c_loop_num == shang)
		{
            /* if(calib_smp_discard_flag >= 1) //表示丢前一波数据
            { */ 
                stop_vread_flag = true; //flag置1, 停止继续读数据
                stop_fpga_dma();

                memcpy(&g_max_char_buf[SIZE_60K*shang], &read_60K_buf, yu*sizeof(float));

                g_smp_buf = (int*)&(g_max_char_buf);
                c_loop_num = 0;
                LOGD("post振动信号量(总长大于16K且有余数) : [%s]\n", log_time());                
                sem_post(&run_sem);
             /*    calib_smp_discard_flag = 0;
            }  */
		}
	}	
}

int v_signal_type = 0, v_voltage_range = 0, v_max_freq = 0, v_min_freq = 0,v_wave_length = 0;
void *vibrate_calib_thread(void* arg) //振动校准线程
{    
    timewave my_timewave;
    my_timewave = *(struct time_wave_para*)arg;	
    int i = 0;
    float calib_value[2] = {0.0};	 //0位:表示增益值，1位：表示偏移值
    float smp_max = 0.0, smp_ave = 0.0; //采集最大值，采集平均值
	float v_CH_data[3] = {0.0};	
      
    g_smpLength = v_wave_length;	//实际采集的波形长度
   // LOGD("振动校准线程v_wave_length = %d, g_smpLength = %d" , v_wave_length, g_smpLength);
            
    //分配需要的内存
    float *calib_CH1_smp_buf = NULL; //振动校准  通道1 采集数据
    calib_CH1_smp_buf =(float*)malloc(g_smpLength * sizeof(float));
    if(calib_CH1_smp_buf == NULL)
    {
        LOGD("calib_CH1_smp_buf 分配内存失败！");
        exit(EXIT_FAILURE);		
    }
    memset(calib_CH1_smp_buf, 0, g_smpLength*sizeof(float)); 
    
    sem_wait(&run_sem);//等待信号量	
        
    memset(calib_CH1_smp_buf, 0, g_smpLength*sizeof(float));  			
    memset(	v_CH_data, 0, 3*sizeof(float));	
    
    for(i=0; i< g_smpLength; i++)	//转换采集的点数	
    {					
        analyze_CH_data(g_smp_buf[i], v_CH_data);	
        calib_CH1_smp_buf[i] = v_CH_data[2]; //单通道振动采集默认CHB 数据   
    }	
                   
    smp_max = rend_value(calib_CH1_smp_buf, v_wave_length, SG_TIMESIGNAL_HANDLE_MAX);	//最大值 4
    smp_ave = rend_value(calib_CH1_smp_buf, v_wave_length, SG_TIMESIGNAL_HANDLE_DC);	//平均值 3       
    LOGD("计算出振动校准 采集最大值 = %f, 平均值(偏移值) = %f\n", smp_max, smp_ave);

    //偏移值
    calib_value[1] = smp_ave;
    //计算增益值
    calib_value[0] = calculate_gain(v_signal_type, smp_max, smp_ave, v_voltage_range);//采集信号类型，最大值，采集平均值，电压量程
    //写增益，偏移到Fram       
    write_calib_para(v_signal_type, calib_value[0], calib_value[1], v_max_freq, v_min_freq, v_voltage_range);//采集信号类型，增益，偏移，上限，下限，电压量程
    
    if(calib_CH1_smp_buf != NULL)
    {
        free(calib_CH1_smp_buf); 
        calib_CH1_smp_buf = NULL;	
    }
    
    poweroff_spi();  //下电
    masterspi_close();
    thread_finished_flag = true;

    flag_init();//正常回调时对所有标识flag清0
    
    LOGD("总值数据线程正常回调数据 restart_power_on_flag = %d : [%s]\n", restart_power_on_flag, log_time());
    vibrate_callback_backup.single_ch_callback(calib_value, sizeof(calib_value)/sizeof(float), true); //回调振动校准	
        
	return NULL;
}


static int start_vibrate_calibration(struct spictl_device_t* dev, struct time_wave_para tWave)//振动校准
{	
	LOGD("点击振动等级开始时 stop_vibrate_smp_ flag = %d, restart_power_on_flag= %d, thread_finished_flag = %d", stop_vibrate_smp_flag, restart_power_on_flag, thread_finished_flag);
    
	memset(read_60K_buf, 0, SIZE_60K*sizeof(unsigned char));
    memset(g_max_char_buf, 0, MAX_SIZE1*sizeof(unsigned char));
    
    LOGD("振动校准开始时 restart_power_on_flag= %d, thread_finished_flag = %d", restart_power_on_flag, thread_finished_flag);
    c_loop_num = 0;
    v_signal_type = tWave.signal_type;
    v_voltage_range = tWave.voltage_range;
    v_max_freq = (int)tWave.max_freq;
    v_min_freq = (int)tWave.min_freq;
    v_wave_length = tWave.wave_length;    
    
	LOGD("响应振动校准开始tWave.chan_num = %d, signal_type = %d, max_freq = %d, min_freq = %d, wave_length = %d, v_voltage_range = %d",tWave.chan_num, tWave.signal_type, v_max_freq, v_min_freq, v_wave_length, v_voltage_range);	
    if(!is_valid_length(v_wave_length))
	{
		LOGD("振动校准下发的长度不支持 tWave.wave_length = %d", v_wave_length);
		return 0;
	}
    
    poweron_spi();
    masterspi_open();     
    set_singleCH_vibrate_reg(tWave.chan_num, v_signal_type, v_max_freq, v_min_freq, TEST_MODE, v_voltage_range);//校准必须是测试版本,信号发生器上测试
	    
    usleep(DELAY_5000000); //用于上层时域波形不丢波形数据，在这里直接延时5S, 让硬件预热稳定
	
	sem_init(&run_sem, 0, 0);
	pthread_create(&c_id, NULL, vibrate_calib_thread, (void*)&tWave);	
	
	signal(SIGIO, read_vibrate_data); 				
	common_start();
    
    while(!thread_finished_flag); // 用于等待算法线程线束时置1，若线程结束时，继续往下运行
	    
    usleep(DELAY_50000); 
	LOGD("响应振动校准结束 [%s]\n", log_time()); 
	return 0;
}