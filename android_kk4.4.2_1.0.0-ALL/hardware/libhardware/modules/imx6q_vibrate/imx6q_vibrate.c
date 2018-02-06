#define LOG_TAG "spiStub"
#include <hardware/hardware.h>
#include <hardware/imx6q_vibrate.h>
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
#include <hardware/imx6q_vibrate_config.h>
#include <hardware/imx6q_spi_vibrate.h>
#include <hardware/imx6q_iir_filter.h>
#include <hardware/imx6q_fir_filter.h>
#include <hardware/imx6q_common_config.h>

#define DEVICE_NAME "/dev/mxc_spidev1" //从设备，用于上传采集数据
#define MODULE_NAME "xxxx"
#define MODULE_AUTHOR "gxl@126.com"

/////////////////////////////////////////////////
#define debug          0
#define HAVE_ALGORITHM  1  //是否经过算法
#define READ_CALIB_PARA 1  //从FRAM读取校准参数

pthread_t  c_id; // 开辟线程 c_id:计算
sem_t   run_sem; //内部信号名  run_sem: 继续运行信号

volatile int fd = -1; //从SPI设备文件
volatile int vol_range = 0; //全局电压量程, 0表示电压量程是25V
volatile int smp_length = 0; //实际采集的波形长度

int *smp_buf = NULL;  //全局采集的数据

unsigned char read_60K_buf[SIZE_60K] = {0};// 读60K数据buf
unsigned char max_char_buf[MAX_SIZE] = {0};
float feature_ret_value[FEATURE_NUM] = {0.0}; //15个特征值

volatile bool thread_finished_flag = false; //表示当前线程完成标识
volatile bool restart_power_on_flag = false;  // spi 重新上电标识, 0标识需要重新上电
volatile bool start_enable_flag = false;// 响应start接口标识
volatile bool stop_vibrate_smp_flag = false; // 点击stop时的停止振动采集标识
volatile bool stop_vread_flag = false;//停止振动读取数据标识
volatile bool auto_voltagerange_flag = false; //自动电压量程标识


//设备打开和关闭接口
static int spi_device_open(const struct hw_module_t* module, const char* name, struct hw_device_t** device);
static int spi_device_close(struct hw_device_t* device);
//设备访问接口

static int start_vibrate_CH_timewave(struct spictl_device_t* dev, struct time_wave_para tWave);//启动振动采集 时域波形
static int start_vibrate_CH_totalrend(struct spictl_device_t* dev, struct total_rend_para tRend);//启动振动采集 总值趋势波形
static int start_vibrate_evalute_level(struct spictl_device_t* dev, struct time_wave_para tWave);//启动振动等级评估
static int stop_vibrate_sample(struct spictl_device_t* dev); //停止振动采集

static float* spi_get_feature_value(struct spictl_device_t* dev, float pData[], int data_len); //获取15个特征值接口
static float* spi_time_to_freq_value(struct spictl_device_t* dev, float pData[], int data_len);  //时域到频域接口
static int spi_set_val(struct spictl_device_t* dev, int val);
static int spi_get_val(struct spictl_device_t* dev, int* val);

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
        id: VIBRATECTL_HARDWARE_MODULE_ID,
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
	if (dev == NULL)
	{
		LOGD("imx6q_vibrate: failed to alloc space");
		return -EFAULT;
	}
	memset(dev, 0, sizeof(struct spictl_device_t));
    dev->common.tag = HARDWARE_DEVICE_TAG;
    dev->common.version = 0;
    dev->common.module = (hw_module_t*)module;
    dev->common.close = spi_device_close;
	dev->set_val = spi_set_val;
    dev->get_val = spi_get_val;
	dev->start_vibrate_timewave = start_vibrate_CH_timewave;  //时域波形
	dev->start_vibrate_totalrend = start_vibrate_CH_totalrend;	//总值趋势
	dev->start_vibrate_evalute = start_vibrate_evalute_level; //等级评估
	dev->stop_vibrate_ad = stop_vibrate_sample; //振动停止采集
	dev->spi_feature_value = spi_get_feature_value;	 //15个特征值
	dev->spi_timetofreq_value = spi_time_to_freq_value;	//时域到频域接口
	dev->get_vibrate_interface = spi_vibrate_interface; //振动回调接口
    *device = &(dev->common);
    return 0;
}

static int spi_device_close(struct hw_device_t* device){
    struct spictl_device_t* devspi_device = (struct spictl_device_t*)device;
    if (devspi_device){
        close(devspi_device->fd);
        free(devspi_device);
    }
    return 0;
}

static int spi_set_val(struct spictl_device_t* dev, int val){
    LOGD("imx6q_vibrate: set value %d to device.", val);
    //write(dev->fd, &val, sizeof(val));
    return 0;
}

static int spi_get_val(struct spictl_device_t* dev, int* val){
    if (!val){
        LOGD("imx6q_vibrate: error val pointer");
        return -EFAULT;
    }
    //read(dev->fd, val, sizeof(*val));
    LOGD("imx6q_vibrate: get value %d from device", *val);
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
    auto_voltagerange_flag = false;
}

void common_start()//打开设备和开始fpga采集
{
    LOGD("common_start stop_vibrate_smp_flag = %d, fd = %d, [%s]\n", stop_vibrate_smp_flag, fd, log_time());
    int res = 0;
    if (fd != -1)
    {
        res = ioctl(fd, SPIDEV_IOC_RXSTREAMOFF, NULL);  //停止DMA搬运
        if (res != 0) //启动DMA采集失败时，重新启动
        {
            LOGD("====打开设备前先停一次，防止前一次有DMA没有关闭， DMA采集-停止失败=====, fd = %d", fd);
        }else{
            LOGD("====打开设备前先停一次，防止前一次有DMA没有关闭， DMA采集-停止成功, fd = %d", fd);
        }
    }

    if (stop_vibrate_smp_flag) // 停止振动采集标识
	{
        LOGD("common_start打开设备时检测到停止振动采集标识, 不再继续打开设备文件, 直接return出去");
		return;
    }

    if ((fd = open(DEVICE_NAME, O_RDWR)) == -1){
        LOGD("common_start打开从spi设备 /dev/mxc_spidev1 失败 -- %s.", strerror(errno));
    }else{
		LOGD("common_start打开从spi设备 /dev/mxc_spidev1 成功. fd = %d", fd);
	}

    fcntl(fd, F_SETOWN, getpid());//将当前进程PID设置为fd文件所对应驱动程序将要发送SIGIO, SIGUSR信号进程PID
    int oflags = fcntl(fd, F_GETFL);//获取fd的打开方式
    fcntl(fd, F_SETFL, oflags | FASYNC);//将fd的打开方式设置为FASYNC --- 即 支持异步通知

   	usleep(DELAY_300000); //用于 stop_vibrate_smp_ flag 和 fd 的在停止时的变化能够生效，防止后面多一次打开DMA, 重要！！！！
    if (stop_vibrate_smp_flag) // 停止振动采集标识
	{
        LOGD("common_start启动DMA时检测到停止振动采集标识, 不再继续打开DMA，直接return出去");
        return;
	}else{
        if (fd == -1)
        { 
        	LOGD("common_start启动DMA时检测到fd = -1, 不再继续打开DMA，直接return出去0000, [%s]\n", log_time());
        	return;  
        }else{
        	if (fd == -1)
        	{        		
        		LOGD("common_start启动DMA时检测到fd = -1, 不再继续打开DMA，直接return出去1111, [%s]\n", log_time());
        		return;
        	}        
            res = ioctl(fd, SPIDEV_IOC_RXSTREAMON, NULL);//启动DMA采集
            if (res != 0) //启动DMA采集失败时，重新启动
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
    if (fd != -1)
    {
        //LOGD("stop_fpga-dma停止FPGA, DMA stop_vibrate_smp_ flag = %d, fd = %d, is : [%s]\n", stop_vibrate_smp_flag, fd, log_time());
        int res = ioctl(fd, SPIDEV_IOC_RXSTREAMOFF, NULL);  //停止DMA搬运
        if (res != 0) //启动DMA采集失败时，重新启动
        {
            LOGD("stop_fpga-dma停止FPGA DMA: DMA采集-停止失败=====, fd = %d", fd);
        }else{
            LOGD("stop_fpga-dma停止FPGA DMA: DMA采集-停止成功, fd = %d", fd);
        }
        if (close(fd) == 0)//关闭从设备文件
        {
            fd = -1; //关闭成功重新初始fd
            LOGD("stop_fpga-dma关闭从SPI设备文件成功,将fd重新初始化为 %d", fd);
        }
        swrite(StopSampleAddr, StopSampleData);//stop fpga
    }

	if (stop_vibrate_smp_flag) // 停止振动采集标识
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
	if (!start_enable_flag)
	{
		LOGD("停止振动采集, 此时start线程刚结束，无线程存在, 直接回调false给上层");
		usleep(DELAY_500000); //用于等待stop_vibrate_smp_ flag 变量变化，防止在还没有关完前，重新响应start
        if (fd != -1)
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
void read_evalute_data(int signo) //读取振动评估数据    通道2, 下限10HZ, 上限1000, 长度4096
{
    int stop_esmpflag = 0;
	if (stop_esmpflag)
	{
		return;
	}
	int total_len = smp_length;  //采样总点数
	int shang = total_len/SIZE_15360;  //15360 = 60K 的倍数
	int yu = total_len%SIZE_15360;  // 余数

	if (shang < 1)
	{
		if (read(fd, max_char_buf, yu*sizeof(float)) < 0)
		{
			LOGD("Error: spi slave device read fail !\n ");
		}
		stop_esmpflag = 1;
		smp_buf = (int*)&(max_char_buf);
	    sem_post(&run_sem);
	}
}

int v_loop_num = 0;
void read_vibrate_data(int signo) //读取振动采集数据
{
    //LOGD("读取振动数据时stop_vread_ flag = %d, v_loop_num = %d, smp_length = %d, stop_vibrate_smp_ flag =%d, [%s]\n", stop_vread_flag, v_loop_num, smp_length, stop_vibrate_smp_flag, log_time());

	if (stop_vread_flag)
	{
		return;
	}

	int i = 0;
	int total_len = smp_length;  //采样总点数
	int shang = total_len/SIZE_15360;  //15360 的倍数
	int yu = total_len%SIZE_15360;  // 余数

	if (shang < 1)
	{
		if (read(fd, max_char_buf, yu*sizeof(float)) < 0)
		{
			LOGD("Error: spi slave device read fail !\n ");
		}
		stop_vread_flag = true;
        stop_fpga_dma();

		smp_buf = (int*)&(max_char_buf);

		//LOGD("post振动信号量(总长小于16K) : [%s]\n", log_time());
        v_loop_num = 0;
		sem_post(&run_sem);
	}

	if (shang > 0 && yu > 0)
	{
		if (read(fd, read_60K_buf, SIZE_60K) < 0)
		{
			LOGD("Error: spi slave device read fail !\n ");
		}
		if (v_loop_num < shang)
		{
			memcpy(&max_char_buf[SIZE_60K*v_loop_num], &read_60K_buf, SIZE_60K);
			v_loop_num++;
		}
        else if (v_loop_num == shang)
		{
            stop_vread_flag = true; //flag置1, 停止继续读数据
            stop_fpga_dma();

			memcpy(&max_char_buf[SIZE_60K*shang], &read_60K_buf, yu*sizeof(float));

            smp_buf = (int*)&(max_char_buf);

            LOGD("post振动信号量(总长大于16K且有余数) : [%s]\n", log_time());
            v_loop_num = 0;
            sem_post(&run_sem);
		}
	}
	if (shang > 0 && yu == 0)
	{
		if (read(fd, read_60K_buf, SIZE_60K) < 0)
		{
			LOGD("Error: spi slave device read fail !\n ");
		}

		if (v_loop_num < shang)
		{
			memcpy(&max_char_buf[SIZE_60K*v_loop_num], &read_60K_buf, SIZE_60K);
			v_loop_num++;
		}
		else if (v_loop_num == shang)
		{
            stop_vread_flag = true;
            stop_fpga_dma();

            smp_buf = (int*)&(max_char_buf);

            //LOGD("post振动信号量(总长大于16K, 无余数) : [%s]\n", log_time());
            v_loop_num = 0;
            sem_post(&run_sem);
		}
	}
}


void *time_wave_thread(void* arg) //时域线程
{
    timewave  my_timewave;
    my_timewave = *(struct time_wave_para*)arg;
    int t_max_freq = (int)my_timewave.max_freq;
    int t_min_freq = (int)my_timewave.min_freq;
    int t_discard_pnts = get_invalid_num(t_max_freq, t_min_freq);
    int t_wave_length = my_timewave.wave_length;
    
    float t_calib_para[2] = {0.0}; //calib_para[0]: 表示增益，calib_para[1]: 表示偏移

    LOGD("时域线程中t_discard_pnts = %d, t_max_freq = %d, t_min_freq = %d, t_wave_length = %d",t_discard_pnts, t_max_freq, t_min_freq, t_wave_length);

	int i= 0;
    int t_temp_len = 0;
	float t_CH_data[3]= {0.0};	 //各振动通道数据
	float t_pkvalue = 0.0;
    float t_invalid_buf[2] = {0.0}; // power off时回调的假buf, 供接口使用，实际无意义
    float probe_vol_range_buf[3] = {10005.0}; //用于电压量程探测回调的buf

	if (my_timewave.chan_num == CH_B)
	{
        
		if (t_max_freq == FREQ_500 || t_max_freq == FREQ_2500)
		{
			smp_length = (t_wave_length + t_discard_pnts +36)*4;	//针对 这两个频率，需要1/4 抽点，所以采集长度 *4
		}else if (t_max_freq == FREQ_1000 || t_max_freq == FREQ_5000)
		{
			smp_length = (t_wave_length + t_discard_pnts +31)*2;	//针对 这两个频率，需要1/2 抽点，所以采集长度 *2
		}else
		{
			smp_length = (t_wave_length + t_discard_pnts);	
		}
		
		float *time_CH1_smp_buf = NULL; 		       
        time_CH1_smp_buf = (float*)malloc(smp_length*sizeof(float));
        if (time_CH1_smp_buf == NULL)
        {
            LOGD("time_CH1_smp_buf 分配内存失败！");
            return NULL;
        }
        memset(time_CH1_smp_buf, 0, smp_length*sizeof(float));

        if (stop_vibrate_smp_flag)
        {
            LOGD("时域运算线程进入第一时间，检测到停止振动采集标识，不经过算法，调用stop_DMA_FPGA, 释放内存，初始化flag变量，回调停止采集数据给上层");
            goto stop_daq;
        }

		sem_wait(&run_sem);//等待信号量

		if (stop_vibrate_smp_flag)//接收信号量后第一时间判断是否有停止振动采集标识
		{
			LOGD("接收信号量后，时域运算线程第一时间检测到停止振动采集标识, 不经过算法，调用stop_DMA_FPGA, 释放内存，初始化flag变量，回调停止采集数据给上层");
			goto stop_daq;
		}

        memset(time_CH1_smp_buf, 0, smp_length*sizeof(float));
	    memset(t_CH_data, 0, 3*sizeof(float));
        
        //LOGD("时域运算线程 signal_type = %d, max_freq = %d, min_freq = %d, vol_range = %d, auto_voltagerange_flag = %d", my_timewave.signal_type, t_max_freq, t_min_freq, vol_range, auto_voltagerange_flag);
        
		for (i= 0; i< smp_length; i++)	
		{
			analyze_CH_data(smp_buf[i], t_CH_data);
			time_CH1_smp_buf[i] = t_CH_data[2];             
        
            if (vol_range == VOL_RANGE_V25)//表示电压量程是25V
            {
                time_CH1_smp_buf[i] = time_CH1_smp_buf[i] * 10; //乘以10 是因为寄存器默认配置是25V，硬件电路会衰减10倍，再此*10补上, 和24V激励没有关系
            }
            else if (vol_range == VOL_RANGE_V2P5)//表示电压量程是2.5V
            {
               ; //2.5V 硬件电路无变化 ，不增益也不衰减
            }
            else if (vol_range == VOL_RANGE_V0P25)//表示电压量程是0.25V
            {
                time_CH1_smp_buf[i] = time_CH1_smp_buf[i] / 10; //除以10 是因为硬件电路增益放大了10倍，再此除以10 去掉增益
            }
		}
        
    	if (auto_voltagerange_flag) //初次探测量程，不运行此代码
    	{  
        	t_pkvalue = rend_value(time_CH1_smp_buf, t_wave_length, 1);	//求峰值
        	goto  auto_volrange;  
    	}
        
    #if READ_CALIB_PARA
        LOGD("从FRAM 读取校准参数===");
        read_calib_para(my_timewave.signal_type, t_max_freq, t_min_freq, vol_range, t_calib_para); //读取校准参数
        if (fabs(t_calib_para[0])< FABS_0)
        {
            t_calib_para[0] = 1.0;
        }
        
        for (i= 0; i< smp_length; i++)	
        {
            time_CH1_smp_buf[i] = time_CH1_smp_buf[i] * t_calib_para[0] - t_calib_para[1]; //校准数据            
        }
    #endif            
            
		dis_dc_func(time_CH1_smp_buf, smp_length);	 //经过去直流分量算法
		
    #if HAVE_ALGORITHM
		t_temp_len = smp_length; ////FIR 低通滤波，单通道，采样长度
		if (t_max_freq == FREQ_500 || t_max_freq == FREQ_2500)//上限是500 2500, 是1/4抽样
		{
			enter_fir_filter(time_CH1_smp_buf, t_temp_len, t_max_freq); //FIR 低通滤波
			t_temp_len = smp_length/4 -36; //计算后长度变为1/4(因为是1/4抽样，所以总长度变小，减36是因为前面最初采集多采36个点，过FIR将其减去)

			enter_iir_filter(time_CH1_smp_buf, t_temp_len, t_max_freq, t_min_freq); //IIR 高通滤波
		}else if (t_max_freq == FREQ_1000 || t_max_freq == FREQ_5000)	 //上限是1000 5000, 是1/2抽样
		{
			enter_fir_filter(time_CH1_smp_buf, smp_length, t_max_freq);
			t_temp_len = smp_length/2 -31; //计算后长度变为1/2

			enter_iir_filter(time_CH1_smp_buf, t_temp_len, t_max_freq, t_min_freq);
		}else  //其它上限频率只经过IIR
		{
			enter_iir_filter(time_CH1_smp_buf, t_temp_len, t_max_freq, t_min_freq);
		}
        
    #if 0  
        if (my_timewave.signal_type == 2)//位移时再过一次IIR 20HZ高通滤波
        {
            enter_iir_filter(time_CH1_smp_buf, t_temp_len, t_max_freq, 20);
        }
    #endif
        
		memcpy(time_CH1_smp_buf, &time_CH1_smp_buf[t_discard_pnts], t_wave_length * sizeof(float));//经IIR滤波后 去掉丢弃的点
    #endif
    
        auto_volrange:
        {
	        if (stop_vibrate_smp_flag)
			{
	            LOGD("时域运算线程检测到停止振动采集标识为1，调用stop_DMA_FPGA, 释放内存，初始化flag变量，回调停止采集数据给上层");
				goto stop_daq;
			}else{            
	            if (auto_voltagerange_flag)
	            {
	                if (fabs(t_pkvalue) < 0.23)
	                {
	                    vol_range = VOL_RANGE_V0P25;//表示电压量程是0.25V
	                }else if (fabs(t_pkvalue) < 2.3 && fabs(t_pkvalue) > 0.23)
	                {
	                    vol_range = VOL_RANGE_V2P5;//表示电压量程是2.5V
	                }else{
	                    vol_range = VOL_RANGE_V25;//表示电压量程是25V
	                }
	                LOGD("fabs(t_pkvalue) = %f, voltage_ range = %d", fabs(t_pkvalue), vol_range);
	                poweroff_spi();  //下电
	                masterspi_close();
					thread_finished_flag = true;
                    
                    if (stop_vibrate_smp_flag)
                    {
                        flag_init();//停止和异常回调 对所有标识flag清0
                        vibrate_callback_backup.single_ch_callback(t_invalid_buf, 0, false);//用于停止采集时的异常回调
                    }else{                        
                        flag_init();
                        auto_voltagerange_flag = true;//第一次探测后，不清0，让第二次量程校准后正式采集参数生效
                        vibrate_callback_backup.single_ch_callback(probe_vol_range_buf, 3, false);	//探测电压量程的回调
                    }
                    
                    LOGD("时域运算线程电压量程探测 正常回调数据  restart_power_on_flag = %d, auto_voltagerange_flag = %d, stop_vibrate_smp_flag = %d: [%s]\n", restart_power_on_flag, auto_voltagerange_flag, stop_vibrate_smp_flag, log_time());
                }
                else //回调正常数据
                {
				    thread_finished_flag = true;
                    
                    if (stop_vibrate_smp_flag)
                    {
                        flag_init();//停止和异常回调 对所有标识flag清0
                        vibrate_callback_backup.single_ch_callback(t_invalid_buf, 0, false);//用于停止采集时的异常回调
                    }else{
                        flag_init();
                        restart_power_on_flag = true;
                        auto_voltagerange_flag = true;//第一次探测后，不清0，让第二次量程校准后正式采集参数生效
                        vibrate_callback_backup.single_ch_callback(time_CH1_smp_buf, t_wave_length, true);	//回调时域波形
                    }
                    
                    LOGD("时域运算线程正常回调数据  restart_power_on_flag = %d, auto_voltagerange_flag = %d ,stop_vibrate_smp_flag = %d: [%s]\n", restart_power_on_flag, auto_voltagerange_flag,stop_vibrate_smp_flag, log_time());
                }
	            if (time_CH1_smp_buf != NULL)
	            {
	                free(time_CH1_smp_buf);
	                time_CH1_smp_buf = NULL;
	            }
	            return NULL;
			}
        }
        stop_daq:
        {
            usleep(DELAY_100000);
            if (fd != -1)
            {
                stop_fpga_dma();
            }
            if (time_CH1_smp_buf != NULL)
            {
                free(time_CH1_smp_buf);
                time_CH1_smp_buf = NULL;
            }
            thread_finished_flag = true;            
            poweroff_spi();  //下电
            masterspi_close();

            LOGD("时域运算线程异常回调数据 is : [%s]\n", log_time());
            flag_init();//停止和异常回调 对所有标识flag清0
            vibrate_callback_backup.single_ch_callback(t_invalid_buf, 0, false);//用于停止采集时的异常回调
            return NULL;
        }
	}
    return NULL;
}

void *total_rend_thread(void* arg) //总值趋势线程
{
    totalrend  my_totalrend;
    my_totalrend = *(struct total_rend_para*)arg;
    int r_max_freq = (int)my_totalrend.max_freq;
    int r_min_freq = (int)my_totalrend.min_freq;
    int r_discard_pnts = get_invalid_num(r_max_freq, r_min_freq);
    int r_wave_length = my_totalrend.wave_length;
    
    float r_calib_para[2] = {0.0}; //calib_para[0]: 表示增益，calib_para[1]: 表示偏移
    float r_invalid_buf[2] = {0.0}; // power off时回调的假buf, 供接口使用，实际无意义
    float probe_vol_range_buf[3] = {10005.0}; //用于电压量程探测回调的buf
    
	int i= 0;
	int r_temp_len = 0;
	float r_CH1_value[1] = {0.0};
	float r_CH2_value[1] = {0.0};
	float r_CH_data[3]= {0.0};
    float r_pkvalue = 0.0;

	if (my_totalrend.chan_num == CH_B)
	{
        if (r_max_freq == FREQ_500 || r_max_freq == FREQ_2500)
		{
			smp_length = (r_wave_length + r_discard_pnts +36)*4;	//针对 这两个频率，需要1/4 抽点，所以采集升度 *4
		}else if (r_max_freq == FREQ_1000 || r_max_freq == FREQ_5000)
		{
			smp_length = (r_wave_length + r_discard_pnts +31)*2;	//针对 这两个频率，需要1/2 抽点，所以采集升度 *2
		}else
		{
			smp_length = (r_wave_length + r_discard_pnts);	//实际采集的波形长度
		}

		//分配需要的内存
		float *rend_CH1_smp_buf = NULL; //通道1 采集数据
		rend_CH1_smp_buf = (float*)malloc(smp_length*sizeof(float));
		if (rend_CH1_smp_buf == NULL)
		{
			LOGD("rend_CH1_smp_buf 分配内存失败！");
			return NULL;
		}
		memset(rend_CH1_smp_buf, 0, smp_length*sizeof(float));

        if (stop_vibrate_smp_flag)
        {
            LOGD("总值趋势运算线程进入第一时间，检测到停止振动采集标识，不经过算法，调用stop_DMA_FPGA, 释放内存，初始化flag变量，回调停止采集数据给上层");
    		goto stop_daq;
        }

		sem_wait(&run_sem);//等待信号量

        if (stop_vibrate_smp_flag)
		{
			LOGD("接收信号量后，总值趋势运算线程第一时间检测到停止振动采集标识, 不经过算法，调用stop_DMA_FPGA, 释放内存，初始化flag变量，回调停止采集数据给上层");
			goto stop_daq;
		}

		memset(rend_CH1_smp_buf, 0, smp_length*sizeof(float));
        memset(r_CH_data, 0, 3*sizeof(float));
        r_CH1_value[0] = 0.0;
		r_CH2_value[0] = 0.0;

       // LOGD("总值趋势线程 signal_type = %d, max_freq = %d, min_freq = %d, vol_range = %d, auto_voltagerange_flag = %d", my_totalrend.signal_type, r_max_freq, r_min_freq, vol_range, auto_voltagerange_flag);
        
	    for (i= 0;i< smp_length;i++)	//转换采集数据，从32位转为24位有效数据
		{
			analyze_CH_data(smp_buf[i], r_CH_data);
			rend_CH1_smp_buf[i] = r_CH_data[2];            
            
            if (vol_range == VOL_RANGE_V25)//表示电压量程是25V
            {
                rend_CH1_smp_buf[i] = rend_CH1_smp_buf[i] * 10; //乘以10 是因为寄存器默认配置是25V，硬件电路会衰减10倍，再此*10补上, 和24V激励没有关系
            }
            else if (vol_range == VOL_RANGE_V2P5)//表示电压量程是2.5V
            {
               ; //2.5V 硬件电路无变化 ，不增益也不衰减
            }
            else if (vol_range == VOL_RANGE_V0P25)//表示电压量程是0.25V
            {
                rend_CH1_smp_buf[i] = rend_CH1_smp_buf[i] / 10; //除以10 是因为硬件电路增益放大了10倍，再此除以10 去掉增益
            }
		}
        
        if (auto_voltagerange_flag) //初次探测量程，不运行此代码
        {  
            r_pkvalue = rend_value(rend_CH1_smp_buf, r_wave_length, 1);	//求峰值 
            goto  auto_volrange;  
        }
    #if READ_CALIB_PARA
        LOGD("从FRAM 读取校准参数===");
        read_calib_para(my_totalrend.signal_type, r_max_freq, r_min_freq, vol_range, r_calib_para);  //读取校准参数 
        if (fabs(r_calib_para[0])< FABS_0)
        {
            r_calib_para[0] = 1.0;
        }
        
        for (i= 0; i< smp_length; i++)	
        {
            rend_CH1_smp_buf[i] = rend_CH1_smp_buf[i] * r_calib_para[0] - r_calib_para[1];
        }
    #endif
        
        dis_dc_func(rend_CH1_smp_buf, smp_length); //去除直流分量算法
		
    #if HAVE_ALGORITHM		
		r_temp_len = smp_length;		////FIR 低通滤波，单通道，采样长度
		if (r_max_freq == FREQ_500 || r_max_freq == FREQ_2500)//上限500 2500, 是1/4抽样
		{
			enter_fir_filter(rend_CH1_smp_buf, r_temp_len, r_max_freq); //FIR 低通滤波
			r_temp_len = smp_length/4 -36; //计算后长度变为1/4

			enter_iir_filter(rend_CH1_smp_buf, r_temp_len, r_max_freq, r_min_freq); //IIR 高通滤波
		}else if (r_max_freq == FREQ_1000 || r_max_freq == FREQ_5000)	 //上限1000 5000, 是1/2抽样
		{
			enter_fir_filter(rend_CH1_smp_buf, r_temp_len, r_max_freq); //FIR 低通滤波
			r_temp_len = smp_length/2 -31; //计算后长度变为1/2

			enter_iir_filter(rend_CH1_smp_buf, r_temp_len, r_max_freq, r_min_freq); //IIR 高通滤波
		}else  //其它上限频率只经过IIR
		{
			enter_iir_filter(rend_CH1_smp_buf, r_temp_len, r_max_freq, r_min_freq); //IIR 高通滤波，长度是采样的点
		} 
        
    #if 0  
        if (my_totalrend.signal_type == 2)//位移时再过一次IIR 20HZ高通滤波
        {
            enter_iir_filter(rend_CH1_smp_buf, r_temp_len, r_max_freq, 20);
        }
    #endif
        
		memcpy(rend_CH1_smp_buf, &rend_CH1_smp_buf[r_discard_pnts], r_wave_length*sizeof(float));//将IIR滤波后 去掉丢弃的点
    #endif

		r_CH1_value[0] = rend_value(rend_CH1_smp_buf, r_wave_length, my_totalrend.total_value_type); //总值趋势算法
		LOGD("计算出的总值为 = %f", r_CH1_value[0]);

    	auto_volrange:
        {
		    if (stop_vibrate_smp_flag)
			{
	            LOGD("总值数据线程检测到停止振动采集标识为1，调用stop_DMA_FPGA, 释放内存，初始化flag变量，回调停止采集数据给上层");
	            goto stop_daq;
			}else{		    
			    if (auto_voltagerange_flag)
	            {
                
	                if (fabs(r_pkvalue) < 0.23)
	                {
	                    vol_range = VOL_RANGE_V0P25;//表示电压量程是0.25V
	                }else if (fabs(r_pkvalue) < 2.3 && fabs(r_pkvalue) > 0.23)
	                {
	                    vol_range = VOL_RANGE_V2P5;//表示电压量程是2.5V
	                }else{
	                    vol_range = VOL_RANGE_V25;//表示电压量程是25V
	                }
	                LOGD("fabs(r_pkvalue) = %f, voltage_ range = %d", fabs(r_pkvalue), vol_range);
                
	                if (rend_CH1_smp_buf != NULL)
	                {
	                    free(rend_CH1_smp_buf);
	                    rend_CH1_smp_buf = NULL;
	                }
					poweroff_spi();  //下电
	                masterspi_close();
					thread_finished_flag = true;
				
                    
                    if (stop_vibrate_smp_flag)
                    {
                        flag_init();//停止和异常回调 对所有标识flag清0
                        vibrate_callback_backup.single_ch_callback(r_invalid_buf, 0, false);//用于停止采集时的异常回调
                    }else{
                        flag_init();
                        auto_voltagerange_flag = true;//第一次探测后，不清0，让第二次量程校准后正式采集参数生效
                        vibrate_callback_backup.single_ch_callback(probe_vol_range_buf, 3, false);
                    }                    
                    
                    LOGD("总值运算线程电压量程探测 正常回调数据  restart_power_on_flag = %d, auto_voltagerange_flag = %d ,stop_vibrate_smp_flag = %d: [%s]\n", restart_power_on_flag, auto_voltagerange_flag,stop_vibrate_smp_flag, log_time());
                }
                else //回调正常数据
                {
	                if (rend_CH1_smp_buf != NULL)
	                {
                        free(rend_CH1_smp_buf);
                        rend_CH1_smp_buf = NULL;
                    }
                    thread_finished_flag = true;
                   
                    if (stop_vibrate_smp_flag)
                    {
                        flag_init();//停止和异常回调 对所有标识flag清0
                        vibrate_callback_backup.single_ch_callback(r_invalid_buf, 0, false);//用于停止采集时的异常回调
                    }else{
                        flag_init();
                        restart_power_on_flag = true;
                        auto_voltagerange_flag = true;//第一次探测后，不清0，让第二次量程校准后正式采集参数生效
                        vibrate_callback_backup.single_ch_callback(r_CH1_value, sizeof(r_CH1_value)/sizeof(float), true); //回调总值单个值
                    }
                    
                    LOGD("总值数据线程正常回调数据 restart_power_on_flag = %d, auto_voltagerange_flag = %d, stop_vibrate_smp_flag = %d: [%s]\n", restart_power_on_flag, auto_voltagerange_flag,stop_vibrate_smp_flag ,log_time());
                }           
                return NULL;
            }
        }
        
        stop_daq:
        {
            usleep(DELAY_100000);
            if (fd != -1)
            {
                stop_fpga_dma();
            }
            if (rend_CH1_smp_buf != NULL)
            {
                free(rend_CH1_smp_buf);
                rend_CH1_smp_buf = NULL;
            }
            thread_finished_flag = true;
            poweroff_spi();  //下电
            masterspi_close();

            LOGD("总值数据线程异常回调数据 is : [%s]\n", log_time());
            flag_init();//停止和异常回调 对所有标识flag清0
            vibrate_callback_backup.single_ch_callback(r_invalid_buf, 0, false); //用于停止采集时的异常回调
            return NULL;
        }
    }
    return NULL;
}

void *evalute_level_thread(void* arg) //振动等级评估线程
{
    timewave  my_timewave;
    my_timewave = *(struct time_wave_para*)arg;
    int e_max_freq = (int)my_timewave.max_freq;
    int e_min_freq = (int)my_timewave.min_freq;
    int e_discard_pnts = get_invalid_num(e_max_freq, e_min_freq);
    int e_wave_length = my_timewave.wave_length;
    
    float e_calib_para[2] = {0.0}; //calib_para[0]: 表示增益，calib_para[1]: 表示偏移
    int i = 0;
    int e_temp_len = 0;
	float evalute_value[1] = {0.0};	 //0位:表示速度
	float e_CH_data[3]= {0.0};

    if (my_timewave.chan_num == CH_B)
    {
		if (e_max_freq == FREQ_1000)
		{
			smp_length = (e_wave_length + e_discard_pnts +31)*2;	//针对 这1000频率，需要1/2 抽点，所以采集升度 *2
		}

	    //分配需要的内存
	    float *evalute_CH1_smp_buf = NULL; //振动评估 通道1 采集数据
		evalute_CH1_smp_buf = (float*)malloc(smp_length*sizeof(float));
		if (evalute_CH1_smp_buf == NULL)
		{
			LOGD("evalute_CH1_smp_buf 分配内存失败！");
			return NULL;
		}
		memset(evalute_CH1_smp_buf, 0, smp_length*sizeof(float));

		sem_wait(&run_sem);//等待信号量

	    memset(evalute_CH1_smp_buf, 0, smp_length*sizeof(float));
	    memset(e_CH_data, 0, 3*sizeof(float));
        
        //LOGD("等级评估线程 signal_type = %d, max_freq = %d, min_freq = %d", my_timewave.signal_type, e_max_freq, e_min_freq);
        
		for (i= 0;i< smp_length;i++)	//转换采集数据，从32位转为24位有效数据
		{
			analyze_CH_data(smp_buf[i], e_CH_data);
			evalute_CH1_smp_buf[i] = e_CH_data[2]; //单通道振动采集默认CHB 数据                
            
            if (vol_range == VOL_RANGE_V25)//表示电压量程是25V
            {
                evalute_CH1_smp_buf[i] = evalute_CH1_smp_buf[i] * 10; //乘以10 是因为寄存器默认配置是25V，硬件电路会衰减10倍，再此*10补上, 和24V激励没有关系
            }
            else if (vol_range == VOL_RANGE_V2P5)//表示电压量程是2.5V
            {
               ; //2.5V 硬件电路无变化 ，不增益也不衰减
            }
            else if (vol_range == VOL_RANGE_V0P25)//表示电压量程是0.25V
            {
                evalute_CH1_smp_buf[i] = evalute_CH1_smp_buf[i] / 10; //除以10 是因为硬件电路增益放大了10倍，再此除以10 去掉增益
            }
		}
        
    #if READ_CALIB_PARA                    
        LOGD("从FRAM 读取校准参数===");
        read_calib_para(my_timewave.signal_type, e_max_freq, e_min_freq, vol_range, e_calib_para); //读取校准参数 
        if (fabs(e_calib_para[0])< 0.00000000001)
        {
            e_calib_para[0] = 1.0;
        }   
        for (i= 0;i< smp_length;i++)
		{
            evalute_CH1_smp_buf[i] = evalute_CH1_smp_buf[i] * e_calib_para[0] - e_calib_para[1];     
        }
    #endif
        
		dis_dc_func(evalute_CH1_smp_buf, smp_length);
        
    #if HAVE_ALGORITHM
		e_temp_len = smp_length; ////FIR 低通滤波，单通道，采样长度
		if (e_max_freq == FREQ_1000)	 //上限是1000 是1/2抽样
		{
			enter_fir_filter(evalute_CH1_smp_buf, smp_length, e_max_freq); //FIR 低通滤波
			e_temp_len = smp_length/2 -31; //计算后长度变为1/2

			enter_iir_filter(evalute_CH1_smp_buf, e_temp_len, e_max_freq, e_min_freq); //IIR 高通滤波
		}
		memcpy(evalute_CH1_smp_buf, &evalute_CH1_smp_buf[e_discard_pnts], e_wave_length*sizeof(float));
    #endif
        
		evalute_value[0] = rend_value(evalute_CH1_smp_buf, e_wave_length, 0);	//速度有效值 0表示有效值
		LOGD("计算出速度有效值 = %f", evalute_value[0]);

	    if (evalute_CH1_smp_buf != NULL)
		{
			free(evalute_CH1_smp_buf);
			evalute_CH1_smp_buf = NULL;
		}

		////////自动stop
		stop_fpga_dma();
		poweroff_spi();  //下电
	    masterspi_close();
	    thread_finished_flag = true;

	    flag_init();//停止和异常回调 对所有标识flag清0
		vibrate_callback_backup.single_ch_callback(evalute_value, sizeof(evalute_value)/sizeof(float), true); //回调等级评估值
		LOGD("等级评估回调结束 is : [%s]\n", log_time());
    }
	return NULL;
}

static int start_vibrate_CH_timewave(struct spictl_device_t* dev, struct time_wave_para tWave)//时域波形
{
    int tmax_freq = 0, tmin_freq = 0, twave_length = 0, t_sleep_time = 0;
    float vib_reg_fail_buf[1] = {10001}; // 振动采集时当寄存器配置失败时，供上面app用
	if (start_enable_flag)
	{
		LOGD("start_enable_ flag = 1, 前一个start还未执行完，此时不响应新的start接口, 直接return出去");
        return 0;
	}else{
	    start_enable_flag = true;//start线程一进入就置为1，start全部结束时置为0
	}

    if (stop_vibrate_smp_flag)
    {
        LOGD("响应时域点击开始stop_vibrate_smp_ flag = %d, [%s]\n", stop_vibrate_smp_flag, log_time());
        stop_vibrate_smp_flag = false;
        return 0;
    }

    memset(read_60K_buf, 0, SIZE_60K*sizeof(unsigned char));
    memset(max_char_buf, 0, MAX_SIZE*sizeof(unsigned char));
    
    v_loop_num = 0;    

    LOGD("点击时域开始时 stop_vibrate_smp_ flag = %d, restart_power_on_flag= %d, thread_finished_flag = %d, auto_voltagerange_ flag = %d, [%s]\n", stop_vibrate_smp_flag, restart_power_on_flag, thread_finished_flag, auto_voltagerange_flag, log_time());
    if (!auto_voltagerange_flag)
    {
        auto_voltagerange_flag = true;//响应start时第一时间标识置1，表示自动量程
        tWave.max_freq = FREQ_10000;//10kHZ
        tWave.min_freq = FREQ_5;//5HZ
        tWave.wave_length = LENGTH_16K;//16K
		vol_range = VOL_RANGE_V25;//以最大25V量程探测
        t_sleep_time = DELAY_1000000;
    }else{
        auto_voltagerange_flag = false; //清0，不再探测自动量程，使用正常配置参数采集
        t_sleep_time = DELAY_3000000;
		if ((tWave.signal_type == ACC_TYPE && (int)tWave.min_freq < FREQ_7) || ((tWave.signal_type == VEL_TYPE || tWave.signal_type == DSP_TYPE) && (int)tWave.min_freq < FREQ_10)) //DC模式时，重新将电压量程设为25V
        {
            LOGD("DC模式强制切换到 25V量程===");
            vol_range = VOL_RANGE_V25;
        }
	}
    LOGD("响应时域开始signal_type = %d, min_freq = %d, max_freq = %d, wave_length = %d, tWave.version_mode = %d, voltage_ range = %d, tWave.chan_num = %d", tWave.signal_type, (int)tWave.min_freq, (int)tWave.max_freq, tWave.wave_length, tWave.version_mode, vol_range, tWave.chan_num);

	if (!is_valid_length(tWave.wave_length)) //判断下发的采样长度是否支持
	{
        LOGD("振动采集时域波形下发的长度不支持 tWave.wave_length = %d", tWave.wave_length);
		return 0;
	}

    if (restart_power_on_flag) // 当内部stop DMA，fpga, 关闭设备后，flag 置1， 当再启动start接口时不再重新上电，当对外大的停止采集下电后，此flag会置为 0，重新采集时再上电
	{
		restart_power_on_flag = false; 
	}else{
		poweron_spi();
        masterspi_open();
      
		int reg_ret_value = set_singleCH_vibrate_reg(tWave.chan_num, tWave.signal_type, (int)tWave.max_freq, (int)tWave.min_freq, tWave.version_mode, vol_range);
        if (reg_ret_value == -1)
		{
            LOGD("寄存器配置失败, 回调寄存器失败数据10001, flag全部初始化为0");
			flag_init(); //停止和异常回调 对所有标识flag清0

            poweroff_spi();
            masterspi_close();
			vibrate_callback_backup.single_ch_callback(vib_reg_fail_buf, 1, true);//回调寄存器失败数据10001
			return 0;
		}
		usleep(t_sleep_time); //用于上层时域波形不丢波形数据，在这里直接延时3S, 让硬 件预热稳定
	}

	sem_init(&run_sem, 0, 0);
	pthread_create(&c_id, NULL, time_wave_thread, (void*)&tWave);

	signal(SIGIO, read_vibrate_data);
    common_start();

	while (!thread_finished_flag); // 用于等待算法线程线束时置1，若线程结束时，继续往下运行

	usleep(DELAY_50000);//等待回调结束延时
    LOGD("响应时域点击结束 [%s]\n", log_time());
	return 0;
}

static int start_vibrate_CH_totalrend(struct spictl_device_t* dev, struct total_rend_para tRend)//总值趋势
{
    int rmax_freq = 0, rmin_freq = 0, rwave_length = 0, r_sleep_time = 0;
    float vib_reg_fail_buf[1] = {10001}; // 振动采集时当寄存器配置失败时，供上面app用
	if (start_enable_flag)
	{
		LOGD("start_enable_ flag = 1, 前一个start还未执行完，此时不响应新的start接口, 直接return出去");
        return 0;
	}else{
	    start_enable_flag = true;
	}

    if (stop_vibrate_smp_flag)
    {
        LOGD("响应总值点击开始stop_vibrate_smp_ flag = %d, [%s]\n", stop_vibrate_smp_flag, log_time());
        stop_vibrate_smp_flag = false;
        return 0;
    }

	memset(read_60K_buf, 0, SIZE_60K*sizeof(unsigned char));
	memset(max_char_buf, 0, MAX_SIZE*sizeof(unsigned char));
    
    v_loop_num = 0;   
    
    //usleep(tRend.interval_time*1000*1000); //下次采集间隔时间  微秒为单位

	LOGD("点击总值开始时 restart_power_on_flag= %d, thread_finished_flag = %d,  auto_voltagerange_ flag = %d", restart_power_on_flag, thread_finished_flag, auto_voltagerange_flag);
     if (!auto_voltagerange_flag)
    {
        auto_voltagerange_flag = true;//响应start时第一时间标识置1，表示自动量程
        tRend.max_freq = FREQ_10000;//10kHZ
        tRend.min_freq = FREQ_5;//5HZ
        tRend.wave_length = LENGTH_16K;//16K
        vol_range = VOL_RANGE_V25;//以最大25V量程探测
        r_sleep_time = DELAY_1000000;
    }else{
        auto_voltagerange_flag = false; //清0，不再探测自动量程，使用正常配置参数采集
        r_sleep_time = DELAY_3000000;
		if ((tRend.signal_type == ACC_TYPE && (int)tRend.min_freq < FREQ_7) || ((tRend.signal_type == VEL_TYPE || tRend.signal_type == DSP_TYPE) && (int)tRend.min_freq < FREQ_10)) //DC模式时，重新将电压量程设为25V
        {
            LOGD("DC模式强制切换到 25V量程===");
            vol_range = VOL_RANGE_V25;
        }
    }

    LOGD("响应总值开始signal_type = %d, min_freq = %d, max_freq = %d, wave_length = %d, tRend.version_mode = %d, voltage_ range = %d, tRend.chan_num = %d", tRend.signal_type, (int)tRend.min_freq, (int)tRend.max_freq, tRend.wave_length, tRend.version_mode, vol_range, tRend.chan_num);
	if (!is_valid_length(tRend.wave_length))
	{
        LOGD("振动采集总值趋势下发的长度不支持 tRend.wave_length = %d", tRend.wave_length);
        return 0;
	}

	if (restart_power_on_flag) // 当内部stop DMA，fpga, 关闭设备后，flag 置1， 当再启动start接口时不再重新上电，当对外大的停止采集下电后，此flag会置为 0，重新采集时再上电
	{
		restart_power_on_flag = false;        
        set_singleCH_vibrate_reg(tRend.chan_num, tRend.signal_type, (int)tRend.max_freq, (int)tRend.min_freq, tRend.version_mode, vol_range);
	}else{
		poweron_spi();
        masterspi_open();

        int reg_ret_value = set_singleCH_vibrate_reg(tRend.chan_num, tRend.signal_type, (int)tRend.max_freq, (int)tRend.min_freq, tRend.version_mode, vol_range);//设置单通道采集寄存器
		if (reg_ret_value == -1)
		{
            LOGD("寄存器配置失败, 回调寄存器失败数据10001, flag全部初始化为0");
			flag_init(); //停止和异常回调 对所有标识flag清0

            poweroff_spi();
            masterspi_close();
			vibrate_callback_backup.single_ch_callback(vib_reg_fail_buf, 1, true);//回调寄存器失败数据10001
			return 0;
		}
		usleep(r_sleep_time); //用于上层时域波形不丢波形数据，在这里直接延时3S, 让硬 件预热稳定
	}

	sem_init(&run_sem, 0, 0);
	pthread_create(&c_id, NULL, total_rend_thread, (void*)&tRend);

	signal(SIGIO, read_vibrate_data);
	common_start();

	while (!thread_finished_flag); // 用于等待算法线程线束时置1，若线程结束时，继续往下运行

	usleep(DELAY_50000);
	LOGD("响应总值点击结束 [%s]\n", log_time());
	return 0;
}

/* 振动等级评估 下限10hz  上限1000Hz  波长4k */
static int start_vibrate_evalute_level(struct spictl_device_t* dev, struct time_wave_para tWave)//振动等级评估
{
    LOGD("点击振动等级开始时 stop_vibrate_smp_ flag = %d, restart_power_on_flag= %d, thread_finished_flag = %d", stop_vibrate_smp_flag, restart_power_on_flag, thread_finished_flag);
    float vib_reg_fail_buf[1] = {10001}; // 振动采集时当寄存器配置失败时，供上面app用
    memset(read_60K_buf, 0, SIZE_60K*sizeof(unsigned char));
    memset(max_char_buf, 0, MAX_SIZE*sizeof(unsigned char));

	LOGD("响应振动等级评估开始signal_type = %d, min_freq = %d, max_freq = %d, wave_length = %d, tWave.version_mode = %d, tWave.chan_num = %d", tWave.signal_type, (int)tWave.min_freq, (int)tWave.max_freq, tWave.wave_length, tWave.version_mode, tWave.chan_num);

	if (!is_valid_length(tWave.wave_length))
	{
		LOGD("振动采集等级评估下发的长度不支持 tWave.wave_length = %d", tWave.wave_length);
		return 0;
	}
	poweron_spi();
    masterspi_open();
    vol_range = VOL_RANGE_V25;//等级评估默认电压量程是25V

	int reg_ret_value = set_singleCH_vibrate_reg(tWave.chan_num, tWave.signal_type, (int)tWave.max_freq, (int)tWave.min_freq, tWave.version_mode, vol_range);//设置单通道采集寄存器，数据类型为 速度时域波形, 等级评估默认电压量程是25V
	if (reg_ret_value == -1)
	{
		LOGD("寄存器配置失败, 回调寄存器失败数据10001, flag全部初始化为0");
		flag_init(); //停止和异常回调 对所有标识flag清0

		poweroff_spi();
        masterspi_close();
		vibrate_callback_backup.single_ch_callback(vib_reg_fail_buf, 1, true);//回调寄存器失败数据10001
		return 0;
	}
	usleep(DELAY_3000000); //用于上层时域波形不丢波形数据，在这里直接延时3S, 让硬 件预热稳定

	sem_init(&run_sem, 0, 0);
	pthread_create(&c_id, NULL, evalute_level_thread, (void*)&tWave);

	signal(SIGIO, read_evalute_data);
    common_start();

 	while (!thread_finished_flag); // 用于等待算法线程线束时置1，若线程结束时，继续往下运行

    usleep(DELAY_50000);
	LOGD("响应振动等级评估结束 [%s]\n", log_time());
	return 0;
}

/* 获取15个特征值 传下来是原始电压值 */
static float* spi_get_feature_value(struct spictl_device_t* dev, float pdata[], int data_len)
{
	memset(feature_ret_value, 0, FEATURE_NUM*sizeof(float)); //每次进来先初始化0
	feature_value(pdata, data_len, feature_ret_value); // 求15个特征值
	return feature_ret_value;
}

/* 时域到频域接口 */
static float* spi_time_to_freq_value(struct spictl_device_t* dev, float pdata[], int data_len)
{
	fft_alg_entry2(pdata, data_len, 0, 0, 0);	//0默认不加窗，不平均，平均方式不进行
	return pdata;
}
