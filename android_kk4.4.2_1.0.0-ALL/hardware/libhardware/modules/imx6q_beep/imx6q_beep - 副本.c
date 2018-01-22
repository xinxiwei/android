
#define LOG_TAG "beep"
#include <hardware/hardware.h>
#include <hardware/imx6q_beep.h>
#include <errno.h>
#include <cutils/log.h>
#include <cutils/atomic.h>
#include <sys/ioctl.h>
#include <hardware/log.h>
#include <hardware/gpio.h>
#include <hardware/errorcode.h>
#include <hardware/beep.h>
#include <fcntl.h>
#include <stdbool.h>
#include <sys/timeb.h>
#include <linux/input.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/wait.h>
#include <sys/param.h>
#include <sys/stat.h>
#include <time.h>
#include <sys/time.h>//itimerval结构体的定义

#define DEVICE_NAME "/dev/input/event0"
#define MODULE_NAME "xxxxx"
#define MODULE_AUTHOR "xxx@126.com"

int beep_keys_fd =-1;
int sensor_keys_fd =-1;
int sensor_backlight_fd = -1;
int beep_fd = -1;

volatile int soundOnOff_flag =0;
volatile int backlightOnOff_flag =0;

int KEY_ESC_CODE = 59;
int KEY_CANCEL_CODE = 223;
static int KEY_LIGHT =9;

pthread_t th_beep;
pthread_t th_sensor;

/*设备打开和关闭接口*/
static int keypad_device_open(const struct hw_module_t* module,const char* name,struct hw_device_t** device);
/*设备访问接口*/
static int beep_set_value(struct keyctl_device_t* device, int key ,int status);
static int sensor_set_value(struct keyctl_device_t* device, int key ,int status);
static int keypad_close(struct keyctl_device_t* device);

/*模块方法表*/
static struct hw_module_methods_t key_module_methods={
   open: keypad_device_open
};
/*模块实例变量*/
struct keyctl_module_t HAL_MODULE_INFO_SYM ={
    common:{
        tag: HARDWARE_MODULE_TAG,
        version_major: 1,
        version_minor: 0,
        id: KEYCTL_HARDWARE_MODULE_ID,
        name: MODULE_NAME,
        author: MODULE_AUTHOR,
        methods: &key_module_methods,
    }
};

static int keypad_device_open(const struct hw_module_t* module,const char* name,struct hw_device_t** device)
{
    struct keyctl_device_t* dev;
    dev = (struct keyctl_device_t*)malloc(sizeof(struct keyctl_device_t));
    if(!dev){
        ALOGD("keyctr Stub: failed to alloc space");
        return -EFAULT;
    }
    memset(dev,0,sizeof(struct keyctl_device_t));
    dev->common.tag = HARDWARE_DEVICE_TAG;
    dev->common.version = 0;
    dev->common.module = (hw_module_t*)module;
    dev->close_key = keypad_close;
	dev->set_beep_key = beep_set_value;
	dev->set_sensor_key = sensor_set_value;
    *device =&(dev->common);
    return 0;
}


static int keypad_close(struct keyctl_device_t* device){ //关闭设备
    struct keyctl_device_t* devkey_device = (struct keyctl_device_t*)device;
    if(devkey_device){
        close(devkey_device->fd);
        free(devkey_device);
    }
    return 0;
}

void beep_th(void *arg) //声音线程
{
    ALOGD("HAL层-enter_KeyOnOff, beep_keys_fd = %d",beep_keys_fd);
    struct input_event t;
    while (soundOnOff_flag)
    {
        read(beep_keys_fd, &t, sizeof (t));//阻塞读。如果读完了所有消息，线程会阻塞在read函数上。一旦有新的event事件，read函数返回。
        if(soundOnOff_flag)
        {
            if(t.type == EV_KEY && t.value == 1)// 按下
            {
                ALOGD ("HAL层-按键声音 beep_keys_fd = %d, key= %d %s\n", beep_keys_fd, t.code,(t.value) ? "Pressed" : "Released");
                BeepOn(4000);//蜂鸣器发声。
                usleep(15000);
                BeepOn(0);//如果有一个键被按下，停止蜂鸣器
            }
            else if(t.type == EV_KEY && t.value == 0)
            {

            }
        }
    }
}


static int beep_set_value(struct keyctl_device_t* device,int key,int status)
{
    if(status)
    {
        if( beep_keys_fd == 0 || beep_keys_fd == -1)
        {
            beep_keys_fd = open (DEVICE_NAME, O_RDONLY);
            if (beep_keys_fd == -1 || beep_keys_fd == 0 )
            {
                beep_keys_fd = open (DEVICE_NAME, O_RDONLY);
                if(beep_keys_fd == -1)
                {
                    ALOGD("打开按键设备文件/dev/input/event0失败");
                    return -1;
                }
            }
        }
        ALOGD("打开按键设备文件/dev/input/event0成功, beep_keys_fd = %d",beep_keys_fd);
        ALOGD("创建beep线程");
        int res=pthread_create(&th_beep, NULL, (void *)beep_th, (void *)0);//创建声音线程
        if(res != 0)
        {
            ALOGD("Fail to create a new thread th_beep");
            return -1;
        }
    }
    if(key == 1) //声音
    {
        if(status == 1)
        {
            ALOGD("HAL层-打开beep声音");
            soundOnOff_flag = 1;
            beep_fd = BeepOpen();
        }else{
            ALOGD("HAL层-关闭beep声音");
            BeepOn(0);
            soundOnOff_flag = 0;
            close(beep_keys_fd);
            beep_keys_fd = -1;
        }
    }
	return 0;
}

void init_timer(void)
{
    struct itimerval timer;
    memset(&timer, 0, sizeof(timer));

    timer.it_interval.tv_sec = 10;//自动装载，之后每10秒响应一次  ，it_interval为计时间隔
    timer.it_interval.tv_usec = 0;//微秒

    timer.it_value.tv_sec = 1;//秒，第一次定时的时间， it_value为延时时长，表示延时多久触发一次SIGALRM信号
    timer.it_value.tv_usec = 0;//微秒

    if( setitimer(ITIMER_REAL, &timer, NULL) < 0)  //////成功返回0；失败返回-1，  ITIMER_REAL：以系统真实的时间来计算，它送出SIGALRM信号。
    {
        printf("Set timer failed!!/n");
    }
    //以上表示延时5秒后触发SIGALARM信号，以后每隔10秒触发SIGALARM信号
}

void signalHandler(int signo)
{
    struct input_event t;
    switch (signo){
        case SIGALRM:
            if ((sensor_backlight_fd = open("/dev/devbacklightctr", O_RDWR)) < 0)
            {
                ALOGD("打开智能感光设备文件dev/devbacklightctr 失败.");
                return ;
            }else{
                ALOGD( "打开智能感光设备文件dev/devbacklightctr 成功. fd = %d",sensor_backlight_fd);
            }

            sensor_keys_fd = open (DEVICE_NAME, O_RDONLY);
            if (sensor_keys_fd <= 0)
            {
                ALOGD ("打开sensor按键设备文件/dev/input/event0失败!");
                return ;
            }
            else{
                ALOGD("打开sensor按键设备文件/dev/input/event0成功 fd=%d",sensor_keys_fd);
            }

            read(sensor_keys_fd, &t, sizeof (t));//阻塞读。如果读完了所有消息，线程会阻塞在read函数上。一旦有新的event事件，read函数返回。
            if(t.type == EV_KEY && t.value == 1)// 有按键，按下
            {
                ALOGD ("HAL层-智能感光 sensor_keys_fd = %d, key= %d %s\n",sensor_keys_fd, t.code,(t.value) ? "Pressed" : "Released");
                ioctl(sensor_backlight_fd,KEY_LIGHT,1);
            }else if(t.type == EV_KEY && t.value == 0)
            {
                usleep(10000000); //只响应10s
                ioctl(sensor_backlight_fd,KEY_LIGHT,0);
                close(sensor_keys_fd);
                close(sensor_backlight_fd);
            }
            break;
        default:
            break;
   }
}

void sensor_th(void *arg)//感光线程
{
    ALOGD("HAL层-enter_sensor_th");
    while (backlightOnOff_flag)
    {
        signal(SIGALRM, signalHandler);//注册一个处理函数signalHandler 给SIGALRM 信号
        init_timer();  //初始化一个信号
        while(1) {	//一个补获信号
	        pause(); //pause()，命令系统进入sleep状态，等待任何signal，一定要用while(1)无穷循环执行pause()，如此才能一直接收 SIGALRM signal以间隔执行function，若拿掉while(1)，则function只会执行一次而已
        }
	}
}

static int sensor_set_value(struct keyctl_device_t* device,int key,int status)
{
    if(status)
    {
        ALOGD(" 创建智能感光线程 th_sensor ");
        int res=pthread_create(&th_sensor, NULL, (void *)sensor_th, (void *)0);
        if(res != 0)
        {
            ALOGD("Fail to create a new thread th_sensor");
            return -1;
        }
    }
    if(key == 2)//背光灯
    {
        if(status == 1)
        {
            ALOGD("HAL层-打开智能感光");
            backlightOnOff_flag = 2;
        }else{
            ALOGD("HAL层-关闭智能感光");
            backlightOnOff_flag = 0;
        }
    }
	return 0;
}
