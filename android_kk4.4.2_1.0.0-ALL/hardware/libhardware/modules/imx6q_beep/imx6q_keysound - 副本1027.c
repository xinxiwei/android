#define LOG_TAG "keysound"
#include <hardware/hardware.h>
#include <hardware/imx6q_keysound.h>
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

#define DEVICE_NAME "/dev/input/event0"
#define MODULE_NAME "xxxxx"
#define MODULE_AUTHOR "xxx@126.com"

int keys_fd =-1;
int backlight_fd = -1;

volatile int soundOnOff_flag =0;
volatile int backlightOnOff_flag =0;
volatile int count = 0;

int KEY_ESC_CODE = 59;
int KEY_CANCEL_CODE = 223;
static int KEY_LIGHT =9;
pthread_t th_sound , th_backlight;

/*设备打开和关闭接口*/
static int keypad_device_open(const struct hw_module_t* module,const char* name,struct hw_device_t** device);
static int keypad_device_close(struct hw_device_t* device);
/*设备访问接口*/
static int keypad_device_enable(struct keyctl_device_t* device,int key);
static int keypad_device_disable(struct keyctl_device_t* device,int key);

static int keypad_set_val(struct keyctl_device_t* dev,int val);
static int keypad_get_val(struct keyctl_device_t* dev,int* val);
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

int backlight_open()
{
	 if ((backlight_fd = open("/dev/devbacklightctr", O_RDWR)) < 0)
	 {
		 ALOGD("backlight open fail.");
		 return -1;
	 }
	 ALOGD( "backlight open /dev/devbacklightctr success. fd = %d",backlight_fd);
	 return 0;
}

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
    dev->common.close = keypad_device_close;
	dev->key_enable = keypad_device_enable;
	dev->key_disable = keypad_device_disable;
	
    dev->set_key_val = keypad_set_val;
    dev->get_key_val = keypad_get_val; 
    *device =&(dev->common);
    
    dev->fd = open (DEVICE_NAME, O_RDONLY);
    keys_fd = dev->fd;    
    if (keys_fd <= 0)
    {
        ALOGD ("open /dev/input/event0 device error!");
        if(close(keys_fd) <0)
        {
            ALOGD ("close /dev/input/event0 error!");
        }
        return -1;
    }
    #if 0
    else 
        {
            if(BeepOpen()<0)
            {
                ALOGD("BeepOpen() error.");
            } 
            if(backlight_open()<0)
            {
                ALOGD("backlight_open() error.");
            }            
        }
    #endif
    
    ALOGD("open /dev/input/event0 success. fd=%d",keys_fd);  
    return 0;
}

static int keypad_device_close(struct hw_device_t* device){ //关闭设备 
    struct keyctl_device_t* devkey_device = (struct keyctl_device_t*)device;	
    if(devkey_device){		
        close(devkey_device->fd);
        free(devkey_device);
    }
    return 0;
}


void soundOnOff(void *arg)
{
    struct input_event t;
    while (1)//线程循环。
    {
        read(keys_fd, &t, sizeof (t));//阻塞读。如果读完了所有消息，线程会阻塞在read函数上。一旦有新的event事件，read函数返回。
        if (t.type == EV_KEY)
        {
            if (t.value == 1)//key down
            {
                count++;
                if(soundOnOff_flag == 1)
                {
                    BeepOn(4000);//如果有一个键被按下，蜂鸣器发声。
                    usleep(100000); //只响100ms
                    BeepOn(0);//停止蜂鸣器。                    
                } 
                if(backlightOnOff_flag == 2)
                {
                    count--;
                    if(count == 0)
                    {
                        ioctl(backlight_fd,KEY_LIGHT,1);
                        usleep(2000000); //只响应5s
                        ioctl(backlight_fd,KEY_LIGHT,0); 
                    }                    
                }
                
                ALOGD ("soundOnOff_thread key= %d %s\n", t.code,(t.value) ? "Pressed" : "Released");
            }
            if(t.value == 0)
            {
                BeepOn(0);//停止蜂鸣器。
            }
          
            if(soundOnOff_flag == 0)
            {
                ALOGD ("soundOnOff_flag=0.soundOnOff thread exit");                    
                return ;
            }
        }        
   }
}


void backlightOn(void *arg)
{
    struct input_event t;

    while (1)//线程循环。
	{
		read(keys_fd, &t, sizeof (t));//阻塞读。如果读完了所有消息，线程会阻塞在read函数上。一旦有新的event事件，read函数返回。
		if (t.type == EV_KEY)
		{
			if (t.value == 1)//key down
			{
                count++;
                ALOGD ("backlightOn_thread key =  %d %s\n", t.code,(t.value) ? "Pressed" : "Released");
                if(soundOnOff_flag == 1)
                {
                    BeepOn(4000);//如果有一个键被按下，蜂鸣器发声。
                    usleep(100000); //只响100ms
                    BeepOn(0);//停止蜂鸣器。                    
                }
                
                if(backlightOnOff_flag == 2)
                {
                    count--;
                    if(count == 0)
                    {
                        ioctl(backlight_fd,KEY_LIGHT,1);
                        usleep(2000000); //只响应5s
                        ioctl(backlight_fd,KEY_LIGHT,0); 
                    }                                       
                }
			}        
           
            if(backlightOnOff_flag == 0)
            {
                ALOGD ("backlightOnOff_flag=0. backlightOn thread exit");                      
                return ;
            }
		}		
   }
}


static int keypad_device_enable(struct keyctl_device_t* device,int key)//打开设备 
{ 
    ALOGD ("keypad_device_enable====start");
    int res =0;
    int res1 =0;   	
    
    if(key == 1) //声音
    {
        ALOGD("enable beep");
        soundOnOff_flag = 1;
        if(BeepOpen()<0)
        {
            ALOGD("BeepOpen() error.");
        } 
        res=pthread_create(&th_sound, NULL, (void *)soundOnOff, (void *)0);//创建声音线程
        if(res != 0)
        {
            ALOGD("Fail to create a new thread th_sound");
            return -1;
        }
    }else if(key == 2)//背光灯
    {
        ALOGD("enable backlight");
        backlightOnOff_flag = 2;
        if(backlight_open()<0)
        {
            ALOGD("backlight_open() error.");
        }
        res1=pthread_create(&th_backlight, NULL, (void*)backlightOn, (void *)0);//创建背光响应线程
        if(res1 != 0)
        {
            ALOGD("Fail to create a new thread th_backlight");
            return -1;
        } 
    }
    ALOGD ("keypad_device_enable====end");
	return 0;
}


static int keypad_device_disable(struct keyctl_device_t* device,int key)//关闭设备 
{ 
    ALOGD ("keypad_device_disable===start");
    if(key == 1)
    {
        ALOGD("disable beep");
        soundOnOff_flag = 0;
        if(BeepClose() <0)
        {
            ALOGD("BeepClose() error.");
            return -1;
        }
    }else if(key ==2)
    {
        ALOGD("disable backlight");
        backlightOnOff_flag = 0;
        if(close(backlight_fd) <0 )
        {
            ALOGD("close backlight_fd device error.");
            return -1;
        }
    }	
    
   /*  if(close(keys_fd) <0 )
    {
        ALOGD("close key  device error.");
        return -1;
    } */
    ALOGD ("keypad_device_disable===end");
    return 0;
}

static int keypad_set_val(struct keyctl_device_t* dev,int val){
    ALOGD("keyctr Stub: set value %d to device.",val);
    write(dev->fd,&val,sizeof(val)); 
    return 0;
}
static int keypad_get_val(struct keyctl_device_t* dev,int* val){
    if(!val){
        ALOGE("keyctr Stub: error val pointer");
        return -EFAULT;
    }
    read(dev->fd,val,sizeof(*val));
    ALOGD("keyctr Stub: get value %d from device",*val);
    return 0;
}
