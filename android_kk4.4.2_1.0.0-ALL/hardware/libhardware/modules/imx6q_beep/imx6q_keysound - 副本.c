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
int KEY_ESC_CODE = 8;
int KeySoundOn =1;
static final int KEY_LIGHT =9;
pthread_t th_sound;

/*设备打开和关闭接口*/
static int keypad_device_open(const struct hw_module_t* module,const char* name,struct hw_device_t** device);
static int keypad_device_close(struct hw_device_t* device);
/*设备访问接口*/

static int backlight_device_enable(struct hw_device_t* device);
static int backlight_device_disable(struct hw_device_t* device);

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

    ALOGD( "keyctr is ready for opening hal-files.");
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
	dev->backlight_enable = backlight_device_enable;
	dev->backlight_disable = backlight_device_disable;
	
    *device =&(dev->common);
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

int backlight_open()
 {
	 if ((backlight_fd = open("/dev/devlightctr", O_RDWR)) < 0)
	 {
		 LOGD("backlight open fail.");
		 return -1;
	 }
	 LOGD( "backlight open /dev/devlightctr success. fd = %d",backlight_fd);
	 ioctl(backlight_fd,KEY_LIGHT,1);
	 usleep(3000000); //只响应3s
	 ioctl(backlight_fd,KEY_LIGHT,0);
	 close(backlight_fd);
	 return 0;
 }

static int backlight_device_enable(struct hw_device_t* device)//打开设备 
{ 
    ALOGD ("backlight_device_enable=============");
    keys_fd = open (DEVICE_NAME, O_RDONLY);
    if (keys_fd <= 0)
    {
        ALOGD ("open /dev/input/event0 device error!");
        if(close(keys_fd) <0)
        {
            ALOGD ("close /dev/input/event0 error!");
        }
        return -1;
    }    
    ALOGD("open keypad device. fd=%d",keys_fd);
	
	if(backlight_open()<0)
	{
		ALOGD("backlight_open() error.");
	}	
	return 0;
}

static int backlight_device_disable(struct hw_device_t* device)//关闭设备 
{ 
    ALOGD ("backlight_device_disable=============");
    ALOGD("close backlight_device. fd=%d",keys_fd);
  
    if(close(keys_fd) <0 )
    {
        ALOGD("close keypad device error.");
        return -1;
    }
	if(close(backlight_fd) <0 )
    {
        ALOGD("close backlight device error.");
        return -1;
    }
    return 0;
}
