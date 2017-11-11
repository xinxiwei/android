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
int KEY_ESC_CODE = 8;
int KeySoundOn =1;
pthread_t th_sound;

/*设备打开和关闭接口*/
static int keypad_device_open(const struct hw_module_t* module,const char* name,struct hw_device_t** device);
static int keypad_device_close(struct hw_device_t* device);
/*设备访问接口*/
static int keypad_device_enable(struct hw_device_t* device);
static int keypad_device_disable(struct hw_device_t* device);
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
	dev->keysound_enable = keypad_device_enable;
	dev->keysound_disable = keypad_device_disable;
    dev->set_val = keypad_set_val;
    dev->get_val = keypad_get_val;
    *device =&(dev->common);
    ALOGD( "keyctr Stub: open /dev/input/event0 successfully.");
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

int EnableKeySound() //键盘声音打开
{
    KeySoundOn = 1;
    return 0;
}

int DisableKeySound() //键盘声音关闭
{
    KeySoundOn = 0;
    return 0;
}

void* soundOnOff(void *arg)
{
	ALOGD("soundOnOff=======");
    struct input_event t;
    int keyDownCount=0;
       while (1)//线程循环。
        {
            read(keys_fd, &t, sizeof (t));//阻塞读。如果读完了所有消息，线程会阻塞在read函数上。一旦有新的event事件，read函数返回。
            if (t.type == EV_KEY)
            {
                if (t.value == 1)//key down
                {
                    keyDownCount++;//对所有键被按下的个数进行计数。
                    if(KeySoundOn == 1)
                    {
                        BeepOn(4000);//如果有一个键被按下，蜂鸣器发声。
						usleep(100000); //只响100ms
						BeepOn(0);//停止蜂鸣器。						
                    }

                    ALOGD ("key %d %s\n", t.code,(t.value) ? "Pressed" : "Released");
                }else if(t.value == 0) //key up
                {
                    keyDownCount--;//对所有按键弹起事件进行计数
                    if(keyDownCount <= 0)//如果弹起事件等于或大于按下事件，说明现在没有键被按下了。
                    {
                        BeepOn(0);//停止蜂鸣器。
                        keyDownCount =0;//复位计数器的值为0。
                    }
                    ALOGD ("key %d %s\n", t.code,(t.value) ? "Pressed" : "Released");
                    if(t.code == KEY_ESC_CODE)//判断如果是按下了退出按钮
                    {
                        ALOGD ("KEY_ESC_CODE checked.");
                        BeepOn(0);//停止蜂鸣器。
                        pthread_exit("soundOnOff thread exit.");//退出当前线程。
                    }
                }

            }

       }
}

int StartKeySoundThread()
{
    ALOGD("StartKeySoundThread=======");
    int res=pthread_create(&th_sound, NULL, soundOnOff, (void *)0);//创建声音线程
    if(res != 0)
    {
        ALOGD("Fail to create a new thread th_sound");
        return -1;
    }
    return 0;
}

static int keypad_device_enable(struct hw_device_t* device)//打开设备 
{ 
    ALOGD ("keypad_device_enable=============");
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
    ALOGD("open key sound device. fd=%d",keys_fd);
	
	if(BeepOpen()<0)
	{
		ALOGD("BeepOpen() error.");
	}
	
	EnableKeySound();
    StartKeySoundThread();
	return 0;
}

static int keypad_device_disable(struct hw_device_t* device)//关闭设备 
{ 
    ALOGD ("keypad_device_disable=============");
    ALOGD("close key sound device. fd=%d",keys_fd);
    DisableKeySound();
	
	if(BeepClose() <0)
	{
		ALOGD("BeepClose() error.");
	}
    if(close(keys_fd) <0 )
    {
        ALOGD("close key sound device error.");
        return -1;
    }
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
