#define LOG_TAG "gpioctrStub"
#include <hardware/hardware.h>
#include <hardware/imx6q_light.h>
#include <hardware/gpio.h>
#include <fcntl.h>
#include <errno.h>
#include <cutils/log.h>
#include <cutils/atomic.h>
#include<sys/ioctl.h>


#define DEVICE_NAME "/dev/devlightctr"  // 控制 红15  绿14  黄13 三个指示灯
#define MODULE_NAME "devlightctr"
#define MODULE_AUTHOR "gxl@126.com"
// 背光四个灯
/*设备打开和关闭接口*/
static int gpio_device_open(const struct hw_module_t* module,const char* name,struct hw_device_t** device);
static int gpio_device_close(struct hw_device_t* device);
/*设备访问接口*/

static int gpio_close(struct gpioctl_device_t* dev);
static int gpio_set_light_val(struct gpioctl_device_t* dev,int value1,int value2);
static int gpio_get_light_val(struct gpioctl_device_t* dev,int* val);
/*模块方法表*/
static struct hw_module_methods_t gpio_module_methods={
   open: gpio_device_open
};
/*模块实例变量*/
struct gpioctl_module_t HAL_MODULE_INFO_SYM ={
    common:{
        tag: HARDWARE_MODULE_TAG,
        version_major: 1,
        version_minor: 0,
        id: GPIOCTL_HARDWARE_MODULE_ID,
        name: MODULE_NAME,
        author: MODULE_AUTHOR,
        methods: &gpio_module_methods,
    }
};
static int gpio_device_open(const struct hw_module_t* module,const char* name,struct hw_device_t** device)
{
    struct gpioctl_device_t* dev;

   // ALOGD( "gpioctr is ready for opening hal-files.");
    dev = (struct gpioctl_device_t*)malloc(sizeof(struct gpioctl_device_t));
    if(!dev){
        ALOGE("gpioctl Stub: failed to alloc space");
        return -EFAULT;
    }
    memset(dev,0,sizeof(struct gpioctl_device_t));
    dev->common.tag = HARDWARE_DEVICE_TAG;
    dev->common.version = 0;
    dev->common.module = (hw_module_t*)module;
    dev->common.close = gpio_device_close;
	dev->gpioctl_close = gpio_close;
    dev->set_light_val = gpio_set_light_val;
    dev->get_light_val = gpio_get_light_val;
    *device =&(dev->common);   
    return 0;
}

static int gpio_device_close(struct hw_device_t* device){
    struct gpioctl_device_t* devpwr_device = (struct gpioctl_device_t*)device;
    if(devpwr_device){
        close(devpwr_device->fd);
        free(devpwr_device);
    }
    return 0;
}
static int gpio_close(struct gpioctl_device_t* device){
    struct gpioctl_device_t* devpwr_device = (struct gpioctl_device_t*)device;
    if(devpwr_device){
        close(devpwr_device->fd);
        free(devpwr_device);
    }
    return 0;
}
static int gpiofd = -1;
static int gpio_set_light_val(struct gpioctl_device_t* dev,int value1,int value2){
	gpiofd = open(DEVICE_NAME,O_RDWR);
	//ALOGE("gpioctr gpiofd = %d.",gpiofd);
     if(gpiofd == -1){		
        ALOGE("gpioctr Stub: failed to open /dev/devlightctr -- %s.",strerror(errno));
        free(dev);
        return -EFAULT;
    }else{
		//ALOGD( "gpioctr Stub: open /dev/devlightctr successfully.");
	}
	int ret = ioctl(gpiofd,value1,value2);
	if(ret<0)
		{
			ALOGE("gpio ioctl operation false.ret=%d.",ret);
			ALOGE("gpio gpiofd= %d",gpiofd);
			ALOGE("gpio value1= %d",value1);
			ALOGE("gpio value2= %d",value2);
		   return -1;
		}
    return 0;
}
static int gpio_get_light_val(struct gpioctl_device_t* dev,int* val){
    if(!val){
        ALOGE("gpioctr Stub: error val pointer");
        return -EFAULT;
    }
    read(dev->fd,val,sizeof(*val));
    ALOGD("gpioctr Stub: get value %d from device",*val);
    return 0;
}
