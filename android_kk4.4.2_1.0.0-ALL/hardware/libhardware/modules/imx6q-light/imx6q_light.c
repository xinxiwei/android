#define LOG_TAG "lightctrStub"
#include <hardware/hardware.h>
#include <hardware/imx6q_light.h>
#include <hardware/gpio.h>
#include <fcntl.h>
#include <errno.h>
#include <cutils/log.h>
#include <cutils/atomic.h>
#include<sys/ioctl.h>


#define DEVICE_NAME "/dev/input/event2"
#define MODULE_NAME "xxx"
#define MODULE_AUTHOR "xxx@126.com"

/*设备打开和关闭接口*/
static int light_device_open(const struct hw_module_t* module,const char* name,struct hw_device_t** device);
static int light_device_close(struct hw_device_t* device);
/*设备访问接口*/
static int light_device_enable(struct hw_device_t* device);
static int keypad_device_disable(struct hw_device_t* device);
static int light_set_val(struct lightctl_device_t* dev,int val);
static int light_get_val(struct lightctl_device_t* dev,int* val);
/*模块方法表*/
static struct hw_module_methods_t light_module_methods={
   open: light_device_open
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
        methods: &light_module_methods,
    }
};
static int light_device_open(const struct hw_module_t* module,const char* name,struct hw_device_t** device)
{
    struct lightctl_device_t* dev;

    ALOGD( "lightctr is ready for opening hal-files.");
    dev = (struct lightctl_device_t*)malloc(sizeof(struct lightctl_device_t));
    if(!dev){
        ALOGE("lightctr: failed to alloc space");
        return -EFAULT;
    }
    memset(dev,0,sizeof(struct lightctl_device_t));
    dev->common.tag = HARDWARE_DEVICE_TAG;
    dev->common.version = 0;
    dev->common.module = (hw_module_t*)module;
    dev->common.close = light_device_close;
    dev->set_val = light_set_val;
    dev->get_val = light_get_val;
    if((dev->fd = open(DEVICE_NAME,O_RDWR)) == -1){
        ALOGE("lightctr: failed to open /dev/input/event2 -- %s.",strerror(errno));
        free(dev);
        return -EFAULT;
    }
    *device =&(dev->common);
    ALOGD( "lightctr: open /dev/input/event2 successfully.");
    return 0;
}
static int light_device_close(struct hw_device_t* device){
    struct lightctl_device_t* devpwr_device = (struct lightctl_device_t*)device;
    if(devpwr_device){
        close(devpwr_device->fd);
        free(devpwr_device);
    }
    return 0;
}
static int light_set_val(struct lightctl_device_t* dev,int val){
    ALOGD("lightctr: set value %d to device.",val);
    write(dev->fd,&val,sizeof(val)); 
/*
    if (val==1)
    {
        ioctl(dev->fd,3,1);
    }else{
        ioctl(dev->fd,3,0);
    }
*/
    return 0;
}
static int light_get_val(struct lightctl_device_t* dev,int* val){
    if(!val){
        ALOGE("lightctr: error val pointer");
        return -EFAULT;
    }
    read(dev->fd,val,sizeof(*val));
    ALOGD("lightctr: get value %d from device",*val);
    return 0;
}
