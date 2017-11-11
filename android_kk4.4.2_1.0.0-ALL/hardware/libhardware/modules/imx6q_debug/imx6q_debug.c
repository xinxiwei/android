#define LOG_TAG "debugStub"
#include <hardware/hardware.h>
#include <hardware/imx6q_debug.h>
#include <hardware/imx6q_spi_config.h>
#include <fcntl.h>
#include <errno.h>
#include <cutils/log.h>
#include <cutils/atomic.h>
#include<sys/ioctl.h>


#define DEVICE_NAME "/dev/devhello"
#define MODULE_NAME "devhello"
#define MODULE_AUTHOR "xxw@126.com"

/*设备打开和关闭接口*/
static int debug_device_open(const struct hw_module_t* module,const char* name,struct hw_device_t** device);
static int debug_device_close(struct debug_device_t* device);
/*设备访问接口*/

static int debug_close(struct debug_device_t* dev);
static int debug_set_val(struct debug_device_t* dev,int val);
static int debug_get_val(struct debug_device_t* dev,int* val);
/*模块方法表*/
static struct hw_module_methods_t debug_module_methods={
   open: debug_device_open
};
/*模块实例变量*/
struct debug_module_t HAL_MODULE_INFO_SYM ={
    common:{
        tag: HARDWARE_MODULE_TAG,
        version_major: 1,
        version_minor: 0,
        id: DEBUG_HARDWARE_MODULE_ID,
        name: MODULE_NAME,
        author: MODULE_AUTHOR,
        methods: &debug_module_methods,
    }
};
static int debug_device_open(const struct hw_module_t* module,const char* name,struct hw_device_t** device)
{
    struct debug_device_t* dev;

    dev = (struct debug_device_t*)malloc(sizeof(struct debug_device_t));
    if(!dev){
        ALOGE("debug Stub: failed to alloc space");
        return -EFAULT;
    }
    memset(dev,0,sizeof(struct debug_device_t));
    dev->common.tag = HARDWARE_DEVICE_TAG;
    dev->common.version = 0;
    dev->common.module = (hw_module_t*)module;
    dev->debug_close = debug_device_close;
    dev->set_dev_val = debug_set_val;
    dev->get_dev_val = debug_get_val;
    *device =&(dev->common);   
    return 0;
}

static int debug_device_close(struct debug_device_t* device){
    struct debug_device_t* devpwr_device = (struct debug_device_t*)device;
    if(devpwr_device){
        close(devpwr_device->fd);
        free(devpwr_device);
    }
    return 0;
}
static int debugfd = -1;
static int debug_set_val(struct debug_device_t* dev,int val){
	debugfd = open(DEVICE_NAME,O_RDWR);
     if(debugfd == -1){		
        ALOGE("debugfd Stub: failed to open /dev/devdebug -- %s.",strerror(errno));
        free(dev);
        return -EFAULT;
    }else{
		ALOGD( "debugfd Stub: open /dev/devdebug successfully.debugfd =%d",debugfd);
	}
    return 0;
}

static int debug_get_val(struct debug_device_t* dev,int* val){
    if(!val){        
        return -EFAULT;
    }
    read(dev->fd,val,sizeof(*val));
    ALOGD("debug Stub: get value %d from device",*val);
    return 0;
}
