#ifndef ANDROID_IMX6Q_CTR_INTERFACE_H
#define ANDROID_IMX6Q_CTR_INTERFACE_H
#include <hardware/hardware.h>

__BEGIN_DECLS

/*定义模块ID*/
#define GPIOCTL_HARDWARE_MODULE_ID "imx6q_ctr"

/*硬件模块结构体*/
struct gpioctl_module_t{
     struct hw_module_t common;
};
/*硬件接口结构体*/
struct gpioctl_device_t {
    struct hw_device_t common;
    int fd;
	int (*gpioctl_close)(struct gpioctl_device_t* dev);
    int (*set_val)(struct gpioctl_device_t* dev,int value1,int value2);
    int (*get_val)(struct gpioctl_device_t* dev,int* val);
};
__END_DECLS
#endif
