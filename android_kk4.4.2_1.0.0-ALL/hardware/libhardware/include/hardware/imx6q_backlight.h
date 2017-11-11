#ifndef ANDROID_IMX6Q_CTR_INTERFACE_H
#define ANDROID_IMX6Q_CTR_INTERFACE_H
#include <hardware/hardware.h>

__BEGIN_DECLS

/*定义模块ID*/
#define BACKLIGHTCTL_HARDWARE_MODULE_ID "imx6q_backlight"

/*硬件模块结构体*/
struct gpioctl_module_t{
     struct hw_module_t common;
};
/*硬件接口结构体*/
struct gpioctl_device_t {
    struct hw_device_t common;
    int fd;
	int (*gpioctl_close)(struct gpioctl_device_t* dev);
    int (*set_backlight_val)(struct gpioctl_device_t* dev,int value1,int value2);
    int (*get_backlight_val)(struct gpioctl_device_t* dev,int* val);
};
__END_DECLS
#endif
