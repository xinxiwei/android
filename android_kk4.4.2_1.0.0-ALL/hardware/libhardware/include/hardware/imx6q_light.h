#ifndef ANDROID_IMX6Q_LIGHT_INTERFACE_H
#define ANDROID_IMX6Q_LIGHT_INTERFACE_H
#include <hardware/hardware.h>

__BEGIN_DECLS

/*定义模块ID*/
#define LIGHTCTL_HARDWARE_MODULE_ID "light sensor"

/*硬件模块结构体*/
struct lightctl_module_t{
     struct hw_module_t common;
};
/*硬件接口结构体*/
struct lightctl_device_t {
    struct hw_device_t common;
    int fd;
    int (*set_val)(struct lightctl_device_t* dev,int val);
    int (*get_val)(struct lightctl_device_t* dev,int* val);
};
__END_DECLS
#endif
