#ifndef ANDROID_IMX6Q_KEY_INTERFACE_H
#define ANDROID_IMX6Q_KEY_INTERFACE_H
#include <hardware/hardware.h>

__BEGIN_DECLS

/*定义模块ID*/
#define KEYCTL_HARDWARE_MODULE_ID "imx6q_keysound"

/*硬件模块结构体*/
struct keyctl_module_t{
     struct hw_module_t common;
};
/*硬件接口结构体*/
struct keyctl_device_t {
    struct hw_device_t common;
    int fd;
	int (*keysound_enable)(struct keyctl_device_t* dev);
	int (*keysound_disable)(struct keyctl_device_t* dev);
    int (*set_val)(struct keyctl_device_t* dev,int val);
    int (*get_val)(struct keyctl_device_t* dev,int* val);
};
__END_DECLS
#endif
