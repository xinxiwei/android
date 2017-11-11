#ifndef ANDROID_XIN_HELLO_INTERFACE_H
#define ANDROID_XIN_HELLO_INTERFACE_H
#include <hardware/hardware.h>


__BEGIN_DECLS

/*����ģ��ID*/
#define DEBUG_HARDWARE_MODULE_ID "xin_debug"

/*Ӳ��ģ��ṹ��*/
struct debug_module_t{
     struct hw_module_t common;
};
/*Ӳ���ӿڽṹ��*/
struct debug_device_t {
    struct hw_device_t common;
    int fd;
    int (*debug_close)(struct debug_device_t* dev);
    int (*set_dev_val)(struct debug_device_t* dev,int val);
    int (*get_dev_val)(struct debug_device_t* dev,int* val);
};
__END_DECLS
#endif
