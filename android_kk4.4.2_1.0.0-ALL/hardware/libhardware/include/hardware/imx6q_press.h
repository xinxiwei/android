#ifndef ANDROID_IMX6Q_SPI_INTERFACE_H
#define ANDROID_IMX6Q_SPI_INTERFACE_H
#include <hardware/hardware.h>
#include <pthread.h>
#include <stdbool.h>

__BEGIN_DECLS
#define DEBUG_TEST  1
/*定义模块ID*/
#define PRESSCTL_HARDWARE_MODULE_ID "imx6q_press"

/*硬件模块结构体*/

typedef void (*spictl_pressure_callback)   (float data[],bool isCollectedData); //函数指针
typedef void (*spictl_stop_press_callback) (bool isCollectedData);

typedef void (*spictl_double_ch_callback) (float data[],float data2[],int length,bool isCollectedData);
typedef void (*spictl_single_ch_callback) (float data[],int length,bool isCollectedData);
typedef void (*spictl_stop_ch_callback)   (bool isCollectedData);
//typedef void (*spictl_error_callback)     (int data, bool isCollectedData);

typedef pthread_t (* spictl_create_thread)(const char* name, void (*start)(void *), void* arg);


typedef struct {
	spictl_pressure_callback          mspictl_callback;
	spictl_create_thread              create_thread_cb;
	spictl_stop_press_callback        stop_press_callback;
} SpiPressureCallbacks;

typedef struct {
    void   (*initPress)( SpiPressureCallbacks* callbacks );
} SpiPressureInterface;


struct spictl_module_t{
     struct hw_module_t common;
};
/*硬件接口结构体*/
struct spictl_device_t {
    struct hw_device_t common;
    int   fd;
    int  (*set_val)(struct spictl_device_t* dev,int val);
    int  (*get_val)(struct spictl_device_t* dev,int* val);

	int  (*start_press_dial)(struct spictl_device_t* dev,float data);
	int  (*start_press_curve)(struct spictl_device_t* dev,float data);
	int  (*start_press_flag0)(struct spictl_device_t* dev);

	int  (*stop_press_ad)(struct spictl_device_t* dev);
	int  (*spi_freq)(struct spictl_device_t* dev);

    const SpiPressureInterface* (*get_pressure_interface)(struct spictl_device_t* dev);
};


__END_DECLS
#endif
