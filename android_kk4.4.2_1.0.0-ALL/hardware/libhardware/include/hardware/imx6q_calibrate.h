#ifndef ANDROID_IMX6Q_SPI_INTERFACE_H
#define ANDROID_IMX6Q_SPI_INTERFACE_H
#include <hardware/hardware.h>
#include <pthread.h>
#include <stdbool.h>

__BEGIN_DECLS
#define DEBUG_TEST  1
/*定义模块ID*/
#define CALIBRATECTL_HARDWARE_MODULE_ID "imx6q_calibrate"

/*硬件模块结构体*/

typedef void (*spictl_double_ch_callback) (float data[],float data2[],int length,bool isCollectedData);
typedef void (*spictl_single_ch_callback) (float data[],int length,bool isCollectedData);
typedef void (*spictl_stop_ch_callback)   (bool isCollectedData);
//typedef void (*spictl_error_callback)     (int data, bool isCollectedData);

typedef pthread_t (* spictl_create_thread)(const char* name, void (*start)(void *), void* arg);

typedef struct {
	spictl_single_ch_callback    single_ch_callback;
	spictl_double_ch_callback    double_ch_callback;
	spictl_stop_ch_callback      stop_ch_callback;
	//spictl_error_callback        error_callback;
} SpiVibrateCallbacks;


typedef struct {
    void   (*initVibrate)( SpiVibrateCallbacks* callbacks );
} SpiVibrateInterface;


struct spictl_module_t{
     struct hw_module_t common;
};
/*硬件接口结构体*/
struct spictl_device_t {
    struct hw_device_t common;
    int  fd;
	int  (*start_vibrate_calibrate)(struct spictl_device_t* dev, struct time_wave_para wave);
	int  (*stop_vibrate_ad)(struct spictl_device_t* dev);
    const SpiVibrateInterface* (*get_vibrate_interface)(struct spictl_device_t* dev);
};

__END_DECLS
#endif
