/*
 *
 * Date:2016-8-9
 * Author:lhx
 * Description: this file call the linux driver .
 *
 *
 */
#include <hardware/log.h>
#include <hardware/gpio.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <pthread.h>

static int gpiofd = -1;

int GpioOpen()
{
    gpiofd = open("/dev/devpwerctr", O_RDWR);
        if (gpiofd < 0)
        {
            LOGE("gpio open error");
            return gpiofd;
        }
        else
        {
            //LOGD("gpio open. fd=%d",gpiofd);
            return 0;
        }
}

int GpioClose()
{
    LOGD("gpio close fd = %d",gpiofd);
    if(close(gpiofd) <0)
    {
        LOGE("gpio close error.");
        return -1;
    }
    return 0;
}


int GpioSet(int device, int value)
{
	usleep( 50000 );
	if(gpiofd <0)
		{
			LOGE("gpio fd < 0");
			return -1;
			
			}
	else
		{
            int ret = ioctl(gpiofd,device,value);
            if(ret<0)
                {
                    LOGE("gpio ioctl operation false.ret=%d.",ret);
                    LOGE("gpio gpiofd= %d",gpiofd);
                    LOGE("gpio device= %d",device);
                    LOGE("gpio value= %d",value);
                   return -1;
                }
			
			}
    return 0;
}






