#define LOG_TAG "spiStub"
#include <hardware/hardware.h>
#include <hardware/imx6q_spi.h>
#include <hardware/imx6q_spi_config.h>
#include <hardware/gpio.h>
#include <hardware/log.h>
#include <hardware/masterspi.h>
#include <hardware/slavespi.h>
#include <hardware/spidev.h>
#include <hardware/errorcode.h>
#include <hardware/beep.h>
#include <fcntl.h>
#include <stdbool.h>
#include <math.h>
#include <errno.h>
#include <cutils/log.h>
#include <cutils/atomic.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <sys/timeb.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h> 


void spi_power_off() //spi设备下电
{	   
	poweroff_spi( );  //spi下电	    
}

/* 判断下发的采样长度 */
bool is_right_length(int length)
{    
    if(length == 1024 || length == 2048 || length == 4096 || length == 8192 || length == 16384 || length == 32768 || length == 65536 || length == 131072 || length == 262144 )
    {
        return true;
    }else{
        LOGD("xin: 下发的波形长度 tWave.wave_length = %d",length);
        return false;
    }
}

///////////////////adc数据提取算法



/*
void analyze_single_data(int *src, int length, float *value1)
{
    if(value1 == NULL || src == NULL)
    {
       exit( EXIT_FAILURE );
    }
	int i =0;
	for(i =0;i <length; i++)
	{
		int higetst_bit =( src[i] >> 31 )&0x1;
		int channel_bit =( src[i] >> 30 )&0x1;
		if( higetst_bit == 0x1 ) //1表示转速数据
		{
			;
		}
		else //振动
		{
			if( channel_bit == 0 ) //0表示 通道1 数据
			{				
				value1[i] = adc_data_to_24value( src[i] );                				
			}							
		}
	}
}

void analyze_double_data(int *src, int length,float *value1,float *value2)
{	
    if(src == NULL || value1 == NULL ||value2 == NULL)
    {
       exit( EXIT_FAILURE );
    }
	int i =0;    
	for(i =0;i < length; i++)
	{		
		int higetst_bit =( src[i] >> 31 )&0x1;
		int channel_bit =( src[i] >> 30 )&0x1;
		if( higetst_bit == 0x1 ) //1表示转速数据
		{
			;
		}
		else //振动
		{			
			if( channel_bit == 0 ) //0表示 通道1 数据
			{									
				value1[i] = adc_data_to_24value( src[i] );				 						
			}else if( channel_bit == 1 ) // 1表示 通道2 数据
			{									
				value2[i] = adc_data_to_24value( src[i] );                					
			}						
		}
	}
}
*/
void dis_dc_func(float *src, int length) //满足中船手持，去除直流分量算法
{
    if(src == NULL)
    {
       exit( EXIT_FAILURE );
    }
	float sum = 0.0;
	float average_value = 0.0;
	int i =0;
	for(i = 0; i< length; i++)
	{
		 sum += src[i];		 
	}
	average_value = sum/length;  //求出直流分量偏置
	for(i =0;i<length;i++)
	{
		 src[i] = src[i] - average_value;
	}
    return ; 		
}
