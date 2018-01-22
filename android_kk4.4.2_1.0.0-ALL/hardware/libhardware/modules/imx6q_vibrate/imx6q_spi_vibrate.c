#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <malloc.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <hardware/log.h>
#include <cutils/log.h>
#include <hardware/math.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <pthread.h>
#include <hardware/gpio.h>
#include <hardware/log.h>
#include <hardware/masterspi.h>
#include <hardware/slavespi.h>
#include <hardware/spidev.h>
#include <sys/timeb.h>
#include <time.h>
#include <math.h>
#include <stdbool.h>
#include <hardware/imx6q_walg_tda.h>
#include <hardware/imx6q_common_config.h>

/* 功能描述：求出15个特征值 */
void feature_value(float *pData, int length, float *ret_value)
{
	int i=0;
    double *src_data=NULL;
	if (pData != NULL && length != 0)
	{
        if (src_data == NULL)
		{
			src_data = (double*)malloc(length *sizeof(double));
			if (src_data == NULL)
			{
				LOGD("src_data 分配内存失败！" );
				return;
			}
			memset(src_data, 0, length*sizeof(double));
		}

		for(i=0;i<length;i++)
		{
			src_data[i] =(double)pData[i];
		}

		ret_value[0] = (float)WALG_TDA_TruePeakValue(src_data, length); //真峰值
		ret_value[1] = (float)WALG_TDA_TruePeakToPeakValue(src_data, length);//真峰峰值
		ret_value[2] = (float)WALG_TDA_RMSValue(src_data, length); //真有效值

		ret_value[3] = (float)WALG_TDA_Max(src_data, length);//最大值
		ret_value[4] = (float)WALG_TDA_Min(src_data, length);//最小值
		ret_value[5] = (float)WALG_TDA_DCValue(src_data, length);//均值

		ret_value[6] = (float)WALG_TDA_Skew(src_data, length);//偏度(歪度)
		ret_value[7] = (float)WALG_TDA_Variance(src_data, length);//方差

		ret_value[8] = (float)WALG_TDA_CrestFactor(src_data, length);//峰值因子
		ret_value[9] = (float)WALG_TDA_Kurtosis(src_data, length);//峭度指标
		ret_value[10] = (float)WALG_TDA_ShapeFactor(src_data, length);//波形因数
		ret_value[11] = (float)WALG_TDA_ImpulseFactor(src_data, length);//脉冲因子
		ret_value[12] = (float)WALG_TDA_ClearanceFactor(src_data, length);//裕度因子

		ret_value[13] = (float)WALG_TDA_AbsDCValue(src_data, length);//平均幅值
		ret_value[14] = (float)WALG_TDA_SqrRootValue(src_data, length);//方根幅值
	}

	if(src_data  !=  NULL)
	{
		free(src_data);
		src_data=NULL;
	}
}

/* 功能描述：针对总值趋势类型分别求 单个特征值 */
float rend_value(float *pData, int length, int totalvalue_type)
{
	float ret_value = 0.0;
	double *src_data2=NULL;
	if (pData != NULL && length != 0)
	{
        if (src_data2 == NULL)
		{
			src_data2 = (double*)malloc(length *sizeof(double));
			if (src_data2 == NULL)
			{
				LOGD("src_data2 分配内存失败！" );
                return 0;
			}
			memset(src_data2, 0, length*sizeof(double));
		}
		int i=0;
		for(i=0;i< length;i++)
		{
			src_data2[i] =(double) pData[i];
		}
		switch (totalvalue_type)
		{
			case SG_TIMESIGNAL_HANDLE_RMS: //有效值0
				 ret_value = (float)WALG_TDA_RMSValue(src_data2, length);
				 break;
			case SG_TIMESIGNAL_HANDLE_PK: //峰值1
				 ret_value = (float)WALG_TDA_TruePeakValue(src_data2, length);
				 break;
			case SG_TIMESIGNAL_HANDLE_PP: //峰峰值2
				 ret_value = (float)WALG_TDA_TruePeakToPeakValue(src_data2, length);
				 break;
            case SG_TIMESIGNAL_HANDLE_DC: //平均值3
				 ret_value = (float)WALG_TDA_DCValue(src_data2, length);
				 break;
			default:
				 break;
		}
	}
	if(src_data2  !=  NULL)
	{
		free(src_data2);
		src_data2 =NULL;
	}

	return  ret_value;
}

bool is_valid_length(int length)
{
    if(length == LENGTH_1K || length == LENGTH_2K || length == LENGTH_4K || length == LENGTH_8K || length == LENGTH_16K || length == LENGTH_32K || length == LENGTH_64K || length == LENGTH_128K || length == LENGTH_256K)
    {
        return true;
    }else{
        return false;
    }
}

inline float adc_data_to_24value(int adc_data)// 将采集32位数据 转换为24位有效数据
{
    bool highbit_flag = false;
    float value = 0.0;
    adc_data &= 0x00ffffff;//24位数据
    highbit_flag = (bool)((adc_data>>23)&0x1); //最高位符号位
    adc_data &= 0x007fffff;
    if(highbit_flag)
    {
        value = (float)((adc_data-0x800000)*2.5)/0x7fffff;//分段函数
    }
    else
    {
        value = (float)(adc_data*2.5)/0x7fffff;
    }
	return value;
}

inline void analyze_CH_data(int adc_data, float *value)//解析出各通道的数据
{
    if(value == NULL)
    {
        return;
    }
	int higetst_bit = (adc_data >> 31)&0x1;
	int channel_bit = (adc_data >> 30)&0x1;

	if(higetst_bit == 0x1) //1表示转速数据
	{
		value[0] = adc_data&0x7fffff;
	}
	else //振动数据
	{
		if(channel_bit == 0) //0表示 通道1 数据
		{
			value[1] = adc_data_to_24value(adc_data);
		}
		else if(channel_bit == 1) // 1表示 通道2 数据
		{
			value[2] = adc_data_to_24value(adc_data);
		}
	}
}

void dis_dc_func(float *src, int length) //满足中船手持，去除直流分量算法
{
    if(src == NULL || length == 0)
    {
        return;
    }
	float sum = 0.0, average_value = 0.0;
	int i = 0;
	for(i = 0; i < length; i++)
	{
	    sum += src[i];
	}
	average_value = sum/length;  //求出直流分量偏置
    LOGD("经过去直流分量算法 average_value = %f", average_value);
	for(i = 0; i < length; i++)
	{
		src[i] = src[i] - average_value;
	}
    return;
}

inline void read_calib_para(int signal_type, int max_freq, int min_freq, int vol_range, float *calib_para)
{
    float mgain = 0.0, moffset = 0.0;
    int offset_value = 0;
	unsigned int gain_addr = 0, offset_addr = 0;
    if(calib_para == NULL)
    {
        return;
    }
    switch(signal_type)
    {
        case ACC_TYPE: //加速度, 只有两种情况，用上限区分
            if(max_freq == FREQ_40000 && vol_range == VOL_RANGE_V25) //40K， 电压量程是25V
            {
                LOGD("read_calib_para加速度信号, 只用于上限 = 40000HZ，且电压量程是25V");
                gain_addr = REG_FRAM_ADDR1;
            }else{
                LOGD("read_calib_para加速度信号, 用于其它各种参数");
                gain_addr = REG_FRAM_ADDR2;
            }
            break;
        case VEL_TYPE: //速度
            if(vol_range == VOL_RANGE_V25)//25V
            {
                if(((max_freq <= FREQ_20000)&&(min_freq >= FREQ_10)) 
                || ((max_freq <= FREQ_20000)&&(max_freq >= FREQ_4000)&& (min_freq < FREQ_10)))
                {
                    LOGD("read_calib_para速度信号，电压量程是 25V, 用于上限 >=500，<=20000HZ, 下限>= 10HZ, 或者上限 >=4000, <=20000， 下限 < 10HZ");
                    gain_addr = REG_FRAM_ADDR3;
                }else if(((max_freq == FREQ_40000) && (min_freq >= FREQ_10))
                       || ((max_freq == FREQ_40000)&& (min_freq < FREQ_10))
                       || ((max_freq <= FREQ_2000)&& (min_freq < FREQ_10)))
                {
                    LOGD("read_calib_para速度信号，电压量程是 25V, 用于上限 = 40000 下限>=10HZ, 或 上限<= 2000, 下限小于10HZ 或 上限 = 40000 下限小于10HZ");
                    gain_addr = REG_FRAM_ADDR4;
                }
            }else if(vol_range == VOL_RANGE_V2P5)//2.5V
            {
                LOGD("read_calib_para速度信号，电压量程是 2.5V");
                gain_addr = REG_FRAM_ADDR5;
            }else if(vol_range == VOL_RANGE_V0P25)//0.25V
            {
                LOGD("read_calib_para速度信号，电压量程是 0.25V");
                gain_addr = REG_FRAM_ADDR6;
            }            
            break; 
        case DSP_TYPE: //位移
            if(vol_range == VOL_RANGE_V25)//25V
            {
                if((max_freq == FREQ_40000)&&(min_freq >= FREQ_10))//40K, AC
                {
                    LOGD("read_calib_para位移信号，电压量程是 25V, 用于上限 = 40000HZ, 下限>= 10HZ");
                    gain_addr = REG_FRAM_ADDR7;
                }else if(((max_freq == FREQ_20000 || max_freq == FREQ_10000 || max_freq == FREQ_5000 || max_freq == FREQ_2500)&&(min_freq >= FREQ_10)) 
                      || ((max_freq == FREQ_20000 || max_freq == FREQ_5000 || max_freq == FREQ_2500)&&(min_freq < FREQ_10)))
                {
                    LOGD("read_calib_para位移信号，电压量程是 25V, 用于上限 = 20000，10000，5000，2500HZ, 下限>= 10HZ 或者上限 = 20000，5000，2500，下限< 10HZ");
                    gain_addr = REG_FRAM_ADDR8;
                }else if(((max_freq == FREQ_4000 || max_freq == FREQ_2000 || max_freq == FREQ_1000 || max_freq == FREQ_500)&&(min_freq >= FREQ_10)) 
                      || ((max_freq == FREQ_40000 || max_freq == FREQ_10000 || max_freq == FREQ_4000 || max_freq == FREQ_2000 || max_freq == FREQ_1000 || max_freq == FREQ_500)&&(min_freq < FREQ_10)))
                {
                    LOGD("read_calib_para位移信号，电压量程是 25V, 用于上限 = 4000，2000，1000，500HZ, 下限>= 10HZ 或者上限 = 40000，10000，4000，2000, 1000, 500，下限< 10HZ");
                    gain_addr = REG_FRAM_ADDR9;
                }
            }else if(vol_range == VOL_RANGE_V2P5)//2.5V
            {
                LOGD("read_calib_para位移信号，电压量程是 2.5V");
                gain_addr = REG_FRAM_ADDR10;
            }else if(vol_range == VOL_RANGE_V0P25)//0.25V
            {
                LOGD("read_calib_para位移信号，电压量程是 0.25V");
                gain_addr = REG_FRAM_ADDR11;
            }            
            break;
        default:
            break;
    }
    
    offset_addr = gain_addr + REG_OFFSET_ADDR1;
    
    mgain = sread(gain_addr);    
    
    offset_value = sread(offset_addr);
    bool highbit_flag = (bool)((offset_value >> 23)&0x1); //最高位符号位
    if(highbit_flag)//表示最高位为1，为负数
    {
        LOGD("读取负数offset");
        offset_value = offset_value & 0x7fffff; //只取后23位数
        moffset = (-1) * offset_value; //添加负号
    }else{
        LOGD("读取正数offset");
        moffset = offset_value;
    }    
    
    calib_para[0] = (float)mgain / MULTIPLE_NUM;
    calib_para[1] = (float)moffset / MULTIPLE_NUM;
    LOGD("read_calib_para 最终gain = %f, offset = %f", calib_para[0], calib_para[1]);
}

