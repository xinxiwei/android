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
#include <hardware/imx6q_walg_tda.h>


#define  SG_TIMESIGNAL_HANDLE_RMS   0 //有效值
#define  SG_TIMESIGNAL_HANDLE_PK    1 //峰值
#define  SG_TIMESIGNAL_HANDLE_PP    2 //峰峰值

/* 功能描述：求出15个特征值 */
void feature_value(float *pData, int length,float *ret_value)
{
	int i=0;	
    double *src_data=NULL;	
	if (pData!=NULL && length!=0)
	{	
        if ( src_data == NULL)
		{
			src_data = (double*)malloc(length *sizeof(double));
			if ( src_data == NULL)
			{
				LOGD( "src_data 分配内存失败！" );
				exit( EXIT_FAILURE );	
			}
			memset(src_data,0 ,length*sizeof(double));
		}
		
		for(i=0;i<length;i++)
		{
			src_data[i] =(double)pData[i];			
		}
		
		ret_value[0] = (float)WALG_TDA_TruePeakValue(src_data,length); //真峰值
		ret_value[1] = (float)WALG_TDA_TruePeakToPeakValue(src_data,length);//真峰峰值
		ret_value[2] = (float)WALG_TDA_RMSValue(src_data,length); //真有效值
		
		ret_value[3] = (float)WALG_TDA_Max(src_data,length);//最大值
		ret_value[4] = (float)WALG_TDA_Min(src_data,length);//最小值
		ret_value[5] = (float)WALG_TDA_DCValue(src_data,length);//均值
		
		ret_value[6] = (float)WALG_TDA_Skew(src_data,length);//偏度(歪度)
		ret_value[7] = (float)WALG_TDA_Variance(src_data,length);//方差
		
		ret_value[8] = (float)WALG_TDA_CrestFactor(src_data,length);//峰值因子
		ret_value[9] = (float)WALG_TDA_Kurtosis(src_data,length);//峭度指标
		ret_value[10] = (float)WALG_TDA_ShapeFactor(src_data,length);//波形因数
		ret_value[11] = (float)WALG_TDA_ImpulseFactor(src_data,length);//脉冲因子
		ret_value[12] = (float)WALG_TDA_ClearanceFactor(src_data,length);//裕度因子
		
		ret_value[13] = (float)WALG_TDA_AbsDCValue(src_data,length);//平均幅值
		ret_value[14] = (float)WALG_TDA_SqrRootValue(src_data,length);//方根幅值			
	}
	
	if( src_data != NULL)
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
	if (pData!=NULL && length!=0)
	{	
        if ( src_data2 == NULL)
		{
			src_data2 = (double*)malloc(length *sizeof(double));
			if ( src_data2 == NULL)
			{
				LOGD( "src_data2 分配内存失败！" );
				exit( EXIT_FAILURE );	
			}
			memset(src_data2,0 ,length*sizeof(double));
		}
		int i=0;
		for(i=0;i< length;i++)
		{
			src_data2[i] =(double) pData[i];	
		}	
		//LOGD("xin: APP下发的总值单个值 = %d",totalvalue_type);
		switch (totalvalue_type)
		{		
			case SG_TIMESIGNAL_HANDLE_RMS: //有效值0
				 ret_value = (float)WALG_TDA_RMSValue(src_data2,length);             	
				 break;
			case SG_TIMESIGNAL_HANDLE_PK: //峰值1
				 ret_value = (float)WALG_TDA_TruePeakValue(src_data2,length);             	
				 break;
			case SG_TIMESIGNAL_HANDLE_PP: //峰峰值2
				 ret_value = (float)WALG_TDA_TruePeakToPeakValue(src_data2,length);             	
				 break;
			default:
				 break;
		}
	}
	if( src_data2 != NULL)
	{
		free(src_data2); 
		src_data2 =NULL;
	}
	
	return  ret_value;
}




































































































































