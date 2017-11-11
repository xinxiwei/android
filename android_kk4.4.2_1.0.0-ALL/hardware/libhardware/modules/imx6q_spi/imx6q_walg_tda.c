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


#define	SIGLIB_DATA_MAX				((double)1.0e30)			/* Maximum realistic sample value */


//函数功能：计算得到信号的平均值，也即数学期望值
 //参数说明：
 //	pTimeWave	- 时域信号的指针，假定不为NULL
 //  length		- 时域信号的长度，假定不为0
 //返回：信号的直流分量，即数据的平均值
 double WALG_TDA_DCValue(const double *pTimeWave, const int length)//信号的直流分量，即数据的平均值
 {
 	register int i;
 	double sum = 0;
 
 	for (i = 0; i < length; i++)
 	{
 		sum += pTimeWave[i];
 	}
 
 	return (sum / length);
 }
 //函数功能：计算时域信号有效值的功能函数，也即均方根值，实际操作时
//			进行了归0化处理，得到实际数值为方差根
//pTimeWave		- 时域信号数据
//length		- 时域信号数据的长度
//假设数据长度length>0，pTimeWave始终不为NULL
//return		- 计算得到的信号有效值
double WALG_TDA_RMSValue(const double *pTimeWave, const int length) //计算得到的信号有效值
{
	register int i;
	double sum = 0;
	double temp, dc;

	//归0化处理
	dc = WALG_TDA_DCValue( pTimeWave, length);	

	//求取有效值
	for(i=0; i<length; i++)
	{
		temp = pTimeWave[i]-dc;
		sum += temp*temp;
	}

	return sqrt(sum/length);
}

//函数功能：计算时域信号有效值的功能函数，也即均方根值(非方差之根)
//pTimeWave		- 时域信号数据
//length		- 时域信号数据的长度
//假设数据长度length>0，pTimeWave始终不为NULL
//return		- 计算得到的信号有效值
double WALG_TDA_RMSRealValue(const double *pTimeWave, const int length)
{
	register int i;
	double sum = 0;

	//求取有效值
	for(i=0; i<length; i++)
	{
		sum += pTimeWave[i]*pTimeWave[i];
	}

	return sqrt(sum/length);
} 

//函数功能：计算时域信号偏度(歪度)的功能函数
//pTimeWave		- 时域信号数据
//length		- 时域信号数据的长度
//假设数据长度length>0，pTimeWave始终不为NULL
//return		- 计算得到的信号偏度(歪度)
double WALG_TDA_Skew(const double *pTimeWave, const int length) //计算得到的信号偏度(歪度)
{
	#if 0
	double dbAbsMean = fabs(WALG_TDA_DCValue(pTimeWave,length));

	double dbMid = 0.0;
	double sum   = 0.0;
	int i=0;
	
	for(i=0; i<length; i++)
	{
		dbMid = (pTimeWave[i]-dbAbsMean);
		sum += dbMid*dbMid*dbMid;
	}
	return (sum/length);
	#else   //偏度有疑问，改算法
	double dbMean = 0.0;
	double dbRMS  = 0.0;
	double dbRMS_Pwr3 = 0.0;
	double dbMid  = 0.0;
	double dbKurtosis = 0.0;
	/*S2:计算均值与有效值*/
	dbMean = WALG_TDA_DCValue(pTimeWave,length);
	dbRMS  = WALG_TDA_RMSValue(pTimeWave,length);
	/*S3:RMS^4*/
	dbRMS_Pwr3 = dbRMS*dbRMS*dbRMS;
	if (dbRMS_Pwr3 == 0.0)
	{
		return dbKurtosis;
	}
	/*S4:计算峭度*/
	/*S4-1:计算SGM(Xi - Xmean)^3*/
	int iXIndex=0;
	for (iXIndex = 0 ; iXIndex <  length ; iXIndex ++)
	{
		dbMid = pTimeWave[iXIndex] - dbMean;
		dbMid = dbMid*dbMid*dbMid;

		dbKurtosis +=dbMid;
	}
	/*S4-2:最终计算*/
	dbKurtosis = dbKurtosis/(length*dbRMS_Pwr3);

	return dbKurtosis;	
	#endif
}

//函数功能：计算时域信号方根的功能函数
//pTimeWave		- 时域信号数据
//length		- 时域信号数据的长度
//假设数据长度length>0，pTimeWave始终不为NULL
//return		- 计算得到的信号方根
double WALG_TDA_SqrRootValue(const double *pTimeWave, const int length) //计算得到的信号方根  ,用于裕度因子
{
	register int i;
	double sum = 0;
	
	//求取有效值
	for(i=0; i<length; i++)
	{
		sum += sqrt(fabs(pTimeWave[i]));
	}	
	return (sum*sum)/(length*length);
}

//函数功能：获取时域信号的最大值
//参数说明：
//	pData		- 时域信号指针
//	length		- 时域信号长度
//返回：时域信号的最大值
double WALG_TDA_Max(const double *pData, const int length) //时域信号的最大值
{
	register int i;
	double max=0.0;
	max = pData[0];
	for(i=1; i<length; i++)
	{
		if( max < pData[i])
			max = pData[i];
	}
	return max;
}

//函数功能：获取时域信号的最小值
//参数说明：
//	pData		- 时域信号指针
//	length		- 时域信号长度
//返回，时域信号的最小值
double WALG_TDA_Min(const double *pData, const int length) //时域信号的最小值
{
	register int i;
	double min=0.0;

	min = pData[0];
	for(i=1; i<length; i++)
	{
		if( min > pData[i])
			min = pData[i];
	}
	return min;
}


//函数功能：计算时域信号峰值因子的功能函数
//pTimeWave		- 时域信号数据
//length		- 时域信号数据的长度
//假设数据长度length>0，pTimeWave始终不为NULL
//return		- 计算得到的峰值因子
//Author: Martin.lee 2007年11月28日
double WALG_TDA_CrestFactor(const double *pTimeWave, const int length) //计算得到的峰值因子
{
	/*S1:变量申明*/
	double dbMax;  //←真峰值
	double dbRMS;  //←有效值
	/*S2:获取真峰值与有效值*/
	dbMax = WALG_TDA_Max(pTimeWave,length);
	dbRMS = WALG_TDA_RMSRealValue(pTimeWave,length);
	/*S3:计算峰值*/
	if (dbRMS != 0.0)
	{
		return dbMax/dbRMS;
	}
	else
	{
		return 0.0;
	}
}

//函数功能：计算时域信号峭度的功能函数
//pTimeWave		- 时域信号数据
//length		- 时域信号数据的长度
//假设数据长度length>0，pTimeWave始终不为NULL
//return		- 计算得到的峭度
//Author: Martin.lee 2007年11月28日
double WALG_TDA_Kurtosis(const double *pTimeWave, const int length) //计算得到的峭度
{
	/*S1:变量申明*/
	double dbMean = 0.0;
	double dbRMS  = 0.0;
	double dbRMS_Pwr4 = 0.0;
	double dbMid  = 0.0;
	double dbKurtosis = 0.0;
	/*S2:计算均值与有效值*/
	dbMean = WALG_TDA_DCValue(pTimeWave,length);
	dbRMS  = WALG_TDA_RMSValue(pTimeWave,length);
	/*S3:RMS^4*/
	dbRMS_Pwr4 = dbRMS*dbRMS*dbRMS*dbRMS;
	if (dbRMS_Pwr4 == 0.0)
	{
		return dbKurtosis;
	}
	/*S4:计算峭度*/
	/*S4-1:计算SGM(Xi - Xmean)^4*/
	int iXIndex=0;
	for (iXIndex = 0 ; iXIndex <  length ; iXIndex ++)
	{
		dbMid = pTimeWave[iXIndex] - dbMean;
		dbMid = dbMid*dbMid*dbMid*dbMid;

		dbKurtosis +=dbMid;
	}
	/*S4-2:最终计算*/
	dbKurtosis = dbKurtosis/(length*dbRMS_Pwr4);

	return dbKurtosis;
}
//函数功能：绝对值的平均值(平均幅值)
//参数说明：
//	pTimeWave	- 时域信号的指针，假定不为NULL
//  length		- 时域信号的长度，假定不为0
//返回：信号的直流分量，即数据的平均值
double WALG_TDA_AbsDCValue(const double *pTimeWave, const int length)// 平均幅值
{
	register int i;
	double sum = 0;
	
	for(i=0; i<length; i++)
	{
		sum += fabs(pTimeWave[i]);
	}
	
	return (sum/length);
}

//函数功能：计算时域信号波形指标的功能函数
//pTimeWave		- 时域信号数据
//length		- 时域信号数据的长度
//假设数据长度length>0，pTimeWave始终不为NULL
//return		- 计算得到的信号波形指标
double WALG_TDA_ShapeFactor(const double *pTimeWave, const int length) //计算得到的信号波形指标
{
	double dbMSR = WALG_TDA_RMSRealValue(pTimeWave,length);
    #if 0
	double dbAbsMean = fabs(WALG_TDA_DCValue(pTimeWave,length));
	#else
	double dbAbsMean = fabs(WALG_TDA_AbsDCValue(pTimeWave,length));
	#endif

	return (dbAbsMean==0.0)?SIGLIB_DATA_MAX:(dbMSR/dbAbsMean);
}

//函数功能：计算时域信号脉冲指标的功能函数
//pTimeWave		- 时域信号数据
//length		- 时域信号数据的长度
//假设数据长度length>0，pTimeWave始终不为NULL
//return		- 计算得到的信号脉冲指标
double WALG_TDA_ImpulseFactor(const double *pTimeWave, const int length) //计算得到的信号脉冲指标
{
	double dbTPK = WALG_TDA_Max(pTimeWave,length);
    #if 0
	double dbAbsMean = fabs(WALG_TDA_DCValue(pTimeWave,length));
	#else
	double dbAbsMean = fabs(WALG_TDA_AbsDCValue(pTimeWave,length));	
	#endif

	return (dbAbsMean==0.0)?SIGLIB_DATA_MAX:(dbTPK/dbAbsMean);
}

//函数功能：计算时域信号裕度指标的功能函数
//pTimeWave		- 时域信号数据
//length		- 时域信号数据的长度
//假设数据长度length>0，pTimeWave始终不为NULL
//return		- 计算得到的信号裕度指标
double WALG_TDA_ClearanceFactor(const double *pTimeWave, const int length) //计算得到的信号裕度指标
{
	double dbTPK = WALG_TDA_Max(pTimeWave,length);

	double dbSqrRoot = WALG_TDA_SqrRootValue(pTimeWave,length);

	return (dbTPK/dbSqrRoot);
}



//函数功能：return True Peak-to-Peak Value，获取时域信号的真峰峰值
//参数说明：
//	pData		- 时域信号指针
//	length		- 时域信号指针
//返回，真峰峰值
double WALG_TDA_TruePeakToPeakValue(const double *pData, const int length) //真峰峰值
{
	double max, min;
	max = WALG_TDA_Max(pData, length);
	min = WALG_TDA_Min(pData, length);
	return (max - min);
}
//函数功能：return True eak Value，获取时域信号的真峰值
//参数说明：
//	pData		- 时域信号指针
//	length		- 时域信号指针
//返回，真峰值
double WALG_TDA_TruePeakValue(const double *pData, const int length)
{
	return (WALG_TDA_TruePeakToPeakValue(pData, length)/2.0);
}

//variance: 方差  = 标准差的平方 ，老手持是 标准差，新手持是老手持的平方
double WALG_TDA_Variance(const double *pSrc, const int length)  //方差
{
	double dc;
	double sum;
	int i;

	dc = WALG_TDA_DCValue( pSrc, length);

	sum = 0;
	for( i=0; i<length; i++)
	{
		sum += (pSrc[i]-dc) *(pSrc[i]-dc);
	}

	return (sum/length);
}



//函数功能：获取时域信号的峰均比
//参数说明：
//	pData		- 时域信号指针
//	length		- 时域信号长度
//返回，时域信号的峰均比
double WALG_TDA_PM(const double *pData, const int length)
{
	double dbPP;
	double dbMean;

	dbPP = WALG_TDA_TruePeakToPeakValue(pData,length);
	dbMean = WALG_TDA_DCValue(pData,length);
	return dbPP/dbMean;
}