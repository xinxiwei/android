//Time Domain Analysis

//#include "WALG_Common.h"
#ifndef __WALG_TDA_H__
#define __WALG_TDA_H__

//函数功能：计算得到信号的平均值，也即数学期望值
//参数说明：
//	pTimeWave	- 时域信号的指针，假定不为NULL
//  length		- 时域信号的长度，假定不为0
//返回：信号的直流分量，即数据的平均值
double WALG_TDA_DCValue( double *pTimeWave,  int length);
//double WALG_TDA_DCValue( INT16 *pTimeWave,  int length);

//函数用于获取时域信号的真有效值
//pTimeWave		- 指向时域数据的指针
//length		- 时域数据的长度
//返回： 时域信号的真有效值
double WALG_TDA_RMSValue( double *pTimeWave,  int length);
//double WALG_TDA_RMSValue( INT16 *pTimeWave,  int length);
//函数用于获取真峰峰值
//pData		- 指向时域数据的指针
//length	- 时域数据的长度
//返回： 真峰峰值
double WALG_TDA_TruePeakToPeakValue( double *pData,  int length);
//INT16 WALG_TDA_TruePeakToPeakValue( INT16 *pData,  int length);

//函数用于获取真峰值
//pData		- 指向时域数据的指针
//length	- 时域数据的长度
//返回： 真峰值
double WALG_TDA_TruePeakValue( double *pData,  int length);
//double WALG_TDA_TruePeakValue( INT16 *pData,  int length);

//函数用于获取等效峰峰值
//fTrueEffectValue		- 时域信号的真有效值
//返回： 等效峰峰值
//double WALG_TDA_PeakToPeakValue( double fTrueEffectValue);

//函数用于获取等效峰峰值
//pTimeWave		- 指向时域数据的指针
//length		- 时域数据的长度
//返回： 等效峰峰值
double WALG_TDA_PeakToPeakValue( double *pTimeWave,  int length);
//double WALG_TDA_PeakToPeakValue( INT16 *pTimeWave,  int length);
//函数用于获取等效峰值
//fTrueEffectValue		- 时域信号的真有效值
//返回： 等效峰值
//double WALG_TDA_PeakValue( double fTrueEffectValue);

//函数用于获取等效峰值
//pTimeWave		- 指向时域数据的指针
//length		- 时域数据的长度
//返回： 等效峰值
double WALG_TDA_PeakValue( double *pTimeWave,  int length);
//double WALG_TDA_PeakValue( INT16 *pTimeWave,  int length);
//使用此函数获取时域波形的轮廓、相位、瞬时频率
//pFftCfg		- FFT预配置指针，若预先没有配置，可置为NULL，由函数自行配置
//pSrc		- 原始时域波形，长度为length
//pProfile	- 返回的时域波形轮廓(瞬时幅值)，若指针为NULL，则不返回
//pPhase	- 返回的时域波形轮廓的相位，若指针为NULL，则不返回
//pInstFreq - 返回的时域波形的瞬时频率，若指针为NULL，则不返回
//length	- 时域波形的长度，应为2的整数次幂
//int WALG_TDA_HilbertTransform(void *pFftCfg,  double *pSrc,  SLArrayIndex_t length, double *pProfile, double *pPhase,  double *pInstFreq);

//函数功能：直接求取自相关函数：Auto-Correlation Function，也可由互相关函数求得
//根据自相关函数的定义直接求取
//参数说明：
//	pSrc		- 原始时域数据指针
//	length		- 输入数据长度
//	pDst		- 输出的处相关函数序列，不可与pSrc同地址
void WALG_TDA_AutoCorrelationFunctionD( double *pSrc,  int length, double *pDst);

//函数功能：利用FFT求取自相关函数：Auto-Correlation Function，也可由互相关函数求得
//先由FFT求得自功率谱，再对自功率谱进行逆FFT变换得到自相关函数
//参数说明：
//	pSrc		- 原始时域数据指针
//	length		- 输入数据长度
//	pDst		- 输出的处相关函数序列，可与pSrc同地址
int WALG_TDA_AutoCorrelationFunction( double *pSrc,  int length, double *pDst);

//函数功能：求取自相关系数：Auto Correlation Coefficient
//参数说明：
//	pSrc	- 原始数据序列
//	length	- 原始数据长度
//	pDst	- 得到的自相关系数指针
void WALG_TDA_AutoCorrelationCoefficient( double *pSrc,  int length, double *pDst);

//函数功能：直接求取互相关函数：Cross Correlation Function
//根据互相关函数的定义直接求取，在pSrc1与pSrc2指向同地址的时候，求得自相关函数
//参数说明：
//	pSrc1		- 第一列原始时域数据指针
//	pSrc2		- 第二列原始时域数据指针
//	length		- 输入数据长度
//	pDst		- 输出的处相关函数序列，不可与pSrc同地址
void WALG_TDA_CrossCorrelationFunctionD( double *pSrc1,  double *pSrc2,  int length, double *pDst);

//函数功能：根据FFT求取互相关函数：Cross Correlation Function
//根据互相关函数的定义直接求取，在pSrc1与pSrc2指向同地址的时候，求得自相关函数
//参数说明：
//	pSrc1		- 第一列原始时域数据指针
//	pSrc2		- 第二列原始时域数据指针
//	length		- 输入数据长度
//	pDst		- 输出的处相关函数序列，不可与pSrc同地址
//返回：	处理成功，返回0；否则，返回错误代码
int WALG_TDA_CrossCorrelationFunction( double *pSrc1,  double *pSrc2,  int length, double *pDst);

//函数功能：求取互相关系数：Cross Correlation Coefficient，在pSrc1与pSrc2为
//参数说明：
//	pSrc1	- 原始数据序列1
//	pSrc2	- 原始数据序列2
//	length	- 原始数据长度
//	pDst	- 互相关系数指针
void WALG_TDA_CrossCorrelationCoefficient( double *pSrc1,  double *pSrc2,  int length, double *pDst);

//函数功能：计算时域信号有效值的功能函数，也即均方根值(非方差之根)
//pTimeWave		- 时域信号数据
//length		- 时域信号数据的长度
//假设数据长度length>0，pTimeWave始终不为NULL
//return		- 计算得到的信号有效值
double WALG_TDA_RMSRealValue( double *pTimeWave,  int length);

//函数功能：计算时域信号偏度(歪度)的功能函数
//pTimeWave		- 时域信号数据
//length		- 时域信号数据的长度
//假设数据长度length>0，pTimeWave始终不为NULL
//return		- 计算得到的信号偏度(歪度)
double WALG_TDA_Skew( double *pTimeWave,  int length);

//函数功能：计算时域信号峰值因子的功能函数
//pTimeWave		- 时域信号数据
//length		- 时域信号数据的长度
//假设数据长度length>0，pTimeWave始终不为NULL
//return		- 计算得到的峰值因子
double WALG_TDA_CrestFactor( double *pTimeWave,  int length);

//函数功能：计算时域信号峭度的功能函数
//pTimeWave		- 时域信号数据
//length		- 时域信号数据的长度
//假设数据长度length>0，pTimeWave始终不为NULL
//return		- 计算得到的峭度
double WALG_TDA_Kurtosis( double *pTimeWave,  int length);

//函数功能：获取时域信号的最大值
//参数说明：
//	pData		- 时域信号指针
//	length		- 时域信号长度
//返回：时域信号的最大值
double WALG_TDA_Max( double *pData,  int length);
//INT16 WALG_TDA_Max( INT16 *pData,  int length);
//函数功能：获取时域信号的最小值
//参数说明：
//	pData		- 时域信号指针
//	length		- 时域信号长度
//返回，时域信号的最小值
double WALG_TDA_Min( double *pData,  int length);
//INT16 WALG_TDA_Min( INT16 *pData,  int length);

//函数功能：获取时域信号的峰均比
//参数说明：
//	pData		- 时域信号指针
//	length		- 时域信号长度
//返回，时域信号的峰均比
//double WALG_TDA_PM( double *pData,  int length);

//函数功能：计算时域信号波形指标的功能函数
//pTimeWave		- 时域信号数据
//length		- 时域信号数据的长度
//假设数据长度length>0，pTimeWave始终不为NULL
//return		- 计算得到的信号波形指标
double WALG_TDA_ShapeFactor( double *pTimeWave,  int length);

//函数功能：计算时域信号脉冲指标的功能函数
//pTimeWave		- 时域信号数据
//length		- 时域信号数据的长度
//假设数据长度length>0，pTimeWave始终不为NULL
//return		- 计算得到的信号脉冲指标
double WALG_TDA_ImpulseFactor( double *pTimeWave,  int length);

//函数功能：计算时域信号方根的功能函数
//pTimeWave		- 时域信号数据
//length		- 时域信号数据的长度
//假设数据长度length>0，pTimeWave始终不为NULL
//return		- 计算得到的信号方根
double WALG_TDA_SqrRootValue( double *pTimeWave,  int length);

//函数功能：计算时域信号裕度指标的功能函数
//pTimeWave		- 时域信号数据
//length		- 时域信号数据的长度
//假设数据长度length>0，pTimeWave始终不为NULL
//return		- 计算得到的信号裕度指标
double WALG_TDA_ClearanceFactor( double *pTimeWave,  int length);

//variance: 方差
double WALG_TDA_Variance( double *pSrc,  int length);

//函数功能：绝对值的平均值(平均幅值)
//参数说明：
//	pTimeWave	- 时域信号的指针，假定不为NULL
//  length		- 时域信号的长度，假定不为0
//返回：信号的直流分量，即数据的平均值
double WALG_TDA_AbsDCValue( double *pTimeWave,  int length);



#endif	//__WALG_TDA_H__