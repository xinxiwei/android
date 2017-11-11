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


//�������ܣ�����õ��źŵ�ƽ��ֵ��Ҳ����ѧ����ֵ
 //����˵����
 //	pTimeWave	- ʱ���źŵ�ָ�룬�ٶ���ΪNULL
 //  length		- ʱ���źŵĳ��ȣ��ٶ���Ϊ0
 //���أ��źŵ�ֱ�������������ݵ�ƽ��ֵ
 double WALG_TDA_DCValue(const double *pTimeWave, const int length)//�źŵ�ֱ�������������ݵ�ƽ��ֵ
 {
 	register int i;
 	double sum = 0;
 
 	for (i = 0; i < length; i++)
 	{
 		sum += pTimeWave[i];
 	}
 
 	return (sum / length);
 }
 //�������ܣ�����ʱ���ź���Чֵ�Ĺ��ܺ�����Ҳ��������ֵ��ʵ�ʲ���ʱ
//			�����˹�0�������õ�ʵ����ֵΪ�����
//pTimeWave		- ʱ���ź�����
//length		- ʱ���ź����ݵĳ���
//�������ݳ���length>0��pTimeWaveʼ�ղ�ΪNULL
//return		- ����õ����ź���Чֵ
double WALG_TDA_RMSValue(const double *pTimeWave, const int length) //����õ����ź���Чֵ
{
	register int i;
	double sum = 0;
	double temp, dc;

	//��0������
	dc = WALG_TDA_DCValue( pTimeWave, length);	

	//��ȡ��Чֵ
	for(i=0; i<length; i++)
	{
		temp = pTimeWave[i]-dc;
		sum += temp*temp;
	}

	return sqrt(sum/length);
}

//�������ܣ�����ʱ���ź���Чֵ�Ĺ��ܺ�����Ҳ��������ֵ(�Ƿ���֮��)
//pTimeWave		- ʱ���ź�����
//length		- ʱ���ź����ݵĳ���
//�������ݳ���length>0��pTimeWaveʼ�ղ�ΪNULL
//return		- ����õ����ź���Чֵ
double WALG_TDA_RMSRealValue(const double *pTimeWave, const int length)
{
	register int i;
	double sum = 0;

	//��ȡ��Чֵ
	for(i=0; i<length; i++)
	{
		sum += pTimeWave[i]*pTimeWave[i];
	}

	return sqrt(sum/length);
} 

//�������ܣ�����ʱ���ź�ƫ��(���)�Ĺ��ܺ���
//pTimeWave		- ʱ���ź�����
//length		- ʱ���ź����ݵĳ���
//�������ݳ���length>0��pTimeWaveʼ�ղ�ΪNULL
//return		- ����õ����ź�ƫ��(���)
double WALG_TDA_Skew(const double *pTimeWave, const int length) //����õ����ź�ƫ��(���)
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
	#else   //ƫ�������ʣ����㷨
	double dbMean = 0.0;
	double dbRMS  = 0.0;
	double dbRMS_Pwr3 = 0.0;
	double dbMid  = 0.0;
	double dbKurtosis = 0.0;
	/*S2:�����ֵ����Чֵ*/
	dbMean = WALG_TDA_DCValue(pTimeWave,length);
	dbRMS  = WALG_TDA_RMSValue(pTimeWave,length);
	/*S3:RMS^4*/
	dbRMS_Pwr3 = dbRMS*dbRMS*dbRMS;
	if (dbRMS_Pwr3 == 0.0)
	{
		return dbKurtosis;
	}
	/*S4:�����Ͷ�*/
	/*S4-1:����SGM(Xi - Xmean)^3*/
	int iXIndex=0;
	for (iXIndex = 0 ; iXIndex <  length ; iXIndex ++)
	{
		dbMid = pTimeWave[iXIndex] - dbMean;
		dbMid = dbMid*dbMid*dbMid;

		dbKurtosis +=dbMid;
	}
	/*S4-2:���ռ���*/
	dbKurtosis = dbKurtosis/(length*dbRMS_Pwr3);

	return dbKurtosis;	
	#endif
}

//�������ܣ�����ʱ���źŷ����Ĺ��ܺ���
//pTimeWave		- ʱ���ź�����
//length		- ʱ���ź����ݵĳ���
//�������ݳ���length>0��pTimeWaveʼ�ղ�ΪNULL
//return		- ����õ����źŷ���
double WALG_TDA_SqrRootValue(const double *pTimeWave, const int length) //����õ����źŷ���  ,����ԣ������
{
	register int i;
	double sum = 0;
	
	//��ȡ��Чֵ
	for(i=0; i<length; i++)
	{
		sum += sqrt(fabs(pTimeWave[i]));
	}	
	return (sum*sum)/(length*length);
}

//�������ܣ���ȡʱ���źŵ����ֵ
//����˵����
//	pData		- ʱ���ź�ָ��
//	length		- ʱ���źų���
//���أ�ʱ���źŵ����ֵ
double WALG_TDA_Max(const double *pData, const int length) //ʱ���źŵ����ֵ
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

//�������ܣ���ȡʱ���źŵ���Сֵ
//����˵����
//	pData		- ʱ���ź�ָ��
//	length		- ʱ���źų���
//���أ�ʱ���źŵ���Сֵ
double WALG_TDA_Min(const double *pData, const int length) //ʱ���źŵ���Сֵ
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


//�������ܣ�����ʱ���źŷ�ֵ���ӵĹ��ܺ���
//pTimeWave		- ʱ���ź�����
//length		- ʱ���ź����ݵĳ���
//�������ݳ���length>0��pTimeWaveʼ�ղ�ΪNULL
//return		- ����õ��ķ�ֵ����
//Author: Martin.lee 2007��11��28��
double WALG_TDA_CrestFactor(const double *pTimeWave, const int length) //����õ��ķ�ֵ����
{
	/*S1:��������*/
	double dbMax;  //�����ֵ
	double dbRMS;  //����Чֵ
	/*S2:��ȡ���ֵ����Чֵ*/
	dbMax = WALG_TDA_Max(pTimeWave,length);
	dbRMS = WALG_TDA_RMSRealValue(pTimeWave,length);
	/*S3:�����ֵ*/
	if (dbRMS != 0.0)
	{
		return dbMax/dbRMS;
	}
	else
	{
		return 0.0;
	}
}

//�������ܣ�����ʱ���ź��ͶȵĹ��ܺ���
//pTimeWave		- ʱ���ź�����
//length		- ʱ���ź����ݵĳ���
//�������ݳ���length>0��pTimeWaveʼ�ղ�ΪNULL
//return		- ����õ����Ͷ�
//Author: Martin.lee 2007��11��28��
double WALG_TDA_Kurtosis(const double *pTimeWave, const int length) //����õ����Ͷ�
{
	/*S1:��������*/
	double dbMean = 0.0;
	double dbRMS  = 0.0;
	double dbRMS_Pwr4 = 0.0;
	double dbMid  = 0.0;
	double dbKurtosis = 0.0;
	/*S2:�����ֵ����Чֵ*/
	dbMean = WALG_TDA_DCValue(pTimeWave,length);
	dbRMS  = WALG_TDA_RMSValue(pTimeWave,length);
	/*S3:RMS^4*/
	dbRMS_Pwr4 = dbRMS*dbRMS*dbRMS*dbRMS;
	if (dbRMS_Pwr4 == 0.0)
	{
		return dbKurtosis;
	}
	/*S4:�����Ͷ�*/
	/*S4-1:����SGM(Xi - Xmean)^4*/
	int iXIndex=0;
	for (iXIndex = 0 ; iXIndex <  length ; iXIndex ++)
	{
		dbMid = pTimeWave[iXIndex] - dbMean;
		dbMid = dbMid*dbMid*dbMid*dbMid;

		dbKurtosis +=dbMid;
	}
	/*S4-2:���ռ���*/
	dbKurtosis = dbKurtosis/(length*dbRMS_Pwr4);

	return dbKurtosis;
}
//�������ܣ�����ֵ��ƽ��ֵ(ƽ����ֵ)
//����˵����
//	pTimeWave	- ʱ���źŵ�ָ�룬�ٶ���ΪNULL
//  length		- ʱ���źŵĳ��ȣ��ٶ���Ϊ0
//���أ��źŵ�ֱ�������������ݵ�ƽ��ֵ
double WALG_TDA_AbsDCValue(const double *pTimeWave, const int length)// ƽ����ֵ
{
	register int i;
	double sum = 0;
	
	for(i=0; i<length; i++)
	{
		sum += fabs(pTimeWave[i]);
	}
	
	return (sum/length);
}

//�������ܣ�����ʱ���źŲ���ָ��Ĺ��ܺ���
//pTimeWave		- ʱ���ź�����
//length		- ʱ���ź����ݵĳ���
//�������ݳ���length>0��pTimeWaveʼ�ղ�ΪNULL
//return		- ����õ����źŲ���ָ��
double WALG_TDA_ShapeFactor(const double *pTimeWave, const int length) //����õ����źŲ���ָ��
{
	double dbMSR = WALG_TDA_RMSRealValue(pTimeWave,length);
    #if 0
	double dbAbsMean = fabs(WALG_TDA_DCValue(pTimeWave,length));
	#else
	double dbAbsMean = fabs(WALG_TDA_AbsDCValue(pTimeWave,length));
	#endif

	return (dbAbsMean==0.0)?SIGLIB_DATA_MAX:(dbMSR/dbAbsMean);
}

//�������ܣ�����ʱ���ź�����ָ��Ĺ��ܺ���
//pTimeWave		- ʱ���ź�����
//length		- ʱ���ź����ݵĳ���
//�������ݳ���length>0��pTimeWaveʼ�ղ�ΪNULL
//return		- ����õ����ź�����ָ��
double WALG_TDA_ImpulseFactor(const double *pTimeWave, const int length) //����õ����ź�����ָ��
{
	double dbTPK = WALG_TDA_Max(pTimeWave,length);
    #if 0
	double dbAbsMean = fabs(WALG_TDA_DCValue(pTimeWave,length));
	#else
	double dbAbsMean = fabs(WALG_TDA_AbsDCValue(pTimeWave,length));	
	#endif

	return (dbAbsMean==0.0)?SIGLIB_DATA_MAX:(dbTPK/dbAbsMean);
}

//�������ܣ�����ʱ���ź�ԣ��ָ��Ĺ��ܺ���
//pTimeWave		- ʱ���ź�����
//length		- ʱ���ź����ݵĳ���
//�������ݳ���length>0��pTimeWaveʼ�ղ�ΪNULL
//return		- ����õ����ź�ԣ��ָ��
double WALG_TDA_ClearanceFactor(const double *pTimeWave, const int length) //����õ����ź�ԣ��ָ��
{
	double dbTPK = WALG_TDA_Max(pTimeWave,length);

	double dbSqrRoot = WALG_TDA_SqrRootValue(pTimeWave,length);

	return (dbTPK/dbSqrRoot);
}



//�������ܣ�return True Peak-to-Peak Value����ȡʱ���źŵ�����ֵ
//����˵����
//	pData		- ʱ���ź�ָ��
//	length		- ʱ���ź�ָ��
//���أ�����ֵ
double WALG_TDA_TruePeakToPeakValue(const double *pData, const int length) //����ֵ
{
	double max, min;
	max = WALG_TDA_Max(pData, length);
	min = WALG_TDA_Min(pData, length);
	return (max - min);
}
//�������ܣ�return True eak Value����ȡʱ���źŵ����ֵ
//����˵����
//	pData		- ʱ���ź�ָ��
//	length		- ʱ���ź�ָ��
//���أ����ֵ
double WALG_TDA_TruePeakValue(const double *pData, const int length)
{
	return (WALG_TDA_TruePeakToPeakValue(pData, length)/2.0);
}

//variance: ����  = ��׼���ƽ�� �����ֳ��� ��׼����ֳ������ֳֵ�ƽ��
double WALG_TDA_Variance(const double *pSrc, const int length)  //����
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



//�������ܣ���ȡʱ���źŵķ����
//����˵����
//	pData		- ʱ���ź�ָ��
//	length		- ʱ���źų���
//���أ�ʱ���źŵķ����
double WALG_TDA_PM(const double *pData, const int length)
{
	double dbPP;
	double dbMean;

	dbPP = WALG_TDA_TruePeakToPeakValue(pData,length);
	dbMean = WALG_TDA_DCValue(pData,length);
	return dbPP/dbMean;
}