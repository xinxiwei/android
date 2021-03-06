﻿#include <stdio.h>
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


//////////////////////
#define	SIGLIB_DO_BIT_REVERSAL		1			// Set to '1' if bit reversal is required 位倒序
#define	SIGLIB_FAST_BIT_REVERSAL	1			// Set to '1' for fast( look up table ) bit reversal
#define SDS_Sin( a )		(( float )sin(( double )a ) )
#define SDS_Swap( a , b ) 	{ register float SIGLIB_Tmp =( float )a; a = b; b = SIGLIB_Tmp; }
#define	SIGLIB_ZERO		(( float )0.0 )/* Zero */
#define	SIGLIB_TWO		(( float )2.0 )
#define SIGLIB_PI		(( float )3.14159265358979323846264338327950288419716939937510 )/* Pi */
#define SIGLIB_TWO_PI	( SIGLIB_TWO * SIGLIB_PI )	/* 2.0 * Pi */


long  data_length = 0;//数组元素个数
long  Log2Size = 0;

//////////////////////
void sda_index_bit_reverse_reorder( long Src[] , 	long Dst[] , 	const long nFFTSize )
{
	register long i ,  j ,  k ,  Temp;
	/* Reorder scrambled data */
	for( j = 0 ,  i = 0; j < nFFTSize; j++ )
	{
		if( j < i )
		{
			/* Use temporary variable so that function can work in-place */
			Temp = Src[i];
			Dst[i] = Src[j];
			Dst[j] = Temp;
		}
		/* Copy data if no swapping */
		else if( j == i )
		{
			Dst[i] = Src[i];
		}

		k = nFFTSize >> 1;
		while(( k <= i ) &&( k >= 1 ) )
		{
			i -= k;
			k >>= 1;
		}
		i += k;
	}
}

long sif_fft( float* pFFTCoeffs , 	long* pBitReverseAddressTable , long mFFTSize )
{
	register long i;

	/* Generate Sine and Cos tables */
	for( i = 0; i <(( 3 * mFFTSize ) >> 2 ); i++ )
	{
		*pFFTCoeffs++ = SDS_Sin(((( float )i ) * SIGLIB_TWO_PI ) /(( float )mFFTSize ) );
	}

#if SIGLIB_FAST_BIT_REVERSAL

	for( i = 0; i < mFFTSize; i++ )
	{
		*pBitReverseAddressTable++ = i;
	}

	pBitReverseAddressTable -= mFFTSize;

	/* Swap look-up table data */
	sda_index_bit_reverse_reorder( pBitReverseAddressTable ,  pBitReverseAddressTable ,  mFFTSize );
#endif

	return 0;
}

void sda_rfft( float RealData[] , 	float ImagData[] , const float* pFFTCoeffs ,
const long* pBitReverseAddressTable , const long pFFTSize , const long mLog2Size )
{
	register const float* pFFTSineCoeffs;
	register const float* pFFTCosineCoeffs;
	register long i ,  j ,  k ,  Stride ,  BflyCounter ,  g ,  h ,  HalfFFTSize;
	long Angle ,  AngleInc;	/* Angle step thro sin & cos tables */
	register float RealTemp ,  ImagTemp ,  Cos ,  Sin;

	pFFTSineCoeffs = pFFTCoeffs;
	pFFTCosineCoeffs = pFFTCoeffs +( pFFTSize >> 2 );

	HalfFFTSize = pFFTSize >> 1;

	Stride = j = HalfFFTSize;
	Angle = h = 0;

	/* First stage */
	for( BflyCounter = 0; BflyCounter < Stride; BflyCounter++ )
	{
		RealTemp = RealData[h] - RealData[j];
		RealData[h] = RealData[h] + RealData[j];
		/* Clear imaginary part */
		ImagData[h] = SIGLIB_ZERO;
		RealData[j] = pFFTCosineCoeffs[Angle] * RealTemp;
		ImagData[j] = -pFFTSineCoeffs[Angle] * RealTemp;
		Angle++;
		h++;
		j++;
	}

	AngleInc = 2;
	/* Middle stages */
	for( i = 1; i <( mLog2Size - 1 ); i++ )
	{
		k = Stride;
		Stride >>= 1;
		Angle = 0;
		for( BflyCounter = 0; BflyCounter < Stride; BflyCounter++ )
		{
			Cos = pFFTCosineCoeffs[Angle];
			Sin = pFFTSineCoeffs[Angle];
			Angle += AngleInc;

			h = BflyCounter;
			j = h + Stride;

			for( g = k; g <= pFFTSize; g += k ,  h += k ,  j += k )
			{
				RealTemp = RealData[h] - RealData[j];
				ImagTemp = ImagData[h] - ImagData[j];
				RealData[h] = RealData[h] + RealData[j];
				ImagData[h] = ImagData[h] + ImagData[j];
				RealData[j] = Cos * RealTemp + Sin * ImagTemp;
				ImagData[j] = Cos * ImagTemp - Sin * RealTemp;
			}
		}
		AngleInc <<= 1;
	}

	/* Final stage */
	for( h = 0 ,  j = 1; h < pFFTSize; h += 2 ,  j += 2 )
	{
		RealTemp = RealData[h] - RealData[j];
		ImagTemp = ImagData[h] - ImagData[j];
		RealData[h] = RealData[h] + RealData[j];
		ImagData[h] = ImagData[h] + ImagData[j];
		/* Cos = 1 ,  sin = 0 */
		RealData[j] = RealTemp;
		ImagData[j] = ImagTemp;
	}

#if SIGLIB_DO_BIT_REVERSAL
#if SIGLIB_FAST_BIT_REVERSAL
	/* Reorder scrambled data - fast mode uses more memory */
	for( i = 1; i <( pFFTSize-1 ); i++ )
	{
		/* Only swap if necessary */
		if( i <( j = *( pBitReverseAddressTable+i ) ) )
		{
			SDS_Swap( RealData[i] ,  RealData[j] );
			SDS_Swap( ImagData[i] ,  ImagData[j] );
		}
	}
#else
	/* Reorder scrambled data - slow mode uses less memory */
	for( j = 0 ,  i = 0; j < pFFTSize; j++ )
	{
		if( j < i )
		{
			SDS_Swap( RealData[i] ,  RealData[j] );
			SDS_Swap( ImagData[i] ,  ImagData[j] );
		}

		k = HalfFFTSize;
		while(( k <= i ) &&( k >= 1 ) )
		{
			i -= k;
			k >>= 1;
		}
		i += k;
	}
#endif
#endif
}


/********************************
 *函数名：swap
 *作用：交换两个数的值
 *参数：交换的两个数
 *返回值：无
 ********************************/
void swap(float *a, float *b)
{
    float temp;
    temp = *a;
    *a = *b;
    *b = temp;
    return ;
}

/************************************
 *函数名：quicksort
 *作用：快速排序算法
 *参数：
 *返回值：无
 ************************************/
float* quicksort(float *array, int maxlen, int begin, int end)
{
    int i, j;

    if(begin < end)
    {
        i = begin + 1;  // 将array[begin]作为基准数，因此从array[begin+1]开始与基准数比较！
        j = end;        // array[end]是数组的最后一位

        while(i < j)
        {
            if(array[i] > array[begin])  // 如果比较的数组元素大于基准数，则交换位置。
            {
                swap(&array[i], &array[j]);  // 交换两个数
                j--;
            }
            else
            {
                i++;  // 将数组向后移一位，继续与基准数比较。
            }
        }

        /* 跳出while循环后，i = j。
         * 此时数组被分割成两个部分  -->  array[begin+1] ~ array[i-1] < array[begin]
         *                           -->  array[i+1] ~ array[end] > array[begin]
         * 这个时候将数组array分成两个部分，再将array[i]与array[begin]进行比较，决定array[i]的位置。
         * 最后将array[i]与array[begin]交换，进行两个分割部分的排序！以此类推，直到最后i = j不满足条件就退出！
         */

        if(array[i] >= array[begin])  // 这里必须要取等“>=”，否则数组元素由相同的值时，会出现错误！
        {
            i--;
        }

        swap(&array[begin], &array[i]);  // 交换array[i]与array[begin]

        quicksort(array, maxlen, begin, i);
        quicksort(array, maxlen, j, end);
    }
    return array;
}

/*
输入：经过FFT变化后的数据
输出：单个周期内的点数
步骤：
1.对FFT数据求出前3个最大值，分别对应的index值
2.对三个index值，求最小值
3.波形长度/index  = 周期内的点数
*/
int get_period_point( float* p_pfftData ) //通过傅立叶得到周期内的点数
{
   int i = 0, j = 0 , m = 0;
   float max_value1=0.0;
   float max_value2=0.0;
   float max_value3=0.0;

   int max_value1_index=0;
   int max_value2_index=0;
   int max_value3_index=0;

   int max_fir_flag =0;
   int max_sec_flag =0;
   int max_thr_flag =0;
   if(p_pfftData == NULL)
   {
       return 0;
   }
   float tmp_data =0.0;
   int period_point = 0;  //每个周期对应的多少个点数
   float *p_backupFftData = NULL;
   p_backupFftData =( float * )malloc(  ((data_length/2)+1) * sizeof( float ) ); //只取前半部分数据计算
   if( p_backupFftData == NULL )
   {
		LOGD( "p_backupFftData 分配内存失败！" );
		return 0;
   }
   memset( p_backupFftData ,  0 ,  ((data_length/2)+1)*sizeof( float ));
   memcpy( &p_backupFftData[1] ,  p_pfftData ,  (data_length/2) *sizeof(float) ); //备份一份FFT变换后的数据

#if 0 //冒泡排序, 比较耗时
   for( j=0;j<(data_length/2);j++ )
   {
		for( i = 0;i< (data_length/2)-1-j;i++ )
		{
			if( p_pfftData[i] < p_pfftData[i+1] ) // fft数据由大到小时改为< ,   由小到大>
			{
				tmp_data = p_pfftData[i];
				p_pfftData[i] = p_pfftData[i+1];
				p_pfftData[i+1] = tmp_data;
			}
		}
   }
   max_value1 = p_pfftData[0];
   max_value2 = p_pfftData[1];
   max_value3 = p_pfftData[2];
#else //快速排序
   quicksort(p_pfftData, data_length/2, 0 ,(data_length/2) -1);

   max_value1 = p_pfftData[(data_length/2) -1];
   max_value2 = p_pfftData[(data_length/2) -2];
   max_value3 = p_pfftData[(data_length/2) -3];
#endif

   LOGD( "傅立叶数据前三个最大值 max_value = %f ,   %f ,   %f ,  \n" , max_value1 , max_value2 , max_value3 ); //fft数据的前3个最大值

   for( m=1;m< ((data_length/2)+1);m++ )//备份数据 和排序后数据对比，求出对应的index
   {
		if( p_backupFftData[m] == max_value1 )
		{
			max_value1_index=m;
			max_fir_flag =1;
		}
		if( p_backupFftData[m] == max_value2 )
		{
			max_value2_index=m;
			max_sec_flag =1;
		}
		if( p_backupFftData[m] == max_value3 )
		{
			max_value3_index=m;
			max_thr_flag =1;
		}
		if(max_fir_flag && max_sec_flag && max_thr_flag) //找到index后提前跳出循环
			break;
   }

	//LOGD( "傅立叶数据前三个最大值对应的index, max_value1_index= %d ,   max_value2_index= %d ,  max_value3_index= %d\n" , max_value1_index , max_value2_index , max_value3_index ); //3个最大值 分别对应的index

	int min_index = max_value1_index;
	if( max_value2_index < min_index )   min_index = max_value2_index;
    if( max_value3_index < min_index )   min_index = max_value3_index;   //求前三个fft数据最大值对应的index 最小值

	if( min_index !=0 )
	{
		period_point = data_length/min_index; // 计算一个周期内的点数
    }

	if( p_backupFftData != NULL)
	{
		free( p_backupFftData );
		p_backupFftData =NULL;
	}
	//LOGD( "周期点数period_point = %d " , period_point );
	return period_point;
}

float right_value[3] ={0.0}; //0位存最大值和，1位存最小值总和，2位存多少个小周期个数
float src_min_value=0.0; //原始数据最小值
int src_min_index=0;  //原始数据最小值 index
/*功能描述：以原始数据最小值为起点，以nperiod_point 点分段，分别求出最大最小值 并平均，注意最后不够周期的余数也要计算在内 */
float* single_period_right_max_min( float* p_RBufSrc ,  int r_period_point )
{
	int i=0,k=0;
	float right_max_sum = 0.0;  //多个小周期段右边最大值总和
	float right_min_sum = 0.0;  //多个小周期段右边最小值总和
	float single_right_max_value=0.0;
    float single_right_min_value=0.0;
	if(p_RBufSrc == NULL)
    {
        return NULL;
    }
    int single_max_index=0;  //每个小周期内最大最小值 对应的index
    int single_min_index=0;
	int right_period_count =( data_length - src_min_index )/r_period_point; //原始数据共可以分多少个小周期
	int right_yu_count =( data_length - src_min_index )%r_period_point; //余数个数

	if(  right_period_count >=1 && right_yu_count ==0 ) //表示至少一个周期且没有余数
	{
		  LOGD( "right 1111111 表示至少一个周期且没有余数\n" );
		 // LOGD( "src_min_index = %d ,  r_period_point= %d ,  right_period_count = %d ,   right_yu_count= %d\n" , src_min_index , r_period_point , right_period_count , right_yu_count );
		  for( i =0;i< right_period_count;i++ )
		  {
			  single_max_index = r_period_point*i+src_min_index;
			  single_min_index = r_period_point*i+src_min_index;
			  for( k = single_max_index;k<( i+1 )*r_period_point+src_min_index;k++ )
			  {
				   if( p_RBufSrc[single_max_index]<=p_RBufSrc[k] ){
						single_max_index=k;
						single_right_max_value=p_RBufSrc[single_max_index];
				   }
				   if( p_RBufSrc[single_min_index]>=p_RBufSrc[k] ){
						single_min_index=k;
						single_right_min_value=p_RBufSrc[single_min_index];
				   }
			  }
			  //LOGD( "=====single_right_max_value is = %f ， single_right_min_value= %f\n" , single_right_max_value , single_right_min_value );
			  right_max_sum += single_right_max_value;
			  right_min_sum += single_right_min_value;

			  right_value[0] =right_max_sum;
	          right_value[1] =right_min_sum;
			  right_value[2] =right_period_count;
		  }
          //LOGD( "=====right___可分的周期个数 = %d  , right_max_sum = %f ,   right_min_sum = %f\n" , (int)right_value[2] , right_value[0] , right_value[1] );
	  }

	if(  right_period_count >=1 && right_yu_count !=0 ) //表示至少一个周期 , 但还有余数
	{
		  LOGD( "right 222222 表示至少一个周期 , 有余数\n" );
		  //LOGD( "src_min_index = %d ,  r_period_point= %d ,  right_period_count = %d ,   right_yu_count= %d\n" , src_min_index , r_period_point , right_period_count , right_yu_count );
		  for( i =0;i< right_period_count;i++ )
		  {
			  single_max_index = r_period_point*i+src_min_index;
			  single_min_index = r_period_point*i+src_min_index;
			  for( k = single_max_index;k<( i+1 )*r_period_point+src_min_index;k++ )
			  {
				   if( p_RBufSrc[single_max_index]<=p_RBufSrc[k] ){
						single_max_index=k;
						single_right_max_value=p_RBufSrc[single_max_index];
				   }
				   if( p_RBufSrc[single_min_index]>=p_RBufSrc[k] ){
						single_min_index=k;
						single_right_min_value=p_RBufSrc[single_min_index];
				   }
			  }
			  //LOGD( "=====single_right_max_value is = %f ， single_right_min_value= %f\n" , single_right_max_value , single_right_min_value );
			  right_max_sum += single_right_max_value;
			  right_min_sum += single_right_min_value;
		  }
		  //LOGD( "=====right___right_max_sum is = %f ,  right_min_sum= %f\n" , right_max_sum , right_min_sum );

		  int yu_max_index=0 , yu_min_index1=0;  //余数内对应的最大最小值 和index
		  float yu_max_value1= p_RBufSrc[data_length -right_yu_count];
		  float yu_min_value1= p_RBufSrc[data_length -right_yu_count];
		  for( k =data_length -right_yu_count ;k< data_length;k++ )
		  {
			   if( yu_max_value1<=p_RBufSrc[k] ){
					yu_max_value1=p_RBufSrc[k];
			   }
			   if( yu_min_value1>=p_RBufSrc[k] ){
					yu_min_value1=p_RBufSrc[k];
			   }
		  }
		   //LOGD( "=====yu_max_value1 is = %f ,  yu_min_value1= %f\n" , yu_max_value1 , yu_min_value1 );
		   right_max_sum += yu_max_value1;
		   right_min_sum += yu_min_value1;
		   //LOGD( "=====right___right_max_sum is = %f ,  right_min_sum = %f\n" , right_max_sum , right_min_sum );
		   right_value[0] =right_max_sum;
	       right_value[1] =right_min_sum;
		   right_value[2] =right_period_count+1;
           //LOGD( "=====right___可分的周期个数 = %d ,  right_max_sum = %f ,  right_min_sum = %f\n" ,(int)right_value[2] , right_value[0] , right_value[1] );
	  }

	if( right_period_count <1 && right_yu_count !=0 ) //表示不够一个周期数，以这些余数计算
	{
		  //LOGD( "right 3333333 表示不够一个周期数，以这些余数计算\n" );
		  //LOGD( "src_min_index = %d ,  r_period_point= %d ,  right_period_count = %d ,   right_yu_count= %d\n" , src_min_index , r_period_point , right_period_count , right_yu_count );
		  int yu_max_index2=0 , yu_min_index2=0;
		  float yu_max_value2= p_RBufSrc[src_min_index];
		  float yu_min_value2= p_RBufSrc[src_min_index];

		  for( k = src_min_index ;k< data_length;k++ )
		  {
			   if( yu_max_value2<=p_RBufSrc[k] ){
					yu_max_value2=p_RBufSrc[k];
			   }
			   if( yu_min_value2>=p_RBufSrc[k] ){
					yu_min_value2=p_RBufSrc[k];
			   }
		  }
		   right_value[0] =yu_max_value2;
	       right_value[1] =yu_min_value2;
           right_value[2] =1;
           //LOGD( "=====right___余数周期数为1 , yu_max_value2 is = %f ,   yu_min_value2 = %f \n" ,  right_value[0] , right_value[1] );
	  }
	  return right_value;
}


float left_value[3]={0.0};//0位存最大值和，1位存最小值总和，2位存多少个小周期个数
/* 功能描述：//以原始数据最小值为起点，以nperiod_point 点分段，分别求出最大最小值 并平均，注意最后不够周期的余数也要计算在内 */
float* single_period_left_max_min( float* p_LBufSrc ,  int nperiod_point )
{
	int i=0, k=0, m=0;
	float left_max_sum = 0.0;  //多个小周期段左边最大值总和
	float left_min_sum = 0.0;  //多个小周期段左边最小值总和
	float single_left_max_value= 0.0;
	float single_left_min_value= 0.0;

	int left_period_count=0 , left_yu_count=0;
	int left_max_index =0 , left_min_index =0;
	if(p_LBufSrc == NULL)
    {
        return NULL;
    }
	left_period_count = src_min_index/nperiod_point;//原始数据共可以分多少个小周期
	left_yu_count = src_min_index%nperiod_point;//余数个数

	if( left_period_count>=1 && left_yu_count ==0  )//表示至少一个周期且没有余数
    {
		 LOGD( "left 1111111 表示至少一个周期且没有余数\n" );
		 //LOGD( "left_src_min_index = %d ,  nperiod_point= %d ,  left_period_count = %d ,  left_yu_count= %d\n" , src_min_index , nperiod_point , left_period_count , left_yu_count );
		 for( m = 0;m< left_period_count; m++ )
         {
             left_max_index= src_min_index-( m*nperiod_point );
             left_min_index= src_min_index-( m*nperiod_point );
		     for(  i=left_max_index; i< src_min_index-( m+1)*nperiod_point ; i--  )
             {

				 if( p_LBufSrc[left_max_index]<=p_LBufSrc[i] ){
						left_max_index= i;
						single_left_max_value = p_LBufSrc[left_max_index];
				 }

				 if( p_LBufSrc[left_min_index]>=p_LBufSrc[i] ){
						left_min_index= i;
						single_left_min_value = p_LBufSrc[left_min_index];
				 }

            }
			 //LOGD( "=====single_left_max_value = %f ,   single_left_min_value= %f\n" , single_left_max_value , single_left_min_value );
			 left_max_sum += single_left_max_value;
             left_min_sum += single_left_min_value;
		 }
		 //LOGD( "=====left___left_max_sum = %f ,   left_min_sum= %f\n" , left_max_sum , left_min_sum );
		 left_value[0] =left_max_sum;
	     left_value[1] =left_min_sum;
		 left_value[2] =left_period_count;
		 //LOGD( "=====left___可分的周期个数 = %d ,  left_max_sum = %f ,  left_min_sum = %f\n" , (int)left_value[2] , left_value[0] , left_value[1] );
	}

	//////////
	if( left_period_count>=1 && left_yu_count !=0  )//表示至少一个周期 ,  有余数
     {
         LOGD( "left 222222 表示至少一个周期 ,  有余数\n" );
		 //LOGD( "left_src_min_index = %d ,  nperiod_point= %d ,  left_period_count = %d ,  left_yu_count= %d\n" , src_min_index , nperiod_point , left_period_count , left_yu_count );
         for( m = 0;m< left_period_count; m++ )
         {
             left_max_index= src_min_index-( m*nperiod_point );
             left_min_index= src_min_index-( m*nperiod_point );
             for(  i=left_max_index; i< src_min_index-( m+1)*nperiod_point ; i--  )
             {

				 if( p_LBufSrc[left_max_index]<=p_LBufSrc[i] ){
						left_max_index= i;
						single_left_max_value = p_LBufSrc[left_max_index];
				 }

				 if( p_LBufSrc[left_min_index]>=p_LBufSrc[i] ){
						left_min_index= i;
						single_left_min_value = p_LBufSrc[left_min_index];
				 }

            }
		    //LOGD( "=====single_left_max_value = %f ,   single_left_min_value= %f\n" , single_left_max_value , single_left_min_value );

		    left_max_sum += single_left_max_value;
		    left_min_sum += single_left_min_value;
         }
		  //LOGD( "=====left___left_max_sum = %f ,   left_min_sum= %f\n" , left_max_sum , left_min_sum );

          int yu_max_index=0 , yu_min_index=0;
          float yu_max1 = p_LBufSrc[0];
          float yu_min1 = p_LBufSrc[0];
          for( k =0 ;k< left_yu_count;k++ )
          {
               if( yu_max1<=p_LBufSrc[k] ){
                    yu_max1=p_LBufSrc[k];
               }
               if( yu_min1>=p_LBufSrc[k] ){
                    yu_min1=p_LBufSrc[k];
               }
          }
          //LOGD( "=====yu_max1 = %f ,   yu_min1 = %f\n" , yu_max1 , yu_min1 );
          left_max_sum += yu_max1;
          left_min_sum += yu_min1;
		  //LOGD( "=====left___left_max_sum = %f ,   left_min_sum = %f\n" , left_max_sum , left_min_sum );
		  left_value[0] =left_max_sum;
	      left_value[1] =left_min_sum;
		  left_value[2] =left_period_count+1;
          //LOGD( "=====left___可分的周期个数 = %d  ,  left_max_sum = %f ,  left_min_sum = %f \n" , (int)left_value[2] , left_value[0] , left_value[1] );
	 }

	////////////

	if( left_period_count <1 &&  left_yu_count!=0 )//表示不够一个周期数，以这些余数计算
    {
		  //LOGD( "left 3333333 表示不够一个周期数，以这些余数计算\n" );
		 // LOGD( "left_src_min_index = %d ,  nperiod_point= %d ,  left_period_count = %d ,  left_yu_count= %d\n" , src_min_index , nperiod_point , left_period_count , left_yu_count );
		  int yu_max_index=0 , yu_min_index=0;
		  float yu_max3 = p_LBufSrc[0];
		  float yu_min3 = p_LBufSrc[0];
		  for( k = 0 ;k< left_yu_count;k++ )
		  {
			   if( yu_max3<=p_LBufSrc[k] ){
					yu_max3=p_LBufSrc[k];
			   }
			   if( yu_min3>=p_LBufSrc[k] ){
					yu_min3=p_LBufSrc[k];
			   }
		  }
		   left_value[0] =yu_max3;
	       left_value[1] =yu_min3;
		   left_value[2] =1;
		   //LOGD( "=====left___可分的周期个数 = 1 ,  yu_max3 = %f ,   yu_min3 = %f" , left_value[0] , left_value[1] );
     }
	 return left_value;
}


float single_period_value[2]={0.0}; //0位表示最大值，1位表示最小值
/* 功能描述：求单个周期内的最大值 ，最小值 */
float* single_period_max_min( float* p_BufSrc ,  int nperiod_point )
{
    if(p_BufSrc == NULL)
    {
        return NULL;
    }
	float *p_rightData = NULL;
    float *p_leftData = NULL;
	p_rightData = single_period_right_max_min( p_BufSrc , nperiod_point );
	p_leftData = single_period_left_max_min( p_BufSrc , nperiod_point );

    if( src_min_index == 0 )
	{
       single_period_value[0] = p_rightData[0]/( int )p_rightData[2];
       single_period_value[1] = p_rightData[1]/( int )p_rightData[2];
	}
	if( src_min_index !=0 )
	{
		single_period_value[0] =( p_rightData[0] + p_leftData[0] )/( int )( p_rightData[2] + p_leftData[2] );
        single_period_value[1] =( p_rightData[1] + p_leftData[1] )/( int )( p_rightData[2] + p_leftData[2] );
	}
    //LOGD( "=====single_period_value[0] is = %f ,   single_period_value[1] is = %f" , single_period_value[0] , single_period_value[1] );
	return single_period_value;
}

/* 功能描述：
1.先求出原始数据最小值及对应的index值
2.将分段平均后的最大最小值及小段数据返回 */
float* get_max_min(  float* p_bufSrc ,  float* p_mretValue ,  int single_period_point )
{
    int i=0;
    int send_num = 0;  //向JNI返回数组的大小
    float *p_Data = NULL;
    float ave_max_value = 0.0; //平均最大值
    float ave_min_value = 0.0; //平均最小值
	if(p_bufSrc == NULL|| p_mretValue == NULL)
    {
        return NULL;
    }
	for( i = 0;i < data_length ;i++ )
	{
		 if( p_bufSrc[src_min_index]>=p_bufSrc[i] ){
            src_min_index=i;
            src_min_value=p_bufSrc[src_min_index];
         }
	}
	//LOGD( "src_min_index is = %d ,  src_min_value = %f\n" , src_min_index , src_min_value );

	p_Data = single_period_max_min( p_bufSrc ,  single_period_point );
	ave_max_value = p_Data[0]; //最大值
	ave_min_value = p_Data[1]; //最小值


	if(( data_length-src_min_index )/single_period_point == 0 )//表示小于一个周期
	{
		//LOGD("小于一个周期");
		send_num = data_length-src_min_index;
	}
	if(( data_length-src_min_index )/single_period_point > 0 ) //表示大于一个周期
	{
		//LOGD("大于一个周期");
		send_num = single_period_point;
	}

	p_mretValue[0]= ave_max_value;  //左右平均后的最大值
    //p_mretValue[1]=( float )( send_num -3);//向JNI 返回波形要绘制的点数,丢3个点，防止APP取数越界
    p_mretValue[1]=( float ) send_num;
    //p_mretValue[1]=( float ) 32768;
	p_mretValue[2]= ave_min_value; //左右平均后的最小值

	memcpy( &p_mretValue[3] , &p_bufSrc[src_min_index] , send_num*sizeof(float) );

	LOGD( "需要回调的左右平均后，最大值 = %f ，最小值 = %f, 返回有效数组大小 = %d" , p_mretValue[0],p_mretValue[2], (int)p_mretValue[1] );

	return p_mretValue;
}

/* 功能描述：傅立叶变换主函数 */
float* fft_func( float* psrc )
{
	int i=0;
	if(psrc == NULL)
    {
        return NULL;
    }
	float *p_imagData = NULL;
	p_imagData =( float * )malloc( data_length *sizeof( float ) );
	if( p_imagData == NULL )
	{
		LOGD( "mImagData分配内存失败！" );
		return NULL;
	}
	memset( p_imagData , 0 , sizeof( float )* data_length );

	float *p_fftCoeffs = NULL;
	p_fftCoeffs =( float * )malloc( data_length *sizeof( float ) );
	if( p_fftCoeffs == NULL )
	{
		LOGD( "pFFTCoeffs分配内存失败！" );
		return NULL;
	}
	memset( p_fftCoeffs , 0 , sizeof( float )* data_length );

	long  *p_bitReverseAddressTable = NULL;
	p_bitReverseAddressTable =( long * )malloc( data_length*sizeof( long ) );
	if( p_bitReverseAddressTable == NULL )
	{
		LOGD( "pBitReverseAddressTable分配内存失败！" );
		return NULL;
	}
	memset( p_bitReverseAddressTable , 0 , sizeof( long )* data_length );

	sif_fft( p_fftCoeffs ,  p_bitReverseAddressTable ,  data_length );    //FFT 计算
	sda_rfft( psrc ,  p_imagData  , p_fftCoeffs ,  p_bitReverseAddressTable ,  data_length ,  Log2Size );

	for( i=0;i< data_length;i++  )//FFTSize
	{
		psrc[i] = sqrt( psrc[i]*psrc[i] + p_imagData[i]*p_imagData[i] );   // R*R +I*I 开根号 求模运算
		psrc[i] /= data_length;
		psrc[i] *= 2;
	}

	#if 0
	for( i=0;i<data_length; i++ )//数据点数
	{
	   LOGD( "傅立叶变换后数据 psrc[%d] = %f " ,  i , psrc[i] );
	}
	#endif

	if( p_fftCoeffs != NULL)
	{
		free( p_fftCoeffs );
		p_fftCoeffs = NULL;
	}

	if( p_bitReverseAddressTable != NULL)
	{
		free( p_bitReverseAddressTable );
		p_bitReverseAddressTable = NULL;
	}

	if( p_imagData != NULL)
	{
		free( p_imagData );
		p_imagData = NULL;
	}

	return psrc;
}

/* 功能描述：压力算法主入口函数 */
void  press_alg_entry( float* p_buf ,  int length , float* p_retValue )
{
    if(p_buf == NULL || p_retValue == NULL)
    {
        return;
    }
	data_length = length;
	Log2Size =( long )log2( data_length );	// 计算对数

	float *buf_src_back = NULL; //备份一份原始数据，因为FFT算法会更改原始数据值
	buf_src_back =( float * )malloc( data_length *sizeof( float ) );
	if( buf_src_back == NULL )
	{
		LOGD( "mImagData分配内存失败！" );
		return;
	}
	memset( buf_src_back , 0 , data_length * sizeof( float ) );
	memcpy( buf_src_back ,  p_buf ,  data_length*sizeof( float ) );

    get_max_min( buf_src_back , p_retValue ,  get_period_point( fft_func( p_buf )) );

	if( buf_src_back != NULL)
	{
		free( buf_src_back );
		buf_src_back = NULL;
	}
}
