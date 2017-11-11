/***************************************************************************************
* ( c ) Copyright 2017 ENNOVAR ,  All rights reserved
*
*    create_by:           gxl
*  								 
*    filename:            Spectrum.c
*
*    description:            
*              
*    revision_history:
*        Date                 By         Description
*        2017/08/02           gxl         created
**************************************************************************************/
/**************************************************************************************
                       Include Files
**************************************************************************************/
#include <hardware/imx6q_spectrum.h>
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
#include <hardware/imx6q_spi_vibrate.h>
#include <sys/timeb.h>
#include <time.h>
#include <math.h>


//////////////////////
#define	SIGLIB_DO_BIT_REVERSAL		1			// Set to '1' if bit reversal is required 位倒序
#define	SIGLIB_FAST_BIT_REVERSAL	1			// Set to '1' for fast( look up table ) bit reversal

SLData_t  SDS_I0Bessel( const SLData_t x )
{
    SLData_t ax ,  y;

    if((  ax = SDS_Abs(  x ) ) < 3.75 )
    {
		y = x /(( SLData_t )3.75 );
		y *= y;
		return(( SLData_t )( SIGLIB_ONE + y *(  3.5156229 + y *(  3.0899424 + y *(  1.2067492 +
				y *(  0.2659732 + y *(  0.360768e-1 + y * 0.45813e-2 ) ) ) ) ) ) );
    }
    else
    {
		y =(( SLData_t )3.75 ) / ax;
		return(( SLData_t )(( SDS_Exp(  ax ) / SDS_Sqrt( ax ) ) *( 0.39894228 + y *( 0.1328592e-1 + y *( 0.225319e-2 +
				y *( -0.157565e-2 + y *( 0.916281e-2 + y *( -0.2057706e-1 +
				y *( 0.2635537e-1 + y *( -0.1647633e-1 + y * 0.392377e-2 ) ) ) ) ) ) ) ) ) );
    }
}

/**************************************************************************************
*     Function:              SIF_Window     
*     Description:           Initialise FFT functionality                       
*     Return:                void
*     Parameter:   			 SLData_t * pFFTCoeffs ,  SLFixData_t FFTSize
enum SLWindow_t                     // Window types
{
    SIGLIB_HANNING = 1 , 	
	SIGLIB_TRIANGLE , 
	SIGLIB_RECTANGLE , 	
    SIGLIB_HAMMING , 	
    SIGLIB_BLACKMAN , 
	SIGLIB_KAISER	
};
**************************************************************************************/
SLFixData_t SIF_Window( SLData_t *wp ,  enum SLWindow_t window_type ,  const SLData_t Coeff ,  SLFixData_t window_size )
{
    SLFixData_t    i;
    SLData_t   theta ,  theta_inc  , z;
    theta = SIGLIB_ZERO;
    theta_inc =( SIGLIB_TWO * SIGLIB_PI ) /(( SLData_t )window_size );

    switch( window_type )
    {
        case SIGLIB_HANNING :              // 汉宁 window 
		    LOGD( "xin:=== SIGLIB_HANNING " );
           	for( i = 0; i < window_size; i++ ,  theta += theta_inc )
			{
				*wp++ = SIGLIB_HALF *( SIGLIB_ONE - SDS_Cos( theta ) );
			}
			return( SIGLIB_NO_ERROR );			
		case SIGLIB_TRIANGLE :              // 三角 window 
		    LOGD( "xin:=== SIGLIB_TRIANGLE " );
            for( i = 0; i < window_size/2; i++ )
			{
				*wp++ = SIGLIB_TWO *(( SLData_t )i ) /(( SLData_t )window_size );
			}
			for( i = 0; i < window_size/2; i++ )
			{
				*wp++ = SIGLIB_ONE -( SIGLIB_TWO *(( SLData_t )i ) /(( SLData_t )window_size ) );
			}
			return( SIGLIB_NO_ERROR );			
		case SIGLIB_RECTANGLE :              // 矩形 window  
		    LOGD( "xin:=== SIGLIB_RECTANGLE " );
            for( i = 0; i < window_size; i++ )
			{
				*wp++ = SIGLIB_ONE;
			}
			return( SIGLIB_NO_ERROR );			
		case SIGLIB_HAMMING :              // 海明 window 
		    LOGD( "xin:=== SIGLIB_HAMMING " );
            for( i = 0; i < window_size; i++ ,  theta += theta_inc )
			{
				*wp++ =(( SLData_t )0.54 ) -((( SLData_t )0.46 ) * SDS_Cos( theta ) );
			}
			return( SIGLIB_NO_ERROR );
		case SIGLIB_BLACKMAN :              // 布拉克曼 window 
		    LOGD( "xin:=== SIGLIB_BLACKMAN " );
            for( i = 0; i < window_size; i++ ,  theta += theta_inc )
			{
				*wp++ =(( SLData_t )0.42 ) -( SIGLIB_HALF * SDS_Cos( theta ) ) +((( SLData_t )0.08 ) * SDS_Cos( SIGLIB_TWO * theta ) );
			}
			return( SIGLIB_NO_ERROR );
		case SIGLIB_KAISER :              // 凯塞-贝塞尔 window 
		    LOGD( "xin:=== SIGLIB_KAISER " );
            z = -(((( SLData_t )window_size ) - SIGLIB_ONE ) / SIGLIB_TWO );
			for( i = 0; i < window_size; i++ ,  z++ )
			{
				*wp++ =( SDS_I0Bessel(( Coeff *
					SDS_Sqrt( SIGLIB_ONE - SDS_Pow(( SIGLIB_TWO * z /( window_size - 1 ) ) ,  SIGLIB_TWO ) ) ) ) /
					SDS_I0Bessel( Coeff ) );
			}
			return( SIGLIB_NO_ERROR );
		
        default :
			return( SIGLIB_PARAMETER_ERROR );	// Incorrect parameter 
    }
}      
	
/**************************************************************************************
*     Function:              SDA_Window     
*     Description:           Apply window to a array of data                       
*     Return:                void
*     Parameter:   			 SLData_t *ip , SLData_t ,  SLData_t *rp , SLData_t *wp
*                            SLFixData_t window_size 
**************************************************************************************/
void SDA_Window( SLData_t *ip ,  SLData_t *rp ,  SLData_t *wp ,  SLFixData_t window_size )
{
    SLFixData_t    i;

    for( i = 0; i < window_size; i++ )
        *rp++ = *ip++ * *wp++;
     	
	return ;
}      

void SDA_Index_Bit_Reverse_Reorder( SLFixData_t Src[] , 	SLFixData_t Dst[] , 	const SLFixData_t nFFTSize )
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

void SIF_Fft( SLData_t* pFFTCoeffs , 	SLFixData_t* pBitReverseAddressTable , SLFixData_t mFFTSize )
{
	register long i;
    if(pFFTCoeffs == NULL)
    {
        exit( EXIT_FAILURE );
    }
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
	SDA_Index_Bit_Reverse_Reorder( pBitReverseAddressTable ,  pBitReverseAddressTable ,  mFFTSize );
#endif
}

void SDA_Rfft( SLData_t RealData[] , 	SLData_t ImagData[] , const SLData_t* pFFTCoeffs , const SLFixData_t* pBitReverseAddressTable , const SLFixData_t pFFTSize , const SLFixData_t mLog2Size )
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

int WALG_SDA_FftAverage( const float pRealData[] , 		//源数据区
	const int nAvgType , 					//FFT平均方式
	const int nLastAvgNum , 				//之前FFT平均次数+1
	const int nLineNum , 					//FFT线数目
	float pAvgData[] )					//平均数据区
{
	register int i;

	if( !pAvgData )
		return SIGLIB_ERROR;

	switch( nAvgType )
	{
	case SIGLIB_FFTAVG_VECTOR: //线性
		for(  i=0; i<nLineNum; i++ )
		{
			pAvgData[i] =( pAvgData[i]*nLastAvgNum + pRealData[i] )/( nLastAvgNum+1 );
		}
		break;
		
	case SIGLIB_FFTAVG_RMS: //均方根
		for(  i=0; i<nLineNum; i++ )
		{
			pAvgData[i] = sqrt(( pAvgData[i]*pAvgData[i]*nLastAvgNum + pRealData[i]*pRealData[i] )/( nLastAvgNum+1 ) );
		}
		break;

	case SIGLIB_FFTAVG_PEAKHOLD: //峰值保持
		for(  i=0; i<nLineNum; i++ )
		{
			if(  pRealData[i] > pAvgData[i] )
			{
				pAvgData[i] = pRealData[i];
			}
		}
		break;

	case SIGLIB_FFTAVG_NONE: //不进行
	default:
		for(  i=0; i<nLineNum; i++ )
		{
			pAvgData[i] = pRealData[i];
		}		
		break;
	}
	return( SIGLIB_NO_ERROR );
}

/*函数功能：通过傅立叶得到频谱 y 最大值 ,及对应的index */
void get_fft_max( float *p_pfftData ,  int size ) 
{
	int i = 0;
	int max_value_index = 0;
	float max_value = 0.0;
    if(p_pfftData == NULL)
    {
        exit( EXIT_FAILURE );
    }
	for( i = 0; i < size; i++)
	{
		 if( p_pfftData[max_value_index] <= p_pfftData[ i ] ){
			max_value_index = i;
			max_value = p_pfftData[max_value_index];
		 }
	}
	LOGD( "fftdata_max_value = %f ,  max_value_index =%d" , max_value  ,  max_value_index ); ///FFT中数据最大值	
}



/* 定义变量 */
//float
SLData_t  *pRealData ,  *pImagData ,  *pWindowCoeffs ,  *pResultsData ,  *pFFTCoeffs ;
//long
SLFixData_t  *p_bitReverseAddressTable;

/* 函数功能:频谱计算入口函数 */
void fft_alg_entry2( float *pSrcBuf ,  long length ,  int window_type ,  int average_mode ,  int average_num )
{
	int i=0;	
	SLFixData_t 	FFT_SIZE =0; //数组元素个数
	SLFixData_t  	WINDOW_SIZE =0;
    if(pSrcBuf == NULL || length == 0)
    {
        exit( EXIT_FAILURE );
    }
	FFT_SIZE = length;
	WINDOW_SIZE = FFT_SIZE;
	
    pRealData = SUF_VectorArrayAllocate( FFT_SIZE );       // Allocate data arrays 
    pImagData = SUF_VectorArrayAllocate( FFT_SIZE );
    pFFTCoeffs = SUF_FftCoefficientAllocate( FFT_SIZE );
    pResultsData = SUF_VectorArrayAllocate( FFT_SIZE );     // RMS result data array 	
	
	p_bitReverseAddressTable =( SLFixData_t* )malloc( FFT_SIZE*sizeof( SLFixData_t ) ); 
	
    pWindowCoeffs = SUF_VectorArrayAllocate( WINDOW_SIZE ); // Window data array
   
    if( pRealData == NULL || pImagData == NULL || pFFTCoeffs == NULL || pResultsData == NULL || p_bitReverseAddressTable == NULL || pWindowCoeffs == NULL )
	{
      LOGD( "fft_alg_entry2 内存分配失败" );
      exit( EXIT_FAILURE );
	}
   
    memcpy( pRealData , pSrcBuf , FFT_SIZE*sizeof( SLData_t ) );
		
	SIF_Fft( pFFTCoeffs ,  p_bitReverseAddressTable ,  FFT_SIZE ); // Init. FFT
	/*
	if( window_type !=0 )  //经过汉宁窗，数值整体小1倍
	{
		LOGD( "window_type =   %d" ,  window_type );
		SIF_Window( pWindowCoeffs ,  window_type ,  SIGLIB_ZERO ,  FFT_SIZE );  // Generate  window table
		SDA_Window( pRealData ,  pRealData ,  pWindowCoeffs ,  WINDOW_SIZE );  // Apply window to data  
	}
	*/
	SDA_Rfft( pRealData ,  pImagData ,  pFFTCoeffs ,  p_bitReverseAddressTable ,  FFT_SIZE , ( long )log2( FFT_SIZE ) );
   
    for( i=0; i<( FFT_SIZE>>1 ); i++  )//FFT_SIZE/2  只对前一半数据求模
	{
	    pSrcBuf[i] = sqrt( pRealData[i]*pRealData[i] + pImagData[i]*pImagData[i] );   // R*R +I*I 开根号	
		pSrcBuf[i] /= length;
		pSrcBuf[i] *= 2;        	
	} 
	
#if 0
/* 	SLData_t  *pFFTData ;
	pFFTData = SUF_VectorArrayAllocate( FFT_SIZE/2 );
    if( pFFTData == NULL)
	{
      LOGD( "fft_alg_entry2 内存分配失败" );
      exit( EXIT_FAILURE );
	}
	memcpy(pFFTData, pSrcBuf, (FFT_SIZE/2)*sizeof(SLData_t));
	get_fft_max( pFFTData , FFT_SIZE/2 );  // 求FFT y值最大值，及对应的x 值
	SUF_MemoryFree( pFFTData );  */
#endif 		
	
    SUF_MemoryFree( pRealData );  pRealData = NULL;
    SUF_MemoryFree( pImagData );  pImagData = NULL;
    SUF_MemoryFree( pFFTCoeffs ); pFFTCoeffs = NULL;
    SUF_MemoryFree( pResultsData );	pResultsData = NULL;
	SUF_MemoryFree( p_bitReverseAddressTable );	 p_bitReverseAddressTable = NULL;
	SUF_MemoryFree( pWindowCoeffs ); pWindowCoeffs = NULL;	   	
}

   
   
   
   
   
   
   
   
   