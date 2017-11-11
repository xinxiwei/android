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


//////////////////////
#define	SIGLIB_DO_BIT_REVERSAL		1			// Set to '1' if bit reversal is required λ����
#define	SIGLIB_FAST_BIT_REVERSAL	1			// Set to '1' for fast( look up table ) bit reversal
#define SDS_Sin( a )		(( float )sin(( double )a ) )
#define SDS_Swap( a , b ) 	{ register float SIGLIB_Tmp =( float )a; a = b; b = SIGLIB_Tmp; }
#define	SIGLIB_ZERO		(( float )0.0 )/* Zero */
#define	SIGLIB_TWO		(( float )2.0 )
#define SIGLIB_PI		(( float )3.14159265358979323846264338327950288419716939937510 )/* Pi */
#define SIGLIB_TWO_PI	( SIGLIB_TWO * SIGLIB_PI )	/* 2.0 * Pi */

#define SIZE_NUM 16384
long  data_length = 0;//����Ԫ�ظ���
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


/*
���룺����FFT�仯�������
��������������ڵĵ���
���裺
1.��FFT�������ǰ3�����ֵ���ֱ��Ӧ��indexֵ
2.������indexֵ������Сֵ
3.���γ���/index  = �����ڵĵ���
*/
int get_period_point( float *p_pfftData ) //ͨ������Ҷ�õ������ڵĵ���
{
   int i , j , m;
   float max_value1=0.0;
   float max_value2=0.0;
   float max_value3=0.0; 
   
   int max_value1_index=0;
   int max_value2_index=0;
   int max_value3_index=0;  
   
   int max_fir_flag =0;
   int max_sec_flag = 0;
   int max_thr_flag =0;
   
   float tmp_data =0.0;
   int period_point = 0;  //ÿ�����ڶ�Ӧ�Ķ��ٸ�����
   float *p_backupFftData = NULL;
   if( p_backupFftData == NULL)
   {
	   p_backupFftData =( float * )malloc(  8193 * sizeof( float ) );  
	   if( p_backupFftData == NULL )
	   {
			LOGD( "p_backupFftData �����ڴ�ʧ�ܣ�" );
			exit( EXIT_FAILURE );	
	   }
	   memset( p_backupFftData ,  0 ,  8193*sizeof( float ));
   }
   memcpy( &p_backupFftData[1] ,  p_pfftData ,  8192 *sizeof(float) ); //����һ��FFT�任�������
   
   for( j=0;j<8192;j++ )
   {
		for( i = 0;i< 8192-1-j;i++ )
		{
			if( p_pfftData[i] < p_pfftData[i+1] ) // fft�����ɴ�Сʱ��Ϊ< ,   ��С����>
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

   //LOGD( "����Ҷ����ǰ�������ֵ max_value = %f ,   %f ,   %f ,  \n" , max_value1 , max_value2 , max_value3 ); //fft���ݵ�ǰ3�����ֵ

   for( m=1;m< 8193;m++ )//�������� ����������ݶԱȣ������Ӧ��index
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
		if(max_fir_flag && max_sec_flag && max_thr_flag) //�ҵ�index����ǰ����ѭ��
			break;        		
   }
   
	//LOGD( "����Ҷ����ǰ�������ֵ��Ӧ��index, max_value1_index= %d ,   max_value2_index= %d ,  max_value3_index= %d\n" , max_value1_index , max_value2_index , max_value3_index ); ///3�����ֵ �ֱ��Ӧ��index

	int min_index = max_value1_index;
	if( max_value2_index < min_index )   min_index = max_value2_index;
    if( max_value3_index < min_index )   min_index = max_value3_index;   //��ǰ����fft�������ֵ��Ӧ��index ��Сֵ
	
	if( min_index !=0 )
	{
		period_point = data_length/min_index; // ����һ�������ڵĵ���
    }
	
	if( p_backupFftData != NULL)
	{
		free( p_backupFftData );
		p_backupFftData =NULL;	
	}
	LOGD( "���ڵ���period_point = %d " , period_point );
	return period_point;
}

float right_value[3] ={0.0}; //0λ�����ֵ�ͣ�1λ����Сֵ�ܺͣ�2λ����ٸ�С���ڸ���
float src_min_value=0.0; //ԭʼ������Сֵ
int src_min_index=0;  //ԭʼ������Сֵ index

float*  single_period_right_max_min( float *p_RBufSrc ,  int r_period_point ) //��ԭʼ������СֵΪ��㣬��nperiod_point ��ֶΣ��ֱ���������Сֵ ��ƽ����ע����󲻹����ڵ�����ҲҪ��������
{
	int i=0;
	int k=0;	    
	float right_max_sum = 0.0;  //���С���ڶ��ұ����ֵ�ܺ�
	float right_min_sum = 0.0;  //���С���ڶ��ұ���Сֵ�ܺ�
	float single_right_max_value=0.0;
    float single_right_min_value=0.0;
	
    int single_max_index=0;  //ÿ��С�����������Сֵ ��Ӧ��index
    int single_min_index=0;
	int right_period_count =( SIZE_NUM - src_min_index )/r_period_point; //ԭʼ���ݹ����Էֶ��ٸ�С����
	int right_yu_count =( SIZE_NUM- src_min_index )%r_period_point; //��������
	
	if(  right_period_count >=1 && right_yu_count ==0 ) //��ʾ����һ��������û������
	{
		  LOGD( "right 1111111 ��ʾ����һ��������û������\n" );
		  LOGD( "src_min_index = %d ,  r_period_point= %d ,  right_period_count = %d ,   right_yu_count= %d\n" , src_min_index , r_period_point , right_period_count , right_yu_count );
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
			  //LOGD( "=====single_right_max_value is = %f �� single_right_min_value= %f\n" , single_right_max_value , single_right_min_value );			  
			  right_max_sum += single_right_max_value;
			  right_min_sum += single_right_min_value;
			  			
			  right_value[0] =right_max_sum;
	          right_value[1] =right_min_sum;
			  right_value[2] =right_period_count;
		  }		   
          LOGD( "=====right___�ɷֵ����ڸ��� = %d  , right_max_sum = %f ,   right_min_sum = %f\n" , (int)right_value[2] , right_value[0] , right_value[1] );		  
	  }
  
	if(  right_period_count >=1 && right_yu_count !=0 ) //��ʾ����һ������ , ����������
	{
		  LOGD( "right 222222 ��ʾ����һ������ , ������\n" );
		  LOGD( "src_min_index = %d ,  r_period_point= %d ,  right_period_count = %d ,   right_yu_count= %d\n" , src_min_index , r_period_point , right_period_count , right_yu_count );
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
			  //LOGD( "=====single_right_max_value is = %f �� single_right_min_value= %f\n" , single_right_max_value , single_right_min_value );	
			  right_max_sum += single_right_max_value;
			  right_min_sum += single_right_min_value;			  				
		  }
		  //LOGD( "=====right___right_max_sum is = %f ,  right_min_sum= %f\n" , right_max_sum , right_min_sum );
	
		  int yu_max_index=0 , yu_min_index1=0;  //�����ڶ�Ӧ�������Сֵ ��index
		  float yu_max_value1= p_RBufSrc[SIZE_NUM -right_yu_count];
		  float yu_min_value1= p_RBufSrc[SIZE_NUM -right_yu_count];		
		  for( k =SIZE_NUM -right_yu_count ;k< SIZE_NUM;k++ )
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
           LOGD( "=====right___�ɷֵ����ڸ��� = %d ,  right_max_sum = %f ,  right_min_sum = %f\n" ,(int)right_value[2] , right_value[0] , right_value[1] );		   
	  }
	  
	if( right_period_count <1 && right_yu_count !=0 ) //��ʾ����һ��������������Щ��������
	{
		  LOGD( "right 3333333 ��ʾ����һ��������������Щ��������\n" );
		  LOGD( "src_min_index = %d ,  r_period_point= %d ,  right_period_count = %d ,   right_yu_count= %d\n" , src_min_index , r_period_point , right_period_count , right_yu_count );		
		  int yu_max_index2=0 , yu_min_index2=0;
		  float yu_max_value2= p_RBufSrc[src_min_index];
		  float yu_min_value2= p_RBufSrc[src_min_index];
		
		  for( k = src_min_index ;k< SIZE_NUM;k++ )
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
           LOGD( "=====right___����������Ϊ1 , yu_max_value2 is = %f ,   yu_min_value2 = %f \n" ,  right_value[0] , right_value[1] );			   
	  }      
	  return right_value;	
}


float left_value[3]={0.0};//0λ�����ֵ�ͣ�1λ����Сֵ�ܺͣ�2λ����ٸ�С���ڸ���
float* single_period_left_max_min( float *p_LBufSrc ,  int nperiod_point ) //��ԭʼ������СֵΪ��㣬��nperiod_point ��ֶΣ��ֱ���������Сֵ ��ƽ����ע����󲻹����ڵ�����ҲҪ��������
{
	int i=0;
	int k=0;	  
    int m=0;	
	float left_max_sum = 0.0;  //���С���ڶ�������ֵ�ܺ�
	float left_min_sum = 0.0;  //���С���ڶ������Сֵ�ܺ�
	float single_left_max_value= 0.0;
	float single_left_min_value= 0.0;  
	
	int left_period_count=0 , left_yu_count=0;
	int left_max_index =0 , left_min_index =0;
	
	left_period_count = src_min_index/nperiod_point;//ԭʼ���ݹ����Էֶ��ٸ�С����
	left_yu_count = src_min_index%nperiod_point;//��������
	
	if( left_period_count>=1 && left_yu_count ==0  )//��ʾ����һ��������û������
    {
		 LOGD( "left 1111111 ��ʾ����һ��������û������\n" );
		 LOGD( "left_src_min_index = %d ,  nperiod_point= %d ,  left_period_count = %d ,  left_yu_count= %d\n" , src_min_index , nperiod_point , left_period_count , left_yu_count );
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
		 LOGD( "=====left___�ɷֵ����ڸ��� = %d ,  left_max_sum = %f ,  left_min_sum = %f\n" , (int)left_value[2] , left_value[0] , left_value[1] );        
	}
	
	//////////
	if( left_period_count>=1 && left_yu_count !=0  )//��ʾ����һ������ ,  ������
     {
        LOGD( "left 222222 ��ʾ����һ������ ,  ������\n" );
		LOGD( "left_src_min_index = %d ,  nperiod_point= %d ,  left_period_count = %d ,  left_yu_count= %d\n" , src_min_index , nperiod_point , left_period_count , left_yu_count );
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
          LOGD( "=====left___�ɷֵ����ڸ��� = %d  ,  left_max_sum = %f ,  left_min_sum = %f \n" , (int)left_value[2] , left_value[0] , left_value[1] );           
	 }
	 
	////////////
	
	if( left_period_count <1 &&  left_yu_count!=0 )//��ʾ����һ��������������Щ��������
    {
		  LOGD( "left 3333333 ��ʾ����һ��������������Щ��������\n" );
		  LOGD( "left_src_min_index = %d ,  nperiod_point= %d ,  left_period_count = %d ,  left_yu_count= %d\n" , src_min_index , nperiod_point , left_period_count , left_yu_count );
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
		   LOGD( "=====left___�ɷֵ����ڸ��� = 1 ,  yu_max3 = %f ,   yu_min3 = %f" , left_value[0] , left_value[1] );		   
     }  
	 return left_value;	
}


float single_period_value[2]={0.0}; //0λ��ʾ���ֵ��1λ��ʾ��Сֵ
float* single_period_max_min( float *p_BufSrc ,  int nperiod_point )// �󵥸������ڵ����ֵ ����Сֵ
{
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
    LOGD( "=====single_period_value[0] is = %f ,   single_period_value[1] is = %f" , single_period_value[0] , single_period_value[1] );
	return single_period_value;
}

float* get_max_min(  float *p_bufSrc ,  float *p_mretValue ,  int single_period_point )//���ֶ�ƽ����������Сֵ��С�����ݷ���
{		  
    int i=0;	
    float *p_Data = NULL; 
    //float *p_leftData = NULL; 
    float final_value[2] = {0.0};  	
	
	for( i=0;i<SIZE_NUM;i++ )
	{
		 if( p_bufSrc[src_min_index]>=p_bufSrc[i] ){
              src_min_index=i;
              src_min_value=p_bufSrc[src_min_index];
         }
	}	
	LOGD( "src_min_index is = %d ,  src_min_value = %f\n" , src_min_index , src_min_value );

	p_Data = single_period_max_min( p_bufSrc ,  single_period_point );
	final_value[0] = p_Data[0]; //���ֵ
	final_value[1] = p_Data[1]; //��Сֵ 
	

    int send_num=0;  //��JNI��������Ĵ�С
	#if 0  //�ɵķ�ʽ
    if(( SIZE_NUM-src_min_index )%single_period_point ==0 )	//����Ϊ0����һ�����ڳ���ȡֵ	
		send_num = single_period_point;
	if(( SIZE_NUM-src_min_index )%single_period_point !=0 ) //������Ϊ0���������������ȡֵ
		send_num =( SIZE_NUM-src_min_index )%single_period_point;		
	#else  //�µķ�ʽ
	if(( SIZE_NUM-src_min_index )/single_period_point == 0 )//��ʾС��һ������
	{
		LOGD("С��һ������");
		send_num = SIZE_NUM-src_min_index;
	}		
	if(( SIZE_NUM-src_min_index )/single_period_point > 0 ) //��ʾ����һ������
	{
		LOGD("����һ������");
		// send_num =( SIZE_NUM-src_min_index )%single_period_point;
		send_num = single_period_point;
	}			
	#endif
    
	
	p_mretValue[0]= final_value[0];  //����ƽ��������ֵ
	p_mretValue[1]=( float )( send_num -3);//��JNI ���ز���Ҫ���Ƶĵ���,��3���㣬��ֹAPPȡ��Խ��
	p_mretValue[2]= final_value[1]; //����ƽ�������Сֵ
	
	memcpy( &p_mretValue[3] , &p_bufSrc[src_min_index] , send_num*sizeof(float) );
	
	LOGD( "��Ҫ�ص�������ƽ�������ֵ = %f ����Сֵ = %f, ������Ч�����С = %d" , p_mretValue[0],p_mretValue[2], (int)p_mretValue[1] );
	
	return p_mretValue;
}

float final_data[SIZE_NUM] ={0.0};
float * p_finalFftData = NULL;  //fftת���������

float* fft_func( float *psrc ) //����Ҷ�任����
{    	
	float src_buf[SIZE_NUM]={0.0};	
	int i=0;
	for( i=0;i< SIZE_NUM ;i++ )
	{
	   src_buf[i] = psrc[i];
	}
	
	float *p_imagData = NULL;
	if( p_imagData == NULL)
	{
		p_imagData =( float * )malloc( SIZE_NUM *sizeof( float ) ); 
		if( p_imagData == NULL )
		{
			LOGD( "mImagData�����ڴ�ʧ�ܣ�" );
			exit( EXIT_FAILURE );	
		}
		memset( p_imagData , 0 , sizeof( float )* SIZE_NUM );	
	}
    	
	float *p_fftCoeffs = NULL;
    if( p_fftCoeffs == NULL)
	{
		p_fftCoeffs =( float * )malloc( SIZE_NUM *sizeof( float ) );
		if( p_fftCoeffs == NULL )
		{
			LOGD( "pFFTCoeffs�����ڴ�ʧ�ܣ�" );
			exit( EXIT_FAILURE );	
		}
		memset( p_fftCoeffs , 0 , sizeof( float )* SIZE_NUM );
	}
    
	long  *p_bitReverseAddressTable = NULL;
	if( p_bitReverseAddressTable == NULL)
	{
		p_bitReverseAddressTable =( long * )malloc( SIZE_NUM*sizeof( long ) );
		if( p_bitReverseAddressTable == NULL )
		{
			LOGD( "pBitReverseAddressTable�����ڴ�ʧ�ܣ�" );
			exit( EXIT_FAILURE );	
		}
		memset( p_bitReverseAddressTable , 0 , sizeof( long )* SIZE_NUM );
	}

	//LOGD( "data_length=%ld ,  Log2Size=%ld " ,  data_length , Log2Size );
	sif_fft( p_fftCoeffs ,  p_bitReverseAddressTable ,  data_length );    //FFT ����
	sda_rfft( src_buf ,  p_imagData  , p_fftCoeffs ,  p_bitReverseAddressTable ,  data_length ,  Log2Size );
	
	for( i=0;i< data_length;i++  )//FFTSize
	{
	    final_data[i] = sqrt( src_buf[i]*src_buf[i] + p_imagData[i]*p_imagData[i] );   // R*R +I*I ������ ��ģ����
        final_data[i] /= data_length;
		final_data[i] *= 2;		
	}	

    p_finalFftData = final_data;
	
	#if 0
	for( i=0;i<data_length; i++ )//���ݵ���
	{
	   LOGD( "����Ҷ�任������111 p_finalFftData[%d] = %f " ,  i , p_finalFftData[i] );
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
	
	return p_finalFftData;
}

void  press_alg_entry( float *p_buf ,  int length , float *p_retValue )//ѹ���㷨����ں���
{	
	data_length = length;
	Log2Size =( long )log2( data_length );	// �������
	
	float buf_src_back[SIZE_NUM]={0.0};
		
	memcpy( buf_src_back ,  p_buf ,  data_length*sizeof( float ) );
	
    float *p_fftData = NULL;    
	p_fftData = fft_func( p_buf );	
		
    get_max_min( buf_src_back , p_retValue ,  get_period_point( p_fftData ) );
}


