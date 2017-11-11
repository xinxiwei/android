/***************************************************************************************
*  (c) Copyright 2017 ENNOVAR, All rights reserved
*
*    create_by:           gxl
*  								 
*    filename:            Alg_IIRFilter.h
*
*    description:            
*              
*    revision_history:
*        Date                 By         Description
*        2017/08/03           gxl         created
**************************************************************************************/
/***************************************************************************************
*Include Files
***************************************************************************************/
#ifndef _HH_ALG_IIRFILTER_H_HH_
#define _HH_ALG_IIRFILTER_H_HH_
/***************************************************************************************
* macro definition
***************************************************************************************/

/********************************upper limiting frequency******************************/
#define UPPER_FRE_500      (500)   
#define UPPER_FRE_1000     (1000)
#define UPPER_FRE_2000     (2000)
#define UPPER_FRE_2500     (2500)
#define UPPER_FRE_4000     (4000)
#define UPPER_FRE_5000     (5000)  
#define UPPER_FRE_10000    (10000)
#define UPPER_FRE_20000    (20000)
#define UPPER_FRE_40000    (40000)

/********************************lower limiting frequency******************************/
#define LOWER_FRE_0_16     (0)
#define LOWER_FRE_1        (1)
#define LOWER_FRE_2        (2)
#define LOWER_FRE_5        (5)
#define LOWER_FRE_10       (10)
#define LOWER_FRE_20       (20)
#define LOWER_FRE_50       (50)
#define LOWER_FRE_100      (100)

#define NUM_LOWER_FRE    (8)

#define IIR_F016            (0)
#define IIR_F1              (1)
#define IIR_F2              (2)
#define IIR_F5              (3)
#define IIR_F10             (4)
#define IIR_F20             (5)
#define IIR_F50             (6)
#define IIR_F100            (7)

/************************************sample frequency**********************************/
#define SAMPLE_FRE_1280    (1280) 
#define SAMPLE_FRE_2560    (2560)
#define SAMPLE_FRE_5120    (5120)
#define SAMPLE_FRE_6400    (6400)
#define SAMPLE_FRE_10240   (10240)
#define SAMPLE_FRE_12800   (12800)
#define SAMPLE_FRE_25600   (25600)
#define SAMPLE_FRE_51200   (51200)
#define SAMPLE_FRE_102400  (102400)

#define NUM_SAMPLERATES   (9)


//#define IIR_FREQ_MAX_NUM    (8)  // iir:  0.16 1 2 5 10 20 50 100  //下限频率
//#define FIR_FREQ_MAX_NUM    (9)   // fir: 500 1000 2000 2500 4000 5000 10000 20000 40000 //上限频率

typedef struct {
    int    numRevCoeffs;          //反向系数个数
    int    numForCoeffs;          //正向系数个数
    float* w;                     //IIR滤波器状态系数
    float* pfltRevCoeffs;         //反向滤波器系数
    float* pfltForCoeffs;         //正向滤波器系数
}tIIRFilter , *tPIIRFilter;

typedef enum _tagSAMPLEFRE
{
    F_102400_0_16 = 0,
    F_102400_1,
	F_102400_2,
	F_102400_5,
	F_102400_10,
	F_102400_20,
	F_102400_50,
	F_102400_100,	
	
	F_51200_0_16 ,
    F_51200_1,
	F_51200_2,
	F_51200_5,
	F_51200_10,
	F_51200_20,
	F_51200_50,
	F_51200_100,	
	
	F_25600_0_16 ,
    F_25600_1,
	F_25600_2,
	F_25600_5,
	F_25600_10,
	F_25600_20,
	F_25600_50,
	F_25600_100,	
	
	F_12800_0_16 ,
    F_12800_1,
	F_12800_2,
	F_12800_5,
	F_12800_10,
	F_12800_20,
	F_12800_50,
	F_12800_100,	
	
	F_10240_0_16 ,
    F_10240_1,
	F_10240_2,
	F_10240_5,
	F_10240_10,
	F_10240_20,
	F_10240_50,
	F_10240_100,	
	
	F_6400_0_16 ,
    F_6400_1,
	F_6400_2,
	F_6400_5,
	F_6400_10,
	F_6400_20,
	F_6400_50,
	F_6400_100,	
	
	F_5120_0_16 ,
    F_5120_1,
	F_5120_2,
	F_5120_5,
	F_5120_10,
	F_5120_20,
	F_5120_50,
	F_5120_100,	
	
	F_2560_0_16 ,
    F_2560_1,
	F_2560_2,
	F_2560_5,
	F_2560_10,
	F_2560_20,
	F_2560_50,
	F_2560_100,
	
	F_1280_0_16 ,
    F_1280_1,
	F_1280_2,
	F_1280_5,
	F_1280_10,
	F_1280_20,
	F_1280_50,
	F_1280_100,	  
}tSAMPLEFRE, *tPSAMPLEFRE;

/***************************************************************************************
* function declaration 
***************************************************************************************/
static int _GetSampRate(int u16Upper);
static tPIIRFilter _GetFilterByFC( int SRID,int FCID);
static void cascade_iir_filter(float*   data,
                               int len,
                               const float*   rev_coeffs,    // 反向滤波器系数
                               const float*   for_coeffs,    // 正向滤波器系数
                               int rev_coeffs_num,      // 反向系数个数
                               int for_coeffs_num,      // 正向系数个数
                               float*   w,                   // IIR滤波器状态系数
                               float    factor);            
void  IIRFilterTable_Init(void);
extern int Get_InvalidNum( int Upper_frq, int Lower_frq );
extern void enter_IIR_Filter(float *src_data ,int length , int up_freq , int lw_freq);
extern void integrate_o2(float* data, int len);//, float alpha,float gamma,float beta, float factor,float x1_state,float x2_state,float y1_state,float y2_state);
extern void integrate_o1(float* data, int len);
#endif

