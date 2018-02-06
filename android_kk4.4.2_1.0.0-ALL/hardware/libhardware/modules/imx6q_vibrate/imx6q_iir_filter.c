/***************************************************************************************
*  (c) Copyright 2017 ENNOVAR,  All rights reserved
*
*    create_by:           gxl
*
*    filename:            Alg_IIRFilter.c
*
*    description:
*
*    revision_history:
*        Date                 By         Description
*        2017/08/03           gxl         created
**************************************************************************************/
/**************************************************************************************
                       Include Files
**************************************************************************************/
#include <hardware/hardware.h>
#include <hardware/gpio.h>
#include <hardware/log.h>
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
#include <hardware/imx6q_iir_filter.h>
#include <hardware/imx6q_iir_coeffs.h>
/* IIR高通滤波器，用下限卡 */
//将imx6q_iir_coeffs.c中的滤波器系数在此声明
extern float S0_F016_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM];//反向系数个数 2
extern float S0_F016_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM];//正向系数个数 3
extern float S0_F1_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM];
extern float S0_F1_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM];
extern float S0_F2_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM];
extern float S0_F2_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM];
extern float S0_F5_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM];
extern float S0_F5_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM];
extern float S0_F10_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM];
extern float S0_F10_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM];
extern float S0_F20_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S0_F20_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S0_F50_ReverseCoeffs[S0_F50_REVERSCOEFFS_NUM];
extern float S0_F50_ForwardCoeffs[S0_F50_FORWARDCOEFFS_NUM];
extern float S0_F100_ReverseCoeffs[S0_F100_REVERSCOEFFS_NUM];
extern float S0_F100_ForwardCoeffs[S0_F100_FORWARDCOEFFS_NUM];

extern float S1_F016_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM];
extern float S1_F016_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM];
extern float S1_F1_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM];
extern float S1_F1_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM];
extern float S1_F2_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM];
extern float S1_F2_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM];
extern float S1_F5_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM];
extern float S1_F5_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM];
extern float S1_F10_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM];
extern float S1_F10_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM];
extern float S1_F20_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S1_F20_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S1_F50_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S1_F50_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S1_F100_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S1_F100_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];

extern float S2_F016_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM];
extern float S2_F016_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM];
extern float S2_F1_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM];
extern float S2_F1_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM];
extern float S2_F2_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM];
extern float S2_F2_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM];
extern float S2_F5_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S2_F5_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S2_F10_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S2_F10_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S2_F20_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S2_F20_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S2_F50_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S2_F50_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S2_F100_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S2_F100_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];

extern float S3_F016_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM];
extern float S3_F016_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM];
extern float S3_F1_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM];
extern float S3_F1_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM];
extern float S3_F2_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM];
extern float S3_F2_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM];
extern float S3_F5_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S3_F5_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S3_F10_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S3_F10_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S3_F20_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S3_F20_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S3_F50_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S3_F50_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S3_F100_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S3_F100_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];

extern float S4_F016_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM];
extern float S4_F016_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM];
extern float S4_F1_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM];
extern float S4_F1_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM];
extern float S4_F2_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S4_F2_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S4_F5_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S4_F5_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S4_F10_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S4_F10_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S4_F20_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S4_F20_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S4_F50_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S4_F50_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S4_F100_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S4_F100_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];

extern float S5_F016_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM];
extern float S5_F016_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM];
extern float S5_F1_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM];
extern float S5_F1_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM];
extern float S5_F2_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S5_F2_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S5_F5_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S5_F5_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S5_F10_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S5_F10_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S5_F20_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S5_F20_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S5_F50_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S5_F50_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S5_F100_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S5_F100_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];

extern float S6_F016_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM];
extern float S6_F016_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM];
extern float S6_F1_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM];
extern float S6_F1_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM];
extern float S6_F2_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S6_F2_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S6_F5_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S6_F5_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S6_F10_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S6_F10_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S6_F20_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S6_F20_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S6_F50_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S6_F50_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S6_F100_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S6_F100_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];

extern float S7_F016_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM];
extern float S7_F016_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM];
extern float S7_F1_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM];
extern float S7_F1_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM];
extern float S7_F2_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S7_F2_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S7_F5_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S7_F5_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S7_F10_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S7_F10_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S7_F20_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S7_F20_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S7_F50_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S7_F50_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S7_F100_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S7_F100_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];

extern float S8_F016_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM];
extern float S8_F016_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM];
extern float S8_F1_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM];
extern float S8_F1_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM];
extern float S8_F2_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S8_F2_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S8_F5_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S8_F5_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S8_F10_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S8_F10_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S8_F20_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S8_F20_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S8_F50_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S8_F50_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];
extern float S8_F100_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM];
extern float S8_F100_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM];

/* fir滤波之后的无效点数 */
// iir:  0.16 1 2 5 10 20 50 100  //下限频率
// fir:  500  1000 2000 2500 4000 5000 10000 20000 40000 //上限频率
static int invalid_num_table[NUM_SAMPLERATES][NUM_LOWER_FRE] ={//9 8
  {8704, 1408, 1536, 768, 358, 179, 77, 38},                 //1.28    上限500
  {17408, 2816, 2560, 1280, 640, 307, 128, 77},              //2.56    上限1000
  {34816, 5632, 5120, 2560, 1024, 614, 307, 102},            //5.12    上限2000
  {43520, 7040, 6400, 3200, 1280, 768, 256, 128},            //6.4     上限2500
  {69632, 11264, 10240, 5120, 3072, 1228, 307, 307},         //10.24   上限4000
  {87040, 14080, 6400, 7168, 2560, 1792, 512, 256},          //12.8    上限5000
  {174080, 28160, 12800, 10240, 5120, 3072, 1024, 512},      //25.6    上限10000
  {348160, 56320, 25600, 10240, 7680, 5120, 3072, 1024},     //51.2    上限20000
  {6.9E+5, 1.1E+5, 51200, 20480, 10240, 12288, 4096, 3072},  //102.4   上限40000
};

//从table表中获取无效点数
static int get_invalid_num_from_table(int* table,  int  lw_freq){
    if(table == NULL)
    {
        return 0;
    }
    switch(lw_freq){
	    case LOWER_FRE_0_16:{
            return table[IIR_F016];
	    }case LOWER_FRE_1:{
            return table[IIR_F1];
        }case LOWER_FRE_2:{
            return table[IIR_F2];
        }case LOWER_FRE_5:{
            return table[IIR_F5];
        }case LOWER_FRE_10:{
            return table[IIR_F10];
        }case LOWER_FRE_20:{
            return table[IIR_F20];
        }case LOWER_FRE_50:{
            return table[IIR_F50];
        }case LOWER_FRE_100:{
            return table[IIR_F100];
        }default:{
            return 0;
        }
    }
}

/* 功能描述：得到无效点数 */
int get_invalid_num(int up_freq,  int lw_freq)
{
    switch(up_freq)
    {
        case UPPER_FRE_500:
            {
                return get_invalid_num_from_table(&invalid_num_table[0][0], lw_freq);
            }
        case UPPER_FRE_1000:
            {
                return get_invalid_num_from_table(&invalid_num_table[1][0], lw_freq);
            }
        case UPPER_FRE_2000:
            {
                return get_invalid_num_from_table(&invalid_num_table[2][0], lw_freq);
            }
        case UPPER_FRE_2500:
            {
                return get_invalid_num_from_table(&invalid_num_table[3][0], lw_freq);
            }
        case UPPER_FRE_4000:
            {
                return get_invalid_num_from_table(&invalid_num_table[4][0], lw_freq);
            }
        case UPPER_FRE_5000:
            {
                return get_invalid_num_from_table(&invalid_num_table[5][0], lw_freq);
            }
        case UPPER_FRE_10000:
            {
                return get_invalid_num_from_table(&invalid_num_table[6][0], lw_freq);
            }
        case UPPER_FRE_20000:
            {
                return get_invalid_num_from_table(&invalid_num_table[7][0], lw_freq);
            }
        case UPPER_FRE_40000:
            {
                return get_invalid_num_from_table(&invalid_num_table[8][0], lw_freq);
            }
        default:
            {
                return 0;
            }
    }
}

//IIR滤波器表
static tIIRFilter iir_filters_table[NUM_SAMPLERATES][NUM_LOWER_FRE];
/**************************************************************************************
*     Function:              get_smp_rate
*     Description:           find samprate for upper frequency
*     Return:                INT32U
*     Parameter:   			 u16Upper
*
**************************************************************************************/
static int get_smp_rate(int up_freq)  // 输入上限，输出采样率
{
    if(UPPER_FRE_500 == up_freq || UPPER_FRE_1000 == up_freq || UPPER_FRE_2000 == up_freq
        || UPPER_FRE_2500 == up_freq || UPPER_FRE_4000  == up_freq || UPPER_FRE_5000 == up_freq
		|| UPPER_FRE_10000 == up_freq|| UPPER_FRE_20000 == up_freq || UPPER_FRE_40000 == up_freq)
    {
        return (2.56*up_freq);
    }
    else
    {
        return 102400;
    }
}
/**************************************************************************************
*     Function:              get_iir_filter
*     Description:           Initialise FFT functionality
*     Return:                void
*     Parameter:   			 float* data,  int len, float* rev_coeffs,
**************************************************************************************/
//通过FIR低通滤波器的截止频率获取降采样后的采样率
static tPIIRFilter get_iir_filter(int samprate,  int lw_freq) //输入采样率 ，下限
{
    int index_sr = 0;
    int index_fc = 0;

    switch( samprate ) //采样率
    {
        case SAMPLE_FRE_102400:
        {
            index_sr = 0;
            break;
        }
        case SAMPLE_FRE_51200:
        {
            index_sr = 1;
            break;
        }
        case SAMPLE_FRE_25600:
        {
            index_sr = 2;
            break;
        }
        case SAMPLE_FRE_12800:
        {
            index_sr = 3;
            break;
        }
        case SAMPLE_FRE_10240:
        {
            index_sr = 4;
            break;
        }
        case SAMPLE_FRE_6400:
        {
            index_sr = 5;
            break;
        }
		case SAMPLE_FRE_5120:
        {
            index_sr = 6;
            break;
        }
		case SAMPLE_FRE_2560:
        {
            index_sr = 7;
            break;
        }
		case SAMPLE_FRE_1280:
        {
            index_sr = 8;
            break;
        }
        default:
        {
            return NULL;
        }
    }

    switch( lw_freq )//下限频率
    {
        case LOWER_FRE_0_16:
        {
            index_fc = 0;
            break;
        }
        case LOWER_FRE_1:
        {
            index_fc = 1;
            break;
        }
        case LOWER_FRE_2:
        {
            index_fc = 2;
            break;
        }
        case LOWER_FRE_5:
        {
            index_fc = 3;
            break;
        }
        case LOWER_FRE_10:
        {
            index_fc = 4;
            break;
        }
        case LOWER_FRE_20:
        {
            index_fc = 5;
            break;
        }
        case LOWER_FRE_50:
        {
            index_fc = 6;
            break;
        }
        case LOWER_FRE_100:
        {
            index_fc = 7;
            break;
        }
        default:
        {
            return NULL;
        }
    }
    //LOGD( "index_sr =%d ,  index_fc= %d" , index_sr , index_fc );
    return &iir_filters_table[index_sr][index_fc];
}

//初始化IIR滤波器表
void  init_iir_filters(void)
{
    memset(iir_filters_table, 0, sizeof(iir_filters_table));
    ///102400
    iir_filters_table[IIR_S1][IIR_F016].pfltRevCoeffs = S0_F016_ReverseCoeffs;
    iir_filters_table[IIR_S1][IIR_F016].pfltForCoeffs = S0_F016_ForwardCoeffs;
    iir_filters_table[IIR_S1][IIR_F016].numRevCoeffs = sizeof(S0_F016_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S1][IIR_F016].numForCoeffs = sizeof(S0_F016_ForwardCoeffs)/ sizeof(float);
    
    iir_filters_table[IIR_S1][IIR_F1].pfltRevCoeffs = S0_F1_ReverseCoeffs;
    iir_filters_table[IIR_S1][IIR_F1].pfltForCoeffs = S0_F1_ForwardCoeffs;
    iir_filters_table[IIR_S1][IIR_F1].numRevCoeffs = sizeof(S0_F1_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S1][IIR_F1].numForCoeffs = sizeof(S0_F1_ForwardCoeffs)/ sizeof(float);
    
    iir_filters_table[IIR_S1][IIR_F2].pfltRevCoeffs = S0_F2_ReverseCoeffs;
    iir_filters_table[IIR_S1][IIR_F2].pfltForCoeffs = S0_F2_ForwardCoeffs;
    iir_filters_table[IIR_S1][IIR_F2].numRevCoeffs = sizeof(S0_F2_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S1][IIR_F2].numForCoeffs = sizeof(S0_F2_ForwardCoeffs)/ sizeof(float);    

    iir_filters_table[IIR_S1][IIR_S5].pfltRevCoeffs = S0_F5_ReverseCoeffs;
    iir_filters_table[IIR_S1][IIR_S5].pfltForCoeffs = S0_F5_ForwardCoeffs;
    iir_filters_table[IIR_S1][IIR_S5].numRevCoeffs = sizeof(S0_F5_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S1][IIR_S5].numForCoeffs = sizeof(S0_F5_ForwardCoeffs)/ sizeof(float);
    
    iir_filters_table[IIR_S1][IIR_F10].pfltRevCoeffs = S0_F10_ReverseCoeffs;
    iir_filters_table[IIR_S1][IIR_F10].pfltForCoeffs = S0_F10_ForwardCoeffs;
    iir_filters_table[IIR_S1][IIR_F10].numRevCoeffs = sizeof(S0_F10_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S1][IIR_F10].numForCoeffs = sizeof(S0_F10_ForwardCoeffs)/ sizeof(float);
    
    iir_filters_table[IIR_S1][IIR_F20].pfltRevCoeffs = S0_F20_ReverseCoeffs;
    iir_filters_table[IIR_S1][IIR_F20].pfltForCoeffs = S0_F20_ForwardCoeffs;
    iir_filters_table[IIR_S1][IIR_F20].numRevCoeffs = sizeof(S0_F20_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S1][IIR_F20].numForCoeffs = sizeof(S0_F20_ForwardCoeffs)/ sizeof(float);
    
    iir_filters_table[IIR_S1][IIR_F50].pfltRevCoeffs = S0_F50_ReverseCoeffs;
    iir_filters_table[IIR_S1][IIR_F50].pfltForCoeffs = S0_F50_ForwardCoeffs;
    iir_filters_table[IIR_S1][IIR_F50].numRevCoeffs = sizeof(S0_F50_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S1][IIR_F50].numForCoeffs = sizeof(S0_F50_ForwardCoeffs)/ sizeof(float);
    
    iir_filters_table[IIR_S1][IIR_F100].pfltRevCoeffs = S0_F100_ReverseCoeffs;
    iir_filters_table[IIR_S1][IIR_F100].pfltForCoeffs = S0_F100_ForwardCoeffs;
    iir_filters_table[IIR_S1][IIR_F100].numRevCoeffs = sizeof(S0_F100_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S1][IIR_F100].numForCoeffs = sizeof(S0_F100_ForwardCoeffs)/ sizeof(float);   
    ///51200
    iir_filters_table[IIR_S2][IIR_F016].pfltRevCoeffs = S1_F016_ReverseCoeffs;
    iir_filters_table[IIR_S2][IIR_F016].pfltForCoeffs = S1_F016_ForwardCoeffs;
    iir_filters_table[IIR_S2][IIR_F016].numRevCoeffs = sizeof(S1_F016_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S2][IIR_F016].numForCoeffs = sizeof(S1_F016_ForwardCoeffs)/ sizeof(float);
    
    iir_filters_table[IIR_S2][IIR_F1].pfltRevCoeffs = S1_F1_ReverseCoeffs;
    iir_filters_table[IIR_S2][IIR_F1].pfltForCoeffs = S1_F1_ForwardCoeffs;
    iir_filters_table[IIR_S2][IIR_F1].numRevCoeffs = sizeof(S1_F1_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S2][IIR_F1].numForCoeffs = sizeof(S1_F1_ForwardCoeffs)/ sizeof(float);
    
    iir_filters_table[IIR_S2][IIR_F2].pfltRevCoeffs = S1_F2_ReverseCoeffs;
    iir_filters_table[IIR_S2][IIR_F2].pfltForCoeffs = S1_F2_ForwardCoeffs;
    iir_filters_table[IIR_S2][IIR_F2].numRevCoeffs = sizeof(S1_F2_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S2][IIR_F2].numForCoeffs = sizeof(S1_F2_ForwardCoeffs)/ sizeof(float);    

    iir_filters_table[IIR_S2][IIR_S5].pfltRevCoeffs = S1_F5_ReverseCoeffs;
    iir_filters_table[IIR_S2][IIR_S5].pfltForCoeffs = S1_F5_ForwardCoeffs;
    iir_filters_table[IIR_S2][IIR_S5].numRevCoeffs = sizeof(S1_F5_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S2][IIR_S5].numForCoeffs = sizeof(S1_F5_ForwardCoeffs)/ sizeof(float);
    
    iir_filters_table[IIR_S2][IIR_F10].pfltRevCoeffs = S1_F10_ReverseCoeffs;
    iir_filters_table[IIR_S2][IIR_F10].pfltForCoeffs = S1_F10_ForwardCoeffs;
    iir_filters_table[IIR_S2][IIR_F10].numRevCoeffs = sizeof(S1_F10_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S2][IIR_F10].numForCoeffs = sizeof(S1_F10_ForwardCoeffs)/ sizeof(float);
    
    iir_filters_table[IIR_S2][IIR_F20].pfltRevCoeffs = S1_F20_ReverseCoeffs;
    iir_filters_table[IIR_S2][IIR_F20].pfltForCoeffs = S1_F20_ForwardCoeffs;
    iir_filters_table[IIR_S2][IIR_F20].numRevCoeffs = sizeof(S1_F20_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S2][IIR_F20].numForCoeffs = sizeof(S1_F20_ForwardCoeffs)/ sizeof(float);
    
    iir_filters_table[IIR_S2][IIR_F50].pfltRevCoeffs = S1_F50_ReverseCoeffs;
    iir_filters_table[IIR_S2][IIR_F50].pfltForCoeffs = S1_F50_ForwardCoeffs;
    iir_filters_table[IIR_S2][IIR_F50].numRevCoeffs = sizeof(S1_F50_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S2][IIR_F50].numForCoeffs = sizeof(S1_F50_ForwardCoeffs)/ sizeof(float);
    
    iir_filters_table[IIR_S2][IIR_F100].pfltRevCoeffs = S1_F100_ReverseCoeffs;
    iir_filters_table[IIR_S2][IIR_F100].pfltForCoeffs = S1_F100_ForwardCoeffs;
    iir_filters_table[IIR_S2][IIR_F100].numRevCoeffs = sizeof(S1_F100_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S2][IIR_F100].numForCoeffs = sizeof(S1_F100_ForwardCoeffs)/ sizeof(float);
    ///25600
    iir_filters_table[IIR_S3][IIR_F016].pfltRevCoeffs = S2_F016_ReverseCoeffs;
    iir_filters_table[IIR_S3][IIR_F016].pfltForCoeffs = S2_F016_ForwardCoeffs;
    iir_filters_table[IIR_S3][IIR_F016].numRevCoeffs = sizeof(S2_F016_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S3][IIR_F016].numForCoeffs = sizeof(S2_F016_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S3][IIR_F1].pfltRevCoeffs = S2_F1_ReverseCoeffs;
    iir_filters_table[IIR_S3][IIR_F1].pfltForCoeffs = S2_F1_ForwardCoeffs;
    iir_filters_table[IIR_S3][IIR_F1].numRevCoeffs = sizeof(S2_F1_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S3][IIR_F1].numForCoeffs = sizeof(S2_F1_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S3][IIR_F2].pfltRevCoeffs = S2_F2_ReverseCoeffs;
    iir_filters_table[IIR_S3][IIR_F2].pfltForCoeffs = S2_F2_ForwardCoeffs;
    iir_filters_table[IIR_S3][IIR_F2].numRevCoeffs = sizeof(S2_F2_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S3][IIR_F2].numForCoeffs = sizeof(S2_F2_ForwardCoeffs)/ sizeof(float);    

    iir_filters_table[IIR_S3][IIR_S5].pfltRevCoeffs = S2_F5_ReverseCoeffs;
    iir_filters_table[IIR_S3][IIR_S5].pfltForCoeffs = S2_F5_ForwardCoeffs;
    iir_filters_table[IIR_S3][IIR_S5].numRevCoeffs = sizeof(S2_F5_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S3][IIR_S5].numForCoeffs = sizeof(S2_F5_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S3][IIR_F10].pfltRevCoeffs = S2_F10_ReverseCoeffs;
    iir_filters_table[IIR_S3][IIR_F10].pfltForCoeffs = S2_F10_ForwardCoeffs;
    iir_filters_table[IIR_S3][IIR_F10].numRevCoeffs = sizeof(S2_F10_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S3][IIR_F10].numForCoeffs = sizeof(S2_F10_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S3][IIR_F20].pfltRevCoeffs = S2_F20_ReverseCoeffs;
    iir_filters_table[IIR_S3][IIR_F20].pfltForCoeffs = S2_F20_ForwardCoeffs;
    iir_filters_table[IIR_S3][IIR_F20].numRevCoeffs = sizeof(S2_F20_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S3][IIR_F20].numForCoeffs = sizeof(S2_F20_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S3][IIR_F50].pfltRevCoeffs = S2_F50_ReverseCoeffs;
    iir_filters_table[IIR_S3][IIR_F50].pfltForCoeffs = S2_F50_ForwardCoeffs;
    iir_filters_table[IIR_S3][IIR_F50].numRevCoeffs = sizeof(S2_F50_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S3][IIR_F50].numForCoeffs = sizeof(S2_F50_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S3][IIR_F100].pfltRevCoeffs = S2_F100_ReverseCoeffs;
    iir_filters_table[IIR_S3][IIR_F100].pfltForCoeffs = S2_F100_ForwardCoeffs;
    iir_filters_table[IIR_S3][IIR_F100].numRevCoeffs = sizeof(S2_F100_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S3][IIR_F100].numForCoeffs = sizeof(S2_F100_ForwardCoeffs)/ sizeof(float);
    ///12800
    iir_filters_table[IIR_S4][IIR_F016].pfltRevCoeffs = S3_F016_ReverseCoeffs;
    iir_filters_table[IIR_S4][IIR_F016].pfltForCoeffs = S3_F016_ForwardCoeffs;
    iir_filters_table[IIR_S4][IIR_F016].numRevCoeffs = sizeof(S3_F016_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S4][IIR_F016].numForCoeffs = sizeof(S3_F016_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S4][IIR_F1].pfltRevCoeffs = S3_F1_ReverseCoeffs;
    iir_filters_table[IIR_S4][IIR_F1].pfltForCoeffs = S3_F1_ForwardCoeffs;
    iir_filters_table[IIR_S4][IIR_F1].numRevCoeffs = sizeof(S3_F1_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S4][IIR_F1].numForCoeffs = sizeof(S3_F1_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S4][IIR_F2].pfltRevCoeffs = S3_F2_ReverseCoeffs;
    iir_filters_table[IIR_S4][IIR_F2].pfltForCoeffs = S3_F2_ForwardCoeffs;
    iir_filters_table[IIR_S4][IIR_F2].numRevCoeffs = sizeof(S3_F2_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S4][IIR_F2].numForCoeffs = sizeof(S3_F2_ForwardCoeffs)/ sizeof(float);    

    iir_filters_table[IIR_S4][IIR_S5].pfltRevCoeffs = S3_F5_ReverseCoeffs;
    iir_filters_table[IIR_S4][IIR_S5].pfltForCoeffs = S3_F5_ForwardCoeffs;
    iir_filters_table[IIR_S4][IIR_S5].numRevCoeffs = sizeof(S3_F5_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S4][IIR_S5].numForCoeffs = sizeof(S3_F5_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S4][IIR_F10].pfltRevCoeffs = S3_F10_ReverseCoeffs;
    iir_filters_table[IIR_S4][IIR_F10].pfltForCoeffs = S3_F10_ForwardCoeffs;
    iir_filters_table[IIR_S4][IIR_F10].numRevCoeffs = sizeof(S3_F10_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S4][IIR_F10].numForCoeffs = sizeof(S3_F10_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S4][IIR_F20].pfltRevCoeffs = S3_F20_ReverseCoeffs;
    iir_filters_table[IIR_S4][IIR_F20].pfltForCoeffs = S3_F20_ForwardCoeffs;
    iir_filters_table[IIR_S4][IIR_F20].numRevCoeffs = sizeof(S3_F20_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S4][IIR_F20].numForCoeffs = sizeof(S3_F20_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S4][IIR_F50].pfltRevCoeffs = S3_F50_ReverseCoeffs;
    iir_filters_table[IIR_S4][IIR_F50].pfltForCoeffs = S3_F50_ForwardCoeffs;
    iir_filters_table[IIR_S4][IIR_F50].numRevCoeffs = sizeof(S3_F50_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S4][IIR_F50].numForCoeffs = sizeof(S3_F50_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S4][IIR_F100].pfltRevCoeffs = S3_F100_ReverseCoeffs;
    iir_filters_table[IIR_S4][IIR_F100].pfltForCoeffs = S3_F100_ForwardCoeffs;
    iir_filters_table[IIR_S4][IIR_F100].numRevCoeffs = sizeof(S3_F100_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S4][IIR_F100].numForCoeffs = sizeof(S3_F100_ForwardCoeffs)/ sizeof(float);
    ///6400
    iir_filters_table[IIR_S5][IIR_F016].pfltRevCoeffs = S4_F016_ReverseCoeffs;
    iir_filters_table[IIR_S5][IIR_F016].pfltForCoeffs = S4_F016_ForwardCoeffs;
    iir_filters_table[IIR_S5][IIR_F016].numRevCoeffs = sizeof(S4_F016_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S5][IIR_F016].numForCoeffs = sizeof(S4_F016_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S5][IIR_F1].pfltRevCoeffs = S4_F1_ReverseCoeffs;
    iir_filters_table[IIR_S5][IIR_F1].pfltForCoeffs = S4_F1_ForwardCoeffs;
    iir_filters_table[IIR_S5][IIR_F1].numRevCoeffs = sizeof(S4_F1_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S5][IIR_F1].numForCoeffs = sizeof(S4_F1_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S5][IIR_F2].pfltRevCoeffs = S4_F2_ReverseCoeffs;
    iir_filters_table[IIR_S5][IIR_F2].pfltForCoeffs = S4_F2_ForwardCoeffs;
    iir_filters_table[IIR_S5][IIR_F2].numRevCoeffs = sizeof(S4_F2_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S5][IIR_F2].numForCoeffs = sizeof(S4_F2_ForwardCoeffs)/ sizeof(float);    

    iir_filters_table[IIR_S5][IIR_S5].pfltRevCoeffs = S4_F5_ReverseCoeffs;
    iir_filters_table[IIR_S5][IIR_S5].pfltForCoeffs = S4_F5_ForwardCoeffs;
    iir_filters_table[IIR_S5][IIR_S5].numRevCoeffs = sizeof(S4_F5_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S5][IIR_S5].numForCoeffs = sizeof(S4_F5_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S5][IIR_F10].pfltRevCoeffs = S4_F10_ReverseCoeffs;
    iir_filters_table[IIR_S5][IIR_F10].pfltForCoeffs = S4_F10_ForwardCoeffs;
    iir_filters_table[IIR_S5][IIR_F10].numRevCoeffs = sizeof(S4_F10_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S5][IIR_F10].numForCoeffs = sizeof(S4_F10_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S5][IIR_F20].pfltRevCoeffs = S4_F20_ReverseCoeffs;
    iir_filters_table[IIR_S5][IIR_F20].pfltForCoeffs = S4_F20_ForwardCoeffs;
    iir_filters_table[IIR_S5][IIR_F20].numRevCoeffs = sizeof(S4_F20_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S5][IIR_F20].numForCoeffs = sizeof(S4_F20_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S5][IIR_F50].pfltRevCoeffs = S4_F50_ReverseCoeffs;
    iir_filters_table[IIR_S5][IIR_F50].pfltForCoeffs = S4_F50_ForwardCoeffs;
    iir_filters_table[IIR_S5][IIR_F50].numRevCoeffs = sizeof(S4_F50_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S5][IIR_F50].numForCoeffs = sizeof(S4_F50_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S5][IIR_F100].pfltRevCoeffs = S4_F100_ReverseCoeffs;
    iir_filters_table[IIR_S5][IIR_F100].pfltForCoeffs = S4_F100_ForwardCoeffs;
    iir_filters_table[IIR_S5][IIR_F100].numRevCoeffs = sizeof(S4_F100_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S5][IIR_F100].numForCoeffs = sizeof(S4_F100_ForwardCoeffs)/ sizeof(float);
    ///5120
    iir_filters_table[IIR_S6][IIR_F016].pfltRevCoeffs = S5_F016_ReverseCoeffs;
    iir_filters_table[IIR_S6][IIR_F016].pfltForCoeffs = S5_F016_ForwardCoeffs;
    iir_filters_table[IIR_S6][IIR_F016].numRevCoeffs = sizeof(S5_F016_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S6][IIR_F016].numForCoeffs = sizeof(S5_F016_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S6][IIR_F1].pfltRevCoeffs = S5_F1_ReverseCoeffs;
    iir_filters_table[IIR_S6][IIR_F1].pfltForCoeffs = S5_F1_ForwardCoeffs;
    iir_filters_table[IIR_S6][IIR_F1].numRevCoeffs = sizeof(S5_F1_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S6][IIR_F1].numForCoeffs = sizeof(S5_F1_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S6][IIR_F2].pfltRevCoeffs = S5_F2_ReverseCoeffs;
    iir_filters_table[IIR_S6][IIR_F2].pfltForCoeffs = S5_F2_ForwardCoeffs;
    iir_filters_table[IIR_S6][IIR_F2].numRevCoeffs = sizeof(S5_F2_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S6][IIR_F2].numForCoeffs = sizeof(S5_F2_ForwardCoeffs)/ sizeof(float);    

    iir_filters_table[IIR_S6][IIR_S6].pfltRevCoeffs = S5_F5_ReverseCoeffs;
    iir_filters_table[IIR_S6][IIR_S6].pfltForCoeffs = S5_F5_ForwardCoeffs;
    iir_filters_table[IIR_S6][IIR_S6].numRevCoeffs = sizeof(S5_F5_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S6][IIR_S6].numForCoeffs = sizeof(S5_F5_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S6][IIR_F10].pfltRevCoeffs = S5_F10_ReverseCoeffs;
    iir_filters_table[IIR_S6][IIR_F10].pfltForCoeffs = S5_F10_ForwardCoeffs;
    iir_filters_table[IIR_S6][IIR_F10].numRevCoeffs = sizeof(S5_F10_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S6][IIR_F10].numForCoeffs = sizeof(S5_F10_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S6][IIR_F20].pfltRevCoeffs = S5_F20_ReverseCoeffs;
    iir_filters_table[IIR_S6][IIR_F20].pfltForCoeffs = S5_F20_ForwardCoeffs;
    iir_filters_table[IIR_S6][IIR_F20].numRevCoeffs = sizeof(S5_F20_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S6][IIR_F20].numForCoeffs = sizeof(S5_F20_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S6][IIR_F50].pfltRevCoeffs = S5_F50_ReverseCoeffs;
    iir_filters_table[IIR_S6][IIR_F50].pfltForCoeffs = S5_F50_ForwardCoeffs;
    iir_filters_table[IIR_S6][IIR_F50].numRevCoeffs = sizeof(S5_F50_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S6][IIR_F50].numForCoeffs = sizeof(S5_F50_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S6][IIR_F100].pfltRevCoeffs = S5_F100_ReverseCoeffs;
    iir_filters_table[IIR_S6][IIR_F100].pfltForCoeffs = S5_F100_ForwardCoeffs;
    iir_filters_table[IIR_S6][IIR_F100].numRevCoeffs = sizeof(S5_F100_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S6][IIR_F100].numForCoeffs = sizeof(S5_F100_ForwardCoeffs)/ sizeof(float);
    ///2560
    iir_filters_table[IIR_S7][IIR_F016].pfltRevCoeffs = S6_F016_ReverseCoeffs;
    iir_filters_table[IIR_S7][IIR_F016].pfltForCoeffs = S6_F016_ForwardCoeffs;
    iir_filters_table[IIR_S7][IIR_F016].numRevCoeffs = sizeof(S6_F016_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S7][IIR_F016].numForCoeffs = sizeof(S6_F016_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S7][IIR_F1].pfltRevCoeffs = S6_F1_ReverseCoeffs;
    iir_filters_table[IIR_S7][IIR_F1].pfltForCoeffs = S6_F1_ForwardCoeffs;
    iir_filters_table[IIR_S7][IIR_F1].numRevCoeffs = sizeof(S6_F1_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S7][IIR_F1].numForCoeffs = sizeof(S6_F1_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S7][IIR_F2].pfltRevCoeffs = S6_F2_ReverseCoeffs;
    iir_filters_table[IIR_S7][IIR_F2].pfltForCoeffs = S6_F2_ForwardCoeffs;
    iir_filters_table[IIR_S7][IIR_F2].numRevCoeffs = sizeof(S6_F2_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S7][IIR_F2].numForCoeffs = sizeof(S6_F2_ForwardCoeffs)/ sizeof(float);    

    iir_filters_table[IIR_S7][IIR_S7].pfltRevCoeffs = S6_F5_ReverseCoeffs;
    iir_filters_table[IIR_S7][IIR_S7].pfltForCoeffs = S6_F5_ForwardCoeffs;
    iir_filters_table[IIR_S7][IIR_S7].numRevCoeffs = sizeof(S6_F5_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S7][IIR_S7].numForCoeffs = sizeof(S6_F5_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S7][IIR_F10].pfltRevCoeffs = S6_F10_ReverseCoeffs;
    iir_filters_table[IIR_S7][IIR_F10].pfltForCoeffs = S6_F10_ForwardCoeffs;
    iir_filters_table[IIR_S7][IIR_F10].numRevCoeffs = sizeof(S6_F10_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S7][IIR_F10].numForCoeffs = sizeof(S6_F10_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S7][IIR_F20].pfltRevCoeffs = S6_F20_ReverseCoeffs;
    iir_filters_table[IIR_S7][IIR_F20].pfltForCoeffs = S6_F20_ForwardCoeffs;
    iir_filters_table[IIR_S7][IIR_F20].numRevCoeffs = sizeof(S6_F20_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S7][IIR_F20].numForCoeffs = sizeof(S6_F20_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S7][IIR_F50].pfltRevCoeffs = S6_F50_ReverseCoeffs;
    iir_filters_table[IIR_S7][IIR_F50].pfltForCoeffs = S6_F50_ForwardCoeffs;
    iir_filters_table[IIR_S7][IIR_F50].numRevCoeffs = sizeof(S6_F50_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S7][IIR_F50].numForCoeffs = sizeof(S6_F50_ForwardCoeffs)/ sizeof(float);

    iir_filters_table[IIR_S7][IIR_F100].pfltRevCoeffs = S6_F100_ReverseCoeffs;
    iir_filters_table[IIR_S7][IIR_F100].pfltForCoeffs = S6_F100_ForwardCoeffs;
    iir_filters_table[IIR_S7][IIR_F100].numRevCoeffs = sizeof(S6_F100_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S7][IIR_F100].numForCoeffs = sizeof(S6_F100_ForwardCoeffs)/ sizeof(float);
    ///1280
    iir_filters_table[IIR_S8][IIR_F016].pfltRevCoeffs = S7_F016_ReverseCoeffs;
    iir_filters_table[IIR_S8][IIR_F016].pfltForCoeffs = S7_F016_ForwardCoeffs;
    iir_filters_table[IIR_S8][IIR_F016].numRevCoeffs = sizeof(S7_F016_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S8][IIR_F016].numForCoeffs = sizeof(S7_F016_ForwardCoeffs)/ sizeof(float);
    
    iir_filters_table[IIR_S8][IIR_F1].pfltRevCoeffs = S7_F1_ReverseCoeffs;
    iir_filters_table[IIR_S8][IIR_F1].pfltForCoeffs = S7_F1_ForwardCoeffs;
    iir_filters_table[IIR_S8][IIR_F1].numRevCoeffs = sizeof(S7_F1_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S8][IIR_F1].numForCoeffs = sizeof(S7_F1_ForwardCoeffs)/ sizeof(float);
    
    iir_filters_table[IIR_S8][IIR_F2].pfltRevCoeffs = S7_F2_ReverseCoeffs;
    iir_filters_table[IIR_S8][IIR_F2].pfltForCoeffs = S7_F2_ForwardCoeffs;
    iir_filters_table[IIR_S8][IIR_F2].numRevCoeffs = sizeof(S7_F2_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S8][IIR_F2].numForCoeffs = sizeof(S7_F2_ForwardCoeffs)/ sizeof(float);    

    iir_filters_table[IIR_S8][IIR_S5].pfltRevCoeffs = S7_F5_ReverseCoeffs;
    iir_filters_table[IIR_S8][IIR_S5].pfltForCoeffs = S7_F5_ForwardCoeffs;
    iir_filters_table[IIR_S8][IIR_S5].numRevCoeffs = sizeof(S7_F5_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S8][IIR_S5].numForCoeffs = sizeof(S7_F5_ForwardCoeffs)/ sizeof(float);
    
    iir_filters_table[IIR_S8][IIR_F10].pfltRevCoeffs = S7_F10_ReverseCoeffs;
    iir_filters_table[IIR_S8][IIR_F10].pfltForCoeffs = S7_F10_ForwardCoeffs;
    iir_filters_table[IIR_S8][IIR_F10].numRevCoeffs = sizeof(S7_F10_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S8][IIR_F10].numForCoeffs = sizeof(S7_F10_ForwardCoeffs)/ sizeof(float);
    
    iir_filters_table[IIR_S8][IIR_F20].pfltRevCoeffs = S7_F20_ReverseCoeffs;
    iir_filters_table[IIR_S8][IIR_F20].pfltForCoeffs = S7_F20_ForwardCoeffs;
    iir_filters_table[IIR_S8][IIR_F20].numRevCoeffs = sizeof(S7_F20_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S8][IIR_F20].numForCoeffs = sizeof(S7_F20_ForwardCoeffs)/ sizeof(float);
    
    iir_filters_table[IIR_S8][IIR_F50].pfltRevCoeffs = S7_F50_ReverseCoeffs;
    iir_filters_table[IIR_S8][IIR_F50].pfltForCoeffs = S7_F50_ForwardCoeffs;
    iir_filters_table[IIR_S8][IIR_F50].numRevCoeffs = sizeof(S7_F50_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S8][IIR_F50].numForCoeffs = sizeof(S7_F50_ForwardCoeffs)/ sizeof(float);
    
    iir_filters_table[IIR_S8][IIR_F100].pfltRevCoeffs = S7_F100_ReverseCoeffs;
    iir_filters_table[IIR_S8][IIR_F100].pfltForCoeffs = S7_F100_ForwardCoeffs;
    iir_filters_table[IIR_S8][IIR_F100].numRevCoeffs = sizeof(S7_F100_ReverseCoeffs)/ sizeof(float);
    iir_filters_table[IIR_S8][IIR_F100].numForCoeffs = sizeof(S7_F100_ForwardCoeffs)/ sizeof(float);
}

/**************************************************************************************
*     Function:              cascade_iir_filter
*     Description:           Initialise FFT functionality
*     Return:                void
*     Parameter:   			 float* data,  int len, float* rev_coeffs,
*                            float* for_coeffs, int rev_coeffs_num
*                            int for_coeffs_num, float* w
*                            float  factor
**************************************************************************************/
static void cascade_iir_filter(float*   data,
                               int len,
                               const float*   rev_coeffs,     // 反向滤波器系数
                               const float*   for_coeffs,     // 正向滤波器系数
                               int rev_coeffs_num,       // 反向系数个数
                               int for_coeffs_num,       // 正向系数个数
                               float*   w,                    // IIR滤波器状态系数
                               float    factor)              // 速度计算的单位转换

{
    uint32_t m  = 0;
    int index = 0;
    float  a[3]  = {0.0};
    float  b[3]  = {0.0};
    if(data == NULL|| rev_coeffs == NULL|| for_coeffs ==NULL || w== NULL)
    {
        return;
    }
    //*s1:求级数
    //*s2:滤波
    int half_coeffs = rev_coeffs_num>>1;
	int loop =0;
    for (loop = 0; loop < half_coeffs; loop++)
    {
        int triple_loop = loop*3;

        b[0] = for_coeffs[triple_loop];
        b[1] = for_coeffs[1+triple_loop];
        b[2] = for_coeffs[2+triple_loop];

        int double_loop = loop<<1;
        a[1] = rev_coeffs[double_loop];
        a[2] = rev_coeffs[1+double_loop];
        //LOGD("b[0] = %E, b[1] = %E, b[2] = %E, a[1] = %E, a[2] = %E",  b[0],b[1],b[2],a[1],a[2]);
		//正向系数023，反向系数01
		//正向系数345，反射系数23
        for(index = 0; index < len; index++)
        {
            m     = loop*3;
            w[m]  = data[index] - a[1]*w[1+m]- a[2]*w[2+m];
            data[index] = (b[0]*w[m] + b[1]*w[1+m] + b[2]*w[2+m])*factor;
            w[2+m] = w[1+m];
            w[1+m] = w[m];
			//LOGD("cascade_iir_filter_data[%d] = %f",  index,  data[index]);
        }
    }
}

/**********************************************************************
* 函数名称: integrate_o1
* 功能描述: 进行一次积分
* 输入参数: args:
    o1->alpha   = 1.952648E-4;
    o1->gamma   = 9.996095E-1;
    o1->x_state = 0.0f;
    o1->y_state = 0.0f;
    o1->base.factor = 100.0f;
**********************************************************************/
void integrate_o1(float*  data,  int len)//,  float alpha,  float gamma,  float factor,  float  x_state,  float  y_state)
{
	LOGD("ENTERintegrate_o1");
    float fltTemp = 0.0f;
    float alpha   = 1.952648E-4;
	float gamma   = 9.996095E-1;
	float x_state = 0.0f;
	float y_state = 0.0f;
	float factor = 100.0f;
    // 入参判断
	if(data == NULL || len == 0)
    {
        return;
    }

	int u16Loop=0;
    for(u16Loop=0; u16Loop < len; u16Loop++)
    {
        fltTemp = ((x_state)+data[u16Loop])*alpha;
        fltTemp += (gamma*(y_state));
        x_state = data[u16Loop];
        y_state = fltTemp;
        data[u16Loop] = fltTemp*factor;
    }
}

/**********************************************************************
* 函数名称: integrate_o2
* 功能描述: 进行二次积分
* 输入参数: args:
    o2->alpha    = 1.906822E-8;
    o2->gamma    = 9.997238E-1;
    o2->beta     = 4.997239E-1;
    o2->x1_state = 0.0f;
    o2->x2_state = 0.0f;
    o2->y1_state = 0.0f;
    o2->y2_state = 0.0f;
    o2->base.factor = 10000.0f;
**********************************************************************/
void integrate_o2(float* data,  int len)//,  float alpha, float gamma, float beta,  float factor, float x1_state, float x2_state, float y1_state, float y2_state)
{
	LOGD("ENTERintegrate_o2");
    float fltTemp = 0.0f;
	float alpha = 1.906822E-8;
	float gamma = 9.997238E-1;
	float beta = 4.997239E-1;
	float factor = 10000.0f;
    float x1_state = 0.0f;
	float x2_state = 0.0f;
    float y1_state = 0.0f;
	float y2_state = 0.0f;
    // 入参判断
    if(data == NULL || len == 0)
    {
        return;
    }

	LOGD("alpha = %f, gamma =%f,  beta = %f,  factor = %f,  x1_state = %f,  x2_state = %f, y1_state = %f,  y1_state = %f",  alpha, gamma, beta, factor, x1_state, x2_state, y1_state, y2_state);

    int u16Loop = 0;
    for (u16Loop = 0; u16Loop < len; u16Loop++)
    {
        fltTemp = ( ( 2*( x1_state ) + ( x2_state ) + data[u16Loop] )*alpha
                  + ( gamma*( y1_state ) - beta*( y2_state ) ) )*2;
        y2_state = y1_state;
        y1_state = fltTemp;
        x2_state = x1_state;
        x1_state = data[u16Loop];
        data[u16Loop] = fltTemp*factor;
		LOGD("integrate_o2_data[%d] = %f", u16Loop, data[u16Loop]);
    }
}

/* 功能描述：IIR高通滤波入口函数 */
void  enter_iir_filter(float *src_data, int length,  int up_freq,  int lw_freq)
{
    LOGD("xin：enter_iir_filter_length = %d,  up_freq = %d,  lw_freq = %d", length, up_freq, lw_freq);
	int i=0;
    if(src_data == NULL)
    {
        return;
    }
	init_iir_filters();//初始化IIR滤波器系数

	tIIRFilter *pIIRFilter= NULL;
	pIIRFilter = get_iir_filter(get_smp_rate(up_freq),  lw_freq);//通过上下限，获取滤波系数

	float *state_para = NULL;
	state_para = (float*)malloc(pIIRFilter->numForCoeffs <<2);//正向系数个数
	if(state_para == NULL)
	{
		LOGD("state_para 分配内存失败！");
		return;
	}
	memset(state_para,  0,  sizeof(float)* pIIRFilter->numForCoeffs);

	cascade_iir_filter(src_data, length,  pIIRFilter->pfltRevCoeffs, pIIRFilter->pfltForCoeffs, pIIRFilter->numRevCoeffs, pIIRFilter->numForCoeffs, state_para, 1.0);

	#if 0
	/* for(i=0;i< length;i++)
	{
		 LOGD("enter_iir_filter_ret_data[%d] = %f", i, src_data[i]);
	} */
	#endif

	if(state_para != NULL)
	{
		free(state_para);
		state_para =NULL;
	}
}
