/***************************************************************************************
*  ( c ) Copyright 2017 ENNOVAR ,  All rights reserved
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
#include <fcntl.h>
#include <hardware/imx6q_iir_filter.h>
#include <hardware/IIRCoeffs.h>
/* IIR高通滤波器，用下限卡 */
extern float S0_F016_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM];
extern float S0_F016_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM];
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
// fir: 500 1000 2000 2500 4000 5000 10000 20000 40000 //上限频率
static int InvalidNumTable[NUM_SAMPLERATES][NUM_LOWER_FRE] ={
  {6.9E+5  , 1.1E+5  , 51200  , 20480  , 10240  , 12288  , 4096  , 3072}  ,  //102.4   上限40000
  {348160  , 56320  , 25600  , 10240  , 7680  , 5120  , 3072  , 1024}  ,     //51.2    上限20000
  {174080  , 28160  , 12800  , 10240  , 5120  , 3072  , 1024  , 512}  ,      //25.6    上限10000
  {87040  , 14080  , 6400  , 7168  , 2560  , 1792  , 512  , 256}  ,          //12.8    上限5000
  {69632  , 11264  , 10240  , 5120  , 3072  , 1228  , 307  , 307}  ,         //10.24   上限4000
  {43520  , 7040  , 6400  , 3200  , 1280  , 768  , 256  , 128}  ,            //6.4     上限2500
  {34816  , 5632  , 5120  , 2560  , 1024  , 614  , 307  , 102}  ,            //5.12    上限2000
  {17408  , 2816  , 2560  , 1280  , 640  , 307  , 128  , 77}  ,              //2.56    上限1000
  {8704  , 1408  , 1536  , 768  , 358  , 179  , 77  , 38}  ,                 //1.28    上限500

};

static int Get_InvalidNum_From_Table( int* table  ,  int  iir_frq  ){
    if(table == NULL)
    {
        return 0;
    }
     switch( iir_frq ){
	    case 0:{
            return table[IIR_F016];
	    }case 1:{
            return table[IIR_F1];
        }case 2:{
            return table[IIR_F2];
        }case 5:{
            return table[IIR_F5];
        }case 10:{
            return table[IIR_F10];
        }case 20:{
            return table[IIR_F20];
        }case 50:{
            return table[IIR_F50];
        }case 100:{
            return table[IIR_F100];
        }default:{
            return 0;
        }
    }
}

/* 功能描述：得到无效点数 */
int get_invalid_num(  int Upper_frq  ,  int Lower_frq  )
{
    switch( Upper_frq )
    {
        case 500:
        {
            return Get_InvalidNum_From_Table( &InvalidNumTable[8][0]  , Lower_frq );
        }
        case 1000:
        {
            return Get_InvalidNum_From_Table( &InvalidNumTable[7][0]  , Lower_frq );
        }
        case 2000:
        {
            return Get_InvalidNum_From_Table( &InvalidNumTable[6][0]  , Lower_frq );
        }
        case 2500:
        {
            return Get_InvalidNum_From_Table( &InvalidNumTable[5][0]  , Lower_frq );
        }
        case 4000:
        {
            return Get_InvalidNum_From_Table( &InvalidNumTable[4][0]  , Lower_frq );
        }
        case 5000:
        {
            return Get_InvalidNum_From_Table( &InvalidNumTable[3][0]  , Lower_frq );
        }
    case 10000:
        {
            return Get_InvalidNum_From_Table( &InvalidNumTable[2][0]  , Lower_frq );
        }
    case 20000:
        {
            return Get_InvalidNum_From_Table( &InvalidNumTable[1][0]  , Lower_frq );
        }
    case 40000:
        {
            return Get_InvalidNum_From_Table( &InvalidNumTable[0][0]  , Lower_frq );
        }
        default:
        {
            return Get_InvalidNum_From_Table( &InvalidNumTable[0][0]  , Lower_frq );
        }
    }
}

//IIR滤波器表
static tIIRFilter IIRFiltersTable[NUM_SAMPLERATES][NUM_LOWER_FRE];
/**************************************************************************************
*     Function:              _GetSampRate
*     Description:           find samprate for upper frequency
*     Return:                INT32U
*     Parameter:   			 u16Upper
*
**************************************************************************************/
static int _GetSampRate( int u16Upper )  // 输入上限，输出采样率
{
    if( UPPER_FRE_500 == u16Upper || UPPER_FRE_1000 == u16Upper || UPPER_FRE_2000 == u16Upper
        || UPPER_FRE_2500 == u16Upper || UPPER_FRE_4000  == u16Upper || UPPER_FRE_5000 == u16Upper
		|| UPPER_FRE_10000 == u16Upper|| UPPER_FRE_20000 == u16Upper || UPPER_FRE_40000 == u16Upper )
    {
        return ( 2.56*u16Upper );
    }
    else
    {
        return 102400;
    }
}
/**************************************************************************************
*     Function:              _GetFilterByFC
*     Description:           Initialise FFT functionality
*     Return:                void
*     Parameter:   			 float* data ,  int len , float* rev_coeffs ,
*
**************************************************************************************/
static tPIIRFilter _GetFilterByFC( int SRID ,  int FCID  ) //输入采样率 ，下限
{
    int index_sr = 0;
    int index_fc = 0;

    switch( SRID ) //采样率
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

    switch( FCID )//下限频率
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
    return &IIRFiltersTable[index_sr][index_fc];
}


static tIIRFilter gIIRFilter[NUM_SAMPLERATES] ={
	{SX_FX_REVERSCOEFFS_COMMON_NUM , SX_FX_FORWARDCOEFFS_COMMON_NUM , NULL , S0_F016_ReverseCoeffs , S0_F016_ForwardCoeffs} ,
	{SX_FX_REVERSCOEFFS_COMMON_NUM , SX_FX_FORWARDCOEFFS_COMMON_NUM , NULL , S0_F1_ReverseCoeffs ,   S0_F1_ForwardCoeffs} ,
	{SX_FX_REVERSCOEFFS_COMMON_NUM , SX_FX_FORWARDCOEFFS_COMMON_NUM , NULL , S0_F2_ReverseCoeffs ,   S0_F2_ForwardCoeffs} ,
	{SX_FX_REVERSCOEFFS_COMMON_NUM , SX_FX_FORWARDCOEFFS_COMMON_NUM , NULL , S0_F5_ReverseCoeffs ,   S0_F5_ForwardCoeffs} ,
	{SX_FX_REVERSCOEFFS_COMMON_NUM , SX_FX_FORWARDCOEFFS_COMMON_NUM , NULL , S0_F10_ReverseCoeffs ,  S0_F10_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM ,   NULL , S0_F20_ReverseCoeffs ,     S0_F20_ForwardCoeffs} ,
	{S0_F50_REVERSCOEFFS_NUM , S0_F50_FORWARDCOEFFS_NUM ,   NULL , S0_F50_ReverseCoeffs ,     S0_F50_ForwardCoeffs} ,
	{S0_F100_REVERSCOEFFS_NUM , S0_F100_FORWARDCOEFFS_NUM , NULL , S0_F100_ReverseCoeffs ,    S0_F100_ForwardCoeffs} ,

	{SX_FX_REVERSCOEFFS_COMMON_NUM , SX_FX_FORWARDCOEFFS_COMMON_NUM , NULL , S1_F016_ReverseCoeffs , S1_F016_ForwardCoeffs} ,
	{SX_FX_REVERSCOEFFS_COMMON_NUM , SX_FX_FORWARDCOEFFS_COMMON_NUM , NULL , S1_F1_ReverseCoeffs , S1_F1_ForwardCoeffs} ,
	{SX_FX_REVERSCOEFFS_COMMON_NUM , SX_FX_FORWARDCOEFFS_COMMON_NUM , NULL , S1_F2_ReverseCoeffs , S1_F2_ForwardCoeffs} ,
	{SX_FX_REVERSCOEFFS_COMMON_NUM , SX_FX_FORWARDCOEFFS_COMMON_NUM , NULL , S1_F5_ReverseCoeffs , S1_F5_ForwardCoeffs} ,
	{SX_FX_REVERSCOEFFS_COMMON_NUM , SX_FX_FORWARDCOEFFS_COMMON_NUM , NULL , S1_F10_ReverseCoeffs , S1_F10_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S1_F20_ReverseCoeffs , S1_F20_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S1_F50_ReverseCoeffs , S1_F50_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S1_F100_ReverseCoeffs , S1_F100_ForwardCoeffs} ,

	{SX_FX_REVERSCOEFFS_COMMON_NUM , SX_FX_FORWARDCOEFFS_COMMON_NUM , NULL , S2_F016_ReverseCoeffs , S2_F016_ForwardCoeffs} ,
	{SX_FX_REVERSCOEFFS_COMMON_NUM , SX_FX_FORWARDCOEFFS_COMMON_NUM , NULL , S2_F1_ReverseCoeffs , S2_F1_ForwardCoeffs} ,
	{SX_FX_REVERSCOEFFS_COMMON_NUM , SX_FX_FORWARDCOEFFS_COMMON_NUM , NULL , S2_F2_ReverseCoeffs , S2_F2_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM ,  NULL , S2_F5_ReverseCoeffs , S2_F5_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM ,  NULL , S2_F10_ReverseCoeffs , S2_F10_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM ,  NULL , S2_F20_ReverseCoeffs , S2_F20_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM ,  NULL , S2_F50_ReverseCoeffs , S2_F50_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM ,  NULL , S2_F100_ReverseCoeffs , S2_F100_ForwardCoeffs} ,

	{SX_FX_REVERSCOEFFS_COMMON_NUM , SX_FX_FORWARDCOEFFS_COMMON_NUM , NULL , S3_F016_ReverseCoeffs , S3_F016_ForwardCoeffs} ,
	{SX_FX_REVERSCOEFFS_COMMON_NUM , SX_FX_FORWARDCOEFFS_COMMON_NUM , NULL , S3_F1_ReverseCoeffs , S3_F1_ForwardCoeffs} ,
	{SX_FX_REVERSCOEFFS_COMMON_NUM , SX_FX_FORWARDCOEFFS_COMMON_NUM , NULL , S3_F2_ReverseCoeffs , S3_F2_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S3_F5_ReverseCoeffs ,  S3_F5_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S3_F10_ReverseCoeffs , S3_F10_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S3_F20_ReverseCoeffs , S3_F20_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S3_F50_ReverseCoeffs , S3_F50_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S3_F100_ReverseCoeffs , S3_F100_ForwardCoeffs} ,

	{SX_FX_REVERSCOEFFS_COMMON_NUM , SX_FX_FORWARDCOEFFS_COMMON_NUM , NULL , S4_F016_ReverseCoeffs , S4_F016_ForwardCoeffs} ,
	{SX_FX_REVERSCOEFFS_COMMON_NUM , SX_FX_FORWARDCOEFFS_COMMON_NUM , NULL , S4_F1_ReverseCoeffs , S4_F1_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S4_F2_ReverseCoeffs , S4_F2_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S4_F5_ReverseCoeffs , S4_F5_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S4_F10_ReverseCoeffs , S4_F10_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S4_F20_ReverseCoeffs , S4_F20_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S4_F50_ReverseCoeffs , S4_F50_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S4_F100_ReverseCoeffs , S4_F100_ForwardCoeffs} ,

	{SX_FX_REVERSCOEFFS_COMMON_NUM , SX_FX_FORWARDCOEFFS_COMMON_NUM , NULL , S5_F016_ReverseCoeffs , S5_F016_ForwardCoeffs} ,
	{SX_FX_REVERSCOEFFS_COMMON_NUM , SX_FX_FORWARDCOEFFS_COMMON_NUM , NULL , S5_F1_ReverseCoeffs , S5_F1_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S5_F2_ReverseCoeffs , S5_F2_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S5_F5_ReverseCoeffs , S5_F5_ReverseCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S5_F10_ReverseCoeffs , S5_F10_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S5_F20_ReverseCoeffs , S5_F20_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S5_F50_ReverseCoeffs , S5_F50_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S5_F100_ReverseCoeffs , S5_F100_ForwardCoeffs} ,

	{SX_FX_REVERSCOEFFS_COMMON_NUM , SX_FX_FORWARDCOEFFS_COMMON_NUM , NULL , S6_F016_ReverseCoeffs , S6_F016_ForwardCoeffs} ,
	{SX_FX_REVERSCOEFFS_COMMON_NUM , SX_FX_FORWARDCOEFFS_COMMON_NUM , NULL , S6_F1_ReverseCoeffs , S6_F1_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S6_F2_ReverseCoeffs , S6_F2_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S6_F5_ReverseCoeffs , S6_F5_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S6_F10_ReverseCoeffs , S6_F10_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S6_F20_ReverseCoeffs , S6_F20_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S6_F50_ReverseCoeffs , S6_F50_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S6_F100_ReverseCoeffs , S6_F100_ForwardCoeffs} ,


	{SX_FX_REVERSCOEFFS_COMMON_NUM , SX_FX_FORWARDCOEFFS_COMMON_NUM , NULL , S7_F016_ReverseCoeffs , S7_F016_ForwardCoeffs} ,
	{SX_FX_REVERSCOEFFS_COMMON_NUM , SX_FX_FORWARDCOEFFS_COMMON_NUM , NULL , S7_F1_ReverseCoeffs , S7_F1_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S7_F2_ReverseCoeffs , S7_F2_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S7_F5_ReverseCoeffs , S7_F5_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S7_F10_ReverseCoeffs , S7_F10_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S7_F20_ReverseCoeffs , S7_F20_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S7_F50_ReverseCoeffs , S7_F50_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S7_F100_ReverseCoeffs , S7_F100_ForwardCoeffs} ,

	{SX_FX_REVERSCOEFFS_COMMON_NUM , SX_FX_FORWARDCOEFFS_COMMON_NUM , NULL , S8_F016_ReverseCoeffs , S8_F016_ForwardCoeffs} ,
	{SX_FX_REVERSCOEFFS_COMMON_NUM , SX_FX_FORWARDCOEFFS_COMMON_NUM , NULL , S8_F1_ReverseCoeffs , S8_F1_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S8_F2_ReverseCoeffs , S8_F2_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S8_F5_ReverseCoeffs , S8_F5_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S8_F10_ReverseCoeffs , S8_F10_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S8_F20_ReverseCoeffs , S8_F20_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S8_F50_ReverseCoeffs , S8_F50_ForwardCoeffs} ,
	{S0_F20_REVERSCOEFFS_NUM , S0_F20_FORWARDCOEFFS_NUM , NULL , S8_F100_ReverseCoeffs , S8_F100_ForwardCoeffs} ,

};


void  IIRFilterTable_Init( void )
{
   int i = 0, j=0;
   for(  i=0; i< NUM_SAMPLERATES ; i++ ){	//9
	    for(  j= 0; j< NUM_LOWER_FRE; j++ ){ //8
		    memcpy( &IIRFiltersTable[i][j]   , &gIIRFilter[j] , sizeof( tIIRFilter ) );
	    }
   }
   //LOGD( "IIRFiltersTable = %d" , IIRFiltersTable[8][0].numRevCoeffs  );
   return ;
}

/**************************************************************************************
*     Function:              cascade_iir_filter
*     Description:           Initialise FFT functionality
*     Return:                void
*     Parameter:   			 float* data ,  int len , float* rev_coeffs ,
*                            float* for_coeffs , int rev_coeffs_num
*                            int for_coeffs_num , float* w
*                            float  factor
**************************************************************************************/
static void cascade_iir_filter( float*   data ,
                               int len ,
                               const float*   rev_coeffs ,     // 反向滤波器系数
                               const float*   for_coeffs ,     // 正向滤波器系数
                               int rev_coeffs_num ,       // 反向系数个数
                               int for_coeffs_num ,       // 正向系数个数
                               float*   w ,                    // IIR滤波器状态系数
                               float    factor )              // 速度计算的单位转换

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
    for ( loop = 0; loop < half_coeffs; loop++ )
    {
        int triple_loop = loop*3;

        b[0] = for_coeffs[triple_loop];
        b[1] = for_coeffs[1+triple_loop];
        b[2] = for_coeffs[2+triple_loop];

        int double_loop = loop<<1;
        a[1] = rev_coeffs[double_loop];
        a[2] = rev_coeffs[1+double_loop];

        for( index = 0; index < len; index++ )
        {
            m     = loop*3;
            w[m]  = data[index] - a[1]*w[1+m]- a[2]*w[2+m];
            data[index] = ( b[0]*w[m] + b[1]*w[1+m] + b[2]*w[2+m] )*factor;
            w[2+m]= w[1+m];
            w[1+m]= w[m];
			//LOGD( "cascade_iir_filter_data[%d] = %f" ,  index ,  data[index] );
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
* 输出参数: void
* 返 回 值: void
* 其他说明: void
* 修改日期       版本号     作者         修改内容
* -----------------------------------------------
* 2016/11/10     V1.0       tyx          新建
* 2016/11/27     V1.1       Kous         按照项目要求改造函数
**********************************************************************/
void integrate_o1( float*  data ,  int len )// ,  float alpha ,  float gamma ,  float factor ,  float  x_state ,  float  y_state )
{
	LOGD( "ENTERintegrate_o1" );
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
    for( u16Loop=0; u16Loop < len; u16Loop++ )
    {
        fltTemp = ( ( x_state )+data[u16Loop] )*alpha;
        fltTemp += ( gamma*( y_state ) );
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
* 输出参数: void
* 返 回 值: void
* 其他说明: void
* 修改日期       版本号     作者         修改内容
* -----------------------------------------------
* 2016/11/10     V1.0       tyx          新建
* 2016/11/27     V1.1       Kous         按照项目要求改造函数
**********************************************************************/
void integrate_o2( float* data ,  int len )// ,  float alpha , float gamma , float beta ,  float factor , float x1_state , float x2_state , float y1_state , float y2_state )
{
	LOGD( "ENTERintegrate_o2" );
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

	LOGD( "alpha = %f , gamma =%f ,  beta = %f ,  factor = %f ,  x1_state = %f ,  x2_state = %f , y1_state = %f ,  y1_state = %f" ,  alpha , gamma , beta , factor , x1_state , x2_state , y1_state , y2_state );

    int u16Loop = 0;
    for (  u16Loop = 0; u16Loop < len; u16Loop++ )
    {
        fltTemp = ( ( 2*( x1_state ) + ( x2_state ) + data[u16Loop] )*alpha
                  + ( gamma*( y1_state ) - beta*( y2_state ) ) )*2;
        y2_state = y1_state;
        y1_state = fltTemp;
        x2_state = x1_state;
        x1_state = data[u16Loop];
        data[u16Loop] = fltTemp*factor;
		LOGD( "integrate_o2_data[%d] = %f" , u16Loop , data[u16Loop] );
    }
}

/* 功能描述：IIR高通滤波入口函数 */
void  enter_iir_filter( float *src_data  , int length  ,  int up_freq  ,  int lw_freq )
{
    //LOGD( "xin：enter_iir_filter_length = %d ,  up_freq = %d ,  lw_freq = %d" , length , up_freq , lw_freq );
	int i=0;
    if(src_data == NULL)
    {
        return;
    }
	IIRFilterTable_Init(  );//初始化IIR滤波器系数

	tIIRFilter *pIIRFilter= NULL;
	pIIRFilter = _GetFilterByFC( _GetSampRate( up_freq )  ,  lw_freq  );//通过上下限，获取滤波系数

	float *state_para = NULL;
	if ( state_para ==  NULL)
	{
		state_para = ( float* )malloc( pIIRFilter->numForCoeffs <<2  );
		if( state_para == NULL )
		{
			LOGD( "state_para 分配内存失败！" );
			return;
		}
		memset( state_para ,  0 ,  sizeof( float )* pIIRFilter->numForCoeffs );
	}

	cascade_iir_filter( src_data   , length  ,  pIIRFilter->pfltRevCoeffs , pIIRFilter->pfltForCoeffs , pIIRFilter->numRevCoeffs , pIIRFilter->numForCoeffs   , state_para , 1.0 );


	#if 0
	/* for( i=0;i< length;i++ )
	{
		 LOGD( "enter_iir_filter_ret_data[%d] = %f" , i , src_data[i] );
	} */
	#endif

	if( state_para != NULL)
	{
		free( state_para );
		state_para =NULL;
	}
}
