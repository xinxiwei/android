#ifndef __HH_FIRCOEFFS_H_HH__
#define __HH_FIRCOEFFS_H_HH__

#define UPPER_FRE_500      (500)
#define UPPER_FRE_1000     (1000)
#define UPPER_FRE_2000     (2000)
#define UPPER_FRE_2500     (2500)
#define UPPER_FRE_4000     (4000)
#define UPPER_FRE_5000     (5000)
#define UPPER_FRE_10000    (10000)
#define UPPER_FRE_20000    (20000)
#define UPPER_FRE_40000    (40000)

#define FIR_F500             (0)
#define FIR_F1000            (1)
#define FIR_F2000            (2)
#define FIR_F2500            (3)
#define FIR_F4000            (4)
#define FIR_F5000            (5)
#define FIR_F10000           (6)
#define FIR_F20000           (7)
#define FIR_F40000           (8)
#define FIR_FILTER_NUMBER    (9)

//分析频率为500Hz时，各阶OnceFIR的滤波器系数个数、抽样点数
#define  FIR_500_STAGE_NUMBER           (2)    //档位系数

#define  FIR_500_1STAGE_SIZE            (13)   //滤波器系数点数
#define  FIR_500_1STAGE_DECIFACTOR      (2)    //抽样点数

#define  FIR_500_2STAGE_SIZE            (65)   //滤波器系数点数
#define  FIR_500_2STAGE_DECIFACTOR      (2)    //抽样点数
//分析频率为1000Hz时，各阶OnceFIR的滤波器系数个数、抽样点数
#define  FIR_1000_STAGE_NUMBER          (1)

#define  FIR_1000_1STAGE_SIZE           (61)
#define  FIR_1000_1STAGE_DECIFACTOR     (2)
//分析频率为2500Hz时，各阶OnceFIR的滤波器系数个数、抽样点数
#define  FIR_2500_STAGE_NUMBER          (2)

#define  FIR_2500_1STAGE_SIZE           (13)
#define  FIR_2500_1STAGE_DECIFACTOR     (2)

#define  FIR_2500_2STAGE_SIZE           (65)
#define  FIR_2500_2STAGE_DECIFACTOR     (2)
//分析频率为5000Hz时，各阶OnceFIR的滤波器系数个数、抽样点数
#define  FIR_5000_STAGE_NUMBER          (1)

#define  FIR_5000_1STAGE_SIZE           (61)
#define  FIR_5000_1STAGE_DECIFACTOR     (2)

//对外声明函数
extern void  enter_fir_filter( float *src_data  , int length  ,  int up_freq);
#endif    /* __HH_FIRCOEFFS_H_HH__ */
