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
#include <stdint.h>
#include "imx6q_fir_filter.h"

/* FIR低通滤波器，用上限卡 */

/*分析频率为500Hz时，各阶OnceFIR的滤波器系数
Input Sampling Rate = 5120Hz
Output Sampling Rate = 1280 Hz
Decimation Factor = 4
Fpass = 500 Hz, Pass Ripple < 0.01dB
Fstop = 640 Hz, Stopband Attenuation > 60dB*/
//Stage 1 : Decimation_Factor = 2, FIR_Parameters = 13
static  float fir_500_1stage_coeffs[FIR_500_1STAGE_SIZE]={
        2.148493E-3,
		1.043179E-2,
		-1.033500E-2,
		-5.929095E-2,
		2.189099E-2,
		2.991959E-1,
		4.724094E-1,
		2.991959E-1,
		2.189099E-2,
		-5.929095E-2,
		-1.033500E-2,
		1.043179E-2,
		2.148493E-3
};
//Stage 2 : Decimation_Factor = 2, FIR_Parameters = 65
static  float fir_500_2stage_coeffs[FIR_500_2STAGE_SIZE] = {
        5.105268E-4,
		-3.414229E-4,
		-8.493122E-4,
		2.767892E-5,
		1.293682E-3,
		3.828157E-4,
		-1.915433E-3,
		-1.373469E-3,
		2.254731E-3,
		2.833411E-3,
		-2.136249E-3,
		-4.810961E-3,
		1.102657E-3,
		7.000915E-3,
		1.168291E-3,
		-8.978565E-3,
		-4.978769E-3,
		1.004102E-2,
		1.040475E-2,
		-9.318693E-3,
		-1.729652E-2,
		5.741392E-3,
		2.521301E-2,
		2.007330E-3,
		-3.347084E-2,
		-1.592129E-2,
		4.121663E-2,
		4.038584E-2,
		-4.756181E-2,
		-9.119646E-2,
		5.172616E-2,
		3.131762E-1,
		4.468226E-1,
		3.131762E-1,
		5.172616E-2,
		-9.119646E-2,
		-4.756181E-2,
		4.038584E-2,
		4.121663E-2,
		-1.592129E-2,
		-3.347084E-2,
		2.007330E-3,
		2.521301E-2,
		5.741392E-3,
		-1.729652E-2,
		-9.318693E-3,
		1.040475E-2,
		1.004102E-2,
		-4.978769E-3,
		-8.978565E-3,
		1.168291E-3,
		7.000915E-3,
		1.102657E-3,
		-4.810961E-3,
		-2.136249E-3,
		2.833411E-3,
		2.254731E-3,
		-1.373469E-3,
		-1.915433E-3,
		3.828157E-4,
		1.293682E-3,
		2.767892E-5,
		-8.493122E-4,
		-3.414229E-4,
		5.105268E-4
};


/*分析频率为1000Hz时，各阶OnceFIR的滤波器系数
Input Sampling Rate = 5120Hz
Output Sampling Rate = 2560 Hz
Decimation Factor = 2
Fpass = 1000 Hz, Pass Ripple < 0.01dB
Fstop = 1280 Hz, Stopband Attenuation > 60dB*/
//Stage 1 : Decimation_Factor = 2, FIR_Parameters = 61
static  float fir_1000_1stage_coeffs[FIR_1000_1STAGE_SIZE]={
        -5.500899E-4,
		4.813905E-4,
		1.194679E-3,
		1.384363E-4,
		-1.650045E-3,
		-7.896170E-4,
		2.281085E-3,
		2.187035E-3,
		-2.414264E-3,
		-4.132936E-3,
		1.739288E-3,
		6.489708E-3,
		2.221724E-4,
		-8.783310E-3,
		-3.817145E-3,
		1.030450E-2,
		9.217067E-3,
		-1.009554E-2,
		-1.630684E-2,
		6.990504E-3,
		2.464662E-2,
		4.390378E-4,
		-3.349330E-2,
		-1.427091E-2,
		4.189396E-2,
		3.893231E-2,
		-4.883438E-2,
		-9.020027E-2,
		5.341432E-2,
		3.128220E-1,
		4.449855E-1,
		3.128220E-1,
		5.341432E-2,
		-9.020027E-2,
		-4.883438E-2,
		3.893231E-2,
		4.189396E-2,
		-1.427091E-2,
		-3.349330E-2,
		4.390378E-4,
		2.464662E-2,
		6.990504E-3,
		-1.630684E-2,
		-1.009554E-2,
		9.217067E-3,
		1.030450E-2,
		-3.817145E-3,
		-8.783310E-3,
		2.221724E-4,
		6.489708E-3,
		1.739288E-3,
		-4.132936E-3,
		-2.414264E-3,
		2.187035E-3,
		2.281085E-3,
		-7.896170E-4,
		-1.650045E-3,
		1.384363E-4,
		1.194679E-3,
		4.813905E-4,
		-5.500899E-4
	};


/*分析频率为2500Hz时，各阶OnceFIR的滤波器系数
Input Sampling Rate = 25600Hz
Output Sampling Rate = 6400 Hz
Decimation Factor = 4
Fpass = 2500 Hz, Pass Ripple < 0.01dB
Fstop = 3200 Hz, Stopband Attenuation > 60dB*/
//Stage 1 : Decimation_Factor = 2, FIR_Parameters = 13
static  float fir_2500_1stage_coeffs[FIR_2500_1STAGE_SIZE]={
        2.148493E-3,
		1.043179E-2,
		-1.033500E-2,
		-5.929095E-2,
		2.189099E-2,
		2.991959E-1,
		4.724094E-1,
		2.991959E-1,
		2.189099E-2,
		-5.929095E-2,
		-1.033500E-2,
		1.043179E-2,
		2.148493E-3
	};

//Stage 2 : Decimation_Factor = 2, FIR_Parameters = 65
static  float fir_2500_2stage_coeffs[FIR_2500_2STAGE_SIZE]={
        5.105268E-4,
		-3.414229E-4,
		-8.493122E-4,
		2.767892E-5,
		1.293682E-3,
		3.828157E-4,
		-1.915433E-3,
		-1.373469E-3,
		2.254731E-3,
		2.833411E-3,
		-2.136249E-3,
		-4.810961E-3,
		1.102657E-3,
		7.000915E-3,
		1.168291E-3,
		-8.978565E-3,
		-4.978769E-3,
		1.004102E-2,
		1.040475E-2,
		-9.318693E-3,
		-1.729652E-2,
		5.741392E-3,
		2.521301E-2,
		2.007330E-3,
		-3.347084E-2,
		-1.592129E-2,
		4.121663E-2,
		4.038584E-2,
		-4.756181E-2,
		-9.119646E-2,
		5.172616E-2,
		3.131762E-1,
		4.468226E-1,
		3.131762E-1,
		5.172616E-2,
		-9.119646E-2,
		-4.756181E-2,
		4.038584E-2,
		4.121663E-2,
		-1.592129E-2,
		-3.347084E-2,
		2.007330E-3,
		2.521301E-2,
		5.741392E-3,
		-1.729652E-2,
		-9.318693E-3,
		1.040475E-2,
		1.004102E-2,
		-4.978769E-3,
		-8.978565E-3,
		1.168291E-3,
		7.000915E-3,
		1.102657E-3,
		-4.810961E-3,
		-2.136249E-3,
		2.833411E-3,
		2.254731E-3,
		-1.373469E-3,
		-1.915433E-3,
		3.828157E-4,
		1.293682E-3,
		2.767892E-5,
		-8.493122E-4,
		-3.414229E-4,
		5.105268E-4
    };


/*分析频率为5000Hz时，各阶OnceFIR的滤波器系数
Input Sampling Rate = 25600Hz
Output Sampling Rate = 12800 Hz
Decimation Factor = 2
Fpass = 5000 Hz, Pass Ripple < 0.01dB
Fstop = 6400 Hz, Stopband Attenuation > 60dB*/
//Stage 1 : Decimation_Factor = 2, FIR_Parameters = 61
static  float fir_5000_1stage_coeffs[FIR_5000_1STAGE_SIZE]={
        -5.500899E-4,
		4.813905E-4,
		1.194679E-3,
		1.384363E-4,
		-1.650045E-3,
		-7.896170E-4,
		2.281085E-3,
		2.187035E-3,
		-2.414264E-3,
		-4.132936E-3,
		1.739288E-3,
		6.489708E-3,
		2.221724E-4,
		-8.783310E-3,
		-3.817145E-3,
		1.030450E-2,
		9.217067E-3,
		-1.009554E-2,
		-1.630684E-2,
		6.990504E-3,
		2.464662E-2,
		4.390378E-4,
		-3.349330E-2,
		-1.427091E-2,
		4.189396E-2,
		3.893231E-2,
		-4.883438E-2,
		-9.020027E-2,
		5.341432E-2,
		3.128220E-1,
		4.449855E-1,
		3.128220E-1,
		5.341432E-2,
		-9.020027E-2,
		-4.883438E-2,
		3.893231E-2,
		4.189396E-2,
		-1.427091E-2,
		-3.349330E-2,
		4.390378E-4,
		2.464662E-2,
		6.990504E-3,
		-1.630684E-2,
		-1.009554E-2,
		9.217067E-3,
		1.030450E-2,
		-3.817145E-3,
		-8.783310E-3,
		2.221724E-4,
		6.489708E-3,
		1.739288E-3,
		-4.132936E-3,
		-2.414264E-3,
		2.187035E-3,
		2.281085E-3,
		-7.896170E-4,
		-1.650045E-3,
		1.384363E-4,
		1.194679E-3,
		4.813905E-4,
		-5.500899E-4
	};

typedef struct
{
    uint32_t decifactor;          // 抽样点数
    uint32_t coeff_number;        // 滤波器系数点数
    float*   coeff;               // 滤波器系数
} fir_stage_t;

typedef struct
{
    uint8_t       stage_number;
    fir_stage_t** stages;
} fir_filter_item_t;

/* Fir低通滤波器系数表，根据上限频率档位划分为9档 */
static fir_filter_item_t fir_items[FIR_FILTER_NUMBER]; //9

/* 功能描述: 通过上限查找滤波器 */
static fir_filter_item_t* get_filter_item_by_up_freq(uint32_t up_freq)
{
    uint8_t index = 0;
    switch (up_freq)
    {
        case 500: index = FIR_F500; break;
        case 1000: index = FIR_F1000; break;
        case 2000: index = FIR_F2000; break;
		case 2500: index = FIR_F2500; break;
		case 4000: index = FIR_F4000; break;
        case 5000: index = FIR_F5000; break;
        case 10000: index = FIR_F10000; break;
		case 20000: index = FIR_F20000; break;
		case 40000: index = FIR_F40000; break;
        default: return NULL;
    }
    return &fir_items[index];
}

/* 功能描述: 初始化FIR滤波器 */
bool init_fir_filters(void)
{
    // 初始化500Hz档位系数
    // 2级降采样
    fir_items[FIR_F500].stage_number = FIR_500_STAGE_NUMBER;
    // 2级滤波器系数数组
    fir_items[FIR_F500].stages = (fir_stage_t**)malloc(sizeof(fir_stage_t*) * FIR_500_STAGE_NUMBER);
    if (NULL == fir_items[FIR_F500].stages){
        return false;
    }

	int i = 0;
    for (i = 0; i < FIR_500_STAGE_NUMBER; ++i) {
        fir_items[FIR_F500].stages[i] = (fir_stage_t*)malloc(sizeof(fir_stage_t));
    }
    // 第一级
    fir_items[FIR_F500].stages[0]->coeff        = fir_500_1stage_coeffs; //滤波器系数
    fir_items[FIR_F500].stages[0]->decifactor   = FIR_500_1STAGE_DECIFACTOR;//抽样点数
    fir_items[FIR_F500].stages[0]->coeff_number = FIR_500_1STAGE_SIZE;//滤波器系数点数
    // 第二级
    fir_items[FIR_F500].stages[1]->coeff        = fir_500_2stage_coeffs;
    fir_items[FIR_F500].stages[1]->decifactor   = FIR_500_2STAGE_DECIFACTOR;
    fir_items[FIR_F500].stages[1]->coeff_number = FIR_500_2STAGE_SIZE;


    // 初始化1000Hz档位系数
    // 1级降采样
    fir_items[FIR_F1000].stage_number = FIR_1000_STAGE_NUMBER;
    //1级滤波器系数数组
    fir_items[FIR_F1000].stages = (fir_stage_t**)malloc(sizeof(fir_stage_t*) * FIR_1000_STAGE_NUMBER);
    if (NULL == fir_items[FIR_F1000].stages)
    {
        if (fir_items[FIR_F500].stages)
            free(fir_items[FIR_F500].stages);
        return false;
    }
    for (i = 0; i < FIR_1000_STAGE_NUMBER; ++i) {
        fir_items[FIR_F1000].stages[i] = (fir_stage_t*)malloc(sizeof(fir_stage_t));
    }
    // 第一级
    fir_items[FIR_F1000].stages[0]->coeff        = fir_1000_1stage_coeffs;
    fir_items[FIR_F1000].stages[0]->decifactor   = FIR_1000_1STAGE_DECIFACTOR;
    fir_items[FIR_F1000].stages[0]->coeff_number = FIR_1000_1STAGE_SIZE;


    // 初始化2500Hz档位系数
    // 2级降采样
    fir_items[FIR_F2500].stage_number = FIR_2500_STAGE_NUMBER;
    // 2级滤波器系数数组
    fir_items[FIR_F2500].stages = (fir_stage_t**)malloc(sizeof(fir_stage_t*) * FIR_2500_STAGE_NUMBER);
    if(NULL == fir_items[FIR_F2500].stages)
    {
        if (fir_items[FIR_F500].stages)
            free(fir_items[FIR_F500].stages);

        if (fir_items[FIR_F1000].stages)
            free(fir_items[FIR_F1000].stages);

        return false;
    }
    for (i = 0; i < FIR_2500_STAGE_NUMBER; ++i) {
        fir_items[FIR_F2500].stages[i] = (fir_stage_t*)malloc(sizeof(fir_stage_t));
    }
	// 第一级
    fir_items[FIR_F2500].stages[0]->coeff        = fir_2500_1stage_coeffs;
    fir_items[FIR_F2500].stages[0]->decifactor   = FIR_2500_1STAGE_DECIFACTOR;
    fir_items[FIR_F2500].stages[0]->coeff_number = FIR_2500_1STAGE_SIZE;
	// 第二级
    fir_items[FIR_F2500].stages[1]->coeff        = fir_2500_2stage_coeffs;
    fir_items[FIR_F2500].stages[1]->decifactor   = FIR_2500_2STAGE_DECIFACTOR;
    fir_items[FIR_F2500].stages[1]->coeff_number = FIR_2500_2STAGE_SIZE;


    // 初始化5000Hz档位系数
    // 1级降采样
    fir_items[FIR_F5000].stage_number = FIR_5000_STAGE_NUMBER;
    // 1级滤波器系数数组
    fir_items[FIR_F5000].stages = (fir_stage_t**)malloc(sizeof(fir_stage_t*) * FIR_5000_STAGE_NUMBER);
    if (NULL == fir_items[FIR_F5000].stages)
    {
        if (fir_items[FIR_F500].stages)
            free(fir_items[FIR_F500].stages);

        if (fir_items[FIR_F1000].stages)
            free(fir_items[FIR_F1000].stages);

        if (fir_items[FIR_F2500].stages)
            free(fir_items[FIR_F2500].stages);

        return false;
    }
    for (i = 0; i < FIR_5000_STAGE_NUMBER; ++i) {
        fir_items[FIR_F5000].stages[i] = (fir_stage_t*)malloc(sizeof(fir_stage_t));
    }
    fir_items[FIR_F5000].stages[0]->coeff        = fir_5000_1stage_coeffs;
    fir_items[FIR_F5000].stages[0]->decifactor   = FIR_5000_1STAGE_DECIFACTOR;
    fir_items[FIR_F5000].stages[0]->coeff_number = FIR_5000_1STAGE_SIZE;

    return true;
}

/* 功能描述: 运行FIR滤波算法 */
void run_fir(float *src,//原数据
          int length,  //原数据长度
		  int coeff_number,//滤波器系数点数
		  int decifactor,//抽样点数
		  float *coeffs, //滤波器系数
		  float *dst //计算后的数据存储buf
		 )
{
	int    idx       = 0;
    int    position  = 0;
	int    i = 0 , read_idx = 0;
	float  fir_data = 0.0;
	int    cur_items = length;
    if(src == NULL || coeffs == NULL || dst == NULL)
    {
        return;
    }
	while(cur_items >= coeff_number)//当长度大于等于FIR滤波系数点数，才可以进行滤波计算
	{
		// 每个抽样的点都计算一次
        for (idx = 0; idx < coeff_number; idx++)
        {
            position = read_idx + idx;
            fir_data += ((src[position])*(coeffs[idx]));
        }

		dst[i++] = fir_data ;
		fir_data = 0;
		read_idx  += decifactor ;
		cur_items -= decifactor;
	}
}

/* 功能描述：fir低通滤波入口函数 */
void  enter_fir_filter(float *src_data, int length,  int up_freq)
{
	//LOGD("xin: enter_fir_filter_length = %d ,  up_freq = %d " , length , up_freq );
    if(src_data == NULL || length ==0 || up_freq ==0)
    {
        return;
    }
	init_fir_filters();//初始化FIR滤波器系数
	fir_filter_item_t *pfir_filter = NULL;
	pfir_filter =  get_filter_item_by_up_freq(up_freq);//根据上限频率获取对应的滤波器

	float *pdst = NULL;
	if(pdst == NULL)
	{
		pdst = (float *)malloc((length/2)*sizeof(float));
		if(pdst == NULL)
		{
			LOGD("pdst 分配内存失败！");
			return;
		}
		memset(pdst, 0, (length/2)*sizeof(float));
	}

    int i = 0;
	for(i = 0; i < pfir_filter->stage_number; i++) //循环多少级档位
	{
		run_fir(src_data, length, pfir_filter->stages[i]->coeff_number , pfir_filter->stages[i]->decifactor, pfir_filter->stages[i]->coeff , pdst);
		memset(src_data, 0, (length)*sizeof(float));
		memcpy(src_data, pdst, (length/2)*sizeof(float));
        length = length/2; //是针对500 2500 两个频率是2级档位，而1000   5000两个频率是1级档位，用不到这句
	}
	//for(i = 0;i< 200; i++)
	//{
		//LOGD("xin: 经过FIR后src_data[%06d]  = %f",i,src_data[i]);
	//}
	if(pdst != NULL)
	{
		free(pdst);
		pdst = NULL;
	}
}
