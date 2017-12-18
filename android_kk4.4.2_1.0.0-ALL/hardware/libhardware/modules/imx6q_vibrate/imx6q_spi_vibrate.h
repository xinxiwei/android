/*
 * Copyright (C) 2009 Mokoid Open Source Project
 * Copyright (C) 2009,2010 Moko365 Inc.
 *
 * Author: Jollen Chen <jollen@moko365.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef ANDROID_IMX6Q_SPI_VIBRATE
#define ANDROID_IMX6Q_SPI_VIBRATE
//CH number
#define SINGLE_CH   1
#define DOUBLE_CH   2
//wave length
#define WAVE_LEN_1K  1024
#define WAVE_LEN_2K  2048
#define WAVE_LEN_4K  4096
#define WAVE_LEN_8K  8192
#define WAVE_LEN_16K  16384
#define WAVE_LEN_32K  32768
#define WAVE_LEN_64K  65536
#define WAVE_LEN_128K 131072
#define WAVE_LEN_256K 262144


#ifdef __cplusplus
extern "C"
{
#endif

extern void feature_value(float *pData, int length, float *ret_value);
extern float rend_value(float *pData,  int length, int totalvalue_type);
extern void fft_alg_entry2(float *p_buf, long length, int window_type, int average_mode, int average_num);

#ifdef __cplusplus
};
#endif

#endif
