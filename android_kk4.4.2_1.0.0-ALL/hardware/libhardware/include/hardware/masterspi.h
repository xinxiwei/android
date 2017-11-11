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
#ifndef ANDROID_LHX_MASTERSPI_H
#define ANDROID_LHX_MASTERSPI_H
#ifdef __cplusplus
extern "C"
{
#endif

int swrite(unsigned int addr,unsigned int data);
unsigned int sread(unsigned int addr);
int masterspi_open();
int masterspi_close();
int masterspi_test(int cmd,int para);
int GetSreadErrorCode();
#ifdef __cplusplus
};
#endif
#endif
