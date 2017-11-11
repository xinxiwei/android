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
#ifndef ANDROID_LHX_ERRORCODE_H
#define ANDROID_LHX_ERRORCODE_H

#ifdef __cplusplus
extern "C"
{
#endif
       int baseoffset;
       int normal_value;
       int masterspi_open_ecode;
       int masterspi_close_ecode;

       int OpenSpiSlaveFd_ecode;
       int CloseSpiSlaveFd_ecode;

       int GpioOpen_ecode;
       int GpioClose_ecode;

       int BeepOpen_ecode;
       int BeepClose_ecode;

       int openEvent0_ecode;
       int closeEvent0_ecode;
       int BeepOn_ecode;
       int BeepOff_ecode;
       int BeepSet_ecode;
       int GpioSet_ecode;
       int GetLock_rwlock2_ecode;
       int ReleaseLock_rwlock2_ecode;
       int AttachCurrentThread_ecode;
       int GetFieldID_ecode;
       int GetMethodID_ecode;
       int unlock_rwlock2_ecode;
       int rwlock_rwlock2_ecode;
       int unlock_rwlock1_ecode;
       int rwlock_rwlock1_ecode;
       int CallVoidMethod_ecode;
       int DetachCurrentThread_ecode;
       int RecvSem_init_ecode;
       int GetJavaVM_ecode;
       int gJavaObj_ecode;
       int th_RcvData_ecode;
       int StartSpiSlave_ecode;
       int SpiSlaveModeOff_ecode;
       int RecvSem_post_ecode;
       int SPI_IOC_WR_BITS_PER_WORD_ecode;
       int SPI_IOC_WR_MAX_SPEED_HZ_ecode;
       int SPI_IOC_MESSAGE_ecode;
       int SPI_IOC_RD_BITS_PER_WORD_ecode;
       int Sread_wr_ecode;
       int SPI_IOC_RD_MAX_SPEED_HZ_ecode;
       int Sread_SPI_IOC_MESSAGE_ecode;
       int dataSignal_overflow_ecode;
       int supermanService_getlock_ecode;
       int supermanService_releaselock_ecode;
       int DeframeOne1_ChannleA_ChDataList_size_ecode;
       int DeframeOne1_ChannleB_ChDataList_size_ecode;
       int DeframeOne1_ChR_RotationDataList_size_ecode;
       int sighandler_read_ecode;
        int sighandler_unlock_rwlock1_ecode;
        int sighandler_rwlock_rwlock1_ecode;
        int SPIDEV_IOC_QUEUE_STATE_RECFULL_ecode;
        int SPIDEV_IOC_QUEUE_STATE_SENDEMPTY_ecode;
        int SPIDEV_IOC_QUEUE_STATE_ecode;

       int initerrorcode();
#ifdef __cplusplus
};
#endif
#endif
