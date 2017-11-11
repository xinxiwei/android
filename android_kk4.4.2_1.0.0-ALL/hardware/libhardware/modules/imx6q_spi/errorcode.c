/*
 *
 * Date:2016-8-9
 * Author:lhx
 * Description: this file call the linux driver .
 *
 *
 */

#include <fcntl.h>
#include <errno.h>
#include <linux/input.h>
#include <hardware/log.h>
#include <sys/ioctl.h>
#include <hardware/errorcode.h>

int initerrorcode()
{
	baseoffset = 0;
	normal_value = baseoffset + 0;
	masterspi_open_ecode= baseoffset + 1;
	masterspi_close_ecode= baseoffset +2;

	OpenSpiSlaveFd_ecode= baseoffset +3;
	CloseSpiSlaveFd_ecode= baseoffset +4;

	GpioOpen_ecode= baseoffset +5;
	GpioClose_ecode= baseoffset +6;

	BeepOpen_ecode= baseoffset +7;
	BeepClose_ecode= baseoffset +8;

	openEvent0_ecode= baseoffset +9;
	closeEvent0_ecode= baseoffset +10;
	BeepOn_ecode= baseoffset +11;
	BeepOff_ecode= baseoffset +12;
	BeepSet_ecode= baseoffset +13;
	GpioSet_ecode= baseoffset +14;
	GetLock_rwlock2_ecode= baseoffset +15;
	ReleaseLock_rwlock2_ecode= baseoffset +16;
	AttachCurrentThread_ecode= baseoffset +17;
	GetFieldID_ecode= baseoffset +18;
	GetMethodID_ecode= baseoffset +19;
	unlock_rwlock2_ecode= baseoffset +20;
	rwlock_rwlock2_ecode= baseoffset +21;
	unlock_rwlock1_ecode= baseoffset +22;
	rwlock_rwlock1_ecode= baseoffset +23;
	CallVoidMethod_ecode= baseoffset +24;
	DetachCurrentThread_ecode= baseoffset +25;
	RecvSem_init_ecode= baseoffset +26;
	GetJavaVM_ecode= baseoffset +27;
	gJavaObj_ecode= baseoffset +28;
	th_RcvData_ecode= baseoffset +29;
	StartSpiSlave_ecode= baseoffset +30;
	SpiSlaveModeOff_ecode= baseoffset +31;
	RecvSem_post_ecode= baseoffset +32;
	SPI_IOC_WR_BITS_PER_WORD_ecode= baseoffset +33;
	SPI_IOC_WR_MAX_SPEED_HZ_ecode= baseoffset +34;
	SPI_IOC_MESSAGE_ecode= baseoffset +35;
	SPI_IOC_RD_BITS_PER_WORD_ecode= baseoffset +36;
	Sread_wr_ecode= baseoffset +37;
	SPI_IOC_RD_MAX_SPEED_HZ_ecode= baseoffset +38;
	Sread_SPI_IOC_MESSAGE_ecode= baseoffset +39;
	dataSignal_overflow_ecode= baseoffset +40;
	supermanService_getlock_ecode= baseoffset +41;
	supermanService_releaselock_ecode= baseoffset +42;
	DeframeOne1_ChannleA_ChDataList_size_ecode= baseoffset +43;
	DeframeOne1_ChannleB_ChDataList_size_ecode= baseoffset +44;
	DeframeOne1_ChR_RotationDataList_size_ecode= baseoffset +45;
	sighandler_read_ecode= baseoffset +45;
    sighandler_unlock_rwlock1_ecode= baseoffset +46;
    sighandler_rwlock_rwlock1_ecode= baseoffset +47;
    SPIDEV_IOC_QUEUE_STATE_RECFULL_ecode= baseoffset +48;
    SPIDEV_IOC_QUEUE_STATE_SENDEMPTY_ecode= baseoffset +49;
    SPIDEV_IOC_QUEUE_STATE_ecode= baseoffset +50;
	return 0;////
}








