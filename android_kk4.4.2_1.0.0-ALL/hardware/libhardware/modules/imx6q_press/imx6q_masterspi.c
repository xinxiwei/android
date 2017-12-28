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
#include <unistd.h>
#include <hardware/spidev.h>
//sdk ndk include dictionary
#include <hardware/log.h>
#include <sys/ioctl.h>
#include <hardware/masterspi.h>
#include <hardware/errorcode.h>
#define DEVICE_NAME "/dev/spidev0.0"  //主设备
/*****************************************************************************/
int wr( unsigned int tbuf[] );
static unsigned char bits_per_word = 32;
static unsigned int speed = 2500000;
static int fd = -1;
static int masterspi_testcmd=0;
static int masterspi_testpara=0;
static int masterspi_errorcode=0;

int masterspi_open(  )
 {
	unsigned char mode;
	fd = open( DEVICE_NAME , O_RDWR );
	if( fd == -1 )
	{
		LOGE( "masterspi open failed  error no= %s." , strerror( errno ) );
		return -1;
   	}else{
		LOGD( "xin: 打开主spi设备 /dev/spidev0.0 成功. fd = %d" , fd );
	}
	mode =0;
	if( ioctl( fd ,  SPI_IOC_WR_MODE ,  &mode )==-1 )
    {
         LOGE( "masterspi_open ioctl SPI_IOC_WR_MODE error" );
         return -1;
    }
 success:
         return 0;
 }

int masterspi_close(  )
{

    if( close( fd ) < 0  )
	{
		LOGE( "master spi close error" );
		return -1;
	}
	else{
        LOGD( "xin: 关闭主spi设备 /dev/spidev0.0 成功. fd = %d" , fd );
    }
    return 0;
}
int swrite( unsigned int  laddr , unsigned int data )
{
	unsigned int localbuf[2] ={0};
	localbuf[0] = ( 0xffffffff &laddr );
	if( ( 0xff000000&localbuf[0] )==0 )
	{
		localbuf[0] |= 0x01000000;
	}
	localbuf[1] = data;
	int ret =  wr( localbuf );
	//if( masterspi_testcmd == 1 && masterspi_testpara == 1 )
	//{
		LOGD( "write addr = %d , value = %d" , ( localbuf[0]&0x00ffffff ) , localbuf[1] );
	//}
	usleep( 20000 ); //50000
	return ret;//0 is right.
}

int wr( unsigned int tbuf[] )
{
   int res=0;
   res = ioctl( fd ,  SPI_IOC_WR_BITS_PER_WORD ,  &bits_per_word );
   if ( res == -1 )
   {
		LOGD( "wr ioctl SPI_IOC_WR_BITS_PER_WORD error." );
		return SPI_IOC_WR_BITS_PER_WORD_ecode;
   }
   res = ioctl( fd ,  SPI_IOC_WR_MAX_SPEED_HZ ,  &speed );
   if ( res == -1 )
   {
		LOGD( "wr ioctl SPI_IOC_WR_MAX_SPEED_HZ error." );
		return SPI_IOC_WR_MAX_SPEED_HZ_ecode;
   }
   struct spi_ioc_transfer tr = {
        .tx_buf = ( unsigned long )tbuf ,
        .rx_buf =  NULL ,
        .len = 2*4 ,
   };
   if( tr.tx_buf == 0)
   {
	    LOGD( "tr.tx_buf == 0=====" );
   }
   res = ioctl( fd ,  SPI_IOC_MESSAGE( 1 ) ,  &tr );
	if ( res == 1 )
	{
	    LOGD( "wr ioctl SPI_IOC_MESSAGE( 1 ) error." );
		return SPI_IOC_MESSAGE_ecode;
	}
   return 0;
}

unsigned int sread( unsigned int addr )
{
	int res = 0;
	//write address
	unsigned int tbuf[2] = {0};
	tbuf[0] = ( 0x02000000 | ( 0x00ffffff &addr ) );
	tbuf[1] = 0;
    masterspi_errorcode =0;
	res=wr( tbuf );//write out the address to read start;
    if( res != 0 )//write error
    {
        LOGE( "sread error because wr( tbuf ) in non zero." );
        masterspi_errorcode = Sread_wr_ecode;
        return res;
    }
	//end write address
	res = ioctl( fd ,  SPI_IOC_RD_BITS_PER_WORD ,  &bits_per_word );
	if ( res == -1 )
	{
		LOGE( "sread ioctl SPI_IOC_RD_BITS_PER_WORD error." );
        masterspi_errorcode=SPI_IOC_RD_BITS_PER_WORD_ecode;
		return masterspi_errorcode;
	}
	res = ioctl( fd ,  SPI_IOC_RD_MAX_SPEED_HZ ,  &speed );
	if ( res == -1 )
	{
		LOGE( "sread ioctl SPI_IOC_RD_MAX_SPEED_HZ error." );
        masterspi_errorcode=SPI_IOC_RD_MAX_SPEED_HZ_ecode;
        return masterspi_errorcode;
	}

	unsigned int localbuf[2] = {0};
	struct spi_ioc_transfer tr = {
		.tx_buf =  NULL ,
		.rx_buf = ( unsigned long ) localbuf ,
		.len =8 ,
	};
	res = ioctl( fd ,  SPI_IOC_MESSAGE( 1 ) ,  &tr );
	if ( res == 1 )
	{
		LOGD( "sread ioctl SPI_IOC_MESSAGE( 1 ) error." );
        masterspi_errorcode=Sread_SPI_IOC_MESSAGE_ecode;
		return masterspi_errorcode;
	}
	//if( masterspi_testcmd == 1 && masterspi_testpara == 1 )
	//{
		LOGD( "read addr = %d , value = %d" , ( tbuf[0]&0x00ffffff ) , localbuf[0] );
	//}
	return  localbuf[0];
}

int masterspi_test( int cmd , int para )
{
	masterspi_testcmd = cmd;
	masterspi_testpara = para;
	return 0;
}

int GetSreadErrorCode(  )
{
    int temperrorcode =0;
    temperrorcode = masterspi_errorcode;
    masterspi_errorcode = 0;
    return temperrorcode;
}
