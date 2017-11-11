#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <semaphore.h>
#include <pthread.h>
#include <linux/types.h>
#include <hardware/spidev.h>
#include <sys/time.h>
#include <signal.h>
#include <hardware/log.h>
#include <hardware/errorcode.h>

#define  DEV_SPI1   "/dev/mxc_spidev1"
#define  SPIDEV_IOC_RXSTREAMON         ( 0x06 )
#define  SPIDEV_IOC_RXSTREAMOFF        ( 0x07 )
#define  SPIDEV_IOC_QUEUE_STATE        ( 0x08 )
sem_t RecvSem;
int spiSlaveFd =-1;
const int lenInt = 15 * 1024;
const int lenByte = 15 * 1024 * 4;
int rxbuf[15 * 1024];//int word

struct sigaction action;
pthread_rwlock_t rwlock1=PTHREAD_RWLOCK_INITIALIZER;
pthread_rwlock_t rwlock2=PTHREAD_RWLOCK_INITIALIZER;

static int signo_count = 0;
static int testCmd = 0;
static int testPara = 0;
static int spiSlaveErrorCode = 0;

int OpenSpiSlaveFd(  )
{
    spiSlaveFd = open( DEV_SPI1 ,  O_RDWR );
    if ( spiSlaveFd < 0 )
    {
        LOGE( "Error:cannot open spi slave fd." );
        return -1;
    }
    LOGD( "slave spi open. fd = %d" , spiSlaveFd );
	return 0;
}

int CloseSpiSlaveFd(  )
{
    LOGD( "spiSlaveFd closed. fd=%d." , spiSlaveFd );
    LOGD( "destroy RecvSem." );
    sem_destroy( &RecvSem );
    if( close( spiSlaveFd ) < 0 )
    {
        LOGE( "spiSlaveFd close error." );
        return -1;
    }
    return 0;
}

int SpiSlaveModeOff(  )
{
    int res = -1;
    res = ioctl( spiSlaveFd ,  SPIDEV_IOC_RXSTREAMOFF ,  NULL );
    if ( res == -1 )
    {
        LOGE( "SpiSlaveModeOff false." );
        return -1;
    }
    LOGD( "SpiSlaveModeOff" );
    return 0;
}

int SpiSlaveModeOn(  )
{
    int res = -1;
    res = ioctl( spiSlaveFd ,  SPIDEV_IOC_RXSTREAMON ,  NULL );
    if ( res == -1 )
    {
        LOGE( "SpiSlaveModeOn false." );
        return -1;
    }
    LOGD( "SpiSlaveModeOn" );
	return 0;
}

int slavespi_quene_state = 0;
void sighandler( int signo ) 
{
    if( signo == SIGIO )
    {
        if( testCmd == 1 && testPara == 1 )
        {
            signo_count++;
            LOGD( "SIGIO received in sighandler. signo_count = %d" , signo_count );
        }
        /////////////////////GET spi slave buffer state///////////////////
        /*state ( 1bit|0bit| ) ---- 1bit --> rec_queue */
        /******************* ---- 0bit --> send_queue*/
        int res = ioctl( spiSlaveFd ,  SPIDEV_IOC_QUEUE_STATE ,  &slavespi_quene_state );
        if ( res == -1 )
        {
            LOGE( "read state failed!!" );
            spiSlaveErrorCode = SPIDEV_IOC_QUEUE_STATE_ecode;
        }
        else
        {
            if( ( slavespi_quene_state & 0x02 ) == 0x02 )
                spiSlaveErrorCode = SPIDEV_IOC_QUEUE_STATE_RECFULL_ecode;
            else if( ( slavespi_quene_state & 0x01 ) == 0x01 )
                spiSlaveErrorCode = SPIDEV_IOC_QUEUE_STATE_SENDEMPTY_ecode;
        }
        /////////////////////GET spi slave buffer state end///////////////////
        if( !pthread_rwlock_wrlock( &rwlock1 ) )
        {
            int ret = read( spiSlaveFd ,  rxbuf ,  lenByte );//byte;  one int word =4 bytes
            if( testCmd == 1 && testPara == 1 )
            {
                LOGD( "sighandler. read from spi driver. rxbuf[1]=%d , rxbuf[2]= %d" , rxbuf[1] , rxbuf[2] );
            }
            if ( ret < 0 )
            {
                LOGE( "Error:spi slave device read fail !" );
                spiSlaveErrorCode = sighandler_read_ecode;
                return;
            }
            if( !pthread_rwlock_unlock( &rwlock1 ) )
            {
                sem_post( &RecvSem );
                if( testCmd == 1 && testPara == 1 )
                {
                    LOGD( "sighandler , send out  RecvSem" );
                }
            }else
            {
                spiSlaveErrorCode = sighandler_unlock_rwlock1_ecode;
            }
        }else
        {
            spiSlaveErrorCode = sighandler_rwlock_rwlock1_ecode;
        }

    }
}

int GetSpiSlaveBufferErrorCode(  )
{
    int temperrorcode =0;
    temperrorcode=spiSlaveErrorCode;
    spiSlaveErrorCode=0;
    return spiSlaveErrorCode;
}

int StartSpiSlave(  )
{
    int res = -1;
    memset( &action ,  0 ,  sizeof( action ) );
    action.sa_handler = sighandler;
    action.sa_flags = 0;
    sigaction( SIGIO ,  &action ,  NULL );
    
    fcntl( spiSlaveFd ,  F_SETOWN ,  getpid(  ) );
    fcntl( spiSlaveFd ,  F_SETFL ,  fcntl( spiSlaveFd ,  F_GETFL ) | FASYNC );
    SpiSlaveModeOn(  );
    return 0;
}

int testSpiSlave( int cmd , int para )
{
    testCmd = cmd;
    testPara = para;
    return 0;
}
