
#ifndef ANDROID_IMX6Q_SPI_PRESSURE
#define ANDROID_IMX6Q_SPI_PRESSURE
//DMA
#define  SPIDEV_IOC_RXSTREAMON         (0x06)
#define  SPIDEV_IOC_RXSTREAMOFF        (0x07)
#define  SPIDEV_IOC_QUEUE_STATE        (0X08)
//FPGA
#define StartSampleAddr  0x03000000
#define StartSampleData  0

#define StopSampleAddr   0x04000000
#define StopSampleData   0

#define ResetFpgaRegAddr   0x05000000
#define ResetFpgaRegData   0   
//POWER
#define FPGA_3V3_CTR    6
#define FPGA_2V5_CTR    7
#define FPGA_1V2_CTR    8
#define FPGA_RESET_CTR  10
//GPIO
#define GPIO_SET_ON   0X01
#define GPIO_SET_OFF  0X00
//freq
#define SPI_SAMPLE  25600   //Hz

#define ADCModeAddr   0


#ifdef __cplusplus
extern "C"
{
#endif	  		
	extern int  spi_freq();
	extern void poweron_spi();
	extern void poweroff_spi();		
	extern int  set_press_reg(int smp_rate);
	extern void press_alg_entry(float *p,int length,float *b);
	extern char *log_time();

#ifdef __cplusplus
};
#endif
	
#endif