#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <malloc.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <hardware/log.h>
#include <cutils/log.h>
#include <hardware/math.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <pthread.h>
#include <hardware/gpio.h>
#include <hardware/log.h>
#include <hardware/masterspi.h>
#include <hardware/slavespi.h>
#include <hardware/spidev.h>

#include <sys/timeb.h>
#include <time.h>

#include <hardware/imx6q_spi_config.h>

#define   SMP_RATE_102400     102400   //����2.56 =40K
#define   SMP_RATE_51200      51200    //20K
#define   SMP_RATE_25600      25600    //10K
#define   SMP_RATE_12800      12800    //5K
#define   SMP_RATE_10240      10240    //4K
#define   SMP_RATE_6400       6400     //2.5K
#define   SMP_RATE_5120       5120     //2K
#define   SMP_RATE_2560       2560     //1K
#define   SMP_RATE_1280       1280     //0.5K


enum {
    HighSpeed , HighPrecision , LowPower , LowSpeed
}ADCModeEnum;//ADC_MODEģʽ

enum {
    DefaultClk , DefaultHalf , DefaultFourth , DefaultEighth
}ADCClkEnum;//ADC_CLK ʱ������

enum  {
    AC , DC
}CoupleModeEnum;//��Ϸ�ʽ

enum  {
    CURRENT , VOLTAGE
}SampleTypeEnum;//�ɼ�����

enum  {
    V25 , V2P5 , V0P25 , VAUTO
}VoltageWindowEnum; //����ѡ��

enum  {
    SPEED , 
	SHIFT
}IntegrateTypeEnum;//����ѡ��

enum {
	AnalogFallingCount , AnalogRisingCount
}SpeedTrigerEnum; //ת�ٴ���������

enum  {
    CHA , CHB , CHR
}CHEnum;//ͨ��ѡ��

enum {
    Manual , RTriger , AnalogRising , AnalogFalling
}TrigerEnum; //ģ�ⴥ����ʽ

enum {
	PresureSample , SingleChSample , DoubleChSample , RotationSample
}AnalyTypeEnum; //�豸��������



struct  main_fpga_reg  
{  
 	float adc_clk ;
	float rotation_resis_value;
	float trig_value;
	bool adc_work_status;
	bool rotat_sample_status;
	int  fifo_byte;	
}; 

/////////////////Ctrl_REG  0x0000
void enable_rotation( bool flag )//ʹ��ת��ͨ�� OK
{
	//LOGD( "xin:===enable_rotation" ); 
	unsigned int data = sread( 0 );
	if( flag )
	{
		data |= 0x1;
	}
	else
	{
		data &=( ~(( unsigned int )( 0x1 ) ) );
	}
	swrite( 0 , data );
	//sread( 0 );
}

int  enable_CH_A( bool flag )//ʹ����CHA OK
{
	LOGD( "xin:===enable_CH_A" ); 
	
	unsigned int data = sread( 0 );
	if( flag )
	{
		data |= 0x2;
	}
	else
	{
		data &=( ~(( unsigned int )( 0x2 ) ) );
	}
	swrite( 0 , data );
	//sread( 0 );
	LOGD("xin: enable_CH_A data = %d",data);
	return data;
}

void enable_CH_A_integrate( bool flag )//ʹ��CHA��Ӳ������ OK
{
	LOGD( "xin:===enable_CH_A_integrate" ); 
	unsigned int data = sread( 0 );
	if( flag )
	{
		data |= 0x4;
	}
	else
	{
		data &=( ~(( unsigned int )( 0x4 ) ) );
	}
	swrite( 0 , data );
	//sread( 0 );
}

int enable_CH_B( bool flag )//ʹ����CHB OK
{
	LOGD( "xin:===enable_CH_B" );
	unsigned int data = sread( 0 );
	if( flag )
	{
		data |= 0x8;
	}
	else
	{
		data &=( ~(( unsigned int )( 0x8 ) ) );
	}
	swrite( 0 , data );
	//sread( 0 );
	return data;
}

void enable_CH_B_integrate( bool flag )//ʹ��CHB��Ӳ������ OK
{
	LOGD( "xin:===enable_CH_B_integrate" );
	unsigned int data = sread( 0 );
	if( flag )
	{
		data |= 0x10;
	}
	else
	{
		data &=( ~(( unsigned int )( 0x10 ) ) );
	}
	swrite( 0 , data );
	//sread( 0 );
}

void set_adc_mode(  int adcMode  )//����ADC�Ĺ���ģʽ OK
{
	LOGD( "xin:===set_adc_mode" );
	unsigned int data =sread( 0 );//adcMode  reg  0x0000
	switch( adcMode )
	{
		case HighSpeed:  //00			 
			break;
		case HighPrecision: //01		
			data |=( 0x1<<8 );
			break;
		case LowPower:   //10			
			data |=( 0x2<<8 );
			break;
		case LowSpeed:  //11
			data |=( 0x3<<8 );
			break;
	}
	swrite( 0 , data );
	//sread( 0 );
}

int get_adc_mode(  )//�õ�adc�Ĺ���ģʽ OK
{
	//LOGD( "xin:===get_adc_mode" );
	unsigned int data = 0;
	int mode =0;
	data=sread( 0 );
	data = data>>8;//7
	data &=0x3;
	switch( data )
	{
		case 0:
		    mode = HighSpeed;//����ģʽ
			break;
		case 1:
			mode = HighPrecision;//�߾���ģʽ
			break;
		case 2:
			mode = LowPower;//�͹���
			break;
		case 3:
			mode = LowSpeed;//����ģʽ
			break;
	}
    return mode;
}

float get_adc_clk_rate(  )//�õ�ADCʱ������ OK  HZ
{
	unsigned int data = 0;
	data=sread( 0 ); //ADC_CLKѡ��  �Ĵ���0x0000
	data = data >> 5;
	data &=0x7;
	float ret=0.0;
	float clk_value = 26214400.0;//Hz
	switch( data )
	{
		case 0:
			ret = clk_value;
			break;
		case 1:
			ret = clk_value/2;
			break;
		case 2:
			ret = clk_value/4;
			break;
		case 3:
			ret = clk_value/8;
			break;
	}
	return ret;
}

void set_adc_clk_rate( float sampleRate )//����ADCʱ������ -------------
{
	LOGD( "xin:===set_adc_clk_rate" );	
	int adc_mode= get_adc_mode(  );
	int A=0;
	switch( adc_mode )
	{
		case HighSpeed: //00
			A=256;
			break;
		case HighPrecision:
			A=512;
			break;
		case LowPower:
			A=512;
			break;
		case LowSpeed:
			A=2560;
			break;
		default:break;
	}

	float real_clk =0.0;
	real_clk = A * sampleRate*1024;  //�õ���ʵ���·�Ƶ�� ��λHZ
		
	double dataGap = 26.2144 * 1024*1024/16;
	double clk_value = dataGap *16;
	unsigned int data = 0;
	data = sread( 0 );
	if( fabs( clk_value-real_clk ) < dataGap )//000
	{
		data &= 0xffffffef;
		swrite( 0 , data );
	}
	else if( fabs(( clk_value/2 )-real_clk ) < dataGap )//001
	{
		data &= 0xffffffef;
		data |=( 0x1<<4 );
		swrite( 0 , data );
	}
	else if( fabs(( clk_value/4 )-real_clk ) < dataGap )//010
	{
		data &= 0xffffffef;
		data |=( 0x1<<5 );
		swrite( 0 , data );
	}
	else if( abs(( clk_value/8 )-real_clk ) < dataGap )//011
	{
		data &= 0xffffffef;
		data |=( 0x1<<5 );
		data |=( 0x1<<4 );
		swrite( 0 , data );
	}
	sread( 0 );	
}

float get_sample_rate(  )//�õ�������  ��λHZ   OK
{
	float data = 0.0;
	float clk_freq = get_adc_clk_rate(  );
	int adc_mode = get_adc_mode(  );
	int A=0;
	switch( adc_mode )
	{
		case HighSpeed:
			A = 256;
			break;
		case HighPrecision:
			A = 512;
			break;
		case LowPower:
			A = 512;
			break;
		case LowSpeed:
			A = 2560;
			break;
	}
	data = clk_freq/A;
	return data;
}

/////////////CH1_Config_REG  0x0004 ,  CH2_Config_REG 0x0008
void set_couple_mode( int ch , int coupleMode )//����ÿ��ͨ����Ӧ����Ϸ�ʽ  OK
{
	LOGD( "xin:===set_couple_mode" );
	unsigned int data=0;
	unsigned int reg_addr =0;
	switch( ch )
	{
		case CHA:
			reg_addr = 0x4;
			break;
		case CHB:
			reg_addr = 0x8;
			break;
		 default:break;
	}
	data=sread( reg_addr );
	switch( coupleMode )
	{
		case AC:
			data &=( ~(( unsigned int )( 0x1 ) ) );
			break;
		case DC:
			data |=0x1;
			break;
		 default:break;
	}
	swrite( reg_addr , data );
	sread( reg_addr );
}

int get_couple_mode( int ch )//�õ���Ϸ�ʽ OK
{
	unsigned int data=0;
	unsigned int reg_addr =0;
	switch( ch )
	{
		case CHA:
			reg_addr = 0x4;
			break;
		case CHB:
			reg_addr = 0x8;
			break;
		 default:break;
	}
	data=sread( reg_addr );
	data &=0x1;
	if( data == 0x0 )
		return AC;
	else
		return DC;
}

void set_sample_type( int ch ,  int sampleType )//���òɼ�����  OK
{
	//LOGD( "xin:===set_sample_type" );
	unsigned int data=0;
	unsigned int reg_addr =0;
	switch( ch )
	{
		case CHA:
			reg_addr = 0x4;
			break;
		case CHB:
			reg_addr = 0x8;
			break;
		 default:break;
	}
	data=sread( reg_addr );
	switch( sampleType )
	{
		case CURRENT://����
			data |=0x2;
			break;
		case VOLTAGE://��ѹ	
			data &=( ~(( unsigned int )( 0x2 ) ) );
			break;
		 default:break;
	}
	swrite( reg_addr , data );
	//sread( reg_addr );
}

int get_sample_type( int ch )//�õ��ɼ����� OK
{
	unsigned int data=0;
	unsigned int reg_addr =0;
	switch( ch )
	{
		case CHA:
			reg_addr = 0x4;
			break;
		case CHB:
			reg_addr = 0x8;
			break;
		 default:break;
	}
	data=sread( reg_addr );
	data &=0x2;
	if( data==0x0 )
		return VOLTAGE; //��ѹ
	else
		return CURRENT; //����
}

void set_24V( int ch ,  bool flag )//����24v��Դ�Ŀ��� OK
{
	//LOGD( "xin:===set_24V" );
	unsigned int data=0;
	unsigned int reg_addr =0;
	switch( ch )
	{
		case CHA:
			reg_addr = 0x4;
			break;
		case CHB:
			reg_addr = 0x8;
			break;
		 default:break;
	}

	data=sread( reg_addr );
	if( flag )
		data |= 0x4;  //����24V@2.4mA����
	else  
		data &=( ~(( unsigned int )( 0x4 ) ) );
	swrite( reg_addr , data );
	//sread( reg_addr );
}

bool get_24V( int ch )//�õ�24v��Դ�Ŀ���״̬  OK
{
	unsigned int data=0;
	unsigned int reg_addr =0;
	switch( ch )
	{
		case CHA:
			reg_addr = 0x4;
			break;
		case CHB:
			reg_addr = 0x8;
			break;
		 default:break;
	}
	data=sread( reg_addr );
	data &=0x4;
	if( data==0x0 )
		return false;
	else
		return true;
}

void set_voltage_range( int ch ,  int voltageRange )//���õ�ѹ���� OK
{
	//LOGD( "xin:===set_voltage_range" );
	unsigned int data=0;
	unsigned int reg_addr =0;
	switch( ch )
	{
		case CHA:
			reg_addr = 0x4;
			break;
		case CHB:
			reg_addr = 0x8;
			break;
		 default:break;
	}
	data=sread( reg_addr );	
	switch( voltageRange )
	{
		case V25:		//000    			
			break;
		case V2P5:   
			data |=( 0x1<<3 );
			break;
		case V0P25:		
			data |=( 0x2<<3 );
			break;
		case VAUTO:			
			data |=( 0x3<<3 );
			break;
		 default:break;
	}
	swrite( reg_addr , data );
	//sread( reg_addr );
}

void set_adc_clk( int adcClk )
{
	LOGD( "xin:===set_adc_clk" );
	unsigned int data =0;
	data = sread( 0 );
	switch( adcClk )
	{
		case  DefaultClk: //26.2144			      
		      break;
		case  DefaultHalf: //26.2144/2		      
			  data |=( 0x1<<5 );			
		      break;
		case  DefaultFourth:  //26.2144/4		      
			  data |=( 0x1<<6 );		     
		      break;
		case  DefaultEighth:  //26.2144/8			  
			  data |=( 0x3<<5 );			  
		      break;	
		default:
		       break;
	}
	swrite( 0 , data );
	//sread( 0 );	
}

//HighSpeed , HighPrecision , LowPower , LowSpeed
//0         1              2          3
//DefaultClk , DefaultHalf , DefaultFourth , DefaultEighth   
//000        001         010           011
void set_sample_rate( int sampleRate )
{
	//LOGD( "xin:===set_sample_rate" );	
	switch( sampleRate )
	{
		case SMP_RATE_102400: //OK
		     LOGD( "enter===SMP_RATE_102400" );             		
			 set_adc_mode( HighSpeed );		 
			 set_adc_clk( DefaultClk );	//22.2144			 			 
			 break;		
		case SMP_RATE_51200:  //OK
		     LOGD( "enter===SMP_RATE_51200" );
		     set_adc_mode( HighPrecision );
			 set_adc_clk( DefaultClk );				            			 
			 break;
		case SMP_RATE_25600: //ok
			 LOGD( "enter===SMP_RATE_25600" );  //�ɵ͹���ģʽ��Ϊ�߾���ģʽ  
		     set_adc_mode( LowPower );		 
			 set_adc_clk( DefaultHalf );
             //set_adc_mode( HighPrecision );		 
			 //set_adc_clk( DefaultHalf );             			 
			 break;
		case SMP_RATE_12800: //OK
		     LOGD( "enter===SMP_RATE_12800" );//��������Ϊ12800ʱ����25.6K �����ʽ��� 1/2���ʵ��
		     //set_adc_mode( HighPrecision );		 
			 //set_adc_clk( DefaultFourth );	
             set_adc_mode( HighPrecision );		 
			 set_adc_clk( DefaultHalf );			 
			 break;		
        case SMP_RATE_10240: //OK
		    LOGD( "enter===SMP_RATE_10240" );
		    set_adc_mode( LowSpeed );		 
			set_adc_clk( DefaultClk );
			break;		
		case SMP_RATE_6400:
		    LOGD( "enter===SMP_RATE_6400" );//��������Ϊ6400ʱ����25.6K �����ʽ��� 1/4���ʵ��
		    //set_adc_mode( LowPower );		 
			//set_adc_clk( DefaultEighth );
            set_adc_mode( HighPrecision );		 
			set_adc_clk( DefaultHalf );			
			break;			 
	    case SMP_RATE_5120:
		    LOGD( "enter===SMP_RATE_5120" );
		    set_adc_mode( LowSpeed );		 
			set_adc_clk( DefaultHalf );	
			break;
		case SMP_RATE_2560:
		    LOGD( "enter===SMP_RATE_2560" );//��������Ϊ2560ʱ����5.12K �����ʽ��� 1/2���ʵ��
		    //set_adc_mode( LowSpeed );		 
			//set_adc_clk( DefaultFourth );
			set_adc_mode( LowSpeed );		 
			set_adc_clk( DefaultHalf );
			break;
		case SMP_RATE_1280:
		    LOGD( "enter===SMP_RATE_1280" ); //��������Ϊ1280ʱ����5.12K �����ʽ��� 1/4���ʵ��
		    //set_adc_mode( LowSpeed );		 
			//set_adc_clk( DefaultEighth );
			set_adc_mode( LowSpeed );		 
			set_adc_clk( DefaultHalf );
			break;	
		default:
		     break;
	}	
}

int get_voltage_range( int ch )//�õ���ѹ���� OK
{
	unsigned int data=0;
	unsigned int reg_addr =0;
	int range=0;	
	switch( ch )
	{
		case CHA:
			reg_addr = 0x4;
			break;
		case CHB:
			reg_addr = 0x8;
			break;
		 default:break;
	}
	data=sread( reg_addr );
	data =data >>3;
	data &=0x7;
    switch( data )
    {
	   case 0:
		   range= V25;
		   break;
	   case 1:
		   range= V2P5;
		   break;
	   case 2:
		   range= V0P25;
		   break;
	   case 3:
		   range= VAUTO;
		   break;
	   default:break;
    }
	return range;
}

void set_integrate( int ch ,  int integrateType )//���û���ѡ�� OK
{
	//LOGD( "xin:===set_integrate" );
	unsigned int data = 0;
	unsigned int reg_addr =0;
	switch( ch )
	{
		case CHA:
			reg_addr = 0x4;
			break;
		case CHB:
			reg_addr = 0x8;
			break;
		default:break;
	}
	data=sread( reg_addr );	
	switch( integrateType )
	{
		case SPEED:
			data |=( 0x1<<6 );
			break;
		case SHIFT:
		    data &=( ~(( unsigned int )( 0x1<<6 ) ) );
			break;
		default:break;
	}
	swrite( reg_addr , data );
	//sread( reg_addr );
}

int get_integrate( int ch )//�õ�����ѡ�� OK
{
	unsigned int data=0;
	unsigned int reg_addr =0;
	int integ_type=0;	
	switch( ch )
	{
		case CHA:
			reg_addr = 0x4;
			break;
		case CHB:
			reg_addr = 0x8;
			break;
		default:break;
	}
	data=sread( reg_addr );
	data =data >>6;
	data &=0x1;
	switch( data )
	{
		case 0:
			integ_type= SHIFT;
			break;
		case 1:
			integ_type= SPEED;
			break;
		default:break;
	}
	return integ_type;
}

////////////////////Speed_Config_REG   0x000C
void set_speed_resistance( float voltage )//����ת��ͨ���ķ�ѹ����ֵ-----------
{
	//LOGD( "xin:===set_speed_resistance" );
	unsigned int data = 0;
	unsigned int reg_addr =0;
	enable_rotation( true ); //ת��ʹ��
	data = sread( 0xc );	
	//data = 10*1000*( 1-voltage/128 );
	swrite( 0xc , data );
	//sread( 0xc );
}

void set_speed_triger_mode( int speedTriMode )//����ת�ٴ�����ʽ  OK
{
	//LOGD( "xin:===set_speed_triger_mode" );
	unsigned int data = 0;
	data=sread( 0xc );// 0x000c	
	switch( speedTriMode )
	{
		case AnalogFallingCount ://�½��ش���������		00	
			break;
		case AnalogRisingCount: //�����ش���������	    01
			data |=( 0x1 << 8 );
			break;
		default:break;
	}  
	swrite( 0xc , data );
	//sread( 0xc );
}

int get_speed_triger_mode(  )//�õ�ת�ٴ�����ʽ  OK
{
	unsigned int data = 0;
	int tri_mode =0;
	data = sread( 0xc );	
	data = data >>8;
	data &= 0x1;	
	switch( data )
	{
		case 0:
			tri_mode = AnalogFallingCount;
			break;
		case 1:
			tri_mode = AnalogRisingCount;
			break;
		default:break;
	}
   return tri_mode;
}

////////////////////Start_Trig_Ctrl_REG  0x0010
void set_triger_ch( int ch )//����ģ�ⴥ��ͨ��  OK
{
	//LOGD( "xin:===set_triger_ch" );
	unsigned int data = 0;
	unsigned int reg_addr =0x10;
	data=sread( reg_addr );	
	switch( ch )
	{
		case CHA:
		    data &=( ~(( unsigned int )( 0xf ) ) );
			break;
		case CHB:
			data |=0x1;
			break;
		default:break;
	}
	swrite( reg_addr , data );
	//sread( reg_addr );
}

int get_triger_ch(  )//�õ�ģ�ⴥ��ͨ��  OK
{
	unsigned int data = 0;
	unsigned int reg_addr =0x10;
	int ch_num =0;
	data = sread( reg_addr );
	data &=( ~(( unsigned int )( 0xf ) ) );	
	switch(( int )data )
	{
		case 0:
			ch_num = CHA;
			break;
		case 1:
			ch_num = CHB;
			break;
		default:break;
	}
   return ch_num;
}

void set_triger_mode( int triMode )//���ÿ�ʼ�ɼ��Ĵ�����ʽ  OK
{
	//LOGD( "xin:===set_triger_mode" );
	unsigned int data = 0;
	data=sread( 0x10 );// 0x0010	
	switch( triMode )
	{
		case  Manual: //( ���� )�ֶ�����		
			break;
		case  RTriger:  //ת�ٴ���
			data |=( 0x1 << 4 );
			break;
		case  AnalogRising://ģ�������ش���			
			data |=( 0x2 << 4 );
			break;
		case  AnalogFalling: //ģ���½��ش���			
			data |=( 0x3 << 4 );
			break;
		default:break;
	}  
	swrite( 0x10 , data );
	//sread( 0x10 );
}

int get_triger_mode(  )//�õ�������ʽ OK
{
	unsigned int data = 0;
	int tm = 0;
	data=sread( 0x10 );
	data = data >> 4;
	data &= 0x3;    
	switch(( int )data )
	{
		case 0:
			tm = Manual; //( ���� )�ֶ�����
			break;
		case 1:
			tm = RTriger;//ת�ٴ���
			break;
		case 2:
			tm = AnalogRising; //ģ�������ش���
			break;
		case 3:
			tm = AnalogFalling; //ģ���½��ش���
			break;
		 default:break;
	}
	return tm;
}

void set_triger_threshold( double threshold )//���ô�����ֵ
{
	//LOGD( "xin:===set_triger_threshold" );
	swrite( 0x14 ,( unsigned int )threshold );
	//sread( 0x14 );	
}

double  get_triger_threshold(  )//�õ�������ֵ
{
	return( double )sread( 0x14 );
}

//////////////////Status_REG   0x0044

bool get_adc_work_status(  )//�õ�adc�Ĺ���״̬ OK
{
	unsigned int data=0;
	data = sread( 0x44 );
	data = data >>6;
	data &= 0x1;
	if( data ==0x1 )
		return true;
	else
		return false;
}

bool get_rotation_sample_status(  )//�õ�ת�ٲɼ�״̬ OK
{
	unsigned int data=0;
	data = sread( 0x44 );
	data = data >>7;
	data &= 0x1;
	if( data ==0x1 )
		return true;
	else
		return false;
}

//////////////Analog_FIFO_Status_REG 0x0048
int get_analog_fifo_byte(  )//�õ�ģ��fifo�е��ֽ��� 0K
{
	return( int )sread( 0x48 );
}

int spi_freq(  )  // �ɼ���ɼ�Ƶ��
{
	//LOGD( "xin:===spi_freq" );
	return SPI_SAMPLE;
}

void poweron_spi(  )//SPI POWER ON
{
	LOGD( "xin:===poweron_spi�豸�ϵ�" );
	GpioOpen(  );
	GpioSet( FPGA_3V3_CTR , GPIO_SET_ON );
	GpioSet( FPGA_1V2_CTR , GPIO_SET_ON );
	GpioSet( FPGA_2V5_CTR , GPIO_SET_ON );
	usleep(130000);
}

void poweroff_spi(  )//SPI POWER OFF
{
    LOGD( "xin:===poweroff_spi�豸�µ�" );    
	GpioSet( FPGA_3V3_CTR , GPIO_SET_OFF );	
	GpioSet( FPGA_2V5_CTR , GPIO_SET_OFF );
	GpioSet( FPGA_1V2_CTR , GPIO_SET_OFF );	
    usleep( 100000 );	
}

void reset_fpga_reg(  ) //��λFPGA�ļĴ���
{
	LOGD( "xin:===reset_fpga_reg" ); 
	swrite( ResetFpgaRegAddr , ResetFpgaRegData );
	usleep( 50000 );
}
	
struct main_fpga_reg para={0.0 , 0.0 , 0.0 , false , false , 0};
int set_press_reg( int smp_rate ) //����ѹ���ɼ��Ĵ�����Ĭ��CHA
{
	LOGD( "xin:=== set_press_reg==========start" ); 
	//poweron_spi(  );//�ϵ����ʱ100 ms
	//usleep( 130000 );	//130ms
	
	int error_value=0;
	if( masterspi_open(  ) == -1 )
		error_value = -1;//�����豸�쳣
	//----------------
	//reset_fpga_reg(  ); //��λ�Ĵ���
	error_value = enable_CH_A( true );//ʹ��CH1
	set_couple_mode( CHA ,  DC );//DC��� ,   AC , DC
	set_24V( CHA ,  true );//true��ʾ����24v��Դ�Ŀ��ؼ�������ʵѹ������������   �� false ��ʾ������24V��������������֤ 
	
	//set_sample_rate( smp_rate );	
	//---------------
	//LOGD( "set_press_reg error_value = %d" , error_value );	
	LOGD( "xin:=== set_press_reg==========end" ); 
	sread( 0 );
	sread( 0x4 );		
	return error_value;	
}

int set_singleCH_vibrate_reg( int signalType , float maxFreq ,  float minFreq ) //�����񶯲ɼ��Ĵ�������ͨ����Ĭ��CHB
{
	LOGD( "xin:=== set_singleCH_vibrate_reg==========start" ); 
	//poweron_spi(  );
	//usleep( 130000 );	
	
	int error_value=0;
	if( masterspi_open(  ) == -1 )
	{
		error_value = -1;//�����豸�쳣
	}
	
	reset_fpga_reg(  ); //��λ�Ĵ���
	
			
	if( signalType == 0 ) //0�����ٶȣ�1���ٶȣ�2��λ��
	{   
	    //LOGD( "xin:=== signalType == 0 ,  ACC" );
	    error_value = enable_CH_B( true );//ʹ��CHA
        if( minFreq < 7 ) //С��7Hz���½�����DC��ϣ����ڵ���7HZ ��AC���
		{
			//LOGD( "xin:=== minFreq <7HZ ,  DC" ); 
			set_couple_mode( CHB ,  DC );//��Ϸ�ʽDC
		}else{
			LOGD( "xin:=== minFreq >7HZ ,  AC" );
			//set_couple_mode( CHB ,  AC );//��Ϸ�ʽ   , Ĭ��AC
		}		
	}else if( signalType == 1 ) //�ٶ�
	{  
	    LOGD( "xin:=== signalType == 1 ,  SPEED" ); 
		enable_CH_B_integrate( true );//ʹ��CHA�Ļ���
		set_integrate( CHB ,  SPEED );//���û���ѡ�� ,   �ٶ�
		if( minFreq < 10 ) //С��10Hz���½�����DC��ϣ����ڵ���10HZ ��AC���
		{
			//LOGD( "xin:=== minFreq <10HZ ,  DC" ); 
			set_couple_mode( CHB ,  DC );//��Ϸ�ʽDC
		}else{
			LOGD( "xin:=== minFreq >10HZ ,  AC" );
			//set_couple_mode( CHB ,  AC );//��Ϸ�ʽ   , Ĭ��AC
		}
	}else if( signalType == 2 ) //λ��
	{
		LOGD( "xin:=== signalType == 2 ,  SHIFT" ); 
		enable_CH_B_integrate( true );//ʹ��CHA�Ļ���
		//set_integrate( CHB ,  SHIFT );//���û���ѡ�� ,   Ĭ��Ϊλ��
		if( minFreq < 10 ) //С��10Hz���½�����DC��ϣ����ڵ���10HZ ��AC���
		{
			//LOGD( "xin:=== minFreq <10HZ ,  DC" ); 
			set_couple_mode( CHB ,  DC );//��Ϸ�ʽDC
		}else{
			LOGD( "xin:=== minFreq >10HZ ,  AC" );
			//set_couple_mode( CHB ,  AC );//��Ϸ�ʽ   , Ĭ��AC
		}
	}	
	
	set_sample_rate(( int )maxFreq*2.56 ); //���ò���Ƶ�ʣ���������ADC CLK��ADC_MODE��
        
#if 0  //���հ汾Ҫ��,��adc������ȡҪͳһ��  *10������ѹ���ɼ�Ҫ ȥ�� *10 ����
    set_24V( CHB ,  true );//����24v��Դ�Ŀ��ؼ��� , Ĭ�ϲ����������հ汾��Ҫ��Ϊ true
	//set_voltage_range( CHB ,  V2P5 );//���õ�ѹ���� , Ĭ��V25 ,   V25  , V2.5 ,  V0.25	
#else
    //set_24V( CHB ,  true );//����24v��Դ�Ŀ��ؼ��� , Ĭ�ϲ����������հ汾��Ҫ��Ϊ true
	set_voltage_range( CHB ,  V2P5 );//���õ�ѹ���� , Ĭ��V25 ,   V25  , V2.5 ,  V0.25    
#endif
    	
	//set_triger_mode( Manual ); //���ÿ�ʼ�ɼ��Ĵ�����ʽ ,  Manual , RTriger , AnalogRising , AnalogFalling

	//set_triger_threshold( para.trig_value );//���ô�����ֵ----
	//LOGD( "set_singleCH_vibrate_reg error_value = %d" , error_value );
	LOGD( "xin:=== set_singleCH_vibrate_reg==========end" ); 	
	sread( 0 );
	sread( 0x8 );	
	return error_value;	
}








int set_doubleCH_vibrate_reg( int signalType , float maxFreq ,  float minFreq )//�����񶯲ɼ��Ĵ���
{
	//LOGD( "xin:=== set_doubleCH_vibrate_reg==========start" ); 
	//poweron_spi(  );//�ϵ����ʱ100 ms
	//usleep( 130000 );	//100ms
	
	int error_value=0;
	int error_value1=0;
	int error_value2=0;
	if( masterspi_open(  ) == -1 )
		error_value = -1;//�����豸�쳣
	
	reset_fpga_reg(  ); //��λ�Ĵ���
		
	//--------------CH1 CH2
			
    if( signalType == 0 ) //���ٶ�
	{		
		LOGD( "xin:=== signalType == 0 ,  ACC" );
		error_value1 = enable_CH_A( true );//ʹ��CHA
		error_value2 = enable_CH_B( true );//ʹ��CHA
		if( minFreq < 7 )  //С��7Hz���½�����DC��ϣ����ڵ���7HZ ��AC���
		{
			//LOGD( "xin:=== minFreq <7HZ ,  DC" ); 
			set_couple_mode( CHA ,  DC );//��Ϸ�ʽ DC
			set_couple_mode( CHB ,  DC );//��Ϸ�ʽ DC
		}else{
			//LOGD( "xin:=== minFreq >7HZ ,  AC" );
			//set_couple_mode( CHA ,  AC );//��Ϸ�ʽ ,  Ĭ��ΪAC
			//set_couple_mode( CHB ,  AC );//��Ϸ�ʽ ,  Ĭ��ΪAC
		}
	}else if( signalType == 1 ) //�ٶ�
	{
		LOGD( "xin:=== signalType == 1 ,  SPEED" );
		enable_CH_A_integrate( true );//ʹ��CHA�Ļ���
		enable_CH_B_integrate( true );//ʹ��CHA�Ļ���
		set_integrate( CHA ,  SPEED );//���û���ѡ�� ,   �ٶ�		
		set_integrate( CHB ,  SPEED );//���û���ѡ�� ,   �ٶ�
		if( minFreq < 10 )  //С��10Hz���½�����DC��ϣ����ڵ���10HZ ��AC���
		{
			//LOGD( "xin:=== minFreq <10HZ ,  DC" ); 
			set_couple_mode( CHA ,  DC );//��Ϸ�ʽ DC
			set_couple_mode( CHB ,  DC );//��Ϸ�ʽ DC
		}else{
			//LOGD( "xin:=== minFreq >10HZ ,  AC" );
			//set_couple_mode( CHA ,  AC );//��Ϸ�ʽ ,  Ĭ��ΪAC
			//set_couple_mode( CHB ,  AC );//��Ϸ�ʽ ,  Ĭ��ΪAC
		}
	}else if( signalType == 2 ) //λ��
	{
		LOGD( "xin:=== signalType == 2 ,  SHIFT" ); 
		enable_CH_A_integrate( true );//ʹ��CHA�Ļ���
		enable_CH_B_integrate( true );//ʹ��CHA�Ļ���
		//set_integrate( CHA ,  SHIFT );//���û���ѡ�� ,  Ĭ��Ϊλ��		
		//set_integrate( CHB ,  SHIFT );//���û���ѡ�� ,  Ĭ��Ϊλ��
		if( minFreq < 10 )  //С��10Hz���½�����DC��ϣ����ڵ���10HZ ��AC���
		{
			//LOGD( "xin:=== minFreq <10HZ ,  DC" ); 
			set_couple_mode( CHA ,  DC );//��Ϸ�ʽ DC
			set_couple_mode( CHB ,  DC );//��Ϸ�ʽ DC
		}else{
			//LOGD( "xin:=== minFreq >10HZ ,  AC" );
			//set_couple_mode( CHA ,  AC );//��Ϸ�ʽ ,  Ĭ��ΪAC
			//set_couple_mode( CHB ,  AC );//��Ϸ�ʽ ,  Ĭ��ΪAC
		}
	}		
	
	set_sample_rate(( int )maxFreq*2.56 );  //���ò���Ƶ�ʣ���������ADC CLK��ADC_MODE��	

	

	set_24V( CHA ,  true );//����24v��Դ�Ŀ��ؼ��� , Ĭ�ϲ����� ���հ汾��Ҫ��Ϊ true
	set_24V( CHB ,  true );//����24v��Դ�Ŀ��ؼ��� , Ĭ�ϲ�����  ���հ汾��Ҫ��Ϊ true
	//set_voltage_range( CHA ,  V2P5 );//���õ�ѹ����  , Ĭ��V25 ,   V25  , V2.5 ,  V0.25
	//set_voltage_range( CHB ,  V2P5 );//���õ�ѹ���� ,   V25  , V2.5 ,  V0.25
	
	
	//set_triger_mode( Manual ); //���ÿ�ʼ�ɼ��Ĵ�����ʽ ,  Manual , RTriger , AnalogRising , AnalogFalling
	
	LOGD( "xin:=== set_doubleCH_vibrate_reg==========end" );
	sread( 0 );
	sread( 0x4 );
	sread( 0x8 );	
	if( error_value1 == -1 ||error_value2 == -1)
	{
        error_value = -1;
    }
	
	return error_value;	
}

int set_rotation_reg(  )//����ת�ٲɼ��Ĵ���
{
	LOGD( "xin:=== set_rotation_reg==========start" );
	poweron_spi(  );//�ϵ����ʱ100 ms
	usleep( 30000 );	//100ms
	
	int error_value=0;
	if( masterspi_open(  ) == -1 )
		error_value = -1;//�����豸�쳣
	//----------------
	reset_fpga_reg(  ); //��λ�Ĵ���
	
	enable_rotation( true );//ʹ��ת��ͨ��
	set_speed_resistance( para.rotation_resis_value );//����ת�ٷ�ѹ����ֵ
	set_speed_triger_mode( AnalogFallingCount );//����ת�ٴ�����ʽ  , AnalogFallingCount , AnalogRisingCount
	//---------------
	LOGD( "xin:=== set_rotation_reg==========end" );
	sread( 0 );
	
	return error_value;
}

char* log_time(  )
{
	struct  tm      *ptm;
	struct  timeb   stTimeb;
	static  char    szTime[19];
	ftime( &stTimeb );
	ptm = localtime( &stTimeb.time );
	sprintf( szTime ,  "%02d-%02d %02d:%02d:%02d.%03d" ,  ptm->tm_mon+1 ,  ptm->tm_mday ,  ptm->tm_hour ,  ptm->tm_min ,  ptm->tm_sec ,  stTimeb.millitm );
	szTime[18] = 0;
	return szTime;
}