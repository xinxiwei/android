#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <hardware/log.h>
#include <hardware/math.h>
#include <hardware/gpio.h>
#include <hardware/masterspi.h>
#include <sys/timeb.h>
#include <time.h>
#include <hardware/imx6q_vibrate_calibrate_config.h>
#include <hardware/imx6q_vibrate_calibrate.h>
#include <hardware/imx6q_common_config.h>

#define   SMP_RATE_102400     102400   //除以2.56 =40K
#define   SMP_RATE_51200      51200    //20K
#define   SMP_RATE_25600      25600    //10K
#define   SMP_RATE_12800      12800    //5K
#define   SMP_RATE_10240      10240    //4K
#define   SMP_RATE_6400       6400     //2.5K
#define   SMP_RATE_5120       5120     //2K
#define   SMP_RATE_2560       2560     //1K
#define   SMP_RATE_1280       1280     //0.5K

typedef enum{
    HighSpeed = 0, //高速
    HighPrecision, //高精度
    LowPower,      //低功耗
    LowSpeed       //低速
}ADCModeEnum;//ADC_MODE模式

typedef enum{
    DefaultClk = 0, //26.2144MHZ
    DefaultHalf,    //26.2144MHZ/2
    DefaultFourth,  //26.2144MHZ/4
    DefaultEighth   //26.2144MHZ/8
}ADCClkEnum;//ADC_CLK 时钟速率

typedef enum{
    AC = 0, 
    DC
}CoupleModeEnum;//耦合方式

typedef enum{
    VOLTAGE = 0,   //电压采集
    CURRENT        //电流采集
}SampleTypeEnum;//采集类型

typedef enum{
    V25 = 0,  //25V
    V2P5,     //2.5V
    V0P25,    //0.25V
    VAUTO     //自动量程
}VoltageWindowEnum; //量程选择

typedef enum{
	SHIFT = 0, //位移积分
    SPEED      //速度积分
}IntegrateTypeEnum;//积分选择

typedef enum{
	AnalogFallingCount = 0, //下降沿触发计数器
	AnalogRisingCount       //上升沿触发计数器
}SpeedTrigerEnum; //转速触发计数器

typedef enum{
    CHA = 0,  //采集通道1
    CHB,      //采集通道2
    CHR       //转速通道
}CHEnum;//通道选择

typedef enum{
    Manual = 0,   //手动触发
    RTriger,      //转速触发
    AnalogRising, //模拟上升沿触发
    AnalogFalling //模拟下降沿触发
}TrigerEnum; //模拟触发方式


/////////////////Ctrl_REG  0x0000
void enable_rotation(bool flag)//使能转速通道 OK
{
	//LOGD("xin:===enable_rotation");
	unsigned int data = sread(0);
	if(flag)
	{
		data |= 0x1;
	}
	else
	{
		data &=(~((unsigned int)(0x1)));
	}
	swrite(0, data);
	//sread(0);
}

int  enable_CH_A(bool flag)//使能震动CHA OK
{
	LOGD("xin:===enable_CH_A");

	unsigned int data = sread(0);
	if(flag)
	{
		data |= 0x2;
	}
	else
	{
		data &=(~((unsigned int)(0x2)));
	}
	swrite(0, data);
	//sread(0);
	LOGD("xin: enable_CH_A data = %d",data);
	return data;
}

int enable_CH_A_integrate(bool flag)//使能CHA的硬件积分 OK
{
	LOGD("xin:===enable_CH_A_integrate");
	unsigned int data = sread(0);
	if(flag)
	{
		data |= 0x4;
	}
	else
	{
		data &=(~((unsigned int)(0x4)));
	}
	swrite(0, data);
	//sread(0);
	return data;
}

int enable_CH_B(bool flag)//使能震动CHB OK
{
	LOGD("xin:===enable_CH_B");
	unsigned int data = sread(0);
	if(flag)
	{
		data |= 0x8;
	}
	else
	{
		data &=(~((unsigned int)(0x8)));
	}
	swrite(0, data);
	//sread(0);
	return data;
}

int enable_CH_B_integrate(bool flag)//使能CHB的硬件积分 OK
{
	LOGD("xin:===enable_CH_B_integrate");
	unsigned int data = sread(0);
	if(flag)
	{
		data |= 0x10;
	}
	else
	{
		data &=(~((unsigned int)(0x10)));
	}
	swrite(0, data);
	//sread(0);
    return data;
}

void set_adc_mode( int adcMode )//设置ADC的工作模式 OK
{
	//LOGD("xin:===set_adc_mode");
	unsigned int data =sread(0);//adcMode  reg  0x0000
	switch(adcMode)
	{
		case HighSpeed:  //00
			break;
		case HighPrecision: //01
			data |=(0x1<<8);
			break;
		case LowPower:   //10
			data |=(0x2<<8);
			break;
		case LowSpeed:  //11
			data |=(0x3<<8);
			break;
	}
	swrite(0, data);
	//sread(0);
}

int get_adc_mode()//得到adc的工作模式 OK
{
	//LOGD("xin:===get_adc_mode");
	unsigned int data = 0;
	int mode =0;
	data=sread(0);
	data = data>>8;//7
	data &=0x3;
	switch(data)
	{
		case 0:
		    mode = HighSpeed;//高速模式
			break;
		case 1:
			mode = HighPrecision;//高精度模式
			break;
		case 2:
			mode = LowPower;//低功耗
			break;
		case 3:
			mode = LowSpeed;//低速模式
			break;
	}
    return mode;
}

float get_adc_clk_rate()//得到ADC时钟速率 OK  HZ
{
	unsigned int data = 0;
	data=sread(0); //ADC_CLK选择  寄存器0x0000
	data = data >> 5;
	data &=0x7;
	float ret=0.0;
	float clk_value = 26214400.0;//Hz
	switch(data)
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

void set_adc_clk_rate(float sampleRate)//设置ADC时钟速率 -------------
{
	LOGD("xin:===set_adc_clk_rate");
	int adc_mode= get_adc_mode();
	int A=0;
	switch(adc_mode)
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

	float real_clk = 0.0;
	real_clk = A * sampleRate*1024;  //得到真实的下发频率 单位HZ

	double dataGap = 26.2144 * 1024*1024/16;
	double clk_value = dataGap *16;
	unsigned int data = 0;
	data = sread(0);
	if(fabs(clk_value-real_clk) < dataGap)//000
	{
		data &= 0xffffffef;
		swrite(0, data);
	}
	else if(fabs((clk_value/2)-real_clk) < dataGap)//001
	{
		data &= 0xffffffef;
		data |=(0x1<<4);
		swrite(0, data);
	}
	else if(fabs((clk_value/4)-real_clk) < dataGap)//010
	{
		data &= 0xffffffef;
		data |=(0x1<<5);
		swrite(0, data);
	}
	else if(abs((clk_value/8)-real_clk) < dataGap)//011
	{
		data &= 0xffffffef;
		data |=(0x1<<5);
		data |=(0x1<<4);
		swrite(0, data);
	}
	sread(0);
}

float get_sample_rate()//得到采样率  单位HZ   OK
{
	float data = 0.0;
	float clk_freq = get_adc_clk_rate();
	int adc_mode = get_adc_mode();
	int A=0;
	switch(adc_mode)
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

/////////////CH1_Config_REG  0x0004, CH2_Config_REG 0x0008
void set_couple_mode(int ch, int coupleMode)//设置每个通道对应的耦合方式  OK
{
	LOGD("xin:===set_couple_mode");
	unsigned int data=0;
	unsigned int reg_addr =0;
	switch(ch)
	{
		case CHA:
			reg_addr = 0x4;
			break;
		case CHB:
			reg_addr = 0x8;
			break;
		 default:break;
	}
	data=sread(reg_addr);
	switch(coupleMode)
	{
		case AC:  // 0
			data &=(~((unsigned int)(0x1)));
			break;
		case DC:  // 1
			data |=0x1;
			break;
		 default:break;
	}
	swrite(reg_addr, data);
	sread(reg_addr);
}

int get_couple_mode(int ch)//得到耦合方式 OK
{
	unsigned int data=0;
	unsigned int reg_addr =0;
	switch(ch)
	{
		case CHA:
			reg_addr = 0x4;
			break;
		case CHB:
			reg_addr = 0x8;
			break;
		 default:break;
	}
	data=sread(reg_addr);
	data &=0x1;
	if(data == 0x0)
		return AC;
	else
		return DC;
}

void set_sample_type(int ch, int sampleType)//设置采集类型  OK
{
	//LOGD("xin:===set_sample_type");
	unsigned int data=0;
	unsigned int reg_addr =0;
	switch(ch)
	{
		case CHA:
			reg_addr = 0x4;
			break;
		case CHB:
			reg_addr = 0x8;
			break;
		 default:break;
	}
	data=sread(reg_addr);
	switch(sampleType)
	{
		case CURRENT://电流1
			data |=0x2;
			break;
		case VOLTAGE://电压0
			data &=(~((unsigned int)(0x2)));
			break;
		 default:break;
	}
	swrite(reg_addr, data);
	//sread(reg_addr);
}

int get_sample_type(int ch)//得到采集类型 OK
{
	unsigned int data=0;
	unsigned int reg_addr =0;
	switch(ch)
	{
		case CHA:
			reg_addr = 0x4;
			break;
		case CHB:
			reg_addr = 0x8;
			break;
		 default:break;
	}
	data=sread(reg_addr);
	data &=0x2;
	if(data == 0x0)
		return VOLTAGE; //电压0
	else
		return CURRENT; //电流1
}

void set_24V(int ch, bool flag)//设置24v电源的开关 OK
{
	LOGD("xin:===set_24V, flag = %d",flag);
	unsigned int data=0;
	unsigned int reg_addr =0;
	switch(ch)
	{
		case CHA:
			reg_addr = 0x4;
			break;
		case CHB:
			reg_addr = 0x8;
			break;
		 default:break;
	}

	data=sread(reg_addr);
	if(flag)
		data |= 0x4;  //启用24V@2.4mA激励
	else
		data &=(~((unsigned int)(0x4)));
	swrite(reg_addr, data);
	sread(reg_addr);
}

bool get_24V(int ch)//得到24v电源的开关状态  OK
{
	unsigned int data=0;
	unsigned int reg_addr =0;
	switch(ch)
	{
		case CHA:
			reg_addr = 0x4;
			break;
		case CHB:
			reg_addr = 0x8;
			break;
		 default:break;
	}
	data=sread(reg_addr);
	data &=0x4;
	if(data==0x0)
		return false;
	else
		return true;
}

void set_voltage_range(int ch, int voltageRange)//设置电压量程 OK
{
	unsigned int data=0;
	unsigned int reg_addr =0;
	switch(ch)
	{
		case CHA:
			reg_addr = 0x4;
			break;
		case CHB:
			reg_addr = 0x8;
			break;
		default:
            break;
	}
	data=sread(reg_addr);
	switch(voltageRange)
	{
		case V25:		//0
            LOGD("xin:===set_voltage_range  = %d, 电压量程为 25V", voltageRange);
			break;
		case V2P5:      //1
            LOGD("xin:===set_voltage_range  = %d, 电压量程为 2.5V", voltageRange);
			data |=(0x1<<3);
			break;
		case V0P25:     //2
            LOGD("xin:===set_voltage_range  = %d, 电压量程为 0.25V", voltageRange);
			data |=(0x2<<3);
			break;
		case VAUTO:     //3
			data |=(0x3<<3);
			break;
		 default:break;
	}
	swrite(reg_addr, data);
	sread(reg_addr);
}

void set_adc_clk(int adcClk)
{
	//LOGD("xin:===set_adc_clk");
	unsigned int data =0;
	data = sread(0);
	switch(adcClk)
	{
		case  DefaultClk: //26.2144
		      break;
		case  DefaultHalf: //26.2144/2
			  data |=(0x1<<5);
		      break;
		case  DefaultFourth:  //26.2144/4
			  data |=(0x1<<6);
		      break;
		case  DefaultEighth:  //26.2144/8
			  data |=(0x3<<5);
		      break;
		default:
		       break;
	}
	swrite(0, data);
	//sread(0);
}

//HighSpeed, HighPrecision, LowPower, LowSpeed
//0         1              2          3
//DefaultClk, DefaultHalf, DefaultFourth, DefaultEighth
//000       001           010         011
void set_sample_rate(int sampleRate)
{
	//LOGD("xin:===set_sample_rate");
	switch(sampleRate)
	{
		case SMP_RATE_102400: //OK
		     LOGD("enter===SMP_RATE_102400");
			 set_adc_mode(HighSpeed);
			 set_adc_clk(DefaultClk);	//22.2144
			 break;
		case SMP_RATE_51200:  //OK
		     LOGD("enter===SMP_RATE_51200");
		     set_adc_mode(HighPrecision);
			 set_adc_clk(DefaultClk);
			 break;
		case SMP_RATE_25600: //ok
			 LOGD("enter===SMP_RATE_25600");
		     set_adc_mode(LowPower);
			 set_adc_clk(DefaultHalf);
             //set_adc_mode(HighPrecision);
			 //set_adc_clk(DefaultHalf);
			 break;
		case SMP_RATE_12800: //OK
		     LOGD("enter===SMP_RATE_12800");//采样速率为12800时，用25.6K 采样率进行 1/2抽点实现
		     //set_adc_mode(HighPrecision);
			 //set_adc_clk(DefaultFourth);
             set_adc_mode(HighPrecision);
			 set_adc_clk(DefaultHalf);
			 break;
        case SMP_RATE_10240: //OK
		    LOGD("enter===SMP_RATE_10240");
		    set_adc_mode(LowSpeed);
			set_adc_clk(DefaultClk);
			break;
		case SMP_RATE_6400:
		    LOGD("enter===SMP_RATE_6400");//采样速率为6400时，用25.6K 采样率进行 1/4抽点实现
		    //set_adc_mode(LowPower);
			//set_adc_clk(DefaultEighth);
            set_adc_mode(HighPrecision);
			set_adc_clk(DefaultHalf);
			break;
	    case SMP_RATE_5120:
		    LOGD("enter===SMP_RATE_5120");
		    set_adc_mode(LowSpeed);
			set_adc_clk(DefaultHalf);
			break;
		case SMP_RATE_2560:
		    LOGD("enter===SMP_RATE_2560");//采样速率为2560时，用5.12K 采样率进行 1/2抽点实现
		    //set_adc_mode(LowSpeed);
			//set_adc_clk(DefaultFourth);
			set_adc_mode(LowSpeed);
			set_adc_clk(DefaultHalf);
			break;
		case SMP_RATE_1280:
		    LOGD("enter===SMP_RATE_1280"); //采样速率为1280时，用5.12K 采样率进行 1/4抽点实现
		    //set_adc_mode(LowSpeed);
			//set_adc_clk(DefaultEighth);
			set_adc_mode(LowSpeed);
			set_adc_clk(DefaultHalf);
			break;
		default:
		     break;
	}
}

int get_voltage_range(int ch)//得到电压量程 OK
{
	unsigned int data=0;
	unsigned int reg_addr =0;
	int range=0;
	switch(ch)
	{
		case CHA:
			reg_addr = 0x4;
			break;
		case CHB:
			reg_addr = 0x8;
			break;
		 default:break;
	}
	data=sread(reg_addr);
	data =data >>3;
	data &=0x7;
    switch(data)
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

void set_integrate(int ch, int integrateType)//设置积分选择 OK
{
	LOGD("xin:===set_integrate");
	unsigned int data = 0;
	unsigned int reg_addr =0;
	switch(ch)
	{
		case CHA:
			reg_addr = 0x4;
			break;
		case CHB:
			reg_addr = 0x8;
			break;
		default:break;
	}
	data=sread(reg_addr);
	switch(integrateType)
	{
		case SPEED: // 1
			data |=(0x1<<6);
			break;
		case SHIFT: // 0
		    data &=(~((unsigned int)(0x1<<6)));
			break;
		default:break;
	}

	swrite(reg_addr, data);
	sread(reg_addr);
}

int get_integrate(int ch)//得到积分选择 OK
{
	unsigned int data=0;
	unsigned int reg_addr =0;
	int integ_type=0;
	switch(ch)
	{
		case CHA:
			reg_addr = 0x4;
			break;
		case CHB:
			reg_addr = 0x8;
			break;
		default:break;
	}
	data=sread(reg_addr);
	data =data >>6;
	data &=0x1;
	switch(data)
	{
		case 0:
			integ_type= SHIFT; //0
			break;
		case 1:
			integ_type= SPEED; // 1
			break;
		default:break;
	}
	return integ_type;
}

////////////////////Speed_Config_REG   0x000C
void set_speed_resistance(float voltage)//设置转速通道的分压电阻值-----------
{
	//LOGD("xin:===set_speed_resistance");
	unsigned int data = 0;
	unsigned int reg_addr =0;
	enable_rotation(true); //转速使能
	data = sread(0xc);
	//data = 10*1000*(1-voltage/128);
	swrite(0xc, data);
	//sread(0xc);
}

void set_speed_triger_mode(int speedTriMode)//设置转速触发方式  OK
{
	//LOGD("xin:===set_speed_triger_mode");
	unsigned int data = 0;
	data=sread(0xc);// 0x000c
	switch(speedTriMode)
	{
		case AnalogFallingCount ://下降沿触发计数器		00
			break;
		case AnalogRisingCount: //上升沿触发计算器	    01
			data |=(0x1 << 8);
			break;
		default:break;
	}
	swrite(0xc, data);
	//sread(0xc);
}

int get_speed_triger_mode()//得到转速触发方式  OK
{
	unsigned int data = 0;
	int tri_mode =0;
	data = sread(0xc);
	data = data >>8;
	data &= 0x1;
	switch(data)
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
void set_triger_ch(int ch)//设置模拟触发通道  OK
{
	//LOGD("xin:===set_triger_ch");
	unsigned int data = 0;
	unsigned int reg_addr =0x10;
	data=sread(reg_addr);
	switch(ch)
	{
		case CHA:
		    data &=(~((unsigned int)(0xf)));
			break;
		case CHB:
			data |=0x1;
			break;
		default:break;
	}
	swrite(reg_addr, data);
	//sread(reg_addr);
}

int get_triger_ch()//得到模拟触发通道  OK
{
	unsigned int data = 0;
	unsigned int reg_addr =0x10;
	int ch_num =0;
	data = sread(reg_addr);
	data &=(~((unsigned int)(0xf)));
	switch((int)data)
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

void set_triger_mode(int triMode)//设置开始采集的触发方式  OK
{
	//LOGD("xin:===set_triger_mode");
	unsigned int data = 0;
	data=sread(0x10);// 0x0010
	switch(triMode)
	{
		case  Manual: //(自由)手动触发
			break;
		case  RTriger:  //转速触发
			data |=(0x1 << 4);
			break;
		case  AnalogRising://模拟上升沿触发
			data |=(0x2 << 4);
			break;
		case  AnalogFalling: //模拟下降沿触发
			data |=(0x3 << 4);
			break;
		default:break;
	}
	swrite(0x10, data);
	//sread(0x10);
}

int get_triger_mode()//得到触发方式 OK
{
	unsigned int data = 0;
	int tm = 0;
	data=sread(0x10);
	data = data >> 4;
	data &= 0x3;
	switch((int)data)
	{
		case 0:
			tm = Manual; //(自由)手动触发
			break;
		case 1:
			tm = RTriger;//转速触发
			break;
		case 2:
			tm = AnalogRising; //模拟上升沿触发
			break;
		case 3:
			tm = AnalogFalling; //模拟下降沿触发
			break;
		 default:break;
	}
	return tm;
}

void set_triger_threshold(double threshold)//设置触发阈值
{
	//LOGD("xin:===set_triger_threshold");
	swrite(0x14,(unsigned int)threshold);
	//sread(0x14);
}

double  get_triger_threshold()//得到触发阈值
{
	return(double)sread(0x14);
}

//////////////////Status_REG   0x0044

bool get_adc_work_status()//得到adc的工作状态 OK
{
	unsigned int data=0;
	data = sread(0x44);
	data = data >>6;
	data &= 0x1;
	if(data ==0x1)
		return true;
	else
		return false;
}

bool get_rotation_sample_status()//得到转速采集状态 OK
{
	unsigned int data=0;
	data = sread(0x44);
	data = data >>7;
	data &= 0x1;
	if(data ==0x1)
		return true;
	else
		return false;
}

//////////////Analog_FIFO_Status_REG 0x0048
int get_analog_fifo_byte()//得到模拟fifo中的字节数 0K
{
	return(int)sread(0x48);
}

void poweron_spi()//SPI POWER ON
{
	//LOGD("xin:===poweron_spi设备上电开始");
	if(GpioOpen()< 0)
    {
        LOGD("xin:===poweron_spi设备上电异常");
    }else{
        GpioSet(FPGA_3V3_CTR, GPIO_SET_ON);
        GpioSet(FPGA_1V2_CTR, GPIO_SET_ON);
        GpioSet(FPGA_2V5_CTR, GPIO_SET_ON);
        usleep(130000);
        LOGD("xin:===poweron_spi设备上电结束");
    } 
}

void poweroff_spi()//SPI POWER OFF
{
	GpioSet(FPGA_3V3_CTR, GPIO_SET_OFF);
	GpioSet(FPGA_2V5_CTR, GPIO_SET_OFF);
	GpioSet(FPGA_1V2_CTR, GPIO_SET_OFF);
    usleep(10000);
	LOGD("xin:===poweroff_spi设备下电结束");
}

void reset_fpga_reg() //复位FPGA的寄存器
{
    LOGD("xin:===reset_fpga_reg ==start");
	swrite(ResetFpgaRegAddr, ResetFpgaRegData);
	usleep(50000);
    swrite(0,0);
    swrite(0x04,0);
    swrite(0x08,0);
    swrite(0x0c,0);
    swrite(0x10,0);
    swrite(0x14,0);
    swrite(0x44,0);
    swrite(0x48,0);   
    LOGD("xin:===reset_fpga_reg ==end");
}

int set_singleCH_vibrate_reg(int ch, int signalType, float maxFreq, float minFreq, int version_mode, int rangeMode) //设置振动采集寄存器，单通道振动默认CHB
{
	LOGD("xin:=== set_singleCH_vibrate_reg==========start [%s]", log_time());
	reset_fpga_reg(); //复位寄存器

    if(ch == CH_B)
	{
		if ((sread(0) == -1) || (sread(0x8) == -1))
	    {
	        return -1;
	    }

		if(signalType == ACC_TYPE) //0：加速度，1：速度，2：位移
		{
		    LOGD("xin:=== signalType == 0, ACC");
		    if (enable_CH_B(true) == -1)//使能CHA
	        {
	            return -1;
	        }

	        if(minFreq < FREQ_7) //小于7Hz以下建议用DC耦合，大于等于7HZ 用AC耦合
			{
				LOGD("xin:=== minFreq <7HZ, DC");
				set_couple_mode(CHB, DC);//耦合方式DC
			}else{
				LOGD("xin:=== minFreq >=7HZ, AC");
				//set_couple_mode(CHB, AC);//耦合方式 , 默认AC
			}

		}else if(signalType == VEL_TYPE) //速度
		{
		    LOGD("xin:=== signalType == 1, SPEED");
	        if(enable_CH_B_integrate(true)== -1)//使能CHB的积分
	        {
	            return -1;
	        }

			set_integrate(CHB, SPEED);//设置积分选择, 速度
			if(minFreq < FREQ_10) //小于10Hz以下建议用DC耦合，大于等于10HZ 用AC耦合
			{
				LOGD("xin:=== minFreq <10HZ, DC");
				set_couple_mode(CHB, DC);//耦合方式DC
			}else{
				LOGD("xin:=== minFreq >=10HZ, AC");
				//set_couple_mode(CHB, AC);//耦合方式 , 默认AC
			}
		}else if(signalType == DSP_TYPE) //位移
		{
			LOGD("xin:=== signalType == 2, SHIFT");
	        if(enable_CH_B_integrate(true) == -1) //使能CHB的积分
	        {
	            return -1;
	        }

			//set_integrate(CHB, SHIFT);//设置积分选择, 默认为位移
			if(minFreq < FREQ_10) //小于10Hz以下建议用DC耦合，大于等于10HZ 用AC耦合
			{
				LOGD("xin:=== minFreq <10HZ, DC");
				set_couple_mode(CHB, DC);//耦合方式DC
			}else{
				LOGD("xin:=== minFreq >=10HZ, AC");
				//set_couple_mode(CHB, AC);//耦合方式 , 默认AC
			}
		}

		if ((sread(0) == -1) || (sread(0x8) == -1))
	    {
	        return -1;
	    }
		set_sample_rate((int)maxFreq*2.56); //设置采样频率（包括设置ADC CLK，ADC_MODE）
	    if ((sread(0) == -1) || (sread(0x8) == -1))
	    {
	        return -1;
	    }

	    if (version_mode == TEST_MODE)  ////如果为1表示内部  测试版本
	    {
	        LOGD("xin:=== version_mode = 1 表示测试版本，无24V激励");
	         //set_24V(CHB, true);//设置24v电源的开关激励, 默认不开启，最终版本是要设为 true
	    }else{//正式版本为0，最终版本要打开,且adc数据提取要特别处理
	        LOGD("xin:=== version_mode = 0 表示正式版本，有24V激励");
	        set_24V(CHB, true);//设置24v电源的开关激励, 默认不开启，最终版本是要设为 true
	    }

	    if(rangeMode == 0)
	        set_voltage_range(CHB, V25);//设置电压量程, 默认V25, 可选有V25, V2P5, V0P25
	    else if (rangeMode == 1)
	        set_voltage_range(CHB, V2P5);
	    else if (rangeMode == 2)
	        set_voltage_range(CHB, V0P25);

	    sread(0);
		sread(0x8);
	}

		
    LOGD("xin:=== set_singleCH_vibrate_reg==========end [%s]", log_time());
	return 0;
}


char* log_time()
{
	struct  tm      *ptm;
	struct  timeb   stTimeb;
	static  char    szTime[19];
	ftime(&stTimeb);
	ptm = localtime(&stTimeb.time);
	sprintf(szTime, "%02d-%02d %02d:%02d:%02d.%03d", ptm->tm_mon+1, ptm->tm_mday, ptm->tm_hour, ptm->tm_min, ptm->tm_sec, stTimeb.millitm);
	szTime[18] = 0;
	return szTime;
}
