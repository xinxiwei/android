//////////////////////////////////////////////////////////////////////////////////////////////////
//文件名  : .c
//描述    :  定义文件头文件
//当前版本: v1.0.0.0
//履历    :
//-------------------------
// 2014-07-19                   模块创建                qiuye
// 

#include <hardware/IIRCoeffs.h>

/***********************************************采样频率102.4kHz**************************************/
/*截止频率为0.16Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass (1 order) 
Cutoff frequency     : 0.16 Hz (attenuation@-3 dB)   
Step response edge   :6.9E+5    */
float S0_F016_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM]={
		-9.999902E-1,
		0.000000E+0
};
float S0_F016_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM]={
		9.999951E-1,
		-9.999951E-1,
		0.000000E+0
};

/*截止频率为1Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 1
Cutoff frequency     : 1 Hz (attenuation@-3.00 dB)
Step response edge   : 1.1E+5  */
float S0_F1_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM]={
		-9.999386E-1,
		0.000000E+0
};
float S0_F1_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM]={
		9.999693E-1,
		-9.999693E-1,
		0.000000E+0
};

/*截止频率为2Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 1
Cutoff frequency     : 2 Hz (attenuation@-3.00 dB)
Step response edge   : 51200 */
float S0_F2_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM]={
		-9.998773E-1,
		0.000000E+0
};
float S0_F2_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM]={
		9.999386E-1,
		-9.999386E-1,
		0.000000E+0
};

/*截止频率为5Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 1
Cutoff frequency     : 5 Hz (attenuation@-3.00 dB)
Step response edge   : 20480 */
float S0_F5_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM]={
		-9.996933E-1,
		0.000000E+0
};
float S0_F5_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM]={
		9.998466E-1,
		-9.998466E-1,
		0.000000E+0
};

/*截止频率为10Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 1
Cutoff frequency     : 10 Hz (attenuation@-3.00 dB)
Step response edge   : 10240  */
float S0_F10_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM]={
		-9.993866E-1,
		0.000000E+0
};
float S0_F10_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM]={
		9.996933E-1,
		-9.996933E-1,
		0.000000E+0
};

/*截止频率为20Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 20 Hz (attenuation@-3.00 dB)
Step response edge   : 12288  */
float S0_F20_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.997734E+0,
		9.977350E-1,
		-1.999060E+0,
		9.990612E-1
};
float S0_F20_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.991986E-1,
		-1.998397E+0,
		9.991986E-1,
		9.991986E-1,
		-1.998397E+0,
		9.991986E-1
};

/*截止频率为50Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 50 Hz (attenuation@-3.00 dB)
Step response edge   : 4096  */
float S0_F50_ReverseCoeffs[S0_F50_REVERSCOEFFS_NUM]={
		-1.994338E+0,
		9.943472E-1,
		-1.997645E+0,
		9.976546E-1
};
float S0_F50_ForwardCoeffs[S0_F50_FORWARDCOEFFS_NUM]={
		9.979978E-1,
		-1.995996E+0,
		9.979978E-1,
		9.979978E-1,
		-1.995996E+0,
		9.979978E-1
};

/*截止频率为100Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 100 Hz (attenuation@-3.00 dB)
Step response edge   : 3072  */
float S0_F100_ReverseCoeffs[S0_F100_REVERSCOEFFS_NUM]={
    -1.988689E+0,
    9.887263E-1,
    -1.995277E+0,
    9.953148E-1
};
float S0_F100_ForwardCoeffs[S0_F100_FORWARDCOEFFS_NUM]={
9.959995E-1,
-1.991999E+0,
9.959995E-1,
9.959995E-1,
-1.991999E+0,
9.959995E-1
};

/***********************************************采样频率51.2kHz**************************************/
/*截止频率为0.16Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass (1 order) 
Cutoff frequency     : 0.16 Hz (attenuation@-3 dB)   
Step response edge   :348160    */
float S1_F016_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM]={
		-9.999804E-1,
		0.000000E+0
};
float S1_F016_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM]={
		9.999902E-1,
		-9.999902E-1,
		0.000000E+0
};


/*截止频率为1Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 1
Cutoff frequency     : 1 Hz (attenuation@-3.00 dB)
Step response edge   : 56320  */
float S1_F1_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM]={
		-9.998773E-1,
		0.000000E+0
};
float S1_F1_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM]={
		9.999386E-1,
		-9.999386E-1,
		0.000000E+0
};

/*截止频率为2Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 1
Cutoff frequency     : 2 Hz (attenuation@-3.00 dB)
Step response edge   : 25600 */
float S1_F2_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM]={
		-9.997546E-1,
		0.000000E+0
};
float S1_F2_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM]={
		9.998773E-1,
		-9.998773E-1,
		0.000000E+0
};

/*截止频率为5Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 1
Cutoff frequency     : 5 Hz (attenuation@-3.00 dB)
Step response edge   : 10240 */
float S1_F5_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM]={
		-9.993866E-1,
		0.000000E+0
};
float S1_F5_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM]={
		9.996933E-1,
		-9.996933E-1,
		0.000000E+0
};

/*截止频率为10Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 2
Cutoff frequency     : 10 Hz (attenuation@-3.00 dB)
Step response edge   : 7680  */
float S1_F10_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM]={
		-1.998264E+0,
		9.982660E-1
};
float S1_F10_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM]={
		9.991326E-1,
		-1.998265E+0,
		9.991326E-1
};

/*截止频率为20Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 20 Hz (attenuation@-3.00 dB)
Step response edge   : 5120  */
float S1_F20_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.995469E+0,
		9.954752E-1,
		-1.998117E+0,
		9.981233E-1
};
float S1_F20_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.983979E-1,
		-1.996796E+0,
		9.983979E-1,
		9.983979E-1,
		-1.996796E+0,
		9.983979E-1
};

/*截止频率为50Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 50 Hz (attenuation@-3.00 dB)
Step response edge   : 3072  */
float S1_F50_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.988689E+0,
		9.887263E-1,
		-1.995277E+0,
		9.953148E-1
};
float S1_F50_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.959995E-1,
		-1.991999E+0,
		9.959995E-1,
		9.959995E-1,
		-1.991999E+0,
		9.959995E-1
};

/*截止频率为100Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 100 Hz (attenuation@-3.00 dB)
Step response edge   : 1024  */
float S1_F100_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.977430E+0,
		9.775793E-1,
		-1.990502E+0,
		9.906517E-1
};
float S1_F100_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.920150E-1,
		-1.984030E+0,
		9.920150E-1,
		9.920150E-1,
		-1.984030E+0,
		9.920150E-1
};

/***********************************************采样频率25.6kHz**************************************/
/*截止频率为0.16Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass (1 order) 
Cutoff frequency     : 0.16 Hz (attenuation@-3 dB)   
Step response edge   :174080    */
float S2_F016_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM]={
		-9.999607E-1,
		0.000000E+0
};
float S2_F016_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM]={
		9.999804E-1,
		-9.999804E-1,
		0.000000E+0
};


/*截止频率为1Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 1
Cutoff frequency     : 1 Hz (attenuation@-3.00 dB)
Step response edge   : 28160  */
float S2_F1_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM]={
		-9.997546E-1,
		0.000000E+0
};
float S2_F1_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM]={
		9.998773E-1,
		-9.998773E-1,
		0.000000E+0
};

/*截止频率为2Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 1
Cutoff frequency     : 2 Hz (attenuation@-3.00 dB)
Step response edge   : 12800 */
float S2_F2_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM]={
		-9.995092E-1,
		0.000000E+0
};
float S2_F2_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM]={
		9.997546E-1,
		-9.997546E-1,
		0.000000E+0
};

/*截止频率为5Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 5 Hz (attenuation@-3.00 dB)
Step response edge   : 10240 */
float S2_F5_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.997734E+0,
		9.977350E-1,
		-1.999060E+0,
		9.990612E-1
};
float S2_F5_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.991986E-1,
		-1.998397E+0,
		9.991986E-1,
		9.991986E-1,
		-1.998397E+0,
		9.991986E-1
};

/*截止频率为10Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 10 Hz (attenuation@-3.00 dB)
Step response edge   : 5120  */
float S2_F10_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.995469E+0,
		9.954752E-1,
		-1.998117E+0,
		9.981233E-1
};
float S2_F10_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.983979E-1,
		-1.996796E+0,
		9.983979E-1,
		9.983979E-1,
		-1.996796E+0,
		9.983979E-1
};

/*截止频率为20Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 20 Hz (attenuation@-3.00 dB)
Step response edge   : 3072  */
float S2_F20_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.990947E+0,
		9.909708E-1,
		-1.996226E+0,
		9.962501E-1
};
float S2_F20_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.967983E-1,
		-1.993597E+0,
		9.967983E-1,
		9.967983E-1,
		-1.993597E+0,
		9.967983E-1
};

/*截止频率为50Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 50 Hz (attenuation@-3.00 dB)
Step response edge   : 1024  */
float S2_F50_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.977430E+0,
		9.775793E-1,
		-1.990502E+0,
		9.906517E-1
};
float S2_F50_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.920150E-1,
		-1.984030E+0,
		9.920150E-1,
		9.920150E-1,
		-1.984030E+0,
		9.920150E-1
};

/*截止频率为100Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 100 Hz (attenuation@-3.00 dB)
Step response edge   : 512  */
float S2_F100_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.955070E+0,
		9.556591E-1,
		-1.980795E+0,
		9.813917E-1
};
float S2_F100_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.840935E-1,
		-1.968187E+0,
		9.840935E-1,
		9.840935E-1,
		-1.968187E+0,
		9.840935E-1
};

/***********************************************采样频率12.8kHz**************************************/
/*截止频率为0.16Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass (1 order) 
Cutoff frequency     : 0.16 Hz (attenuation@-3 dB)   
Step response edge   :87040    */
float S3_F016_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM]={
		-9.999215E-1,
		0.000000E+0
};
float S3_F016_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM]={
		9.999607E-1,
		-9.999607E-1,
		0.000000E+0
};


/*截止频率为1Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 1
Cutoff frequency     : 1 Hz (attenuation@-3.00 dB)
Step response edge   : 14080  */
float S3_F1_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM]={
		-9.995092E-1,
		0.000000E+0
};
float S3_F1_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM]={
		9.997546E-1,
		-9.997546E-1,
		0.000000E+0
};

/*截止频率为2Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 1
Cutoff frequency     : 2 Hz (attenuation@-3.00 dB)
Step response edge   : 6400 */
float S3_F2_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM]={
		-9.990187E-1,
		0.000000E+0
};
float S3_F2_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM]={
		9.995094E-1,
		-9.995094E-1,
		0.000000E+0
};

/*截止频率为5Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 5 Hz (attenuation@-3.00 dB)
Step response edge   : 7168 */
float S3_F5_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.995469E+0,
		9.954752E-1,
		-1.998117E+0,
		9.981233E-1
};
float S3_F5_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.983979E-1,
		-1.996796E+0,
		9.983979E-1,
		9.983979E-1,
		-1.996796E+0,
		9.983979E-1
};

/*截止频率为10Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 10 Hz (attenuation@-3.00 dB)
Step response edge   : 2560  */
float S3_F10_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.990947E+0,
		9.909708E-1,
		-1.996226E+0,
		9.962501E-1
};
float S3_F10_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.967983E-1,
		-1.993597E+0,
		9.967983E-1,
		9.967983E-1,
		-1.993597E+0,
		9.967983E-1
};

/*截止频率为20Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 20 Hz (attenuation@-3.00 dB)
Step response edge   : 1792  */
float S3_F20_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.981927E+0,
		9.820230E-1,
		-1.992418E+0,
		9.925143E-1
};
float S3_F20_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.936069E-1,
		-1.987214E+0,
		9.936069E-1,
		9.936069E-1,
		-1.987214E+0,
		9.936069E-1
};

/*截止频率为50Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 50 Hz (attenuation@-3.00 dB)
Step response edge   : 512  */
float S3_F50_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.955070E+0,
		9.556591E-1,
		-1.980795E+0,
		9.813917E-1
};
float S3_F50_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.840935E-1,
		-1.968187E+0,
		9.840935E-1,
		9.840935E-1,
		-1.968187E+0,
		9.840935E-1
};

/*截止频率为100Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 100 Hz (attenuation@-3.00 dB)
Step response edge   : 256  */
float S3_F100_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.910962E+0,
		9.132666E-1,
		-1.960773E+0,
		9.631374E-1
};
float S3_F100_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.684372E-1,
		-1.936874E+0,
		9.684372E-1,
		9.684372E-1,
		-1.936874E+0,
		9.684372E-1
};

/***********************************************采样频率10.24kHz**************************************/
/*截止频率为0.16Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass (1 order) 
Cutoff frequency     : 0.16 Hz (attenuation@-3 dB)   
Step response edge   :69632    */
float S4_F016_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM]={
		-9.999018E-1,
		0.000000E+0
};
float S4_F016_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM]={
		9.999509E-1,
		-9.999509E-1,
		0.000000E+0
};


/*截止频率为1Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 1
Cutoff frequency     : 1 Hz (attenuation@-3.00 dB)
Step response edge   : 11264  */
float S4_F1_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM]={
		-9.993866E-1,
		0.000000E+0
};
float S4_F1_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM]={
		9.996933E-1,
		-9.996933E-1,
		0.000000E+0
};

/*截止频率为2Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 2 Hz (attenuation@-3.00 dB)
Step response edge   : 10240 */
float S4_F2_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.997734E+0,
		9.977350E-1,
		-1.999060E+0,
		9.990612E-1
};
float S4_F2_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.991986E-1,
		-1.998397E+0,
		9.991986E-1,
		9.991986E-1,
		-1.998397E+0,
		9.991986E-1
};

/*截止频率为5Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 5 Hz (attenuation@-3.00 dB)
Step response edge   : 5120 */
float S4_F5_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.994338E+0,
		9.943472E-1,
		-1.997645E+0,
		9.976546E-1
};
float S4_F5_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.979978E-1,
		-1.995996E+0,
		9.979978E-1,
		9.979978E-1,
		-1.995996E+0,
		9.979978E-1
};

/*截止频率为10Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 10 Hz (attenuation@-3.00 dB)
Step response edge   : 3072  */
float S4_F10_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.988689E+0,
		9.887263E-1,
		-1.995277E+0,
		9.953148E-1
};
float S4_F10_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.959995E-1,
		-1.991999E+0,
		9.959995E-1,
		9.959995E-1,
		-1.991999E+0,
		9.959995E-1
};

/*截止频率为20Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 20 Hz (attenuation@-3.00 dB)
Step response edge   : 1228  */
float S4_F20_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.977430E+0,
		9.775793E-1,
		-1.990502E+0,
		9.906517E-1
};
float S4_F20_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.920150E-1,
		-1.984030E+0,
		9.920150E-1,
		9.920150E-1,
		-1.984030E+0,
		9.920150E-1
};

/*截止频率为50Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 50 Hz (attenuation@-3.00 dB)
Step response edge   : 307  */
float S4_F50_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.943967E+0,
		9.448824E-1,
		-1.975865E+0,
		9.767949E-1
};
float S4_F50_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.801562E-1,
		-1.960312E+0,
		9.801562E-1,
		9.801562E-1,
		-1.960312E+0,
		9.801562E-1
};

/*截止频率为100Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 100 Hz (attenuation@-3.00 dB)
Step response edge   : 307  */
float S4_F100_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.889207E+0,
		8.927690E-1,
		-1.950466E+0,
		9.541432E-1
};
float S4_F100_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.607008E-1,
		-1.921402E+0,
		9.607008E-1,
		9.607008E-1,
		-1.921402E+0,
		9.607008E-1
};

/***********************************************采样频率6.4kHz**************************************/
/*截止频率为0.16Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass (1 order) 
Cutoff frequency     : 0.16 Hz (attenuation@-3 dB)   
Step response edge   : 43520    */
float S5_F016_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM]={
		-9.998429E-1,
		0.000000E+0
};
float S5_F016_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM]={
		9.999215E-1,
		-9.999215E-1,
		0.000000E+0
};


/*截止频率为1Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 1
Cutoff frequency     : 1 Hz (attenuation@-3.00 dB)
Step response edge   : 7040  */
float S5_F1_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM]={
		-9.990187E-1,
		0.000000E+0
};
float S5_F1_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM]={
		9.995094E-1,
		-9.995094E-1,
		0.000000E+0
};

/*截止频率为2Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 2 Hz (attenuation@-3.00 dB)
Step response edge   : 6400 */
float S5_F2_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.996375E+0,
		9.963785E-1,
		-1.998494E+0,
		9.984983E-1
};
float S5_F2_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.987181E-1,
		-1.997436E+0,
		9.987181E-1,
		9.987181E-1,
		-1.997436E+0,
		9.987181E-1
};

/*截止频率为5Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 5 Hz (attenuation@-3.00 dB)
Step response edge   : 3200 */
float S5_F5_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.990947E+0,
		9.909708E-1,
		-1.996226E+0,
		9.962501E-1
};
float S5_F5_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.967983E-1,
		-1.993597E+0,
		9.967983E-1,
		9.967983E-1,
		-1.993597E+0,
		9.967983E-1
};

/*截止频率为10Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 10 Hz (attenuation@-3.00 dB)
Step response edge   : 1280  */
float S5_F10_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.981927E+0,
		9.820230E-1,
		-1.992418E+0,
		9.925143E-1
};
float S5_F10_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.936069E-1,
		-1.987214E+0,
		9.936069E-1,
		9.936069E-1,
		-1.987214E+0,
		9.936069E-1
};

/*截止频率为20Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 20 Hz (attenuation@-3.00 dB)
Step response edge   : 768  */
float S5_F20_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.963989E+0,
		9.643680E-1,
		-1.984702E+0,
		9.850851E-1
};
float S5_F20_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.872545E-1,
		-1.974509E+0,
		9.872545E-1,
		9.872545E-1,
		-1.974509E+0,
		9.872545E-1
};

/*截止频率为50Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 50 Hz (attenuation@-3.00 dB)
Step response edge   : 256  */
float S5_F50_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.910962E+0,
		9.132666E-1,
		-1.960773E+0,
		9.631374E-1
};
float S5_F50_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.684372E-1,
		-1.936874E+0,
		9.684372E-1,
		9.684372E-1,
		-1.936874E+0,
		9.684372E-1
};

/*截止频率为100Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 100 Hz (attenuation@-3.00 dB)
Step response edge   : 128  */
float S5_F100_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.825096E+0,
		8.339269E-1,
		-1.918411E+0,
		9.276931E-1
};
float S5_F100_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.378493E-1,
		-1.875699E+0,
		9.378493E-1,
		9.378493E-1,
		-1.875699E+0,
		9.378493E-1
};

/***********************************************采样频率5.12kHz**************************************/
/*截止频率为0.16Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass (1 order) 
Cutoff frequency     : 0.16 Hz (attenuation@-3 dB)   
Step response edge   : 34816    */
float S6_F016_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM]={
		-9.998037E-1,
		0.000000E+0
};
float S6_F016_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM]={
		9.999018E-1,
		-9.999018E-1,
		0.000000E+0
};


/*截止频率为1Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 1
Cutoff frequency     : 1 Hz (attenuation@-3.00 dB)
Step response edge   : 5632  */
float S6_F1_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM]={
		-9.987736E-1,
		0.000000E+0
};
float S6_F1_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM]={
		9.993868E-1,
		-9.993868E-1,
		0.000000E+0
};

/*截止频率为2Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 2 Hz (attenuation@-3.00 dB)
Step response edge   : 5120 */
float S6_F2_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.995469E+0,
		9.954752E-1,
		-1.998117E+0,
		9.981233E-1
};
float S6_F2_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.983979E-1,
		-1.996796E+0,
		9.983979E-1,
		9.983979E-1,
		-1.996796E+0,
		9.983979E-1
};

/*截止频率为5Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 5 Hz (attenuation@-3.00 dB)
Step response edge   : 2560 */
float S6_F5_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.988689E+0,
		9.887263E-1,
		-1.995277E+0,
		9.953148E-1
};
float S6_F5_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.959995E-1,
		-1.991999E+0,
		9.959995E-1,
		9.959995E-1,
		-1.991999E+0,
		9.959995E-1
};

/*截止频率为10Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 10 Hz (attenuation@-3.00 dB)
Step response edge   : 1024  */
float S6_F10_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.977430E+0,
		9.775793E-1,
		-1.990502E+0,
		9.906517E-1
};
float S6_F10_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.920150E-1,
		-1.984030E+0,
		9.920150E-1,
		9.920150E-1,
		-1.984030E+0,
		9.920150E-1
};

/*截止频率为20Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 20 Hz (attenuation@-3.00 dB)
Step response edge   : 614  */
float S6_F20_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.955070E+0,
		9.556591E-1,
		-1.980795E+0,
		9.813917E-1
};
float S6_F20_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.840935E-1,
		-1.968187E+0,
		9.840935E-1,
		9.840935E-1,
		-1.968187E+0,
		9.840935E-1
};

/*截止频率为50Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 50 Hz (attenuation@-3.00 dB)
Step response edge   : 307  */
float S6_F50_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.889207E+0,
		8.927690E-1,
		-1.950466E+0,
		9.541432E-1
};
float S6_F50_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.607008E-1,
		-1.921402E+0,
		9.607008E-1,
		9.607008E-1,
		-1.921402E+0,
		9.607008E-1
};

/*截止频率为100Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 100 Hz (attenuation@-3.00 dB)
Step response edge   : 102  */
float S6_F100_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.783283E+0,
		7.967955E-1,
		-1.896135E+0,
		9.105034E-1
};
float S6_F100_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.229052E-1,
		-1.845810E+0,
		9.229052E-1,
		9.229052E-1,
		-1.845810E+0,
		9.229052E-1
};

/***********************************************采样频率2.56kHz**************************************/
/*截止频率为0.16Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass (1 order) 
Cutoff frequency     : 0.16 Hz (attenuation@-3 dB)   
Step response edge   : 17408    */
float S7_F016_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM]={
		-9.996074E-1,
		0.000000E+0
};
float S7_F016_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM]={
		9.998037E-1,
		-9.998037E-1,
		0.000000E+0
};


/*截止频率为1Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 1
Cutoff frequency     : 1 Hz (attenuation@-3.00 dB)
Step response edge   : 2816  */
float S7_F1_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM]={
		-9.975486E-1,
		0.000000E+0
};
float S7_F1_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM]={
		9.987743E-1,
		-9.987743E-1,
		0.000000E+0
};

/*截止频率为2Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 2 Hz (attenuation@-3.00 dB)
Step response edge   : 2560 */
float S7_F2_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.990947E+0,
		9.909708E-1,
		-1.996226E+0,
		9.962501E-1
};
float S7_F2_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.967983E-1,
		-1.993597E+0,
		9.967983E-1,
		9.967983E-1,
		-1.993597E+0,
		9.967983E-1
};

/*截止频率为5Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 5 Hz (attenuation@-3.00 dB)
Step response edge   : 1280 */
float S7_F5_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.977430E+0,
		9.775793E-1,
		-1.990502E+0,
		9.906517E-1
};
float S7_F5_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.920150E-1,
		-1.984030E+0,
		9.920150E-1,
		9.920150E-1,
		-1.984030E+0,
		9.920150E-1
};

/*截止频率为10Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 10 Hz (attenuation@-3.00 dB)
Step response edge   : 640  */
float S7_F10_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.955070E+0,
		9.556591E-1,
		-1.980795E+0,
		9.813917E-1
};
float S7_F10_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.840935E-1,
		-1.968187E+0,
		9.840935E-1,
		9.840935E-1,
		-1.968187E+0,
		9.840935E-1
};

/*截止频率为20Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 20 Hz (attenuation@-3.00 dB)
Step response edge   : 307  */
float S7_F20_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.910962E+0,
		9.132666E-1,
		-1.960773E+0,
		9.631374E-1
};
float S7_F20_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.684372E-1,
		-1.936874E+0,
		9.684372E-1,
		9.684372E-1,
		-1.936874E+0,
		9.684372E-1
};

/*截止频率为50Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 50 Hz (attenuation@-3.00 dB)
Step response edge   : 128  */
float S7_F50_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.783283E+0,
		7.967955E-1,
		-1.896135E+0,
		9.105034E-1
};
float S7_F50_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.229052E-1,
		-1.845810E+0,
		9.229052E-1,
		9.229052E-1,
		-1.845810E+0,
		9.229052E-1
};

/*截止频率为100Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 100 Hz (attenuation@-3.00 dB)
Step response edge   : 77  */
float S7_F100_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.584391E+0,
		6.333405E-1,
		-1.775014E+0,
		8.298521E-1
};
float S7_F100_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		8.514507E-1,
		-1.702901E+0,
		8.514507E-1,
		8.514507E-1,
		-1.702901E+0,
		8.514507E-1
};

/***********************************************采样频率1.28kHz**************************************/
/*截止频率为0.16Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass (1 order) 
Cutoff frequency     : 0.16 Hz (attenuation@-3 dB)   
Step response edge   : 8704    */
float S8_F016_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM]={
		-9.992149E-1,
		0.000000E+0
};
float S8_F016_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM]={
		9.996075E-1,
		-9.996075E-1,
		0.000000E+0
};


/*截止频率为1Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 1
Cutoff frequency     : 1 Hz (attenuation@-3.00 dB)
Step response edge   : 1408  */
float S8_F1_ReverseCoeffs[SX_FX_REVERSCOEFFS_COMMON_NUM]={
		-9.951033E-1,
		0.000000E+0
};
float S8_F1_ForwardCoeffs[SX_FX_FORWARDCOEFFS_COMMON_NUM]={
		9.975516E-1,
		-9.975516E-1,
		0.000000E+0
};

/*截止频率为2Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 2 Hz (attenuation@-3.00 dB)
Step response edge   : 1536 */
float S8_F2_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.981927E+0,
		9.820230E-1,
		-1.992418E+0,
		9.925143E-1
};
float S8_F2_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.936069E-1,
		-1.987214E+0,
		9.936069E-1,
		9.936069E-1,
		-1.987214E+0,
		9.936069E-1
};

/*截止频率为5Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 5 Hz (attenuation@-3.00 dB)
Step response edge   : 768 */
float S8_F5_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.955070E+0,
		9.556591E-1,
		-1.980795E+0,
		9.813917E-1
};
float S8_F5_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.840935E-1,
		-1.968187E+0,
		9.840935E-1,
		9.840935E-1,
		-1.968187E+0,
		9.840935E-1
};

/*截止频率为10Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 10 Hz (attenuation@-3.00 dB)
Step response edge   : 358  */
float S8_F10_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.910962E+0,
		9.132666E-1,
		-1.960773E+0,
		9.631374E-1
};
float S8_F10_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.684372E-1,
		-1.936874E+0,
		9.684372E-1,
		9.684372E-1,
		-1.936874E+0,
		9.684372E-1
};

/*截止频率为20Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 20 Hz (attenuation@-3.00 dB)
Step response edge   : 179  */
float S8_F20_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.825096E+0,
		8.339269E-1,
		-1.918411E+0,
		9.276931E-1
};
float S8_F20_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		9.378493E-1,
		-1.875699E+0,
		9.378493E-1,
		9.378493E-1,
		-1.875699E+0,
		9.378493E-1
};

/*截止频率为50Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 50 Hz (attenuation@-3.00 dB)
Step response edge   : 77  */
float S8_F50_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.584391E+0,
		6.333405E-1,
		-1.775014E+0,
		8.298521E-1
};
float S8_F50_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
		8.514507E-1,
		-1.702901E+0,
		8.514507E-1,
		8.514507E-1,
		-1.702901E+0,
		8.514507E-1
};

/*截止频率为100Hz时，IIR滤波器的系数
Filter type          : Butterworth Highpass
Filter order         : 4
Cutoff frequency     : 100 Hz (attenuation@-3.00 dB)
Step response edge   : 38  */
float S8_F100_ReverseCoeffs[S0_F20_REVERSCOEFFS_NUM]={
		-1.228719E+0,
		3.932294E-1,
		-1.494281E+0,
		6.943470E-1
};
float S8_F100_ForwardCoeffs[S0_F20_FORWARDCOEFFS_NUM]={
	7.228596E-1,
	-1.445719E+0,
	7.228596E-1,
	7.228596E-1,
	-1.445719E+0,
	7.228596E-1
};