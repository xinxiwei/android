/**************************************************************************
File Name               : siglib_macros.h   | Author    : JOHN EDWARDS
Siglib Library Version  : 8.60              |
----------------------------------------+----------------------------------
Compiler  : Independent                 | Start Date    : 22/01/00
Options   :                             | Latest Update : 01/01/15
---------------------------------------------------------------------------
Support for SigLib is available via EMail : support@numerix-dsp.com

This file may be modified, in any way, providing that this header remains
within the file and none of the header contents are removed or modified.

THIS FILE MAY ONLY BE USED IN ACCORDANCE WITH THE TERMS OF THE NUMERIX LTD.
END USER LICENSE AGREEMENT.
DELETION OF ANY INFORMATION IN THIS HEADER IS IN VIOLATION OF YOUR LICENSE.

Copyright (C) 1999 to 2017 Sigma Numerix Ltd.
---------------------------------------------------------------------------
Description : Macro definitions header file for SigLib DSP library

Update history :
        See history.txt for more details.

****************************************************************************/


#if SIGLIB

#ifndef     _SIGLIB_MACROS_H
#define     _SIGLIB_MACROS_H

                        /* Macros to handle standard C rounds down
                            These macros also allow for floating point not quantizing to perfect integer values */
                        /* Macros that return type SLFixData_t */
#define SDS_RoundDown(a)        ((SLFixData_t)(((SLData_t)a)+SIGLIB_EPSILON))                   /* Round down fixed point number */
#define SDS_RoundUp(a)          ((SLFixData_t)(((SLData_t)a)+SIGLIB_ONE-SIGLIB_EPSILON))        /* Round up fixed point number */
#define SDS_RoundToNearest(a)   ((SLFixData_t)(((SLData_t)a)+SIGLIB_HALF-SIGLIB_EPSILON))       /* Round to nearest fixed point number */
                        /* Macros that return type SLArrayIndex_t */
#define SAI_RoundDown(a)        ((SLArrayIndex_t)(((SLData_t)a)+SIGLIB_EPSILON))                /* Round down fixed point number */
#define SAI_RoundUp(a)          ((SLArrayIndex_t)(((SLData_t)a)+SIGLIB_ONE-SIGLIB_EPSILON))     /* Round up fixed point number */
#define SAI_RoundToNearest(a)   ((SLArrayIndex_t)(((SLData_t)a)+SIGLIB_HALF-SIGLIB_EPSILON))    /* Round to nearest fixed point number */

                        /* Macros that output type SLFixData_t */
#define SDS_Odd(a)              (SDS_RoundToNearest((SLData_t)a) & ((SLArrayIndex_t)1))                 /* Returns 1 if a is odd, 0 otherwise */
#define SDS_Even(a)             (((SLArrayIndex_t)1)-(SDS_RoundToNearest((SLData_t)a) & ((SLArrayIndex_t)1)))               /* Returns 1 if a is even, 0 otherwise */
#define SDS_PowerOfTwo(a)       ((SDS_RoundToNearest(a) & (SDS_RoundToNearest(a) - ((SLArrayIndex_t)1))) ? ((SLArrayIndex_t)0) : ((SLArrayIndex_t)1))   /* Returns 1 if a is a power of two, 0 otherwise */
#define SDS_Absolute(a)         ((((SLData_t)a) > SIGLIB_ZERO) ? (a) : (-(a)))                  /* Returns the absolute value of a */
#define SDS_Sign(a)             ((((SLData_t)a) >= SIGLIB_ZERO) ? ((SLData_t)SIGLIB_POSITIVE) : ((SLData_t)SIGLIB_NEGATIVE))    /* Returns the sign of a */

                        /* Macros that output type SLArrayIndex_t */
#define SAI_Odd(a)              (SAI_RoundToNearest((SLData_t)a) & ((SLArrayIndex_t)1))         /* Returns 1 if a is odd, 0 otherwise */
#define SAI_Even(a)             (1-SAI_RoundToNearest((SLData_t)a)) & ((SLArrayIndex_t)1))      /* Returns 1 if a is even, 0 otherwise */
#define SAI_PowerOfTwo(a)       ((((SLArrayIndex_t)a) & (((SLArrayIndex_t)a) - 1)) ? ((SLArrayIndex_t)0) : ((SLArrayIndex_t)1)) /* Returns 1 if a is a power of two, 0 otherwise */
#define SAI_Absolute(a)         (((a) > SIGLIB_ZERO) ? (SAI_RoundToNearest((SLData_t)a)) : (-(SAI_RoundToNearest((SLData_t)a))))    /* Returns the absolute value of a */
#define SAI_Sign(a)             (((a) >= SIGLIB_ZERO) ? SIGLIB_POSITIVE : SIGLIB_NEGATIVE)      /* Returns the sign of a */
#define SAI_Log2(a)             (SAI_RoundToNearest(SDS_Log10((SLData_t)a) * SIGLIB_INV_LOG10_OF_2))    /* Returns the nearest integer log2(n) */
#define SAI_NumberOfElements(a) ((SLArrayIndex_t)(sizeof((a)) / sizeof((a)[0])))                /* Returns the number of elements in the array */


#define SDS_BitTest(a,Mask)     ((((a) & (Mask))==(Mask)) ? ((SLArrayIndex_t)1) : ((SLArrayIndex_t)0))  /* Returns 1 if all bits in mask set, 0 otherwise */
#define SDS_BitMask(a)          (~(-1 << a))                        /* Sets 'a' LSBs to 1 and the remainder to 0 */

                        /* Macros that output type SLData_t */
#define SDA_Average(a,b)        SDA_Mean(a,b)                       /* Another name for the same function */

#define SDA_Add(a,b,c,d)        SDA_Offset(a,b,c,d)                 /* Another name for the same function */

#define SDA_Subtract(a,b,c,d)   SDA_Offset(a,b,-c,d)                /* Another name for the same function */

#define SDS_Square(a)           ((SLData_t)((a) * (a)))
#define SDS_Asinh(a)            ((SLData_t)SDS_Log ((a) + SDS_Sqrt ((a) * (a) + SIGLIB_ONE)))

#define SDS_Swap(a,b)           {register SLData_t SIGLIB_Tmp = (SLData_t)a; a = b; b = SIGLIB_Tmp; }
#define SDS_Swap16(a,b)         {register SLInt16_t SIGLIB_Tmp = (SLInt16_t)a; a = b; b = SIGLIB_Tmp; }
#define SDS_Swap32(a,b)         {register SLInt32_t SIGLIB_Tmp = (SLInt32_t)a; a = b; b = SIGLIB_Tmp; }

#define SDS_Swap2(a,b)          {a ^= b; b ^= a; a ^= b;}

#define SDS_Sort2(a,b)          if ((a) > (b)) SDS_Swap(a,b)

#define SDS_Sort3(a,b,c)        SDS_Sort2(b,c); SDS_Sort2(a,c); SDS_Sort2(a,b)

#define SDS_Sort4(a,b,c,d)      SDS_Sort2(a,b); SDS_Sort2(c,d); SDS_Sort2(a,c); SDS_Sort2(b,d)

#define SDS_Sort5(a,b,c,d,e)    SDS_Sort2(a,b); SDS_Sort2(c,d); SDS_Sort2(a,c); \
                                SDS_Sort2(a,e); SDS_Sort2(d,e); SDS_Sort2(b,e)

#define SDS_Sort6(a,b,c,d,e,f)  SDS_Sort2(a,d); SDS_Sort2(b,e); SDS_Sort2(c,f); \
                                SDS_Sort2(a,b); SDS_Sort2(a,c); \
                                SDS_Sort2(e,f); SDS_Sort2(d,f)

#define SDS_SwapImageData(a,b)          {register SLImageData_t SIGLIB_Tmp = (SLImageData_t)a; a = b; b = SIGLIB_Tmp; }

#define SDS_Sort2ImageData(a,b)         if ((a) > (b)) SDS_SwapImageData(a,b)

#define SDS_Sort3ImageData(a,b,c)       SDS_Sort2ImageData(b,c); SDS_Sort2ImageData(a,c); SDS_Sort2ImageData(a,b)

#define SDS_Sort4ImageData(a,b,c,d)     SDS_Sort2ImageData(a,b); SDS_Sort2ImageData(c,d); SDS_Sort2ImageData(a,c); SDS_Sort2ImageData(b,d)

#define SDS_Sort5ImageData(a,b,c,d,e)   SDS_Sort2ImageData(a,b); SDS_Sort2ImageData(c,d); SDS_Sort2ImageData(a,c); \
                                        SDS_Sort2ImageData(a,e); SDS_Sort2ImageData(d,e); SDS_Sort2ImageData(b,e)

#define SDS_Sort6ImageData(a,b,c,d,e,f) SDS_Sort2ImageData(a,d); SDS_Sort2ImageData(b,e); SDS_Sort2ImageData(c,f); \
                                        SDS_Sort2ImageData(a,b); SDS_Sort2ImageData(a,c); \
                                        SDS_Sort2ImageData(e,f); SDS_Sort2ImageData(d,f)

/* Signal generation */

#define SDA_SignalGenerateRamp(Address, Peak, Offset, PhasePtr, ArrayLength) { \
        SLData_t    sl_Dummy; \
        SDA_SignalGenerate (Address, SIGLIB_TRIANGLE_WAVE, Peak, SIGLIB_FILL, \
                            (SIGLIB_HALF / ((SLData_t)ArrayLength)), Offset, \
                            SIGLIB_ZERO, SIGLIB_ZERO, PhasePtr, &sl_Dummy, ArrayLength); \
        }

#define SDA_SignalGenerateImpulse(Address, Peak, ArrayLength) { \
        SLData_t    sl_Dummy; \
        SDA_SignalGenerate (Address, IMPULSE, Peak, SIGLIB_FILL, SIGLIB_ZERO, SIGLIB_ZERO, \
                            SIGLIB_ZERO, SIGLIB_ZERO, &sl_Dummy, &sl_Dummy, ArrayLength); \
        }

#define SDA_SignalGenerateKronekerDeltaFunction(Address, Peak, Delay, ArrayLength) { \
        SLData_t    sl_Dummy; \
        SDA_SignalGenerate (Address, IMPULSE, Peak, SIGLIB_FILL, SIGLIB_ZERO, SIGLIB_ZERO, \
                            Delay, SIGLIB_ZERO, &sl_Dummy, &sl_Dummy, ArrayLength); \
        }

#define SDA_SignalGenerateWhiteNoise(Address, Peak, Fill_Add, ArrayLength) { \
        SLData_t    sl_Dummy; \
        SDA_SignalGenerate (Address, SIGLIB_WHITE_NOISE, Peak, Fill_Add, SIGLIB_ZERO, SIGLIB_ZERO, \
                            SIGLIB_ZERO, SIGLIB_ZERO, &sl_Dummy, &sl_Dummy, ArrayLength); \
        }

#define SDS_SignalGenerateWhiteNoise(Address, Peak, Fill_Add) { \
        SLData_t    sl_Dummy; \
        SDA_SignalGenerate (Address, SIGLIB_WHITE_NOISE, Peak, Fill_Add, SIGLIB_ZERO, SIGLIB_ZERO, \
                            SIGLIB_ZERO, SIGLIB_ZERO, &sl_Dummy, &sl_Dummy); \
        }

#define SDA_SignalGenerateGaussianNoise(Address, Fill_Add, Variance, pPhase, pValue, ArrayLength) { \
        SDA_SignalGenerate (Address, SIGLIB_GAUSSIAN_NOISE, SIGLIB_ZERO, Fill_Add, SIGLIB_ZERO, Variance, \
                            SIGLIB_ZERO, pPhase, pValue, ArrayLength); \
        }

#define SDS_SignalGenerateGaussianNoise(Address, Fill_Add, Variance, pPhase, pValue) { \
        SDA_SignalGenerate (Address, SIGLIB_GAUSSIAN_NOISE, SIGLIB_ZERO, Fill_Add, SIGLIB_ZERO, Variance, \
                            SIGLIB_ZERO, pPhase, pValue); \
        }


/* Complex numbers */

#define SCV_Real(r)         (r).real
#define SCV_Imaginary(i)    (i).imag

#define SCV_CopyMacro(IVect, OVect) {OVect.real = IVect.real; OVect.imag = IVect.imag;}


/* Buffer / matrix manipulation */

#define SDA_Operate(IPtr1, IPtr2, OPtr, Operation, ArrayLength) { \
        long    SIGLIB_i; \
        for(SIGLIB_i = 0; SIGLIB_i < ArrayLength; SIGLIB_i++) \
            *OPtr++ = (*IPtr1++) Operation (*IPtr2++); \
        IPtr1 -= ArrayLength; \
        IPtr2 -= ArrayLength; \
        OPtr -= ArrayLength; \
        }

                                                            /* Memory allocation functions */
#define SUF_VectorArrayAllocate(a)              ((SLData_t *)SUF_MemoryAllocate(((size_t)a) * sizeof (SLData_t)))               /* Vector array */
#define SUF_FftCoefficientAllocate(a)           ((SLData_t *)SUF_MemoryAllocate(((3 * ((size_t)a)) / 4) * sizeof (SLData_t)))   /* FFT twiddle factor coefficient array */
#define SUF_FirExtendedArrayAllocate(a)         ((SLData_t *)SUF_MemoryAllocate((2 * ((size_t)a)) * sizeof (SLData_t)))         /* FIR extended state array filter state array */
#define SUF_IirStateArrayAllocate(a)            ((SLData_t *)SUF_MemoryAllocate((2 * ((size_t)a)) * sizeof (SLData_t)))         /* IIR filter state array */
#define SUF_IirCoefficientAllocate(a)           ((SLData_t *)SUF_MemoryAllocate((((size_t)SIGLIB_IIR_COEFFS_PER_BIQUAD) * ((size_t)a)) * sizeof (SLData_t)))    /* IIR filter coefficients array */
#define SUF_AmCarrierArrayAllocate(a,b)         ((SLData_t *)SUF_MemoryAllocate(((size_t)(b/a)) * sizeof (SLData_t)))           /* AM carrier array */
#define SUF_FastSinCosArrayAllocate(a)          ((SLData_t *)SUF_MemoryAllocate(((5 * ((size_t)a)) / 4) * sizeof (SLData_t)))   /* QAM carrier array */
#define SUF_QuickSinCosArrayAllocate(a)         ((SLData_t *)SUF_MemoryAllocate(((5 * ((size_t)a)) / 4) * sizeof (SLData_t)))   /* QAM carrier array */
#define SUF_CostasLoopVCOArrayAllocate(a)       ((SLData_t *)SUF_MemoryAllocate(((5 * ((size_t)a)) / 4) * sizeof (SLData_t)))   /* QAM carrier array */
#define SUF_ComplexFMArrayAllocate(a)           ((SLData_t *)SUF_MemoryAllocate(((5 * ((size_t)a)) / 4) * sizeof (SLData_t)))   /* QAM carrier array */
#define SUF_QamCarrierArrayAllocate(a)          ((SLData_t *)SUF_MemoryAllocate(((5 * ((size_t)a)) / 4) * sizeof (SLData_t)))   /* QAM carrier array */
#define SUF_QPSKCarrierArrayAllocate(a)         ((SLData_t *)SUF_MemoryAllocate(((5 * ((size_t)a)) / 4) * sizeof (SLData_t)))   /* QPSK carrier array */
#define SUF_OrderAnalysisArrayAllocate(a)       ((SLData_t *)SUF_MemoryAllocate((2 * ((size_t)a)) * sizeof (SLData_t)))         /* IIR filter state array */
#define SUF_ComplexRectArrayAllocate(a)         ((SLComplexRect_s *)SUF_MemoryAllocate(((size_t)a) * sizeof (SLComplexRect_s)));    /* Rectangular complex variable array */
#define SUF_ComplexPolarArrayAllocate(a)        ((SLComplexPolar_s *)SUF_MemoryAllocate(((size_t)a) * sizeof (SLComplexPolar_s)));  /* Polar complex variable array */

#define SUF_VectorArrayAllocate16(a)            ((SLInt16_t *)SUF_MemoryAllocate(((size_t)a) * sizeof (SLInt16_t)))             /* Vector array */
#define SUF_FftCoefficientAllocate16(a)         ((SLInt16_t *)SUF_MemoryAllocate(((3 * ((size_t)a)) / 4) * sizeof (SLInt16_t))) /* FFT twiddle factor coefficient array */
#define SUF_FirExtendedArrayAllocate16(a)       ((SLInt16_t *)SUF_MemoryAllocate((2 * ((size_t)a)) * sizeof (SLInt16_t)))       /* FIR extended state array filter state array */
#define SUF_IirStateArrayAllocate16(a)          ((SLInt16_t *)SUF_MemoryAllocate((2 * ((size_t)a)) * sizeof (SLInt16_t)))       /* IIR filter state array */
#define SUF_IirCoefficientAllocate16(a)         ((SLInt16_t *)SUF_MemoryAllocate((((size_t)SIGLIB_IIR_COEFFS_PER_BIQUAD) * ((size_t)a)) * sizeof (SLInt16_t)))  /* IIR filter coefficients array */
#define SUF_AmCarrierArrayAllocate16(a,b)       ((SLInt16_t *)SUF_MemoryAllocate(((size_t)(b/a)) * sizeof (SLInt16_t)))         /* AM carrier array */
#define SUF_FastSinCosArrayAllocate16(a)        ((SLInt16_t *)SUF_MemoryAllocate(((5 * ((size_t)a)) / 4) * sizeof (SLInt16_t))) /* QAM carrier array */
#define SUF_CostasLoopVCOArrayAllocate16(a)     ((SLInt16_t *)SUF_MemoryAllocate(((5 * ((size_t)a)) / 4) * sizeof (SLInt16_t))) /* QAM carrier array */
#define SUF_ComplexFMArrayAllocate16(a)         ((SLInt16_t *)SUF_MemoryAllocate(((5 * ((size_t)a)) / 4) * sizeof (SLInt16_t))) /* QAM carrier array */
#define SUF_QuickSinCosArrayAllocate16(a)       ((SLInt16_t *)SUF_MemoryAllocate(((5 * ((size_t)a)) / 4) * sizeof (SLInt16_t))) /* QAM carrier array */
#define SUF_QamCarrierArrayAllocate16(a)        ((SLInt16_t *)SUF_MemoryAllocate(((5 * ((size_t)a)) / 4) * sizeof (SLInt16_t))) /* QAM carrier array */
#define SUF_QPSKCarrierArrayAllocate16(a)       ((SLInt16_t *)SUF_MemoryAllocate(((5 * ((size_t)a)) / 4) * sizeof (SLInt16_t))) /* QPSK carrier array */
#define SUF_OrderAnalysisArrayAllocate16(a)     ((SLInt16_t *)SUF_MemoryAllocate((2 * ((size_t)a)) * sizeof (SLInt16_t)))       /* IIR filter state array */
#define SUF_ComplexRectArrayAllocate16(a)       ((SLComplexRect16_s *)SUF_MemoryAllocate(((size_t)a) * sizeof (SLComplexRect16_s)));    /* Rectangular complex variable array */
#define SUF_ComplexPolarArrayAllocate16(a)      ((SLComplexPolar16_s *)SUF_MemoryAllocate(((size_t)a) * sizeof (SLComplexPolar16_s)));  /* Polar complex variable array */

#define SUF_VectorArrayAllocate32(a)            ((SLInt32_t *)SUF_MemoryAllocate(((size_t)a) * sizeof (SLInt32_t)))             /* Vector array */
#define SUF_FftCoefficientAllocate32(a)         ((SLInt32_t *)SUF_MemoryAllocate(((3 * ((size_t)a)) / 4) * sizeof (SLInt32_t))) /* FFT twiddle factor coefficient array */
#define SUF_FirExtendedArrayAllocate32(a)       ((SLInt32_t *)SUF_MemoryAllocate((2 * ((size_t)a)) * sizeof (SLInt32_t)))       /* FIR extended state array filter state array */
#define SUF_IirStateArrayAllocate32(a)          ((SLInt32_t *)SUF_MemoryAllocate((2 * ((size_t)a)) * sizeof (SLInt32_t)))       /* IIR filter state array */
#define SUF_IirCoefficientAllocate32(a)         ((SLInt32_t *)SUF_MemoryAllocate((((size_t)SIGLIB_IIR_COEFFS_PER_BIQUAD) * ((size_t)a)) * sizeof (SLInt32_t)))  /* IIR filter coefficients array */
#define SUF_AmCarrierArrayAllocate32(a,b)       ((SLInt32_t *)SUF_MemoryAllocate(((size_t)(b/a)) * sizeof (SLInt32_t)))         /* AM carrier array */
#define SUF_FastSinCosArrayAllocate32(a)        ((SLInt32_t *)SUF_MemoryAllocate(((5 * ((size_t)a)) / 4) * sizeof (SLInt32_t))) /* QAM carrier array */
#define SUF_CostasLoopVCOArrayAllocate32(a)     ((SLInt32_t *)SUF_MemoryAllocate(((5 * ((size_t)a)) / 4) * sizeof (SLInt32_t))) /* QAM carrier array */
#define SUF_ComplexFMArrayAllocate32(a)         ((SLInt32_t *)SUF_MemoryAllocate(((5 * ((size_t)a)) / 4) * sizeof (SLInt32_t))) /* QAM carrier array */
#define SUF_QuickSinCosArrayAllocate32(a)       ((SLInt32_t *)SUF_MemoryAllocate(((5 * ((size_t)a)) / 4) * sizeof (SLInt32_t))) /* QAM carrier array */
#define SUF_QamCarrierArrayAllocate32(a)        ((SLInt32_t *)SUF_MemoryAllocate(((5 * ((size_t)a)) / 4) * sizeof (SLInt32_t))) /* QAM carrier array */
#define SUF_QPSKCarrierArrayAllocate32(a)       ((SLInt32_t *)SUF_MemoryAllocate(((5 * ((size_t)a)) / 4) * sizeof (SLInt32_t))) /* QPSK carrier array */
#define SUF_OrderAnalysisArrayAllocate32(a)     ((SLInt32_t *)SUF_MemoryAllocate((2 * ((size_t)a)) * sizeof (SLInt32_t)))       /* IIR filter state array */
#define SUF_ComplexRectArrayAllocate32(a)       ((SLComplexRect32_s *)SUF_MemoryAllocate(((size_t)a) * sizeof (SLComplexRect32_s)));    /* Rectangular complex variable array */
#define SUF_ComplexPolarArrayAllocate32(a)      ((SLComplexPolar32_s *)SUF_MemoryAllocate(((size_t)a) * sizeof (SLComplexPolar32_s)));  /* Polar complex variable array */

#define SUF_IndexArrayAllocate(a)               ((SLArrayIndex_t *)SUF_MemoryAllocate(((size_t)a) * sizeof (SLArrayIndex_t)))   /* Index array */
#define SUF_FixDataArrayAllocate(a)             ((SLArrayIndex_t *)SUF_MemoryAllocate(((size_t)a) * sizeof (SLFixData_t)))      /* Fixed point data array */


                            /* Macros to translate frequencies to FFT bin numbers and vice versa */
#define SUF_BinNumberToFrequency(Bin,FFTSize,SampleRate)    (SampleRate * (((SLData_t)Bin) / ((SLData_t)FFTSize)))
#define SUF_BinNumberToFrequency2(Bin,InvFFTSize,SampleRate)    (SampleRate * (((SLData_t)Bin) * InvFFTSize))

#define SUF_FrequencyToBinNumber(Freq,FFTSize,SampleRate)   (SAI_RoundToNearest((Freq / SampleRate) * ((SLData_t)FFTSize)))
#define SUF_FrequencyToBinNumber2(Freq,FFTSize,InvSampleRate)   (SAI_RoundToNearest((Freq * InvSampleRate) * ((SLData_t)FFTSize)))


/* Utility to halt execution of application */

#define SUF_Halt    while(1);


/* Debug utilities */

#ifdef SIGLIB_ENABLE_LOG
#if SIGLIB_ENABLE_LOG
#define SUF_Log(String) { \
        SUF_Debugfprintf (String); \
        }
#else
#define SUF_Log
#endif
#else
#define SUF_Log
#endif


#define SUF_Debug1(Str1,Var1)           printf("%s = ", Str1); printf(" = %lf\n", Var1);_getch();
#define SUF_Debug2(Str1,Var1,Str2,Var2) printf("%s = ", Str1); printf("%lf\n", Var1);\
                                        printf("%s = ", Str2); printf(" = %lf\n", Var2);_getch();
#define SUF_Debug3(Str1,Var1,Str2,Var2,Str3,Var3)   printf("%s = ", Str1); printf(" = %lf\n", Var1);\
                                        printf("%s = ", Str2); printf(" = %lf\n", Var2);\
                                        printf("%s = ", Str3); printf(" = %lf\n", Var3);_getch();
#define SUF_Debug4(Str1,Var1,Str2,Var2,Str3,Var3,Str4,Var4) printf("%s = ", Str1); printf(" = %lf\n", Var1);\
                                        printf("%s = ", Str2); printf(" = %lf\n", Var2);\
                                        printf("%s = ", Str3); printf(" = %lf\n", Var3);\
                                        printf("%s = ", Str4); printf(" = %lf\n", Var4);_getch();

#endif                      /* End of #if _SIGLIB_MACROS_H */

#endif                      /* End of #if SIGLIB */


