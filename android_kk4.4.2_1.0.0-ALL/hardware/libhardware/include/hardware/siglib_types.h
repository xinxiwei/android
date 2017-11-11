/**************************************************************************
File Name               : siglib_types.h    | Author    : JOHN EDWARDS
Siglib Library Version  : 8.60              |
----------------------------------------+----------------------------------
Compiler  : Independent                 | Start Date    : 22/01/00
Options   :                             | Latest Update : 02/01/17
---------------------------------------------------------------------------
Support for SigLib is available via EMail : support@numerix-dsp.com

This file may be modified, in any way, providing that this header remains
within the file and none of the header contents are removed or modified.

THIS FILE MAY ONLY BE USED IN ACCORDANCE WITH THE TERMS OF THE NUMERIX LTD.
END USER LICENSE AGREEMENT.
DELETION OF ANY INFORMATION IN THIS HEADER IS IN VIOLATION OF YOUR LICENSE.

Copyright (C) 1999 to 2017 Sigma Numerix Ltd.
---------------------------------------------------------------------------
Description : Structure and enumerated type definitions header file for
        SigLib DSP library

Update history :
        See history.txt for more details.

****************************************************************************/


#if SIGLIB

#ifndef     _SIGLIB_TYPES_H
#define     _SIGLIB_TYPES_H

            // SigLib complex data structures
typedef struct                      // Complex Cartesian (Rectangular) numbers
{
    SLData_t    real;
    SLData_t    imag;
} SLComplexRect_s;

typedef struct                      // Complex Polar numbers
{
    SLData_t    magn;
    SLData_t    angle;
} SLComplexPolar_s;


typedef struct                      // Complex Cartesian (Rectangular) numbers
{
    SLInt16_t   real;
    SLInt16_t   imag;
} SLComplexRect16_s;

typedef struct                      // Complex Polar numbers
{
    SLInt16_t   magn;
    SLInt16_t   angle;
} SLComplexPolar16_s;


typedef struct                      // Complex Cartesian (Rectangular) numbers
{
    SLInt32_t   real;
    SLInt32_t   imag;
} SLComplexRect32_s;

typedef struct                      // Complex Polar numbers
{
    SLInt32_t   magn;
    SLInt32_t   angle;
} SLComplexPolar32_s;


            // SigLib enumerated data types
enum SLWindow_t                     // Window types
{
    SIGLIB_HANNING,
    SIGLIB_HAMMING,
    SIGLIB_BLACKMAN,
    SIGLIB_BARTLETT_TRIANGLE_ZERO_END_POINTS,
    SIGLIB_BARTLETT_TRIANGLE_NON_ZERO_END_POINTS,
    SIGLIB_KAISER,
    SIGLIB_BMAN_HARRIS,
    SIGLIB_RECTANGLE,
    SIGLIB_FLAT_TOP
};

enum SLSignal_t                     // Signal generation types
{
    SIGLIB_SINE_WAVE,
    SIGLIB_COS_WAVE,
    SIGLIB_WHITE_NOISE,
    SIGLIB_GAUSSIAN_NOISE,
    SIGLIB_CHIRP_LIN,
    SIGLIB_CHIRP_NL,
    SIGLIB_SQUARE_WAVE,
    SIGLIB_TRIANGLE_WAVE,
    SIGLIB_IMPULSE,
    SIGLIB_IMPULSE_STREAM,
    SIGLIB_STEP,
    SIGLIB_PN_SEQUENCE,
    SIGLIB_DC_LEVEL
};

enum SLSignalFillMode_t             // Signal buffer fill modes
{
    SIGLIB_FILL,
    SIGLIB_ADD
};

enum SLSignalSign_t                 // Signal data sign types
{
    SIGLIB_SIGNED_DATA,
    SIGLIB_UNSIGNED_DATA
};

enum SLEcho_t                       // Echo types - feedbackward / feedforward
{
    SIGLIB_ECHO,
    SIGLIB_REVERB
};

enum SLRoundingMode_t               // Rounding of data mode
{
    SIGLIB_ROUND_UP,
    SIGLIB_ROUND_TO_NEAREST,
    SIGLIB_ROUND_DOWN,
    SIGLIB_ROUND_TO_ZERO,
    SIGLIB_ROUND_AWAY_FROM_ZERO
};

enum SLModuloMode_t                 // Data modulo arithmetic mode
{
    SIGLIB_SINGLE_SIDED_MODULO,
    SIGLIB_DOUBLE_SIDED_MODULO
};

enum SLClipMode_t                   // Data clipping modes
{
    SIGLIB_CLIP_ABOVE = 1,
    SIGLIB_CLIP_BOTH = 0,
    SIGLIB_CLIP_BELOW = -1
};

enum SLThresholdMode_t              // Data threshold and clamping modes
{
    SIGLIB_SINGLE_SIDED_THOLD,
    SIGLIB_DOUBLE_SIDED_THOLD
};

enum SLLevelCrossingMode_t          // Zero crossing detection modes
{
    SIGLIB_POSITIVE_LEVEL_CROSS,
    SIGLIB_NEGATIVE_LEVEL_CROSS,
    SIGLIB_ALL_LEVEL_CROSS
};

enum SLArbitraryFFT_t               // Arbitrary FFT type
{
    SIGLIB_ARB_FFT_DO_CZT,              // Array is NOT integer power of 2 length, use chirp z-transform
    SIGLIB_ARB_FFT_DO_FFT               // Array is integer power of 2 length, use FFT
};

enum SLParity_t                     // Asynchronous data parity types
{
    SIGLIB_NO_PARITY,
    SIGLIB_EVEN_PARITY,
    SIGLIB_ODD_PARITY
};

enum SLELGTriggerTiming_t           // Early-late gate trigger timing
{
    SIGLIB_ELG_TRIGGER_START,           // Locate the trigger at the start of the symbol
    SIGLIB_ELG_TRIGGER_MIDDLE           // Locate the trigger in the middle of the symbol
};

enum SLCostasLoopFeedbackMode_t     // Costas loop feedback mode
{
    SIGLIB_COSTAS_LOOP_MULTIPLY_LOOP,
    SIGLIB_COSTAS_LOOP_POLARITY_LOOP,
    SIGLIB_COSTAS_LOOP_HARD_LIMITED_LOOP
};

enum SLIIRNormalizedCoeffs_t        // Normalized filter coefficients
{
    SIGLIB_BUTTERWORTH_IIR_NORM_COEFFS,
    SIGLIB_BESSEL_IIR_NORM_COEFFS
};

enum SL3x3Coeffs_t                  // 3x3 filter coefficients
{
    SIGLIB_EDGE_ENHANCEMENT,
    SIGLIB_HORIZONTAL_EDGE,
    SIGLIB_VERTICAL_EDGE
};

enum SLSignalCoherenceType_t        // Signal coherence type - used in order analysis
{
    SIGLIB_SIGNAL_COHERENT,
    SIGLIB_SIGNAL_INCOHERENT
};

enum SLFindType_t                   // Find type
{
    SIGLIB_FIND_GREATER_THAN_ZERO,
    SIGLIB_FIND_GREATER_THAN_OR_EQUAL_TO_ZERO,
    SIGLIB_FIND_EQUAL_TO_ZERO,
    SIGLIB_FIND_LESS_THAN_ZERO,
    SIGLIB_FIND_LESS_THAN_OR_EQUAL_TO_ZERO,
    SIGLIB_FIND_NOT_EQUAL_TO_ZERO
};

enum SLCompareType_t                // Compare type
{
    SIGLIB_NOT_EQUAL,
    SIGLIB_EQUAL
};

enum SLDeGlitchMode_t               // Data de-glitch modes
{
    SIGLIB_DEGLITCH_ABOVE = 1,
    SIGLIB_DEGLITCH_BOTH = 0,
    SIGLIB_DEGLITCH_BELOW = -1
};

enum SLAlign_t                      // Signal alignment types
{
    SIGLIB_ALIGN_EXTEND,
    SIGLIB_ALIGN_CROP
};


#endif                      // End of #if _SIGLIB_TYPES_H

#endif                      // End of #if SIGLIB

