/***************************************************************************************
*  (c) Copyright 2017 ENNOVAR, All rights reserved
*
*    create_by:           gxl
*  								 
*    filename:            Spectrum.h
*
*    description:            
*              
*    revision_history:
*        Date                 By         Description
*        2017/08/02           gxl         created
**************************************************************************************/
/***************************************************************************************
*Include Files
***************************************************************************************/
#ifndef SPECTRUM_H
#define SPECTRUM_H

#include "math.h"
/***************************************************************************************
* macro definition
***************************************************************************************/
#define SIGLIB_PI               (3.14159265358979323846264338327950288419716939937510)  // Pi
#define SIGLIB_TWO_PI           (2.0 * SIGLIB_PI)                                       // 2.0 * Pi
#define SIGLIB_ZERO             (0.0) // Zero
#define SIGLIB_ONE              (1.0) // Two
#define SIGLIB_TWO              (2.0) // One
#define SIGLIB_HALF             (0.5)

#define SIGLIB_NO_ERROR         0   // Indicates no SigLib error occurred in SigLib operation
#define SIGLIB_ERROR            1   // Indicates a generic SigLib error has occurred
#define SIGLIB_MEM_ALLOC_ERROR  2   // Indicates SigLib memory allocation error occurred
#define SIGLIB_PARAMETER_ERROR  3   // Indicates SigLib function parameter error occurred

typedef    float      SLData_t;       // Declare data types
typedef    long       SLFixData_t; 

enum SLWindow_t                     // Window types
{
    SIGLIB_HANNING = 1,	
	SIGLIB_TRIANGLE,
	SIGLIB_RECTANGLE,	
    SIGLIB_HAMMING,	
    SIGLIB_BLACKMAN,
	SIGLIB_KAISER	
};
//FFT Average type definition
enum SLFftAverage_t
{
	SIGLIB_FFTAVG_NONE,			//No average
	SIGLIB_FFTAVG_PEAKHOLD,		//Peakhold of each point
	SIGLIB_FFTAVG_VECTOR,		//Average each point directly
	SIGLIB_FFTAVG_RMS			//Process each point with RMS
	
};

#define SDS_Sin(a)		((SLData_t)sin((double)a))
#define SDS_Cos(a)      ((SLData_t)cos((double)a))
#define SDS_Pow(a,b)                ((SLData_t)pow((double)a,(double)b))
#define SDS_Sqrt(a)                 ((SLData_t)sqrt((double)a))
#define	SDS_Abs(a)					((SLData_t)fabs((double)a))
#define	SDS_Exp(a)					((SLData_t)exp((double)a))

#define SDS_Swap(a,b)           {SLData_t SIGLIB_Tmp = (SLData_t)a; a = b; b = SIGLIB_Tmp; }
#define SUF_MemoryAllocate(a)       malloc((size_t)(a))         // Define host memory allocation functions
#define SUF_MemoryFree(a)           free(a)
#define SUF_VectorArrayAllocate(a)              ((SLData_t *)SUF_MemoryAllocate(((size_t)a) * sizeof (SLData_t)))               /* Vector array */
#define SUF_FftCoefficientAllocate(a)           ((SLData_t *)SUF_MemoryAllocate(((3 * ((size_t)a)) / 4) * sizeof (SLData_t)))   /* FFT twiddle factor coefficient array */


/***************************************************************************************
* function declaration 
***************************************************************************************/

#endif

