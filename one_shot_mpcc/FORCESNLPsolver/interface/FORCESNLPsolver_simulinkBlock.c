/*
FORCESNLPsolver : A fast customized optimization solver.

Copyright (C) 2013-2016 EMBOTECH GMBH [info@embotech.com]. All rights reserved.


This software is intended for simulation and testing purposes only. 
Use of this software for any commercial purpose is prohibited.

This program is distributed in the hope that it will be useful.
EMBOTECH makes NO WARRANTIES with respect to the use of the software 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE. 

EMBOTECH shall not have any liability for any damage arising from the use
of the software.

This Agreement shall exclusively be governed by and interpreted in 
accordance with the laws of Switzerland, excluding its principles
of conflict of laws. The Courts of Zurich-City shall have exclusive 
jurisdiction in case of any dispute.

*/


#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME FORCESNLPsolver_simulinkBlock

#include "simstruc.h"



/* SYSTEM INCLUDES FOR TIMING ------------------------------------------ */


/* include FORCES functions and defs */
#include "../include/FORCESNLPsolver.h" 

#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif

typedef FORCESNLPsolverINTERFACE_FLOAT FORCESNLPsolverNMPC_FLOAT;

extern void FORCESNLPsolver_casadi2forces(double *x, double *y, double *l, double *p, double *f, double *nabla_f, double *c, double *nabla_c, double *h, double *nabla_h, double *H, int stage);
FORCESNLPsolver_ExtFunc pt2Function = &FORCESNLPsolver_casadi2forces;




/*====================*
 * S-function methods *
 *====================*/
/* Function: mdlInitializeSizes =========================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{

    DECL_AND_INIT_DIMSINFO(inputDimsInfo);
    DECL_AND_INIT_DIMSINFO(outputDimsInfo);
    ssSetNumSFcnParams(S, 0);
     if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
	 return; /* Parameter mismatch will be reported by Simulink */
     }

	/* initialize size of continuous and discrete states to zero */
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

	/* initialize input ports - there are 3 in total */
    if (!ssSetNumInputPorts(S, 3)) return;
    	
	/* Input Port 0 */
    ssSetInputPortMatrixDimensions(S,  0, 3220, 1);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 0, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/	
	/* Input Port 1 */
    ssSetInputPortMatrixDimensions(S,  1, 15, 1);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 1, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 1, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 1, 1); /*direct input signal access*/	
	/* Input Port 2 */
    ssSetInputPortMatrixDimensions(S,  2, 3220, 1);
    ssSetInputPortDataType(S, 2, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 2, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 2, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 2, 1); /*direct input signal access*/ 


	/* initialize output ports - there are 161 in total */
    if (!ssSetNumOutputPorts(S, 161)) return;    
		
	/* Output Port 0 */
    ssSetOutputPortMatrixDimensions(S,  0, 20, 1);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 1 */
    ssSetOutputPortMatrixDimensions(S,  1, 20, 1);
    ssSetOutputPortDataType(S, 1, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 1, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 2 */
    ssSetOutputPortMatrixDimensions(S,  2, 20, 1);
    ssSetOutputPortDataType(S, 2, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 2, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 3 */
    ssSetOutputPortMatrixDimensions(S,  3, 20, 1);
    ssSetOutputPortDataType(S, 3, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 3, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 4 */
    ssSetOutputPortMatrixDimensions(S,  4, 20, 1);
    ssSetOutputPortDataType(S, 4, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 4, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 5 */
    ssSetOutputPortMatrixDimensions(S,  5, 20, 1);
    ssSetOutputPortDataType(S, 5, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 5, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 6 */
    ssSetOutputPortMatrixDimensions(S,  6, 20, 1);
    ssSetOutputPortDataType(S, 6, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 6, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 7 */
    ssSetOutputPortMatrixDimensions(S,  7, 20, 1);
    ssSetOutputPortDataType(S, 7, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 7, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 8 */
    ssSetOutputPortMatrixDimensions(S,  8, 20, 1);
    ssSetOutputPortDataType(S, 8, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 8, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 9 */
    ssSetOutputPortMatrixDimensions(S,  9, 20, 1);
    ssSetOutputPortDataType(S, 9, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 9, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 10 */
    ssSetOutputPortMatrixDimensions(S,  10, 20, 1);
    ssSetOutputPortDataType(S, 10, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 10, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 11 */
    ssSetOutputPortMatrixDimensions(S,  11, 20, 1);
    ssSetOutputPortDataType(S, 11, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 11, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 12 */
    ssSetOutputPortMatrixDimensions(S,  12, 20, 1);
    ssSetOutputPortDataType(S, 12, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 12, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 13 */
    ssSetOutputPortMatrixDimensions(S,  13, 20, 1);
    ssSetOutputPortDataType(S, 13, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 13, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 14 */
    ssSetOutputPortMatrixDimensions(S,  14, 20, 1);
    ssSetOutputPortDataType(S, 14, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 14, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 15 */
    ssSetOutputPortMatrixDimensions(S,  15, 20, 1);
    ssSetOutputPortDataType(S, 15, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 15, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 16 */
    ssSetOutputPortMatrixDimensions(S,  16, 20, 1);
    ssSetOutputPortDataType(S, 16, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 16, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 17 */
    ssSetOutputPortMatrixDimensions(S,  17, 20, 1);
    ssSetOutputPortDataType(S, 17, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 17, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 18 */
    ssSetOutputPortMatrixDimensions(S,  18, 20, 1);
    ssSetOutputPortDataType(S, 18, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 18, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 19 */
    ssSetOutputPortMatrixDimensions(S,  19, 20, 1);
    ssSetOutputPortDataType(S, 19, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 19, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 20 */
    ssSetOutputPortMatrixDimensions(S,  20, 20, 1);
    ssSetOutputPortDataType(S, 20, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 20, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 21 */
    ssSetOutputPortMatrixDimensions(S,  21, 20, 1);
    ssSetOutputPortDataType(S, 21, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 21, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 22 */
    ssSetOutputPortMatrixDimensions(S,  22, 20, 1);
    ssSetOutputPortDataType(S, 22, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 22, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 23 */
    ssSetOutputPortMatrixDimensions(S,  23, 20, 1);
    ssSetOutputPortDataType(S, 23, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 23, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 24 */
    ssSetOutputPortMatrixDimensions(S,  24, 20, 1);
    ssSetOutputPortDataType(S, 24, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 24, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 25 */
    ssSetOutputPortMatrixDimensions(S,  25, 20, 1);
    ssSetOutputPortDataType(S, 25, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 25, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 26 */
    ssSetOutputPortMatrixDimensions(S,  26, 20, 1);
    ssSetOutputPortDataType(S, 26, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 26, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 27 */
    ssSetOutputPortMatrixDimensions(S,  27, 20, 1);
    ssSetOutputPortDataType(S, 27, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 27, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 28 */
    ssSetOutputPortMatrixDimensions(S,  28, 20, 1);
    ssSetOutputPortDataType(S, 28, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 28, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 29 */
    ssSetOutputPortMatrixDimensions(S,  29, 20, 1);
    ssSetOutputPortDataType(S, 29, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 29, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 30 */
    ssSetOutputPortMatrixDimensions(S,  30, 20, 1);
    ssSetOutputPortDataType(S, 30, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 30, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 31 */
    ssSetOutputPortMatrixDimensions(S,  31, 20, 1);
    ssSetOutputPortDataType(S, 31, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 31, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 32 */
    ssSetOutputPortMatrixDimensions(S,  32, 20, 1);
    ssSetOutputPortDataType(S, 32, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 32, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 33 */
    ssSetOutputPortMatrixDimensions(S,  33, 20, 1);
    ssSetOutputPortDataType(S, 33, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 33, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 34 */
    ssSetOutputPortMatrixDimensions(S,  34, 20, 1);
    ssSetOutputPortDataType(S, 34, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 34, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 35 */
    ssSetOutputPortMatrixDimensions(S,  35, 20, 1);
    ssSetOutputPortDataType(S, 35, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 35, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 36 */
    ssSetOutputPortMatrixDimensions(S,  36, 20, 1);
    ssSetOutputPortDataType(S, 36, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 36, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 37 */
    ssSetOutputPortMatrixDimensions(S,  37, 20, 1);
    ssSetOutputPortDataType(S, 37, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 37, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 38 */
    ssSetOutputPortMatrixDimensions(S,  38, 20, 1);
    ssSetOutputPortDataType(S, 38, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 38, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 39 */
    ssSetOutputPortMatrixDimensions(S,  39, 20, 1);
    ssSetOutputPortDataType(S, 39, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 39, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 40 */
    ssSetOutputPortMatrixDimensions(S,  40, 20, 1);
    ssSetOutputPortDataType(S, 40, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 40, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 41 */
    ssSetOutputPortMatrixDimensions(S,  41, 20, 1);
    ssSetOutputPortDataType(S, 41, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 41, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 42 */
    ssSetOutputPortMatrixDimensions(S,  42, 20, 1);
    ssSetOutputPortDataType(S, 42, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 42, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 43 */
    ssSetOutputPortMatrixDimensions(S,  43, 20, 1);
    ssSetOutputPortDataType(S, 43, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 43, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 44 */
    ssSetOutputPortMatrixDimensions(S,  44, 20, 1);
    ssSetOutputPortDataType(S, 44, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 44, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 45 */
    ssSetOutputPortMatrixDimensions(S,  45, 20, 1);
    ssSetOutputPortDataType(S, 45, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 45, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 46 */
    ssSetOutputPortMatrixDimensions(S,  46, 20, 1);
    ssSetOutputPortDataType(S, 46, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 46, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 47 */
    ssSetOutputPortMatrixDimensions(S,  47, 20, 1);
    ssSetOutputPortDataType(S, 47, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 47, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 48 */
    ssSetOutputPortMatrixDimensions(S,  48, 20, 1);
    ssSetOutputPortDataType(S, 48, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 48, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 49 */
    ssSetOutputPortMatrixDimensions(S,  49, 20, 1);
    ssSetOutputPortDataType(S, 49, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 49, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 50 */
    ssSetOutputPortMatrixDimensions(S,  50, 20, 1);
    ssSetOutputPortDataType(S, 50, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 50, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 51 */
    ssSetOutputPortMatrixDimensions(S,  51, 20, 1);
    ssSetOutputPortDataType(S, 51, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 51, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 52 */
    ssSetOutputPortMatrixDimensions(S,  52, 20, 1);
    ssSetOutputPortDataType(S, 52, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 52, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 53 */
    ssSetOutputPortMatrixDimensions(S,  53, 20, 1);
    ssSetOutputPortDataType(S, 53, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 53, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 54 */
    ssSetOutputPortMatrixDimensions(S,  54, 20, 1);
    ssSetOutputPortDataType(S, 54, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 54, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 55 */
    ssSetOutputPortMatrixDimensions(S,  55, 20, 1);
    ssSetOutputPortDataType(S, 55, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 55, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 56 */
    ssSetOutputPortMatrixDimensions(S,  56, 20, 1);
    ssSetOutputPortDataType(S, 56, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 56, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 57 */
    ssSetOutputPortMatrixDimensions(S,  57, 20, 1);
    ssSetOutputPortDataType(S, 57, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 57, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 58 */
    ssSetOutputPortMatrixDimensions(S,  58, 20, 1);
    ssSetOutputPortDataType(S, 58, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 58, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 59 */
    ssSetOutputPortMatrixDimensions(S,  59, 20, 1);
    ssSetOutputPortDataType(S, 59, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 59, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 60 */
    ssSetOutputPortMatrixDimensions(S,  60, 20, 1);
    ssSetOutputPortDataType(S, 60, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 60, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 61 */
    ssSetOutputPortMatrixDimensions(S,  61, 20, 1);
    ssSetOutputPortDataType(S, 61, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 61, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 62 */
    ssSetOutputPortMatrixDimensions(S,  62, 20, 1);
    ssSetOutputPortDataType(S, 62, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 62, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 63 */
    ssSetOutputPortMatrixDimensions(S,  63, 20, 1);
    ssSetOutputPortDataType(S, 63, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 63, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 64 */
    ssSetOutputPortMatrixDimensions(S,  64, 20, 1);
    ssSetOutputPortDataType(S, 64, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 64, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 65 */
    ssSetOutputPortMatrixDimensions(S,  65, 20, 1);
    ssSetOutputPortDataType(S, 65, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 65, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 66 */
    ssSetOutputPortMatrixDimensions(S,  66, 20, 1);
    ssSetOutputPortDataType(S, 66, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 66, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 67 */
    ssSetOutputPortMatrixDimensions(S,  67, 20, 1);
    ssSetOutputPortDataType(S, 67, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 67, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 68 */
    ssSetOutputPortMatrixDimensions(S,  68, 20, 1);
    ssSetOutputPortDataType(S, 68, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 68, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 69 */
    ssSetOutputPortMatrixDimensions(S,  69, 20, 1);
    ssSetOutputPortDataType(S, 69, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 69, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 70 */
    ssSetOutputPortMatrixDimensions(S,  70, 20, 1);
    ssSetOutputPortDataType(S, 70, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 70, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 71 */
    ssSetOutputPortMatrixDimensions(S,  71, 20, 1);
    ssSetOutputPortDataType(S, 71, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 71, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 72 */
    ssSetOutputPortMatrixDimensions(S,  72, 20, 1);
    ssSetOutputPortDataType(S, 72, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 72, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 73 */
    ssSetOutputPortMatrixDimensions(S,  73, 20, 1);
    ssSetOutputPortDataType(S, 73, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 73, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 74 */
    ssSetOutputPortMatrixDimensions(S,  74, 20, 1);
    ssSetOutputPortDataType(S, 74, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 74, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 75 */
    ssSetOutputPortMatrixDimensions(S,  75, 20, 1);
    ssSetOutputPortDataType(S, 75, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 75, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 76 */
    ssSetOutputPortMatrixDimensions(S,  76, 20, 1);
    ssSetOutputPortDataType(S, 76, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 76, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 77 */
    ssSetOutputPortMatrixDimensions(S,  77, 20, 1);
    ssSetOutputPortDataType(S, 77, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 77, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 78 */
    ssSetOutputPortMatrixDimensions(S,  78, 20, 1);
    ssSetOutputPortDataType(S, 78, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 78, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 79 */
    ssSetOutputPortMatrixDimensions(S,  79, 20, 1);
    ssSetOutputPortDataType(S, 79, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 79, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 80 */
    ssSetOutputPortMatrixDimensions(S,  80, 20, 1);
    ssSetOutputPortDataType(S, 80, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 80, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 81 */
    ssSetOutputPortMatrixDimensions(S,  81, 20, 1);
    ssSetOutputPortDataType(S, 81, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 81, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 82 */
    ssSetOutputPortMatrixDimensions(S,  82, 20, 1);
    ssSetOutputPortDataType(S, 82, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 82, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 83 */
    ssSetOutputPortMatrixDimensions(S,  83, 20, 1);
    ssSetOutputPortDataType(S, 83, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 83, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 84 */
    ssSetOutputPortMatrixDimensions(S,  84, 20, 1);
    ssSetOutputPortDataType(S, 84, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 84, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 85 */
    ssSetOutputPortMatrixDimensions(S,  85, 20, 1);
    ssSetOutputPortDataType(S, 85, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 85, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 86 */
    ssSetOutputPortMatrixDimensions(S,  86, 20, 1);
    ssSetOutputPortDataType(S, 86, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 86, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 87 */
    ssSetOutputPortMatrixDimensions(S,  87, 20, 1);
    ssSetOutputPortDataType(S, 87, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 87, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 88 */
    ssSetOutputPortMatrixDimensions(S,  88, 20, 1);
    ssSetOutputPortDataType(S, 88, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 88, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 89 */
    ssSetOutputPortMatrixDimensions(S,  89, 20, 1);
    ssSetOutputPortDataType(S, 89, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 89, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 90 */
    ssSetOutputPortMatrixDimensions(S,  90, 20, 1);
    ssSetOutputPortDataType(S, 90, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 90, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 91 */
    ssSetOutputPortMatrixDimensions(S,  91, 20, 1);
    ssSetOutputPortDataType(S, 91, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 91, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 92 */
    ssSetOutputPortMatrixDimensions(S,  92, 20, 1);
    ssSetOutputPortDataType(S, 92, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 92, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 93 */
    ssSetOutputPortMatrixDimensions(S,  93, 20, 1);
    ssSetOutputPortDataType(S, 93, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 93, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 94 */
    ssSetOutputPortMatrixDimensions(S,  94, 20, 1);
    ssSetOutputPortDataType(S, 94, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 94, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 95 */
    ssSetOutputPortMatrixDimensions(S,  95, 20, 1);
    ssSetOutputPortDataType(S, 95, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 95, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 96 */
    ssSetOutputPortMatrixDimensions(S,  96, 20, 1);
    ssSetOutputPortDataType(S, 96, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 96, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 97 */
    ssSetOutputPortMatrixDimensions(S,  97, 20, 1);
    ssSetOutputPortDataType(S, 97, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 97, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 98 */
    ssSetOutputPortMatrixDimensions(S,  98, 20, 1);
    ssSetOutputPortDataType(S, 98, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 98, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 99 */
    ssSetOutputPortMatrixDimensions(S,  99, 20, 1);
    ssSetOutputPortDataType(S, 99, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 99, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 100 */
    ssSetOutputPortMatrixDimensions(S,  100, 20, 1);
    ssSetOutputPortDataType(S, 100, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 100, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 101 */
    ssSetOutputPortMatrixDimensions(S,  101, 20, 1);
    ssSetOutputPortDataType(S, 101, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 101, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 102 */
    ssSetOutputPortMatrixDimensions(S,  102, 20, 1);
    ssSetOutputPortDataType(S, 102, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 102, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 103 */
    ssSetOutputPortMatrixDimensions(S,  103, 20, 1);
    ssSetOutputPortDataType(S, 103, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 103, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 104 */
    ssSetOutputPortMatrixDimensions(S,  104, 20, 1);
    ssSetOutputPortDataType(S, 104, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 104, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 105 */
    ssSetOutputPortMatrixDimensions(S,  105, 20, 1);
    ssSetOutputPortDataType(S, 105, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 105, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 106 */
    ssSetOutputPortMatrixDimensions(S,  106, 20, 1);
    ssSetOutputPortDataType(S, 106, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 106, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 107 */
    ssSetOutputPortMatrixDimensions(S,  107, 20, 1);
    ssSetOutputPortDataType(S, 107, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 107, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 108 */
    ssSetOutputPortMatrixDimensions(S,  108, 20, 1);
    ssSetOutputPortDataType(S, 108, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 108, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 109 */
    ssSetOutputPortMatrixDimensions(S,  109, 20, 1);
    ssSetOutputPortDataType(S, 109, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 109, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 110 */
    ssSetOutputPortMatrixDimensions(S,  110, 20, 1);
    ssSetOutputPortDataType(S, 110, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 110, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 111 */
    ssSetOutputPortMatrixDimensions(S,  111, 20, 1);
    ssSetOutputPortDataType(S, 111, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 111, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 112 */
    ssSetOutputPortMatrixDimensions(S,  112, 20, 1);
    ssSetOutputPortDataType(S, 112, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 112, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 113 */
    ssSetOutputPortMatrixDimensions(S,  113, 20, 1);
    ssSetOutputPortDataType(S, 113, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 113, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 114 */
    ssSetOutputPortMatrixDimensions(S,  114, 20, 1);
    ssSetOutputPortDataType(S, 114, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 114, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 115 */
    ssSetOutputPortMatrixDimensions(S,  115, 20, 1);
    ssSetOutputPortDataType(S, 115, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 115, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 116 */
    ssSetOutputPortMatrixDimensions(S,  116, 20, 1);
    ssSetOutputPortDataType(S, 116, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 116, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 117 */
    ssSetOutputPortMatrixDimensions(S,  117, 20, 1);
    ssSetOutputPortDataType(S, 117, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 117, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 118 */
    ssSetOutputPortMatrixDimensions(S,  118, 20, 1);
    ssSetOutputPortDataType(S, 118, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 118, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 119 */
    ssSetOutputPortMatrixDimensions(S,  119, 20, 1);
    ssSetOutputPortDataType(S, 119, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 119, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 120 */
    ssSetOutputPortMatrixDimensions(S,  120, 20, 1);
    ssSetOutputPortDataType(S, 120, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 120, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 121 */
    ssSetOutputPortMatrixDimensions(S,  121, 20, 1);
    ssSetOutputPortDataType(S, 121, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 121, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 122 */
    ssSetOutputPortMatrixDimensions(S,  122, 20, 1);
    ssSetOutputPortDataType(S, 122, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 122, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 123 */
    ssSetOutputPortMatrixDimensions(S,  123, 20, 1);
    ssSetOutputPortDataType(S, 123, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 123, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 124 */
    ssSetOutputPortMatrixDimensions(S,  124, 20, 1);
    ssSetOutputPortDataType(S, 124, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 124, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 125 */
    ssSetOutputPortMatrixDimensions(S,  125, 20, 1);
    ssSetOutputPortDataType(S, 125, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 125, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 126 */
    ssSetOutputPortMatrixDimensions(S,  126, 20, 1);
    ssSetOutputPortDataType(S, 126, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 126, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 127 */
    ssSetOutputPortMatrixDimensions(S,  127, 20, 1);
    ssSetOutputPortDataType(S, 127, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 127, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 128 */
    ssSetOutputPortMatrixDimensions(S,  128, 20, 1);
    ssSetOutputPortDataType(S, 128, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 128, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 129 */
    ssSetOutputPortMatrixDimensions(S,  129, 20, 1);
    ssSetOutputPortDataType(S, 129, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 129, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 130 */
    ssSetOutputPortMatrixDimensions(S,  130, 20, 1);
    ssSetOutputPortDataType(S, 130, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 130, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 131 */
    ssSetOutputPortMatrixDimensions(S,  131, 20, 1);
    ssSetOutputPortDataType(S, 131, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 131, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 132 */
    ssSetOutputPortMatrixDimensions(S,  132, 20, 1);
    ssSetOutputPortDataType(S, 132, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 132, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 133 */
    ssSetOutputPortMatrixDimensions(S,  133, 20, 1);
    ssSetOutputPortDataType(S, 133, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 133, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 134 */
    ssSetOutputPortMatrixDimensions(S,  134, 20, 1);
    ssSetOutputPortDataType(S, 134, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 134, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 135 */
    ssSetOutputPortMatrixDimensions(S,  135, 20, 1);
    ssSetOutputPortDataType(S, 135, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 135, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 136 */
    ssSetOutputPortMatrixDimensions(S,  136, 20, 1);
    ssSetOutputPortDataType(S, 136, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 136, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 137 */
    ssSetOutputPortMatrixDimensions(S,  137, 20, 1);
    ssSetOutputPortDataType(S, 137, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 137, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 138 */
    ssSetOutputPortMatrixDimensions(S,  138, 20, 1);
    ssSetOutputPortDataType(S, 138, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 138, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 139 */
    ssSetOutputPortMatrixDimensions(S,  139, 20, 1);
    ssSetOutputPortDataType(S, 139, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 139, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 140 */
    ssSetOutputPortMatrixDimensions(S,  140, 20, 1);
    ssSetOutputPortDataType(S, 140, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 140, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 141 */
    ssSetOutputPortMatrixDimensions(S,  141, 20, 1);
    ssSetOutputPortDataType(S, 141, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 141, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 142 */
    ssSetOutputPortMatrixDimensions(S,  142, 20, 1);
    ssSetOutputPortDataType(S, 142, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 142, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 143 */
    ssSetOutputPortMatrixDimensions(S,  143, 20, 1);
    ssSetOutputPortDataType(S, 143, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 143, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 144 */
    ssSetOutputPortMatrixDimensions(S,  144, 20, 1);
    ssSetOutputPortDataType(S, 144, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 144, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 145 */
    ssSetOutputPortMatrixDimensions(S,  145, 20, 1);
    ssSetOutputPortDataType(S, 145, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 145, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 146 */
    ssSetOutputPortMatrixDimensions(S,  146, 20, 1);
    ssSetOutputPortDataType(S, 146, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 146, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 147 */
    ssSetOutputPortMatrixDimensions(S,  147, 20, 1);
    ssSetOutputPortDataType(S, 147, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 147, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 148 */
    ssSetOutputPortMatrixDimensions(S,  148, 20, 1);
    ssSetOutputPortDataType(S, 148, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 148, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 149 */
    ssSetOutputPortMatrixDimensions(S,  149, 20, 1);
    ssSetOutputPortDataType(S, 149, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 149, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 150 */
    ssSetOutputPortMatrixDimensions(S,  150, 20, 1);
    ssSetOutputPortDataType(S, 150, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 150, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 151 */
    ssSetOutputPortMatrixDimensions(S,  151, 20, 1);
    ssSetOutputPortDataType(S, 151, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 151, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 152 */
    ssSetOutputPortMatrixDimensions(S,  152, 20, 1);
    ssSetOutputPortDataType(S, 152, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 152, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 153 */
    ssSetOutputPortMatrixDimensions(S,  153, 20, 1);
    ssSetOutputPortDataType(S, 153, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 153, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 154 */
    ssSetOutputPortMatrixDimensions(S,  154, 20, 1);
    ssSetOutputPortDataType(S, 154, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 154, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 155 */
    ssSetOutputPortMatrixDimensions(S,  155, 20, 1);
    ssSetOutputPortDataType(S, 155, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 155, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 156 */
    ssSetOutputPortMatrixDimensions(S,  156, 20, 1);
    ssSetOutputPortDataType(S, 156, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 156, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 157 */
    ssSetOutputPortMatrixDimensions(S,  157, 20, 1);
    ssSetOutputPortDataType(S, 157, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 157, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 158 */
    ssSetOutputPortMatrixDimensions(S,  158, 20, 1);
    ssSetOutputPortDataType(S, 158, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 158, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 159 */
    ssSetOutputPortMatrixDimensions(S,  159, 20, 1);
    ssSetOutputPortDataType(S, 159, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 159, COMPLEX_NO); /* no complex signals suppported */
	
	/* Output Port 160 */
    ssSetOutputPortMatrixDimensions(S,  160, 20, 1);
    ssSetOutputPortDataType(S, 160, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 160, COMPLEX_NO); /* no complex signals suppported */


	/* set sampling time */
    ssSetNumSampleTimes(S, 1);

	/* set internal memory of block */
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
	/* SS_OPTION_USE_TLC_WITH_ACCELERATOR removed */ 
	/* SS_OPTION_USE_TLC_WITH_ACCELERATOR removed */ 
    /* ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE |
		             SS_OPTION_WORKS_WITH_CODE_REUSE)); */
	ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE );

	
}

#if defined(MATLAB_MEX_FILE)
#define MDL_SET_INPUT_PORT_DIMENSION_INFO
static void mdlSetInputPortDimensionInfo(SimStruct        *S, 
                                         int_T            port,
                                         const DimsInfo_T *dimsInfo)
{
    if(!ssSetInputPortDimensionInfo(S, port, dimsInfo)) return;
}
#endif

#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO
#if defined(MDL_SET_OUTPUT_PORT_DIMENSION_INFO)
static void mdlSetOutputPortDimensionInfo(SimStruct        *S, 
                                          int_T            port, 
                                          const DimsInfo_T *dimsInfo)
{
 if (!ssSetOutputPortDimensionInfo(S, port, dimsInfo)) return;
}
#endif
# define MDL_SET_INPUT_PORT_FRAME_DATA
static void mdlSetInputPortFrameData(SimStruct  *S, 
                                     int_T      port,
                                     Frame_T    frameData)
{
    ssSetInputPortFrameData(S, port, frameData);
}
/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy  the sample time.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_SET_INPUT_PORT_DATA_TYPE
static void mdlSetInputPortDataType(SimStruct *S, int port, DTypeId dType)
{
    ssSetInputPortDataType( S, 0, dType);
}
#define MDL_SET_OUTPUT_PORT_DATA_TYPE
static void mdlSetOutputPortDataType(SimStruct *S, int port, DTypeId dType)
{
    ssSetOutputPortDataType(S, 0, dType);
}

#define MDL_SET_DEFAULT_PORT_DATA_TYPES
static void mdlSetDefaultPortDataTypes(SimStruct *S)
{
  ssSetInputPortDataType( S, 0, SS_DOUBLE);
 ssSetOutputPortDataType(S, 0, SS_DOUBLE);
}





/* Function: mdlOutputs =======================================================
 *
*/
static void mdlOutputs(SimStruct *S, int_T tid)
{
	int i, j, k;
	
	/* file pointer for printing */
	FILE *fp = NULL;

	/* Simulink data */
	const real_T *x0 = (const real_T*) ssGetInputPortSignal(S,0);
	const real_T *xinit = (const real_T*) ssGetInputPortSignal(S,1);
	const real_T *all_parameters = (const real_T*) ssGetInputPortSignal(S,2);
	
    real_T *x001 = (real_T*) ssGetOutputPortSignal(S,0);
	real_T *x002 = (real_T*) ssGetOutputPortSignal(S,1);
	real_T *x003 = (real_T*) ssGetOutputPortSignal(S,2);
	real_T *x004 = (real_T*) ssGetOutputPortSignal(S,3);
	real_T *x005 = (real_T*) ssGetOutputPortSignal(S,4);
	real_T *x006 = (real_T*) ssGetOutputPortSignal(S,5);
	real_T *x007 = (real_T*) ssGetOutputPortSignal(S,6);
	real_T *x008 = (real_T*) ssGetOutputPortSignal(S,7);
	real_T *x009 = (real_T*) ssGetOutputPortSignal(S,8);
	real_T *x010 = (real_T*) ssGetOutputPortSignal(S,9);
	real_T *x011 = (real_T*) ssGetOutputPortSignal(S,10);
	real_T *x012 = (real_T*) ssGetOutputPortSignal(S,11);
	real_T *x013 = (real_T*) ssGetOutputPortSignal(S,12);
	real_T *x014 = (real_T*) ssGetOutputPortSignal(S,13);
	real_T *x015 = (real_T*) ssGetOutputPortSignal(S,14);
	real_T *x016 = (real_T*) ssGetOutputPortSignal(S,15);
	real_T *x017 = (real_T*) ssGetOutputPortSignal(S,16);
	real_T *x018 = (real_T*) ssGetOutputPortSignal(S,17);
	real_T *x019 = (real_T*) ssGetOutputPortSignal(S,18);
	real_T *x020 = (real_T*) ssGetOutputPortSignal(S,19);
	real_T *x021 = (real_T*) ssGetOutputPortSignal(S,20);
	real_T *x022 = (real_T*) ssGetOutputPortSignal(S,21);
	real_T *x023 = (real_T*) ssGetOutputPortSignal(S,22);
	real_T *x024 = (real_T*) ssGetOutputPortSignal(S,23);
	real_T *x025 = (real_T*) ssGetOutputPortSignal(S,24);
	real_T *x026 = (real_T*) ssGetOutputPortSignal(S,25);
	real_T *x027 = (real_T*) ssGetOutputPortSignal(S,26);
	real_T *x028 = (real_T*) ssGetOutputPortSignal(S,27);
	real_T *x029 = (real_T*) ssGetOutputPortSignal(S,28);
	real_T *x030 = (real_T*) ssGetOutputPortSignal(S,29);
	real_T *x031 = (real_T*) ssGetOutputPortSignal(S,30);
	real_T *x032 = (real_T*) ssGetOutputPortSignal(S,31);
	real_T *x033 = (real_T*) ssGetOutputPortSignal(S,32);
	real_T *x034 = (real_T*) ssGetOutputPortSignal(S,33);
	real_T *x035 = (real_T*) ssGetOutputPortSignal(S,34);
	real_T *x036 = (real_T*) ssGetOutputPortSignal(S,35);
	real_T *x037 = (real_T*) ssGetOutputPortSignal(S,36);
	real_T *x038 = (real_T*) ssGetOutputPortSignal(S,37);
	real_T *x039 = (real_T*) ssGetOutputPortSignal(S,38);
	real_T *x040 = (real_T*) ssGetOutputPortSignal(S,39);
	real_T *x041 = (real_T*) ssGetOutputPortSignal(S,40);
	real_T *x042 = (real_T*) ssGetOutputPortSignal(S,41);
	real_T *x043 = (real_T*) ssGetOutputPortSignal(S,42);
	real_T *x044 = (real_T*) ssGetOutputPortSignal(S,43);
	real_T *x045 = (real_T*) ssGetOutputPortSignal(S,44);
	real_T *x046 = (real_T*) ssGetOutputPortSignal(S,45);
	real_T *x047 = (real_T*) ssGetOutputPortSignal(S,46);
	real_T *x048 = (real_T*) ssGetOutputPortSignal(S,47);
	real_T *x049 = (real_T*) ssGetOutputPortSignal(S,48);
	real_T *x050 = (real_T*) ssGetOutputPortSignal(S,49);
	real_T *x051 = (real_T*) ssGetOutputPortSignal(S,50);
	real_T *x052 = (real_T*) ssGetOutputPortSignal(S,51);
	real_T *x053 = (real_T*) ssGetOutputPortSignal(S,52);
	real_T *x054 = (real_T*) ssGetOutputPortSignal(S,53);
	real_T *x055 = (real_T*) ssGetOutputPortSignal(S,54);
	real_T *x056 = (real_T*) ssGetOutputPortSignal(S,55);
	real_T *x057 = (real_T*) ssGetOutputPortSignal(S,56);
	real_T *x058 = (real_T*) ssGetOutputPortSignal(S,57);
	real_T *x059 = (real_T*) ssGetOutputPortSignal(S,58);
	real_T *x060 = (real_T*) ssGetOutputPortSignal(S,59);
	real_T *x061 = (real_T*) ssGetOutputPortSignal(S,60);
	real_T *x062 = (real_T*) ssGetOutputPortSignal(S,61);
	real_T *x063 = (real_T*) ssGetOutputPortSignal(S,62);
	real_T *x064 = (real_T*) ssGetOutputPortSignal(S,63);
	real_T *x065 = (real_T*) ssGetOutputPortSignal(S,64);
	real_T *x066 = (real_T*) ssGetOutputPortSignal(S,65);
	real_T *x067 = (real_T*) ssGetOutputPortSignal(S,66);
	real_T *x068 = (real_T*) ssGetOutputPortSignal(S,67);
	real_T *x069 = (real_T*) ssGetOutputPortSignal(S,68);
	real_T *x070 = (real_T*) ssGetOutputPortSignal(S,69);
	real_T *x071 = (real_T*) ssGetOutputPortSignal(S,70);
	real_T *x072 = (real_T*) ssGetOutputPortSignal(S,71);
	real_T *x073 = (real_T*) ssGetOutputPortSignal(S,72);
	real_T *x074 = (real_T*) ssGetOutputPortSignal(S,73);
	real_T *x075 = (real_T*) ssGetOutputPortSignal(S,74);
	real_T *x076 = (real_T*) ssGetOutputPortSignal(S,75);
	real_T *x077 = (real_T*) ssGetOutputPortSignal(S,76);
	real_T *x078 = (real_T*) ssGetOutputPortSignal(S,77);
	real_T *x079 = (real_T*) ssGetOutputPortSignal(S,78);
	real_T *x080 = (real_T*) ssGetOutputPortSignal(S,79);
	real_T *x081 = (real_T*) ssGetOutputPortSignal(S,80);
	real_T *x082 = (real_T*) ssGetOutputPortSignal(S,81);
	real_T *x083 = (real_T*) ssGetOutputPortSignal(S,82);
	real_T *x084 = (real_T*) ssGetOutputPortSignal(S,83);
	real_T *x085 = (real_T*) ssGetOutputPortSignal(S,84);
	real_T *x086 = (real_T*) ssGetOutputPortSignal(S,85);
	real_T *x087 = (real_T*) ssGetOutputPortSignal(S,86);
	real_T *x088 = (real_T*) ssGetOutputPortSignal(S,87);
	real_T *x089 = (real_T*) ssGetOutputPortSignal(S,88);
	real_T *x090 = (real_T*) ssGetOutputPortSignal(S,89);
	real_T *x091 = (real_T*) ssGetOutputPortSignal(S,90);
	real_T *x092 = (real_T*) ssGetOutputPortSignal(S,91);
	real_T *x093 = (real_T*) ssGetOutputPortSignal(S,92);
	real_T *x094 = (real_T*) ssGetOutputPortSignal(S,93);
	real_T *x095 = (real_T*) ssGetOutputPortSignal(S,94);
	real_T *x096 = (real_T*) ssGetOutputPortSignal(S,95);
	real_T *x097 = (real_T*) ssGetOutputPortSignal(S,96);
	real_T *x098 = (real_T*) ssGetOutputPortSignal(S,97);
	real_T *x099 = (real_T*) ssGetOutputPortSignal(S,98);
	real_T *x100 = (real_T*) ssGetOutputPortSignal(S,99);
	real_T *x101 = (real_T*) ssGetOutputPortSignal(S,100);
	real_T *x102 = (real_T*) ssGetOutputPortSignal(S,101);
	real_T *x103 = (real_T*) ssGetOutputPortSignal(S,102);
	real_T *x104 = (real_T*) ssGetOutputPortSignal(S,103);
	real_T *x105 = (real_T*) ssGetOutputPortSignal(S,104);
	real_T *x106 = (real_T*) ssGetOutputPortSignal(S,105);
	real_T *x107 = (real_T*) ssGetOutputPortSignal(S,106);
	real_T *x108 = (real_T*) ssGetOutputPortSignal(S,107);
	real_T *x109 = (real_T*) ssGetOutputPortSignal(S,108);
	real_T *x110 = (real_T*) ssGetOutputPortSignal(S,109);
	real_T *x111 = (real_T*) ssGetOutputPortSignal(S,110);
	real_T *x112 = (real_T*) ssGetOutputPortSignal(S,111);
	real_T *x113 = (real_T*) ssGetOutputPortSignal(S,112);
	real_T *x114 = (real_T*) ssGetOutputPortSignal(S,113);
	real_T *x115 = (real_T*) ssGetOutputPortSignal(S,114);
	real_T *x116 = (real_T*) ssGetOutputPortSignal(S,115);
	real_T *x117 = (real_T*) ssGetOutputPortSignal(S,116);
	real_T *x118 = (real_T*) ssGetOutputPortSignal(S,117);
	real_T *x119 = (real_T*) ssGetOutputPortSignal(S,118);
	real_T *x120 = (real_T*) ssGetOutputPortSignal(S,119);
	real_T *x121 = (real_T*) ssGetOutputPortSignal(S,120);
	real_T *x122 = (real_T*) ssGetOutputPortSignal(S,121);
	real_T *x123 = (real_T*) ssGetOutputPortSignal(S,122);
	real_T *x124 = (real_T*) ssGetOutputPortSignal(S,123);
	real_T *x125 = (real_T*) ssGetOutputPortSignal(S,124);
	real_T *x126 = (real_T*) ssGetOutputPortSignal(S,125);
	real_T *x127 = (real_T*) ssGetOutputPortSignal(S,126);
	real_T *x128 = (real_T*) ssGetOutputPortSignal(S,127);
	real_T *x129 = (real_T*) ssGetOutputPortSignal(S,128);
	real_T *x130 = (real_T*) ssGetOutputPortSignal(S,129);
	real_T *x131 = (real_T*) ssGetOutputPortSignal(S,130);
	real_T *x132 = (real_T*) ssGetOutputPortSignal(S,131);
	real_T *x133 = (real_T*) ssGetOutputPortSignal(S,132);
	real_T *x134 = (real_T*) ssGetOutputPortSignal(S,133);
	real_T *x135 = (real_T*) ssGetOutputPortSignal(S,134);
	real_T *x136 = (real_T*) ssGetOutputPortSignal(S,135);
	real_T *x137 = (real_T*) ssGetOutputPortSignal(S,136);
	real_T *x138 = (real_T*) ssGetOutputPortSignal(S,137);
	real_T *x139 = (real_T*) ssGetOutputPortSignal(S,138);
	real_T *x140 = (real_T*) ssGetOutputPortSignal(S,139);
	real_T *x141 = (real_T*) ssGetOutputPortSignal(S,140);
	real_T *x142 = (real_T*) ssGetOutputPortSignal(S,141);
	real_T *x143 = (real_T*) ssGetOutputPortSignal(S,142);
	real_T *x144 = (real_T*) ssGetOutputPortSignal(S,143);
	real_T *x145 = (real_T*) ssGetOutputPortSignal(S,144);
	real_T *x146 = (real_T*) ssGetOutputPortSignal(S,145);
	real_T *x147 = (real_T*) ssGetOutputPortSignal(S,146);
	real_T *x148 = (real_T*) ssGetOutputPortSignal(S,147);
	real_T *x149 = (real_T*) ssGetOutputPortSignal(S,148);
	real_T *x150 = (real_T*) ssGetOutputPortSignal(S,149);
	real_T *x151 = (real_T*) ssGetOutputPortSignal(S,150);
	real_T *x152 = (real_T*) ssGetOutputPortSignal(S,151);
	real_T *x153 = (real_T*) ssGetOutputPortSignal(S,152);
	real_T *x154 = (real_T*) ssGetOutputPortSignal(S,153);
	real_T *x155 = (real_T*) ssGetOutputPortSignal(S,154);
	real_T *x156 = (real_T*) ssGetOutputPortSignal(S,155);
	real_T *x157 = (real_T*) ssGetOutputPortSignal(S,156);
	real_T *x158 = (real_T*) ssGetOutputPortSignal(S,157);
	real_T *x159 = (real_T*) ssGetOutputPortSignal(S,158);
	real_T *x160 = (real_T*) ssGetOutputPortSignal(S,159);
	real_T *x161 = (real_T*) ssGetOutputPortSignal(S,160);
	
	

	/* Solver data */
	FORCESNLPsolver_params params;
	FORCESNLPsolver_output output;
	FORCESNLPsolver_info info;	
	int exitflag;

	/* Extra NMPC data */
	

	/* Copy inputs */
	for( i=0; i<3220; i++){ params.x0[i] = (double) x0[i]; }
	for( i=0; i<15; i++){ params.xinit[i] = (double) xinit[i]; }
	for( i=0; i<3220; i++){ params.all_parameters[i] = (double) all_parameters[i]; }
	

	

    #if FORCESNLPsolver_SET_PRINTLEVEL > 0
		/* Prepare file for printfs */
        fp = fopen("stdout_temp","w+");
		if( fp == NULL ) {
			mexErrMsgTxt("freopen of stdout did not work.");
		}
		rewind(fp);
	#endif

	/* Call solver */
	exitflag = FORCESNLPsolver_solve(&params, &output, &info, fp ,pt2Function);

	#if FORCESNLPsolver_SET_PRINTLEVEL > 0
		/* Read contents of printfs printed to file */
		rewind(fp);
		while( (i = fgetc(fp)) != EOF ) {
			ssPrintf("%c",i);
		}
		fclose(fp);
	#endif

	

	/* Copy outputs */
	for( i=0; i<20; i++){ x001[i] = (real_T) output.x001[i]; }
	for( i=0; i<20; i++){ x002[i] = (real_T) output.x002[i]; }
	for( i=0; i<20; i++){ x003[i] = (real_T) output.x003[i]; }
	for( i=0; i<20; i++){ x004[i] = (real_T) output.x004[i]; }
	for( i=0; i<20; i++){ x005[i] = (real_T) output.x005[i]; }
	for( i=0; i<20; i++){ x006[i] = (real_T) output.x006[i]; }
	for( i=0; i<20; i++){ x007[i] = (real_T) output.x007[i]; }
	for( i=0; i<20; i++){ x008[i] = (real_T) output.x008[i]; }
	for( i=0; i<20; i++){ x009[i] = (real_T) output.x009[i]; }
	for( i=0; i<20; i++){ x010[i] = (real_T) output.x010[i]; }
	for( i=0; i<20; i++){ x011[i] = (real_T) output.x011[i]; }
	for( i=0; i<20; i++){ x012[i] = (real_T) output.x012[i]; }
	for( i=0; i<20; i++){ x013[i] = (real_T) output.x013[i]; }
	for( i=0; i<20; i++){ x014[i] = (real_T) output.x014[i]; }
	for( i=0; i<20; i++){ x015[i] = (real_T) output.x015[i]; }
	for( i=0; i<20; i++){ x016[i] = (real_T) output.x016[i]; }
	for( i=0; i<20; i++){ x017[i] = (real_T) output.x017[i]; }
	for( i=0; i<20; i++){ x018[i] = (real_T) output.x018[i]; }
	for( i=0; i<20; i++){ x019[i] = (real_T) output.x019[i]; }
	for( i=0; i<20; i++){ x020[i] = (real_T) output.x020[i]; }
	for( i=0; i<20; i++){ x021[i] = (real_T) output.x021[i]; }
	for( i=0; i<20; i++){ x022[i] = (real_T) output.x022[i]; }
	for( i=0; i<20; i++){ x023[i] = (real_T) output.x023[i]; }
	for( i=0; i<20; i++){ x024[i] = (real_T) output.x024[i]; }
	for( i=0; i<20; i++){ x025[i] = (real_T) output.x025[i]; }
	for( i=0; i<20; i++){ x026[i] = (real_T) output.x026[i]; }
	for( i=0; i<20; i++){ x027[i] = (real_T) output.x027[i]; }
	for( i=0; i<20; i++){ x028[i] = (real_T) output.x028[i]; }
	for( i=0; i<20; i++){ x029[i] = (real_T) output.x029[i]; }
	for( i=0; i<20; i++){ x030[i] = (real_T) output.x030[i]; }
	for( i=0; i<20; i++){ x031[i] = (real_T) output.x031[i]; }
	for( i=0; i<20; i++){ x032[i] = (real_T) output.x032[i]; }
	for( i=0; i<20; i++){ x033[i] = (real_T) output.x033[i]; }
	for( i=0; i<20; i++){ x034[i] = (real_T) output.x034[i]; }
	for( i=0; i<20; i++){ x035[i] = (real_T) output.x035[i]; }
	for( i=0; i<20; i++){ x036[i] = (real_T) output.x036[i]; }
	for( i=0; i<20; i++){ x037[i] = (real_T) output.x037[i]; }
	for( i=0; i<20; i++){ x038[i] = (real_T) output.x038[i]; }
	for( i=0; i<20; i++){ x039[i] = (real_T) output.x039[i]; }
	for( i=0; i<20; i++){ x040[i] = (real_T) output.x040[i]; }
	for( i=0; i<20; i++){ x041[i] = (real_T) output.x041[i]; }
	for( i=0; i<20; i++){ x042[i] = (real_T) output.x042[i]; }
	for( i=0; i<20; i++){ x043[i] = (real_T) output.x043[i]; }
	for( i=0; i<20; i++){ x044[i] = (real_T) output.x044[i]; }
	for( i=0; i<20; i++){ x045[i] = (real_T) output.x045[i]; }
	for( i=0; i<20; i++){ x046[i] = (real_T) output.x046[i]; }
	for( i=0; i<20; i++){ x047[i] = (real_T) output.x047[i]; }
	for( i=0; i<20; i++){ x048[i] = (real_T) output.x048[i]; }
	for( i=0; i<20; i++){ x049[i] = (real_T) output.x049[i]; }
	for( i=0; i<20; i++){ x050[i] = (real_T) output.x050[i]; }
	for( i=0; i<20; i++){ x051[i] = (real_T) output.x051[i]; }
	for( i=0; i<20; i++){ x052[i] = (real_T) output.x052[i]; }
	for( i=0; i<20; i++){ x053[i] = (real_T) output.x053[i]; }
	for( i=0; i<20; i++){ x054[i] = (real_T) output.x054[i]; }
	for( i=0; i<20; i++){ x055[i] = (real_T) output.x055[i]; }
	for( i=0; i<20; i++){ x056[i] = (real_T) output.x056[i]; }
	for( i=0; i<20; i++){ x057[i] = (real_T) output.x057[i]; }
	for( i=0; i<20; i++){ x058[i] = (real_T) output.x058[i]; }
	for( i=0; i<20; i++){ x059[i] = (real_T) output.x059[i]; }
	for( i=0; i<20; i++){ x060[i] = (real_T) output.x060[i]; }
	for( i=0; i<20; i++){ x061[i] = (real_T) output.x061[i]; }
	for( i=0; i<20; i++){ x062[i] = (real_T) output.x062[i]; }
	for( i=0; i<20; i++){ x063[i] = (real_T) output.x063[i]; }
	for( i=0; i<20; i++){ x064[i] = (real_T) output.x064[i]; }
	for( i=0; i<20; i++){ x065[i] = (real_T) output.x065[i]; }
	for( i=0; i<20; i++){ x066[i] = (real_T) output.x066[i]; }
	for( i=0; i<20; i++){ x067[i] = (real_T) output.x067[i]; }
	for( i=0; i<20; i++){ x068[i] = (real_T) output.x068[i]; }
	for( i=0; i<20; i++){ x069[i] = (real_T) output.x069[i]; }
	for( i=0; i<20; i++){ x070[i] = (real_T) output.x070[i]; }
	for( i=0; i<20; i++){ x071[i] = (real_T) output.x071[i]; }
	for( i=0; i<20; i++){ x072[i] = (real_T) output.x072[i]; }
	for( i=0; i<20; i++){ x073[i] = (real_T) output.x073[i]; }
	for( i=0; i<20; i++){ x074[i] = (real_T) output.x074[i]; }
	for( i=0; i<20; i++){ x075[i] = (real_T) output.x075[i]; }
	for( i=0; i<20; i++){ x076[i] = (real_T) output.x076[i]; }
	for( i=0; i<20; i++){ x077[i] = (real_T) output.x077[i]; }
	for( i=0; i<20; i++){ x078[i] = (real_T) output.x078[i]; }
	for( i=0; i<20; i++){ x079[i] = (real_T) output.x079[i]; }
	for( i=0; i<20; i++){ x080[i] = (real_T) output.x080[i]; }
	for( i=0; i<20; i++){ x081[i] = (real_T) output.x081[i]; }
	for( i=0; i<20; i++){ x082[i] = (real_T) output.x082[i]; }
	for( i=0; i<20; i++){ x083[i] = (real_T) output.x083[i]; }
	for( i=0; i<20; i++){ x084[i] = (real_T) output.x084[i]; }
	for( i=0; i<20; i++){ x085[i] = (real_T) output.x085[i]; }
	for( i=0; i<20; i++){ x086[i] = (real_T) output.x086[i]; }
	for( i=0; i<20; i++){ x087[i] = (real_T) output.x087[i]; }
	for( i=0; i<20; i++){ x088[i] = (real_T) output.x088[i]; }
	for( i=0; i<20; i++){ x089[i] = (real_T) output.x089[i]; }
	for( i=0; i<20; i++){ x090[i] = (real_T) output.x090[i]; }
	for( i=0; i<20; i++){ x091[i] = (real_T) output.x091[i]; }
	for( i=0; i<20; i++){ x092[i] = (real_T) output.x092[i]; }
	for( i=0; i<20; i++){ x093[i] = (real_T) output.x093[i]; }
	for( i=0; i<20; i++){ x094[i] = (real_T) output.x094[i]; }
	for( i=0; i<20; i++){ x095[i] = (real_T) output.x095[i]; }
	for( i=0; i<20; i++){ x096[i] = (real_T) output.x096[i]; }
	for( i=0; i<20; i++){ x097[i] = (real_T) output.x097[i]; }
	for( i=0; i<20; i++){ x098[i] = (real_T) output.x098[i]; }
	for( i=0; i<20; i++){ x099[i] = (real_T) output.x099[i]; }
	for( i=0; i<20; i++){ x100[i] = (real_T) output.x100[i]; }
	for( i=0; i<20; i++){ x101[i] = (real_T) output.x101[i]; }
	for( i=0; i<20; i++){ x102[i] = (real_T) output.x102[i]; }
	for( i=0; i<20; i++){ x103[i] = (real_T) output.x103[i]; }
	for( i=0; i<20; i++){ x104[i] = (real_T) output.x104[i]; }
	for( i=0; i<20; i++){ x105[i] = (real_T) output.x105[i]; }
	for( i=0; i<20; i++){ x106[i] = (real_T) output.x106[i]; }
	for( i=0; i<20; i++){ x107[i] = (real_T) output.x107[i]; }
	for( i=0; i<20; i++){ x108[i] = (real_T) output.x108[i]; }
	for( i=0; i<20; i++){ x109[i] = (real_T) output.x109[i]; }
	for( i=0; i<20; i++){ x110[i] = (real_T) output.x110[i]; }
	for( i=0; i<20; i++){ x111[i] = (real_T) output.x111[i]; }
	for( i=0; i<20; i++){ x112[i] = (real_T) output.x112[i]; }
	for( i=0; i<20; i++){ x113[i] = (real_T) output.x113[i]; }
	for( i=0; i<20; i++){ x114[i] = (real_T) output.x114[i]; }
	for( i=0; i<20; i++){ x115[i] = (real_T) output.x115[i]; }
	for( i=0; i<20; i++){ x116[i] = (real_T) output.x116[i]; }
	for( i=0; i<20; i++){ x117[i] = (real_T) output.x117[i]; }
	for( i=0; i<20; i++){ x118[i] = (real_T) output.x118[i]; }
	for( i=0; i<20; i++){ x119[i] = (real_T) output.x119[i]; }
	for( i=0; i<20; i++){ x120[i] = (real_T) output.x120[i]; }
	for( i=0; i<20; i++){ x121[i] = (real_T) output.x121[i]; }
	for( i=0; i<20; i++){ x122[i] = (real_T) output.x122[i]; }
	for( i=0; i<20; i++){ x123[i] = (real_T) output.x123[i]; }
	for( i=0; i<20; i++){ x124[i] = (real_T) output.x124[i]; }
	for( i=0; i<20; i++){ x125[i] = (real_T) output.x125[i]; }
	for( i=0; i<20; i++){ x126[i] = (real_T) output.x126[i]; }
	for( i=0; i<20; i++){ x127[i] = (real_T) output.x127[i]; }
	for( i=0; i<20; i++){ x128[i] = (real_T) output.x128[i]; }
	for( i=0; i<20; i++){ x129[i] = (real_T) output.x129[i]; }
	for( i=0; i<20; i++){ x130[i] = (real_T) output.x130[i]; }
	for( i=0; i<20; i++){ x131[i] = (real_T) output.x131[i]; }
	for( i=0; i<20; i++){ x132[i] = (real_T) output.x132[i]; }
	for( i=0; i<20; i++){ x133[i] = (real_T) output.x133[i]; }
	for( i=0; i<20; i++){ x134[i] = (real_T) output.x134[i]; }
	for( i=0; i<20; i++){ x135[i] = (real_T) output.x135[i]; }
	for( i=0; i<20; i++){ x136[i] = (real_T) output.x136[i]; }
	for( i=0; i<20; i++){ x137[i] = (real_T) output.x137[i]; }
	for( i=0; i<20; i++){ x138[i] = (real_T) output.x138[i]; }
	for( i=0; i<20; i++){ x139[i] = (real_T) output.x139[i]; }
	for( i=0; i<20; i++){ x140[i] = (real_T) output.x140[i]; }
	for( i=0; i<20; i++){ x141[i] = (real_T) output.x141[i]; }
	for( i=0; i<20; i++){ x142[i] = (real_T) output.x142[i]; }
	for( i=0; i<20; i++){ x143[i] = (real_T) output.x143[i]; }
	for( i=0; i<20; i++){ x144[i] = (real_T) output.x144[i]; }
	for( i=0; i<20; i++){ x145[i] = (real_T) output.x145[i]; }
	for( i=0; i<20; i++){ x146[i] = (real_T) output.x146[i]; }
	for( i=0; i<20; i++){ x147[i] = (real_T) output.x147[i]; }
	for( i=0; i<20; i++){ x148[i] = (real_T) output.x148[i]; }
	for( i=0; i<20; i++){ x149[i] = (real_T) output.x149[i]; }
	for( i=0; i<20; i++){ x150[i] = (real_T) output.x150[i]; }
	for( i=0; i<20; i++){ x151[i] = (real_T) output.x151[i]; }
	for( i=0; i<20; i++){ x152[i] = (real_T) output.x152[i]; }
	for( i=0; i<20; i++){ x153[i] = (real_T) output.x153[i]; }
	for( i=0; i<20; i++){ x154[i] = (real_T) output.x154[i]; }
	for( i=0; i<20; i++){ x155[i] = (real_T) output.x155[i]; }
	for( i=0; i<20; i++){ x156[i] = (real_T) output.x156[i]; }
	for( i=0; i<20; i++){ x157[i] = (real_T) output.x157[i]; }
	for( i=0; i<20; i++){ x158[i] = (real_T) output.x158[i]; }
	for( i=0; i<20; i++){ x159[i] = (real_T) output.x159[i]; }
	for( i=0; i<20; i++){ x160[i] = (real_T) output.x160[i]; }
	for( i=0; i<20; i++){ x161[i] = (real_T) output.x161[i]; }
	
}





/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
}
#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif


