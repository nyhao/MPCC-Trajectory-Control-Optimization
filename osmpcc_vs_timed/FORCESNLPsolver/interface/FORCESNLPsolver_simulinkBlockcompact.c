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
#define S_FUNCTION_NAME FORCESNLPsolver_simulinkBlockcompact

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

	/* initialize input ports - there are 4 in total */
    if (!ssSetNumInputPorts(S, 4)) return;
    	
	/* Input Port 0 */
    ssSetInputPortMatrixDimensions(S,  0, 5313, 1);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 0, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/	
	/* Input Port 1 */
    ssSetInputPortMatrixDimensions(S,  1, 26, 1);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 1, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 1, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 1, 1); /*direct input signal access*/	
	/* Input Port 2 */
    ssSetInputPortMatrixDimensions(S,  2, 6, 1);
    ssSetInputPortDataType(S, 2, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 2, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 2, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 2, 1); /*direct input signal access*/	
	/* Input Port 3 */
    ssSetInputPortMatrixDimensions(S,  3, 6440, 1);
    ssSetInputPortDataType(S, 3, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 3, COMPLEX_NO); /* no complex signals suppported */
    ssSetInputPortDirectFeedThrough(S, 3, 1); /* Feedthrough enabled */
    ssSetInputPortRequiredContiguous(S, 3, 1); /*direct input signal access*/ 


	/* initialize output ports - there are 1 in total */
    if (!ssSetNumOutputPorts(S, 1)) return;    
		
	/* Output Port 0 */
    ssSetOutputPortMatrixDimensions(S,  0, 5313, 1);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 0, COMPLEX_NO); /* no complex signals suppported */


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
	const real_T *xfinal = (const real_T*) ssGetInputPortSignal(S,2);
	const real_T *all_parameters = (const real_T*) ssGetInputPortSignal(S,3);
	
    real_T *outputs = (real_T*) ssGetOutputPortSignal(S,0);
	
	

	/* Solver data */
	FORCESNLPsolver_params params;
	FORCESNLPsolver_output output;
	FORCESNLPsolver_info info;	
	int exitflag;

	/* Extra NMPC data */
	

	/* Copy inputs */
	for( i=0; i<5313; i++){ params.x0[i] = (double) x0[i]; }
	for( i=0; i<26; i++){ params.xinit[i] = (double) xinit[i]; }
	for( i=0; i<6; i++){ params.xfinal[i] = (double) xfinal[i]; }
	for( i=0; i<6440; i++){ params.all_parameters[i] = (double) all_parameters[i]; }
	

	

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
	for( i=0; i<33; i++){ outputs[i] = (real_T) output.x001[i]; }
	k=33; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x002[i]; }
	k=66; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x003[i]; }
	k=99; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x004[i]; }
	k=132; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x005[i]; }
	k=165; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x006[i]; }
	k=198; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x007[i]; }
	k=231; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x008[i]; }
	k=264; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x009[i]; }
	k=297; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x010[i]; }
	k=330; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x011[i]; }
	k=363; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x012[i]; }
	k=396; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x013[i]; }
	k=429; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x014[i]; }
	k=462; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x015[i]; }
	k=495; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x016[i]; }
	k=528; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x017[i]; }
	k=561; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x018[i]; }
	k=594; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x019[i]; }
	k=627; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x020[i]; }
	k=660; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x021[i]; }
	k=693; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x022[i]; }
	k=726; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x023[i]; }
	k=759; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x024[i]; }
	k=792; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x025[i]; }
	k=825; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x026[i]; }
	k=858; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x027[i]; }
	k=891; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x028[i]; }
	k=924; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x029[i]; }
	k=957; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x030[i]; }
	k=990; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x031[i]; }
	k=1023; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x032[i]; }
	k=1056; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x033[i]; }
	k=1089; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x034[i]; }
	k=1122; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x035[i]; }
	k=1155; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x036[i]; }
	k=1188; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x037[i]; }
	k=1221; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x038[i]; }
	k=1254; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x039[i]; }
	k=1287; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x040[i]; }
	k=1320; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x041[i]; }
	k=1353; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x042[i]; }
	k=1386; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x043[i]; }
	k=1419; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x044[i]; }
	k=1452; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x045[i]; }
	k=1485; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x046[i]; }
	k=1518; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x047[i]; }
	k=1551; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x048[i]; }
	k=1584; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x049[i]; }
	k=1617; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x050[i]; }
	k=1650; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x051[i]; }
	k=1683; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x052[i]; }
	k=1716; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x053[i]; }
	k=1749; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x054[i]; }
	k=1782; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x055[i]; }
	k=1815; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x056[i]; }
	k=1848; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x057[i]; }
	k=1881; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x058[i]; }
	k=1914; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x059[i]; }
	k=1947; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x060[i]; }
	k=1980; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x061[i]; }
	k=2013; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x062[i]; }
	k=2046; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x063[i]; }
	k=2079; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x064[i]; }
	k=2112; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x065[i]; }
	k=2145; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x066[i]; }
	k=2178; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x067[i]; }
	k=2211; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x068[i]; }
	k=2244; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x069[i]; }
	k=2277; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x070[i]; }
	k=2310; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x071[i]; }
	k=2343; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x072[i]; }
	k=2376; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x073[i]; }
	k=2409; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x074[i]; }
	k=2442; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x075[i]; }
	k=2475; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x076[i]; }
	k=2508; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x077[i]; }
	k=2541; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x078[i]; }
	k=2574; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x079[i]; }
	k=2607; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x080[i]; }
	k=2640; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x081[i]; }
	k=2673; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x082[i]; }
	k=2706; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x083[i]; }
	k=2739; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x084[i]; }
	k=2772; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x085[i]; }
	k=2805; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x086[i]; }
	k=2838; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x087[i]; }
	k=2871; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x088[i]; }
	k=2904; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x089[i]; }
	k=2937; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x090[i]; }
	k=2970; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x091[i]; }
	k=3003; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x092[i]; }
	k=3036; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x093[i]; }
	k=3069; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x094[i]; }
	k=3102; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x095[i]; }
	k=3135; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x096[i]; }
	k=3168; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x097[i]; }
	k=3201; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x098[i]; }
	k=3234; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x099[i]; }
	k=3267; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x100[i]; }
	k=3300; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x101[i]; }
	k=3333; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x102[i]; }
	k=3366; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x103[i]; }
	k=3399; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x104[i]; }
	k=3432; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x105[i]; }
	k=3465; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x106[i]; }
	k=3498; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x107[i]; }
	k=3531; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x108[i]; }
	k=3564; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x109[i]; }
	k=3597; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x110[i]; }
	k=3630; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x111[i]; }
	k=3663; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x112[i]; }
	k=3696; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x113[i]; }
	k=3729; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x114[i]; }
	k=3762; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x115[i]; }
	k=3795; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x116[i]; }
	k=3828; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x117[i]; }
	k=3861; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x118[i]; }
	k=3894; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x119[i]; }
	k=3927; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x120[i]; }
	k=3960; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x121[i]; }
	k=3993; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x122[i]; }
	k=4026; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x123[i]; }
	k=4059; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x124[i]; }
	k=4092; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x125[i]; }
	k=4125; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x126[i]; }
	k=4158; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x127[i]; }
	k=4191; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x128[i]; }
	k=4224; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x129[i]; }
	k=4257; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x130[i]; }
	k=4290; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x131[i]; }
	k=4323; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x132[i]; }
	k=4356; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x133[i]; }
	k=4389; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x134[i]; }
	k=4422; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x135[i]; }
	k=4455; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x136[i]; }
	k=4488; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x137[i]; }
	k=4521; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x138[i]; }
	k=4554; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x139[i]; }
	k=4587; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x140[i]; }
	k=4620; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x141[i]; }
	k=4653; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x142[i]; }
	k=4686; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x143[i]; }
	k=4719; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x144[i]; }
	k=4752; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x145[i]; }
	k=4785; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x146[i]; }
	k=4818; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x147[i]; }
	k=4851; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x148[i]; }
	k=4884; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x149[i]; }
	k=4917; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x150[i]; }
	k=4950; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x151[i]; }
	k=4983; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x152[i]; }
	k=5016; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x153[i]; }
	k=5049; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x154[i]; }
	k=5082; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x155[i]; }
	k=5115; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x156[i]; }
	k=5148; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x157[i]; }
	k=5181; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x158[i]; }
	k=5214; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x159[i]; }
	k=5247; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x160[i]; }
	k=5280; for( i=0; i<33; i++){ outputs[k++] = (real_T) output.x161[i]; }
	
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


