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

/* Generated by FORCES PRO v1.6.100 on Tuesday, May 30, 2017 at 4:39:42 PM */

#include <stdio.h>

#ifndef __FORCESNLPsolver_H__
#define __FORCESNLPsolver_H__

/* DATA TYPE ------------------------------------------------------------*/
typedef double FORCESNLPsolver_FLOAT;

typedef double FORCESNLPsolverINTERFACE_FLOAT;

/* SOLVER SETTINGS ------------------------------------------------------*/
/* print level */
#ifndef FORCESNLPsolver_SET_PRINTLEVEL
#define FORCESNLPsolver_SET_PRINTLEVEL    (0)
#endif

/* timing */
#ifndef FORCESNLPsolver_SET_TIMING
#define FORCESNLPsolver_SET_TIMING    (1)
#endif

/* Numeric Warnings */
/* #define PRINTNUMERICALWARNINGS */

/* maximum number of iterations  */
#define FORCESNLPsolver_SET_MAXIT			(8000)	

/* scaling factor of line search (FTB rule) */
#define FORCESNLPsolver_SET_FLS_SCALE		(FORCESNLPsolver_FLOAT)(0.99)      

/* maximum number of supported elements in the filter */
#define FORCESNLPsolver_MAX_FILTER_SIZE	(8000) 

/* maximum number of supported elements in the filter */
#define FORCESNLPsolver_MAX_SOC_IT			(4) 

/* desired relative duality gap */
#define FORCESNLPsolver_SET_ACC_RDGAP		(FORCESNLPsolver_FLOAT)(0.0001)

/* desired maximum residual on equality constraints */
#define FORCESNLPsolver_SET_ACC_RESEQ		(FORCESNLPsolver_FLOAT)(1E-06)

/* desired maximum residual on inequality constraints */
#define FORCESNLPsolver_SET_ACC_RESINEQ	(FORCESNLPsolver_FLOAT)(1E-06)

/* desired maximum violation of complementarity */
#define FORCESNLPsolver_SET_ACC_KKTCOMPL	(FORCESNLPsolver_FLOAT)(1E-06)


/* RETURN CODES----------------------------------------------------------*/
/* solver has converged within desired accuracy */
#define FORCESNLPsolver_OPTIMAL      (1)

/* maximum number of iterations has been reached */
#define FORCESNLPsolver_MAXITREACHED (0)

/* NaN encountered in function evaluations */
#define FORCESNLPsolver_BADFUNCEVAL  (-6)

/* no progress in method possible */
#define FORCESNLPsolver_NOPROGRESS   (-7)



/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct FORCESNLPsolver_params
{
    /* vector of size 4991 */
    FORCESNLPsolver_FLOAT x0[4991];

    /* vector of size 24 */
    FORCESNLPsolver_FLOAT xinit[24];

    /* vector of size 5313 */
    FORCESNLPsolver_FLOAT all_parameters[5313];

} FORCESNLPsolver_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct FORCESNLPsolver_output
{
    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x001[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x002[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x003[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x004[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x005[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x006[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x007[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x008[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x009[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x010[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x011[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x012[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x013[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x014[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x015[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x016[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x017[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x018[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x019[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x020[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x021[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x022[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x023[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x024[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x025[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x026[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x027[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x028[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x029[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x030[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x031[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x032[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x033[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x034[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x035[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x036[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x037[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x038[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x039[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x040[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x041[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x042[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x043[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x044[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x045[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x046[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x047[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x048[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x049[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x050[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x051[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x052[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x053[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x054[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x055[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x056[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x057[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x058[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x059[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x060[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x061[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x062[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x063[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x064[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x065[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x066[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x067[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x068[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x069[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x070[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x071[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x072[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x073[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x074[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x075[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x076[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x077[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x078[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x079[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x080[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x081[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x082[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x083[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x084[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x085[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x086[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x087[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x088[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x089[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x090[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x091[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x092[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x093[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x094[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x095[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x096[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x097[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x098[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x099[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x100[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x101[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x102[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x103[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x104[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x105[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x106[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x107[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x108[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x109[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x110[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x111[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x112[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x113[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x114[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x115[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x116[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x117[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x118[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x119[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x120[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x121[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x122[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x123[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x124[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x125[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x126[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x127[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x128[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x129[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x130[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x131[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x132[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x133[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x134[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x135[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x136[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x137[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x138[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x139[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x140[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x141[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x142[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x143[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x144[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x145[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x146[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x147[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x148[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x149[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x150[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x151[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x152[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x153[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x154[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x155[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x156[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x157[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x158[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x159[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x160[31];

    /* vector of size 31 */
    FORCESNLPsolver_FLOAT x161[31];

} FORCESNLPsolver_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct FORCESNLPsolver_info
{
    /* iteration number */
    int it;

	/* number of iterations needed to optimality (branch-and-bound) */
	int it2opt;
	
    /* inf-norm of equality constraint residuals */
    FORCESNLPsolver_FLOAT res_eq;
	
    /* inf-norm of inequality constraint residuals */
    FORCESNLPsolver_FLOAT res_ineq;

    /* primal objective */
    FORCESNLPsolver_FLOAT pobj;	
	
    /* dual objective */
    FORCESNLPsolver_FLOAT dobj;	

    /* duality gap := pobj - dobj */
    FORCESNLPsolver_FLOAT dgap;		
	
    /* relative duality gap := |dgap / pobj | */
    FORCESNLPsolver_FLOAT rdgap;		

    /* duality measure */
    FORCESNLPsolver_FLOAT mu;

	/* duality measure (after affine step) */
    FORCESNLPsolver_FLOAT mu_aff;
	
    /* centering parameter */
    FORCESNLPsolver_FLOAT sigma;
	
    /* number of backtracking line search steps (affine direction) */
    int lsit_aff;
    
    /* number of backtracking line search steps (combined direction) */
    int lsit_cc;
    
    /* step size (affine direction) */
    FORCESNLPsolver_FLOAT step_aff;
    
    /* step size (combined direction) */
    FORCESNLPsolver_FLOAT step_cc;    

	/* solvertime */
	FORCESNLPsolver_FLOAT solvetime;   

	/* time spent in function evaluations */
	FORCESNLPsolver_FLOAT fevalstime;  

} FORCESNLPsolver_info;








/* SOLVER FUNCTION DEFINITION -------------------------------------------*/
/* examine exitflag before using the result! */
#ifdef __cplusplus
extern "C" {
#endif		

typedef void (*FORCESNLPsolver_ExtFunc)(FORCESNLPsolver_FLOAT*, FORCESNLPsolver_FLOAT*, FORCESNLPsolver_FLOAT*, FORCESNLPsolver_FLOAT*, FORCESNLPsolver_FLOAT*, FORCESNLPsolver_FLOAT*, FORCESNLPsolver_FLOAT*, FORCESNLPsolver_FLOAT*, FORCESNLPsolver_FLOAT*, FORCESNLPsolver_FLOAT*, FORCESNLPsolver_FLOAT*, int);

int FORCESNLPsolver_solve(FORCESNLPsolver_params* params, FORCESNLPsolver_output* output, FORCESNLPsolver_info* info, FILE* fs, FORCESNLPsolver_ExtFunc FORCESNLPsolver_evalExtFunctions);	


#ifdef __cplusplus
}
#endif

#endif