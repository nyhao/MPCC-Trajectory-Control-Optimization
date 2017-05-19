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

#include <stdio.h>

#ifndef __FORCESNLPsolver_H__
#define __FORCESNLPsolver_H__

/* DATA TYPE ------------------------------------------------------------*/
typedef double FORCESNLPsolver_FLOAT;

typedef double FORCESNLPsolverINTERFACE_FLOAT;

/* SOLVER SETTINGS ------------------------------------------------------*/
/* print level */
#ifndef FORCESNLPsolver_SET_PRINTLEVEL
#define FORCESNLPsolver_SET_PRINTLEVEL    (2)
#endif

/* timing */
#ifndef FORCESNLPsolver_SET_TIMING
#define FORCESNLPsolver_SET_TIMING    (1)
#endif

/* Numeric Warnings */
/* #define PRINTNUMERICALWARNINGS */

/* maximum number of iterations  */
#define FORCESNLPsolver_SET_MAXIT			(5000)	

/* scaling factor of line search (FTB rule) */
#define FORCESNLPsolver_SET_FLS_SCALE		(FORCESNLPsolver_FLOAT)(0.99)      

/* maximum number of supported elements in the filter */
#define FORCESNLPsolver_MAX_FILTER_SIZE	(5000) 

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
    /* vector of size 4186 */
    FORCESNLPsolver_FLOAT x0[4186];

    /* vector of size 19 */
    FORCESNLPsolver_FLOAT xinit[19];

    /* vector of size 4186 */
    FORCESNLPsolver_FLOAT all_parameters[4186];

} FORCESNLPsolver_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct FORCESNLPsolver_output
{
    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x001[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x002[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x003[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x004[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x005[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x006[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x007[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x008[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x009[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x010[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x011[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x012[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x013[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x014[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x015[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x016[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x017[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x018[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x019[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x020[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x021[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x022[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x023[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x024[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x025[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x026[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x027[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x028[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x029[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x030[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x031[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x032[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x033[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x034[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x035[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x036[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x037[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x038[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x039[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x040[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x041[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x042[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x043[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x044[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x045[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x046[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x047[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x048[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x049[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x050[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x051[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x052[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x053[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x054[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x055[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x056[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x057[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x058[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x059[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x060[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x061[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x062[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x063[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x064[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x065[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x066[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x067[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x068[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x069[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x070[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x071[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x072[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x073[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x074[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x075[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x076[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x077[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x078[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x079[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x080[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x081[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x082[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x083[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x084[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x085[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x086[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x087[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x088[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x089[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x090[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x091[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x092[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x093[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x094[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x095[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x096[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x097[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x098[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x099[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x100[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x101[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x102[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x103[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x104[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x105[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x106[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x107[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x108[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x109[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x110[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x111[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x112[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x113[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x114[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x115[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x116[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x117[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x118[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x119[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x120[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x121[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x122[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x123[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x124[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x125[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x126[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x127[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x128[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x129[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x130[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x131[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x132[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x133[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x134[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x135[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x136[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x137[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x138[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x139[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x140[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x141[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x142[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x143[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x144[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x145[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x146[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x147[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x148[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x149[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x150[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x151[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x152[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x153[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x154[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x155[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x156[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x157[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x158[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x159[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x160[26];

    /* vector of size 26 */
    FORCESNLPsolver_FLOAT x161[26];

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