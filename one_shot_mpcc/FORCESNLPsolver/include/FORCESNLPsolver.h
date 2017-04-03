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
    /* vector of size 3220 */
    FORCESNLPsolver_FLOAT x0[3220];

    /* vector of size 15 */
    FORCESNLPsolver_FLOAT xinit[15];

    /* vector of size 2898 */
    FORCESNLPsolver_FLOAT all_parameters[2898];

} FORCESNLPsolver_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct FORCESNLPsolver_output
{
    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x001[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x002[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x003[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x004[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x005[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x006[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x007[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x008[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x009[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x010[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x011[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x012[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x013[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x014[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x015[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x016[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x017[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x018[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x019[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x020[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x021[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x022[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x023[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x024[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x025[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x026[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x027[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x028[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x029[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x030[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x031[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x032[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x033[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x034[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x035[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x036[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x037[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x038[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x039[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x040[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x041[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x042[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x043[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x044[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x045[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x046[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x047[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x048[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x049[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x050[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x051[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x052[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x053[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x054[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x055[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x056[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x057[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x058[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x059[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x060[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x061[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x062[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x063[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x064[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x065[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x066[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x067[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x068[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x069[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x070[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x071[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x072[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x073[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x074[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x075[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x076[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x077[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x078[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x079[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x080[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x081[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x082[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x083[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x084[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x085[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x086[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x087[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x088[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x089[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x090[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x091[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x092[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x093[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x094[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x095[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x096[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x097[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x098[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x099[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x100[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x101[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x102[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x103[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x104[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x105[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x106[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x107[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x108[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x109[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x110[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x111[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x112[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x113[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x114[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x115[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x116[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x117[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x118[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x119[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x120[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x121[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x122[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x123[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x124[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x125[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x126[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x127[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x128[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x129[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x130[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x131[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x132[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x133[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x134[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x135[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x136[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x137[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x138[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x139[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x140[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x141[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x142[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x143[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x144[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x145[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x146[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x147[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x148[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x149[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x150[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x151[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x152[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x153[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x154[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x155[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x156[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x157[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x158[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x159[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x160[20];

    /* vector of size 20 */
    FORCESNLPsolver_FLOAT x161[20];

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