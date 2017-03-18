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

#include "mex.h"
#include "math.h"
#include "../include/FORCESNLPsolver.h"
#include <stdio.h>



/* copy functions */
void copyCArrayToM(double *src, double *dest, int dim) {
    while (dim--) {
        *dest++ = (double)*src++;
    }
}
void copyMArrayToC(double *src, double *dest, int dim) {
    while (dim--) {
        *dest++ = (double) (*src++) ;
    }
}


extern void FORCESNLPsolver_casadi2forces(FORCESNLPsolver_FLOAT *x, FORCESNLPsolver_FLOAT *y, FORCESNLPsolver_FLOAT *l, FORCESNLPsolver_FLOAT *p, FORCESNLPsolver_FLOAT *f, FORCESNLPsolver_FLOAT *nabla_f, FORCESNLPsolver_FLOAT *c, FORCESNLPsolver_FLOAT *nabla_c, FORCESNLPsolver_FLOAT *h, FORCESNLPsolver_FLOAT *nabla_h, FORCESNLPsolver_FLOAT *H, int stage);
FORCESNLPsolver_ExtFunc pt2Function = &FORCESNLPsolver_casadi2forces;


/* Some memory for mex-function */
FORCESNLPsolver_params params;
FORCESNLPsolver_output output;
FORCESNLPsolver_info info;

/* THE mex-function */
void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )  
{
	/* file pointer for printing */
	FILE *fp = NULL;

	/* define variables */	
	mxArray *par;
	mxArray *outvar;
	const mxArray *PARAMS = prhs[0];
	double *pvalue;
	int i;
	int exitflag;
	const char *fname;
	const char *outputnames[11] = {"x01","x02","x03","x04","x05","x06","x07","x08","x09","x10","x11"};
	const char *infofields[8] = { "it", "it2opt", "res_eq", "res_ineq",  "pobj",  "mu",  "solvetime",  "fevalstime"};
	
	/* Check for proper number of arguments */
    if (nrhs != 1) {
        mexErrMsgTxt("This function requires exactly 1 input: PARAMS struct.\nType 'help FORCESNLPsolver_mex' for details.");
    }    
	if (nlhs > 3) {
        mexErrMsgTxt("This function returns at most 3 outputs.\nType 'help FORCESNLPsolver_mex' for details.");
    }

	/* Check whether params is actually a structure */
	if( !mxIsStruct(PARAMS) ) {
		mexErrMsgTxt("PARAMS must be a structure.");
	}

	/* copy parameters into the right location */
	par = mxGetField(PARAMS, 0, "x0");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.x0 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.x0 must be a double.");
    }
    if( mxGetM(par) != 308 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.x0 must be of size [308 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.x0, 308);

	par = mxGetField(PARAMS, 0, "xinit");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.xinit not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.xinit must be a double.");
    }
    if( mxGetM(par) != 21 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.xinit must be of size [21 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.xinit, 21);

	par = mxGetField(PARAMS, 0, "all_parameters");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.all_parameters not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.all_parameters must be a double.");
    }
    if( mxGetM(par) != 165 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.all_parameters must be of size [165 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.all_parameters, 165);

	#if FORCESNLPsolver_SET_PRINTLEVEL > 0
		/* Prepare file for printfs */
		/*fp = freopen("stdout_temp","w+",stdout);*/
        fp = fopen("stdout_temp","w+");
		if( fp == NULL ) {
			mexErrMsgTxt("freopen of stdout did not work.");
		}
		rewind(fp);
	#endif

	/* call solver */
	exitflag = FORCESNLPsolver_solve(&params, &output, &info, fp ,pt2Function);

	/* close stdout */
	/* fclose(fp); */
	
	#if FORCESNLPsolver_SET_PRINTLEVEL > 0
		/* Read contents of printfs printed to file */
		rewind(fp);
		while( (i = fgetc(fp)) != EOF ) {
			mexPrintf("%c",i);
		}
		fclose(fp);
	#endif

	/* copy output to matlab arrays */
	plhs[0] = mxCreateStructMatrix(1, 1, 11, outputnames);
	outvar = mxCreateDoubleMatrix(28, 1, mxREAL);
	copyCArrayToM( output.x01, mxGetPr(outvar), 28);
	mxSetField(plhs[0], 0, "x01", outvar);

	outvar = mxCreateDoubleMatrix(28, 1, mxREAL);
	copyCArrayToM( output.x02, mxGetPr(outvar), 28);
	mxSetField(plhs[0], 0, "x02", outvar);

	outvar = mxCreateDoubleMatrix(28, 1, mxREAL);
	copyCArrayToM( output.x03, mxGetPr(outvar), 28);
	mxSetField(plhs[0], 0, "x03", outvar);

	outvar = mxCreateDoubleMatrix(28, 1, mxREAL);
	copyCArrayToM( output.x04, mxGetPr(outvar), 28);
	mxSetField(plhs[0], 0, "x04", outvar);

	outvar = mxCreateDoubleMatrix(28, 1, mxREAL);
	copyCArrayToM( output.x05, mxGetPr(outvar), 28);
	mxSetField(plhs[0], 0, "x05", outvar);

	outvar = mxCreateDoubleMatrix(28, 1, mxREAL);
	copyCArrayToM( output.x06, mxGetPr(outvar), 28);
	mxSetField(plhs[0], 0, "x06", outvar);

	outvar = mxCreateDoubleMatrix(28, 1, mxREAL);
	copyCArrayToM( output.x07, mxGetPr(outvar), 28);
	mxSetField(plhs[0], 0, "x07", outvar);

	outvar = mxCreateDoubleMatrix(28, 1, mxREAL);
	copyCArrayToM( output.x08, mxGetPr(outvar), 28);
	mxSetField(plhs[0], 0, "x08", outvar);

	outvar = mxCreateDoubleMatrix(28, 1, mxREAL);
	copyCArrayToM( output.x09, mxGetPr(outvar), 28);
	mxSetField(plhs[0], 0, "x09", outvar);

	outvar = mxCreateDoubleMatrix(28, 1, mxREAL);
	copyCArrayToM( output.x10, mxGetPr(outvar), 28);
	mxSetField(plhs[0], 0, "x10", outvar);

	outvar = mxCreateDoubleMatrix(28, 1, mxREAL);
	copyCArrayToM( output.x11, mxGetPr(outvar), 28);
	mxSetField(plhs[0], 0, "x11", outvar);	

	/* copy exitflag */
	if( nlhs > 1 )
	{
		plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(plhs[1]) = (double)exitflag;
	}

	/* copy info struct */
	if( nlhs > 2 )
	{
		        plhs[2] = mxCreateStructMatrix(1, 1, 8, infofields);
         
		
		/* iterations */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.it;
		mxSetField(plhs[2], 0, "it", outvar);

		/* iterations to optimality (branch and bound) */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.it2opt;
		mxSetField(plhs[2], 0, "it2opt", outvar);
		
		/* res_eq */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.res_eq;
		mxSetField(plhs[2], 0, "res_eq", outvar);

		/* res_ineq */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.res_ineq;
		mxSetField(plhs[2], 0, "res_ineq", outvar);

		/* pobj */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.pobj;
		mxSetField(plhs[2], 0, "pobj", outvar);

		/* mu */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.mu;
		mxSetField(plhs[2], 0, "mu", outvar);

		/* solver time */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.solvetime;
		mxSetField(plhs[2], 0, "solvetime", outvar);

		/* solver time */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.fevalstime;
		mxSetField(plhs[2], 0, "fevalstime", outvar);
	}
}