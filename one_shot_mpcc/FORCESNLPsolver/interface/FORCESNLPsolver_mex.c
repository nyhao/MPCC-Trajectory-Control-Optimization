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
	const char *outputnames[161] = {"x001","x002","x003","x004","x005","x006","x007","x008","x009","x010","x011","x012","x013","x014","x015","x016","x017","x018","x019","x020","x021","x022","x023","x024","x025","x026","x027","x028","x029","x030","x031","x032","x033","x034","x035","x036","x037","x038","x039","x040","x041","x042","x043","x044","x045","x046","x047","x048","x049","x050","x051","x052","x053","x054","x055","x056","x057","x058","x059","x060","x061","x062","x063","x064","x065","x066","x067","x068","x069","x070","x071","x072","x073","x074","x075","x076","x077","x078","x079","x080","x081","x082","x083","x084","x085","x086","x087","x088","x089","x090","x091","x092","x093","x094","x095","x096","x097","x098","x099","x100","x101","x102","x103","x104","x105","x106","x107","x108","x109","x110","x111","x112","x113","x114","x115","x116","x117","x118","x119","x120","x121","x122","x123","x124","x125","x126","x127","x128","x129","x130","x131","x132","x133","x134","x135","x136","x137","x138","x139","x140","x141","x142","x143","x144","x145","x146","x147","x148","x149","x150","x151","x152","x153","x154","x155","x156","x157","x158","x159","x160","x161"};
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
    if( mxGetM(par) != 3220 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.x0 must be of size [3220 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.x0, 3220);

	par = mxGetField(PARAMS, 0, "xinit");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.xinit not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.xinit must be a double.");
    }
    if( mxGetM(par) != 15 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.xinit must be of size [15 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.xinit, 15);

	par = mxGetField(PARAMS, 0, "all_parameters");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.all_parameters not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.all_parameters must be a double.");
    }
    if( mxGetM(par) != 3220 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.all_parameters must be of size [3220 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.all_parameters, 3220);

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
	plhs[0] = mxCreateStructMatrix(1, 1, 161, outputnames);
	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x001, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x001", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x002, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x002", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x003, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x003", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x004, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x004", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x005, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x005", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x006, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x006", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x007, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x007", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x008, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x008", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x009, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x009", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x010, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x010", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x011, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x011", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x012, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x012", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x013, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x013", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x014, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x014", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x015, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x015", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x016, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x016", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x017, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x017", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x018, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x018", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x019, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x019", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x020, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x020", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x021, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x021", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x022, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x022", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x023, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x023", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x024, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x024", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x025, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x025", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x026, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x026", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x027, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x027", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x028, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x028", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x029, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x029", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x030, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x030", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x031, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x031", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x032, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x032", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x033, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x033", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x034, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x034", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x035, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x035", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x036, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x036", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x037, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x037", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x038, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x038", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x039, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x039", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x040, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x040", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x041, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x041", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x042, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x042", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x043, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x043", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x044, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x044", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x045, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x045", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x046, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x046", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x047, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x047", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x048, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x048", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x049, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x049", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x050, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x050", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x051, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x051", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x052, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x052", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x053, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x053", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x054, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x054", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x055, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x055", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x056, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x056", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x057, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x057", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x058, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x058", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x059, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x059", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x060, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x060", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x061, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x061", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x062, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x062", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x063, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x063", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x064, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x064", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x065, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x065", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x066, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x066", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x067, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x067", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x068, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x068", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x069, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x069", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x070, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x070", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x071, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x071", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x072, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x072", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x073, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x073", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x074, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x074", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x075, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x075", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x076, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x076", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x077, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x077", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x078, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x078", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x079, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x079", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x080, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x080", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x081, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x081", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x082, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x082", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x083, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x083", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x084, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x084", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x085, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x085", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x086, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x086", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x087, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x087", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x088, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x088", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x089, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x089", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x090, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x090", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x091, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x091", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x092, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x092", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x093, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x093", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x094, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x094", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x095, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x095", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x096, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x096", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x097, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x097", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x098, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x098", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x099, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x099", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x100, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x100", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x101, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x101", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x102, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x102", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x103, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x103", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x104, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x104", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x105, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x105", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x106, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x106", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x107, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x107", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x108, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x108", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x109, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x109", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x110, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x110", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x111, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x111", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x112, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x112", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x113, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x113", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x114, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x114", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x115, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x115", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x116, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x116", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x117, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x117", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x118, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x118", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x119, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x119", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x120, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x120", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x121, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x121", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x122, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x122", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x123, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x123", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x124, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x124", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x125, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x125", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x126, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x126", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x127, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x127", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x128, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x128", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x129, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x129", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x130, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x130", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x131, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x131", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x132, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x132", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x133, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x133", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x134, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x134", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x135, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x135", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x136, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x136", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x137, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x137", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x138, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x138", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x139, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x139", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x140, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x140", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x141, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x141", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x142, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x142", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x143, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x143", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x144, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x144", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x145, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x145", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x146, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x146", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x147, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x147", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x148, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x148", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x149, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x149", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x150, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x150", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x151, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x151", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x152, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x152", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x153, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x153", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x154, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x154", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x155, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x155", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x156, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x156", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x157, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x157", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x158, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x158", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x159, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x159", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x160, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x160", outvar);

	outvar = mxCreateDoubleMatrix(20, 1, mxREAL);
	copyCArrayToM( output.x161, mxGetPr(outvar), 20);
	mxSetField(plhs[0], 0, "x161", outvar);	

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