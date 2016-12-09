/*  * CasADi to FORCES Template - missing information to be filled in by createCasadi.m 
 * (C) embotech GmbH, Zurich, Switzerland, 2013-16. All rights reserved.
 *
 * This file is part of the FORCES client, and carries the same license.
 */ 

#ifdef __cplusplus
extern "C" {
#endif
    
#include "FORCESNLPsolver/include/FORCESNLPsolver.h"    
    
/* prototyes for models */
extern void FORCESNLPsolver_model_1(const double** arg, double** res);
extern void FORCESNLPsolver_model_1_sparsity(int i, int *nrow, int *ncol, const int **colind, const int **row);
extern void FORCESNLPsolver_model_11(const double** arg, double** res);
extern void FORCESNLPsolver_model_11_sparsity(int i, int *nrow, int *ncol, const int **colind, const int **row);
    

/* copies data from sparse matrix into a dense one */
void sparse2fullCopy(int nrow, int ncol, const int* colidx, const int* row, FORCESNLPsolver_FLOAT *data, FORCESNLPsolver_FLOAT *Out)
{
    int i, j;
    
    /* copy data into dense matrix */
    for(i=0; i<ncol; i++){
        for( j=colidx[i]; j < colidx[i+1]; j++ ){
            Out[i*nrow + row[j]] = data[j];
        }
    }
}

/* CasADi - FORCES interface */
void FORCESNLPsolver_casadi2forces(FORCESNLPsolver_FLOAT *x,        /* primal vars                                         */
                                 FORCESNLPsolver_FLOAT *y,        /* eq. constraint multiplers                           */
                                 FORCESNLPsolver_FLOAT *l,        /* ineq. constraint multipliers                        */
                                 FORCESNLPsolver_FLOAT *p,        /* parameters                                          */
                                 FORCESNLPsolver_FLOAT *f,        /* objective function (scalar)                         */
                                 FORCESNLPsolver_FLOAT *nabla_f,  /* gradient of objective function                      */
                                 FORCESNLPsolver_FLOAT *c,        /* dynamics                                            */
                                 FORCESNLPsolver_FLOAT *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                 FORCESNLPsolver_FLOAT *h,        /* inequality constraints                              */
                                 FORCESNLPsolver_FLOAT *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                 FORCESNLPsolver_FLOAT *H,        /* Hessian (column major)                              */
                                 int stage                      /* stage number (0 indexed)                            */
                  )
{
    /* CasADi input and output arrays */
    const FORCESNLPsolver_FLOAT *in[4];
    FORCESNLPsolver_FLOAT *out[7];
    
    /* temporary storage for casadi sparse output */
    FORCESNLPsolver_FLOAT this_f;
    FORCESNLPsolver_FLOAT nabla_f_sparse[20];
    
    
    double c_sparse[20];
    double nabla_c_sparse[32];
            
    
    /* pointers to row and column info for 
     * column compressed format used by CasADi */
    int nrow, ncol;
    const int *colind, *row;
    
    /* set inputs for CasADi */
    in[0] = x;
    in[1] = p; /* maybe should be made conditional */
    in[2] = l; /* maybe should be made conditional */     
    in[3] = y; /* maybe should be made conditional */
    
    /* set outputs for CasADi */
    out[0] = &this_f;
    out[1] = nabla_f_sparse;
                
	 if (stage >= 0 && stage < 10)
	 {
		 /* set inputs */
		 out[2] = c_sparse;
		 out[3] = nabla_c_sparse;
		 /* call CasADi */
		 FORCESNLPsolver_model_1(in, out);

		 /* copy to dense */
		 if( nabla_f ){ FORCESNLPsolver_model_1_sparsity(3, &nrow, &ncol, &colind, &row); sparse2fullCopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f); }
		 if( c ){ FORCESNLPsolver_model_1_sparsity(4, &nrow, &ncol, &colind, &row); sparse2fullCopy(nrow, ncol, colind, row, c_sparse, c); }
		 if( nabla_c ){ FORCESNLPsolver_model_1_sparsity(5, &nrow, &ncol, &colind, &row); sparse2fullCopy(nrow, ncol, colind, row, nabla_c_sparse, nabla_c); }
		 
	 }

	 if (stage >= 10 && stage < 11)
	 {
		 /* call CasADi */
		 FORCESNLPsolver_model_11(in, out);

		 /* copy to dense */
		 if( nabla_f ){ FORCESNLPsolver_model_11_sparsity(3, &nrow, &ncol, &colind, &row); sparse2fullCopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f); }
		 
	 }

         
    
    /* add to objective */
    if( f ){
        *f += this_f;
    }
}

#ifdef __cplusplus
} /* extern "C" */
#endif