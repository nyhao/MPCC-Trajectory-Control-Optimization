/* This function was automatically generated by CasADi */#ifdef __cplusplus
extern "C" {
#endif

#ifdef CODEGEN_PREFIX
  #define NAMESPACE_CONCAT(NS, ID) _NAMESPACE_CONCAT(NS, ID)
  #define _NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else /* CODEGEN_PREFIX */
  #define CASADI_PREFIX(ID) FORCESNLPsolver_model_41_ ## ID
#endif /* CODEGEN_PREFIX */

#include <math.h>

#include "FORCESNLPsolver/include/FORCESNLPsolver.h"

#define PRINTF printf
FORCESNLPsolver_FLOAT CASADI_PREFIX(sq)(FORCESNLPsolver_FLOAT x) { return x*x;}
#define sq(x) CASADI_PREFIX(sq)(x)

FORCESNLPsolver_FLOAT CASADI_PREFIX(sign)(FORCESNLPsolver_FLOAT x) { return x<0 ? -1 : x>0 ? 1 : x;}
#define sign(x) CASADI_PREFIX(sign)(x)

static const int CASADI_PREFIX(s0)[] = {24, 1, 0, 24, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23};
#define s0 CASADI_PREFIX(s0)
static const int CASADI_PREFIX(s1)[] = {40, 1, 0, 40, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39};
#define s1 CASADI_PREFIX(s1)
static const int CASADI_PREFIX(s2)[] = {1, 1, 0, 1, 0};
#define s2 CASADI_PREFIX(s2)
static const int CASADI_PREFIX(s3)[] = {1, 24, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
#define s3 CASADI_PREFIX(s3)
/* evaluate_stages */
int FORCESNLPsolver_model_41(const FORCESNLPsolver_FLOAT** arg, FORCESNLPsolver_FLOAT** res) {
     FORCESNLPsolver_FLOAT a0,a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14,a15,a16,a17,a18,a19,a20,a21,a22,a23,a24,a25,a26,a27,a28,a29,a30,a31,a32,a33,a34,a35,a36,a37,a38,a39,a40,a41,a42,a43,a44,a45,a46,a47,a48,a49,a50,a51,a52,a53,a54,a55,a56,a57,a58,a59,a60,a61;
         a0=arg[0] ? arg[0][4] : 0;
         a1=arg[1] ? arg[1][0] : 0;
         a2=(a0-a1);
         a3=arg[1] ? arg[1][20] : 0;
  a2=(a2*a3);
  a0=(a0-a1);
  a1=(a2*a0);
         a4=arg[0] ? arg[0][5] : 0;
         a5=arg[1] ? arg[1][1] : 0;
         a6=(a4-a5);
         a7=arg[1] ? arg[1][21] : 0;
  a6=(a6*a7);
  a4=(a4-a5);
  a5=(a6*a4);
  a1=(a1+a5);
  a5=arg[0] ? arg[0][6] : 0;
         a8=arg[1] ? arg[1][2] : 0;
         a9=(a5-a8);
         a10=arg[1] ? arg[1][22] : 0;
  a9=(a9*a10);
  a5=(a5-a8);
  a8=(a9*a5);
  a1=(a1+a8);
  a8=arg[0] ? arg[0][7] : 0;
         a11=arg[1] ? arg[1][3] : 0;
         a12=(a8-a11);
         a13=arg[1] ? arg[1][23] : 0;
  a12=(a12*a13);
  a8=(a8-a11);
  a11=(a12*a8);
  a1=(a1+a11);
  a11=arg[0] ? arg[0][8] : 0;
         a14=arg[1] ? arg[1][4] : 0;
         a15=(a11-a14);
         a16=arg[1] ? arg[1][24] : 0;
  a15=(a15*a16);
  a11=(a11-a14);
  a14=(a15*a11);
  a1=(a1+a14);
  a14=arg[0] ? arg[0][9] : 0;
         a17=arg[1] ? arg[1][5] : 0;
         a18=(a14-a17);
         a19=arg[1] ? arg[1][25] : 0;
  a18=(a18*a19);
  a14=(a14-a17);
  a17=(a18*a14);
  a1=(a1+a17);
  a17=arg[0] ? arg[0][10] : 0;
         a20=arg[1] ? arg[1][6] : 0;
         a21=(a17-a20);
         a22=arg[1] ? arg[1][26] : 0;
  a21=(a21*a22);
  a17=(a17-a20);
  a20=(a21*a17);
  a1=(a1+a20);
  a20=arg[0] ? arg[0][11] : 0;
         a23=arg[1] ? arg[1][7] : 0;
         a24=(a20-a23);
         a25=arg[1] ? arg[1][27] : 0;
  a24=(a24*a25);
  a20=(a20-a23);
  a23=(a24*a20);
  a1=(a1+a23);
  a23=arg[0] ? arg[0][12] : 0;
         a26=arg[1] ? arg[1][8] : 0;
         a27=(a23-a26);
         a28=arg[1] ? arg[1][28] : 0;
  a27=(a27*a28);
  a23=(a23-a26);
  a26=(a27*a23);
  a1=(a1+a26);
  a26=arg[0] ? arg[0][13] : 0;
         a29=arg[1] ? arg[1][9] : 0;
         a30=(a26-a29);
         a31=arg[1] ? arg[1][29] : 0;
  a30=(a30*a31);
  a26=(a26-a29);
  a29=(a30*a26);
  a1=(a1+a29);
  a29=arg[0] ? arg[0][14] : 0;
         a32=arg[1] ? arg[1][10] : 0;
         a33=(a29-a32);
         a34=arg[1] ? arg[1][30] : 0;
  a33=(a33*a34);
  a29=(a29-a32);
  a32=(a33*a29);
  a1=(a1+a32);
  a32=arg[0] ? arg[0][15] : 0;
         a35=arg[1] ? arg[1][11] : 0;
         a36=(a32-a35);
         a37=arg[1] ? arg[1][31] : 0;
  a36=(a36*a37);
  a32=(a32-a35);
  a35=(a36*a32);
  a1=(a1+a35);
  a35=arg[0] ? arg[0][16] : 0;
         a38=arg[1] ? arg[1][12] : 0;
         a39=(a35-a38);
         a40=arg[1] ? arg[1][32] : 0;
  a39=(a39*a40);
  a35=(a35-a38);
  a38=(a39*a35);
  a1=(a1+a38);
  a38=arg[0] ? arg[0][17] : 0;
         a41=arg[1] ? arg[1][13] : 0;
         a42=(a38-a41);
         a43=arg[1] ? arg[1][33] : 0;
  a42=(a42*a43);
  a38=(a38-a41);
  a41=(a42*a38);
  a1=(a1+a41);
  a41=arg[0] ? arg[0][18] : 0;
         a44=arg[1] ? arg[1][14] : 0;
         a45=(a41-a44);
         a46=arg[1] ? arg[1][34] : 0;
  a45=(a45*a46);
  a41=(a41-a44);
  a44=(a45*a41);
  a1=(a1+a44);
  a44=arg[0] ? arg[0][19] : 0;
         a47=arg[1] ? arg[1][15] : 0;
         a48=(a44-a47);
         a49=arg[1] ? arg[1][35] : 0;
  a48=(a48*a49);
  a44=(a44-a47);
  a47=(a48*a44);
  a1=(a1+a47);
  a47=arg[0] ? arg[0][20] : 0;
         a50=arg[1] ? arg[1][16] : 0;
         a51=(a47-a50);
         a52=arg[1] ? arg[1][36] : 0;
  a51=(a51*a52);
  a47=(a47-a50);
  a50=(a51*a47);
  a1=(a1+a50);
  a50=arg[0] ? arg[0][21] : 0;
         a53=arg[1] ? arg[1][17] : 0;
         a54=(a50-a53);
         a55=arg[1] ? arg[1][37] : 0;
  a54=(a54*a55);
  a50=(a50-a53);
  a53=(a54*a50);
  a1=(a1+a53);
  a53=arg[0] ? arg[0][22] : 0;
         a56=arg[1] ? arg[1][18] : 0;
         a57=(a53-a56);
         a58=arg[1] ? arg[1][38] : 0;
  a57=(a57*a58);
  a53=(a53-a56);
  a56=(a57*a53);
  a1=(a1+a56);
  a56=arg[0] ? arg[0][23] : 0;
         a59=arg[1] ? arg[1][19] : 0;
         a60=(a56-a59);
         a61=arg[1] ? arg[1][39] : 0;
  a60=(a60*a61);
  a56=(a56-a59);
  a59=(a60*a56);
  a1=(a1+a59);
  if (res[0]!=0) res[0][0]=a1;
  a3=(a3*a0);
  a2=(a2+a3);
  if (res[1]!=0) res[1][0]=a2;
  a7=(a7*a4);
  a6=(a6+a7);
  if (res[1]!=0) res[1][1]=a6;
  a10=(a10*a5);
  a9=(a9+a10);
  if (res[1]!=0) res[1][2]=a9;
  a13=(a13*a8);
  a12=(a12+a13);
  if (res[1]!=0) res[1][3]=a12;
  a16=(a16*a11);
  a15=(a15+a16);
  if (res[1]!=0) res[1][4]=a15;
  a19=(a19*a14);
  a18=(a18+a19);
  if (res[1]!=0) res[1][5]=a18;
  a22=(a22*a17);
  a21=(a21+a22);
  if (res[1]!=0) res[1][6]=a21;
  a25=(a25*a20);
  a24=(a24+a25);
  if (res[1]!=0) res[1][7]=a24;
  a28=(a28*a23);
  a27=(a27+a28);
  if (res[1]!=0) res[1][8]=a27;
  a31=(a31*a26);
  a30=(a30+a31);
  if (res[1]!=0) res[1][9]=a30;
  a34=(a34*a29);
  a33=(a33+a34);
  if (res[1]!=0) res[1][10]=a33;
  a37=(a37*a32);
  a36=(a36+a37);
  if (res[1]!=0) res[1][11]=a36;
  a40=(a40*a35);
  a39=(a39+a40);
  if (res[1]!=0) res[1][12]=a39;
  a43=(a43*a38);
  a42=(a42+a43);
  if (res[1]!=0) res[1][13]=a42;
  a46=(a46*a41);
  a45=(a45+a46);
  if (res[1]!=0) res[1][14]=a45;
  a49=(a49*a44);
  a48=(a48+a49);
  if (res[1]!=0) res[1][15]=a48;
  a52=(a52*a47);
  a51=(a51+a52);
  if (res[1]!=0) res[1][16]=a51;
  a55=(a55*a50);
  a54=(a54+a55);
  if (res[1]!=0) res[1][17]=a54;
  a58=(a58*a53);
  a57=(a57+a58);
  if (res[1]!=0) res[1][18]=a57;
  a61=(a61*a56);
  a60=(a60+a61);
  if (res[1]!=0) res[1][19]=a60;
  return 0;
}

int FORCESNLPsolver_model_41_init(int *f_type, int *n_in, int *n_out, int *sz_arg, int* sz_res) {
  *f_type = 1;
  *n_in = 2;
  *n_out = 2;
  *sz_arg = 2;
  *sz_res = 2;
  return 0;
}

int FORCESNLPsolver_model_41_sparsity(int i, int *nrow, int *ncol, const int **colind, const int **row) {
  const int* s;
  switch (i) {
    case 0:
      s = s0; break;
    case 1:
      s = s1; break;
    case 2:
      s = s2; break;
    case 3:
      s = s3; break;
    default:
      return 1;
  }

  *nrow = s[0];
  *ncol = s[1];
  *colind = s + 2;
  *row = s + 2 + (*ncol + 1);
  return 0;
}

int FORCESNLPsolver_model_41_work(int *sz_iw, int *sz_w) {
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 62;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
