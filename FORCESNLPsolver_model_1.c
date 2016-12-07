/* This function was automatically generated by CasADi */#ifdef __cplusplus
extern "C" {
#endif

#ifdef CODEGEN_PREFIX
  #define NAMESPACE_CONCAT(NS, ID) _NAMESPACE_CONCAT(NS, ID)
  #define _NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else /* CODEGEN_PREFIX */
  #define CASADI_PREFIX(ID) FORCESNLPsolver_model_1_ ## ID
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
static const int CASADI_PREFIX(s4)[] = {20, 1, 0, 20, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
#define s4 CASADI_PREFIX(s4)
static const int CASADI_PREFIX(s5)[] = {20, 24, 0, 2, 4, 6, 8, 9, 10, 11, 12, 15, 18, 21, 24, 25, 26, 27, 28, 29, 30, 31, 32, 32, 32, 32, 32, 0, 4, 1, 5, 2, 6, 3, 7, 0, 1, 2, 3, 0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11, 12, 13, 14, 15, 16, 17, 18, 19};
#define s5 CASADI_PREFIX(s5)
/* evaluate_stages */
int FORCESNLPsolver_model_1(const FORCESNLPsolver_FLOAT** arg, FORCESNLPsolver_FLOAT** res) {
     FORCESNLPsolver_FLOAT a0,a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14,a15,a16,a17,a18,a19,a20,a21,a22,a23,a24,a25,a26,a27,a28,a29,a30,a31,a32,a33,a34,a35,a36,a37,a38,a39,a40,a41,a42,a43,a44,a45,a46,a47,a48,a49,a50,a51,a52,a53,a54,a55,a56,a57,a58,a59,a60,a61,a62,a63,a64,a65,a66,a67,a68,a69,a70,a71,a72,a73,a74,a75,a76,a77;
         a0=arg[0] ? arg[0][4] : 0;
         a1=arg[1] ? arg[1][0] : 0;
         a2=(a0-a1);
         a3=arg[1] ? arg[1][20] : 0;
  a2=(a2*a3);
  a1=(a0-a1);
         a4=(a2*a1);
         a5=arg[0] ? arg[0][5] : 0;
         a6=arg[1] ? arg[1][1] : 0;
         a7=(a5-a6);
         a8=arg[1] ? arg[1][21] : 0;
  a7=(a7*a8);
  a6=(a5-a6);
         a9=(a7*a6);
  a4=(a4+a9);
  a9=arg[0] ? arg[0][6] : 0;
         a10=arg[1] ? arg[1][2] : 0;
         a11=(a9-a10);
         a12=arg[1] ? arg[1][22] : 0;
  a11=(a11*a12);
  a10=(a9-a10);
         a13=(a11*a10);
  a4=(a4+a13);
  a13=arg[0] ? arg[0][7] : 0;
         a14=arg[1] ? arg[1][3] : 0;
         a15=(a13-a14);
         a16=arg[1] ? arg[1][23] : 0;
  a15=(a15*a16);
  a14=(a13-a14);
         a17=(a15*a14);
  a4=(a4+a17);
  a17=arg[0] ? arg[0][8] : 0;
         a18=arg[1] ? arg[1][4] : 0;
         a19=(a17-a18);
         a20=arg[1] ? arg[1][24] : 0;
  a19=(a19*a20);
  a18=(a17-a18);
         a21=(a19*a18);
  a4=(a4+a21);
  a21=arg[0] ? arg[0][9] : 0;
         a22=arg[1] ? arg[1][5] : 0;
         a23=(a21-a22);
         a24=arg[1] ? arg[1][25] : 0;
  a23=(a23*a24);
  a22=(a21-a22);
         a25=(a23*a22);
  a4=(a4+a25);
  a25=arg[0] ? arg[0][10] : 0;
         a26=arg[1] ? arg[1][6] : 0;
         a27=(a25-a26);
         a28=arg[1] ? arg[1][26] : 0;
  a27=(a27*a28);
  a26=(a25-a26);
         a29=(a27*a26);
  a4=(a4+a29);
  a29=arg[0] ? arg[0][11] : 0;
         a30=arg[1] ? arg[1][7] : 0;
         a31=(a29-a30);
         a32=arg[1] ? arg[1][27] : 0;
  a31=(a31*a32);
  a30=(a29-a30);
         a33=(a31*a30);
  a4=(a4+a33);
  a33=arg[0] ? arg[0][12] : 0;
         a34=arg[1] ? arg[1][8] : 0;
         a35=(a33-a34);
         a36=arg[1] ? arg[1][28] : 0;
  a35=(a35*a36);
  a34=(a33-a34);
         a37=(a35*a34);
  a4=(a4+a37);
  a37=arg[0] ? arg[0][13] : 0;
         a38=arg[1] ? arg[1][9] : 0;
         a39=(a37-a38);
         a40=arg[1] ? arg[1][29] : 0;
  a39=(a39*a40);
  a38=(a37-a38);
         a41=(a39*a38);
  a4=(a4+a41);
  a41=arg[0] ? arg[0][14] : 0;
         a42=arg[1] ? arg[1][10] : 0;
         a43=(a41-a42);
         a44=arg[1] ? arg[1][30] : 0;
  a43=(a43*a44);
  a42=(a41-a42);
         a45=(a43*a42);
  a4=(a4+a45);
  a45=arg[0] ? arg[0][15] : 0;
         a46=arg[1] ? arg[1][11] : 0;
         a47=(a45-a46);
         a48=arg[1] ? arg[1][31] : 0;
  a47=(a47*a48);
  a46=(a45-a46);
         a49=(a47*a46);
  a4=(a4+a49);
  a49=arg[0] ? arg[0][16] : 0;
         a50=arg[1] ? arg[1][12] : 0;
         a51=(a49-a50);
         a52=arg[1] ? arg[1][32] : 0;
  a51=(a51*a52);
  a50=(a49-a50);
         a53=(a51*a50);
  a4=(a4+a53);
  a53=arg[0] ? arg[0][17] : 0;
         a54=arg[1] ? arg[1][13] : 0;
         a55=(a53-a54);
         a56=arg[1] ? arg[1][33] : 0;
  a55=(a55*a56);
  a54=(a53-a54);
         a57=(a55*a54);
  a4=(a4+a57);
  a57=arg[0] ? arg[0][18] : 0;
         a58=arg[1] ? arg[1][14] : 0;
         a59=(a57-a58);
         a60=arg[1] ? arg[1][34] : 0;
  a59=(a59*a60);
  a58=(a57-a58);
         a61=(a59*a58);
  a4=(a4+a61);
  a61=arg[0] ? arg[0][19] : 0;
         a62=arg[1] ? arg[1][15] : 0;
         a63=(a61-a62);
         a64=arg[1] ? arg[1][35] : 0;
  a63=(a63*a64);
  a62=(a61-a62);
         a65=(a63*a62);
  a4=(a4+a65);
  a65=arg[0] ? arg[0][20] : 0;
         a66=arg[1] ? arg[1][16] : 0;
         a67=(a65-a66);
         a68=arg[1] ? arg[1][36] : 0;
  a67=(a67*a68);
  a65=(a65-a66);
  a66=(a67*a65);
  a4=(a4+a66);
  a66=arg[0] ? arg[0][21] : 0;
         a69=arg[1] ? arg[1][17] : 0;
         a70=(a66-a69);
         a71=arg[1] ? arg[1][37] : 0;
  a70=(a70*a71);
  a66=(a66-a69);
  a69=(a70*a66);
  a4=(a4+a69);
  a69=arg[0] ? arg[0][22] : 0;
         a72=arg[1] ? arg[1][18] : 0;
         a73=(a69-a72);
         a74=arg[1] ? arg[1][38] : 0;
  a73=(a73*a74);
  a69=(a69-a72);
  a72=(a73*a69);
  a4=(a4+a72);
  a72=arg[0] ? arg[0][23] : 0;
         a75=arg[1] ? arg[1][19] : 0;
         a76=(a72-a75);
         a77=arg[1] ? arg[1][39] : 0;
  a76=(a76*a77);
  a72=(a72-a75);
  a75=(a76*a72);
  a4=(a4+a75);
  if (res[0]!=0) res[0][0]=a4;
  a3=(a3*a1);
  a2=(a2+a3);
  if (res[1]!=0) res[1][0]=a2;
  a8=(a8*a6);
  a7=(a7+a8);
  if (res[1]!=0) res[1][1]=a7;
  a12=(a12*a10);
  a11=(a11+a12);
  if (res[1]!=0) res[1][2]=a11;
  a16=(a16*a14);
  a15=(a15+a16);
  if (res[1]!=0) res[1][3]=a15;
  a20=(a20*a18);
  a19=(a19+a20);
  if (res[1]!=0) res[1][4]=a19;
  a24=(a24*a22);
  a23=(a23+a24);
  if (res[1]!=0) res[1][5]=a23;
  a28=(a28*a26);
  a27=(a27+a28);
  if (res[1]!=0) res[1][6]=a27;
  a32=(a32*a30);
  a31=(a31+a32);
  if (res[1]!=0) res[1][7]=a31;
  a36=(a36*a34);
  a35=(a35+a36);
  if (res[1]!=0) res[1][8]=a35;
  a40=(a40*a38);
  a39=(a39+a40);
  if (res[1]!=0) res[1][9]=a39;
  a44=(a44*a42);
  a43=(a43+a44);
  if (res[1]!=0) res[1][10]=a43;
  a48=(a48*a46);
  a47=(a47+a48);
  if (res[1]!=0) res[1][11]=a47;
  a52=(a52*a50);
  a51=(a51+a52);
  if (res[1]!=0) res[1][12]=a51;
  a56=(a56*a54);
  a55=(a55+a56);
  if (res[1]!=0) res[1][13]=a55;
  a60=(a60*a58);
  a59=(a59+a60);
  if (res[1]!=0) res[1][14]=a59;
  a64=(a64*a62);
  a63=(a63+a64);
  if (res[1]!=0) res[1][15]=a63;
  a68=(a68*a65);
  a67=(a67+a68);
  if (res[1]!=0) res[1][16]=a67;
  a71=(a71*a66);
  a70=(a70+a71);
  if (res[1]!=0) res[1][17]=a70;
  a74=(a74*a69);
  a73=(a73+a74);
  if (res[1]!=0) res[1][18]=a73;
  a77=(a77*a72);
  a76=(a76+a77);
  if (res[1]!=0) res[1][19]=a76;
  a76=1.0000000000000001e-01;
  a77=(a76*a17);
  a0=(a0+a77);
  a77=5.0000000000000010e-03;
  a72=arg[0] ? arg[0][0] : 0;
  a73=(a77*a72);
  a0=(a0+a73);
  if (res[2]!=0) res[2][0]=a0;
  a0=(a76*a21);
  a5=(a5+a0);
  a0=arg[0] ? arg[0][1] : 0;
  a73=(a77*a0);
  a5=(a5+a73);
  if (res[2]!=0) res[2][1]=a5;
  a5=(a76*a25);
  a9=(a9+a5);
  a5=arg[0] ? arg[0][2] : 0;
  a73=(a77*a5);
  a9=(a9+a73);
  a73=-4.9000000000000016e-02;
  a9=(a9+a73);
  if (res[2]!=0) res[2][2]=a9;
  a9=(a76*a29);
  a13=(a13+a9);
  a9=arg[0] ? arg[0][3] : 0;
  a73=(a77*a9);
  a13=(a13+a73);
  if (res[2]!=0) res[2][3]=a13;
  a72=(a76*a72);
  a72=(a17+a72);
  if (res[2]!=0) res[2][4]=a72;
  a0=(a76*a0);
  a0=(a21+a0);
  if (res[2]!=0) res[2][5]=a0;
  a5=(a76*a5);
  a5=(a25+a5);
  a0=-9.8000000000000009e-01;
  a5=(a5+a0);
  if (res[2]!=0) res[2][6]=a5;
  a9=(a76*a9);
  a9=(a29+a9);
  if (res[2]!=0) res[2][7]=a9;
  a9=-10.;
  a17=(a9*a17);
  if (res[2]!=0) res[2][8]=a17;
  a21=(a9*a21);
  if (res[2]!=0) res[2][9]=a21;
  a25=(a9*a25);
  if (res[2]!=0) res[2][10]=a25;
  a29=(a9*a29);
  if (res[2]!=0) res[2][11]=a29;
  a33=(a9*a33);
  if (res[2]!=0) res[2][12]=a33;
  a37=(a9*a37);
  if (res[2]!=0) res[2][13]=a37;
  a41=(a9*a41);
  if (res[2]!=0) res[2][14]=a41;
  a45=(a9*a45);
  if (res[2]!=0) res[2][15]=a45;
  a49=(a9*a49);
  if (res[2]!=0) res[2][16]=a49;
  a53=(a9*a53);
  if (res[2]!=0) res[2][17]=a53;
  a57=(a9*a57);
  if (res[2]!=0) res[2][18]=a57;
  a61=(a9*a61);
  if (res[2]!=0) res[2][19]=a61;
  if (res[3]!=0) res[3][0]=a77;
  if (res[3]!=0) res[3][1]=a76;
  if (res[3]!=0) res[3][2]=a77;
  if (res[3]!=0) res[3][3]=a76;
  if (res[3]!=0) res[3][4]=a77;
  if (res[3]!=0) res[3][5]=a76;
  if (res[3]!=0) res[3][6]=a77;
  if (res[3]!=0) res[3][7]=a76;
  a77=1.;
  if (res[3]!=0) res[3][8]=a77;
  if (res[3]!=0) res[3][9]=a77;
  if (res[3]!=0) res[3][10]=a77;
  if (res[3]!=0) res[3][11]=a77;
  if (res[3]!=0) res[3][12]=a76;
  if (res[3]!=0) res[3][13]=a77;
  if (res[3]!=0) res[3][14]=a9;
  if (res[3]!=0) res[3][15]=a76;
  if (res[3]!=0) res[3][16]=a77;
  if (res[3]!=0) res[3][17]=a9;
  if (res[3]!=0) res[3][18]=a76;
  if (res[3]!=0) res[3][19]=a77;
  if (res[3]!=0) res[3][20]=a9;
  if (res[3]!=0) res[3][21]=a76;
  if (res[3]!=0) res[3][22]=a77;
  if (res[3]!=0) res[3][23]=a9;
  if (res[3]!=0) res[3][24]=a9;
  if (res[3]!=0) res[3][25]=a9;
  if (res[3]!=0) res[3][26]=a9;
  if (res[3]!=0) res[3][27]=a9;
  if (res[3]!=0) res[3][28]=a9;
  if (res[3]!=0) res[3][29]=a9;
  if (res[3]!=0) res[3][30]=a9;
  if (res[3]!=0) res[3][31]=a9;
  return 0;
}

int FORCESNLPsolver_model_1_init(int *f_type, int *n_in, int *n_out, int *sz_arg, int* sz_res) {
  *f_type = 1;
  *n_in = 2;
  *n_out = 4;
  *sz_arg = 2;
  *sz_res = 4;
  return 0;
}

int FORCESNLPsolver_model_1_sparsity(int i, int *nrow, int *ncol, const int **colind, const int **row) {
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
    case 4:
      s = s4; break;
    case 5:
      s = s5; break;
    default:
      return 1;
  }

  *nrow = s[0];
  *ncol = s[1];
  *colind = s + 2;
  *row = s + 2 + (*ncol + 1);
  return 0;
}

int FORCESNLPsolver_model_1_work(int *sz_iw, int *sz_w) {
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 78;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
