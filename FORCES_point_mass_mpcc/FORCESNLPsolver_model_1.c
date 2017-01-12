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

static const int CASADI_PREFIX(s0)[] = {30, 1, 0, 30, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29};
#define s0 CASADI_PREFIX(s0)
static const int CASADI_PREFIX(s1)[] = {0, 1, 0, 0};
#define s1 CASADI_PREFIX(s1)
static const int CASADI_PREFIX(s2)[] = {1, 1, 0, 1, 0};
#define s2 CASADI_PREFIX(s2)
static const int CASADI_PREFIX(s3)[] = {1, 30, 0, 0, 0, 0, 0, 0, 1, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 13, 14, 15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
#define s3 CASADI_PREFIX(s3)
static const int CASADI_PREFIX(s4)[] = {25, 1, 0, 25, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24};
#define s4 CASADI_PREFIX(s4)
static const int CASADI_PREFIX(s5)[] = {25, 30, 0, 2, 4, 6, 8, 10, 11, 12, 13, 14, 17, 20, 23, 26, 27, 28, 29, 30, 31, 32, 33, 34, 34, 34, 34, 34, 35, 38, 39, 40, 40, 0, 4, 1, 5, 2, 6, 3, 7, 20, 21, 0, 1, 2, 3, 0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 20, 21, 22, 23, 24};
#define s5 CASADI_PREFIX(s5)
/* evaluate_stages */
int FORCESNLPsolver_model_1(const FORCESNLPsolver_FLOAT** arg, FORCESNLPsolver_FLOAT** res) {
     FORCESNLPsolver_FLOAT a0,a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14,a15,a16,a17,a18,a19,a20,a21,a22,a23,a24,a25,a26,a27,a28,a29,a30,a31,a32,a33,a34,a35,a36,a37,a38,a39,a40,a41,a42,a43,a44,a45,a46,a47,a48,a49,a50,a51,a52,a53,a54,a55,a56,a57,a58,a59,a60,a61,a62,a63,a64,a65,a66,a67,a68,a69,a70,a71,a72,a73,a74;
         a0=arg[0] ? arg[0][25] : 0;
         a1=cos(a0);
         a2=arg[0] ? arg[0][5] : 0;
  a1=(a2-a1);
         a3=cos(a0);
         a4=sin(a0);
         a5=(-a4);
  a5=atan2(a3,a5);
         a6=sin(a5);
         a7=(a6*a1);
         a8=sin(a0);
         a9=arg[0] ? arg[0][6] : 0;
  a8=(a9-a8);
         a10=cos(a0);
         a11=sin(a0);
         a12=(-a11);
  a12=atan2(a10,a12);
         a13=cos(a12);
         a14=(a13*a8);
  a7=(a7-a14);
  a14=10000.;
  a7=(a14*a7);
         a15=cos(a0);
  a15=(a2-a15);
         a16=cos(a0);
         a17=sin(a0);
         a18=(-a17);
  a18=atan2(a16,a18);
         a19=sin(a18);
         a20=(a19*a15);
         a21=sin(a0);
  a21=(a9-a21);
         a22=cos(a0);
         a23=sin(a0);
         a24=(-a23);
  a24=atan2(a22,a24);
         a25=cos(a24);
         a26=(a25*a21);
  a20=(a20-a26);
  a26=(a7*a20);
         a27=cos(a0);
  a27=(a2-a27);
         a28=cos(a0);
         a29=sin(a0);
         a30=(-a29);
  a30=atan2(a28,a30);
         a31=cos(a30);
         a32=(a31*a27);
         a33=sin(a0);
  a33=(a9-a33);
         a34=cos(a0);
         a35=sin(a0);
         a36=(-a35);
  a36=atan2(a34,a36);
         a37=sin(a36);
         a38=(a37*a33);
  a32=(a32+a38);
  a32=(a14*a32);
  a38=cos(a0);
  a38=(a2-a38);
         a39=cos(a0);
         a40=sin(a0);
         a41=(-a40);
  a41=atan2(a39,a41);
         a42=cos(a41);
         a43=(a42*a38);
         a44=sin(a0);
  a44=(a9-a44);
         a45=cos(a0);
         a46=sin(a0);
         a47=(-a46);
  a47=atan2(a45,a47);
         a48=sin(a47);
         a49=(a48*a44);
  a43=(a43+a49);
  a49=(a32*a43);
  a26=(a26+a49);
  a49=arg[0] ? arg[0][7] : 0;
         a50=(a14*a49);
         a51=(a50*a49);
  a26=(a26+a51);
  a51=100.;
         a52=arg[0] ? arg[0][26] : 0;
  a51=(a51*a52);
  a26=(a26-a51);
  a51=9.9999999999999995e-07;
         a53=arg[0] ? arg[0][17] : 0;
         a54=(a51*a53);
         a55=(a54*a53);
         a56=arg[0] ? arg[0][18] : 0;
         a57=(a51*a56);
         a58=(a57*a56);
  a55=(a55+a58);
  a58=arg[0] ? arg[0][19] : 0;
         a59=(a51*a58);
         a60=(a59*a58);
  a55=(a55+a60);
  a60=arg[0] ? arg[0][20] : 0;
         a61=(a51*a60);
         a62=(a61*a60);
  a55=(a55+a62);
  a62=arg[0] ? arg[0][21] : 0;
         a63=(a51*a62);
         a64=(a63*a62);
  a55=(a55+a64);
  a64=arg[0] ? arg[0][22] : 0;
         a65=(a51*a64);
         a66=(a65*a64);
  a55=(a55+a66);
  a66=arg[0] ? arg[0][23] : 0;
         a67=(a51*a66);
         a68=(a67*a66);
  a55=(a55+a68);
  a68=arg[0] ? arg[0][24] : 0;
         a69=(a51*a68);
         a70=(a69*a68);
  a55=(a55+a70);
  a26=(a26+a55);
  a55=arg[0] ? arg[0][28] : 0;
  a70=(a51*a55);
         a71=(a70*a55);
         a72=arg[0] ? arg[0][29] : 0;
         a73=(a51*a72);
         a74=(a73*a72);
  a71=(a71+a74);
  a26=(a26+a71);
  if (res[0]!=0) res[0][0]=a26;
  a42=(a42*a32);
  a43=(a14*a43);
  a31=(a31*a43);
  a26=(a42+a31);
  a19=(a19*a7);
  a26=(a26+a19);
  a20=(a14*a20);
  a6=(a6*a20);
  a26=(a26+a6);
  if (res[1]!=0) res[1][0]=a26;
  a48=(a48*a32);
  a37=(a37*a43);
  a26=(a48+a37);
  a25=(a25*a7);
  a26=(a26-a25);
  a13=(a13*a20);
  a26=(a26-a13);
  if (res[1]!=0) res[1][1]=a26;
  a14=(a14*a49);
  a50=(a50+a14);
  if (res[1]!=0) res[1][2]=a50;
  a50=(a51*a53);
  a54=(a54+a50);
  if (res[1]!=0) res[1][3]=a54;
  a54=(a51*a56);
  a57=(a57+a54);
  if (res[1]!=0) res[1][4]=a57;
  a57=(a51*a58);
  a59=(a59+a57);
  if (res[1]!=0) res[1][5]=a59;
  a59=(a51*a60);
  a61=(a61+a59);
  if (res[1]!=0) res[1][6]=a61;
  a62=(a51*a62);
  a63=(a63+a62);
  if (res[1]!=0) res[1][7]=a63;
  a64=(a51*a64);
  a65=(a65+a64);
  if (res[1]!=0) res[1][8]=a65;
  a66=(a51*a66);
  a67=(a67+a66);
  if (res[1]!=0) res[1][9]=a67;
  a68=(a51*a68);
  a69=(a69+a68);
  if (res[1]!=0) res[1][10]=a69;
  a44=(a44*a32);
  a47=cos(a47);
  a47=(a47*a44);
  a45=(a45*a47);
  a44=cos(a0);
  a44=(a44*a45);
  a46=(a46*a47);
  a47=sin(a0);
  a47=(a47*a46);
  a44=(a44+a47);
  a47=cos(a0);
  a47=(a47*a48);
  a44=(a44-a47);
  a38=(a38*a32);
  a41=sin(a41);
  a41=(a41*a38);
  a39=(a39*a41);
  a38=cos(a0);
  a38=(a38*a39);
  a44=(a44-a38);
  a40=(a40*a41);
  a41=sin(a0);
  a41=(a41*a40);
  a44=(a44-a41);
  a41=sin(a0);
  a41=(a41*a42);
  a44=(a44+a41);
  a33=(a33*a43);
  a36=cos(a36);
  a36=(a36*a33);
  a34=(a34*a36);
  a33=cos(a0);
  a33=(a33*a34);
  a44=(a44+a33);
  a35=(a35*a36);
  a36=sin(a0);
  a36=(a36*a35);
  a44=(a44+a36);
  a36=cos(a0);
  a36=(a36*a37);
  a44=(a44-a36);
  a27=(a27*a43);
  a30=sin(a30);
  a30=(a30*a27);
  a28=(a28*a30);
  a27=cos(a0);
  a27=(a27*a28);
  a44=(a44-a27);
  a29=(a29*a30);
  a30=sin(a0);
  a30=(a30*a29);
  a44=(a44-a30);
  a30=sin(a0);
  a30=(a30*a31);
  a44=(a44+a30);
  a21=(a21*a7);
  a24=sin(a24);
  a24=(a24*a21);
  a22=(a22*a24);
  a21=cos(a0);
  a21=(a21*a22);
  a44=(a44+a21);
  a23=(a23*a24);
  a24=sin(a0);
  a24=(a24*a23);
  a44=(a44+a24);
  a24=cos(a0);
  a24=(a24*a25);
  a44=(a44+a24);
  a15=(a15*a7);
  a18=cos(a18);
  a18=(a18*a15);
  a16=(a16*a18);
  a15=cos(a0);
  a15=(a15*a16);
  a44=(a44+a15);
  a17=(a17*a18);
  a18=sin(a0);
  a18=(a18*a17);
  a44=(a44+a18);
  a18=sin(a0);
  a18=(a18*a19);
  a44=(a44+a18);
  a8=(a8*a20);
  a12=sin(a12);
  a12=(a12*a8);
  a10=(a10*a12);
  a8=cos(a0);
  a8=(a8*a10);
  a44=(a44+a8);
  a11=(a11*a12);
  a12=sin(a0);
  a12=(a12*a11);
  a44=(a44+a12);
  a12=cos(a0);
  a12=(a12*a13);
  a44=(a44+a12);
  a1=(a1*a20);
  a5=cos(a5);
  a5=(a5*a1);
  a3=(a3*a5);
  a1=cos(a0);
  a1=(a1*a3);
  a44=(a44+a1);
  a4=(a4*a5);
  a5=sin(a0);
  a5=(a5*a4);
  a44=(a44+a5);
  a5=sin(a0);
  a5=(a5*a6);
  a44=(a44+a5);
  if (res[1]!=0) res[1][11]=a44;
  a44=-100.;
  if (res[1]!=0) res[1][12]=a44;
  a44=(a51*a55);
  a70=(a70+a44);
  if (res[1]!=0) res[1][13]=a70;
  a51=(a51*a72);
  a73=(a73+a51);
  if (res[1]!=0) res[1][14]=a73;
  a73=5.0000000000000003e-02;
  a51=arg[0] ? arg[0][9] : 0;
  a72=(a73*a51);
  a2=(a2+a72);
  a72=1.2500000000000002e-03;
  a70=arg[0] ? arg[0][0] : 0;
  a44=(a72*a70);
  a2=(a2+a44);
  if (res[2]!=0) res[2][0]=a2;
  a2=arg[0] ? arg[0][10] : 0;
  a44=(a73*a2);
  a9=(a9+a44);
  a44=arg[0] ? arg[0][1] : 0;
  a5=(a72*a44);
  a9=(a9+a5);
  if (res[2]!=0) res[2][1]=a9;
  a9=arg[0] ? arg[0][11] : 0;
  a5=(a73*a9);
  a49=(a49+a5);
  a5=arg[0] ? arg[0][2] : 0;
  a6=(a72*a5);
  a49=(a49+a6);
  a6=-1.2250000000000004e-02;
  a49=(a49+a6);
  if (res[2]!=0) res[2][2]=a49;
  a49=arg[0] ? arg[0][12] : 0;
  a6=(a73*a49);
  a4=arg[0] ? arg[0][8] : 0;
  a4=(a4+a6);
  a6=arg[0] ? arg[0][3] : 0;
  a1=(a72*a6);
  a4=(a4+a1);
  if (res[2]!=0) res[2][3]=a4;
  a70=(a73*a70);
  a70=(a51+a70);
  if (res[2]!=0) res[2][4]=a70;
  a44=(a73*a44);
  a44=(a2+a44);
  if (res[2]!=0) res[2][5]=a44;
  a5=(a73*a5);
  a5=(a9+a5);
  a44=-4.9000000000000005e-01;
  a5=(a5+a44);
  if (res[2]!=0) res[2][6]=a5;
  a6=(a73*a6);
  a6=(a49+a6);
  if (res[2]!=0) res[2][7]=a6;
  a6=-20.;
  a51=(a6*a51);
  if (res[2]!=0) res[2][8]=a51;
  a2=(a6*a2);
  if (res[2]!=0) res[2][9]=a2;
  a9=(a6*a9);
  if (res[2]!=0) res[2][10]=a9;
  a49=(a6*a49);
  if (res[2]!=0) res[2][11]=a49;
  a49=arg[0] ? arg[0][13] : 0;
  a49=(a6*a49);
  if (res[2]!=0) res[2][12]=a49;
  a49=arg[0] ? arg[0][14] : 0;
  a49=(a6*a49);
  if (res[2]!=0) res[2][13]=a49;
  a49=arg[0] ? arg[0][15] : 0;
  a49=(a6*a49);
  if (res[2]!=0) res[2][14]=a49;
  a49=arg[0] ? arg[0][16] : 0;
  a49=(a6*a49);
  if (res[2]!=0) res[2][15]=a49;
  a53=(a6*a53);
  if (res[2]!=0) res[2][16]=a53;
  a56=(a6*a56);
  if (res[2]!=0) res[2][17]=a56;
  a58=(a6*a58);
  if (res[2]!=0) res[2][18]=a58;
  a60=(a6*a60);
  if (res[2]!=0) res[2][19]=a60;
  a60=(a73*a52);
  a0=(a0+a60);
  a60=arg[0] ? arg[0][4] : 0;
  a58=(a72*a60);
  a0=(a0+a58);
  if (res[2]!=0) res[2][20]=a0;
  a60=(a73*a60);
  a60=(a52+a60);
  if (res[2]!=0) res[2][21]=a60;
  a52=(a6*a52);
  if (res[2]!=0) res[2][22]=a52;
  a52=arg[0] ? arg[0][27] : 0;
  a52=(a6*a52);
  if (res[2]!=0) res[2][23]=a52;
  a55=(a6*a55);
  if (res[2]!=0) res[2][24]=a55;
  if (res[3]!=0) res[3][0]=a72;
  if (res[3]!=0) res[3][1]=a73;
  if (res[3]!=0) res[3][2]=a72;
  if (res[3]!=0) res[3][3]=a73;
  if (res[3]!=0) res[3][4]=a72;
  if (res[3]!=0) res[3][5]=a73;
  if (res[3]!=0) res[3][6]=a72;
  if (res[3]!=0) res[3][7]=a73;
  if (res[3]!=0) res[3][8]=a72;
  if (res[3]!=0) res[3][9]=a73;
  a72=1.;
  if (res[3]!=0) res[3][10]=a72;
  if (res[3]!=0) res[3][11]=a72;
  if (res[3]!=0) res[3][12]=a72;
  if (res[3]!=0) res[3][13]=a72;
  if (res[3]!=0) res[3][14]=a73;
  if (res[3]!=0) res[3][15]=a72;
  if (res[3]!=0) res[3][16]=a6;
  if (res[3]!=0) res[3][17]=a73;
  if (res[3]!=0) res[3][18]=a72;
  if (res[3]!=0) res[3][19]=a6;
  if (res[3]!=0) res[3][20]=a73;
  if (res[3]!=0) res[3][21]=a72;
  if (res[3]!=0) res[3][22]=a6;
  if (res[3]!=0) res[3][23]=a73;
  if (res[3]!=0) res[3][24]=a72;
  if (res[3]!=0) res[3][25]=a6;
  if (res[3]!=0) res[3][26]=a6;
  if (res[3]!=0) res[3][27]=a6;
  if (res[3]!=0) res[3][28]=a6;
  if (res[3]!=0) res[3][29]=a6;
  if (res[3]!=0) res[3][30]=a6;
  if (res[3]!=0) res[3][31]=a6;
  if (res[3]!=0) res[3][32]=a6;
  if (res[3]!=0) res[3][33]=a6;
  if (res[3]!=0) res[3][34]=a72;
  if (res[3]!=0) res[3][35]=a73;
  if (res[3]!=0) res[3][36]=a72;
  if (res[3]!=0) res[3][37]=a6;
  if (res[3]!=0) res[3][38]=a6;
  if (res[3]!=0) res[3][39]=a6;
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
  if (sz_w) *sz_w = 75;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
