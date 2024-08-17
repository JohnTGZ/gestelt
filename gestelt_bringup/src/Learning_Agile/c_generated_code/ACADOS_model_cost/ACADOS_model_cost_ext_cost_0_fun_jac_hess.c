/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) ACADOS_model_cost_ext_cost_0_fun_jac_hess_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_s6 CASADI_PREFIX(s6)
#define casadi_s7 CASADI_PREFIX(s7)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[14] = {10, 1, 0, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
static const casadi_int casadi_s1[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[26] = {22, 1, 0, 22, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21};
static const casadi_int casadi_s4[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s5[18] = {14, 1, 0, 14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
static const casadi_int casadi_s6[43] = {14, 14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 14, 18, 22, 26, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 10, 11, 12, 13, 10, 11, 12, 13, 10, 11, 12, 13};
static const casadi_int casadi_s7[17] = {0, 14, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* ACADOS_model_cost_ext_cost_0_fun_jac_hess:(i0[10],i1[4],i2[],i3[22])->(o0,o1[14],o2[14x14,26nz],o3[],o4[0x14]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a7, a8, a9;
  a0=5.;
  a1=arg[0]? arg[0][0] : 0;
  a2=arg[3]? arg[3][0] : 0;
  a3=(a1-a2);
  a4=casadi_sq(a3);
  a5=arg[0]? arg[0][1] : 0;
  a6=arg[3]? arg[3][1] : 0;
  a7=(a5-a6);
  a8=casadi_sq(a7);
  a4=(a4+a8);
  a8=arg[0]? arg[0][2] : 0;
  a9=arg[3]? arg[3][2] : 0;
  a10=(a8-a9);
  a11=casadi_sq(a10);
  a4=(a4+a11);
  a4=(a0*a4);
  a11=arg[0]? arg[0][3] : 0;
  a12=arg[3]? arg[3][3] : 0;
  a13=(a11-a12);
  a14=casadi_sq(a13);
  a15=arg[0]? arg[0][4] : 0;
  a16=arg[3]? arg[3][4] : 0;
  a17=(a15-a16);
  a18=casadi_sq(a17);
  a14=(a14+a18);
  a18=arg[0]? arg[0][5] : 0;
  a19=arg[3]? arg[3][5] : 0;
  a20=(a18-a19);
  a21=casadi_sq(a20);
  a14=(a14+a21);
  a4=(a4+a14);
  a14=1.;
  a21=2.;
  a22=arg[3]? arg[3][8] : 0;
  a23=casadi_sq(a22);
  a24=arg[3]? arg[3][9] : 0;
  a25=casadi_sq(a24);
  a23=(a23+a25);
  a23=(a21*a23);
  a23=(a14-a23);
  a25=arg[0]? arg[0][8] : 0;
  a26=casadi_sq(a25);
  a27=arg[0]? arg[0][9] : 0;
  a28=casadi_sq(a27);
  a26=(a26+a28);
  a26=(a21*a26);
  a26=(a14-a26);
  a28=(a23*a26);
  a29=arg[3]? arg[3][7] : 0;
  a30=(a29*a22);
  a31=arg[3]? arg[3][6] : 0;
  a32=(a31*a24);
  a30=(a30-a32);
  a30=(a21*a30);
  a32=arg[0]? arg[0][7] : 0;
  a33=(a32*a25);
  a34=arg[0]? arg[0][6] : 0;
  a35=(a34*a27);
  a33=(a33-a35);
  a33=(a21*a33);
  a35=(a30*a33);
  a28=(a28+a35);
  a35=(a29*a24);
  a36=(a31*a22);
  a35=(a35+a36);
  a35=(a21*a35);
  a36=(a32*a27);
  a37=(a34*a25);
  a36=(a36+a37);
  a36=(a21*a36);
  a37=(a35*a36);
  a28=(a28+a37);
  a28=(a14-a28);
  a37=(a29*a22);
  a38=(a31*a24);
  a37=(a37+a38);
  a37=(a21*a37);
  a38=(a32*a25);
  a39=(a34*a27);
  a38=(a38+a39);
  a38=(a21*a38);
  a39=(a37*a38);
  a40=casadi_sq(a29);
  a41=casadi_sq(a24);
  a40=(a40+a41);
  a40=(a21*a40);
  a40=(a14-a40);
  a41=casadi_sq(a32);
  a42=casadi_sq(a27);
  a41=(a41+a42);
  a41=(a21*a41);
  a41=(a14-a41);
  a42=(a40*a41);
  a39=(a39+a42);
  a42=(a22*a24);
  a43=(a31*a29);
  a42=(a42-a43);
  a42=(a21*a42);
  a43=(a25*a27);
  a44=(a34*a32);
  a43=(a43-a44);
  a43=(a21*a43);
  a44=(a42*a43);
  a39=(a39+a44);
  a39=(a14-a39);
  a28=(a28+a39);
  a39=(a29*a24);
  a44=(a31*a22);
  a39=(a39-a44);
  a39=(a21*a39);
  a44=(a32*a27);
  a45=(a34*a25);
  a44=(a44-a45);
  a44=(a21*a44);
  a45=(a39*a44);
  a46=(a22*a24);
  a47=(a31*a29);
  a46=(a46+a47);
  a46=(a21*a46);
  a47=(a25*a27);
  a48=(a34*a32);
  a47=(a47+a48);
  a47=(a21*a47);
  a48=(a46*a47);
  a45=(a45+a48);
  a48=casadi_sq(a29);
  a49=casadi_sq(a22);
  a48=(a48+a49);
  a48=(a21*a48);
  a48=(a14-a48);
  a49=casadi_sq(a32);
  a50=casadi_sq(a25);
  a49=(a49+a50);
  a49=(a21*a49);
  a49=(a14-a49);
  a50=(a48*a49);
  a45=(a45+a50);
  a45=(a14-a45);
  a28=(a28+a45);
  a4=(a4+a28);
  a2=(a1-a2);
  a28=casadi_sq(a2);
  a6=(a5-a6);
  a45=casadi_sq(a6);
  a28=(a28+a45);
  a9=(a8-a9);
  a45=casadi_sq(a9);
  a28=(a28+a45);
  a28=(a0*a28);
  a11=(a11-a12);
  a12=casadi_sq(a11);
  a15=(a15-a16);
  a16=casadi_sq(a15);
  a12=(a12+a16);
  a18=(a18-a19);
  a19=casadi_sq(a18);
  a12=(a12+a19);
  a28=(a28+a12);
  a12=casadi_sq(a22);
  a19=casadi_sq(a24);
  a12=(a12+a19);
  a12=(a21*a12);
  a12=(a14-a12);
  a26=(a12*a26);
  a19=(a29*a22);
  a16=(a31*a24);
  a19=(a19-a16);
  a19=(a21*a19);
  a33=(a19*a33);
  a26=(a26+a33);
  a33=(a29*a24);
  a16=(a31*a22);
  a33=(a33+a16);
  a33=(a21*a33);
  a36=(a33*a36);
  a26=(a26+a36);
  a26=(a14-a26);
  a36=(a29*a22);
  a16=(a31*a24);
  a36=(a36+a16);
  a36=(a21*a36);
  a38=(a36*a38);
  a16=casadi_sq(a29);
  a45=casadi_sq(a24);
  a16=(a16+a45);
  a16=(a21*a16);
  a16=(a14-a16);
  a41=(a16*a41);
  a38=(a38+a41);
  a41=(a22*a24);
  a45=(a31*a29);
  a41=(a41-a45);
  a41=(a21*a41);
  a43=(a41*a43);
  a38=(a38+a43);
  a38=(a14-a38);
  a26=(a26+a38);
  a38=(a29*a24);
  a43=(a31*a22);
  a38=(a38-a43);
  a38=(a21*a38);
  a44=(a38*a44);
  a24=(a22*a24);
  a31=(a31*a29);
  a24=(a24+a31);
  a24=(a21*a24);
  a47=(a24*a47);
  a44=(a44+a47);
  a29=casadi_sq(a29);
  a22=casadi_sq(a22);
  a29=(a29+a22);
  a29=(a21*a29);
  a29=(a14-a29);
  a49=(a29*a49);
  a44=(a44+a49);
  a44=(a14-a44);
  a26=(a26+a44);
  a28=(a28+a26);
  a4=(a4+a28);
  a28=arg[3]? arg[3][21] : 0;
  a26=70.;
  a44=-10.;
  a49=arg[3]? arg[3][20] : 0;
  a49=(a28-a49);
  a49=casadi_sq(a49);
  a44=(a44*a49);
  a44=exp(a44);
  a26=(a26*a44);
  a44=arg[3]? arg[3][14] : 0;
  a1=(a1-a44);
  a44=casadi_sq(a1);
  a49=arg[3]? arg[3][15] : 0;
  a5=(a5-a49);
  a49=casadi_sq(a5);
  a44=(a44+a49);
  a49=arg[3]? arg[3][16] : 0;
  a8=(a8-a49);
  a49=casadi_sq(a8);
  a44=(a44+a49);
  a44=(a0*a44);
  a49=80.;
  a22=arg[3]? arg[3][17] : 0;
  a47=casadi_sq(a22);
  a31=arg[3]? arg[3][18] : 0;
  a43=casadi_sq(a31);
  a47=(a47+a43);
  a43=arg[3]? arg[3][19] : 0;
  a45=casadi_sq(a43);
  a47=(a47+a45);
  a47=sqrt(a47);
  a47=atan(a47);
  a45=sin(a47);
  a50=1.0000000000000000e-08;
  a22=(a22+a50);
  a50=casadi_sq(a22);
  a51=casadi_sq(a31);
  a50=(a50+a51);
  a51=casadi_sq(a43);
  a50=(a50+a51);
  a50=sqrt(a50);
  a31=(a31/a50);
  a31=(a45*a31);
  a51=casadi_sq(a31);
  a43=(a43/a50);
  a43=(a45*a43);
  a52=casadi_sq(a43);
  a51=(a51+a52);
  a51=(a21*a51);
  a51=(a14-a51);
  a52=casadi_sq(a25);
  a53=casadi_sq(a27);
  a52=(a52+a53);
  a52=(a21*a52);
  a52=(a14-a52);
  a52=(a51*a52);
  a22=(a22/a50);
  a45=(a45*a22);
  a22=(a45*a31);
  a47=cos(a47);
  a50=(a47*a43);
  a22=(a22-a50);
  a22=(a21*a22);
  a50=(a32*a25);
  a53=(a34*a27);
  a50=(a50-a53);
  a50=(a21*a50);
  a50=(a22*a50);
  a52=(a52+a50);
  a50=(a45*a43);
  a53=(a47*a31);
  a50=(a50+a53);
  a50=(a21*a50);
  a53=(a32*a27);
  a54=(a34*a25);
  a53=(a53+a54);
  a53=(a21*a53);
  a53=(a50*a53);
  a52=(a52+a53);
  a52=(a14-a52);
  a53=(a45*a31);
  a54=(a47*a43);
  a53=(a53+a54);
  a53=(a21*a53);
  a54=(a32*a25);
  a55=(a34*a27);
  a54=(a54+a55);
  a54=(a21*a54);
  a54=(a53*a54);
  a55=casadi_sq(a45);
  a56=casadi_sq(a43);
  a55=(a55+a56);
  a55=(a21*a55);
  a55=(a14-a55);
  a56=casadi_sq(a32);
  a57=casadi_sq(a27);
  a56=(a56+a57);
  a56=(a21*a56);
  a56=(a14-a56);
  a56=(a55*a56);
  a54=(a54+a56);
  a56=(a31*a43);
  a57=(a47*a45);
  a56=(a56-a57);
  a56=(a21*a56);
  a57=(a25*a27);
  a58=(a34*a32);
  a57=(a57-a58);
  a57=(a21*a57);
  a57=(a56*a57);
  a54=(a54+a57);
  a54=(a14-a54);
  a52=(a52+a54);
  a54=(a45*a43);
  a57=(a47*a31);
  a54=(a54-a57);
  a54=(a21*a54);
  a57=(a32*a27);
  a58=(a34*a25);
  a57=(a57-a58);
  a57=(a21*a57);
  a57=(a54*a57);
  a43=(a31*a43);
  a47=(a47*a45);
  a43=(a43+a47);
  a43=(a21*a43);
  a47=(a25*a27);
  a58=(a34*a32);
  a47=(a47+a58);
  a47=(a21*a47);
  a47=(a43*a47);
  a57=(a57+a47);
  a45=casadi_sq(a45);
  a31=casadi_sq(a31);
  a45=(a45+a31);
  a45=(a21*a45);
  a45=(a14-a45);
  a31=casadi_sq(a32);
  a47=casadi_sq(a25);
  a31=(a31+a47);
  a31=(a21*a31);
  a31=(a14-a31);
  a31=(a45*a31);
  a57=(a57+a31);
  a57=(a14-a57);
  a52=(a52+a57);
  a57=casadi_sq(a52);
  a57=(a49*a57);
  a44=(a44+a57);
  a44=(a26*a44);
  a44=(a28*a44);
  a4=(a4+a44);
  a44=10.;
  a57=arg[1]? arg[1][0] : 0;
  a31=arg[3]? arg[3][10] : 0;
  a31=(a57-a31);
  a47=casadi_sq(a31);
  a58=arg[1]? arg[1][1] : 0;
  a59=arg[3]? arg[3][11] : 0;
  a59=(a58-a59);
  a60=casadi_sq(a59);
  a47=(a47+a60);
  a60=arg[1]? arg[1][2] : 0;
  a61=arg[3]? arg[3][12] : 0;
  a61=(a60-a61);
  a62=casadi_sq(a61);
  a47=(a47+a62);
  a62=arg[1]? arg[1][3] : 0;
  a63=arg[3]? arg[3][13] : 0;
  a63=(a62-a63);
  a64=casadi_sq(a63);
  a47=(a47+a64);
  a47=(a44*a47);
  a4=(a4+a47);
  a47=100.;
  a64=casadi_sq(a58);
  a65=casadi_sq(a60);
  a64=(a64+a65);
  a65=casadi_sq(a62);
  a64=(a64+a65);
  a64=(a47*a64);
  a57=(a57+a64);
  a4=(a4+a57);
  if (res[0]!=0) res[0][0]=a4;
  a31=(a31+a31);
  a31=(a44*a31);
  a14=(a14+a31);
  if (res[1]!=0) res[1][0]=a14;
  a58=(a58+a58);
  a58=(a47*a58);
  a59=(a59+a59);
  a59=(a44*a59);
  a58=(a58+a59);
  if (res[1]!=0) res[1][1]=a58;
  a60=(a60+a60);
  a60=(a47*a60);
  a61=(a61+a61);
  a61=(a44*a61);
  a60=(a60+a61);
  if (res[1]!=0) res[1][2]=a60;
  a62=(a62+a62);
  a47=(a47*a62);
  a63=(a63+a63);
  a63=(a44*a63);
  a47=(a47+a63);
  if (res[1]!=0) res[1][3]=a47;
  a1=(a1+a1);
  a26=(a26*a28);
  a28=(a0*a26);
  a1=(a1*a28);
  a2=(a2+a2);
  a2=(a0*a2);
  a1=(a1+a2);
  a3=(a3+a3);
  a3=(a0*a3);
  a1=(a1+a3);
  if (res[1]!=0) res[1][4]=a1;
  a5=(a5+a5);
  a5=(a5*a28);
  a6=(a6+a6);
  a6=(a0*a6);
  a5=(a5+a6);
  a7=(a7+a7);
  a7=(a0*a7);
  a5=(a5+a7);
  if (res[1]!=0) res[1][5]=a5;
  a8=(a8+a8);
  a8=(a8*a28);
  a9=(a9+a9);
  a9=(a0*a9);
  a8=(a8+a9);
  a10=(a10+a10);
  a0=(a0*a10);
  a8=(a8+a0);
  if (res[1]!=0) res[1][6]=a8;
  a11=(a11+a11);
  a13=(a13+a13);
  a11=(a11+a13);
  if (res[1]!=0) res[1][7]=a11;
  a15=(a15+a15);
  a17=(a17+a17);
  a15=(a15+a17);
  if (res[1]!=0) res[1][8]=a15;
  a18=(a18+a18);
  a20=(a20+a20);
  a18=(a18+a20);
  if (res[1]!=0) res[1][9]=a18;
  a52=(a52+a52);
  a49=(a49*a26);
  a52=(a52*a49);
  a26=(a54*a52);
  a26=(a21*a26);
  a18=(a25*a26);
  a20=(a43*a52);
  a20=(a21*a20);
  a15=(a32*a20);
  a18=(a18-a15);
  a15=(a56*a52);
  a15=(a21*a15);
  a17=(a32*a15);
  a18=(a18+a17);
  a17=(a53*a52);
  a17=(a21*a17);
  a11=(a27*a17);
  a18=(a18-a11);
  a11=(a50*a52);
  a11=(a21*a11);
  a13=(a25*a11);
  a18=(a18-a13);
  a13=(a22*a52);
  a13=(a21*a13);
  a8=(a27*a13);
  a18=(a18+a8);
  a24=(a24+a46);
  a24=(a21*a24);
  a46=(a32*a24);
  a18=(a18-a46);
  a38=(a38+a39);
  a38=(a21*a38);
  a39=(a25*a38);
  a18=(a18+a39);
  a41=(a41+a42);
  a41=(a21*a41);
  a42=(a32*a41);
  a18=(a18+a42);
  a36=(a36+a37);
  a36=(a21*a36);
  a37=(a27*a36);
  a18=(a18-a37);
  a33=(a33+a35);
  a33=(a21*a33);
  a35=(a25*a33);
  a18=(a18-a35);
  a19=(a19+a30);
  a19=(a21*a19);
  a30=(a27*a19);
  a18=(a18+a30);
  if (res[1]!=0) res[1][10]=a18;
  a18=(a32+a32);
  a30=(a45*a52);
  a30=(a21*a30);
  a35=(a18*a30);
  a37=(a34*a20);
  a35=(a35-a37);
  a37=(a27*a26);
  a35=(a35-a37);
  a37=(a34*a15);
  a35=(a35+a37);
  a37=(a32+a32);
  a42=(a55*a52);
  a42=(a21*a42);
  a39=(a37*a42);
  a35=(a35+a39);
  a39=(a25*a17);
  a35=(a35-a39);
  a39=(a27*a11);
  a35=(a35-a39);
  a39=(a25*a13);
  a35=(a35-a39);
  a39=(a32+a32);
  a29=(a29+a48);
  a29=(a21*a29);
  a39=(a39*a29);
  a35=(a35+a39);
  a39=(a34*a24);
  a35=(a35-a39);
  a39=(a27*a38);
  a35=(a35-a39);
  a39=(a34*a41);
  a35=(a35+a39);
  a39=(a32+a32);
  a16=(a16+a40);
  a16=(a21*a16);
  a39=(a39*a16);
  a35=(a35+a39);
  a39=(a25*a36);
  a35=(a35-a39);
  a39=(a27*a33);
  a35=(a35-a39);
  a39=(a25*a19);
  a35=(a35-a39);
  if (res[1]!=0) res[1][11]=a35;
  a35=(a25+a25);
  a39=(a35*a30);
  a40=(a27*a20);
  a39=(a39-a40);
  a40=(a34*a26);
  a39=(a39+a40);
  a40=(a27*a15);
  a39=(a39-a40);
  a40=(a32*a17);
  a39=(a39-a40);
  a40=(a34*a11);
  a39=(a39-a40);
  a40=(a32*a13);
  a39=(a39-a40);
  a40=(a25+a25);
  a52=(a51*a52);
  a52=(a21*a52);
  a48=(a40*a52);
  a39=(a39+a48);
  a48=(a25+a25);
  a48=(a48*a29);
  a39=(a39+a48);
  a48=(a27*a24);
  a39=(a39-a48);
  a48=(a34*a38);
  a39=(a39+a48);
  a48=(a27*a41);
  a39=(a39-a48);
  a48=(a32*a36);
  a39=(a39-a48);
  a48=(a34*a33);
  a39=(a39-a48);
  a48=(a32*a19);
  a39=(a39-a48);
  a48=(a25+a25);
  a12=(a12+a23);
  a12=(a21*a12);
  a48=(a48*a12);
  a39=(a39+a48);
  if (res[1]!=0) res[1][12]=a39;
  a39=(a27+a27);
  a48=(a39*a42);
  a23=(a25*a20);
  a46=(a32*a26);
  a23=(a23+a46);
  a46=(a25*a15);
  a23=(a23+a46);
  a48=(a48-a23);
  a23=(a34*a17);
  a48=(a48-a23);
  a23=(a32*a11);
  a48=(a48-a23);
  a23=(a34*a13);
  a48=(a48+a23);
  a23=(a27+a27);
  a46=(a23*a52);
  a48=(a48+a46);
  a46=(a25*a24);
  a48=(a48-a46);
  a46=(a32*a38);
  a48=(a48-a46);
  a46=(a25*a41);
  a48=(a48-a46);
  a46=(a27+a27);
  a46=(a46*a16);
  a48=(a48+a46);
  a46=(a34*a36);
  a48=(a48-a46);
  a46=(a32*a33);
  a48=(a48-a46);
  a46=(a34*a19);
  a48=(a48+a46);
  a46=(a27+a27);
  a46=(a46*a12);
  a48=(a48+a46);
  if (res[1]!=0) res[1][13]=a48;
  a48=20.;
  if (res[2]!=0) res[2][0]=a48;
  a48=220.;
  if (res[2]!=0) res[2][1]=a48;
  if (res[2]!=0) res[2][2]=a48;
  if (res[2]!=0) res[2][3]=a48;
  a48=(a21*a28);
  a48=(a48+a44);
  a48=(a48+a44);
  if (res[2]!=0) res[2][4]=a48;
  a48=(a21*a28);
  a48=(a48+a44);
  a48=(a48+a44);
  if (res[2]!=0) res[2][5]=a48;
  a28=(a21*a28);
  a28=(a28+a44);
  a28=(a28+a44);
  if (res[2]!=0) res[2][6]=a28;
  a28=4.;
  if (res[2]!=0) res[2][7]=a28;
  if (res[2]!=0) res[2][8]=a28;
  if (res[2]!=0) res[2][9]=a28;
  a28=(a21*a25);
  a28=(a50*a28);
  a44=(a21*a27);
  a44=(a22*a44);
  a28=(a28-a44);
  a44=(a21*a27);
  a44=(a53*a44);
  a48=(a21*a32);
  a48=(a56*a48);
  a44=(a44-a48);
  a28=(a28+a44);
  a44=(a21*a32);
  a44=(a43*a44);
  a48=(a21*a25);
  a48=(a54*a48);
  a44=(a44-a48);
  a28=(a28+a44);
  a28=(a28+a28);
  a28=(a49*a28);
  a44=(a43*a28);
  a44=(a21*a44);
  a44=(a32*a44);
  a48=(a54*a28);
  a48=(a21*a48);
  a48=(a25*a48);
  a44=(a44-a48);
  a48=(a56*a28);
  a48=(a21*a48);
  a48=(a32*a48);
  a44=(a44-a48);
  a48=(a53*a28);
  a48=(a21*a48);
  a48=(a27*a48);
  a44=(a44+a48);
  a48=(a50*a28);
  a48=(a21*a48);
  a48=(a25*a48);
  a44=(a44+a48);
  a28=(a22*a28);
  a28=(a21*a28);
  a28=(a27*a28);
  a44=(a44-a28);
  if (res[2]!=0) res[2][10]=a44;
  a44=(a21*a25);
  a44=(a22*a44);
  a28=(a21*a27);
  a28=(a50*a28);
  a44=(a44+a28);
  a28=(a21*a25);
  a28=(a53*a28);
  a48=(a32+a32);
  a48=(a21*a48);
  a48=(a55*a48);
  a28=(a28-a48);
  a48=(a21*a34);
  a48=(a56*a48);
  a28=(a28-a48);
  a44=(a44+a28);
  a28=(a21*a27);
  a28=(a54*a28);
  a48=(a21*a34);
  a48=(a43*a48);
  a28=(a28+a48);
  a48=(a32+a32);
  a48=(a21*a48);
  a48=(a45*a48);
  a28=(a28-a48);
  a44=(a44+a28);
  a44=(a44+a44);
  a44=(a49*a44);
  a28=(a56*a44);
  a28=(a21*a28);
  a48=(a32*a28);
  a48=(a15-a48);
  a46=(a54*a44);
  a46=(a21*a46);
  a8=(a25*a46);
  a0=(a43*a44);
  a0=(a21*a0);
  a10=(a32*a0);
  a10=(a20-a10);
  a8=(a8+a10);
  a48=(a48-a8);
  a8=(a53*a44);
  a8=(a21*a8);
  a10=(a27*a8);
  a48=(a48+a10);
  a10=(a50*a44);
  a10=(a21*a10);
  a9=(a25*a10);
  a48=(a48+a9);
  a9=(a22*a44);
  a9=(a21*a9);
  a5=(a27*a9);
  a48=(a48-a5);
  a48=(a48-a24);
  a48=(a48+a41);
  if (res[2]!=0) res[2][11]=a48;
  a5=(a21*a32);
  a5=(a22*a5);
  a7=(a25+a25);
  a7=(a21*a7);
  a7=(a51*a7);
  a5=(a5-a7);
  a7=(a21*a34);
  a7=(a50*a7);
  a5=(a5+a7);
  a7=(a21*a32);
  a7=(a53*a7);
  a6=(a21*a27);
  a6=(a56*a6);
  a7=(a7+a6);
  a5=(a5+a7);
  a7=(a21*a27);
  a7=(a43*a7);
  a6=(a21*a34);
  a6=(a54*a6);
  a7=(a7-a6);
  a6=(a25+a25);
  a6=(a21*a6);
  a6=(a45*a6);
  a7=(a7-a6);
  a5=(a5+a7);
  a5=(a5+a5);
  a5=(a49*a5);
  a7=(a54*a5);
  a7=(a21*a7);
  a6=(a25*a7);
  a6=(a26-a6);
  a1=(a43*a5);
  a1=(a21*a1);
  a3=(a32*a1);
  a6=(a6+a3);
  a3=(a56*a5);
  a3=(a21*a3);
  a2=(a32*a3);
  a6=(a6-a2);
  a2=(a53*a5);
  a2=(a21*a2);
  a47=(a27*a2);
  a6=(a6+a47);
  a47=(a50*a5);
  a47=(a21*a47);
  a63=(a25*a47);
  a63=(a11-a63);
  a6=(a6-a63);
  a63=(a22*a5);
  a63=(a21*a63);
  a62=(a27*a63);
  a6=(a6-a62);
  a6=(a6+a38);
  a6=(a6-a33);
  if (res[2]!=0) res[2][12]=a6;
  a62=(a21*a32);
  a62=(a50*a62);
  a60=(a27+a27);
  a60=(a21*a60);
  a60=(a51*a60);
  a61=(a21*a34);
  a61=(a22*a61);
  a60=(a60+a61);
  a62=(a62-a60);
  a60=(a21*a34);
  a60=(a53*a60);
  a61=(a27+a27);
  a61=(a21*a61);
  a61=(a55*a61);
  a60=(a60-a61);
  a61=(a21*a25);
  a61=(a56*a61);
  a60=(a60+a61);
  a62=(a62+a60);
  a60=(a21*a32);
  a60=(a54*a60);
  a61=(a21*a25);
  a61=(a43*a61);
  a60=(a60+a61);
  a62=(a62+a60);
  a62=(a62+a62);
  a49=(a49*a62);
  a43=(a43*a49);
  a43=(a21*a43);
  a62=(a32*a43);
  a54=(a54*a49);
  a54=(a21*a54);
  a60=(a25*a54);
  a62=(a62-a60);
  a56=(a56*a49);
  a56=(a21*a56);
  a60=(a32*a56);
  a62=(a62-a60);
  a53=(a53*a49);
  a53=(a21*a53);
  a60=(a27*a53);
  a60=(a17-a60);
  a62=(a62-a60);
  a50=(a50*a49);
  a50=(a21*a50);
  a60=(a25*a50);
  a62=(a62+a60);
  a22=(a22*a49);
  a22=(a21*a22);
  a60=(a27*a22);
  a60=(a13-a60);
  a62=(a62+a60);
  a62=(a62-a36);
  a62=(a62+a19);
  if (res[2]!=0) res[2][13]=a62;
  if (res[2]!=0) res[2][14]=a48;
  a48=(a21*a30);
  a60=(a45*a44);
  a60=(a21*a60);
  a60=(a18*a60);
  a48=(a48-a60);
  a0=(a34*a0);
  a48=(a48+a0);
  a46=(a27*a46);
  a48=(a48+a46);
  a28=(a34*a28);
  a48=(a48-a28);
  a28=(a21*a42);
  a44=(a55*a44);
  a44=(a21*a44);
  a44=(a37*a44);
  a28=(a28-a44);
  a48=(a48+a28);
  a8=(a25*a8);
  a48=(a48+a8);
  a10=(a27*a10);
  a48=(a48+a10);
  a9=(a25*a9);
  a48=(a48+a9);
  a9=(a21*a29);
  a48=(a48+a9);
  a9=(a21*a16);
  a48=(a48+a9);
  if (res[2]!=0) res[2][15]=a48;
  a48=(a34*a1);
  a9=(a45*a5);
  a9=(a21*a9);
  a10=(a18*a9);
  a48=(a48-a10);
  a10=(a27*a7);
  a48=(a48+a10);
  a10=(a34*a3);
  a48=(a48-a10);
  a10=(a55*a5);
  a10=(a21*a10);
  a10=(a37*a10);
  a48=(a48-a10);
  a10=(a25*a2);
  a17=(a17-a10);
  a48=(a48-a17);
  a17=(a27*a47);
  a48=(a48+a17);
  a17=(a25*a63);
  a13=(a13-a17);
  a48=(a48-a13);
  a48=(a48-a36);
  a48=(a48-a19);
  if (res[2]!=0) res[2][16]=a48;
  a19=(a34*a43);
  a45=(a45*a49);
  a45=(a21*a45);
  a18=(a18*a45);
  a19=(a19-a18);
  a18=(a27*a54);
  a26=(a26-a18);
  a19=(a19-a26);
  a26=(a34*a56);
  a19=(a19-a26);
  a55=(a55*a49);
  a55=(a21*a55);
  a37=(a37*a55);
  a19=(a19-a37);
  a37=(a25*a53);
  a19=(a19+a37);
  a37=(a27*a50);
  a11=(a11-a37);
  a19=(a19-a11);
  a11=(a25*a22);
  a19=(a19+a11);
  a19=(a19-a38);
  a19=(a19-a33);
  if (res[2]!=0) res[2][17]=a19;
  if (res[2]!=0) res[2][18]=a6;
  if (res[2]!=0) res[2][19]=a48;
  a30=(a21*a30);
  a9=(a35*a9);
  a30=(a30-a9);
  a1=(a27*a1);
  a30=(a30+a1);
  a7=(a34*a7);
  a30=(a30-a7);
  a3=(a27*a3);
  a30=(a30+a3);
  a2=(a32*a2);
  a30=(a30+a2);
  a47=(a34*a47);
  a30=(a30+a47);
  a63=(a32*a63);
  a30=(a30+a63);
  a63=(a21*a52);
  a5=(a51*a5);
  a5=(a21*a5);
  a5=(a40*a5);
  a63=(a63-a5);
  a30=(a30+a63);
  a29=(a21*a29);
  a30=(a30+a29);
  a29=(a21*a12);
  a30=(a30+a29);
  if (res[2]!=0) res[2][20]=a30;
  a30=(a32*a53);
  a35=(a35*a45);
  a45=(a27*a43);
  a20=(a20-a45);
  a35=(a35+a20);
  a20=(a34*a54);
  a35=(a35+a20);
  a27=(a27*a56);
  a15=(a15-a27);
  a35=(a35+a15);
  a30=(a30-a35);
  a35=(a34*a50);
  a30=(a30+a35);
  a35=(a32*a22);
  a30=(a30+a35);
  a51=(a51*a49);
  a51=(a21*a51);
  a40=(a40*a51);
  a30=(a30-a40);
  a30=(a30-a24);
  a30=(a30-a41);
  if (res[2]!=0) res[2][21]=a30;
  if (res[2]!=0) res[2][22]=a62;
  if (res[2]!=0) res[2][23]=a19;
  if (res[2]!=0) res[2][24]=a30;
  a42=(a21*a42);
  a39=(a39*a55);
  a42=(a42-a39);
  a43=(a25*a43);
  a54=(a32*a54);
  a43=(a43+a54);
  a25=(a25*a56);
  a43=(a43+a25);
  a42=(a42+a43);
  a53=(a34*a53);
  a42=(a42+a53);
  a32=(a32*a50);
  a42=(a42+a32);
  a34=(a34*a22);
  a42=(a42-a34);
  a52=(a21*a52);
  a23=(a23*a51);
  a52=(a52-a23);
  a42=(a42+a52);
  a16=(a21*a16);
  a42=(a42+a16);
  a21=(a21*a12);
  a42=(a42+a21);
  if (res[2]!=0) res[2][25]=a42;
  return 0;
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_0_fun_jac_hess(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_0_fun_jac_hess_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_0_fun_jac_hess_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_0_fun_jac_hess_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_0_fun_jac_hess_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_0_fun_jac_hess_release(int mem) {
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_0_fun_jac_hess_incref(void) {
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_0_fun_jac_hess_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int ACADOS_model_cost_ext_cost_0_fun_jac_hess_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int ACADOS_model_cost_ext_cost_0_fun_jac_hess_n_out(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_real ACADOS_model_cost_ext_cost_0_fun_jac_hess_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ACADOS_model_cost_ext_cost_0_fun_jac_hess_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ACADOS_model_cost_ext_cost_0_fun_jac_hess_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    case 3: return "o3";
    case 4: return "o4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ACADOS_model_cost_ext_cost_0_fun_jac_hess_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ACADOS_model_cost_ext_cost_0_fun_jac_hess_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    case 1: return casadi_s5;
    case 2: return casadi_s6;
    case 3: return casadi_s2;
    case 4: return casadi_s7;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_0_fun_jac_hess_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 5;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
