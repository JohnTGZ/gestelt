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
  #define CASADI_PREFIX(ID) ACADOS_model_cost_ext_cost_fun_jac_ ## ID
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

/* ACADOS_model_cost_ext_cost_fun_jac:(i0[10],i1[4],i2[],i3[22])->(o0,o1[14]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a7, a8, a9;
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
  a28=70.;
  a26=-10.;
  a44=arg[3]? arg[3][21] : 0;
  a49=arg[3]? arg[3][20] : 0;
  a44=(a44-a49);
  a44=casadi_sq(a44);
  a26=(a26*a44);
  a26=exp(a26);
  a28=(a28*a26);
  a26=arg[3]? arg[3][14] : 0;
  a1=(a1-a26);
  a26=casadi_sq(a1);
  a44=arg[3]? arg[3][15] : 0;
  a5=(a5-a44);
  a44=casadi_sq(a5);
  a26=(a26+a44);
  a44=arg[3]? arg[3][16] : 0;
  a8=(a8-a44);
  a44=casadi_sq(a8);
  a26=(a26+a44);
  a26=(a0*a26);
  a44=80.;
  a49=arg[3]? arg[3][17] : 0;
  a22=casadi_sq(a49);
  a47=arg[3]? arg[3][18] : 0;
  a31=casadi_sq(a47);
  a22=(a22+a31);
  a31=arg[3]? arg[3][19] : 0;
  a43=casadi_sq(a31);
  a22=(a22+a43);
  a22=sqrt(a22);
  a22=atan(a22);
  a43=sin(a22);
  a45=1.0000000000000000e-08;
  a49=(a49+a45);
  a45=casadi_sq(a49);
  a50=casadi_sq(a47);
  a45=(a45+a50);
  a50=casadi_sq(a31);
  a45=(a45+a50);
  a45=sqrt(a45);
  a47=(a47/a45);
  a47=(a43*a47);
  a50=casadi_sq(a47);
  a31=(a31/a45);
  a31=(a43*a31);
  a51=casadi_sq(a31);
  a50=(a50+a51);
  a50=(a21*a50);
  a50=(a14-a50);
  a51=casadi_sq(a25);
  a52=casadi_sq(a27);
  a51=(a51+a52);
  a51=(a21*a51);
  a51=(a14-a51);
  a51=(a50*a51);
  a49=(a49/a45);
  a43=(a43*a49);
  a49=(a43*a47);
  a22=cos(a22);
  a45=(a22*a31);
  a49=(a49-a45);
  a49=(a21*a49);
  a45=(a32*a25);
  a52=(a34*a27);
  a45=(a45-a52);
  a45=(a21*a45);
  a45=(a49*a45);
  a51=(a51+a45);
  a45=(a43*a31);
  a52=(a22*a47);
  a45=(a45+a52);
  a45=(a21*a45);
  a52=(a32*a27);
  a53=(a34*a25);
  a52=(a52+a53);
  a52=(a21*a52);
  a52=(a45*a52);
  a51=(a51+a52);
  a51=(a14-a51);
  a52=(a43*a47);
  a53=(a22*a31);
  a52=(a52+a53);
  a52=(a21*a52);
  a53=(a32*a25);
  a54=(a34*a27);
  a53=(a53+a54);
  a53=(a21*a53);
  a53=(a52*a53);
  a54=casadi_sq(a43);
  a55=casadi_sq(a31);
  a54=(a54+a55);
  a54=(a21*a54);
  a54=(a14-a54);
  a55=casadi_sq(a32);
  a56=casadi_sq(a27);
  a55=(a55+a56);
  a55=(a21*a55);
  a55=(a14-a55);
  a55=(a54*a55);
  a53=(a53+a55);
  a55=(a47*a31);
  a56=(a22*a43);
  a55=(a55-a56);
  a55=(a21*a55);
  a56=(a25*a27);
  a57=(a34*a32);
  a56=(a56-a57);
  a56=(a21*a56);
  a56=(a55*a56);
  a53=(a53+a56);
  a53=(a14-a53);
  a51=(a51+a53);
  a53=(a43*a31);
  a56=(a22*a47);
  a53=(a53-a56);
  a53=(a21*a53);
  a56=(a32*a27);
  a57=(a34*a25);
  a56=(a56-a57);
  a56=(a21*a56);
  a56=(a53*a56);
  a31=(a47*a31);
  a22=(a22*a43);
  a31=(a31+a22);
  a31=(a21*a31);
  a22=(a25*a27);
  a57=(a34*a32);
  a22=(a22+a57);
  a22=(a21*a22);
  a22=(a31*a22);
  a56=(a56+a22);
  a43=casadi_sq(a43);
  a47=casadi_sq(a47);
  a43=(a43+a47);
  a43=(a21*a43);
  a43=(a14-a43);
  a47=casadi_sq(a32);
  a22=casadi_sq(a25);
  a47=(a47+a22);
  a47=(a21*a47);
  a47=(a14-a47);
  a47=(a43*a47);
  a56=(a56+a47);
  a14=(a14-a56);
  a51=(a51+a14);
  a14=casadi_sq(a51);
  a14=(a44*a14);
  a26=(a26+a14);
  a26=(a28*a26);
  a4=(a4+a26);
  a26=10.;
  a14=arg[1]? arg[1][0] : 0;
  a56=arg[3]? arg[3][10] : 0;
  a56=(a14-a56);
  a47=casadi_sq(a56);
  a22=arg[1]? arg[1][1] : 0;
  a57=arg[3]? arg[3][11] : 0;
  a57=(a22-a57);
  a58=casadi_sq(a57);
  a47=(a47+a58);
  a58=arg[1]? arg[1][2] : 0;
  a59=arg[3]? arg[3][12] : 0;
  a59=(a58-a59);
  a60=casadi_sq(a59);
  a47=(a47+a60);
  a60=arg[1]? arg[1][3] : 0;
  a61=arg[3]? arg[3][13] : 0;
  a61=(a60-a61);
  a62=casadi_sq(a61);
  a47=(a47+a62);
  a47=(a26*a47);
  a4=(a4+a47);
  a14=(a26*a14);
  a47=100.;
  a62=casadi_sq(a22);
  a63=casadi_sq(a58);
  a62=(a62+a63);
  a63=casadi_sq(a60);
  a62=(a62+a63);
  a62=(a47*a62);
  a14=(a14+a62);
  a4=(a4+a14);
  if (res[0]!=0) res[0][0]=a4;
  a56=(a56+a56);
  a56=(a26*a56);
  a56=(a26+a56);
  if (res[1]!=0) res[1][0]=a56;
  a22=(a22+a22);
  a22=(a47*a22);
  a57=(a57+a57);
  a57=(a26*a57);
  a22=(a22+a57);
  if (res[1]!=0) res[1][1]=a22;
  a58=(a58+a58);
  a58=(a47*a58);
  a59=(a59+a59);
  a59=(a26*a59);
  a58=(a58+a59);
  if (res[1]!=0) res[1][2]=a58;
  a60=(a60+a60);
  a47=(a47*a60);
  a61=(a61+a61);
  a26=(a26*a61);
  a47=(a47+a26);
  if (res[1]!=0) res[1][3]=a47;
  a1=(a1+a1);
  a47=(a0*a28);
  a1=(a1*a47);
  a2=(a2+a2);
  a2=(a0*a2);
  a1=(a1+a2);
  a3=(a3+a3);
  a3=(a0*a3);
  a1=(a1+a3);
  if (res[1]!=0) res[1][4]=a1;
  a5=(a5+a5);
  a5=(a5*a47);
  a6=(a6+a6);
  a6=(a0*a6);
  a5=(a5+a6);
  a7=(a7+a7);
  a7=(a0*a7);
  a5=(a5+a7);
  if (res[1]!=0) res[1][5]=a5;
  a8=(a8+a8);
  a8=(a8*a47);
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
  a51=(a51+a51);
  a44=(a44*a28);
  a51=(a51*a44);
  a53=(a53*a51);
  a53=(a21*a53);
  a44=(a25*a53);
  a31=(a31*a51);
  a31=(a21*a31);
  a28=(a32*a31);
  a44=(a44-a28);
  a55=(a55*a51);
  a55=(a21*a55);
  a28=(a32*a55);
  a44=(a44+a28);
  a52=(a52*a51);
  a52=(a21*a52);
  a28=(a27*a52);
  a44=(a44-a28);
  a45=(a45*a51);
  a45=(a21*a45);
  a28=(a25*a45);
  a44=(a44-a28);
  a49=(a49*a51);
  a49=(a21*a49);
  a28=(a27*a49);
  a44=(a44+a28);
  a24=(a24+a46);
  a24=(a21*a24);
  a46=(a32*a24);
  a44=(a44-a46);
  a38=(a38+a39);
  a38=(a21*a38);
  a39=(a25*a38);
  a44=(a44+a39);
  a41=(a41+a42);
  a41=(a21*a41);
  a42=(a32*a41);
  a44=(a44+a42);
  a36=(a36+a37);
  a36=(a21*a36);
  a37=(a27*a36);
  a44=(a44-a37);
  a33=(a33+a35);
  a33=(a21*a33);
  a35=(a25*a33);
  a44=(a44-a35);
  a19=(a19+a30);
  a19=(a21*a19);
  a30=(a27*a19);
  a44=(a44+a30);
  if (res[1]!=0) res[1][10]=a44;
  a44=(a32+a32);
  a43=(a43*a51);
  a43=(a21*a43);
  a44=(a44*a43);
  a30=(a34*a31);
  a44=(a44-a30);
  a30=(a27*a53);
  a44=(a44-a30);
  a30=(a34*a55);
  a44=(a44+a30);
  a30=(a32+a32);
  a54=(a54*a51);
  a54=(a21*a54);
  a30=(a30*a54);
  a44=(a44+a30);
  a30=(a25*a52);
  a44=(a44-a30);
  a30=(a27*a45);
  a44=(a44-a30);
  a30=(a25*a49);
  a44=(a44-a30);
  a30=(a32+a32);
  a29=(a29+a48);
  a29=(a21*a29);
  a30=(a30*a29);
  a44=(a44+a30);
  a30=(a34*a24);
  a44=(a44-a30);
  a30=(a27*a38);
  a44=(a44-a30);
  a30=(a34*a41);
  a44=(a44+a30);
  a30=(a32+a32);
  a16=(a16+a40);
  a16=(a21*a16);
  a30=(a30*a16);
  a44=(a44+a30);
  a30=(a25*a36);
  a44=(a44-a30);
  a30=(a27*a33);
  a44=(a44-a30);
  a30=(a25*a19);
  a44=(a44-a30);
  if (res[1]!=0) res[1][11]=a44;
  a44=(a25+a25);
  a44=(a44*a43);
  a43=(a27*a31);
  a44=(a44-a43);
  a43=(a34*a53);
  a44=(a44+a43);
  a43=(a27*a55);
  a44=(a44-a43);
  a43=(a32*a52);
  a44=(a44-a43);
  a43=(a34*a45);
  a44=(a44-a43);
  a43=(a32*a49);
  a44=(a44-a43);
  a43=(a25+a25);
  a50=(a50*a51);
  a50=(a21*a50);
  a43=(a43*a50);
  a44=(a44+a43);
  a43=(a25+a25);
  a43=(a43*a29);
  a44=(a44+a43);
  a43=(a27*a24);
  a44=(a44-a43);
  a43=(a34*a38);
  a44=(a44+a43);
  a43=(a27*a41);
  a44=(a44-a43);
  a43=(a32*a36);
  a44=(a44-a43);
  a43=(a34*a33);
  a44=(a44-a43);
  a43=(a32*a19);
  a44=(a44-a43);
  a43=(a25+a25);
  a12=(a12+a23);
  a21=(a21*a12);
  a43=(a43*a21);
  a44=(a44+a43);
  if (res[1]!=0) res[1][12]=a44;
  a44=(a27+a27);
  a44=(a44*a54);
  a31=(a25*a31);
  a53=(a32*a53);
  a31=(a31+a53);
  a55=(a25*a55);
  a31=(a31+a55);
  a44=(a44-a31);
  a52=(a34*a52);
  a44=(a44-a52);
  a45=(a32*a45);
  a44=(a44-a45);
  a49=(a34*a49);
  a44=(a44+a49);
  a49=(a27+a27);
  a49=(a49*a50);
  a44=(a44+a49);
  a24=(a25*a24);
  a44=(a44-a24);
  a38=(a32*a38);
  a44=(a44-a38);
  a25=(a25*a41);
  a44=(a44-a25);
  a25=(a27+a27);
  a25=(a25*a16);
  a44=(a44+a25);
  a36=(a34*a36);
  a44=(a44-a36);
  a32=(a32*a33);
  a44=(a44-a32);
  a34=(a34*a19);
  a44=(a44+a34);
  a27=(a27+a27);
  a27=(a27*a21);
  a44=(a44+a27);
  if (res[1]!=0) res[1][13]=a44;
  return 0;
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_fun_jac(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_fun_jac_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_fun_jac_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_fun_jac_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_fun_jac_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_fun_jac_release(int mem) {
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_fun_jac_incref(void) {
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_fun_jac_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int ACADOS_model_cost_ext_cost_fun_jac_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int ACADOS_model_cost_ext_cost_fun_jac_n_out(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_real ACADOS_model_cost_ext_cost_fun_jac_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ACADOS_model_cost_ext_cost_fun_jac_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ACADOS_model_cost_ext_cost_fun_jac_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ACADOS_model_cost_ext_cost_fun_jac_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ACADOS_model_cost_ext_cost_fun_jac_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    case 1: return casadi_s5;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_fun_jac_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
