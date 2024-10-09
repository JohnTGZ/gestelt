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
  #define CASADI_PREFIX(ID) ACADOS_model_cost_ext_cost_0_fun_jac_ ## ID
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
static const casadi_int casadi_s3[22] = {18, 1, 0, 18, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};
static const casadi_int casadi_s4[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s5[18] = {14, 1, 0, 14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};

/* ACADOS_model_cost_ext_cost_0_fun_jac:(i0[10],i1[4],i2[],i3[18])->(o0,o1[14]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a5, a6, a7, a8, a9;
  a0=10.;
  a1=arg[0]? arg[0][0] : 0;
  a2=arg[3]? arg[3][0] : 0;
  a2=(a1-a2);
  a3=casadi_sq(a2);
  a4=arg[0]? arg[0][1] : 0;
  a5=arg[3]? arg[3][1] : 0;
  a5=(a4-a5);
  a6=casadi_sq(a5);
  a3=(a3+a6);
  a6=arg[0]? arg[0][2] : 0;
  a7=arg[3]? arg[3][2] : 0;
  a7=(a6-a7);
  a8=casadi_sq(a7);
  a3=(a3+a8);
  a3=(a0*a3);
  a8=1.0000000000000000e-02;
  a9=arg[0]? arg[0][3] : 0;
  a10=arg[3]? arg[3][3] : 0;
  a9=(a9-a10);
  a10=casadi_sq(a9);
  a11=arg[0]? arg[0][4] : 0;
  a12=arg[3]? arg[3][4] : 0;
  a11=(a11-a12);
  a12=casadi_sq(a11);
  a10=(a10+a12);
  a12=arg[0]? arg[0][5] : 0;
  a13=arg[3]? arg[3][5] : 0;
  a12=(a12-a13);
  a13=casadi_sq(a12);
  a10=(a10+a13);
  a10=(a8*a10);
  a3=(a3+a10);
  a10=1.;
  a13=2.;
  a14=arg[0]? arg[0][8] : 0;
  a15=casadi_sq(a14);
  a16=arg[0]? arg[0][9] : 0;
  a17=casadi_sq(a16);
  a15=(a15+a17);
  a15=(a13*a15);
  a15=(a10-a15);
  a17=arg[3]? arg[3][8] : 0;
  a18=casadi_sq(a17);
  a19=arg[3]? arg[3][9] : 0;
  a20=casadi_sq(a19);
  a18=(a18+a20);
  a18=(a13*a18);
  a18=(a10-a18);
  a15=(a15-a18);
  a18=casadi_sq(a15);
  a20=arg[0]? arg[0][7] : 0;
  a21=(a20*a14);
  a22=arg[0]? arg[0][6] : 0;
  a23=(a22*a16);
  a21=(a21+a23);
  a21=(a13*a21);
  a23=arg[3]? arg[3][7] : 0;
  a24=(a23*a17);
  a25=arg[3]? arg[3][6] : 0;
  a26=(a25*a19);
  a24=(a24+a26);
  a24=(a13*a24);
  a21=(a21-a24);
  a24=casadi_sq(a21);
  a18=(a18+a24);
  a24=(a20*a16);
  a26=(a22*a14);
  a24=(a24-a26);
  a24=(a13*a24);
  a26=(a23*a19);
  a27=(a25*a17);
  a26=(a26-a27);
  a26=(a13*a26);
  a24=(a24-a26);
  a26=casadi_sq(a24);
  a18=(a18+a26);
  a26=(a20*a14);
  a27=(a22*a16);
  a26=(a26-a27);
  a26=(a13*a26);
  a27=(a23*a17);
  a28=(a25*a19);
  a27=(a27-a28);
  a27=(a13*a27);
  a26=(a26-a27);
  a27=casadi_sq(a26);
  a28=casadi_sq(a20);
  a29=casadi_sq(a16);
  a28=(a28+a29);
  a28=(a13*a28);
  a28=(a10-a28);
  a29=casadi_sq(a23);
  a30=casadi_sq(a19);
  a29=(a29+a30);
  a29=(a13*a29);
  a29=(a10-a29);
  a28=(a28-a29);
  a29=casadi_sq(a28);
  a27=(a27+a29);
  a29=(a14*a16);
  a30=(a22*a20);
  a29=(a29+a30);
  a29=(a13*a29);
  a30=(a17*a19);
  a31=(a25*a23);
  a30=(a30+a31);
  a30=(a13*a30);
  a29=(a29-a30);
  a30=casadi_sq(a29);
  a27=(a27+a30);
  a18=(a18+a27);
  a27=(a20*a16);
  a30=(a22*a14);
  a27=(a27+a30);
  a27=(a13*a27);
  a30=(a23*a19);
  a31=(a25*a17);
  a30=(a30+a31);
  a30=(a13*a30);
  a27=(a27-a30);
  a30=casadi_sq(a27);
  a31=(a14*a16);
  a32=(a22*a20);
  a31=(a31-a32);
  a31=(a13*a31);
  a19=(a17*a19);
  a25=(a25*a23);
  a19=(a19-a25);
  a19=(a13*a19);
  a31=(a31-a19);
  a19=casadi_sq(a31);
  a30=(a30+a19);
  a19=casadi_sq(a20);
  a25=casadi_sq(a14);
  a19=(a19+a25);
  a19=(a13*a19);
  a19=(a10-a19);
  a23=casadi_sq(a23);
  a17=casadi_sq(a17);
  a23=(a23+a17);
  a23=(a13*a23);
  a23=(a10-a23);
  a19=(a19-a23);
  a23=casadi_sq(a19);
  a30=(a30+a23);
  a18=(a18+a30);
  a18=(a8*a18);
  a3=(a3+a18);
  a18=100.;
  a30=-10.;
  a23=arg[3]? arg[3][17] : 0;
  a17=arg[3]? arg[3][16] : 0;
  a23=(a23-a17);
  a23=casadi_sq(a23);
  a30=(a30*a23);
  a30=exp(a30);
  a18=(a18*a30);
  a30=5.;
  a23=arg[3]? arg[3][10] : 0;
  a1=(a1-a23);
  a23=casadi_sq(a1);
  a17=arg[3]? arg[3][11] : 0;
  a4=(a4-a17);
  a17=casadi_sq(a4);
  a23=(a23+a17);
  a17=arg[3]? arg[3][12] : 0;
  a6=(a6-a17);
  a17=casadi_sq(a6);
  a23=(a23+a17);
  a23=(a30*a23);
  a17=casadi_sq(a14);
  a25=casadi_sq(a16);
  a17=(a17+a25);
  a17=(a13*a17);
  a17=(a10-a17);
  a25=arg[3]? arg[3][13] : 0;
  a32=casadi_sq(a25);
  a33=arg[3]? arg[3][14] : 0;
  a34=casadi_sq(a33);
  a32=(a32+a34);
  a34=arg[3]? arg[3][15] : 0;
  a35=casadi_sq(a34);
  a32=(a32+a35);
  a32=sqrt(a32);
  a32=atan(a32);
  a35=sin(a32);
  a36=1.0000000000000000e-08;
  a37=(a25+a36);
  a37=casadi_sq(a37);
  a38=casadi_sq(a33);
  a37=(a37+a38);
  a38=casadi_sq(a34);
  a37=(a37+a38);
  a37=sqrt(a37);
  a36=(a36/a37);
  a25=(a25+a36);
  a36=casadi_sq(a25);
  a37=casadi_sq(a33);
  a36=(a36+a37);
  a37=casadi_sq(a34);
  a36=(a36+a37);
  a36=sqrt(a36);
  a33=(a33/a36);
  a33=(a35*a33);
  a37=casadi_sq(a33);
  a34=(a34/a36);
  a34=(a35*a34);
  a38=casadi_sq(a34);
  a37=(a37+a38);
  a37=(a13*a37);
  a37=(a10-a37);
  a17=(a17-a37);
  a37=casadi_sq(a17);
  a38=(a20*a14);
  a39=(a22*a16);
  a38=(a38+a39);
  a38=(a13*a38);
  a25=(a25/a36);
  a35=(a35*a25);
  a25=(a35*a33);
  a32=cos(a32);
  a36=(a32*a34);
  a25=(a25+a36);
  a25=(a13*a25);
  a38=(a38-a25);
  a25=casadi_sq(a38);
  a37=(a37+a25);
  a25=(a20*a16);
  a36=(a22*a14);
  a25=(a25-a36);
  a25=(a13*a25);
  a36=(a35*a34);
  a39=(a32*a33);
  a36=(a36-a39);
  a36=(a13*a36);
  a25=(a25-a36);
  a36=casadi_sq(a25);
  a37=(a37+a36);
  a36=(a20*a14);
  a39=(a22*a16);
  a36=(a36-a39);
  a36=(a13*a36);
  a39=(a35*a33);
  a40=(a32*a34);
  a39=(a39-a40);
  a39=(a13*a39);
  a36=(a36-a39);
  a39=casadi_sq(a36);
  a40=casadi_sq(a20);
  a41=casadi_sq(a16);
  a40=(a40+a41);
  a40=(a13*a40);
  a40=(a10-a40);
  a41=casadi_sq(a35);
  a42=casadi_sq(a34);
  a41=(a41+a42);
  a41=(a13*a41);
  a41=(a10-a41);
  a40=(a40-a41);
  a41=casadi_sq(a40);
  a39=(a39+a41);
  a41=(a14*a16);
  a42=(a22*a20);
  a41=(a41+a42);
  a41=(a13*a41);
  a42=(a33*a34);
  a43=(a32*a35);
  a42=(a42+a43);
  a42=(a13*a42);
  a41=(a41-a42);
  a42=casadi_sq(a41);
  a39=(a39+a42);
  a37=(a37+a39);
  a39=(a20*a16);
  a42=(a22*a14);
  a39=(a39+a42);
  a39=(a13*a39);
  a42=(a35*a34);
  a43=(a32*a33);
  a42=(a42+a43);
  a42=(a13*a42);
  a39=(a39-a42);
  a42=casadi_sq(a39);
  a43=(a14*a16);
  a44=(a22*a20);
  a43=(a43-a44);
  a43=(a13*a43);
  a34=(a33*a34);
  a32=(a32*a35);
  a34=(a34-a32);
  a34=(a13*a34);
  a43=(a43-a34);
  a34=casadi_sq(a43);
  a42=(a42+a34);
  a34=casadi_sq(a20);
  a32=casadi_sq(a14);
  a34=(a34+a32);
  a34=(a13*a34);
  a34=(a10-a34);
  a35=casadi_sq(a35);
  a33=casadi_sq(a33);
  a35=(a35+a33);
  a35=(a13*a35);
  a10=(a10-a35);
  a34=(a34-a10);
  a10=casadi_sq(a34);
  a42=(a42+a10);
  a37=(a37+a42);
  a37=(a0*a37);
  a23=(a23+a37);
  a23=(a18*a23);
  a3=(a3+a23);
  a23=arg[1]? arg[1][0] : 0;
  a37=casadi_sq(a23);
  a42=arg[1]? arg[1][1] : 0;
  a10=casadi_sq(a42);
  a35=arg[1]? arg[1][2] : 0;
  a33=casadi_sq(a35);
  a10=(a10+a33);
  a10=(a30*a10);
  a37=(a37+a10);
  a10=50.;
  a33=arg[1]? arg[1][3] : 0;
  a32=casadi_sq(a33);
  a32=(a10*a32);
  a37=(a37+a32);
  a3=(a3+a37);
  if (res[0]!=0) res[0][0]=a3;
  a23=(a23+a23);
  if (res[1]!=0) res[1][0]=a23;
  a42=(a42+a42);
  a42=(a30*a42);
  if (res[1]!=0) res[1][1]=a42;
  a35=(a35+a35);
  a35=(a30*a35);
  if (res[1]!=0) res[1][2]=a35;
  a33=(a33+a33);
  a10=(a10*a33);
  if (res[1]!=0) res[1][3]=a10;
  a1=(a1+a1);
  a30=(a30*a18);
  a1=(a1*a30);
  a2=(a2+a2);
  a2=(a0*a2);
  a1=(a1+a2);
  if (res[1]!=0) res[1][4]=a1;
  a4=(a4+a4);
  a4=(a4*a30);
  a5=(a5+a5);
  a5=(a0*a5);
  a4=(a4+a5);
  if (res[1]!=0) res[1][5]=a4;
  a6=(a6+a6);
  a6=(a6*a30);
  a7=(a7+a7);
  a7=(a0*a7);
  a6=(a6+a7);
  if (res[1]!=0) res[1][6]=a6;
  a9=(a9+a9);
  a9=(a8*a9);
  if (res[1]!=0) res[1][7]=a9;
  a11=(a11+a11);
  a11=(a8*a11);
  if (res[1]!=0) res[1][8]=a11;
  a12=(a12+a12);
  a12=(a8*a12);
  if (res[1]!=0) res[1][9]=a12;
  a39=(a39+a39);
  a0=(a0*a18);
  a39=(a39*a0);
  a39=(a13*a39);
  a18=(a14*a39);
  a43=(a43+a43);
  a43=(a43*a0);
  a43=(a13*a43);
  a12=(a20*a43);
  a18=(a18-a12);
  a41=(a41+a41);
  a41=(a41*a0);
  a41=(a13*a41);
  a12=(a20*a41);
  a18=(a18+a12);
  a36=(a36+a36);
  a36=(a36*a0);
  a36=(a13*a36);
  a12=(a16*a36);
  a18=(a18-a12);
  a25=(a25+a25);
  a25=(a25*a0);
  a25=(a13*a25);
  a12=(a14*a25);
  a18=(a18-a12);
  a38=(a38+a38);
  a38=(a38*a0);
  a38=(a13*a38);
  a12=(a16*a38);
  a18=(a18+a12);
  a31=(a31+a31);
  a31=(a8*a31);
  a31=(a13*a31);
  a12=(a20*a31);
  a18=(a18-a12);
  a27=(a27+a27);
  a27=(a8*a27);
  a27=(a13*a27);
  a12=(a14*a27);
  a18=(a18+a12);
  a29=(a29+a29);
  a29=(a8*a29);
  a29=(a13*a29);
  a12=(a20*a29);
  a18=(a18+a12);
  a26=(a26+a26);
  a26=(a8*a26);
  a26=(a13*a26);
  a12=(a16*a26);
  a18=(a18-a12);
  a24=(a24+a24);
  a24=(a8*a24);
  a24=(a13*a24);
  a12=(a14*a24);
  a18=(a18-a12);
  a21=(a21+a21);
  a21=(a8*a21);
  a21=(a13*a21);
  a12=(a16*a21);
  a18=(a18+a12);
  if (res[1]!=0) res[1][10]=a18;
  a18=(a16*a39);
  a12=(a20+a20);
  a34=(a34+a34);
  a34=(a34*a0);
  a34=(a13*a34);
  a12=(a12*a34);
  a11=(a22*a43);
  a12=(a12+a11);
  a18=(a18-a12);
  a12=(a22*a41);
  a18=(a18+a12);
  a12=(a20+a20);
  a40=(a40+a40);
  a40=(a40*a0);
  a40=(a13*a40);
  a12=(a12*a40);
  a18=(a18-a12);
  a12=(a14*a36);
  a18=(a18+a12);
  a12=(a16*a25);
  a18=(a18+a12);
  a12=(a14*a38);
  a18=(a18+a12);
  a12=(a20+a20);
  a19=(a19+a19);
  a19=(a8*a19);
  a19=(a13*a19);
  a12=(a12*a19);
  a18=(a18-a12);
  a12=(a22*a31);
  a18=(a18-a12);
  a12=(a16*a27);
  a18=(a18+a12);
  a12=(a22*a29);
  a18=(a18+a12);
  a12=(a20+a20);
  a28=(a28+a28);
  a28=(a8*a28);
  a28=(a13*a28);
  a12=(a12*a28);
  a18=(a18-a12);
  a12=(a14*a26);
  a18=(a18+a12);
  a12=(a16*a24);
  a18=(a18+a12);
  a12=(a14*a21);
  a18=(a18+a12);
  if (res[1]!=0) res[1][11]=a18;
  a18=(a16*a43);
  a12=(a14+a14);
  a12=(a12*a34);
  a18=(a18-a12);
  a12=(a22*a39);
  a18=(a18+a12);
  a12=(a16*a41);
  a18=(a18+a12);
  a12=(a20*a36);
  a18=(a18+a12);
  a12=(a22*a25);
  a18=(a18-a12);
  a12=(a20*a38);
  a18=(a18+a12);
  a12=(a14+a14);
  a17=(a17+a17);
  a17=(a17*a0);
  a17=(a13*a17);
  a12=(a12*a17);
  a18=(a18-a12);
  a12=(a14+a14);
  a12=(a12*a19);
  a18=(a18-a12);
  a12=(a16*a31);
  a18=(a18+a12);
  a12=(a22*a27);
  a18=(a18+a12);
  a12=(a16*a29);
  a18=(a18+a12);
  a12=(a20*a26);
  a18=(a18+a12);
  a12=(a22*a24);
  a18=(a18-a12);
  a12=(a20*a21);
  a18=(a18+a12);
  a12=(a14+a14);
  a15=(a15+a15);
  a8=(a8*a15);
  a13=(a13*a8);
  a12=(a12*a13);
  a18=(a18-a12);
  if (res[1]!=0) res[1][12]=a18;
  a43=(a14*a43);
  a39=(a20*a39);
  a43=(a43+a39);
  a41=(a14*a41);
  a43=(a43+a41);
  a41=(a16+a16);
  a41=(a41*a40);
  a43=(a43-a41);
  a36=(a22*a36);
  a43=(a43-a36);
  a25=(a20*a25);
  a43=(a43+a25);
  a38=(a22*a38);
  a43=(a43+a38);
  a38=(a16+a16);
  a38=(a38*a17);
  a43=(a43-a38);
  a31=(a14*a31);
  a43=(a43+a31);
  a27=(a20*a27);
  a43=(a43+a27);
  a14=(a14*a29);
  a43=(a43+a14);
  a14=(a16+a16);
  a14=(a14*a28);
  a43=(a43-a14);
  a26=(a22*a26);
  a43=(a43-a26);
  a20=(a20*a24);
  a43=(a43+a20);
  a22=(a22*a21);
  a43=(a43+a22);
  a16=(a16+a16);
  a16=(a16*a13);
  a43=(a43-a16);
  if (res[1]!=0) res[1][13]=a43;
  return 0;
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_0_fun_jac(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_0_fun_jac_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_0_fun_jac_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_0_fun_jac_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_0_fun_jac_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_0_fun_jac_release(int mem) {
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_0_fun_jac_incref(void) {
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_0_fun_jac_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int ACADOS_model_cost_ext_cost_0_fun_jac_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int ACADOS_model_cost_ext_cost_0_fun_jac_n_out(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_real ACADOS_model_cost_ext_cost_0_fun_jac_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ACADOS_model_cost_ext_cost_0_fun_jac_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ACADOS_model_cost_ext_cost_0_fun_jac_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ACADOS_model_cost_ext_cost_0_fun_jac_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ACADOS_model_cost_ext_cost_0_fun_jac_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    case 1: return casadi_s5;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_0_fun_jac_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
