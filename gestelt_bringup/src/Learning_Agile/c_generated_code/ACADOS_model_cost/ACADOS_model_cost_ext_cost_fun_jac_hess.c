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
  #define CASADI_PREFIX(ID) ACADOS_model_cost_ext_cost_fun_jac_hess_ ## ID
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

static const casadi_int casadi_s0[17] = {13, 1, 0, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
static const casadi_int casadi_s1[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[22] = {18, 1, 0, 18, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};
static const casadi_int casadi_s4[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s5[21] = {17, 1, 0, 17, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
static const casadi_int casadi_s6[49] = {17, 17, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 14, 18, 22, 26, 27, 28, 29, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 10, 11, 12, 13, 10, 11, 12, 13, 10, 11, 12, 13, 14, 15, 16};
static const casadi_int casadi_s7[20] = {0, 17, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* ACADOS_model_cost_ext_cost_fun_jac_hess:(i0[13],i1[4],i2[],i3[18])->(o0,o1[17],o2[17x17,29nz],o3[],o4[0x17]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a4, a5, a6, a7, a8, a9;
  a0=5.;
  a1=arg[0]? arg[0][0] : 0;
  a2=2.;
  a3=(a1-a2);
  a4=casadi_sq(a3);
  a5=arg[0]? arg[0][1] : 0;
  a6=-1.8000000000000000e+00;
  a6=(a5-a6);
  a7=casadi_sq(a6);
  a4=(a4+a7);
  a7=arg[0]? arg[0][2] : 0;
  a8=1.;
  a9=(a7-a8);
  a10=casadi_sq(a9);
  a4=(a4+a10);
  a10=(a0*a4);
  a11=arg[0]? arg[0][3] : 0;
  a12=casadi_sq(a11);
  a13=arg[0]? arg[0][4] : 0;
  a14=casadi_sq(a13);
  a12=(a12+a14);
  a14=arg[0]? arg[0][5] : 0;
  a15=casadi_sq(a14);
  a12=(a12+a15);
  a15=(a0*a12);
  a10=(a10+a15);
  a15=3.;
  a16=arg[0]? arg[0][10] : 0;
  a17=casadi_sq(a16);
  a18=arg[0]? arg[0][11] : 0;
  a19=casadi_sq(a18);
  a17=(a17+a19);
  a19=arg[0]? arg[0][12] : 0;
  a20=casadi_sq(a19);
  a17=(a17+a20);
  a20=(a15*a17);
  a10=(a10+a20);
  a20=1.0000000000000001e-01;
  a21=arg[1]? arg[1][0] : 0;
  a22=casadi_sq(a21);
  a23=arg[1]? arg[1][1] : 0;
  a24=casadi_sq(a23);
  a22=(a22+a24);
  a24=arg[1]? arg[1][2] : 0;
  a25=casadi_sq(a24);
  a22=(a22+a25);
  a25=arg[1]? arg[1][3] : 0;
  a26=casadi_sq(a25);
  a22=(a22+a26);
  a22=(a20*a22);
  a10=(a10+a22);
  a4=(a0*a4);
  a12=(a0*a12);
  a4=(a4+a12);
  a15=(a15*a17);
  a4=(a4+a15);
  a10=(a10+a4);
  a4=arg[3]? arg[3][17] : 0;
  a15=casadi_sq(a1);
  a17=casadi_sq(a5);
  a15=(a15+a17);
  a17=casadi_sq(a7);
  a15=(a15+a17);
  a15=(a0*a15);
  a17=8.;
  a12=7.6484218728448838e-01;
  a22=arg[0]? arg[0][8] : 0;
  a26=casadi_sq(a22);
  a27=arg[0]? arg[0][9] : 0;
  a28=casadi_sq(a27);
  a26=(a26+a28);
  a26=(a2*a26);
  a26=(a8-a26);
  a26=(a12*a26);
  a28=6.4421768723769102e-01;
  a29=arg[0]? arg[0][7] : 0;
  a30=(a29*a27);
  a31=arg[0]? arg[0][6] : 0;
  a32=(a31*a22);
  a30=(a30+a32);
  a30=(a2*a30);
  a30=(a28*a30);
  a26=(a26+a30);
  a26=(a8-a26);
  a30=casadi_sq(a29);
  a32=casadi_sq(a27);
  a30=(a30+a32);
  a30=(a2*a30);
  a30=(a8-a30);
  a30=(a8-a30);
  a26=(a26+a30);
  a30=-6.4421768723769102e-01;
  a32=(a29*a27);
  a33=(a31*a22);
  a32=(a32-a33);
  a32=(a2*a32);
  a32=(a30*a32);
  a33=casadi_sq(a29);
  a34=casadi_sq(a22);
  a33=(a33+a34);
  a33=(a2*a33);
  a33=(a8-a33);
  a33=(a12*a33);
  a32=(a32+a33);
  a8=(a8-a32);
  a26=(a26+a8);
  a8=casadi_sq(a26);
  a8=(a17*a8);
  a15=(a15+a8);
  a15=(a4*a15);
  a10=(a10+a15);
  if (res[0]!=0) res[0][0]=a10;
  a21=(a21+a21);
  a21=(a20*a21);
  if (res[1]!=0) res[1][0]=a21;
  a23=(a23+a23);
  a23=(a20*a23);
  if (res[1]!=0) res[1][1]=a23;
  a24=(a24+a24);
  a24=(a20*a24);
  if (res[1]!=0) res[1][2]=a24;
  a25=(a25+a25);
  a20=(a20*a25);
  if (res[1]!=0) res[1][3]=a20;
  a1=(a1+a1);
  a0=(a0*a4);
  a1=(a1*a0);
  a20=10.;
  a3=(a3+a3);
  a3=(a20*a3);
  a1=(a1+a3);
  if (res[1]!=0) res[1][4]=a1;
  a5=(a5+a5);
  a5=(a5*a0);
  a6=(a6+a6);
  a6=(a20*a6);
  a5=(a5+a6);
  if (res[1]!=0) res[1][5]=a5;
  a7=(a7+a7);
  a7=(a7*a0);
  a9=(a9+a9);
  a9=(a20*a9);
  a7=(a7+a9);
  if (res[1]!=0) res[1][6]=a7;
  a11=(a11+a11);
  a11=(a20*a11);
  if (res[1]!=0) res[1][7]=a11;
  a13=(a13+a13);
  a13=(a20*a13);
  if (res[1]!=0) res[1][8]=a13;
  a14=(a14+a14);
  a20=(a20*a14);
  if (res[1]!=0) res[1][9]=a20;
  a26=(a26+a26);
  a17=(a17*a4);
  a26=(a26*a17);
  a4=(a30*a26);
  a4=(a2*a4);
  a20=(a22*a4);
  a14=(a28*a26);
  a14=(a2*a14);
  a13=(a22*a14);
  a20=(a20-a13);
  if (res[1]!=0) res[1][10]=a20;
  a20=(a29+a29);
  a13=(a12*a26);
  a13=(a2*a13);
  a11=(a20*a13);
  a7=(a27*a4);
  a11=(a11-a7);
  a7=(a29+a29);
  a9=(a2*a26);
  a5=(a7*a9);
  a11=(a11+a5);
  a5=(a27*a14);
  a11=(a11-a5);
  if (res[1]!=0) res[1][11]=a11;
  a11=(a22+a22);
  a5=(a11*a13);
  a6=(a31*a4);
  a5=(a5+a6);
  a6=(a31*a14);
  a5=(a5-a6);
  a6=(a22+a22);
  a26=(a12*a26);
  a26=(a2*a26);
  a1=(a6*a26);
  a5=(a5+a1);
  if (res[1]!=0) res[1][12]=a5;
  a5=(a27+a27);
  a1=(a5*a9);
  a3=(a29*a4);
  a1=(a1-a3);
  a3=(a29*a14);
  a1=(a1-a3);
  a3=(a27+a27);
  a25=(a3*a26);
  a1=(a1+a25);
  if (res[1]!=0) res[1][13]=a1;
  a1=6.;
  a16=(a16+a16);
  a16=(a1*a16);
  if (res[1]!=0) res[1][14]=a16;
  a18=(a18+a18);
  a18=(a1*a18);
  if (res[1]!=0) res[1][15]=a18;
  a19=(a19+a19);
  a1=(a1*a19);
  if (res[1]!=0) res[1][16]=a1;
  a1=2.0000000000000001e-01;
  if (res[2]!=0) res[2][0]=a1;
  if (res[2]!=0) res[2][1]=a1;
  if (res[2]!=0) res[2][2]=a1;
  if (res[2]!=0) res[2][3]=a1;
  a1=(a2*a0);
  a19=20.;
  a1=(a1+a19);
  if (res[2]!=0) res[2][4]=a1;
  a1=(a2*a0);
  a1=(a1+a19);
  if (res[2]!=0) res[2][5]=a1;
  a0=(a2*a0);
  a0=(a0+a19);
  if (res[2]!=0) res[2][6]=a0;
  if (res[2]!=0) res[2][7]=a19;
  if (res[2]!=0) res[2][8]=a19;
  if (res[2]!=0) res[2][9]=a19;
  a19=(a2*a22);
  a19=(a30*a19);
  a0=(a2*a22);
  a0=(a28*a0);
  a19=(a19-a0);
  a19=(a19+a19);
  a19=(a17*a19);
  a0=(a30*a19);
  a0=(a2*a0);
  a0=(a22*a0);
  a19=(a28*a19);
  a19=(a2*a19);
  a19=(a22*a19);
  a0=(a0-a19);
  if (res[2]!=0) res[2][10]=a0;
  a0=(a29+a29);
  a0=(a2*a0);
  a19=(a2*a27);
  a19=(a28*a19);
  a0=(a0-a19);
  a19=(a2*a27);
  a19=(a30*a19);
  a1=(a29+a29);
  a1=(a2*a1);
  a1=(a12*a1);
  a19=(a19-a1);
  a0=(a0-a19);
  a0=(a0+a0);
  a0=(a17*a0);
  a19=(a30*a0);
  a19=(a2*a19);
  a1=(a22*a19);
  a18=(a28*a0);
  a18=(a2*a18);
  a16=(a22*a18);
  a1=(a1-a16);
  if (res[2]!=0) res[2][11]=a1;
  a16=(a2*a31);
  a16=(a30*a16);
  a25=(a22+a22);
  a25=(a2*a25);
  a25=(a12*a25);
  a16=(a16+a25);
  a25=(a2*a31);
  a25=(a28*a25);
  a24=(a22+a22);
  a24=(a2*a24);
  a24=(a12*a24);
  a25=(a25-a24);
  a16=(a16-a25);
  a16=(a16+a16);
  a16=(a17*a16);
  a25=(a30*a16);
  a25=(a2*a25);
  a24=(a22*a25);
  a24=(a4+a24);
  a23=(a28*a16);
  a23=(a2*a23);
  a21=(a22*a23);
  a21=(a14+a21);
  a24=(a24-a21);
  if (res[2]!=0) res[2][12]=a24;
  a21=(a27+a27);
  a21=(a2*a21);
  a10=(a2*a29);
  a10=(a28*a10);
  a15=(a27+a27);
  a15=(a2*a15);
  a15=(a12*a15);
  a10=(a10-a15);
  a21=(a21-a10);
  a10=(a2*a29);
  a10=(a30*a10);
  a21=(a21-a10);
  a21=(a21+a21);
  a17=(a17*a21);
  a30=(a30*a17);
  a30=(a2*a30);
  a21=(a22*a30);
  a28=(a28*a17);
  a28=(a2*a28);
  a22=(a22*a28);
  a21=(a21-a22);
  if (res[2]!=0) res[2][13]=a21;
  if (res[2]!=0) res[2][14]=a1;
  a1=(a2*a13);
  a22=(a12*a0);
  a22=(a2*a22);
  a22=(a20*a22);
  a1=(a1+a22);
  a19=(a27*a19);
  a1=(a1-a19);
  a19=(a2*a9);
  a0=(a2*a0);
  a0=(a7*a0);
  a19=(a19+a0);
  a1=(a1+a19);
  a18=(a27*a18);
  a1=(a1-a18);
  if (res[2]!=0) res[2][15]=a1;
  a1=(a12*a16);
  a1=(a2*a1);
  a18=(a20*a1);
  a19=(a27*a25);
  a18=(a18-a19);
  a19=(a2*a16);
  a19=(a7*a19);
  a18=(a18+a19);
  a19=(a27*a23);
  a18=(a18-a19);
  if (res[2]!=0) res[2][16]=a18;
  a19=(a12*a17);
  a19=(a2*a19);
  a20=(a20*a19);
  a0=(a27*a30);
  a4=(a4+a0);
  a20=(a20-a4);
  a4=(a2*a17);
  a7=(a7*a4);
  a20=(a20+a7);
  a27=(a27*a28);
  a14=(a14+a27);
  a20=(a20-a14);
  if (res[2]!=0) res[2][17]=a20;
  if (res[2]!=0) res[2][18]=a24;
  if (res[2]!=0) res[2][19]=a18;
  a13=(a2*a13);
  a1=(a11*a1);
  a13=(a13+a1);
  a25=(a31*a25);
  a13=(a13+a25);
  a23=(a31*a23);
  a13=(a13-a23);
  a23=(a2*a26);
  a16=(a12*a16);
  a16=(a2*a16);
  a16=(a6*a16);
  a23=(a23+a16);
  a13=(a13+a23);
  if (res[2]!=0) res[2][20]=a13;
  a11=(a11*a19);
  a19=(a31*a30);
  a11=(a11+a19);
  a31=(a31*a28);
  a11=(a11-a31);
  a12=(a12*a17);
  a12=(a2*a12);
  a6=(a6*a12);
  a11=(a11+a6);
  if (res[2]!=0) res[2][21]=a11;
  if (res[2]!=0) res[2][22]=a21;
  if (res[2]!=0) res[2][23]=a20;
  if (res[2]!=0) res[2][24]=a11;
  a9=(a2*a9);
  a5=(a5*a4);
  a9=(a9+a5);
  a30=(a29*a30);
  a9=(a9-a30);
  a29=(a29*a28);
  a9=(a9-a29);
  a2=(a2*a26);
  a3=(a3*a12);
  a2=(a2+a3);
  a9=(a9+a2);
  if (res[2]!=0) res[2][25]=a9;
  a9=12.;
  if (res[2]!=0) res[2][26]=a9;
  if (res[2]!=0) res[2][27]=a9;
  if (res[2]!=0) res[2][28]=a9;
  return 0;
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_fun_jac_hess(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_fun_jac_hess_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_fun_jac_hess_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_fun_jac_hess_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_fun_jac_hess_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_fun_jac_hess_release(int mem) {
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_fun_jac_hess_incref(void) {
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_fun_jac_hess_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int ACADOS_model_cost_ext_cost_fun_jac_hess_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int ACADOS_model_cost_ext_cost_fun_jac_hess_n_out(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_real ACADOS_model_cost_ext_cost_fun_jac_hess_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ACADOS_model_cost_ext_cost_fun_jac_hess_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ACADOS_model_cost_ext_cost_fun_jac_hess_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    case 3: return "o3";
    case 4: return "o4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ACADOS_model_cost_ext_cost_fun_jac_hess_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ACADOS_model_cost_ext_cost_fun_jac_hess_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    case 1: return casadi_s5;
    case 2: return casadi_s6;
    case 3: return casadi_s2;
    case 4: return casadi_s7;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_fun_jac_hess_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 5;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
