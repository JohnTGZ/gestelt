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
  #define CASADI_PREFIX(ID) ACADOS_model_cost_ext_cost_fun_ ## ID
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

/* ACADOS_model_cost_ext_cost_fun:(i0[10],i1[4],i2[],i3[22])->(o0) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a4, a5, a6, a7, a8, a9;
  a0=5.;
  a1=arg[0]? arg[0][0] : 0;
  a2=arg[3]? arg[3][0] : 0;
  a3=(a1-a2);
  a3=casadi_sq(a3);
  a4=arg[0]? arg[0][1] : 0;
  a5=arg[3]? arg[3][1] : 0;
  a6=(a4-a5);
  a6=casadi_sq(a6);
  a3=(a3+a6);
  a6=arg[0]? arg[0][2] : 0;
  a7=arg[3]? arg[3][2] : 0;
  a8=(a6-a7);
  a8=casadi_sq(a8);
  a3=(a3+a8);
  a3=(a0*a3);
  a8=arg[0]? arg[0][3] : 0;
  a9=arg[3]? arg[3][3] : 0;
  a10=(a8-a9);
  a10=casadi_sq(a10);
  a11=arg[0]? arg[0][4] : 0;
  a12=arg[3]? arg[3][4] : 0;
  a13=(a11-a12);
  a13=casadi_sq(a13);
  a10=(a10+a13);
  a13=arg[0]? arg[0][5] : 0;
  a14=arg[3]? arg[3][5] : 0;
  a15=(a13-a14);
  a15=casadi_sq(a15);
  a10=(a10+a15);
  a3=(a3+a10);
  a10=1.;
  a15=2.;
  a16=arg[3]? arg[3][8] : 0;
  a17=casadi_sq(a16);
  a18=arg[3]? arg[3][9] : 0;
  a19=casadi_sq(a18);
  a17=(a17+a19);
  a17=(a15*a17);
  a17=(a10-a17);
  a19=arg[0]? arg[0][8] : 0;
  a20=casadi_sq(a19);
  a21=arg[0]? arg[0][9] : 0;
  a22=casadi_sq(a21);
  a20=(a20+a22);
  a20=(a15*a20);
  a20=(a10-a20);
  a17=(a17*a20);
  a22=arg[3]? arg[3][7] : 0;
  a23=(a22*a16);
  a24=arg[3]? arg[3][6] : 0;
  a25=(a24*a18);
  a23=(a23-a25);
  a23=(a15*a23);
  a25=arg[0]? arg[0][7] : 0;
  a26=(a25*a19);
  a27=arg[0]? arg[0][6] : 0;
  a28=(a27*a21);
  a26=(a26-a28);
  a26=(a15*a26);
  a23=(a23*a26);
  a17=(a17+a23);
  a23=(a22*a18);
  a28=(a24*a16);
  a23=(a23+a28);
  a23=(a15*a23);
  a28=(a25*a21);
  a29=(a27*a19);
  a28=(a28+a29);
  a28=(a15*a28);
  a23=(a23*a28);
  a17=(a17+a23);
  a17=(a10-a17);
  a23=(a22*a16);
  a29=(a24*a18);
  a23=(a23+a29);
  a23=(a15*a23);
  a29=(a25*a19);
  a30=(a27*a21);
  a29=(a29+a30);
  a29=(a15*a29);
  a23=(a23*a29);
  a30=casadi_sq(a22);
  a31=casadi_sq(a18);
  a30=(a30+a31);
  a30=(a15*a30);
  a30=(a10-a30);
  a31=casadi_sq(a25);
  a32=casadi_sq(a21);
  a31=(a31+a32);
  a31=(a15*a31);
  a31=(a10-a31);
  a30=(a30*a31);
  a23=(a23+a30);
  a30=(a16*a18);
  a32=(a24*a22);
  a30=(a30-a32);
  a30=(a15*a30);
  a32=(a19*a21);
  a33=(a27*a25);
  a32=(a32-a33);
  a32=(a15*a32);
  a30=(a30*a32);
  a23=(a23+a30);
  a23=(a10-a23);
  a17=(a17+a23);
  a23=(a22*a18);
  a30=(a24*a16);
  a23=(a23-a30);
  a23=(a15*a23);
  a30=(a25*a21);
  a33=(a27*a19);
  a30=(a30-a33);
  a30=(a15*a30);
  a23=(a23*a30);
  a33=(a16*a18);
  a34=(a24*a22);
  a33=(a33+a34);
  a33=(a15*a33);
  a34=(a19*a21);
  a35=(a27*a25);
  a34=(a34+a35);
  a34=(a15*a34);
  a33=(a33*a34);
  a23=(a23+a33);
  a33=casadi_sq(a22);
  a35=casadi_sq(a16);
  a33=(a33+a35);
  a33=(a15*a33);
  a33=(a10-a33);
  a35=casadi_sq(a25);
  a36=casadi_sq(a19);
  a35=(a35+a36);
  a35=(a15*a35);
  a35=(a10-a35);
  a33=(a33*a35);
  a23=(a23+a33);
  a23=(a10-a23);
  a17=(a17+a23);
  a3=(a3+a17);
  a2=(a1-a2);
  a2=casadi_sq(a2);
  a5=(a4-a5);
  a5=casadi_sq(a5);
  a2=(a2+a5);
  a7=(a6-a7);
  a7=casadi_sq(a7);
  a2=(a2+a7);
  a2=(a0*a2);
  a8=(a8-a9);
  a8=casadi_sq(a8);
  a11=(a11-a12);
  a11=casadi_sq(a11);
  a8=(a8+a11);
  a13=(a13-a14);
  a13=casadi_sq(a13);
  a8=(a8+a13);
  a2=(a2+a8);
  a8=casadi_sq(a16);
  a13=casadi_sq(a18);
  a8=(a8+a13);
  a8=(a15*a8);
  a8=(a10-a8);
  a8=(a8*a20);
  a20=(a22*a16);
  a13=(a24*a18);
  a20=(a20-a13);
  a20=(a15*a20);
  a20=(a20*a26);
  a8=(a8+a20);
  a20=(a22*a18);
  a26=(a24*a16);
  a20=(a20+a26);
  a20=(a15*a20);
  a20=(a20*a28);
  a8=(a8+a20);
  a8=(a10-a8);
  a20=(a22*a16);
  a28=(a24*a18);
  a20=(a20+a28);
  a20=(a15*a20);
  a20=(a20*a29);
  a29=casadi_sq(a22);
  a28=casadi_sq(a18);
  a29=(a29+a28);
  a29=(a15*a29);
  a29=(a10-a29);
  a29=(a29*a31);
  a20=(a20+a29);
  a29=(a16*a18);
  a31=(a24*a22);
  a29=(a29-a31);
  a29=(a15*a29);
  a29=(a29*a32);
  a20=(a20+a29);
  a20=(a10-a20);
  a8=(a8+a20);
  a20=(a22*a18);
  a29=(a24*a16);
  a20=(a20-a29);
  a20=(a15*a20);
  a20=(a20*a30);
  a18=(a16*a18);
  a24=(a24*a22);
  a18=(a18+a24);
  a18=(a15*a18);
  a18=(a18*a34);
  a20=(a20+a18);
  a22=casadi_sq(a22);
  a16=casadi_sq(a16);
  a22=(a22+a16);
  a22=(a15*a22);
  a22=(a10-a22);
  a22=(a22*a35);
  a20=(a20+a22);
  a20=(a10-a20);
  a8=(a8+a20);
  a2=(a2+a8);
  a3=(a3+a2);
  a2=70.;
  a8=-10.;
  a20=arg[3]? arg[3][21] : 0;
  a22=arg[3]? arg[3][20] : 0;
  a20=(a20-a22);
  a20=casadi_sq(a20);
  a8=(a8*a20);
  a8=exp(a8);
  a2=(a2*a8);
  a8=arg[3]? arg[3][14] : 0;
  a1=(a1-a8);
  a1=casadi_sq(a1);
  a8=arg[3]? arg[3][15] : 0;
  a4=(a4-a8);
  a4=casadi_sq(a4);
  a1=(a1+a4);
  a4=arg[3]? arg[3][16] : 0;
  a6=(a6-a4);
  a6=casadi_sq(a6);
  a1=(a1+a6);
  a0=(a0*a1);
  a1=80.;
  a6=arg[3]? arg[3][17] : 0;
  a4=casadi_sq(a6);
  a8=arg[3]? arg[3][18] : 0;
  a20=casadi_sq(a8);
  a4=(a4+a20);
  a20=arg[3]? arg[3][19] : 0;
  a22=casadi_sq(a20);
  a4=(a4+a22);
  a4=sqrt(a4);
  a4=atan(a4);
  a22=sin(a4);
  a35=1.0000000000000000e-08;
  a6=(a6+a35);
  a35=casadi_sq(a6);
  a16=casadi_sq(a8);
  a35=(a35+a16);
  a16=casadi_sq(a20);
  a35=(a35+a16);
  a35=sqrt(a35);
  a8=(a8/a35);
  a8=(a22*a8);
  a16=casadi_sq(a8);
  a20=(a20/a35);
  a20=(a22*a20);
  a18=casadi_sq(a20);
  a16=(a16+a18);
  a16=(a15*a16);
  a16=(a10-a16);
  a18=casadi_sq(a19);
  a34=casadi_sq(a21);
  a18=(a18+a34);
  a18=(a15*a18);
  a18=(a10-a18);
  a16=(a16*a18);
  a6=(a6/a35);
  a22=(a22*a6);
  a6=(a22*a8);
  a4=cos(a4);
  a35=(a4*a20);
  a6=(a6-a35);
  a6=(a15*a6);
  a35=(a25*a19);
  a18=(a27*a21);
  a35=(a35-a18);
  a35=(a15*a35);
  a6=(a6*a35);
  a16=(a16+a6);
  a6=(a22*a20);
  a35=(a4*a8);
  a6=(a6+a35);
  a6=(a15*a6);
  a35=(a25*a21);
  a18=(a27*a19);
  a35=(a35+a18);
  a35=(a15*a35);
  a6=(a6*a35);
  a16=(a16+a6);
  a16=(a10-a16);
  a6=(a22*a8);
  a35=(a4*a20);
  a6=(a6+a35);
  a6=(a15*a6);
  a35=(a25*a19);
  a18=(a27*a21);
  a35=(a35+a18);
  a35=(a15*a35);
  a6=(a6*a35);
  a35=casadi_sq(a22);
  a18=casadi_sq(a20);
  a35=(a35+a18);
  a35=(a15*a35);
  a35=(a10-a35);
  a18=casadi_sq(a25);
  a34=casadi_sq(a21);
  a18=(a18+a34);
  a18=(a15*a18);
  a18=(a10-a18);
  a35=(a35*a18);
  a6=(a6+a35);
  a35=(a8*a20);
  a18=(a4*a22);
  a35=(a35-a18);
  a35=(a15*a35);
  a18=(a19*a21);
  a34=(a27*a25);
  a18=(a18-a34);
  a18=(a15*a18);
  a35=(a35*a18);
  a6=(a6+a35);
  a6=(a10-a6);
  a16=(a16+a6);
  a6=(a22*a20);
  a35=(a4*a8);
  a6=(a6-a35);
  a6=(a15*a6);
  a35=(a25*a21);
  a18=(a27*a19);
  a35=(a35-a18);
  a35=(a15*a35);
  a6=(a6*a35);
  a20=(a8*a20);
  a4=(a4*a22);
  a20=(a20+a4);
  a20=(a15*a20);
  a21=(a19*a21);
  a27=(a27*a25);
  a21=(a21+a27);
  a21=(a15*a21);
  a20=(a20*a21);
  a6=(a6+a20);
  a22=casadi_sq(a22);
  a8=casadi_sq(a8);
  a22=(a22+a8);
  a22=(a15*a22);
  a22=(a10-a22);
  a25=casadi_sq(a25);
  a19=casadi_sq(a19);
  a25=(a25+a19);
  a15=(a15*a25);
  a15=(a10-a15);
  a22=(a22*a15);
  a6=(a6+a22);
  a10=(a10-a6);
  a16=(a16+a10);
  a16=casadi_sq(a16);
  a1=(a1*a16);
  a0=(a0+a1);
  a2=(a2*a0);
  a3=(a3+a2);
  a2=10.;
  a0=arg[1]? arg[1][0] : 0;
  a1=arg[3]? arg[3][10] : 0;
  a1=(a0-a1);
  a1=casadi_sq(a1);
  a16=arg[1]? arg[1][1] : 0;
  a10=arg[3]? arg[3][11] : 0;
  a10=(a16-a10);
  a10=casadi_sq(a10);
  a1=(a1+a10);
  a10=arg[1]? arg[1][2] : 0;
  a6=arg[3]? arg[3][12] : 0;
  a6=(a10-a6);
  a6=casadi_sq(a6);
  a1=(a1+a6);
  a6=arg[1]? arg[1][3] : 0;
  a22=arg[3]? arg[3][13] : 0;
  a22=(a6-a22);
  a22=casadi_sq(a22);
  a1=(a1+a22);
  a1=(a2*a1);
  a3=(a3+a1);
  a2=(a2*a0);
  a0=100.;
  a16=casadi_sq(a16);
  a10=casadi_sq(a10);
  a16=(a16+a10);
  a6=casadi_sq(a6);
  a16=(a16+a6);
  a0=(a0*a16);
  a2=(a2+a0);
  a3=(a3+a2);
  if (res[0]!=0) res[0][0]=a3;
  return 0;
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int ACADOS_model_cost_ext_cost_fun_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int ACADOS_model_cost_ext_cost_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real ACADOS_model_cost_ext_cost_fun_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ACADOS_model_cost_ext_cost_fun_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ACADOS_model_cost_ext_cost_fun_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ACADOS_model_cost_ext_cost_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ACADOS_model_cost_ext_cost_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
