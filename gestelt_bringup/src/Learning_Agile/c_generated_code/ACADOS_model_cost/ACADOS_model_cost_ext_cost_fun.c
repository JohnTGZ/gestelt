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
static const casadi_int casadi_s3[22] = {18, 1, 0, 18, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};
static const casadi_int casadi_s4[5] = {1, 1, 0, 1, 0};

/* ACADOS_model_cost_ext_cost_fun:(i0[10],i1[4],i2[],i3[18])->(o0) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a3, a4, a5, a6, a7, a8, a9;
  a0=10.;
  a1=arg[0]? arg[0][0] : 0;
  a2=arg[3]? arg[3][0] : 0;
  a2=(a1-a2);
  a2=casadi_sq(a2);
  a3=arg[0]? arg[0][1] : 0;
  a4=arg[3]? arg[3][1] : 0;
  a4=(a3-a4);
  a4=casadi_sq(a4);
  a2=(a2+a4);
  a4=arg[0]? arg[0][2] : 0;
  a5=arg[3]? arg[3][2] : 0;
  a5=(a4-a5);
  a5=casadi_sq(a5);
  a2=(a2+a5);
  a2=(a0*a2);
  a5=1.0000000000000000e-02;
  a6=arg[0]? arg[0][3] : 0;
  a7=arg[3]? arg[3][3] : 0;
  a6=(a6-a7);
  a6=casadi_sq(a6);
  a7=arg[0]? arg[0][4] : 0;
  a8=arg[3]? arg[3][4] : 0;
  a7=(a7-a8);
  a7=casadi_sq(a7);
  a6=(a6+a7);
  a7=arg[0]? arg[0][5] : 0;
  a8=arg[3]? arg[3][5] : 0;
  a7=(a7-a8);
  a7=casadi_sq(a7);
  a6=(a6+a7);
  a6=(a5*a6);
  a2=(a2+a6);
  a6=1.;
  a7=2.;
  a8=arg[0]? arg[0][8] : 0;
  a9=casadi_sq(a8);
  a10=arg[0]? arg[0][9] : 0;
  a11=casadi_sq(a10);
  a9=(a9+a11);
  a9=(a7*a9);
  a9=(a6-a9);
  a11=arg[3]? arg[3][8] : 0;
  a12=casadi_sq(a11);
  a13=arg[3]? arg[3][9] : 0;
  a14=casadi_sq(a13);
  a12=(a12+a14);
  a12=(a7*a12);
  a12=(a6-a12);
  a9=(a9-a12);
  a9=casadi_sq(a9);
  a12=arg[0]? arg[0][7] : 0;
  a14=(a12*a8);
  a15=arg[0]? arg[0][6] : 0;
  a16=(a15*a10);
  a14=(a14+a16);
  a14=(a7*a14);
  a16=arg[3]? arg[3][7] : 0;
  a17=(a16*a11);
  a18=arg[3]? arg[3][6] : 0;
  a19=(a18*a13);
  a17=(a17+a19);
  a17=(a7*a17);
  a14=(a14-a17);
  a14=casadi_sq(a14);
  a9=(a9+a14);
  a14=(a12*a10);
  a17=(a15*a8);
  a14=(a14-a17);
  a14=(a7*a14);
  a17=(a16*a13);
  a19=(a18*a11);
  a17=(a17-a19);
  a17=(a7*a17);
  a14=(a14-a17);
  a14=casadi_sq(a14);
  a9=(a9+a14);
  a14=(a12*a8);
  a17=(a15*a10);
  a14=(a14-a17);
  a14=(a7*a14);
  a17=(a16*a11);
  a19=(a18*a13);
  a17=(a17-a19);
  a17=(a7*a17);
  a14=(a14-a17);
  a14=casadi_sq(a14);
  a17=casadi_sq(a12);
  a19=casadi_sq(a10);
  a17=(a17+a19);
  a17=(a7*a17);
  a17=(a6-a17);
  a19=casadi_sq(a16);
  a20=casadi_sq(a13);
  a19=(a19+a20);
  a19=(a7*a19);
  a19=(a6-a19);
  a17=(a17-a19);
  a17=casadi_sq(a17);
  a14=(a14+a17);
  a17=(a8*a10);
  a19=(a15*a12);
  a17=(a17+a19);
  a17=(a7*a17);
  a19=(a11*a13);
  a20=(a18*a16);
  a19=(a19+a20);
  a19=(a7*a19);
  a17=(a17-a19);
  a17=casadi_sq(a17);
  a14=(a14+a17);
  a9=(a9+a14);
  a14=(a12*a10);
  a17=(a15*a8);
  a14=(a14+a17);
  a14=(a7*a14);
  a17=(a16*a13);
  a19=(a18*a11);
  a17=(a17+a19);
  a17=(a7*a17);
  a14=(a14-a17);
  a14=casadi_sq(a14);
  a17=(a8*a10);
  a19=(a15*a12);
  a17=(a17-a19);
  a17=(a7*a17);
  a13=(a11*a13);
  a18=(a18*a16);
  a13=(a13-a18);
  a13=(a7*a13);
  a17=(a17-a13);
  a17=casadi_sq(a17);
  a14=(a14+a17);
  a17=casadi_sq(a12);
  a13=casadi_sq(a8);
  a17=(a17+a13);
  a17=(a7*a17);
  a17=(a6-a17);
  a16=casadi_sq(a16);
  a11=casadi_sq(a11);
  a16=(a16+a11);
  a16=(a7*a16);
  a16=(a6-a16);
  a17=(a17-a16);
  a17=casadi_sq(a17);
  a14=(a14+a17);
  a9=(a9+a14);
  a5=(a5*a9);
  a2=(a2+a5);
  a5=100.;
  a9=-10.;
  a14=arg[3]? arg[3][17] : 0;
  a17=arg[3]? arg[3][16] : 0;
  a14=(a14-a17);
  a14=casadi_sq(a14);
  a9=(a9*a14);
  a9=exp(a9);
  a5=(a5*a9);
  a9=5.;
  a14=arg[3]? arg[3][10] : 0;
  a1=(a1-a14);
  a1=casadi_sq(a1);
  a14=arg[3]? arg[3][11] : 0;
  a3=(a3-a14);
  a3=casadi_sq(a3);
  a1=(a1+a3);
  a3=arg[3]? arg[3][12] : 0;
  a4=(a4-a3);
  a4=casadi_sq(a4);
  a1=(a1+a4);
  a1=(a9*a1);
  a4=casadi_sq(a8);
  a3=casadi_sq(a10);
  a4=(a4+a3);
  a4=(a7*a4);
  a4=(a6-a4);
  a3=arg[3]? arg[3][13] : 0;
  a14=casadi_sq(a3);
  a17=arg[3]? arg[3][14] : 0;
  a16=casadi_sq(a17);
  a14=(a14+a16);
  a16=arg[3]? arg[3][15] : 0;
  a11=casadi_sq(a16);
  a14=(a14+a11);
  a14=sqrt(a14);
  a14=atan(a14);
  a11=sin(a14);
  a13=1.0000000000000000e-08;
  a18=(a3+a13);
  a18=casadi_sq(a18);
  a19=casadi_sq(a17);
  a18=(a18+a19);
  a19=casadi_sq(a16);
  a18=(a18+a19);
  a18=sqrt(a18);
  a13=(a13/a18);
  a3=(a3+a13);
  a13=casadi_sq(a3);
  a18=casadi_sq(a17);
  a13=(a13+a18);
  a18=casadi_sq(a16);
  a13=(a13+a18);
  a13=sqrt(a13);
  a17=(a17/a13);
  a17=(a11*a17);
  a18=casadi_sq(a17);
  a16=(a16/a13);
  a16=(a11*a16);
  a19=casadi_sq(a16);
  a18=(a18+a19);
  a18=(a7*a18);
  a18=(a6-a18);
  a4=(a4-a18);
  a4=casadi_sq(a4);
  a18=(a12*a8);
  a19=(a15*a10);
  a18=(a18+a19);
  a18=(a7*a18);
  a3=(a3/a13);
  a11=(a11*a3);
  a3=(a11*a17);
  a14=cos(a14);
  a13=(a14*a16);
  a3=(a3+a13);
  a3=(a7*a3);
  a18=(a18-a3);
  a18=casadi_sq(a18);
  a4=(a4+a18);
  a18=(a12*a10);
  a3=(a15*a8);
  a18=(a18-a3);
  a18=(a7*a18);
  a3=(a11*a16);
  a13=(a14*a17);
  a3=(a3-a13);
  a3=(a7*a3);
  a18=(a18-a3);
  a18=casadi_sq(a18);
  a4=(a4+a18);
  a18=(a12*a8);
  a3=(a15*a10);
  a18=(a18-a3);
  a18=(a7*a18);
  a3=(a11*a17);
  a13=(a14*a16);
  a3=(a3-a13);
  a3=(a7*a3);
  a18=(a18-a3);
  a18=casadi_sq(a18);
  a3=casadi_sq(a12);
  a13=casadi_sq(a10);
  a3=(a3+a13);
  a3=(a7*a3);
  a3=(a6-a3);
  a13=casadi_sq(a11);
  a19=casadi_sq(a16);
  a13=(a13+a19);
  a13=(a7*a13);
  a13=(a6-a13);
  a3=(a3-a13);
  a3=casadi_sq(a3);
  a18=(a18+a3);
  a3=(a8*a10);
  a13=(a15*a12);
  a3=(a3+a13);
  a3=(a7*a3);
  a13=(a17*a16);
  a19=(a14*a11);
  a13=(a13+a19);
  a13=(a7*a13);
  a3=(a3-a13);
  a3=casadi_sq(a3);
  a18=(a18+a3);
  a4=(a4+a18);
  a18=(a12*a10);
  a3=(a15*a8);
  a18=(a18+a3);
  a18=(a7*a18);
  a3=(a11*a16);
  a13=(a14*a17);
  a3=(a3+a13);
  a3=(a7*a3);
  a18=(a18-a3);
  a18=casadi_sq(a18);
  a10=(a8*a10);
  a15=(a15*a12);
  a10=(a10-a15);
  a10=(a7*a10);
  a16=(a17*a16);
  a14=(a14*a11);
  a16=(a16-a14);
  a16=(a7*a16);
  a10=(a10-a16);
  a10=casadi_sq(a10);
  a18=(a18+a10);
  a12=casadi_sq(a12);
  a8=casadi_sq(a8);
  a12=(a12+a8);
  a12=(a7*a12);
  a12=(a6-a12);
  a11=casadi_sq(a11);
  a17=casadi_sq(a17);
  a11=(a11+a17);
  a7=(a7*a11);
  a6=(a6-a7);
  a12=(a12-a6);
  a12=casadi_sq(a12);
  a18=(a18+a12);
  a4=(a4+a18);
  a0=(a0*a4);
  a1=(a1+a0);
  a5=(a5*a1);
  a2=(a2+a5);
  a5=arg[1]? arg[1][0] : 0;
  a5=casadi_sq(a5);
  a1=arg[1]? arg[1][1] : 0;
  a1=casadi_sq(a1);
  a0=arg[1]? arg[1][2] : 0;
  a0=casadi_sq(a0);
  a1=(a1+a0);
  a9=(a9*a1);
  a5=(a5+a9);
  a9=50.;
  a1=arg[1]? arg[1][3] : 0;
  a1=casadi_sq(a1);
  a9=(a9*a1);
  a5=(a5+a9);
  a2=(a2+a5);
  if (res[0]!=0) res[0][0]=a2;
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
