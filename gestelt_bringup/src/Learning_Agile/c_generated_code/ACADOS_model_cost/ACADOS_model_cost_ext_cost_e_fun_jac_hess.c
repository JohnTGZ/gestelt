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
  #define CASADI_PREFIX(ID) ACADOS_model_cost_ext_cost_e_fun_jac_hess_ ## ID
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
static const casadi_int casadi_s1[3] = {0, 0, 0};
static const casadi_int casadi_s2[26] = {22, 1, 0, 22, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21};
static const casadi_int casadi_s3[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s4[34] = {10, 10, 0, 1, 2, 3, 4, 5, 6, 9, 13, 17, 21, 0, 1, 2, 3, 4, 5, 7, 8, 9, 6, 7, 8, 9, 6, 7, 8, 9, 6, 7, 8, 9};
static const casadi_int casadi_s5[13] = {0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* ACADOS_model_cost_ext_cost_e_fun_jac_hess:(i0[10],i1[],i2[],i3[22])->(o0,o1[10],o2[10x10,21nz],o3[],o4[0x10]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a3, a4, a5, a6, a7, a8, a9;
  a0=5.;
  a1=arg[0]? arg[0][0] : 0;
  a2=arg[3]? arg[3][0] : 0;
  a1=(a1-a2);
  a2=casadi_sq(a1);
  a3=arg[0]? arg[0][1] : 0;
  a4=arg[3]? arg[3][1] : 0;
  a3=(a3-a4);
  a4=casadi_sq(a3);
  a2=(a2+a4);
  a4=arg[0]? arg[0][2] : 0;
  a5=arg[3]? arg[3][2] : 0;
  a4=(a4-a5);
  a5=casadi_sq(a4);
  a2=(a2+a5);
  a2=(a0*a2);
  a5=arg[0]? arg[0][3] : 0;
  a6=arg[3]? arg[3][3] : 0;
  a5=(a5-a6);
  a6=casadi_sq(a5);
  a7=arg[0]? arg[0][4] : 0;
  a8=arg[3]? arg[3][4] : 0;
  a7=(a7-a8);
  a8=casadi_sq(a7);
  a6=(a6+a8);
  a8=arg[0]? arg[0][5] : 0;
  a9=arg[3]? arg[3][5] : 0;
  a8=(a8-a9);
  a9=casadi_sq(a8);
  a6=(a6+a9);
  a6=(a0*a6);
  a2=(a2+a6);
  a6=1.;
  a9=2.;
  a10=arg[3]? arg[3][8] : 0;
  a11=casadi_sq(a10);
  a12=arg[3]? arg[3][9] : 0;
  a13=casadi_sq(a12);
  a11=(a11+a13);
  a11=(a9*a11);
  a11=(a6-a11);
  a13=arg[0]? arg[0][8] : 0;
  a14=casadi_sq(a13);
  a15=arg[0]? arg[0][9] : 0;
  a16=casadi_sq(a15);
  a14=(a14+a16);
  a14=(a9*a14);
  a14=(a6-a14);
  a14=(a11*a14);
  a16=arg[3]? arg[3][7] : 0;
  a17=(a16*a10);
  a18=arg[3]? arg[3][6] : 0;
  a19=(a18*a12);
  a17=(a17-a19);
  a17=(a9*a17);
  a19=arg[0]? arg[0][7] : 0;
  a20=(a19*a13);
  a21=arg[0]? arg[0][6] : 0;
  a22=(a21*a15);
  a20=(a20-a22);
  a20=(a9*a20);
  a20=(a17*a20);
  a14=(a14+a20);
  a20=(a16*a12);
  a22=(a18*a10);
  a20=(a20+a22);
  a20=(a9*a20);
  a22=(a19*a15);
  a23=(a21*a13);
  a22=(a22+a23);
  a22=(a9*a22);
  a22=(a20*a22);
  a14=(a14+a22);
  a14=(a6-a14);
  a22=(a16*a10);
  a23=(a18*a12);
  a22=(a22+a23);
  a22=(a9*a22);
  a23=(a19*a13);
  a24=(a21*a15);
  a23=(a23+a24);
  a23=(a9*a23);
  a23=(a22*a23);
  a24=casadi_sq(a16);
  a25=casadi_sq(a12);
  a24=(a24+a25);
  a24=(a9*a24);
  a24=(a6-a24);
  a25=casadi_sq(a19);
  a26=casadi_sq(a15);
  a25=(a25+a26);
  a25=(a9*a25);
  a25=(a6-a25);
  a25=(a24*a25);
  a23=(a23+a25);
  a25=(a10*a12);
  a26=(a18*a16);
  a25=(a25-a26);
  a25=(a9*a25);
  a26=(a13*a15);
  a27=(a21*a19);
  a26=(a26-a27);
  a26=(a9*a26);
  a26=(a25*a26);
  a23=(a23+a26);
  a23=(a6-a23);
  a14=(a14+a23);
  a23=(a16*a12);
  a26=(a18*a10);
  a23=(a23-a26);
  a23=(a9*a23);
  a26=(a19*a15);
  a27=(a21*a13);
  a26=(a26-a27);
  a26=(a9*a26);
  a26=(a23*a26);
  a12=(a10*a12);
  a18=(a18*a16);
  a12=(a12+a18);
  a12=(a9*a12);
  a18=(a13*a15);
  a27=(a21*a19);
  a18=(a18+a27);
  a18=(a9*a18);
  a18=(a12*a18);
  a26=(a26+a18);
  a16=casadi_sq(a16);
  a10=casadi_sq(a10);
  a16=(a16+a10);
  a16=(a9*a16);
  a16=(a6-a16);
  a10=casadi_sq(a19);
  a18=casadi_sq(a13);
  a10=(a10+a18);
  a10=(a9*a10);
  a10=(a6-a10);
  a10=(a16*a10);
  a26=(a26+a10);
  a6=(a6-a26);
  a14=(a14+a6);
  a2=(a2+a14);
  if (res[0]!=0) res[0][0]=a2;
  a1=(a1+a1);
  a1=(a0*a1);
  if (res[1]!=0) res[1][0]=a1;
  a3=(a3+a3);
  a3=(a0*a3);
  if (res[1]!=0) res[1][1]=a3;
  a4=(a4+a4);
  a4=(a0*a4);
  if (res[1]!=0) res[1][2]=a4;
  a5=(a5+a5);
  a5=(a0*a5);
  if (res[1]!=0) res[1][3]=a5;
  a7=(a7+a7);
  a7=(a0*a7);
  if (res[1]!=0) res[1][4]=a7;
  a8=(a8+a8);
  a0=(a0*a8);
  if (res[1]!=0) res[1][5]=a0;
  a23=(a9*a23);
  a0=(a13*a23);
  a12=(a9*a12);
  a8=(a19*a12);
  a0=(a0-a8);
  a25=(a9*a25);
  a8=(a19*a25);
  a0=(a0+a8);
  a22=(a9*a22);
  a8=(a15*a22);
  a0=(a0-a8);
  a20=(a9*a20);
  a8=(a13*a20);
  a0=(a0-a8);
  a17=(a9*a17);
  a8=(a15*a17);
  a0=(a0+a8);
  if (res[1]!=0) res[1][6]=a0;
  a0=(a19+a19);
  a16=(a9*a16);
  a0=(a0*a16);
  a8=(a21*a12);
  a0=(a0-a8);
  a8=(a15*a23);
  a0=(a0-a8);
  a8=(a21*a25);
  a0=(a0+a8);
  a8=(a19+a19);
  a24=(a9*a24);
  a8=(a8*a24);
  a0=(a0+a8);
  a8=(a13*a22);
  a0=(a0-a8);
  a8=(a15*a20);
  a0=(a0-a8);
  a8=(a13*a17);
  a0=(a0-a8);
  if (res[1]!=0) res[1][7]=a0;
  a0=(a13+a13);
  a0=(a0*a16);
  a8=(a15*a12);
  a0=(a0-a8);
  a8=(a21*a23);
  a0=(a0+a8);
  a8=(a15*a25);
  a0=(a0-a8);
  a8=(a19*a22);
  a0=(a0-a8);
  a8=(a21*a20);
  a0=(a0-a8);
  a8=(a19*a17);
  a0=(a0-a8);
  a8=(a13+a13);
  a11=(a9*a11);
  a8=(a8*a11);
  a0=(a0+a8);
  if (res[1]!=0) res[1][8]=a0;
  a0=(a15+a15);
  a0=(a0*a24);
  a8=(a13*a12);
  a7=(a19*a23);
  a8=(a8+a7);
  a13=(a13*a25);
  a8=(a8+a13);
  a0=(a0-a8);
  a8=(a21*a22);
  a0=(a0-a8);
  a19=(a19*a20);
  a0=(a0-a19);
  a21=(a21*a17);
  a0=(a0+a21);
  a15=(a15+a15);
  a15=(a15*a11);
  a0=(a0+a15);
  if (res[1]!=0) res[1][9]=a0;
  a0=10.;
  if (res[2]!=0) res[2][0]=a0;
  if (res[2]!=0) res[2][1]=a0;
  if (res[2]!=0) res[2][2]=a0;
  if (res[2]!=0) res[2][3]=a0;
  if (res[2]!=0) res[2][4]=a0;
  if (res[2]!=0) res[2][5]=a0;
  a0=(a25-a12);
  if (res[2]!=0) res[2][6]=a0;
  a15=(a23-a20);
  if (res[2]!=0) res[2][7]=a15;
  a21=(a17-a22);
  if (res[2]!=0) res[2][8]=a21;
  if (res[2]!=0) res[2][9]=a0;
  a0=(a9*a16);
  a19=(a9*a24);
  a0=(a0+a19);
  if (res[2]!=0) res[2][10]=a0;
  a22=(a22+a17);
  a22=(-a22);
  if (res[2]!=0) res[2][11]=a22;
  a23=(a23+a20);
  a23=(-a23);
  if (res[2]!=0) res[2][12]=a23;
  if (res[2]!=0) res[2][13]=a15;
  if (res[2]!=0) res[2][14]=a22;
  a16=(a9*a16);
  a22=(a9*a11);
  a16=(a16+a22);
  if (res[2]!=0) res[2][15]=a16;
  a12=(a12+a25);
  a12=(-a12);
  if (res[2]!=0) res[2][16]=a12;
  if (res[2]!=0) res[2][17]=a21;
  if (res[2]!=0) res[2][18]=a23;
  if (res[2]!=0) res[2][19]=a12;
  a24=(a9*a24);
  a9=(a9*a11);
  a24=(a24+a9);
  if (res[2]!=0) res[2][20]=a24;
  return 0;
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_e_fun_jac_hess(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_e_fun_jac_hess_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_e_fun_jac_hess_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_e_fun_jac_hess_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_e_fun_jac_hess_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_e_fun_jac_hess_release(int mem) {
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_e_fun_jac_hess_incref(void) {
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_e_fun_jac_hess_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int ACADOS_model_cost_ext_cost_e_fun_jac_hess_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int ACADOS_model_cost_ext_cost_e_fun_jac_hess_n_out(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_real ACADOS_model_cost_ext_cost_e_fun_jac_hess_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ACADOS_model_cost_ext_cost_e_fun_jac_hess_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ACADOS_model_cost_ext_cost_e_fun_jac_hess_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    case 3: return "o3";
    case 4: return "o4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ACADOS_model_cost_ext_cost_e_fun_jac_hess_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s1;
    case 3: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ACADOS_model_cost_ext_cost_e_fun_jac_hess_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    case 1: return casadi_s0;
    case 2: return casadi_s4;
    case 3: return casadi_s1;
    case 4: return casadi_s5;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_e_fun_jac_hess_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 5;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
