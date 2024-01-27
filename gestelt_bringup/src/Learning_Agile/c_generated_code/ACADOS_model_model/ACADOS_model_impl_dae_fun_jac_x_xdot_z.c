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
  #define CASADI_PREFIX(ID) ACADOS_model_impl_dae_fun_jac_x_xdot_z_ ## ID
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

static const casadi_int casadi_s0[17] = {13, 1, 0, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
static const casadi_int casadi_s1[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[72] = {13, 13, 0, 1, 2, 3, 5, 7, 9, 15, 22, 29, 35, 42, 49, 56, 0, 1, 2, 0, 3, 1, 4, 2, 5, 3, 4, 6, 7, 8, 9, 3, 4, 5, 6, 7, 8, 9, 3, 4, 5, 6, 7, 8, 9, 3, 4, 6, 7, 8, 9, 6, 7, 8, 9, 10, 11, 12, 6, 7, 8, 9, 10, 11, 12, 6, 7, 8, 9, 10, 11, 12};
static const casadi_int casadi_s4[29] = {13, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
static const casadi_int casadi_s5[3] = {13, 0, 0};

/* ACADOS_model_impl_dae_fun_jac_x_xdot_z:(i0[13],i1[13],i2[4],i3[],i4[],i5[])->(o0[13],o1[13x13,56nz],o2[13x13,13nz],o3[13x0]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[1]? arg[1][0] : 0;
  a1=arg[0]? arg[0][0] : 0;
  a2=1.0000000000000001e-01;
  a3=arg[0]? arg[0][3] : 0;
  a4=(a2*a3);
  a1=(a1+a4);
  a0=(a0-a1);
  if (res[0]!=0) res[0][0]=a0;
  a0=arg[1]? arg[1][1] : 0;
  a1=arg[0]? arg[0][1] : 0;
  a4=arg[0]? arg[0][4] : 0;
  a5=(a2*a4);
  a1=(a1+a5);
  a0=(a0-a1);
  if (res[0]!=0) res[0][1]=a0;
  a0=arg[1]? arg[1][2] : 0;
  a1=arg[0]? arg[0][2] : 0;
  a5=arg[0]? arg[0][5] : 0;
  a6=(a2*a5);
  a1=(a1+a6);
  a0=(a0-a1);
  if (res[0]!=0) res[0][2]=a0;
  a0=arg[1]? arg[1][3] : 0;
  a1=4.8780487804878048e+00;
  a6=2.;
  a7=arg[0]? arg[0][7] : 0;
  a8=arg[0]? arg[0][9] : 0;
  a9=(a7*a8);
  a10=arg[0]? arg[0][6] : 0;
  a11=arg[0]? arg[0][8] : 0;
  a12=(a10*a11);
  a9=(a9+a12);
  a9=(a6*a9);
  a12=arg[2]? arg[2][0] : 0;
  a13=arg[2]? arg[2][1] : 0;
  a14=(a12+a13);
  a15=arg[2]? arg[2][2] : 0;
  a14=(a14+a15);
  a16=arg[2]? arg[2][3] : 0;
  a14=(a14+a16);
  a9=(a9*a14);
  a9=(a1*a9);
  a9=(a2*a9);
  a3=(a3+a9);
  a0=(a0-a3);
  if (res[0]!=0) res[0][3]=a0;
  a0=arg[1]? arg[1][4] : 0;
  a3=(a11*a8);
  a9=(a10*a7);
  a3=(a3-a9);
  a3=(a6*a3);
  a3=(a3*a14);
  a3=(a1*a3);
  a3=(a2*a3);
  a4=(a4+a3);
  a0=(a0-a4);
  if (res[0]!=0) res[0][4]=a0;
  a0=arg[1]? arg[1][5] : 0;
  a4=1.;
  a3=casadi_sq(a7);
  a9=casadi_sq(a11);
  a3=(a3+a9);
  a3=(a6*a3);
  a3=(a4-a3);
  a3=(a3*a14);
  a3=(a1*a3);
  a9=-9.7799999999999994e+00;
  a3=(a3+a9);
  a3=(a2*a3);
  a5=(a5+a3);
  a0=(a0-a5);
  if (res[0]!=0) res[0][5]=a0;
  a0=arg[1]? arg[1][6] : 0;
  a5=5.0000000000000000e-01;
  a3=arg[0]? arg[0][10] : 0;
  a9=(a3*a7);
  a17=arg[0]? arg[0][11] : 0;
  a18=(a17*a11);
  a9=(a9+a18);
  a18=arg[0]? arg[0][12] : 0;
  a19=(a18*a8);
  a9=(a9+a19);
  a9=(a5*a9);
  a9=(a2*a9);
  a9=(a10-a9);
  a0=(a0-a9);
  if (res[0]!=0) res[0][6]=a0;
  a0=arg[1]? arg[1][7] : 0;
  a9=(a3*a10);
  a19=(a18*a11);
  a9=(a9+a19);
  a19=(a17*a8);
  a9=(a9-a19);
  a9=(a5*a9);
  a9=(a2*a9);
  a9=(a7+a9);
  a0=(a0-a9);
  if (res[0]!=0) res[0][7]=a0;
  a0=arg[1]? arg[1][8] : 0;
  a9=(a17*a10);
  a19=(a18*a7);
  a9=(a9-a19);
  a19=(a3*a8);
  a9=(a9+a19);
  a9=(a5*a9);
  a9=(a2*a9);
  a9=(a11+a9);
  a0=(a0-a9);
  if (res[0]!=0) res[0][8]=a0;
  a0=arg[1]? arg[1][9] : 0;
  a9=(a18*a10);
  a19=(a17*a7);
  a9=(a9+a19);
  a19=(a3*a11);
  a9=(a9-a19);
  a9=(a5*a9);
  a9=(a2*a9);
  a9=(a8+a9);
  a0=(a0-a9);
  if (res[0]!=0) res[0][9]=a0;
  a0=arg[1]? arg[1][10] : 0;
  a9=2.5510204081632655e+03;
  a19=1.6500000000000001e-01;
  a20=(a19*a16);
  a20=(a20/a6);
  a21=(a19*a13);
  a21=(a21/a6);
  a20=(a20-a21);
  a21=6.3900000000000003e-04;
  a22=(a21*a17);
  a23=(a22*a18);
  a24=4.0499999999999998e-04;
  a25=(a24*a18);
  a26=(a25*a17);
  a23=(a23-a26);
  a20=(a20-a23);
  a20=(a9*a20);
  a20=(a2*a20);
  a20=(a3+a20);
  a0=(a0-a20);
  if (res[0]!=0) res[0][10]=a0;
  a0=arg[1]? arg[1][11] : 0;
  a20=2.4691358024691358e+03;
  a23=(a19*a15);
  a23=(a23/a6);
  a19=(a19*a12);
  a19=(a19/a6);
  a23=(a23-a19);
  a19=3.9199999999999999e-04;
  a26=(a19*a18);
  a27=(a26*a3);
  a28=(a21*a3);
  a29=(a28*a18);
  a27=(a27-a29);
  a23=(a23-a27);
  a23=(a20*a23);
  a23=(a2*a23);
  a23=(a17+a23);
  a0=(a0-a23);
  if (res[0]!=0) res[0][11]=a0;
  a0=arg[1]? arg[1][12] : 0;
  a23=1.5649452269170579e+03;
  a27=2.9265000000000002e-07;
  a12=(a12-a13);
  a12=(a12+a15);
  a12=(a12-a16);
  a27=(a27*a12);
  a12=(a24*a3);
  a16=(a12*a17);
  a15=(a19*a17);
  a13=(a15*a3);
  a16=(a16-a13);
  a27=(a27-a16);
  a27=(a23*a27);
  a27=(a2*a27);
  a27=(a18+a27);
  a0=(a0-a27);
  if (res[0]!=0) res[0][12]=a0;
  a0=-1.;
  if (res[1]!=0) res[1][0]=a0;
  if (res[1]!=0) res[1][1]=a0;
  if (res[1]!=0) res[1][2]=a0;
  a27=-1.0000000000000001e-01;
  if (res[1]!=0) res[1][3]=a27;
  if (res[1]!=0) res[1][4]=a0;
  if (res[1]!=0) res[1][5]=a27;
  if (res[1]!=0) res[1][6]=a0;
  if (res[1]!=0) res[1][7]=a27;
  if (res[1]!=0) res[1][8]=a0;
  a27=(a6*a11);
  a27=(a14*a27);
  a27=(a1*a27);
  a27=(a2*a27);
  a27=(-a27);
  if (res[1]!=0) res[1][9]=a27;
  a27=(a6*a7);
  a27=(a14*a27);
  a27=(a1*a27);
  a27=(a2*a27);
  if (res[1]!=0) res[1][10]=a27;
  if (res[1]!=0) res[1][11]=a0;
  a27=(a5*a3);
  a27=(a2*a27);
  a27=(-a27);
  if (res[1]!=0) res[1][12]=a27;
  a27=(a5*a17);
  a27=(a2*a27);
  a27=(-a27);
  if (res[1]!=0) res[1][13]=a27;
  a27=(a5*a18);
  a27=(a2*a27);
  a27=(-a27);
  if (res[1]!=0) res[1][14]=a27;
  a27=(a6*a8);
  a27=(a14*a27);
  a27=(a1*a27);
  a27=(a2*a27);
  a27=(-a27);
  if (res[1]!=0) res[1][15]=a27;
  a27=(a6*a10);
  a27=(a14*a27);
  a27=(a1*a27);
  a27=(a2*a27);
  if (res[1]!=0) res[1][16]=a27;
  a27=(a7+a7);
  a27=(a6*a27);
  a27=(a14*a27);
  a27=(a1*a27);
  a27=(a2*a27);
  if (res[1]!=0) res[1][17]=a27;
  a27=(a5*a3);
  a27=(a2*a27);
  if (res[1]!=0) res[1][18]=a27;
  if (res[1]!=0) res[1][19]=a0;
  a27=(a5*a18);
  a27=(a2*a27);
  if (res[1]!=0) res[1][20]=a27;
  a27=(a5*a17);
  a27=(a2*a27);
  a27=(-a27);
  if (res[1]!=0) res[1][21]=a27;
  a27=(a6*a10);
  a27=(a14*a27);
  a27=(a1*a27);
  a27=(a2*a27);
  a27=(-a27);
  if (res[1]!=0) res[1][22]=a27;
  a27=(a6*a8);
  a27=(a14*a27);
  a27=(a1*a27);
  a27=(a2*a27);
  a27=(-a27);
  if (res[1]!=0) res[1][23]=a27;
  a27=(a11+a11);
  a27=(a6*a27);
  a27=(a14*a27);
  a27=(a1*a27);
  a27=(a2*a27);
  if (res[1]!=0) res[1][24]=a27;
  a27=(a5*a17);
  a27=(a2*a27);
  if (res[1]!=0) res[1][25]=a27;
  a27=(a5*a18);
  a27=(a2*a27);
  a27=(-a27);
  if (res[1]!=0) res[1][26]=a27;
  if (res[1]!=0) res[1][27]=a0;
  a27=(a5*a3);
  a27=(a2*a27);
  if (res[1]!=0) res[1][28]=a27;
  a27=(a6*a7);
  a27=(a14*a27);
  a27=(a1*a27);
  a27=(a2*a27);
  a27=(-a27);
  if (res[1]!=0) res[1][29]=a27;
  a6=(a6*a11);
  a14=(a14*a6);
  a1=(a1*a14);
  a1=(a2*a1);
  a1=(-a1);
  if (res[1]!=0) res[1][30]=a1;
  a1=(a5*a18);
  a1=(a2*a1);
  if (res[1]!=0) res[1][31]=a1;
  a1=(a5*a17);
  a1=(a2*a1);
  if (res[1]!=0) res[1][32]=a1;
  a1=(a5*a3);
  a1=(a2*a1);
  a1=(-a1);
  if (res[1]!=0) res[1][33]=a1;
  if (res[1]!=0) res[1][34]=a0;
  a1=(a5*a7);
  a1=(a2*a1);
  if (res[1]!=0) res[1][35]=a1;
  a1=(a5*a10);
  a1=(a2*a1);
  a1=(-a1);
  if (res[1]!=0) res[1][36]=a1;
  a1=(a5*a8);
  a1=(a2*a1);
  a1=(-a1);
  if (res[1]!=0) res[1][37]=a1;
  a1=(a5*a11);
  a1=(a2*a1);
  if (res[1]!=0) res[1][38]=a1;
  if (res[1]!=0) res[1][39]=a0;
  a1=(a21*a18);
  a26=(a26-a1);
  a26=(a20*a26);
  a26=(a2*a26);
  if (res[1]!=0) res[1][40]=a26;
  a26=(a24*a17);
  a26=(a26-a15);
  a26=(a23*a26);
  a26=(a2*a26);
  if (res[1]!=0) res[1][41]=a26;
  a26=(a5*a11);
  a26=(a2*a26);
  if (res[1]!=0) res[1][42]=a26;
  a26=(a5*a8);
  a26=(a2*a26);
  if (res[1]!=0) res[1][43]=a26;
  a26=(a5*a10);
  a26=(a2*a26);
  a26=(-a26);
  if (res[1]!=0) res[1][44]=a26;
  a26=(a5*a7);
  a26=(a2*a26);
  a26=(-a26);
  if (res[1]!=0) res[1][45]=a26;
  a21=(a21*a18);
  a21=(a21-a25);
  a21=(a9*a21);
  a21=(a2*a21);
  if (res[1]!=0) res[1][46]=a21;
  if (res[1]!=0) res[1][47]=a0;
  a21=(a19*a3);
  a12=(a12-a21);
  a23=(a23*a12);
  a23=(a2*a23);
  if (res[1]!=0) res[1][48]=a23;
  a8=(a5*a8);
  a8=(a2*a8);
  if (res[1]!=0) res[1][49]=a8;
  a11=(a5*a11);
  a11=(a2*a11);
  a11=(-a11);
  if (res[1]!=0) res[1][50]=a11;
  a7=(a5*a7);
  a7=(a2*a7);
  if (res[1]!=0) res[1][51]=a7;
  a5=(a5*a10);
  a5=(a2*a5);
  a5=(-a5);
  if (res[1]!=0) res[1][52]=a5;
  a24=(a24*a17);
  a22=(a22-a24);
  a9=(a9*a22);
  a9=(a2*a9);
  if (res[1]!=0) res[1][53]=a9;
  a19=(a19*a3);
  a19=(a19-a28);
  a20=(a20*a19);
  a2=(a2*a20);
  if (res[1]!=0) res[1][54]=a2;
  if (res[1]!=0) res[1][55]=a0;
  if (res[2]!=0) res[2][0]=a4;
  if (res[2]!=0) res[2][1]=a4;
  if (res[2]!=0) res[2][2]=a4;
  if (res[2]!=0) res[2][3]=a4;
  if (res[2]!=0) res[2][4]=a4;
  if (res[2]!=0) res[2][5]=a4;
  if (res[2]!=0) res[2][6]=a4;
  if (res[2]!=0) res[2][7]=a4;
  if (res[2]!=0) res[2][8]=a4;
  if (res[2]!=0) res[2][9]=a4;
  if (res[2]!=0) res[2][10]=a4;
  if (res[2]!=0) res[2][11]=a4;
  if (res[2]!=0) res[2][12]=a4;
  return 0;
}

CASADI_SYMBOL_EXPORT int ACADOS_model_impl_dae_fun_jac_x_xdot_z(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int ACADOS_model_impl_dae_fun_jac_x_xdot_z_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int ACADOS_model_impl_dae_fun_jac_x_xdot_z_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ACADOS_model_impl_dae_fun_jac_x_xdot_z_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int ACADOS_model_impl_dae_fun_jac_x_xdot_z_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ACADOS_model_impl_dae_fun_jac_x_xdot_z_release(int mem) {
}

CASADI_SYMBOL_EXPORT void ACADOS_model_impl_dae_fun_jac_x_xdot_z_incref(void) {
}

CASADI_SYMBOL_EXPORT void ACADOS_model_impl_dae_fun_jac_x_xdot_z_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int ACADOS_model_impl_dae_fun_jac_x_xdot_z_n_in(void) { return 6;}

CASADI_SYMBOL_EXPORT casadi_int ACADOS_model_impl_dae_fun_jac_x_xdot_z_n_out(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_real ACADOS_model_impl_dae_fun_jac_x_xdot_z_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ACADOS_model_impl_dae_fun_jac_x_xdot_z_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    case 5: return "i5";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ACADOS_model_impl_dae_fun_jac_x_xdot_z_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    case 3: return "o3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ACADOS_model_impl_dae_fun_jac_x_xdot_z_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    case 2: return casadi_s1;
    case 3: return casadi_s2;
    case 4: return casadi_s2;
    case 5: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ACADOS_model_impl_dae_fun_jac_x_xdot_z_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s3;
    case 2: return casadi_s4;
    case 3: return casadi_s5;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int ACADOS_model_impl_dae_fun_jac_x_xdot_z_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 6;
  if (sz_res) *sz_res = 4;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
