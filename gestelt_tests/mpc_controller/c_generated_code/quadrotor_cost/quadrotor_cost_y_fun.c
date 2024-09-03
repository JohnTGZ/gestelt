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
  #define CASADI_PREFIX(ID) quadrotor_cost_y_fun_ ## ID
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

static const casadi_int casadi_s0[24] = {20, 1, 0, 20, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
static const casadi_int casadi_s1[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[28] = {24, 1, 0, 24, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23};

/* quadrotor_cost_y_fun:(i0[20],i1[4],i2[],i3[],i4[])->(o0[24]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real w0;
  /* #0: @0 = input[0][0] */
  w0 = arg[0] ? arg[0][0] : 0;
  /* #1: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  /* #2: @0 = input[0][1] */
  w0 = arg[0] ? arg[0][1] : 0;
  /* #3: output[0][1] = @0 */
  if (res[0]) res[0][1] = w0;
  /* #4: @0 = input[0][2] */
  w0 = arg[0] ? arg[0][2] : 0;
  /* #5: output[0][2] = @0 */
  if (res[0]) res[0][2] = w0;
  /* #6: @0 = input[0][3] */
  w0 = arg[0] ? arg[0][3] : 0;
  /* #7: output[0][3] = @0 */
  if (res[0]) res[0][3] = w0;
  /* #8: @0 = input[0][4] */
  w0 = arg[0] ? arg[0][4] : 0;
  /* #9: output[0][4] = @0 */
  if (res[0]) res[0][4] = w0;
  /* #10: @0 = input[0][5] */
  w0 = arg[0] ? arg[0][5] : 0;
  /* #11: output[0][5] = @0 */
  if (res[0]) res[0][5] = w0;
  /* #12: @0 = input[0][6] */
  w0 = arg[0] ? arg[0][6] : 0;
  /* #13: output[0][6] = @0 */
  if (res[0]) res[0][6] = w0;
  /* #14: @0 = input[0][7] */
  w0 = arg[0] ? arg[0][7] : 0;
  /* #15: output[0][7] = @0 */
  if (res[0]) res[0][7] = w0;
  /* #16: @0 = input[0][8] */
  w0 = arg[0] ? arg[0][8] : 0;
  /* #17: output[0][8] = @0 */
  if (res[0]) res[0][8] = w0;
  /* #18: @0 = input[0][9] */
  w0 = arg[0] ? arg[0][9] : 0;
  /* #19: output[0][9] = @0 */
  if (res[0]) res[0][9] = w0;
  /* #20: @0 = input[0][10] */
  w0 = arg[0] ? arg[0][10] : 0;
  /* #21: output[0][10] = @0 */
  if (res[0]) res[0][10] = w0;
  /* #22: @0 = input[0][11] */
  w0 = arg[0] ? arg[0][11] : 0;
  /* #23: output[0][11] = @0 */
  if (res[0]) res[0][11] = w0;
  /* #24: @0 = input[0][12] */
  w0 = arg[0] ? arg[0][12] : 0;
  /* #25: output[0][12] = @0 */
  if (res[0]) res[0][12] = w0;
  /* #26: @0 = input[0][13] */
  w0 = arg[0] ? arg[0][13] : 0;
  /* #27: output[0][13] = @0 */
  if (res[0]) res[0][13] = w0;
  /* #28: @0 = input[0][14] */
  w0 = arg[0] ? arg[0][14] : 0;
  /* #29: output[0][14] = @0 */
  if (res[0]) res[0][14] = w0;
  /* #30: @0 = input[0][15] */
  w0 = arg[0] ? arg[0][15] : 0;
  /* #31: output[0][15] = @0 */
  if (res[0]) res[0][15] = w0;
  /* #32: @0 = input[0][16] */
  w0 = arg[0] ? arg[0][16] : 0;
  /* #33: output[0][16] = @0 */
  if (res[0]) res[0][16] = w0;
  /* #34: @0 = input[0][17] */
  w0 = arg[0] ? arg[0][17] : 0;
  /* #35: output[0][17] = @0 */
  if (res[0]) res[0][17] = w0;
  /* #36: @0 = input[0][18] */
  w0 = arg[0] ? arg[0][18] : 0;
  /* #37: output[0][18] = @0 */
  if (res[0]) res[0][18] = w0;
  /* #38: @0 = input[0][19] */
  w0 = arg[0] ? arg[0][19] : 0;
  /* #39: output[0][19] = @0 */
  if (res[0]) res[0][19] = w0;
  /* #40: @0 = input[1][0] */
  w0 = arg[1] ? arg[1][0] : 0;
  /* #41: output[0][20] = @0 */
  if (res[0]) res[0][20] = w0;
  /* #42: @0 = input[1][1] */
  w0 = arg[1] ? arg[1][1] : 0;
  /* #43: output[0][21] = @0 */
  if (res[0]) res[0][21] = w0;
  /* #44: @0 = input[1][2] */
  w0 = arg[1] ? arg[1][2] : 0;
  /* #45: output[0][22] = @0 */
  if (res[0]) res[0][22] = w0;
  /* #46: @0 = input[1][3] */
  w0 = arg[1] ? arg[1][3] : 0;
  /* #47: output[0][23] = @0 */
  if (res[0]) res[0][23] = w0;
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_y_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_y_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_y_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_y_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_y_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_y_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_y_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_y_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_cost_y_fun_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_cost_y_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_cost_y_fun_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_cost_y_fun_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_cost_y_fun_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_cost_y_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s2;
    case 4: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_cost_y_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_y_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 1;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
