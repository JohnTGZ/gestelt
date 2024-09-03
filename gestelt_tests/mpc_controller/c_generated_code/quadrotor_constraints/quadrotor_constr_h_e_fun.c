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
  #define CASADI_PREFIX(ID) quadrotor_constr_h_e_fun_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_c0 CASADI_PREFIX(c0)
#define casadi_c1 CASADI_PREFIX(c1)
#define casadi_c2 CASADI_PREFIX(c2)
#define casadi_c3 CASADI_PREFIX(c3)
#define casadi_clear CASADI_PREFIX(clear)
#define casadi_clear_casadi_int CASADI_PREFIX(clear_casadi_int)
#define casadi_de_boor CASADI_PREFIX(de_boor)
#define casadi_dot CASADI_PREFIX(dot)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_f1 CASADI_PREFIX(f1)
#define casadi_f2 CASADI_PREFIX(f2)
#define casadi_f3 CASADI_PREFIX(f3)
#define casadi_fill CASADI_PREFIX(fill)
#define casadi_fill_casadi_int CASADI_PREFIX(fill_casadi_int)
#define casadi_low CASADI_PREFIX(low)
#define casadi_nd_boor_eval CASADI_PREFIX(nd_boor_eval)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_s6 CASADI_PREFIX(s6)

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

void casadi_de_boor(casadi_real x, const casadi_real* knots, casadi_int n_knots, casadi_int degree, casadi_real* boor) {
  casadi_int d, i;
  for (d=1;d<degree+1;++d) {
    for (i=0;i<n_knots-d-1;++i) {
      casadi_real b, bottom;
      b = 0;
      bottom = knots[i + d] - knots[i];
      if (bottom) b = (x - knots[i]) * boor[i] / bottom;
      bottom = knots[i + d + 1] - knots[i + 1];
      if (bottom) b += (knots[i + d + 1] - x) * boor[i + 1] / bottom;
      boor[i] = b;
    }
  }
}

void casadi_fill(casadi_real* x, casadi_int n, casadi_real alpha) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = alpha;
  }
}

void casadi_fill_casadi_int(casadi_int* x, casadi_int n, casadi_int alpha) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = alpha;
  }
}

void casadi_clear(casadi_real* x, casadi_int n) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = 0;
  }
}

void casadi_clear_casadi_int(casadi_int* x, casadi_int n) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = 0;
  }
}

casadi_int casadi_low(casadi_real x, const casadi_real* grid, casadi_int ng, casadi_int lookup_mode) {
  switch (lookup_mode) {
    case 1:
      {
        casadi_real g0, dg;
        casadi_int ret;
        g0 = grid[0];
        dg = grid[ng-1]-g0;
        ret = (casadi_int) ((x-g0)*(ng-1)/dg);
        if (ret<0) ret=0;
        if (ret>ng-2) ret=ng-2;
        return ret;
      }
    case 2:
      {
        casadi_int start, stop, pivot;
        if (ng<2 || x<grid[1]) return 0;
        if (x>grid[ng-1]) return ng-2;
        start = 0;
        stop  = ng-1;
        while (1) {
          pivot = (stop+start)/2;
          if (x < grid[pivot]) {
            if (pivot==stop) return pivot;
            stop = pivot;
          } else {
            if (pivot==start) return pivot;
            start = pivot;
          }
        }
      }
    default:
      {
        casadi_int i;
        for (i=0; i<ng-2; ++i) {
          if (x < grid[i+1]) break;
        }
        return i;
      }
  }
}

void casadi_nd_boor_eval(casadi_real* ret, casadi_int n_dims, const casadi_real* all_knots, const casadi_int* offset, const casadi_int* all_degree, const casadi_int* strides, const casadi_real* c, casadi_int m, const casadi_real* all_x, const casadi_int* lookup_mode, casadi_int* iw, casadi_real* w) {
  casadi_int n_iter, k, i, pivot;
  casadi_int *boor_offset, *starts, *index, *coeff_offset;
  casadi_real *cumprod, *all_boor;
  boor_offset = iw; iw+=n_dims+1;
  starts = iw; iw+=n_dims;
  index = iw; iw+=n_dims;
  coeff_offset = iw;
  cumprod = w; w+= n_dims+1;
  all_boor = w;
  boor_offset[0] = 0;
  cumprod[n_dims] = 1;
  coeff_offset[n_dims] = 0;
  n_iter = 1;
  for (k=0;k<n_dims;++k) {
    casadi_real *boor;
    const casadi_real* knots;
    casadi_real x;
    casadi_int degree, n_knots, n_b, L, start;
    boor = all_boor+boor_offset[k];
    degree = all_degree[k];
    knots = all_knots + offset[k];
    n_knots = offset[k+1]-offset[k];
    n_b = n_knots-degree-1;
    x = all_x[k];
    L = casadi_low(x, knots+degree, n_knots-2*degree, lookup_mode[k]);
    start = L;
    if (start>n_b-degree-1) start = n_b-degree-1;
    starts[k] = start;
    casadi_clear(boor, 2*degree+1);
    if (x>=knots[0] && x<=knots[n_knots-1]) {
      if (x==knots[1]) {
        casadi_fill(boor, degree+1, 1.0);
      } else if (x==knots[n_knots-1]) {
        boor[degree] = 1;
      } else if (knots[L+degree]==x) {
        boor[degree-1] = 1;
      } else {
        boor[degree] = 1;
      }
    }
    casadi_de_boor(x, knots+start, 2*degree+2, degree, boor);
    boor+= degree+1;
    n_iter*= degree+1;
    boor_offset[k+1] = boor_offset[k] + degree+1;
  }
  casadi_clear_casadi_int(index, n_dims);
  for (pivot=n_dims-1;pivot>=0;--pivot) {
    cumprod[pivot] = (*(all_boor+boor_offset[pivot]))*cumprod[pivot+1];
    coeff_offset[pivot] = starts[pivot]*strides[pivot]+coeff_offset[pivot+1];
  }
  for (k=0;k<n_iter;++k) {
    casadi_int pivot = 0;
    for (i=0;i<m;++i) ret[i] += c[coeff_offset[0]+i]*cumprod[0];
    index[0]++;
    {
      while (index[pivot]==boor_offset[pivot+1]-boor_offset[pivot]) {
        index[pivot] = 0;
        if (pivot==n_dims-1) break;
        index[++pivot]++;
      }
      while (pivot>0) {
        cumprod[pivot] = (*(all_boor+boor_offset[pivot]+index[pivot]))*cumprod[pivot+1];
        coeff_offset[pivot] = (starts[pivot]+index[pivot])*strides[pivot]+coeff_offset[pivot+1];
        pivot--;
      }
    }
    cumprod[0] = (*(all_boor+index[0]))*cumprod[1];
    coeff_offset[0] = (starts[0]+index[0])*m+coeff_offset[1];
  }
}

casadi_real casadi_dot(casadi_int n, const casadi_real* x, const casadi_real* y) {
  casadi_int i;
  casadi_real r = 0;
  for (i=0; i<n; ++i) r += *x++ * *y++;
  return r;
}

static const casadi_int casadi_s0[1] = {0};
static const casadi_int casadi_s1[1] = {1};
static const casadi_int casadi_s2[1] = {3};
static const casadi_int casadi_s3[2] = {0, 26};
static const casadi_int casadi_s4[24] = {20, 1, 0, 20, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
static const casadi_int casadi_s5[3] = {0, 0, 0};
static const casadi_int casadi_s6[5] = {1, 1, 0, 1, 0};

static const casadi_real casadi_c0[22] = {5.5333515547317802e-13, -2.1316282072803006e-13, 1.2523315717771766e-13, -2.1804780203638074e-13, 1.7186252421197423e-13, 1.7008616737257398e-13, -1.0080825063596421e-13, -3.3306690738754696e-14, -4.6629367034256575e-14, -7.9936057773011271e-14, 7.1054273576010019e-14, 2.4158453015843406e-13, 8.8817841970012523e-15, -4.1300296516055823e-13, 7.1054273576010019e-14, 2.6645352591003757e-13, -1.0658141036401503e-13, -1.4210854715202004e-13, 1.2434497875801753e-13, 6.1728400169158704e-14, -7.1054273576010019e-14, -5.6843418860808015e-13};
static const casadi_real casadi_c1[26] = {0., 0., 0., 0., 9.0000000000000002e-01, 1.2000000000000002e+00, 1.5000000000000000e+00, 1.8000000000000000e+00, 2.1000000000000001e+00, 2.4000000000000004e+00, 2.7000000000000002e+00, 3., 3.3000000000000003e+00, 3.6000000000000001e+00, 3.9000000000000004e+00, 4.2000000000000002e+00, 4.5000000000000000e+00, 4.8000000000000007e+00, 5.1000000000000005e+00, 5.4000000000000004e+00, 5.7000000000000002e+00, 6., 6.9000000000000004e+00, 6.9000000000000004e+00, 6.9000000000000004e+00, 6.9000000000000004e+00};
static const casadi_real casadi_c2[22] = {1.7281054460599656e+00, -1.9610802269919962e+00, 1.7546626649672736e+00, -1.5040208673481674e+00, 8.9278637143525970e+00, -1.1292057275181094e+01, 9.6745039731068143e+00, -3.5484255113074283e+00, 5.0740931759725516e+00, -4.6420515212587006e+00, 1.7210825927554827e+00, -8.4077405882549305e-02, 4.2138078078415964e+00, -1.0782656772684220e+01, 2.7026907324296752e+00, -1.4228709773460819e+00, 1.4189920192604641e+00, -2.4716102664124424e+00, 4.9003483340970044e+00, -2.8563999339530968e+00, 1.8462478685389998e+00, -1.1881392367963406e+00};
static const casadi_real casadi_c3[22] = {-1.5789838572446671e-14, 1.7763568394002505e-14, -1.5395092608135503e-14, 1.9737298215558338e-14, 3.7007434154171876e-14, 1.7763568394002511e-14, -2.9605947323337510e-14, -9.4739031434680029e-14, -2.5165055224836877e-14, 9.4739031434680042e-14, 7.8455760406844391e-14, -1.3470706032118565e-13, -2.5165055224836884e-14, 1.5247062871518808e-13, 4.5889218351173141e-14, -1.3618735768735254e-13, -1.0954200509634877e-13, 1.1842378929335009e-13, 3.5527136788005009e-14, 9.4739031434680067e-15, -1.2730557349035127e-13, 1.5789838572446665e-14};

/* fwd1_jac_x_ref:(x,out_f[1x1,0nz],out_jac[1x1,0nz],fwd_x,fwd_out_f[1x1,0nz])->(fwd_jac) */
static int casadi_f1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real w0, w1;
  /* #0: @0 = input[0][0] */
  w0 = arg[0] ? arg[0][0] : 0;
  /* #1: @1 = BSpline(@0) */
  casadi_clear((&w1), 1);
  CASADI_PREFIX(nd_boor_eval)((&w1),1,casadi_c1,casadi_s3,casadi_s2,casadi_s1,casadi_c0,1,(&w0),casadi_s0, iw, w);
  /* #2: @0 = input[3][0] */
  w0 = arg[3] ? arg[3][0] : 0;
  /* #3: @1 = (@1*@0) */
  w1 *= w0;
  /* #4: output[0][0] = @1 */
  if (res[0]) res[0][0] = w1;
  return 0;
}

/* fwd1_jac_y_ref:(x,out_f[1x1,0nz],out_jac[1x1,0nz],fwd_x,fwd_out_f[1x1,0nz])->(fwd_jac) */
static int casadi_f2(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real w0, w1;
  /* #0: @0 = input[0][0] */
  w0 = arg[0] ? arg[0][0] : 0;
  /* #1: @1 = BSpline(@0) */
  casadi_clear((&w1), 1);
  CASADI_PREFIX(nd_boor_eval)((&w1),1,casadi_c1,casadi_s3,casadi_s2,casadi_s1,casadi_c2,1,(&w0),casadi_s0, iw, w);
  /* #2: @0 = input[3][0] */
  w0 = arg[3] ? arg[3][0] : 0;
  /* #3: @1 = (@1*@0) */
  w1 *= w0;
  /* #4: output[0][0] = @1 */
  if (res[0]) res[0][0] = w1;
  return 0;
}

/* fwd1_jac_z_ref:(x,out_f[1x1,0nz],out_jac[1x1,0nz],fwd_x,fwd_out_f[1x1,0nz])->(fwd_jac) */
static int casadi_f3(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real w0, w1;
  /* #0: @0 = input[0][0] */
  w0 = arg[0] ? arg[0][0] : 0;
  /* #1: @1 = BSpline(@0) */
  casadi_clear((&w1), 1);
  CASADI_PREFIX(nd_boor_eval)((&w1),1,casadi_c1,casadi_s3,casadi_s2,casadi_s1,casadi_c3,1,(&w0),casadi_s0, iw, w);
  /* #2: @0 = input[3][0] */
  w0 = arg[3] ? arg[3][0] : 0;
  /* #3: @1 = (@1*@0) */
  w1 *= w0;
  /* #4: output[0][0] = @1 */
  if (res[0]) res[0][0] = w1;
  return 0;
}

/* quadrotor_constr_h_e_fun:(i0[20],i1[],i2[],i3[])->(o0) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real **res1=res+1, *rr;
  const casadi_real **arg1=arg+4;
  casadi_real w0, w3, w5, w6, w7, *w8=w+16;
  /* #0: @0 = input[0][0] */
  w0 = arg[0] ? arg[0][0] : 0;
  /* #1: @1 = 00 */
  /* #2: @2 = 00 */
  /* #3: @3 = 1 */
  w3 = 1.;
  /* #4: @4 = 00 */
  /* #5: @5 = fwd1_jac_x_ref(@0, @1, @2, @3, @4) */
  arg1[0]=(&w0);
  arg1[1]=0;
  arg1[2]=0;
  arg1[3]=(&w3);
  arg1[4]=0;
  res1[0]=(&w5);
  if (casadi_f1(arg1, res1, iw, w, 0)) return 1;
  /* #6: @1 = 00 */
  /* #7: @2 = 00 */
  /* #8: @3 = 1 */
  w3 = 1.;
  /* #9: @4 = 00 */
  /* #10: @6 = fwd1_jac_y_ref(@0, @1, @2, @3, @4) */
  arg1[0]=(&w0);
  arg1[1]=0;
  arg1[2]=0;
  arg1[3]=(&w3);
  arg1[4]=0;
  res1[0]=(&w6);
  if (casadi_f2(arg1, res1, iw, w, 0)) return 1;
  /* #11: @1 = 00 */
  /* #12: @2 = 00 */
  /* #13: @3 = 1 */
  w3 = 1.;
  /* #14: @4 = 00 */
  /* #15: @7 = fwd1_jac_z_ref(@0, @1, @2, @3, @4) */
  arg1[0]=(&w0);
  arg1[1]=0;
  arg1[2]=0;
  arg1[3]=(&w3);
  arg1[4]=0;
  res1[0]=(&w7);
  if (casadi_f3(arg1, res1, iw, w, 0)) return 1;
  /* #16: @8 = vertcat(@5, @6, @7) */
  rr=w8;
  *rr++ = w5;
  *rr++ = w6;
  *rr++ = w7;
  /* #17: @5 = ||@8||_F */
  w5 = sqrt(casadi_dot(3, w8, w8));
  /* #18: @6 = input[0][1] */
  w6 = arg[0] ? arg[0][1] : 0;
  /* #19: @5 = (@5*@6) */
  w5 *= w6;
  /* #20: output[0][0] = @5 */
  if (res[0]) res[0][0] = w5;
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_constr_h_e_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_constr_h_e_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_constr_h_e_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_constr_h_e_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_constr_h_e_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_constr_h_e_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_constr_h_e_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_constr_h_e_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_constr_h_e_fun_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_constr_h_e_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_constr_h_e_fun_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_constr_h_e_fun_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_constr_h_e_fun_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_constr_h_e_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    case 1: return casadi_s5;
    case 2: return casadi_s5;
    case 3: return casadi_s5;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_constr_h_e_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s6;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_constr_h_e_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 11;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 6;
  if (sz_w) *sz_w = 19;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
