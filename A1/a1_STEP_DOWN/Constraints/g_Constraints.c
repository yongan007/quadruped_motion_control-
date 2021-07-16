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
  #define CASADI_PREFIX(ID) g_Constraints_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_f1 CASADI_PREFIX(f1)
#define casadi_f2 CASADI_PREFIX(f2)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)

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

static const casadi_int casadi_s0[16] = {12, 1, 0, 12, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
static const casadi_int casadi_s1[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s2[18] = {1, 12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 0, 0, 0};

/* g_Constraint:(q[12])->(Task) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=-4.7000000000000000e-02;
  a1=-8.5050000000000001e-02;
  a2=arg[0]? arg[0][9] : 0;
  a3=cos(a2);
  a1=(a1*a3);
  a0=(a0+a1);
  a1=-2.0000000000000001e-01;
  a2=sin(a2);
  a3=arg[0]? arg[0][10] : 0;
  a4=cos(a3);
  a4=(a2*a4);
  a5=(a1*a4);
  a0=(a0-a5);
  a5=2.7755575615628914e-17;
  a3=sin(a3);
  a2=(a2*a3);
  a3=-2.3344536385529172e-01;
  a6=arg[0]? arg[0][11] : 0;
  a7=cos(a6);
  a8=(a3*a7);
  a9=-9.7236992039782388e-01;
  a10=sin(a6);
  a11=(a9*a10);
  a8=(a8-a11);
  a8=(a2*a8);
  a11=9.7236992039782388e-01;
  a7=(a11*a7);
  a10=(a3*a10);
  a7=(a7-a10);
  a7=(a4*a7);
  a8=(a8-a7);
  a5=(a5*a8);
  a8=sin(a6);
  a7=(a3*a8);
  a6=cos(a6);
  a9=(a9*a6);
  a7=(a7+a9);
  a2=(a2*a7);
  a11=(a11*a8);
  a3=(a3*a6);
  a11=(a11+a3);
  a4=(a4*a11);
  a2=(a2-a4);
  a1=(a1*a2);
  a5=(a5+a1);
  a0=(a0+a5);
  if (res[0]!=0) res[0][0]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int g_Constraint(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int g_Constraint_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int g_Constraint_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void g_Constraint_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int g_Constraint_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void g_Constraint_release(int mem) {
}

CASADI_SYMBOL_EXPORT void g_Constraint_incref(void) {
}

CASADI_SYMBOL_EXPORT void g_Constraint_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int g_Constraint_n_in(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_int g_Constraint_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real g_Constraint_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* g_Constraint_name_in(casadi_int i){
  switch (i) {
    case 0: return "q";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* g_Constraint_name_out(casadi_int i){
  switch (i) {
    case 0: return "Task";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* g_Constraint_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* g_Constraint_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int g_Constraint_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* g_Constraint_Jacobian:(q[12])->(TaskJacobian[1x12,3nz]) */
static int casadi_f1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][9] : 0;
  a1=cos(a0);
  a2=arg[0]? arg[0][10] : 0;
  a3=sin(a2);
  a4=-2.0000000000000001e-01;
  a5=-2.3344536385529172e-01;
  a6=arg[0]? arg[0][11] : 0;
  a7=sin(a6);
  a8=(a5*a7);
  a9=-9.7236992039782388e-01;
  a10=cos(a6);
  a11=(a9*a10);
  a8=(a8+a11);
  a8=(a4*a8);
  a11=2.7755575615628914e-17;
  a12=cos(a6);
  a13=(a5*a12);
  a14=sin(a6);
  a15=(a9*a14);
  a13=(a13-a15);
  a13=(a11*a13);
  a8=(a8+a13);
  a13=(a3*a8);
  a15=cos(a2);
  a16=2.0000000000000001e-01;
  a17=9.7236992039782388e-01;
  a7=(a17*a7);
  a10=(a5*a10);
  a7=(a7+a10);
  a7=(a16*a7);
  a10=-2.7755575615628914e-17;
  a12=(a17*a12);
  a14=(a5*a14);
  a12=(a12-a14);
  a12=(a10*a12);
  a7=(a7+a12);
  a7=(a7+a16);
  a12=(a15*a7);
  a13=(a13+a12);
  a1=(a1*a13);
  a13=-8.5050000000000001e-02;
  a12=sin(a0);
  a13=(a13*a12);
  a1=(a1-a13);
  if (res[0]!=0) res[0][0]=a1;
  a1=cos(a2);
  a0=sin(a0);
  a8=(a0*a8);
  a1=(a1*a8);
  a2=sin(a2);
  a7=(a0*a7);
  a2=(a2*a7);
  a1=(a1-a2);
  if (res[0]!=0) res[0][1]=a1;
  a1=cos(a6);
  a15=(a0*a15);
  a16=(a16*a15);
  a2=(a17*a16);
  a0=(a0*a3);
  a4=(a4*a0);
  a3=(a5*a4);
  a2=(a2+a3);
  a1=(a1*a2);
  a2=sin(a6);
  a16=(a5*a16);
  a4=(a9*a4);
  a16=(a16+a4);
  a2=(a2*a16);
  a1=(a1-a2);
  a2=cos(a6);
  a10=(a10*a15);
  a15=(a5*a10);
  a11=(a11*a0);
  a9=(a9*a11);
  a15=(a15+a9);
  a2=(a2*a15);
  a1=(a1-a2);
  a6=sin(a6);
  a17=(a17*a10);
  a5=(a5*a11);
  a17=(a17+a5);
  a6=(a6*a17);
  a1=(a1-a6);
  if (res[0]!=0) res[0][2]=a1;
  return 0;
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f1(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void g_Constraint_Jacobian_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void g_Constraint_Jacobian_release(int mem) {
}

CASADI_SYMBOL_EXPORT void g_Constraint_Jacobian_incref(void) {
}

CASADI_SYMBOL_EXPORT void g_Constraint_Jacobian_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int g_Constraint_Jacobian_n_in(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_int g_Constraint_Jacobian_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real g_Constraint_Jacobian_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* g_Constraint_Jacobian_name_in(casadi_int i){
  switch (i) {
    case 0: return "q";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* g_Constraint_Jacobian_name_out(casadi_int i){
  switch (i) {
    case 0: return "TaskJacobian";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* g_Constraint_Jacobian_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* g_Constraint_Jacobian_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* g_Constraint_Jacobian_derivative:(q[12],v[12])->(TaskJacobian_derivative[1x12,3nz]) */
static int casadi_f2(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][9] : 0;
  a1=cos(a0);
  a2=-2.0000000000000001e-01;
  a3=-2.3344536385529172e-01;
  a4=arg[0]? arg[0][11] : 0;
  a5=sin(a4);
  a6=(a3*a5);
  a7=-9.7236992039782388e-01;
  a8=cos(a4);
  a9=(a7*a8);
  a6=(a6+a9);
  a6=(a2*a6);
  a9=2.7755575615628914e-17;
  a10=cos(a4);
  a11=(a3*a10);
  a12=sin(a4);
  a13=(a7*a12);
  a11=(a11-a13);
  a11=(a9*a11);
  a6=(a6+a11);
  a11=arg[0]? arg[0][10] : 0;
  a13=cos(a11);
  a14=(a6*a13);
  a15=2.0000000000000001e-01;
  a16=9.7236992039782388e-01;
  a5=(a16*a5);
  a8=(a3*a8);
  a5=(a5+a8);
  a5=(a15*a5);
  a8=-2.7755575615628914e-17;
  a10=(a16*a10);
  a12=(a3*a12);
  a10=(a10-a12);
  a10=(a8*a10);
  a5=(a5+a10);
  a5=(a5+a15);
  a10=sin(a11);
  a12=(a5*a10);
  a14=(a14-a12);
  a14=(a1*a14);
  a12=arg[1]? arg[1][10] : 0;
  a14=(a14*a12);
  a17=sin(a11);
  a18=(a17*a6);
  a19=cos(a11);
  a20=(a19*a5);
  a18=(a18+a20);
  a20=sin(a0);
  a18=(a18*a20);
  a20=-8.5050000000000001e-02;
  a21=cos(a0);
  a20=(a20*a21);
  a18=(a18+a20);
  a20=arg[1]? arg[1][9] : 0;
  a18=(a18*a20);
  a14=(a14-a18);
  a18=cos(a4);
  a21=(a3*a18);
  a22=sin(a4);
  a23=(a7*a22);
  a21=(a21-a23);
  a21=(a2*a21);
  a23=sin(a4);
  a24=(a3*a23);
  a25=cos(a4);
  a26=(a7*a25);
  a24=(a24+a26);
  a24=(a9*a24);
  a21=(a21-a24);
  a24=(a17*a21);
  a18=(a16*a18);
  a22=(a3*a22);
  a18=(a18-a22);
  a18=(a15*a18);
  a23=(a16*a23);
  a25=(a3*a25);
  a23=(a23+a25);
  a23=(a8*a23);
  a18=(a18-a23);
  a23=(a19*a18);
  a24=(a24+a23);
  a1=(a1*a24);
  a24=arg[1]? arg[1][11] : 0;
  a1=(a1*a24);
  a14=(a14+a1);
  if (res[0]!=0) res[0][0]=a14;
  a14=cos(a11);
  a1=cos(a0);
  a23=(a6*a1);
  a23=(a14*a23);
  a25=sin(a11);
  a22=(a5*a1);
  a22=(a25*a22);
  a23=(a23-a22);
  a23=(a23*a20);
  a0=sin(a0);
  a6=(a0*a6);
  a22=sin(a11);
  a6=(a6*a22);
  a5=(a0*a5);
  a11=cos(a11);
  a5=(a5*a11);
  a6=(a6+a5);
  a6=(a6*a12);
  a23=(a23-a6);
  a21=(a0*a21);
  a14=(a14*a21);
  a18=(a0*a18);
  a25=(a25*a18);
  a14=(a14-a25);
  a14=(a14*a24);
  a23=(a23+a14);
  if (res[0]!=0) res[0][1]=a23;
  a23=cos(a4);
  a14=(a19*a1);
  a25=(a15*a14);
  a18=(a16*a25);
  a1=(a17*a1);
  a21=(a2*a1);
  a6=(a3*a21);
  a18=(a18+a6);
  a18=(a23*a18);
  a6=sin(a4);
  a25=(a3*a25);
  a21=(a7*a21);
  a25=(a25+a21);
  a25=(a6*a25);
  a18=(a18-a25);
  a25=cos(a4);
  a14=(a8*a14);
  a21=(a3*a14);
  a1=(a9*a1);
  a5=(a7*a1);
  a21=(a21+a5);
  a21=(a25*a21);
  a18=(a18-a21);
  a21=sin(a4);
  a14=(a16*a14);
  a1=(a3*a1);
  a14=(a14+a1);
  a14=(a21*a14);
  a18=(a18-a14);
  a18=(a18*a20);
  a13=(a0*a13);
  a20=(a2*a13);
  a14=(a3*a20);
  a10=(a0*a10);
  a1=(a15*a10);
  a5=(a16*a1);
  a14=(a14-a5);
  a23=(a23*a14);
  a20=(a7*a20);
  a1=(a3*a1);
  a20=(a20-a1);
  a6=(a6*a20);
  a23=(a23-a6);
  a13=(a9*a13);
  a6=(a7*a13);
  a10=(a8*a10);
  a20=(a3*a10);
  a6=(a6-a20);
  a25=(a25*a6);
  a23=(a23-a25);
  a13=(a3*a13);
  a10=(a16*a10);
  a13=(a13-a10);
  a21=(a21*a13);
  a23=(a23-a21);
  a23=(a23*a12);
  a18=(a18+a23);
  a19=(a0*a19);
  a8=(a8*a19);
  a23=(a3*a8);
  a0=(a0*a17);
  a9=(a9*a0);
  a17=(a7*a9);
  a23=(a23+a17);
  a17=sin(a4);
  a23=(a23*a17);
  a15=(a15*a19);
  a19=(a16*a15);
  a2=(a2*a0);
  a0=(a3*a2);
  a19=(a19+a0);
  a0=sin(a4);
  a19=(a19*a0);
  a15=(a3*a15);
  a7=(a7*a2);
  a15=(a15+a7);
  a7=cos(a4);
  a15=(a15*a7);
  a19=(a19+a15);
  a23=(a23-a19);
  a16=(a16*a8);
  a3=(a3*a9);
  a16=(a16+a3);
  a4=cos(a4);
  a16=(a16*a4);
  a23=(a23-a16);
  a23=(a23*a24);
  a18=(a18+a23);
  if (res[0]!=0) res[0][2]=a18;
  return 0;
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian_derivative(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f2(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian_derivative_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian_derivative_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void g_Constraint_Jacobian_derivative_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian_derivative_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void g_Constraint_Jacobian_derivative_release(int mem) {
}

CASADI_SYMBOL_EXPORT void g_Constraint_Jacobian_derivative_incref(void) {
}

CASADI_SYMBOL_EXPORT void g_Constraint_Jacobian_derivative_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int g_Constraint_Jacobian_derivative_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int g_Constraint_Jacobian_derivative_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real g_Constraint_Jacobian_derivative_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* g_Constraint_Jacobian_derivative_name_in(casadi_int i){
  switch (i) {
    case 0: return "q";
    case 1: return "v";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* g_Constraint_Jacobian_derivative_name_out(casadi_int i){
  switch (i) {
    case 0: return "TaskJacobian_derivative";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* g_Constraint_Jacobian_derivative_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* g_Constraint_Jacobian_derivative_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian_derivative_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
