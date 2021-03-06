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
  #define CASADI_PREFIX(ID) g_InverseKinematics_ ## ID
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

static const casadi_int casadi_s0[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s1[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s2[12] = {2, 3, 0, 2, 4, 6, 0, 1, 0, 1, 0, 1};

/* g_InverseKinematics_Task:(q[3])->(Task[2]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=5.0000000000000000e-01;
  a1=arg[0]? arg[0][0] : 0;
  a2=sin(a1);
  a3=(a0*a2);
  a4=cos(a1);
  a5=arg[0]? arg[0][1] : 0;
  a6=sin(a5);
  a7=(a4*a6);
  a8=cos(a5);
  a9=(a2*a8);
  a7=(a7+a9);
  a9=(a0*a7);
  a3=(a3+a9);
  a9=cos(a5);
  a4=(a4*a9);
  a5=sin(a5);
  a2=(a2*a5);
  a4=(a4-a2);
  a2=arg[0]? arg[0][2] : 0;
  a10=sin(a2);
  a4=(a4*a10);
  a2=cos(a2);
  a7=(a7*a2);
  a4=(a4+a7);
  a4=(a0*a4);
  a3=(a3+a4);
  a3=(-a3);
  if (res[0]!=0) res[0][0]=a3;
  a3=cos(a1);
  a4=(a0*a3);
  a8=(a3*a8);
  a1=sin(a1);
  a6=(a1*a6);
  a8=(a8-a6);
  a6=(a0*a8);
  a4=(a4+a6);
  a8=(a8*a2);
  a1=(a1*a9);
  a3=(a3*a5);
  a1=(a1+a3);
  a1=(a1*a10);
  a8=(a8-a1);
  a0=(a0*a8);
  a4=(a4+a0);
  if (res[0]!=0) res[0][1]=a4;
  return 0;
}

CASADI_SYMBOL_EXPORT int g_InverseKinematics_Task(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int g_InverseKinematics_Task_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int g_InverseKinematics_Task_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void g_InverseKinematics_Task_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int g_InverseKinematics_Task_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void g_InverseKinematics_Task_release(int mem) {
}

CASADI_SYMBOL_EXPORT void g_InverseKinematics_Task_incref(void) {
}

CASADI_SYMBOL_EXPORT void g_InverseKinematics_Task_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int g_InverseKinematics_Task_n_in(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_int g_InverseKinematics_Task_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real g_InverseKinematics_Task_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* g_InverseKinematics_Task_name_in(casadi_int i){
  switch (i) {
    case 0: return "q";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* g_InverseKinematics_Task_name_out(casadi_int i){
  switch (i) {
    case 0: return "Task";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* g_InverseKinematics_Task_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* g_InverseKinematics_Task_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int g_InverseKinematics_Task_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* g_InverseKinematics_TaskJacobian:(q[3])->(TaskJacobian[2x3]) */
static int casadi_f1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=5.0000000000000000e-01;
  a1=arg[0]? arg[0][0] : 0;
  a2=cos(a1);
  a3=(a0*a2);
  a4=arg[0]? arg[0][1] : 0;
  a5=cos(a4);
  a6=(a5*a2);
  a7=sin(a4);
  a8=sin(a1);
  a9=(a7*a8);
  a6=(a6-a9);
  a9=(a0*a6);
  a3=(a3+a9);
  a9=arg[0]? arg[0][2] : 0;
  a10=cos(a9);
  a6=(a10*a6);
  a11=sin(a9);
  a12=cos(a4);
  a8=(a12*a8);
  a13=sin(a4);
  a2=(a13*a2);
  a8=(a8+a2);
  a8=(a11*a8);
  a6=(a6-a8);
  a6=(a0*a6);
  a3=(a3+a6);
  a3=(-a3);
  if (res[0]!=0) res[0][0]=a3;
  a3=sin(a1);
  a6=(a0*a3);
  a8=(a5*a3);
  a2=cos(a1);
  a14=(a7*a2);
  a8=(a8+a14);
  a14=(a0*a8);
  a6=(a6+a14);
  a8=(a10*a8);
  a2=(a12*a2);
  a3=(a13*a3);
  a2=(a2-a3);
  a2=(a11*a2);
  a8=(a8+a2);
  a8=(a0*a8);
  a6=(a6+a8);
  a6=(-a6);
  if (res[0]!=0) res[0][1]=a6;
  a6=cos(a1);
  a8=cos(a4);
  a2=(a6*a8);
  a3=sin(a1);
  a14=sin(a4);
  a15=(a3*a14);
  a2=(a2-a15);
  a15=(a0*a2);
  a2=(a10*a2);
  a16=sin(a4);
  a17=(a6*a16);
  a4=cos(a4);
  a18=(a3*a4);
  a17=(a17+a18);
  a17=(a11*a17);
  a2=(a2-a17);
  a2=(a0*a2);
  a15=(a15+a2);
  a15=(-a15);
  if (res[0]!=0) res[0][2]=a15;
  a15=cos(a1);
  a14=(a15*a14);
  a1=sin(a1);
  a8=(a1*a8);
  a14=(a14+a8);
  a8=(a0*a14);
  a10=(a10*a14);
  a4=(a15*a4);
  a16=(a1*a16);
  a4=(a4-a16);
  a11=(a11*a4);
  a10=(a10+a11);
  a10=(a0*a10);
  a8=(a8+a10);
  a8=(-a8);
  if (res[0]!=0) res[0][3]=a8;
  a8=(a6*a12);
  a10=(a3*a13);
  a8=(a8-a10);
  a10=cos(a9);
  a8=(a8*a10);
  a6=(a6*a7);
  a3=(a3*a5);
  a6=(a6+a3);
  a9=sin(a9);
  a6=(a6*a9);
  a8=(a8-a6);
  a8=(a0*a8);
  a8=(-a8);
  if (res[0]!=0) res[0][4]=a8;
  a5=(a15*a5);
  a7=(a1*a7);
  a5=(a5-a7);
  a5=(a5*a9);
  a1=(a1*a12);
  a15=(a15*a13);
  a1=(a1+a15);
  a1=(a1*a10);
  a5=(a5+a1);
  a0=(a0*a5);
  a0=(-a0);
  if (res[0]!=0) res[0][5]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int g_InverseKinematics_TaskJacobian(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f1(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int g_InverseKinematics_TaskJacobian_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int g_InverseKinematics_TaskJacobian_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void g_InverseKinematics_TaskJacobian_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int g_InverseKinematics_TaskJacobian_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void g_InverseKinematics_TaskJacobian_release(int mem) {
}

CASADI_SYMBOL_EXPORT void g_InverseKinematics_TaskJacobian_incref(void) {
}

CASADI_SYMBOL_EXPORT void g_InverseKinematics_TaskJacobian_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int g_InverseKinematics_TaskJacobian_n_in(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_int g_InverseKinematics_TaskJacobian_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real g_InverseKinematics_TaskJacobian_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* g_InverseKinematics_TaskJacobian_name_in(casadi_int i){
  switch (i) {
    case 0: return "q";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* g_InverseKinematics_TaskJacobian_name_out(casadi_int i){
  switch (i) {
    case 0: return "TaskJacobian";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* g_InverseKinematics_TaskJacobian_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* g_InverseKinematics_TaskJacobian_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int g_InverseKinematics_TaskJacobian_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* g_InverseKinematics_TaskJacobian_derivative:(q[3],v[3])->(TaskJacobian_derivative[2x3]) */
static int casadi_f2(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a4, a5, a6, a7, a8, a9;
  a0=5.0000000000000000e-01;
  a1=arg[0]? arg[0][0] : 0;
  a2=sin(a1);
  a3=(a0*a2);
  a4=arg[0]? arg[0][1] : 0;
  a5=cos(a4);
  a6=(a5*a2);
  a7=sin(a4);
  a8=cos(a1);
  a9=(a7*a8);
  a6=(a6+a9);
  a9=(a0*a6);
  a3=(a3+a9);
  a9=arg[0]? arg[0][2] : 0;
  a10=cos(a9);
  a6=(a10*a6);
  a11=sin(a9);
  a12=cos(a4);
  a8=(a12*a8);
  a13=sin(a4);
  a2=(a13*a2);
  a8=(a8-a2);
  a8=(a11*a8);
  a6=(a6+a8);
  a6=(a0*a6);
  a3=(a3+a6);
  a6=arg[1]? arg[1][0] : 0;
  a3=(a3*a6);
  a8=cos(a1);
  a2=sin(a4);
  a14=(a8*a2);
  a15=sin(a1);
  a16=cos(a4);
  a17=(a15*a16);
  a14=(a14+a17);
  a17=(a0*a14);
  a14=(a10*a14);
  a18=cos(a4);
  a19=(a8*a18);
  a20=sin(a4);
  a21=(a15*a20);
  a19=(a19-a21);
  a19=(a11*a19);
  a14=(a14+a19);
  a14=(a0*a14);
  a17=(a17+a14);
  a14=arg[1]? arg[1][1] : 0;
  a17=(a17*a14);
  a3=(a3+a17);
  a17=(a5*a8);
  a19=(a7*a15);
  a17=(a17-a19);
  a19=sin(a9);
  a17=(a17*a19);
  a15=(a12*a15);
  a8=(a13*a8);
  a15=(a15+a8);
  a8=cos(a9);
  a15=(a15*a8);
  a17=(a17+a15);
  a17=(a0*a17);
  a15=arg[1]? arg[1][2] : 0;
  a17=(a17*a15);
  a3=(a3+a17);
  if (res[0]!=0) res[0][0]=a3;
  a3=cos(a1);
  a17=(a0*a3);
  a21=(a5*a3);
  a22=sin(a1);
  a23=(a7*a22);
  a21=(a21-a23);
  a23=(a0*a21);
  a17=(a17+a23);
  a21=(a10*a21);
  a22=(a12*a22);
  a3=(a13*a3);
  a22=(a22+a3);
  a22=(a11*a22);
  a21=(a21-a22);
  a21=(a0*a21);
  a17=(a17+a21);
  a17=(a17*a6);
  a21=cos(a1);
  a22=(a21*a16);
  a3=sin(a1);
  a23=(a3*a2);
  a22=(a22-a23);
  a23=(a0*a22);
  a22=(a10*a22);
  a24=(a21*a20);
  a25=(a3*a18);
  a24=(a24+a25);
  a24=(a11*a24);
  a22=(a22-a24);
  a22=(a0*a22);
  a23=(a23+a22);
  a23=(a23*a14);
  a17=(a17+a23);
  a23=(a12*a21);
  a22=(a13*a3);
  a23=(a23-a22);
  a23=(a23*a8);
  a3=(a5*a3);
  a21=(a7*a21);
  a3=(a3+a21);
  a3=(a3*a19);
  a23=(a23-a3);
  a23=(a0*a23);
  a23=(a23*a15);
  a17=(a17+a23);
  a17=(-a17);
  if (res[0]!=0) res[0][1]=a17;
  a17=cos(a4);
  a23=sin(a1);
  a3=(a17*a23);
  a21=sin(a4);
  a22=cos(a1);
  a24=(a21*a22);
  a3=(a3+a24);
  a24=(a0*a3);
  a3=(a10*a3);
  a25=cos(a4);
  a26=(a25*a22);
  a27=sin(a4);
  a28=(a27*a23);
  a26=(a26-a28);
  a26=(a11*a26);
  a3=(a3+a26);
  a3=(a0*a3);
  a24=(a24+a3);
  a24=(a24*a6);
  a3=cos(a1);
  a26=sin(a4);
  a28=(a3*a26);
  a29=sin(a1);
  a30=cos(a4);
  a31=(a29*a30);
  a28=(a28+a31);
  a31=(a0*a28);
  a28=(a10*a28);
  a32=cos(a4);
  a33=(a3*a32);
  a4=sin(a4);
  a34=(a29*a4);
  a33=(a33-a34);
  a33=(a11*a33);
  a28=(a28+a33);
  a28=(a0*a28);
  a31=(a31+a28);
  a31=(a31*a14);
  a24=(a24+a31);
  a31=(a3*a17);
  a28=(a29*a21);
  a31=(a31-a28);
  a31=(a31*a19);
  a28=(a3*a27);
  a33=(a29*a25);
  a28=(a28+a33);
  a28=(a28*a8);
  a31=(a31+a28);
  a31=(a0*a31);
  a31=(a31*a15);
  a24=(a24+a31);
  if (res[0]!=0) res[0][2]=a24;
  a24=cos(a1);
  a31=(a17*a24);
  a28=sin(a1);
  a33=(a21*a28);
  a31=(a31-a33);
  a33=(a0*a31);
  a31=(a10*a31);
  a34=(a25*a28);
  a35=(a27*a24);
  a34=(a34+a35);
  a34=(a11*a34);
  a31=(a31-a34);
  a31=(a0*a31);
  a33=(a33+a31);
  a33=(a33*a6);
  a31=cos(a1);
  a30=(a31*a30);
  a1=sin(a1);
  a26=(a1*a26);
  a30=(a30-a26);
  a26=(a0*a30);
  a10=(a10*a30);
  a4=(a31*a4);
  a32=(a1*a32);
  a4=(a4+a32);
  a11=(a11*a4);
  a10=(a10-a11);
  a10=(a0*a10);
  a26=(a26+a10);
  a26=(a26*a14);
  a33=(a33+a26);
  a25=(a31*a25);
  a27=(a1*a27);
  a25=(a25-a27);
  a25=(a25*a8);
  a21=(a31*a21);
  a17=(a1*a17);
  a21=(a21+a17);
  a21=(a21*a19);
  a25=(a25-a21);
  a25=(a0*a25);
  a25=(a25*a15);
  a33=(a33+a25);
  a33=(-a33);
  if (res[0]!=0) res[0][3]=a33;
  a33=cos(a9);
  a25=(a12*a23);
  a21=(a13*a22);
  a25=(a25+a21);
  a25=(a33*a25);
  a21=sin(a9);
  a22=(a5*a22);
  a23=(a7*a23);
  a22=(a22-a23);
  a22=(a21*a22);
  a25=(a25+a22);
  a25=(a0*a25);
  a25=(a25*a6);
  a22=(a3*a20);
  a23=(a29*a18);
  a22=(a22+a23);
  a22=(a33*a22);
  a23=(a3*a16);
  a19=(a29*a2);
  a23=(a23-a19);
  a23=(a21*a23);
  a22=(a22+a23);
  a22=(a0*a22);
  a22=(a22*a14);
  a25=(a25+a22);
  a22=(a3*a12);
  a23=(a29*a13);
  a22=(a22-a23);
  a23=sin(a9);
  a22=(a22*a23);
  a3=(a3*a7);
  a29=(a29*a5);
  a3=(a3+a29);
  a9=cos(a9);
  a3=(a3*a9);
  a22=(a22+a3);
  a22=(a0*a22);
  a22=(a22*a15);
  a25=(a25+a22);
  if (res[0]!=0) res[0][4]=a25;
  a25=(a12*a24);
  a22=(a13*a28);
  a25=(a25-a22);
  a25=(a33*a25);
  a28=(a5*a28);
  a24=(a7*a24);
  a28=(a28+a24);
  a28=(a21*a28);
  a25=(a25-a28);
  a25=(a0*a25);
  a25=(a25*a6);
  a18=(a31*a18);
  a20=(a1*a20);
  a18=(a18-a20);
  a33=(a33*a18);
  a2=(a31*a2);
  a16=(a1*a16);
  a2=(a2+a16);
  a21=(a21*a2);
  a33=(a33-a21);
  a33=(a0*a33);
  a33=(a33*a14);
  a25=(a25+a33);
  a5=(a31*a5);
  a7=(a1*a7);
  a5=(a5-a7);
  a5=(a5*a9);
  a1=(a1*a12);
  a31=(a31*a13);
  a1=(a1+a31);
  a1=(a1*a23);
  a5=(a5-a1);
  a0=(a0*a5);
  a0=(a0*a15);
  a25=(a25+a0);
  a25=(-a25);
  if (res[0]!=0) res[0][5]=a25;
  return 0;
}

CASADI_SYMBOL_EXPORT int g_InverseKinematics_TaskJacobian_derivative(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f2(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int g_InverseKinematics_TaskJacobian_derivative_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int g_InverseKinematics_TaskJacobian_derivative_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void g_InverseKinematics_TaskJacobian_derivative_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int g_InverseKinematics_TaskJacobian_derivative_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void g_InverseKinematics_TaskJacobian_derivative_release(int mem) {
}

CASADI_SYMBOL_EXPORT void g_InverseKinematics_TaskJacobian_derivative_incref(void) {
}

CASADI_SYMBOL_EXPORT void g_InverseKinematics_TaskJacobian_derivative_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int g_InverseKinematics_TaskJacobian_derivative_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int g_InverseKinematics_TaskJacobian_derivative_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real g_InverseKinematics_TaskJacobian_derivative_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* g_InverseKinematics_TaskJacobian_derivative_name_in(casadi_int i){
  switch (i) {
    case 0: return "q";
    case 1: return "v";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* g_InverseKinematics_TaskJacobian_derivative_name_out(casadi_int i){
  switch (i) {
    case 0: return "TaskJacobian_derivative";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* g_InverseKinematics_TaskJacobian_derivative_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* g_InverseKinematics_TaskJacobian_derivative_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int g_InverseKinematics_TaskJacobian_derivative_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
