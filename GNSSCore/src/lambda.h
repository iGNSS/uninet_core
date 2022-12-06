#ifndef _LAMBDA_H_
#define _LAMBDA_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

#ifdef WIN_DLL
#define EXPORT __declspec(dllexport) /* for Windows DLL */
#else
#define EXPORT
#endif

#ifndef LOOPMAX
#define LOOPMAX     10000           /* maximum count of search loop */
#endif

#ifndef SGN
#define SGN(x)      ((x)<=0.0?-1.0:1.0)
#endif

#ifndef ROUND
#define ROUND(x)    (floor((x)+0.5))
#endif

#ifndef SWAP
#define SWAP(x,y)   do {double tmp_; tmp_=x; x=y; y=tmp_;} while (0)
#endif

/* matrix and vector functions -----------------------------------------------*/
EXPORT double *mat  (int n, int m);
EXPORT int    *imat (int n, int m);
EXPORT double *zeros(int n, int m);
EXPORT double *eye  (int n);
EXPORT void matcpy(double *A, const double *B, int n, int m);
EXPORT void matmul(const char *tr, int n, int k, int m, double alpha, const double *A, const double *B, double beta, double *C);
EXPORT int  matinv(double *A, int n);
EXPORT int  solve (const char *tr, const double *A, const double *Y, int n, int m, double *X);
EXPORT int  lsq   (const double *A, const double *y, int n, int m, double *x, double *Q);
EXPORT int  filter(double *x, double *P, const double *H, const double *v, const double *R, int n, int m);
EXPORT int  smoother(const double *xf, const double *Qf, const double *xb, const double *Qb, int n, double *xs, double *Qs);
EXPORT void matprint (const double *A, int n, int m, int p, int q);
EXPORT void matfprint(const double *A, int n, int m, int p, int q, FILE *fp);

/* integer ambiguity resolution ----------------------------------------------*/
EXPORT int lambda(int n, int m, const double *a, const double *Q, double *F, double *s);
EXPORT int lambda_reduction(int n, const double *Q, double *Z);
EXPORT int lambda_search(int n, int m, const double *a, const double *Q, double *F, double *s);

#ifdef __cplusplus
}
#endif
#endif /* _LAMBDA_H_ */
