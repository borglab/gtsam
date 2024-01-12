#ifndef CEPHES_H
#define CEPHES_H

#include "cephes/cephes_names.h"
#include "dllexport.h"

#ifdef __cplusplus
extern "C" {
#endif

CEPHES_EXTERN_EXPORT int airy(double x, double *ai, double *aip, double *bi,
                              double *bip);

CEPHES_EXTERN_EXPORT double bdtrc(double k, int n, double p);
CEPHES_EXTERN_EXPORT double bdtr(double k, int n, double p);
CEPHES_EXTERN_EXPORT double bdtri(double k, int n, double y);

CEPHES_EXTERN_EXPORT double besselpoly(double a, double lambda, double nu);

CEPHES_EXTERN_EXPORT double beta(double a, double b);
CEPHES_EXTERN_EXPORT double lbeta(double a, double b);

CEPHES_EXTERN_EXPORT double btdtr(double a, double b, double x);

CEPHES_EXTERN_EXPORT double cbrt(double x);
CEPHES_EXTERN_EXPORT double chbevl(double x, double array[], int n);
CEPHES_EXTERN_EXPORT double chdtrc(double df, double x);
CEPHES_EXTERN_EXPORT double chdtr(double df, double x);
CEPHES_EXTERN_EXPORT double chdtri(double df, double y);
CEPHES_EXTERN_EXPORT double dawsn(double xx);

CEPHES_EXTERN_EXPORT double ellie(double phi, double m);
CEPHES_EXTERN_EXPORT double ellik(double phi, double m);
CEPHES_EXTERN_EXPORT double ellpe(double x);

CEPHES_EXTERN_EXPORT int ellpj(double u, double m, double *sn, double *cn,
                               double *dn, double *ph);
CEPHES_EXTERN_EXPORT double ellpk(double x);
CEPHES_EXTERN_EXPORT double exp10(double x);
CEPHES_EXTERN_EXPORT double exp2(double x);

CEPHES_EXTERN_EXPORT double expn(int n, double x);

CEPHES_EXTERN_EXPORT double fdtrc(double a, double b, double x);
CEPHES_EXTERN_EXPORT double fdtr(double a, double b, double x);
CEPHES_EXTERN_EXPORT double fdtri(double a, double b, double y);

CEPHES_EXTERN_EXPORT int fresnl(double xxa, double *ssa, double *cca);
CEPHES_EXTERN_EXPORT double Gamma(double x);
CEPHES_EXTERN_EXPORT double lgam(double x);
CEPHES_EXTERN_EXPORT double lgam_sgn(double x, int *sign);
CEPHES_EXTERN_EXPORT double gammasgn(double x);

CEPHES_EXTERN_EXPORT double gdtr(double a, double b, double x);
CEPHES_EXTERN_EXPORT double gdtrc(double a, double b, double x);
CEPHES_EXTERN_EXPORT double gdtri(double a, double b, double y);

CEPHES_EXTERN_EXPORT double hyp2f1(double a, double b, double c, double x);
CEPHES_EXTERN_EXPORT double hyperg(double a, double b, double x);
CEPHES_EXTERN_EXPORT double threef0(double a, double b, double c, double x,
                                    double *err);

CEPHES_EXTERN_EXPORT double i0(double x);
CEPHES_EXTERN_EXPORT double i0e(double x);
CEPHES_EXTERN_EXPORT double i1(double x);
CEPHES_EXTERN_EXPORT double i1e(double x);
CEPHES_EXTERN_EXPORT double igamc(double a, double x);
CEPHES_EXTERN_EXPORT double igam(double a, double x);
CEPHES_EXTERN_EXPORT double igam_fac(double a, double x);
CEPHES_EXTERN_EXPORT double igamci(double a, double q);
CEPHES_EXTERN_EXPORT double igami(double a, double p);

CEPHES_EXTERN_EXPORT double incbet(double aa, double bb, double xx);
CEPHES_EXTERN_EXPORT double incbi(double aa, double bb, double yy0);

CEPHES_EXTERN_EXPORT double iv(double v, double x);
CEPHES_EXTERN_EXPORT double j0(double x);
CEPHES_EXTERN_EXPORT double y0(double x);
CEPHES_EXTERN_EXPORT double j1(double x);
CEPHES_EXTERN_EXPORT double y1(double x);

CEPHES_EXTERN_EXPORT double jn(int n, double x);
CEPHES_EXTERN_EXPORT double jv(double n, double x);
CEPHES_EXTERN_EXPORT double k0(double x);
CEPHES_EXTERN_EXPORT double k0e(double x);
CEPHES_EXTERN_EXPORT double k1(double x);
CEPHES_EXTERN_EXPORT double k1e(double x);
CEPHES_EXTERN_EXPORT double kn(int nn, double x);

CEPHES_EXTERN_EXPORT double nbdtrc(int k, int n, double p);
CEPHES_EXTERN_EXPORT double nbdtr(int k, int n, double p);
CEPHES_EXTERN_EXPORT double nbdtri(int k, int n, double p);

CEPHES_EXTERN_EXPORT double ndtr(double a);
CEPHES_EXTERN_EXPORT double log_ndtr(double a);
CEPHES_EXTERN_EXPORT double erfc(double a);
CEPHES_EXTERN_EXPORT double erf(double x);
CEPHES_EXTERN_EXPORT double erfinv(double y);
CEPHES_EXTERN_EXPORT double erfcinv(double y);
CEPHES_EXTERN_EXPORT double ndtri(double y0);

CEPHES_EXTERN_EXPORT double pdtrc(double k, double m);
CEPHES_EXTERN_EXPORT double pdtr(double k, double m);
CEPHES_EXTERN_EXPORT double pdtri(int k, double y);

CEPHES_EXTERN_EXPORT double poch(double x, double m);

CEPHES_EXTERN_EXPORT double psi(double x);

CEPHES_EXTERN_EXPORT double rgamma(double x);
CEPHES_EXTERN_EXPORT double round(double x);

CEPHES_EXTERN_EXPORT int shichi(double x, double *si, double *ci);
CEPHES_EXTERN_EXPORT int sici(double x, double *si, double *ci);

CEPHES_EXTERN_EXPORT double radian(double d, double m, double s);
CEPHES_EXTERN_EXPORT double sindg(double x);
CEPHES_EXTERN_EXPORT double sinpi(double x);
CEPHES_EXTERN_EXPORT double cosdg(double x);
CEPHES_EXTERN_EXPORT double cospi(double x);

CEPHES_EXTERN_EXPORT double spence(double x);

CEPHES_EXTERN_EXPORT double stdtr(int k, double t);
CEPHES_EXTERN_EXPORT double stdtri(int k, double p);

CEPHES_EXTERN_EXPORT double struve_h(double v, double x);
CEPHES_EXTERN_EXPORT double struve_l(double v, double x);
CEPHES_EXTERN_EXPORT double struve_power_series(double v, double x, int is_h,
                                                double *err);
CEPHES_EXTERN_EXPORT double struve_asymp_large_z(double v, double z, int is_h,
                                                 double *err);
CEPHES_EXTERN_EXPORT double struve_bessel_series(double v, double z, int is_h,
                                                 double *err);

CEPHES_EXTERN_EXPORT double yv(double v, double x);

CEPHES_EXTERN_EXPORT double tandg(double x);
CEPHES_EXTERN_EXPORT double cotdg(double x);

CEPHES_EXTERN_EXPORT double log1p(double x);
CEPHES_EXTERN_EXPORT double log1pmx(double x);
CEPHES_EXTERN_EXPORT double expm1(double x);
CEPHES_EXTERN_EXPORT double cosm1(double x);
CEPHES_EXTERN_EXPORT double lgam1p(double x);

CEPHES_EXTERN_EXPORT double yn(int n, double x);
CEPHES_EXTERN_EXPORT double zeta(double x, double q);
CEPHES_EXTERN_EXPORT double zetac(double x);

CEPHES_EXTERN_EXPORT double smirnov(int n, double d);
CEPHES_EXTERN_EXPORT double smirnovi(int n, double p);
CEPHES_EXTERN_EXPORT double smirnovp(int n, double d);
CEPHES_EXTERN_EXPORT double smirnovc(int n, double d);
CEPHES_EXTERN_EXPORT double smirnovci(int n, double p);
CEPHES_EXTERN_EXPORT double kolmogorov(double x);
CEPHES_EXTERN_EXPORT double kolmogi(double p);
CEPHES_EXTERN_EXPORT double kolmogp(double x);
CEPHES_EXTERN_EXPORT double kolmogc(double x);
CEPHES_EXTERN_EXPORT double kolmogci(double p);

CEPHES_EXTERN_EXPORT double lanczos_sum_expg_scaled(double x);

CEPHES_EXTERN_EXPORT double owens_t(double h, double a);

#ifdef __cplusplus
}
#endif

#endif /* CEPHES_H */
