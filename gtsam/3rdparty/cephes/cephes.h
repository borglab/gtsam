#ifndef CEPHES_H
#define CEPHES_H

#ifdef __cplusplus
extern "C" {
#endif

int airy(double x, double *ai, double *aip, double *bi, double *bip);

double bdtrc(double k, int n, double p);
double bdtr(double k, int n, double p);
double bdtri(double k, int n, double y);

double besselpoly(double a, double lambda, double nu);

double beta(double a, double b);
double lbeta(double a, double b);

double btdtr(double a, double b, double x);

double chbevl(double x, double array[], int n);
double chdtrc(double df, double x);
double chdtr(double df, double x);
double chdtri(double df, double y);
double dawsn(double xx);

double ellie(double phi, double m);
double ellik(double phi, double m);
double ellpe(double x);

int ellpj(double u, double m, double *sn, double *cn, double *dn, double *ph);
double ellpk(double x);
double exp10(double x);

double expn(int n, double x);

double fdtrc(double a, double b, double x);
double fdtr(double a, double b, double x);
double fdtri(double a, double b, double y);

int fresnl(double xxa, double *ssa, double *cca);
double Gamma(double x);
double lgam(double x);
double lgam_sgn(double x, int *sign);
double gammasgn(double x);

double gdtr(double a, double b, double x);
double gdtrc(double a, double b, double x);
double gdtri(double a, double b, double y);

double hyp2f1(double a, double b, double c, double x);
double hyperg(double a, double b, double x);
double threef0(double a, double b, double c, double x, double *err);

double i0(double x);
double i0e(double x);
double i1(double x);
double i1e(double x);
double igamc(double a, double x);
double igam(double a, double x);
double igam_fac(double a, double x);
double igamci(double a, double q);
double igami(double a, double p);

double incbet(double aa, double bb, double xx);
double incbi(double aa, double bb, double yy0);

double iv(double v, double x);

double jv(double n, double x);
double k0(double x);
double k0e(double x);
double k1(double x);
double k1e(double x);
double kn(int nn, double x);

double nbdtrc(int k, int n, double p);
double nbdtr(int k, int n, double p);
double nbdtri(int k, int n, double p);

double ndtr(double a);
double log_ndtr(double a);
double erfinv(double y);
double erfcinv(double y);
double ndtri(double y0);

double pdtrc(double k, double m);
double pdtr(double k, double m);
double pdtri(int k, double y);

double poch(double x, double m);

double psi(double x);

double rgamma(double x);

int shichi(double x, double *si, double *ci);
int sici(double x, double *si, double *ci);

double radian(double d, double m, double s);
double sindg(double x);
double sinpi(double x);
double cosdg(double x);
double cospi(double x);

double spence(double x);

double stdtr(int k, double t);
double stdtri(int k, double p);

double struve_h(double v, double x);
double struve_l(double v, double x);
double struve_power_series(double v, double x, int is_h, double *err);
double struve_asymp_large_z(double v, double z, int is_h, double *err);
double struve_bessel_series(double v, double z, int is_h, double *err);

double yv(double v, double x);

double tandg(double x);
double cotdg(double x);

double log1pmx(double x);
double cosm1(double x);
double lgam1p(double x);

double zeta(double x, double q);
double zetac(double x);

double smirnov(int n, double d);
double smirnovi(int n, double p);
double smirnovp(int n, double d);
double smirnovc(int n, double d);
double smirnovci(int n, double p);
double kolmogorov(double x);
double kolmogi(double p);
double kolmogp(double x);
double kolmogc(double x);
double kolmogci(double p);

double lanczos_sum_expg_scaled(double x);

double owens_t(double h, double a);

#ifdef __cplusplus
}
#endif

#endif /* CEPHES_H */
