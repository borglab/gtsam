#ifndef CEPHES_H
#define CEPHES_H

#ifdef __cplusplus
extern "C" {
#endif

double gtsam_cephes_Gamma(double x);
double gtsam_cephes_lgam(double x);
double gtsam_cephes_lgam_sgn(double x, int *sign);
double gtsam_cephes_gammasgn(double x);

double gtsam_cephes_igamc(double a, double x);
double gtsam_cephes_igam(double a, double x);
double gtsam_cephes_igam_fac(double a, double x);
double gtsam_cephes_igamci(double a, double q);
double gtsam_cephes_igami(double a, double p);

double gtsam_cephes_log1pmx(double x);
double gtsam_cephes_cosm1(double x);
double gtsam_cephes_lgam1p(double x);

double gtsam_cephes_zeta(double x, double q);
double gtsam_cephes_zetac(double x);

double gtsam_cephes_lanczos_sum_expg_scaled(double x);

#ifdef __cplusplus
}
#endif

#endif /* CEPHES_H */
