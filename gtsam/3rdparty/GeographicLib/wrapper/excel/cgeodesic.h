#if !defined(CGEODESIC_H)
#define CGEODESIC_H 1

#if defined(__cplusplus)
extern "C" {
#endif

  void gdirect(double lat1, double lon1, double azi1, double s12,
               double& lat2, double& lon2, double& azi2);

  void ginverse(double lat1, double lon1, double lat2, double lon2,
                double& s12, double& azi1, double& azi2);

  void rdirect(double lat1, double lon1, double azi12, double s12,
               double& lat2, double& lon2);

  void rinverse(double lat1, double lon1, double lat2, double lon2,
                double& s12, double& azi12);

#if defined(__cplusplus)
}
#endif

#endif  /* CGEODESIC_H */
