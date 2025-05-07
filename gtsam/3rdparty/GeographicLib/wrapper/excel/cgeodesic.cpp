#include "cgeodesic.h"
#include "GeographicLib/Geodesic.hpp"
#include "GeographicLib/Rhumb.hpp"

extern "C" {

  void gdirect(double lat1, double lon1, double azi1, double s12,
               double& lat2, double& lon2, double& azi2) {
    GeographicLib::Geodesic::WGS84().Direct(lat1, lon1, azi1, s12,
                                            lat2, lon2, azi2);
  }

  void ginverse(double lat1, double lon1, double lat2, double lon2,
                double& s12, double& azi1, double& azi2) {
    GeographicLib::Geodesic::WGS84().Inverse(lat1, lon1, lat2, lon2,
                                             s12, azi1, azi2);
  }

  void rdirect(double lat1, double lon1, double azi12, double s12,
               double& lat2, double& lon2) {
    GeographicLib::Rhumb::WGS84().Direct(lat1, lon1, azi12, s12,
                                         lat2, lon2);
  }

  void rinverse(double lat1, double lon1, double lat2, double lon2,
                double& s12, double& azi12) {
    GeographicLib::Rhumb::WGS84().Inverse(lat1, lon1, lat2, lon2,
                                          s12, azi12);
  }

}
