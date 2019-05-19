/**
 * \file Gnomonic.cpp
 * \brief Implementation for GeographicLib::Gnomonic class
 *
 * Copyright (c) Charles Karney (2010-2015) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <GeographicLib/Gnomonic.hpp>

#if defined(_MSC_VER)
// Squelch warnings about potentially uninitialized local variables and
// constant conditional expressions
#  pragma warning (disable: 4701 4127)
#endif

namespace GeographicLib {

  using namespace std;

  Gnomonic::Gnomonic(const Geodesic& earth)
    : eps0_(numeric_limits<real>::epsilon())
    , eps_(real(0.01) * sqrt(eps0_))
    , _earth(earth)
    , _a(_earth.MajorRadius())
    , _f(_earth.Flattening())
  {}

  void Gnomonic::Forward(real lat0, real lon0, real lat, real lon,
                         real& x, real& y, real& azi, real& rk) const {
    real azi0, m, M, t;
    _earth.GenInverse(lat0, lon0, lat, lon,
                      Geodesic::AZIMUTH | Geodesic::REDUCEDLENGTH |
                      Geodesic::GEODESICSCALE,
                      t, azi0, azi, m, M, t, t);
    rk = M;
    if (M <= 0)
      x = y = Math::NaN();
    else {
      real rho = m/M;
      Math::sincosd(azi0, x, y);
      x *= rho; y *= rho;
    }
  }

  void Gnomonic::Reverse(real lat0, real lon0, real x, real y,
                         real& lat, real& lon, real& azi, real& rk) const {
    real
      azi0 = Math::atan2d(x, y),
      rho = Math::hypot(x, y),
      s = _a * atan(rho/_a);
    bool little = rho <= _a;
    if (!little)
      rho = 1/rho;
    GeodesicLine line(_earth.Line(lat0, lon0, azi0,
                                  Geodesic::LATITUDE | Geodesic::LONGITUDE |
                                  Geodesic::AZIMUTH | Geodesic::DISTANCE_IN |
                                  Geodesic::REDUCEDLENGTH |
                                  Geodesic::GEODESICSCALE));
    int count = numit_, trip = 0;
    real lat1, lon1, azi1, M;
    while (count-- || GEOGRAPHICLIB_PANIC) {
      real m, t;
      line.Position(s, lat1, lon1, azi1, m, M, t);
      if (trip)
        break;
      // If little, solve rho(s) = rho with drho(s)/ds = 1/M^2
      // else solve 1/rho(s) = 1/rho with d(1/rho(s))/ds = -1/m^2
      real ds = little ? (m/M - rho) * M * M : (rho - M/m) * m * m;
      s -= ds;
      // Reversed test to allow escape with NaNs
      if (!(abs(ds) >= eps_ * _a))
        ++trip;
    }
    if (trip) {
      lat = lat1; lon = lon1; azi = azi1; rk = M;
    } else
      lat = lon = azi = rk = Math::NaN();
    return;
  }

} // namespace GeographicLib
