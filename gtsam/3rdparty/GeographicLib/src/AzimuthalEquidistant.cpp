/**
 * \file AzimuthalEquidistant.cpp
 * \brief Implementation for GeographicLib::AzimuthalEquidistant class
 *
 * Copyright (c) Charles Karney (2009-2015) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <GeographicLib/AzimuthalEquidistant.hpp>

namespace GeographicLib {

  using namespace std;

  AzimuthalEquidistant::AzimuthalEquidistant(const Geodesic& earth)
      : eps_(real(0.01) * sqrt(numeric_limits<real>::min()))
      , _earth(earth) {}

  void AzimuthalEquidistant::Forward(real lat0, real lon0, real lat, real lon,
                                     real& x, real& y,
                                     real& azi, real& rk) const {
    real sig, s, azi0, m;
    sig = _earth.Inverse(lat0, lon0, lat, lon, s, azi0, azi, m);
    Math::sincosd(azi0, x, y);
    x *= s; y *= s;
    rk = !(sig <= eps_) ? m / s : 1;
  }

  void AzimuthalEquidistant::Reverse(real lat0, real lon0, real x, real y,
                                     real& lat, real& lon,
                                     real& azi, real& rk) const {
    real
      azi0 = Math::atan2d(x, y),
      s = Math::hypot(x, y);
    real sig, m;
    sig = _earth.Direct(lat0, lon0, azi0, s, lat, lon, azi, m);
    rk = !(sig <= eps_) ? m / s : 1;
  }

} // namespace GeographicLib
