/**
 * \file CassiniSoldner.cpp
 * \brief Implementation for GeographicLib::CassiniSoldner class
 *
 * Copyright (c) Charles Karney (2009-2015) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <GeographicLib/CassiniSoldner.hpp>

namespace GeographicLib {

  using namespace std;

  CassiniSoldner::CassiniSoldner(const Geodesic& earth)
    : _earth(earth) {}

  CassiniSoldner::CassiniSoldner(real lat0, real lon0, const Geodesic& earth)
    : _earth(earth)
  { Reset(lat0, lon0); }

  void CassiniSoldner::Reset(real lat0, real lon0) {
    _meridian = _earth.Line(lat0, lon0, real(0),
                            Geodesic::LATITUDE | Geodesic::LONGITUDE |
                            Geodesic::DISTANCE | Geodesic::DISTANCE_IN |
                            Geodesic::AZIMUTH);
    real f = _earth.Flattening();
    Math::sincosd(LatitudeOrigin(), _sbet0, _cbet0);
    _sbet0 *= (1 - f);
    Math::norm(_sbet0, _cbet0);
  }

  void CassiniSoldner::Forward(real lat, real lon, real& x, real& y,
                               real& azi, real& rk) const {
    if (!Init())
      return;
    real dlon = Math::AngDiff(LongitudeOrigin(), lon);
    real sig12, s12, azi1, azi2;
    sig12 = _earth.Inverse(lat, -abs(dlon), lat, abs(dlon), s12, azi1, azi2);
    sig12 *= real(0.5);
    s12 *= real(0.5);
    if (s12 == 0) {
      real da = Math::AngDiff(azi1, azi2)/2;
      if (abs(dlon) <= 90) {
        azi1 = 90 - da;
        azi2 = 90 + da;
      } else {
        azi1 = -90 - da;
        azi2 = -90 + da;
      }
    }
    if (dlon < 0) {
      azi2 = azi1;
      s12 = -s12;
      sig12 = -sig12;
    }
    x = s12;
    azi = Math::AngNormalize(azi2);
    GeodesicLine perp(_earth.Line(lat, dlon, azi, Geodesic::GEODESICSCALE));
    real t;
    perp.GenPosition(true, -sig12,
                     Geodesic::GEODESICSCALE,
                     t, t, t, t, t, t, rk, t);

    real salp0, calp0;
    Math::sincosd(perp.EquatorialAzimuth(), salp0, calp0);
    real
      sbet1 = lat >=0 ? calp0 : -calp0,
      cbet1 = abs(dlon) <= 90 ? abs(salp0) : -abs(salp0),
      sbet01 = sbet1 * _cbet0 - cbet1 * _sbet0,
      cbet01 = cbet1 * _cbet0 + sbet1 * _sbet0,
      sig01 = atan2(sbet01, cbet01) / Math::degree();
    _meridian.GenPosition(true, sig01,
                          Geodesic::DISTANCE,
                          t, t, t, y, t, t, t, t);
  }

  void CassiniSoldner::Reverse(real x, real y, real& lat, real& lon,
                               real& azi, real& rk) const {
    if (!Init())
      return;
    real lat1, lon1;
    real azi0, t;
    _meridian.Position(y, lat1, lon1, azi0);
    _earth.Direct(lat1, lon1, azi0 + 90, x, lat, lon, azi, rk, t);
  }

} // namespace GeographicLib
