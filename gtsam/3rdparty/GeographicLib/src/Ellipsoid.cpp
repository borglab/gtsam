/**
 * \file Ellipsoid.cpp
 * \brief Implementation for GeographicLib::Ellipsoid class
 *
 * Copyright (c) Charles Karney (2012-2023) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <GeographicLib/Ellipsoid.hpp>

namespace GeographicLib {

  using namespace std;

  /// \cond SKIP
  Ellipsoid::Ellipsoid(real a, real f)
    : stol_(real(0.01) * sqrt(numeric_limits<real>::epsilon()))
    , _a(a)
    , _f(f)
    , _b(_a * (1 - _f))
    , _e2(_f * (2 - _f))
    , _e12(_e2 / (1 - _e2))
    , _n(_f / (2  - _f))
    , _aux(_a, _f)
    , _rm(_aux.RectifyingRadius(true))
    , _c2(_aux.AuthalicRadiusSquared(true))
  {}
  /// \endcond

  const Ellipsoid& Ellipsoid::WGS84() {
    static const Ellipsoid wgs84(Constants::WGS84_a(), Constants::WGS84_f());
    return wgs84;
  }

  Math::real Ellipsoid::QuarterMeridian() const
  { return Math::pi()/2 * _rm; }

  Math::real Ellipsoid::Area() const
  { return 4 * Math::pi() * _c2; }

  Math::real Ellipsoid::ParametricLatitude(real phi) const {
    return _aux.Convert(AuxLatitude::PHI, AuxLatitude::BETA,
                        Math::LatFix(phi), true);
  }

  Math::real Ellipsoid::InverseParametricLatitude(real beta) const {
    return _aux.Convert(AuxLatitude::BETA, AuxLatitude::PHI,
                        Math::LatFix(beta), true);
  }

  Math::real Ellipsoid::GeocentricLatitude(real phi) const {
    return _aux.Convert(AuxLatitude::PHI, AuxLatitude::THETA,
                        Math::LatFix(phi), true);
  }

  Math::real Ellipsoid::InverseGeocentricLatitude(real theta) const {
    return _aux.Convert(AuxLatitude::THETA, AuxLatitude::PHI,
                        Math::LatFix(theta), true);
  }

  Math::real Ellipsoid::RectifyingLatitude(real phi) const {
    return _aux.Convert(AuxLatitude::PHI, AuxLatitude::MU,
                        Math::LatFix(phi), true);
  }

  Math::real Ellipsoid::InverseRectifyingLatitude(real mu) const {
    return _aux.Convert(AuxLatitude::MU, AuxLatitude::PHI,
                        Math::LatFix(mu), true);
  }

  Math::real Ellipsoid::AuthalicLatitude(real phi) const {
    return _aux.Convert(AuxLatitude::PHI, AuxLatitude::XI,
                        Math::LatFix(phi), true);
  }

  Math::real Ellipsoid::InverseAuthalicLatitude(real xi) const {
    return _aux.Convert(AuxLatitude::XI, AuxLatitude::PHI,
                        Math::LatFix(xi), true);
  }

  Math::real Ellipsoid::ConformalLatitude(real phi) const {
    return _aux.Convert(AuxLatitude::PHI, AuxLatitude::CHI,
                        Math::LatFix(phi), true);
  }

  Math::real Ellipsoid::InverseConformalLatitude(real chi) const {
    return _aux.Convert(AuxLatitude::CHI, AuxLatitude::PHI,
                        Math::LatFix(chi), true);
  }

  Math::real Ellipsoid::IsometricLatitude(real phi) const {
    return _aux.Convert(AuxLatitude::PHI, AuxLatitude::CHI,
                        AuxAngle::degrees(Math::LatFix(phi)), true).lamd();
  }

  Math::real Ellipsoid::InverseIsometricLatitude(real psi) const {
    return  _aux.Convert(AuxLatitude::CHI, AuxLatitude::PHI,
                         AuxAngle::lamd(psi), true).degrees();
  }

  Math::real Ellipsoid::CircleRadius(real phi) const {
    // a * cos(beta)
    AuxAngle beta(_aux.Convert(AuxLatitude::PHI, AuxLatitude::BETA,
                               AuxAngle::degrees(Math::LatFix(phi)),
                               true).normalized());
    return _a * beta.x();
  }

  Math::real Ellipsoid::CircleHeight(real phi) const {
    // b * sin(beta)
    AuxAngle beta(_aux.Convert(AuxLatitude::PHI, AuxLatitude::BETA,
                               AuxAngle::degrees(Math::LatFix(phi)),
                               true).normalized());
    return _b * beta.y();
  }

  Math::real Ellipsoid::MeridianDistance(real phi) const {
    return _rm * _aux.Convert(AuxLatitude::PHI, AuxLatitude::MU,
                              AuxAngle::degrees(Math::LatFix(phi)),
                              true).radians();
  }

  Math::real Ellipsoid::MeridionalCurvatureRadius(real phi) const {
    real v = 1 - _e2 * Math::sq(Math::sind(Math::LatFix(phi)));
    return _a * (1 - _e2) / (v * sqrt(v));
  }

  Math::real Ellipsoid::TransverseCurvatureRadius(real phi) const {
    real v = 1 - _e2 * Math::sq(Math::sind(Math::LatFix(phi)));
    return _a / sqrt(v);
  }

  Math::real Ellipsoid::NormalCurvatureRadius(real phi, real azi) const {
    real calp, salp,
      v = 1 - _e2 * Math::sq(Math::sind(Math::LatFix(phi)));
    Math::sincosd(azi, salp, calp);
    return _a / (sqrt(v) * (Math::sq(calp) * v / (1 - _e2) + Math::sq(salp)));
  }

} // namespace GeographicLib
