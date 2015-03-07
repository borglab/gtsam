/**
 * \file GravityCircle.cpp
 * \brief Implementation for GeographicLib::GravityCircle class
 *
 * Copyright (c) Charles Karney (2011) <charles@karney.com> and licensed under
 * the MIT/X11 License.  For more information, see
 * http://geographiclib.sourceforge.net/
 **********************************************************************/

#include <GeographicLib/GravityCircle.hpp>
#include <fstream>
#include <sstream>
#include <GeographicLib/Geocentric.hpp>

namespace GeographicLib {

  using namespace std;

  Math::real GravityCircle::Gravity(real lon, real& gx, real& gy, real& gz)
    const throw() {
    real clam, slam, M[Geocentric::dim2_];
    CircularEngine::cossin(lon, clam, slam);
    real Wres = W(clam, slam, gx, gy, gz);
    Geocentric::Rotation(_sphi, _cphi, slam, clam, M);
    Geocentric::Unrotate(M, gx, gy, gz, gx, gy, gz);
    return Wres;
  }

  Math::real GravityCircle::Disturbance(real lon, real& deltax, real& deltay,
                                        real& deltaz) const throw() {
    real clam, slam, M[Geocentric::dim2_];
    CircularEngine::cossin(lon, clam, slam);
    real Tres = InternalT(clam, slam, deltax, deltay, deltaz, true, true);
    Geocentric::Rotation(_sphi, _cphi, slam, clam, M);
    Geocentric::Unrotate(M, deltax, deltay, deltaz, deltax, deltay, deltaz);
    return Tres;
  }

  Math::real GravityCircle::GeoidHeight(real lon) const throw() {
    if ((_caps & GEOID_HEIGHT) != GEOID_HEIGHT)
      return Math::NaN<real>();
    real clam, slam, dummy;
    CircularEngine::cossin(lon, clam, slam);
    real T = InternalT(clam, slam, dummy, dummy, dummy, false, false);
    real correction = _corrmult * _correction(clam, slam);
    return T/_gamma0 + correction;
  }

  void GravityCircle::SphericalAnomaly(real lon,
                                       real& Dg01, real& xi, real& eta)
    const throw() {
    if ((_caps & SPHERICAL_ANOMALY) != SPHERICAL_ANOMALY) {
      Dg01 = xi = eta = Math::NaN<real>();
      return;
    }
    real clam, slam;
    CircularEngine::cossin(lon, clam, slam);
    real
      deltax, deltay, deltaz,
      T = InternalT(clam, slam, deltax, deltay, deltaz, true, false);
    // Rotate cartesian into spherical coordinates
    real MC[Geocentric::dim2_];
    Geocentric::Rotation(_spsi, _cpsi, slam, clam, MC);
    Geocentric::Unrotate(MC, deltax, deltay, deltaz, deltax, deltay, deltaz);
    // H+M, Eq 2-151c
    Dg01 = - deltaz - 2 * T * _invR;
    xi  = -(deltay/_gamma) / Math::degree<real>();
    eta = -(deltax/_gamma) / Math::degree<real>();
  }

  Math::real GravityCircle::W(real clam, real slam,
                              real& gX, real& gY, real& gZ) const throw() {
    real Wres = V(clam, slam, gX, gY, gZ) + _frot * _Px / 2;
    gX += _frot * clam;
    gY += _frot * slam;
    return Wres;
  }

  Math::real GravityCircle::V(real clam, real slam,
                              real& GX, real& GY, real& GZ)
    const throw() {
    if ((_caps & GRAVITY) != GRAVITY) {
      GX = GY = GZ = Math::NaN<real>();
      return Math::NaN<real>();
    }
    real
      Vres = _gravitational(clam, slam, GX, GY, GZ),
      f = _GMmodel / _amodel;
    Vres *= f;
    GX *= f;
    GY *= f;
    GZ *= f;
    return Vres;
  }

  Math::real GravityCircle::InternalT(real clam, real slam,
                                      real& deltaX, real& deltaY, real& deltaZ,
                                      bool gradp, bool correct) const throw() {
    if (gradp) {
      if ((_caps & DISTURBANCE) != DISTURBANCE) {
        deltaX = deltaY = deltaZ = Math::NaN<real>();
        return Math::NaN<real>();
      }
    } else {
      if ((_caps & DISTURBING_POTENTIAL) != DISTURBING_POTENTIAL)
        return Math::NaN<real>();
    }
    if (_dzonal0 == 0)
      correct = false;
    real T = (gradp
              ? _disturbing(clam, slam, deltaX, deltaY, deltaZ)
              : _disturbing(clam, slam));
    T = (T / _amodel - (correct ? _dzonal0 : 0) * _invR) * _GMmodel;
    if (gradp) {
      real f = _GMmodel / _amodel;
      deltaX *= f;
      deltaY *= f;
      deltaZ *= f;
      if (correct) {
        real r3 = _GMmodel * _dzonal0 * _invR * _invR * _invR;
        deltaX += _Px * clam * r3;
        deltaY += _Px * slam * r3;
        deltaZ += _Z * r3;
      }
    }
    return T;
  }

} // namespace GeographicLib
