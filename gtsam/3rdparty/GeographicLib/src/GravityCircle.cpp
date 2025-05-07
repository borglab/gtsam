/**
 * \file GravityCircle.cpp
 * \brief Implementation for GeographicLib::GravityCircle class
 *
 * Copyright (c) Charles Karney (2011-2020) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <GeographicLib/GravityCircle.hpp>
#include <fstream>
#include <sstream>
#include <GeographicLib/Geocentric.hpp>

namespace GeographicLib {

  using namespace std;

  GravityCircle::GravityCircle(mask caps, real a, real f, real lat, real h,
                               real Z, real P, real cphi, real sphi,
                               real amodel, real GMmodel,
                               real dzonal0, real corrmult,
                               real gamma0, real gamma, real frot,
                               const CircularEngine& gravitational,
                               const CircularEngine& disturbing,
                               const CircularEngine& correction)
    : _caps(caps)
    , _a(a)
    , _f(f)
    , _lat(Math::LatFix(lat))
    , _h(h)
    , _zZ(Z)
    , _pPx(P)
    , _invR(1 / hypot(_pPx, _zZ))
    , _cpsi(_pPx * _invR)
    , _spsi(_zZ * _invR)
    , _cphi(cphi)
    , _sphi(sphi)
    , _amodel(amodel)
    , _gGMmodel(GMmodel)
    , _dzonal0(dzonal0)
    , _corrmult(corrmult)
    , _gamma0(gamma0)
    , _gamma(gamma)
    , _frot(frot)
    , _gravitational(gravitational)
    , _disturbing(disturbing)
    , _correction(correction)
    {}

  Math::real GravityCircle::Gravity(real lon,
                                    real& gx, real& gy, real& gz) const {
    real slam, clam, M[Geocentric::dim2_];
    Math::sincosd(lon, slam, clam);
    real Wres = W(slam, clam, gx, gy, gz);
    Geocentric::Rotation(_sphi, _cphi, slam, clam, M);
    Geocentric::Unrotate(M, gx, gy, gz, gx, gy, gz);
    return Wres;
  }

  Math::real GravityCircle::Disturbance(real lon, real& deltax, real& deltay,
                                        real& deltaz) const {
    real slam, clam, M[Geocentric::dim2_];
    Math::sincosd(lon, slam, clam);
    real Tres = InternalT(slam, clam, deltax, deltay, deltaz, true, true);
    Geocentric::Rotation(_sphi, _cphi, slam, clam, M);
    Geocentric::Unrotate(M, deltax, deltay, deltaz, deltax, deltay, deltaz);
    return Tres;
  }

  Math::real GravityCircle::GeoidHeight(real lon) const {
    if ((_caps & GEOID_HEIGHT) != GEOID_HEIGHT)
      return Math::NaN();
    real slam, clam, dummy;
    Math::sincosd(lon, slam, clam);
    real T = InternalT(slam, clam, dummy, dummy, dummy, false, false);
    real correction = _corrmult * _correction(slam, clam);
    return T/_gamma0 + correction;
  }

  void GravityCircle::SphericalAnomaly(real lon,
                                       real& Dg01, real& xi, real& eta) const {
    if ((_caps & SPHERICAL_ANOMALY) != SPHERICAL_ANOMALY) {
      Dg01 = xi = eta = Math::NaN();
      return;
    }
    real slam, clam;
    Math::sincosd(lon, slam, clam);
    real
      deltax, deltay, deltaz,
      T = InternalT(slam, clam, deltax, deltay, deltaz, true, false);
    // Rotate cartesian into spherical coordinates
    real MC[Geocentric::dim2_];
    Geocentric::Rotation(_spsi, _cpsi, slam, clam, MC);
    Geocentric::Unrotate(MC, deltax, deltay, deltaz, deltax, deltay, deltaz);
    // H+M, Eq 2-151c
    Dg01 = - deltaz - 2 * T * _invR;
    xi  = -(deltay/_gamma) / Math::degree();
    eta = -(deltax/_gamma) / Math::degree();
  }

  Math::real GravityCircle::W(real slam, real clam,
                              real& gX, real& gY, real& gZ) const {
    real Wres = V(slam, clam, gX, gY, gZ) + _frot * _pPx / 2;
    gX += _frot * clam;
    gY += _frot * slam;
    return Wres;
  }

  Math::real GravityCircle::V(real slam, real clam,
                              real& GX, real& GY, real& GZ) const {
    if ((_caps & GRAVITY) != GRAVITY) {
      GX = GY = GZ = Math::NaN();
      return Math::NaN();
    }
    real
      Vres = _gravitational(slam, clam, GX, GY, GZ),
      f = _gGMmodel / _amodel;
    Vres *= f;
    GX *= f;
    GY *= f;
    GZ *= f;
    return Vres;
  }

  Math::real GravityCircle::InternalT(real slam, real clam,
                                      real& deltaX, real& deltaY, real& deltaZ,
                                      bool gradp, bool correct) const {
    if (gradp) {
      if ((_caps & DISTURBANCE) != DISTURBANCE) {
        deltaX = deltaY = deltaZ = Math::NaN();
        return Math::NaN();
      }
    } else {
      if ((_caps & DISTURBING_POTENTIAL) != DISTURBING_POTENTIAL)
        return Math::NaN();
    }
    if (_dzonal0 == 0)
      correct = false;
    real T = (gradp
              ? _disturbing(slam, clam, deltaX, deltaY, deltaZ)
              : _disturbing(slam, clam));
    T = (T / _amodel - (correct ? _dzonal0 : 0) * _invR) * _gGMmodel;
    if (gradp) {
      real f = _gGMmodel / _amodel;
      deltaX *= f;
      deltaY *= f;
      deltaZ *= f;
      if (correct) {
        real r3 = _gGMmodel * _dzonal0 * _invR * _invR * _invR;
        deltaX += _pPx * clam * r3;
        deltaY += _pPx * slam * r3;
        deltaZ += _zZ * r3;
      }
    }
    return T;
  }

} // namespace GeographicLib
