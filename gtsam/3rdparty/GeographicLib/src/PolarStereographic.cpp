/**
 * \file PolarStereographic.cpp
 * \brief Implementation for GeographicLib::PolarStereographic class
 *
 * Copyright (c) Charles Karney (2008-2012) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * http://geographiclib.sourceforge.net/
 **********************************************************************/

#include <GeographicLib/PolarStereographic.hpp>

namespace GeographicLib {

  using namespace std;

  const Math::real PolarStereographic::tol_ =
    real(0.1)*sqrt(numeric_limits<real>::epsilon());
  // Overflow value s.t. atan(overflow_) = pi/2
  const Math::real PolarStereographic::overflow_ =
    1 / Math::sq(numeric_limits<real>::epsilon());

  PolarStereographic::PolarStereographic(real a, real f, real k0)
    : _a(a)
    , _f(f <= 1 ? f : 1/f)
    , _e2(_f * (2 - _f))
    , _e(sqrt(abs(_e2)))
    , _e2m(1 - _e2)
    , _Cx(exp(eatanhe(real(1))))
    , _c( (1 - _f) * _Cx )
    , _k0(k0)
  {
    if (!(Math::isfinite(_a) && _a > 0))
      throw GeographicErr("Major radius is not positive");
    if (!(Math::isfinite(_f) && _f < 1))
      throw GeographicErr("Minor radius is not positive");
    if (!(Math::isfinite(_k0) && _k0 > 0))
      throw GeographicErr("Scale is not positive");
  }

  const PolarStereographic
  PolarStereographic::UPS(Constants::WGS84_a<real>(),
                          Constants::WGS84_f<real>(),
                          Constants::UPS_k0<real>());

  // This formulation converts to conformal coordinates by tau = tan(phi) and
  // tau' = tan(phi') where phi' is the conformal latitude.  The formulas are:
  //    tau = tan(phi)
  //    secphi = hypot(1, tau)
  //    sig = sinh(e * atanh(e * tau / secphi))
  //    taup = tan(phip) = tau * hypot(1, sig) - sig * hypot(1, tau)
  //    c = (1 - f) * exp(e * atanh(e))
  //
  // Forward:
  //   rho = (2*k0*a/c) / (hypot(1, taup) + taup)  (taup >= 0)
  //       = (2*k0*a/c) * (hypot(1, taup) - taup)  (taup <  0)
  //
  // Reverse:
  //   taup = ((2*k0*a/c) / rho - rho / (2*k0*a/c))/2
  //
  // Scale:
  //   k = (rho/a) * secphi * sqrt((1-e2) + e2 / secphi^2)
  //
  // In limit rho -> 0, tau -> inf, taup -> inf, secphi -> inf, secphip -> inf
  //   secphip = taup = exp(-e * atanh(e)) * tau = exp(-e * atanh(e)) * secphi

  void PolarStereographic::Forward(bool northp, real lat, real lon,
                                   real& x, real& y, real& gamma, real& k)
    const throw() {
    lat *= northp ? 1 : -1;
    real
      phi = lat * Math::degree<real>(),
      tau = lat != -90 ? tanx(phi) : -overflow_,
      secphi = Math::hypot(real(1), tau),
      sig = sinh( eatanhe(tau / secphi) ),
      taup = Math::hypot(real(1), sig) * tau - sig * secphi,
      rho = Math::hypot(real(1), taup) + abs(taup);
    rho = taup >= 0 ? (lat != 90 ? 1/rho : 0) : rho;
    rho *= 2 * _k0 * _a / _c;
    k = lat != 90 ? (rho / _a) * secphi * sqrt(_e2m + _e2 / Math::sq(secphi)) :
      _k0;
    lon = Math::AngNormalize(lon);
    real
      lam = lon * Math::degree<real>();
    x = rho * (lon == -180 ? 0 : sin(lam));
    y = (northp ? -rho : rho) * (abs(lon) == 90 ? 0 : cos(lam));
    gamma = northp ? lon : -lon;
  }

  void PolarStereographic::Reverse(bool northp, real x, real y,
                                   real& lat, real& lon, real& gamma, real& k)
    const throw() {
    real
      rho = Math::hypot(x, y),
      t = rho / (2 * _k0 * _a / _c),
      taup = (1 / t - t) / 2,
      tau = taup * _Cx,
      stol = tol_ * max(real(1), abs(taup));
    if (abs(tau) < overflow_) {
      // min iterations = 1, max iterations = 2; mean = 1.99
      for (int i = 0; i < numit_; ++i) {
        real
          tau1 = Math::hypot(real(1), tau),
          sig = sinh( eatanhe( tau / tau1 ) ),
          taupa = Math::hypot(real(1), sig) * tau - sig * tau1,
          dtau = (taup - taupa) * (1 + _e2m * Math::sq(tau)) /
          ( _e2m * tau1 * Math::hypot(real(1), taupa) );
        tau += dtau;
        if (!(abs(dtau) >= stol))
          break;
      }
    }
    real
      phi = atan(tau),
      secphi = Math::hypot(real(1), tau);
    k = rho != 0 ?
      (rho / _a) * secphi * sqrt(_e2m + _e2 / Math::sq(secphi)) : _k0;
    lat = (northp ? 1 : -1) * (rho != 0 ? phi / Math::degree<real>() : 90);
    lon = -atan2( -x, northp ? -y : y ) / Math::degree<real>();
    gamma = northp ? lon : -lon;
  }

  void PolarStereographic::SetScale(real lat, real k) {
    if (!(Math::isfinite(k) && k > 0))
      throw GeographicErr("Scale is not positive");
    if (!(-90 < lat && lat <= 90))
      throw GeographicErr("Latitude must be in (-90d, 90d]");
    real x, y, gamma, kold;
    _k0 = 1;
    Forward(true, lat, 0, x, y, gamma, kold);
    _k0 *= k/kold;
  }

} // namespace GeographicLib
