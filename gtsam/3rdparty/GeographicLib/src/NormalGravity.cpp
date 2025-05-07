/**
 * \file NormalGravity.cpp
 * \brief Implementation for GeographicLib::NormalGravity class
 *
 * Copyright (c) Charles Karney (2011-2022) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <GeographicLib/NormalGravity.hpp>

namespace GeographicLib {

  using namespace std;

  void NormalGravity::Initialize(real a, real GM, real omega, real f_J2,
                                 bool geometricp) {
    _a = a;
    if (!(isfinite(_a) && _a > 0))
      throw GeographicErr("Equatorial radius is not positive");
    _gGM = GM;
    if (!isfinite(_gGM))
      throw GeographicErr("Gravitational constant is not finite");
    _omega = omega;
    _omega2 = Math::sq(_omega);
    _aomega2 = Math::sq(_omega * _a);
    if (!(isfinite(_omega2) && isfinite(_aomega2)))
      throw GeographicErr("Rotation velocity is not finite");
    _f = geometricp ? f_J2 : J2ToFlattening(_a, _gGM, _omega, f_J2);
    _b = _a * (1 - _f);
    if (!(isfinite(_b) && _b > 0))
      throw GeographicErr("Polar semi-axis is not positive");
    _jJ2 = geometricp ? FlatteningToJ2(_a, _gGM, _omega, f_J2) : f_J2;
    _e2 = _f * (2 - _f);
    _ep2 = _e2 / (1 - _e2);
    real ex2 = _f < 0 ? -_e2 : _ep2;
    _qQ0 = Qf(ex2, _f < 0);
    _earth = Geocentric(_a, _f);
    _eE = _a * sqrt(fabs(_e2));  // H+M, Eq 2-54
    // H+M, Eq 2-61
    _uU0 = _gGM * atanzz(ex2, _f < 0) / _b + _aomega2 / 3;
    real P = Hf(ex2, _f < 0) / (6 * _qQ0);
    // H+M, Eq 2-73
    _gammae = _gGM / (_a * _b) - (1 + P) * _a * _omega2;
    // H+M, Eq 2-74
    _gammap = _gGM / (_a * _a) + 2 * P * _b * _omega2;
    // k = gammae * (b * gammap / (a * gammae) - 1)
    //   = (b * gammap - a * gammae) / a
    _k = -_e2 * _gGM / (_a * _b) +
      _omega2 * (P * (_a + 2 * _b * (1 - _f)) + _a);
    // f* = (gammap - gammae) / gammae
    _fstar = (-_f * _gGM / (_a * _b) + _omega2 * (P * (_a + 2 * _b) + _a)) /
      _gammae;
  }

  NormalGravity::NormalGravity(real a, real GM, real omega, real f_J2,
                               bool geometricp) {
    Initialize(a, GM, omega, f_J2, geometricp);
  }

  const NormalGravity& NormalGravity::WGS84() {
    static const NormalGravity wgs84(Constants::WGS84_a(),
                                     Constants::WGS84_GM(),
                                     Constants::WGS84_omega(),
                                     Constants::WGS84_f(), true);
    return wgs84;
  }

  const NormalGravity& NormalGravity::GRS80() {
    static const NormalGravity grs80(Constants::GRS80_a(),
                                     Constants::GRS80_GM(),
                                     Constants::GRS80_omega(),
                                     Constants::GRS80_J2(), false);
    return grs80;
  }

  Math::real NormalGravity::atan7series(real x) {
    // compute -sum( (-x)^n/(2*n+7), n, 0, inf)
    //   = -1/7 + x/9 - x^2/11 + x^3/13 ...
    //   = (atan(sqrt(x))/sqrt(x)-(1-x/3+x^2/5)) / x^3 (x > 0)
    //   = (atanh(sqrt(-x))/sqrt(-x)-(1-x/3+x^2/5)) / x^3 (x < 0)
    // require abs(x) < 1/2, but better to restrict calls to abs(x) < 1/4
    static const real lg2eps_ = -log2(numeric_limits<real>::epsilon() / 2);
    int e;
    (void) frexp(x, &e);
    e = max(-e, 1);             // Here's where abs(x) < 1/2 is assumed
    // x = [0.5,1) * 2^(-e)
    // estimate n s.t. x^n/n < 1/7 * epsilon/2
    // a stronger condition is x^n < epsilon/2
    // taking log2 of both sides, a stronger condition is n*(-e) < -lg2eps;
    // or n*e > lg2eps or n > ceiling(lg2eps/e)
    int n = x == 0 ? 1 : int(ceil(lg2eps_ / e));
    Math::real v = 0;
    while (n--)                 // iterating from n-1 down to 0
      v = - x * v - 1/Math::real(2*n + 7);
    return v;
  }

  Math::real NormalGravity::atan5series(real x) {
    // Compute Taylor series approximations to
    //   (atan(z)-(z-z^3/3))/z^5,
    // z = sqrt(x)
    // require abs(x) < 1/2, but better to restrict calls to abs(x) < 1/4
    return 1/real(5) + x * atan7series(x);
  }

  Math::real NormalGravity::Qf(real x, bool alt) {
    // Compute
    //   Q(z) = (((1 + 3/z^2) * atan(z) - 3/z)/2) / z^3
    //        = q(z)/z^3 with q(z) defined by H+M, Eq 2-57 with z = E/u
    //   z = sqrt(x)
    real y = alt ? -x / (1 + x) : x;
    return !(4 * fabs(y) < 1) ?  // Backwards test to allow NaNs through
      ((1 + 3/y) * atanzz(x, alt) - 3/y) / (2 * y) :
      (3 * (3 + y) * atan5series(y) - 1) / 6;
  }

  Math::real NormalGravity::Hf(real x, bool alt) {
    // z = sqrt(x)
    // Compute
    //   H(z) = (3*Q(z)+z*diff(Q(z),z))*(1+z^2)
    //        = (3 * (1 + 1/z^2) * (1 - atan(z)/z) - 1) / z^2
    //        = q'(z)/z^2, with q'(z) defined by H+M, Eq 2-67, with z = E/u
    real y = alt ? -x / (1 + x) : x;
    return !(4 * fabs(y) < 1) ?  // Backwards test to allow NaNs through
      (3 * (1 + 1/y) * (1 - atanzz(x, alt)) - 1) / y :
      1 - 3 * (1 + y) * atan5series(y);
  }

  Math::real NormalGravity::QH3f(real x, bool alt) {
    // z = sqrt(x)
    // (Q(z) - H(z)/3) / z^2
    //   = - (1+z^2)/(3*z) * d(Q(z))/dz - Q(z)
    //   = ((15+9*z^2)*atan(z)-4*z^3-15*z)/(6*z^7)
    //   = ((25+15*z^2)*atan7+3)/10
    real y = alt ? -x / (1 + x) : x;
    return !(4 * fabs(y) < 1) ? // Backwards test to allow NaNs through
      ((9 + 15/y) * atanzz(x, alt) - 4 - 15/y) / (6 * Math::sq(y)) :
      ((25 + 15*y) * atan7series(y) + 3)/10;
  }

  Math::real NormalGravity::Jn(int n) const {
    // Note Jn(0) = -1; Jn(2) = _jJ2; Jn(odd) = 0
    if (n & 1 || n < 0)
      return 0;
    n /= 2;
    real e2n = 1;            // Perhaps this should just be e2n = pow(-_e2, n);
    for (int j = n; j--;)
      e2n *= -_e2;
    return                      // H+M, Eq 2-92
      -3 * e2n * ((1 - n) + 5 * n * _jJ2 / _e2) / ((2 * n + 1) * (2 * n + 3));
  }

  Math::real NormalGravity::SurfaceGravity(real lat) const {
    real sphi = Math::sind(Math::LatFix(lat));
    // H+M, Eq 2-78
    return (_gammae + _k * Math::sq(sphi)) / sqrt(1 - _e2 * Math::sq(sphi));
  }

  Math::real NormalGravity::V0(real X, real Y, real Z,
                               real& GammaX, real& GammaY, real& GammaZ) const
  {
    // See H+M, Sec 6-2
    real
      p = hypot(X, Y),
      clam = p != 0 ? X/p : 1,
      slam = p != 0 ? Y/p : 0,
      r = hypot(p, Z);
    if (_f < 0) swap(p, Z);
    real
      Q = Math::sq(r) - Math::sq(_eE),
      t2 = Math::sq(2 * _eE * Z),
      disc = sqrt(Math::sq(Q) + t2),
      // This is H+M, Eq 6-8a, but generalized to deal with Q negative
      // accurately.
      u = sqrt((Q >= 0 ? (Q + disc) : t2 / (disc - Q)) / 2),
      uE = hypot(u, _eE),
      // H+M, Eq 6-8b
      sbet = u != 0 ? Z * uE : copysign(sqrt(-Q), Z),
      cbet = u != 0 ? p * u : p,
      s = hypot(cbet, sbet);
    sbet = s != 0 ? sbet/s : 1;
    cbet = s != 0 ? cbet/s : 0;
    real
      z = _eE/u,
      z2 = Math::sq(z),
      den = hypot(u, _eE * sbet);
    if (_f < 0) {
      swap(sbet, cbet);
      swap(u, uE);
    }
    real
      invw = uE / den,          // H+M, Eq 2-63
      bu = _b / (u != 0 || _f < 0 ? u : _eE),
      // Qf(z2->inf, false) = pi/(4*z^3)
      q = ((u != 0 || _f < 0 ? Qf(z2, _f < 0) : Math::pi() / 4) / _qQ0) *
        bu * Math::sq(bu),
      qp = _b * Math::sq(bu) * (u != 0 || _f < 0 ? Hf(z2, _f < 0) : 2) / _qQ0,
      ang = (Math::sq(sbet) - 1/real(3)) / 2,
      // H+M, Eqs 2-62 + 6-9, but omitting last (rotational) term.
      Vres = _gGM * (u != 0 || _f < 0 ?
                    atanzz(z2, _f < 0) / u :
                    Math::pi() / (2 * _eE)) + _aomega2 * q * ang,
      // H+M, Eq 6-10
      gamu = - (_gGM + (_aomega2 * qp * ang)) * invw / Math::sq(uE),
      gamb = _aomega2 * q * sbet * cbet * invw / uE,
      t = u * invw / uE,
      gamp = t * cbet * gamu - invw * sbet * gamb;
    // H+M, Eq 6-12
    GammaX = gamp * clam;
    GammaY = gamp * slam;
    GammaZ = invw * sbet * gamu + t * cbet * gamb;
    return Vres;
  }

  Math::real NormalGravity::Phi(real X, real Y, real& fX, real& fY) const {
    fX = _omega2 * X;
    fY = _omega2 * Y;
    // N.B. fZ = 0;
    return _omega2 * (Math::sq(X) + Math::sq(Y)) / 2;
  }

  Math::real NormalGravity::U(real X, real Y, real Z,
                              real& gammaX, real& gammaY, real& gammaZ) const {
    real fX, fY;
    real Ures = V0(X, Y, Z, gammaX, gammaY, gammaZ) + Phi(X, Y, fX, fY);
    gammaX += fX;
    gammaY += fY;
    return Ures;
  }

  Math::real NormalGravity::Gravity(real lat, real h,
                                    real& gammay, real& gammaz) const {
    real X, Y, Z;
    real M[Geocentric::dim2_];
    _earth.IntForward(lat, 0, h, X, Y, Z, M);
    real gammaX, gammaY, gammaZ,
      Ures = U(X, Y, Z, gammaX, gammaY, gammaZ);
    // gammax = M[0] * gammaX + M[3] * gammaY + M[6] * gammaZ;
    gammay = M[1] * gammaX + M[4] * gammaY + M[7] * gammaZ;
    gammaz = M[2] * gammaX + M[5] * gammaY + M[8] * gammaZ;
    return Ures;
  }

  Math::real NormalGravity::J2ToFlattening(real a, real GM,
                                           real omega, real J2) {
    // Solve
    //   f = e^2 * (1 -  K * e/q0) - 3 * J2 = 0
    // for e^2 using Newton's method
    static const real maxe_ = 1 - numeric_limits<real>::epsilon();
    static const real eps2_ = sqrt(numeric_limits<real>::epsilon()) / 100;
    real
      K = 2 * Math::sq(a * omega) * a / (15 * GM),
      J0 = (1 - 4 * K / Math::pi()) / 3;
    if (!(GM > 0 && isfinite(K) && K >= 0))
      return Math::NaN();
    if (!(isfinite(J2) && J2 <= J0)) return Math::NaN();
    if (J2 == J0) return 1;
    // Solve e2 - f1 * f2 * K / Q0 - 3 * J2 = 0 for J2 close to J0;
    // subst e2 = ep2/(1+ep2), f2 = 1/(1+ep2), f1 = 1/sqrt(1+ep2), J2 = J0-dJ2,
    // Q0 = pi/(4*z^3) - 2/z^4 + (3*pi)/(4*z^5), z = sqrt(ep2), and balance two
    // leading terms to give
    real
      ep2 = fmax(Math::sq(32 * K / (3 * Math::sq(Math::pi()) * (J0 - J2))),
                -maxe_),
      e2 = fmin(ep2 / (1 + ep2), maxe_);
    for (int j = 0;
         j < maxit_ ||
           GEOGRAPHICLIB_PANIC("Convergence failure in NormalGravity");
         ++j) {
      real
        e2a = e2, ep2a = ep2,
        f2 = 1 - e2,            // (1 - f)^2
        f1 = sqrt(f2),          // (1 - f)
        Q0 = Qf(e2 < 0 ? -e2 : ep2, e2 < 0),
        h = e2 - f1 * f2 * K / Q0 - 3 * J2,
        dh = 1 - 3 * f1 * K * QH3f(e2 < 0 ? -e2 : ep2, e2 < 0) /
                     (2 * Math::sq(Q0));
      e2 = fmin(e2a - h / dh, maxe_);
      ep2 = fmax(e2 / (1 - e2), -maxe_);
      if (fabs(h) < eps2_ || e2 == e2a || ep2 == ep2a)
        break;
    }
    return e2 / (1 + sqrt(1 - e2));
  }

  Math::real NormalGravity::FlatteningToJ2(real a, real GM,
                                           real omega, real f) {
    real
      K = 2 * Math::sq(a * omega) * a / (15 * GM),
      f1 = 1 - f,
      f2 = Math::sq(f1),
      e2 = f * (2 - f);
    // H+M, Eq 2-90 + 2-92'
    return (e2 - K * f1 * f2 / Qf(f < 0 ? -e2 : e2 / f2, f < 0)) / 3;
  }

} // namespace GeographicLib
