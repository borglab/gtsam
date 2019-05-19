/**
 * \file TransverseMercatorExact.cpp
 * \brief Implementation for GeographicLib::TransverseMercatorExact class
 *
 * Copyright (c) Charles Karney (2008-2017) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 *
 * The relevant section of Lee's paper is part V, pp 67--101,
 * <a href="https://doi.org/10.3138/X687-1574-4325-WM62">Conformal
 * Projections Based On Jacobian Elliptic Functions</a>.
 *
 * The method entails using the Thompson Transverse Mercator as an
 * intermediate projection.  The projections from the intermediate
 * coordinates to [\e phi, \e lam] and [\e x, \e y] are given by elliptic
 * functions.  The inverse of these projections are found by Newton's method
 * with a suitable starting guess.
 *
 * This implementation and notation closely follows Lee, with the following
 * exceptions:
 * <center><table>
 * <tr><th>Lee    <th>here    <th>Description
 * <tr><td>x/a    <td>xi      <td>Northing (unit Earth)
 * <tr><td>y/a    <td>eta     <td>Easting (unit Earth)
 * <tr><td>s/a    <td>sigma   <td>xi + i * eta
 * <tr><td>y      <td>x       <td>Easting
 * <tr><td>x      <td>y       <td>Northing
 * <tr><td>k      <td>e       <td>eccentricity
 * <tr><td>k^2    <td>mu      <td>elliptic function parameter
 * <tr><td>k'^2   <td>mv      <td>elliptic function complementary parameter
 * <tr><td>m      <td>k       <td>scale
 * <tr><td>zeta   <td>zeta    <td>complex longitude = Mercator = chi in paper
 * <tr><td>s      <td>sigma   <td>complex GK = zeta in paper
 * </table></center>
 *
 * Minor alterations have been made in some of Lee's expressions in an
 * attempt to control round-off.  For example atanh(sin(phi)) is replaced by
 * asinh(tan(phi)) which maintains accuracy near phi = pi/2.  Such changes
 * are noted in the code.
 **********************************************************************/

#include <GeographicLib/TransverseMercatorExact.hpp>

#if defined(_MSC_VER)
// Squelch warnings about constant conditional expressions
#  pragma warning (disable: 4127)
#endif

namespace GeographicLib {

  using namespace std;

  TransverseMercatorExact::TransverseMercatorExact(real a, real f, real k0,
                                                   bool extendp)
    : tol_(numeric_limits<real>::epsilon())
    , tol2_(real(0.1) * tol_)
    , taytol_(pow(tol_, real(0.6)))
    , _a(a)
    , _f(f)
    , _k0(k0)
    , _mu(_f * (2 - _f))        // e^2
    , _mv(1 - _mu)              // 1 - e^2
    , _e(sqrt(_mu))
    , _extendp(extendp)
    , _Eu(_mu)
    , _Ev(_mv)
  {
    if (!(Math::isfinite(_a) && _a > 0))
      throw GeographicErr("Equatorial radius is not positive");
    if (!(_f > 0))
      throw GeographicErr("Flattening is not positive");
    if (!(_f < 1))
      throw GeographicErr("Polar semi-axis is not positive");
    if (!(Math::isfinite(_k0) && _k0 > 0))
      throw GeographicErr("Scale is not positive");
  }

  const TransverseMercatorExact& TransverseMercatorExact::UTM() {
    static const TransverseMercatorExact utm(Constants::WGS84_a(),
                                             Constants::WGS84_f(),
                                             Constants::UTM_k0());
    return utm;
  }

  void TransverseMercatorExact::zeta(real /*u*/, real snu, real cnu, real dnu,
                                     real /*v*/, real snv, real cnv, real dnv,
                                     real& taup, real& lam) const {
    // Lee 54.17 but write
    // atanh(snu * dnv) = asinh(snu * dnv / sqrt(cnu^2 + _mv * snu^2 * snv^2))
    // atanh(_e * snu / dnv) =
    //         asinh(_e * snu / sqrt(_mu * cnu^2 + _mv * cnv^2))
    // Overflow value s.t. atan(overflow) = pi/2
    static const real
      overflow = 1 / Math::sq(std::numeric_limits<real>::epsilon());
    real
      d1 = sqrt(Math::sq(cnu) + _mv * Math::sq(snu * snv)),
      d2 = sqrt(_mu * Math::sq(cnu) + _mv * Math::sq(cnv)),
      t1 = (d1 != 0 ? snu * dnv / d1 : (snu < 0 ? -overflow : overflow)),
      t2 = (d2 != 0 ? sinh( _e * Math::asinh(_e * snu / d2) ) :
            (snu < 0 ? -overflow : overflow));
    // psi = asinh(t1) - asinh(t2)
    // taup = sinh(psi)
    taup = t1 * Math::hypot(real(1), t2) - t2 * Math::hypot(real(1), t1);
    lam = (d1 != 0 && d2 != 0) ?
      atan2(dnu * snv, cnu * cnv) - _e * atan2(_e * cnu * snv, dnu * cnv) :
      0;
  }

  void TransverseMercatorExact::dwdzeta(real /*u*/,
                                        real snu, real cnu, real dnu,
                                        real /*v*/,
                                        real snv, real cnv, real dnv,
                                        real& du, real& dv) const {
    // Lee 54.21 but write (1 - dnu^2 * snv^2) = (cnv^2 + _mu * snu^2 * snv^2)
    // (see A+S 16.21.4)
    real d = _mv * Math::sq(Math::sq(cnv) + _mu * Math::sq(snu * snv));
    du =  cnu * dnu * dnv * (Math::sq(cnv) - _mu * Math::sq(snu * snv)) / d;
    dv = -snu * snv * cnv * (Math::sq(dnu * dnv) + _mu * Math::sq(cnu)) / d;
  }

  // Starting point for zetainv
  bool TransverseMercatorExact::zetainv0(real psi, real lam,
                                         real& u, real& v) const {
    bool retval = false;
    if (psi < -_e * Math::pi()/4 &&
        lam > (1 - 2 * _e) * Math::pi()/2 &&
        psi < lam - (1 - _e) * Math::pi()/2) {
      // N.B. this branch is normally not taken because psi < 0 is converted
      // psi > 0 by Forward.
      //
      // There's a log singularity at w = w0 = Eu.K() + i * Ev.K(),
      // corresponding to the south pole, where we have, approximately
      //
      //   psi = _e + i * pi/2 - _e * atanh(cos(i * (w - w0)/(1 + _mu/2)))
      //
      // Inverting this gives:
      real
        psix = 1 - psi / _e,
        lamx = (Math::pi()/2 - lam) / _e;
      u = Math::asinh(sin(lamx) / Math::hypot(cos(lamx), sinh(psix))) *
        (1 + _mu/2);
      v = atan2(cos(lamx), sinh(psix)) * (1 + _mu/2);
      u = _Eu.K() - u;
      v = _Ev.K() - v;
    } else if (psi < _e * Math::pi()/2 &&
               lam > (1 - 2 * _e) * Math::pi()/2) {
      // At w = w0 = i * Ev.K(), we have
      //
      //     zeta = zeta0 = i * (1 - _e) * pi/2
      //     zeta' = zeta'' = 0
      //
      // including the next term in the Taylor series gives:
      //
      // zeta = zeta0 - (_mv * _e) / 3 * (w - w0)^3
      //
      // When inverting this, we map arg(w - w0) = [-90, 0] to
      // arg(zeta - zeta0) = [-90, 180]
      real
        dlam = lam - (1 - _e) * Math::pi()/2,
        rad = Math::hypot(psi, dlam),
        // atan2(dlam-psi, psi+dlam) + 45d gives arg(zeta - zeta0) in range
        // [-135, 225).  Subtracting 180 (since multiplier is negative) makes
        // range [-315, 45).  Multiplying by 1/3 (for cube root) gives range
        // [-105, 15).  In particular the range [-90, 180] in zeta space maps
        // to [-90, 0] in w space as required.
        ang = atan2(dlam-psi, psi+dlam) - real(0.75) * Math::pi();
      // Error using this guess is about 0.21 * (rad/e)^(5/3)
      retval = rad < _e * taytol_;
      rad = Math::cbrt(3 / (_mv * _e) * rad);
      ang /= 3;
      u = rad * cos(ang);
      v = rad * sin(ang) + _Ev.K();
    } else {
      // Use spherical TM, Lee 12.6 -- writing atanh(sin(lam) / cosh(psi)) =
      // asinh(sin(lam) / hypot(cos(lam), sinh(psi))).  This takes care of the
      // log singularity at zeta = Eu.K() (corresponding to the north pole)
      v = Math::asinh(sin(lam) / Math::hypot(cos(lam), sinh(psi)));
      u = atan2(sinh(psi), cos(lam));
      // But scale to put 90,0 on the right place
      u *= _Eu.K() / (Math::pi()/2);
      v *= _Eu.K() / (Math::pi()/2);
    }
    return retval;
  }

  // Invert zeta using Newton's method
  void TransverseMercatorExact::zetainv(real taup, real lam,
                                        real& u, real& v) const  {
    real
      psi = Math::asinh(taup),
      scal = 1/Math::hypot(real(1), taup);
    if (zetainv0(psi, lam, u, v))
      return;
    real stol2 = tol2_ / Math::sq(max(psi, real(1)));
    // min iterations = 2, max iterations = 6; mean = 4.0
    for (int i = 0, trip = 0; i < numit_ || GEOGRAPHICLIB_PANIC; ++i) {
      real snu, cnu, dnu, snv, cnv, dnv;
      _Eu.sncndn(u, snu, cnu, dnu);
      _Ev.sncndn(v, snv, cnv, dnv);
      real tau1, lam1, du1, dv1;
      zeta(u, snu, cnu, dnu, v, snv, cnv, dnv, tau1, lam1);
      dwdzeta(u, snu, cnu, dnu, v, snv, cnv, dnv, du1, dv1);
      tau1 -= taup;
      lam1 -= lam;
      tau1 *= scal;
      real
        delu = tau1 * du1 - lam1 * dv1,
        delv = tau1 * dv1 + lam1 * du1;
      u -= delu;
      v -= delv;
      if (trip)
        break;
      real delw2 = Math::sq(delu) + Math::sq(delv);
      if (!(delw2 >= stol2))
        ++trip;
    }
  }

  void TransverseMercatorExact::sigma(real /*u*/, real snu, real cnu, real dnu,
                                      real v, real snv, real cnv, real dnv,
                                      real& xi, real& eta) const {
    // Lee 55.4 writing
    // dnu^2 + dnv^2 - 1 = _mu * cnu^2 + _mv * cnv^2
    real d = _mu * Math::sq(cnu) + _mv * Math::sq(cnv);
    xi = _Eu.E(snu, cnu, dnu) - _mu * snu * cnu * dnu / d;
    eta = v - _Ev.E(snv, cnv, dnv) + _mv * snv * cnv * dnv / d;
  }

  void TransverseMercatorExact::dwdsigma(real /*u*/,
                                         real snu, real cnu, real dnu,
                                         real /*v*/,
                                         real snv, real cnv, real dnv,
                                         real& du, real& dv) const {
    // Reciprocal of 55.9: dw/ds = dn(w)^2/_mv, expanding complex dn(w) using
    // A+S 16.21.4
    real d = _mv * Math::sq(Math::sq(cnv) + _mu * Math::sq(snu * snv));
    real
      dnr = dnu * cnv * dnv,
      dni = - _mu * snu * cnu * snv;
    du = (Math::sq(dnr) - Math::sq(dni)) / d;
    dv = 2 * dnr * dni / d;
  }

  // Starting point for sigmainv
  bool TransverseMercatorExact::sigmainv0(real xi, real eta,
                                          real& u, real& v) const {
    bool retval = false;
    if (eta > real(1.25) * _Ev.KE() ||
        (xi < -real(0.25) * _Eu.E() && xi < eta - _Ev.KE())) {
      // sigma as a simple pole at w = w0 = Eu.K() + i * Ev.K() and sigma is
      // approximated by
      //
      // sigma = (Eu.E() + i * Ev.KE()) + 1/(w - w0)
      real
        x = xi - _Eu.E(),
        y = eta - _Ev.KE(),
        r2 = Math::sq(x) + Math::sq(y);
      u = _Eu.K() + x/r2;
      v = _Ev.K() - y/r2;
    } else if ((eta > real(0.75) * _Ev.KE() && xi < real(0.25) * _Eu.E())
               || eta > _Ev.KE()) {
      // At w = w0 = i * Ev.K(), we have
      //
      //     sigma = sigma0 = i * Ev.KE()
      //     sigma' = sigma'' = 0
      //
      // including the next term in the Taylor series gives:
      //
      // sigma = sigma0 - _mv / 3 * (w - w0)^3
      //
      // When inverting this, we map arg(w - w0) = [-pi/2, -pi/6] to
      // arg(sigma - sigma0) = [-pi/2, pi/2]
      // mapping arg = [-pi/2, -pi/6] to [-pi/2, pi/2]
      real
        deta = eta - _Ev.KE(),
        rad = Math::hypot(xi, deta),
        // Map the range [-90, 180] in sigma space to [-90, 0] in w space.  See
        // discussion in zetainv0 on the cut for ang.
        ang = atan2(deta-xi, xi+deta) - real(0.75) * Math::pi();
      // Error using this guess is about 0.068 * rad^(5/3)
      retval = rad < 2 * taytol_;
      rad = Math::cbrt(3 / _mv * rad);
      ang /= 3;
      u = rad * cos(ang);
      v = rad * sin(ang) + _Ev.K();
    } else {
      // Else use w = sigma * Eu.K/Eu.E (which is correct in the limit _e -> 0)
      u = xi * _Eu.K()/_Eu.E();
      v = eta * _Eu.K()/_Eu.E();
    }
    return retval;
  }

  // Invert sigma using Newton's method
  void TransverseMercatorExact::sigmainv(real xi, real eta,
                                         real& u, real& v) const {
    if (sigmainv0(xi, eta, u, v))
      return;
    // min iterations = 2, max iterations = 7; mean = 3.9
    for (int i = 0, trip = 0; i < numit_ || GEOGRAPHICLIB_PANIC; ++i) {
      real snu, cnu, dnu, snv, cnv, dnv;
      _Eu.sncndn(u, snu, cnu, dnu);
      _Ev.sncndn(v, snv, cnv, dnv);
      real xi1, eta1, du1, dv1;
      sigma(u, snu, cnu, dnu, v, snv, cnv, dnv, xi1, eta1);
      dwdsigma(u, snu, cnu, dnu, v, snv, cnv, dnv, du1, dv1);
      xi1 -= xi;
      eta1 -= eta;
      real
        delu = xi1 * du1 - eta1 * dv1,
        delv = xi1 * dv1 + eta1 * du1;
      u -= delu;
      v -= delv;
      if (trip)
        break;
      real delw2 = Math::sq(delu) + Math::sq(delv);
      if (!(delw2 >= tol2_))
        ++trip;
    }
  }

  void TransverseMercatorExact::Scale(real tau, real /*lam*/,
                                      real snu, real cnu, real dnu,
                                      real snv, real cnv, real dnv,
                                      real& gamma, real& k) const {
    real sec2 = 1 + Math::sq(tau);    // sec(phi)^2
    // Lee 55.12 -- negated for our sign convention.  gamma gives the bearing
    // (clockwise from true north) of grid north
    gamma = atan2(_mv * snu * snv * cnv, cnu * dnu * dnv);
    // Lee 55.13 with nu given by Lee 9.1 -- in sqrt change the numerator
    // from
    //
    //    (1 - snu^2 * dnv^2) to (_mv * snv^2 + cnu^2 * dnv^2)
    //
    // to maintain accuracy near phi = 90 and change the denomintor from
    //
    //    (dnu^2 + dnv^2 - 1) to (_mu * cnu^2 + _mv * cnv^2)
    //
    // to maintain accuracy near phi = 0, lam = 90 * (1 - e).  Similarly
    // rewrite sqrt term in 9.1 as
    //
    //    _mv + _mu * c^2 instead of 1 - _mu * sin(phi)^2
    k = sqrt(_mv + _mu / sec2) * sqrt(sec2) *
      sqrt( (_mv * Math::sq(snv) + Math::sq(cnu * dnv)) /
            (_mu * Math::sq(cnu) + _mv * Math::sq(cnv)) );
  }

  void TransverseMercatorExact::Forward(real lon0, real lat, real lon,
                                        real& x, real& y,
                                        real& gamma, real& k) const {
    lat = Math::LatFix(lat);
    lon = Math::AngDiff(lon0, lon);
    // Explicitly enforce the parity
    int
      latsign = (!_extendp && lat < 0) ? -1 : 1,
      lonsign = (!_extendp && lon < 0) ? -1 : 1;
    lon *= lonsign;
    lat *= latsign;
    bool backside = !_extendp && lon > 90;
    if (backside) {
      if (lat == 0)
        latsign = -1;
      lon = 180 - lon;
    }
    real
      lam = lon * Math::degree(),
      tau = Math::tand(lat);

    // u,v = coordinates for the Thompson TM, Lee 54
    real u, v;
    if (lat == 90) {
      u = _Eu.K();
      v = 0;
    } else if (lat == 0 && lon == 90 * (1 - _e)) {
      u = 0;
      v = _Ev.K();
    } else
      // tau = tan(phi), taup = sinh(psi)
      zetainv(Math::taupf(tau, _e), lam, u, v);

    real snu, cnu, dnu, snv, cnv, dnv;
    _Eu.sncndn(u, snu, cnu, dnu);
    _Ev.sncndn(v, snv, cnv, dnv);

    real xi, eta;
    sigma(u, snu, cnu, dnu, v, snv, cnv, dnv, xi, eta);
    if (backside)
      xi = 2 * _Eu.E() - xi;
    y = xi * _a * _k0 * latsign;
    x = eta * _a * _k0 * lonsign;

    if (lat == 90) {
      gamma = lon;
      k = 1;
    } else {
      // Recompute (tau, lam) from (u, v) to improve accuracy of Scale
      zeta(u, snu, cnu, dnu, v, snv, cnv, dnv, tau, lam);
      tau = Math::tauf(tau, _e);
      Scale(tau, lam, snu, cnu, dnu, snv, cnv, dnv, gamma, k);
      gamma /= Math::degree();
    }
    if (backside)
      gamma = 180 - gamma;
    gamma *= latsign * lonsign;
    k *= _k0;
  }

  void TransverseMercatorExact::Reverse(real lon0, real x, real y,
                                        real& lat, real& lon,
                                        real& gamma, real& k) const {
    // This undoes the steps in Forward.
    real
      xi = y / (_a * _k0),
      eta = x / (_a * _k0);
    // Explicitly enforce the parity
    int
      latsign = !_extendp && y < 0 ? -1 : 1,
      lonsign = !_extendp && x < 0 ? -1 : 1;
    xi *= latsign;
    eta *= lonsign;
    bool backside = !_extendp && xi > _Eu.E();
    if (backside)
      xi = 2 * _Eu.E()- xi;

    // u,v = coordinates for the Thompson TM, Lee 54
    real u, v;
    if (xi == 0 && eta == _Ev.KE()) {
      u = 0;
      v = _Ev.K();
    } else
      sigmainv(xi, eta, u, v);

    real snu, cnu, dnu, snv, cnv, dnv;
    _Eu.sncndn(u, snu, cnu, dnu);
    _Ev.sncndn(v, snv, cnv, dnv);
    real phi, lam, tau;
    if (v != 0 || u != _Eu.K()) {
      zeta(u, snu, cnu, dnu, v, snv, cnv, dnv, tau, lam);
      tau = Math::tauf(tau, _e);
      phi = atan(tau);
      lat = phi / Math::degree();
      lon = lam / Math::degree();
      Scale(tau, lam, snu, cnu, dnu, snv, cnv, dnv, gamma, k);
      gamma /= Math::degree();
    } else {
      lat = 90;
      lon = lam = gamma = 0;
      k = 1;
    }

    if (backside)
      lon = 180 - lon;
    lon *= lonsign;
    lon = Math::AngNormalize(lon + Math::AngNormalize(lon0));
    lat *= latsign;
    if (backside)
      gamma = 180 - gamma;
    gamma *= latsign * lonsign;
    k *= _k0;
  }

} // namespace GeographicLib
