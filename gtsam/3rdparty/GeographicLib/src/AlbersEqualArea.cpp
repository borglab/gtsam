/**
 * \file AlbersEqualArea.cpp
 * \brief Implementation for GeographicLib::AlbersEqualArea class
 *
 * Copyright (c) Charles Karney (2010-2017) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <GeographicLib/AlbersEqualArea.hpp>

#if defined(_MSC_VER)
// Squelch warnings about constant conditional expressions
#  pragma warning (disable: 4127)
#endif

namespace GeographicLib {

  using namespace std;

  AlbersEqualArea::AlbersEqualArea(real a, real f, real stdlat, real k0)
    : eps_(numeric_limits<real>::epsilon())
    , epsx_(Math::sq(eps_))
    , epsx2_(Math::sq(epsx_))
    , tol_(sqrt(eps_))
    , tol0_(tol_ * sqrt(sqrt(eps_)))
    , _a(a)
    , _f(f)
    , _fm(1 - _f)
    , _e2(_f * (2 - _f))
    , _e(sqrt(abs(_e2)))
    , _e2m(1 - _e2)
    , _qZ(1 + _e2m * atanhee(real(1)))
    , _qx(_qZ / ( 2 * _e2m ))
  {
    if (!(Math::isfinite(_a) && _a > 0))
      throw GeographicErr("Equatorial radius is not positive");
    if (!(Math::isfinite(_f) && _f < 1))
      throw GeographicErr("Polar semi-axis is not positive");
    if (!(Math::isfinite(k0) && k0 > 0))
      throw GeographicErr("Scale is not positive");
    if (!(abs(stdlat) <= 90))
      throw GeographicErr("Standard latitude not in [-90d, 90d]");
    real sphi, cphi;
    Math::sincosd(stdlat, sphi, cphi);
    Init(sphi, cphi, sphi, cphi, k0);
  }

  AlbersEqualArea::AlbersEqualArea(real a, real f, real stdlat1, real stdlat2,
                                   real k1)
    : eps_(numeric_limits<real>::epsilon())
    , epsx_(Math::sq(eps_))
    , epsx2_(Math::sq(epsx_))
    , tol_(sqrt(eps_))
    , tol0_(tol_ * sqrt(sqrt(eps_)))
    , _a(a)
    , _f(f)
    , _fm(1 - _f)
    , _e2(_f * (2 - _f))
    , _e(sqrt(abs(_e2)))
    , _e2m(1 - _e2)
    , _qZ(1 + _e2m * atanhee(real(1)))
    , _qx(_qZ / ( 2 * _e2m ))
  {
    if (!(Math::isfinite(_a) && _a > 0))
      throw GeographicErr("Equatorial radius is not positive");
    if (!(Math::isfinite(_f) && _f < 1))
      throw GeographicErr("Polar semi-axis is not positive");
    if (!(Math::isfinite(k1) && k1 > 0))
      throw GeographicErr("Scale is not positive");
    if (!(abs(stdlat1) <= 90))
      throw GeographicErr("Standard latitude 1 not in [-90d, 90d]");
    if (!(abs(stdlat2) <= 90))
      throw GeographicErr("Standard latitude 2 not in [-90d, 90d]");
    real sphi1, cphi1, sphi2, cphi2;
    Math::sincosd(stdlat1, sphi1, cphi1);
    Math::sincosd(stdlat2, sphi2, cphi2);
    Init(sphi1, cphi1, sphi2, cphi2, k1);
  }

  AlbersEqualArea::AlbersEqualArea(real a, real f,
                                   real sinlat1, real coslat1,
                                   real sinlat2, real coslat2,
                                   real k1)
    : eps_(numeric_limits<real>::epsilon())
    , epsx_(Math::sq(eps_))
    , epsx2_(Math::sq(epsx_))
    , tol_(sqrt(eps_))
    , tol0_(tol_ * sqrt(sqrt(eps_)))
    , _a(a)
    , _f(f)
    , _fm(1 - _f)
    , _e2(_f * (2 - _f))
    , _e(sqrt(abs(_e2)))
    , _e2m(1 - _e2)
    , _qZ(1 + _e2m * atanhee(real(1)))
    , _qx(_qZ / ( 2 * _e2m ))
  {
    if (!(Math::isfinite(_a) && _a > 0))
      throw GeographicErr("Equatorial radius is not positive");
    if (!(Math::isfinite(_f) && _f < 1))
      throw GeographicErr("Polar semi-axis is not positive");
    if (!(Math::isfinite(k1) && k1 > 0))
      throw GeographicErr("Scale is not positive");
    if (!(coslat1 >= 0))
      throw GeographicErr("Standard latitude 1 not in [-90d, 90d]");
    if (!(coslat2 >= 0))
      throw GeographicErr("Standard latitude 2 not in [-90d, 90d]");
    if (!(abs(sinlat1) <= 1 && coslat1 <= 1) || (coslat1 == 0 && sinlat1 == 0))
      throw GeographicErr("Bad sine/cosine of standard latitude 1");
    if (!(abs(sinlat2) <= 1 && coslat2 <= 1) || (coslat2 == 0 && sinlat2 == 0))
      throw GeographicErr("Bad sine/cosine of standard latitude 2");
    if (coslat1 == 0 && coslat2 == 0 && sinlat1 * sinlat2 <= 0)
      throw GeographicErr
        ("Standard latitudes cannot be opposite poles");
    Init(sinlat1, coslat1, sinlat2, coslat2, k1);
  }

  void AlbersEqualArea::Init(real sphi1, real cphi1,
                             real sphi2, real cphi2, real k1) {
    {
      real r;
      r = Math::hypot(sphi1, cphi1);
      sphi1 /= r; cphi1 /= r;
      r = Math::hypot(sphi2, cphi2);
      sphi2 /= r; cphi2 /= r;
    }
    bool polar = (cphi1 == 0);
    cphi1 = max(epsx_, cphi1);   // Avoid singularities at poles
    cphi2 = max(epsx_, cphi2);
    // Determine hemisphere of tangent latitude
    _sign = sphi1 + sphi2 >= 0 ? 1 : -1;
    // Internally work with tangent latitude positive
    sphi1 *= _sign; sphi2 *= _sign;
    if (sphi1 > sphi2) {
      swap(sphi1, sphi2); swap(cphi1, cphi2); // Make phi1 < phi2
    }
    real
      tphi1 = sphi1/cphi1, tphi2 = sphi2/cphi2;

    // q = (1-e^2)*(sphi/(1-e^2*sphi^2) - atanhee(sphi))
    // qZ = q(pi/2) = (1 + (1-e^2)*atanhee(1))
    // atanhee(x) = atanh(e*x)/e
    // q = sxi * qZ
    // dq/dphi = 2*(1-e^2)*cphi/(1-e^2*sphi^2)^2
    //
    // n = (m1^2-m2^2)/(q2-q1) -> sin(phi0) for phi1, phi2 -> phi0
    // C = m1^2 + n*q1 = (m1^2*q2-m2^2*q1)/(q2-q1)
    // let
    //   rho(pi/2)/rho(-pi/2) = (1-s)/(1+s)
    //   s = n*qZ/C
    //     = qZ * (m1^2-m2^2)/(m1^2*q2-m2^2*q1)
    //     = qZ * (scbet2^2 - scbet1^2)/(scbet2^2*q2 - scbet1^2*q1)
    //     = (scbet2^2 - scbet1^2)/(scbet2^2*sxi2 - scbet1^2*sxi1)
    //     = (tbet2^2 - tbet1^2)/(scbet2^2*sxi2 - scbet1^2*sxi1)
    // 1-s = -((1-sxi2)*scbet2^2 - (1-sxi1)*scbet1^2)/
    //         (scbet2^2*sxi2 - scbet1^2*sxi1)
    //
    // Define phi0 to give same value of s, i.e.,
    //  s = sphi0 * qZ / (m0^2 + sphi0*q0)
    //    = sphi0 * scbet0^2 / (1/qZ + sphi0 * scbet0^2 * sxi0)

    real tphi0, C;
    if (polar || tphi1 == tphi2) {
      tphi0 = tphi2;
      C = 1;                    // ignored
    } else {
      real
        tbet1 = _fm * tphi1, scbet12 = 1 + Math::sq(tbet1),
        tbet2 = _fm * tphi2, scbet22 = 1 + Math::sq(tbet2),
        txi1 = txif(tphi1), cxi1 = 1/hyp(txi1), sxi1 = txi1 * cxi1,
        txi2 = txif(tphi2), cxi2 = 1/hyp(txi2), sxi2 = txi2 * cxi2,
        dtbet2 = _fm * (tbet1 + tbet2),
        es1 = 1 - _e2 * Math::sq(sphi1), es2 = 1 - _e2 * Math::sq(sphi2),
        /*
        dsxi = ( (_e2 * sq(sphi2 + sphi1) + es2 + es1) / (2 * es2 * es1) +
                 Datanhee(sphi2, sphi1) ) * Dsn(tphi2, tphi1, sphi2, sphi1) /
        ( 2 * _qx ),
        */
        dsxi = ( (1 + _e2 * sphi1 * sphi2) / (es2 * es1) +
                 Datanhee(sphi2, sphi1) ) * Dsn(tphi2, tphi1, sphi2, sphi1) /
        ( 2 * _qx ),
        den = (sxi2 + sxi1) * dtbet2 + (scbet22 + scbet12) * dsxi,
        // s = (sq(tbet2) - sq(tbet1)) / (scbet22*sxi2 - scbet12*sxi1)
        s = 2 * dtbet2 / den,
        // 1-s = -(sq(scbet2)*(1-sxi2) - sq(scbet1)*(1-sxi1)) /
        //        (scbet22*sxi2 - scbet12*sxi1)
        // Write
        //   sq(scbet)*(1-sxi) = sq(scbet)*(1-sphi) * (1-sxi)/(1-sphi)
        sm1 = -Dsn(tphi2, tphi1, sphi2, sphi1) *
        ( -( ((sphi2 <= 0 ? (1 - sxi2) / (1 - sphi2) :
               Math::sq(cxi2/cphi2) * (1 + sphi2) / (1 + sxi2)) +
              (sphi1 <= 0 ? (1 - sxi1) / (1 - sphi1) :
               Math::sq(cxi1/cphi1) * (1 + sphi1) / (1 + sxi1))) ) *
          (1 + _e2 * (sphi1 + sphi2 + sphi1 * sphi2)) /
          (1 +       (sphi1 + sphi2 + sphi1 * sphi2)) +
          (scbet22 * (sphi2 <= 0 ? 1 - sphi2 :
                      Math::sq(cphi2) / ( 1 + sphi2)) +
           scbet12 * (sphi1 <= 0 ? 1 - sphi1 : Math::sq(cphi1) / ( 1 + sphi1)))
          * (_e2 * (1 + sphi1 + sphi2 + _e2 * sphi1 * sphi2)/(es1 * es2)
          +_e2m * DDatanhee(sphi1, sphi2) ) / _qZ ) / den;
      // C = (scbet22*sxi2 - scbet12*sxi1) / (scbet22 * scbet12 * (sx2 - sx1))
      C = den / (2 * scbet12 * scbet22 * dsxi);
      tphi0 = (tphi2 + tphi1)/2;
      real stol = tol0_ * max(real(1), abs(tphi0));
      for (int i = 0; i < 2*numit0_ || GEOGRAPHICLIB_PANIC; ++i) {
        // Solve (scbet0^2 * sphi0) / (1/qZ + scbet0^2 * sphi0 * sxi0) = s
        // for tphi0 by Newton's method on
        // v(tphi0) = (scbet0^2 * sphi0) - s * (1/qZ + scbet0^2 * sphi0 * sxi0)
        //          = 0
        // Alt:
        // (scbet0^2 * sphi0) / (1/qZ - scbet0^2 * sphi0 * (1-sxi0))
        //          = s / (1-s)
        // w(tphi0) = (1-s) * (scbet0^2 * sphi0)
        //             - s  * (1/qZ - scbet0^2 * sphi0 * (1-sxi0))
        //          = (1-s) * (scbet0^2 * sphi0)
        //             - S/qZ  * (1 - scbet0^2 * sphi0 * (qZ-q0))
        // Now
        // qZ-q0 = (1+e2*sphi0)*(1-sphi0)/(1-e2*sphi0^2) +
        //         (1-e2)*atanhee((1-sphi0)/(1-e2*sphi0))
        // In limit sphi0 -> 1, qZ-q0 -> 2*(1-sphi0)/(1-e2), so wrte
        // qZ-q0 = 2*(1-sphi0)/(1-e2) + A + B
        // A = (1-sphi0)*( (1+e2*sphi0)/(1-e2*sphi0^2) - (1+e2)/(1-e2) )
        //   = -e2 *(1-sphi0)^2 * (2+(1+e2)*sphi0) / ((1-e2)*(1-e2*sphi0^2))
        // B = (1-e2)*atanhee((1-sphi0)/(1-e2*sphi0)) - (1-sphi0)
        //   = (1-sphi0)*(1-e2)/(1-e2*sphi0)*
        //     ((atanhee(x)/x-1) - e2*(1-sphi0)/(1-e2))
        // x = (1-sphi0)/(1-e2*sphi0), atanhee(x)/x = atanh(e*x)/(e*x)
        //
        // 1 - scbet0^2 * sphi0 * (qZ-q0)
        //   = 1 - scbet0^2 * sphi0 * (2*(1-sphi0)/(1-e2) + A + B)
        //   = D - scbet0^2 * sphi0 * (A + B)
        // D = 1 - scbet0^2 * sphi0 * 2*(1-sphi0)/(1-e2)
        //   = (1-sphi0)*(1-e2*(1+2*sphi0*(1+sphi0)))/((1-e2)*(1+sphi0))
        // dD/dsphi0 = -2*(1-e2*sphi0^2*(2*sphi0+3))/((1-e2)*(1+sphi0)^2)
        // d(A+B)/dsphi0 = 2*(1-sphi0^2)*e2*(2-e2*(1+sphi0^2))/
        //                 ((1-e2)*(1-e2*sphi0^2)^2)

        real
          scphi02 = 1 + Math::sq(tphi0), scphi0 = sqrt(scphi02),
          // sphi0m = 1-sin(phi0) = 1/( sec(phi0) * (tan(phi0) + sec(phi0)) )
          sphi0 = tphi0 / scphi0, sphi0m = 1/(scphi0 * (tphi0 + scphi0)),
          // scbet0^2 * sphi0
          g = (1 + Math::sq( _fm * tphi0 )) * sphi0,
          // dg/dsphi0 = dg/dtphi0 * scphi0^3
          dg = _e2m * scphi02 * (1 + 2 * Math::sq(tphi0)) + _e2,
          D = sphi0m * (1 - _e2*(1 + 2*sphi0*(1+sphi0))) / (_e2m * (1+sphi0)),
          // dD/dsphi0
          dD = -2 * (1 - _e2*Math::sq(sphi0) * (2*sphi0+3)) /
               (_e2m * Math::sq(1+sphi0)),
          A = -_e2 * Math::sq(sphi0m) * (2+(1+_e2)*sphi0) /
              (_e2m*(1-_e2*Math::sq(sphi0))),
          B = (sphi0m * _e2m / (1 - _e2*sphi0) *
               (atanhxm1(_e2 *
                         Math::sq(sphi0m / (1-_e2*sphi0))) - _e2*sphi0m/_e2m)),
          // d(A+B)/dsphi0
          dAB = (2 * _e2 * (2 - _e2 * (1 + Math::sq(sphi0))) /
                 (_e2m * Math::sq(1 - _e2*Math::sq(sphi0)) * scphi02)),
          u = sm1 * g - s/_qZ * ( D - g * (A + B) ),
          // du/dsphi0
          du = sm1 * dg - s/_qZ * (dD - dg * (A + B) - g * dAB),
          dtu = -u/du * (scphi0 * scphi02);
        tphi0 += dtu;
        if (!(abs(dtu) >= stol))
          break;
      }
    }
    _txi0 = txif(tphi0); _scxi0 = hyp(_txi0); _sxi0 = _txi0 / _scxi0;
    _n0 = tphi0/hyp(tphi0);
    _m02 = 1 / (1 + Math::sq(_fm * tphi0));
    _nrho0 = polar ? 0 : _a * sqrt(_m02);
    _k0 = sqrt(tphi1 == tphi2 ? 1 : C / (_m02 + _n0 * _qZ * _sxi0)) * k1;
    _k2 = Math::sq(_k0);
    _lat0 = _sign * atan(tphi0)/Math::degree();
  }

  const AlbersEqualArea& AlbersEqualArea::CylindricalEqualArea() {
    static const AlbersEqualArea
      cylindricalequalarea(Constants::WGS84_a(), Constants::WGS84_f(),
                           real(0), real(1), real(0), real(1), real(1));
    return cylindricalequalarea;
  }

  const AlbersEqualArea& AlbersEqualArea::AzimuthalEqualAreaNorth() {
    static const AlbersEqualArea
      azimuthalequalareanorth(Constants::WGS84_a(), Constants::WGS84_f(),
                              real(1), real(0), real(1), real(0), real(1));
    return azimuthalequalareanorth;
  }

  const AlbersEqualArea& AlbersEqualArea::AzimuthalEqualAreaSouth() {
    static const AlbersEqualArea
      azimuthalequalareasouth(Constants::WGS84_a(), Constants::WGS84_f(),
                              real(-1), real(0), real(-1), real(0), real(1));
    return azimuthalequalareasouth;
  }

  Math::real AlbersEqualArea::txif(real tphi) const {
    // sxi = ( sphi/(1-e2*sphi^2) + atanhee(sphi) ) /
    //       ( 1/(1-e2) + atanhee(1) )
    //
    // txi = ( sphi/(1-e2*sphi^2) + atanhee(sphi) ) /
    //       sqrt( ( (1+e2*sphi)*(1-sphi)/( (1-e2*sphi^2) * (1-e2) ) +
    //               atanhee((1-sphi)/(1-e2*sphi)) ) *
    //             ( (1-e2*sphi)*(1+sphi)/( (1-e2*sphi^2) * (1-e2) ) +
    //               atanhee((1+sphi)/(1+e2*sphi)) ) )
    //
    // subst 1-sphi = cphi^2/(1+sphi)
    int s = tphi < 0 ? -1 : 1;  // Enforce odd parity
    tphi *= s;
    real
      cphi2 = 1 / (1 + Math::sq(tphi)),
      sphi = tphi * sqrt(cphi2),
      es1 = _e2 * sphi,
      es2m1 = 1 - es1 * sphi,
      sp1 = 1 + sphi,
      es1m1 = (1 - es1) * sp1,
      es2m1a = _e2m * es2m1,
      es1p1 = sp1 / (1 + es1);
    return s * ( sphi / es2m1 + atanhee(sphi) ) /
      sqrt( ( cphi2 / (es1p1 * es2m1a) + atanhee(cphi2 / es1m1) ) *
            ( es1m1 / es2m1a + atanhee(es1p1) ) );
  }

  Math::real AlbersEqualArea::tphif(real txi) const {
    real
      tphi = txi,
      stol = tol_ * max(real(1), abs(txi));
    // CHECK: min iterations = 1, max iterations = 2; mean = 1.99
    for (int i = 0; i < numit_ || GEOGRAPHICLIB_PANIC; ++i) {
      // dtxi/dtphi = (scxi/scphi)^3 * 2*(1-e^2)/(qZ*(1-e^2*sphi^2)^2)
      real
        txia = txif(tphi),
        tphi2 = Math::sq(tphi),
        scphi2 = 1 + tphi2,
        scterm = scphi2/(1 + Math::sq(txia)),
        dtphi = (txi - txia) * scterm * sqrt(scterm) *
        _qx * Math::sq(1 - _e2 * tphi2 / scphi2);
      tphi += dtphi;
      if (!(abs(dtphi) >= stol))
        break;
    }
    return tphi;
  }

  // return atanh(sqrt(x))/sqrt(x) - 1 = y/3 + y^2/5 + y^3/7 + ...
  // typical x < e^2 = 2*f
  Math::real AlbersEqualArea::atanhxm1(real x) {
    real s = 0;
    if (abs(x) < real(0.5)) {
      real os = -1, y = 1, k = 1;
      while (os != s) {
        os = s;
        y *= x;                 // y = x^n
        k += 2;                 // k = 2*n + 1
        s += y/k;               // sum( x^n/(2*n + 1) )
      }
    } else {
      real xs = sqrt(abs(x));
      s = (x > 0 ? Math::atanh(xs) : atan(xs)) / xs - 1;
    }
    return s;
  }

  // return (Datanhee(1,y) - Datanhee(1,x))/(y-x)
  Math::real AlbersEqualArea::DDatanhee(real x, real y) const {
    real s = 0;
    if (_e2 * (abs(x) + abs(y)) < real(0.5)) {
      real os = -1, z = 1, k = 1, t = 0, c = 0, en = 1;
      while (os != s) {
        os = s;
        t = y * t + z; c += t; z *= x;
        t = y * t + z; c += t; z *= x;
        k += 2; en *= _e2;
        // Here en[l] = e2^l, k[l] = 2*l + 1,
        // c[l] = sum( x^i * y^j; i >= 0, j >= 0, i+j < 2*l)
        s += en * c / k;
      }
      // Taylor expansion is
      // s = sum( c[l] * e2^l / (2*l + 1), l, 1, N)
    } else
      s = (Datanhee(1, y) - Datanhee(x, y))/(1 - x);
    return s;
  }

  void AlbersEqualArea::Forward(real lon0, real lat, real lon,
                                real& x, real& y, real& gamma, real& k) const {
    lon = Math::AngDiff(lon0, lon);
    lat *= _sign;
    real sphi, cphi;
    Math::sincosd(Math::LatFix(lat) * _sign, sphi, cphi);
    cphi = max(epsx_, cphi);
    real
      lam = lon * Math::degree(),
      tphi = sphi/cphi, txi = txif(tphi), sxi = txi/hyp(txi),
      dq = _qZ * Dsn(txi, _txi0, sxi, _sxi0) * (txi - _txi0),
      drho = - _a * dq / (sqrt(_m02 - _n0 * dq) + _nrho0 / _a),
      theta = _k2 * _n0 * lam, stheta = sin(theta), ctheta = cos(theta),
      t = _nrho0 + _n0 * drho;
    x = t * (_n0 != 0 ? stheta / _n0 : _k2 * lam) / _k0;
    y = (_nrho0 *
         (_n0 != 0 ?
          (ctheta < 0 ? 1 - ctheta : Math::sq(stheta)/(1 + ctheta)) / _n0 :
          0)
         - drho * ctheta) / _k0;
    k = _k0 * (t != 0 ? t * hyp(_fm * tphi) / _a : 1);
    y *= _sign;
    gamma = _sign * theta / Math::degree();
  }

  void AlbersEqualArea::Reverse(real lon0, real x, real y,
                                real& lat, real& lon,
                                real& gamma, real& k) const {
    y *= _sign;
    real
      nx = _k0 * _n0 * x, ny = _k0 * _n0 * y, y1 =  _nrho0 - ny,
      den = Math::hypot(nx, y1) + _nrho0, // 0 implies origin with polar aspect
      drho = den != 0 ? (_k0*x*nx - 2*_k0*y*_nrho0 + _k0*y*ny) / den : 0,
      // dsxia = scxi0 * dsxi
      dsxia = - _scxi0 * (2 * _nrho0 + _n0 * drho) * drho /
              (Math::sq(_a) * _qZ),
      txi = (_txi0 + dsxia) / sqrt(max(1 - dsxia * (2*_txi0 + dsxia), epsx2_)),
      tphi = tphif(txi),
      theta = atan2(nx, y1),
      lam = _n0 != 0 ? theta / (_k2 * _n0) : x / (y1 * _k0);
    gamma = _sign * theta / Math::degree();
    lat = Math::atand(_sign * tphi);
    lon = lam / Math::degree();
    lon = Math::AngNormalize(lon + Math::AngNormalize(lon0));
    k = _k0 * (den != 0 ? (_nrho0 + _n0 * drho) * hyp(_fm * tphi) / _a : 1);
  }

  void AlbersEqualArea::SetScale(real lat, real k) {
    if (!(Math::isfinite(k) && k > 0))
      throw GeographicErr("Scale is not positive");
    if (!(abs(lat) < 90))
      throw GeographicErr("Latitude for SetScale not in (-90d, 90d)");
    real x, y, gamma, kold;
    Forward(0, lat, 0, x, y, gamma, kold);
    k /= kold;
    _k0 *= k;
    _k2 = Math::sq(_k0);
  }

} // namespace GeographicLib
