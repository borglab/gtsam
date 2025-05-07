/**
 * \file Rhumb.cpp
 * \brief Implementation for GeographicLib::Rhumb and GeographicLib::RhumbLine
 * classes
 *
 * Copyright (c) Charles Karney (2014-2023) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <GeographicLib/Rhumb.hpp>
#include <GeographicLib/DST.hpp>

namespace GeographicLib {

  using namespace std;

  Rhumb::Rhumb(real a, real f, bool exact)
    : _aux(a, f)
    , _exact(exact)
    , _a(a)
    , _f(f)
    , _n(_f / (2 - _f))
    , _rm(_aux.RectifyingRadius(_exact))
    , _c2(_aux.AuthalicRadiusSquared(_exact) * Math::degree())
    , _lL(_exact ? 8 : Lmax_)   // 8 is starting size for DFT fit
    , _pP(_lL)
  {
    AreaCoeffs();
  }

  const Rhumb& Rhumb::WGS84() {
    static const Rhumb
      wgs84(Constants::WGS84_a(), Constants::WGS84_f());
    return wgs84;
  }

  void Rhumb::AreaCoeffs() {
    // Set up coefficients for area calculation
    if (_exact) {
      // Compute coefficients by Fourier transform of integrand
      static const real eps = numeric_limits<real>::epsilon()/2;
      qIntegrand f(_aux);
      int L = 4;
      vector<real> c(L);
      DST fft(L); fft.transform(f, c.data()); L *= 2;
      // For |n| <= 0.99, actual max for doubles is 2163.  This scales as
      // Math::digits() and for long doubles (GEOGRAPHICLIB_PRECISION = 3,
      // digits = 64), this becomes 2163 * 64 / 53 = 2612.  Round this up to
      // 2^12 = 4096 and scale this by Math::digits()/64 if digits() > 64.
      //
      // 64 = digits for long double, 6 = 12 - log2(64)
      int Lmax = 1<<(int(ceil(log2(max(Math::digits(), 64)))) + 6);
      for (_lL = 0; L <= Lmax && _lL == 0; L *=2) {
        fft.reset(L/2); c.resize(L); fft.refine(f, c.data());
        _pP.resize(L);
        for (int l = 0, k = -1; l < L; ++l) {
          // Compute Fourier coefficients of integral
          _pP[l] = (c[l] + (l+1 < L ? c[l+1] : 0)) / (-4 * (l+1));
          if (fabs(_pP[l]) <= eps) {
            if (k < 0) k = l;   // mark as first small value
          } else
            k = -1;             // run interrupted
          if (k >= 0 && l - k + 1 >= (l + 1 + 7) / 8) {
            // run of small values of at least l/8?
            _lL = l + 1; _pP.resize(_lL); break;
          }
        }
        // loop exits if _lL > 0
      }
      if (_lL == 0)          // Hasn't converged -- just use the values we have
        _lL = int(_pP.size());
    } else {
      // Use series expansions in n for Fourier coeffients of the integral
      // See "Series expansions for computing rhumb areas"
      // https://doi.org/10.5281/zenodo.7685484
#if GEOGRAPHICLIB_RHUMBAREA_ORDER == 4
      static const real coeffs[] = {
        // Coefficients in matrix Q
        596/real(2025), -398/real(945), 22/real(45), -1/real(3),
        1543/real(4725), -118/real(315), 1/real(5),
        152/real(945), -17/real(315),
        5/real(252),
      };
#elif GEOGRAPHICLIB_RHUMBAREA_ORDER == 5
      static const real coeffs[] = {
        // Coefficients in matrix Q
        -102614/real(467775), 596/real(2025), -398/real(945), 22/real(45),
        -1/real(3),
        -24562/real(155925), 1543/real(4725), -118/real(315), 1/real(5),
        -38068/real(155925), 152/real(945), -17/real(315),
        -752/real(10395), 5/real(252),
        -101/real(17325),
      };
#elif GEOGRAPHICLIB_RHUMBAREA_ORDER == 6
      static const real coeffs[] = {
        // Coefficients in matrix Q
        138734126/real(638512875), -102614/real(467775), 596/real(2025),
        -398/real(945), 22/real(45), -1/real(3),
        17749373/real(425675250), -24562/real(155925), 1543/real(4725),
        -118/real(315), 1/real(5),
        1882432/real(8513505), -38068/real(155925), 152/real(945),
        -17/real(315),
        268864/real(2027025), -752/real(10395), 5/real(252),
        62464/real(2027025), -101/real(17325),
        11537/real(4054050),
      };
#elif GEOGRAPHICLIB_RHUMBAREA_ORDER == 7
      static const real coeffs[] = {
        // Coefficients in matrix Q
        -565017322/real(1915538625), 138734126/real(638512875),
        -102614/real(467775), 596/real(2025), -398/real(945), 22/real(45),
        -1/real(3),
        -1969276/real(58046625), 17749373/real(425675250), -24562/real(155925),
        1543/real(4725), -118/real(315), 1/real(5),
        -58573784/real(638512875), 1882432/real(8513505), -38068/real(155925),
        152/real(945), -17/real(315),
        -6975184/real(42567525), 268864/real(2027025), -752/real(10395),
        5/real(252),
        -112832/real(1447875), 62464/real(2027025), -101/real(17325),
        -4096/real(289575), 11537/real(4054050),
        -311/real(525525),
      };
#elif GEOGRAPHICLIB_RHUMBAREA_ORDER == 8
      static const real coeffs[] = {
        // Coefficients in matrix Q
        188270561816LL/real(488462349375LL), -565017322/real(1915538625),
        138734126/real(638512875), -102614/real(467775), 596/real(2025),
        -398/real(945), 22/real(45), -1/real(3),
        2332829602LL/real(23260111875LL), -1969276/real(58046625),
        17749373/real(425675250), -24562/real(155925), 1543/real(4725),
        -118/real(315), 1/real(5),
        -41570288/real(930404475), -58573784/real(638512875),
        1882432/real(8513505), -38068/real(155925), 152/real(945),
        -17/real(315),
        1538774036/real(10854718875LL), -6975184/real(42567525),
        268864/real(2027025), -752/real(10395), 5/real(252),
        436821248/real(3618239625LL), -112832/real(1447875),
        62464/real(2027025), -101/real(17325),
        3059776/real(80405325), -4096/real(289575), 11537/real(4054050),
        4193792/real(723647925), -311/real(525525),
        1097653/real(1929727800),
      };
#else
#error "Bad value for GEOGRAPHICLIB_RHUMBAREA_ORDER"
#endif

      static_assert(sizeof(coeffs) / sizeof(real) ==
                    (Lmax_ * (Lmax_ + 1))/2,
                    "Coefficient array size mismatch for Rhumb");
      real d = 1;
      int o = 0;
      for (int l = 0; l < Lmax_; ++l) {
        int m = Lmax_ - l - 1;
        d *= _n;
        _pP[l] = d * Math::polyval(m, coeffs + o, _n);
        o += m + 1;
      }
    }
    // Post condition: o == sizeof(coeffs) / sizeof(real)
  }

  Rhumb::qIntegrand::qIntegrand(const AuxLatitude& aux)
    : _aux(aux) {}

  Math::real Rhumb::qIntegrand::operator()(real beta) const {
    // pbeta(beta) = integrate(q(beta), beta)
    //   q(beta) = (1-f) * (sin(xi) - sin(chi)) / cos(phi)
    //           = (1-f) * (cos(chi) - cos(xi)) / cos(phi) *
    //   (cos(xi) + cos(chi)) / (sin(xi) + sin(chi))
    // Fit q(beta)/cos(beta) with Fourier transform
    //   q(beta)/cos(beta) = sum(c[k] * sin((2*k+1)*beta), k, 0, K-1)
    // then the integral is
    //   pbeta = sum(d[k] * cos((2*k+2)*beta), k, 0, K-1)
    // where
    //   d[k] = -1/(4*(k+1)) * (c[k] + c[k+1]) for k in 0..K-2
    //   d[K-1] = -1/(4*K) * c[K-1]
    AuxAngle betaa(AuxAngle::radians(beta)),
      phia(_aux.Convert(AuxLatitude::BETA, AuxLatitude::PHI,
                        betaa, true).normalized()),
      chia(_aux.Convert(AuxLatitude::PHI , AuxLatitude::CHI,
                        phia , true).normalized()),
      xia (_aux.Convert(AuxLatitude::PHI , AuxLatitude::XI ,
                        phia , true).normalized());
    real schi = chia.y(), cchi = chia.x(), sxi = xia.y(), cxi = xia.x(),
      cphi = phia.x(), cbeta = betaa.x();
    return (1 - _aux.Flattening()) *
      ( fabs(schi) < fabs(cchi) ? sxi - schi :
        (cchi - cxi) *  (cxi + cchi) / (sxi + schi) ) / (cphi * cbeta);
    // Value for beta = pi/2.  This isn't needed because beta is given in
    // radians and cos(pi/2) is never exactly 0.  See formulas in the auxlat
    // paper for tan(chi)/tan(phi) and tan(xi)/tan(phi) in the limit phi ->
    // pi/2.
    //
    // n = 1/2;
    // e = 2*sqrt(n) / (1 + n);
    // e1 = 2*sqrt(n) / (1 - n);
    // at = f == 0 ? 1 : (f < 0 ? atan(|e|)/|e| : asinh(e1)/e);
    // q1 = at + 1 / (1 - e^2);
    // s1 = sinh(e^2 * at);
    // h1 = f < 0 ? hypot(1, s1) -  s1 : 1/(hypot(1, s1) + s1);
    // v = (1 - e^2) / (2 * h1^2) - 1 / ((1 - e^2) * q1);
  }

  void Rhumb::GenInverse(real lat1, real lon1, real lat2, real lon2,
                         unsigned outmask,
                         real& s12, real& azi12, real& S12) const {
    using std::isinf;           // Needed for Centos 7, ubuntu 14
    AuxAngle phi1(AuxAngle::degrees(lat1)), phi2(AuxAngle::degrees(lat2)),
      chi1(_aux.Convert(_aux.PHI, _aux.CHI, phi1, _exact)),
      chi2(_aux.Convert(_aux.PHI, _aux.CHI, phi2, _exact));
    real
      lon12 = Math::AngDiff(lon1, lon2),
      lam12 = lon12 * Math::degree<real>(),
      psi1 = chi1.lam(),
      psi2 = chi2.lam(),
      psi12 = psi2 - psi1;
    if (outmask & AZIMUTH)
      azi12 = Math::atan2d(lam12, psi12);
    if (outmask & DISTANCE) {
      if (isinf(psi1) || isinf(psi2)) {
        s12 = fabs(_aux.Convert(AuxLatitude::PHI, AuxLatitude::MU,
                                phi2, _exact).radians() -
                   _aux.Convert(AuxLatitude::PHI, AuxLatitude::MU,
                                phi1, _exact).radians()) * _rm;
      } else {
      real h = hypot(lam12, psi12);
      // dmu/dpsi = dmu/dchi / dpsi/dchi
      real dmudpsi = _exact ?
        _aux.DRectifying(phi1, phi2) / _aux.DIsometric(phi1, phi2) :
        _aux.DConvert(AuxLatitude::CHI, AuxLatitude::MU, chi1, chi2)
        / DAuxLatitude::Dlam(chi1.tan(), chi2.tan());
      s12 = h * dmudpsi * _rm;
      }
    }
    if (outmask & AREA)
      S12 = _c2 * lon12 * MeanSinXi(chi1, chi2);
  }

  RhumbLine Rhumb::Line(real lat1, real lon1, real azi12) const
  { return RhumbLine(*this, lat1, lon1, azi12); }

  void Rhumb::GenDirect(real lat1, real lon1, real azi12, real s12,
                        unsigned outmask,
                        real& lat2, real& lon2, real& S12) const
  { Line(lat1, lon1, azi12).GenPosition(s12, outmask, lat2, lon2, S12); }

  Math::real Rhumb::MeanSinXi(const AuxAngle& chix, const AuxAngle& chiy)
    const {
    AuxAngle
      phix (_aux.Convert(_aux.CHI, _aux.PHI , chix, _exact)),
      phiy (_aux.Convert(_aux.CHI, _aux.PHI , chiy, _exact)),
      betax(_aux.Convert(_aux.PHI, _aux.BETA, phix, _exact).normalized()),
      betay(_aux.Convert(_aux.PHI, _aux.BETA, phiy, _exact).normalized());
    real DpbetaDbeta =
      DAuxLatitude::DClenshaw(false,
                        betay.radians() - betax.radians(),
                        betax.y(), betax.x(), betay.y(), betay.x(),
                        _pP.data(), _lL),
      tx = chix.tan(), ty = chiy.tan(),
      DbetaDpsi = _exact ?
      _aux.DParametric(phix, phiy) / _aux.DIsometric(phix, phiy) :
      _aux.DConvert(AuxLatitude::CHI, AuxLatitude::BETA, chix, chiy) /
      DAuxLatitude::Dlam(tx, ty);
    return DAuxLatitude::Dp0Dpsi(tx, ty) + DpbetaDbeta * DbetaDpsi;
  }

  RhumbLine::RhumbLine(const Rhumb& rh, real lat1, real lon1, real azi12)
    : _rh(rh)
    , _lat1(Math::LatFix(lat1))
    , _lon1(lon1)
    , _azi12(Math::AngNormalize(azi12))
  {
    Math::sincosd(_azi12, _salp, _calp);
    _phi1 = AuxAngle::degrees(lat1);
    _mu1 = _rh._aux.Convert(AuxLatitude::PHI, AuxLatitude::MU,
                            _phi1, _rh._exact).degrees();
    _chi1 = _rh._aux.Convert(AuxLatitude::PHI, AuxLatitude::CHI,
                             _phi1, _rh._exact);
    _psi1 = _chi1.lam();
  }

  void RhumbLine::GenPosition(real s12, unsigned outmask,
                              real& lat2, real& lon2, real& S12) const {
    real
      r12 = s12 / (_rh._rm * Math::degree()), // scaled distance in degrees
      mu12 = r12 * _calp,
      mu2 = _mu1 + mu12;
    real lat2x, lon2x;
    if (fabs(mu2) <= Math::qd) {
      AuxAngle mu2a(AuxAngle::degrees(mu2)),
        phi2(_rh._aux.Convert(AuxLatitude::MU, AuxLatitude::PHI,
                              mu2a, _rh._exact)),
        chi2(_rh._aux.Convert(AuxLatitude::PHI, AuxLatitude::CHI,
                              phi2, _rh._exact));
      lat2x = phi2.degrees();
      real dmudpsi = _rh._exact ?
        _rh._aux.DRectifying(_phi1, phi2) / _rh._aux.DIsometric(_phi1, phi2) :
        _rh._aux.DConvert(AuxLatitude::CHI, AuxLatitude::MU, _chi1, chi2)
        / DAuxLatitude::Dlam(_chi1.tan(), chi2.tan());
      lon2x = r12 * _salp / dmudpsi;
      if (outmask & AREA)
        S12 = _rh._c2 * lon2x * _rh.MeanSinXi(_chi1, chi2);
      lon2x = outmask & LONG_UNROLL ? _lon1 + lon2x :
        Math::AngNormalize(Math::AngNormalize(_lon1) + lon2x);
    } else {
      // Reduce to the interval [-180, 180)
      mu2 = Math::AngNormalize(mu2);
      // Deal with points on the anti-meridian
      if (fabs(mu2) > Math::qd) mu2 = Math::AngNormalize(Math::hd - mu2);
      lat2x = _rh._aux.Convert(AuxLatitude::MU, AuxLatitude::PHI,
                               AuxAngle::degrees(mu2), _rh._exact).degrees();
      lon2x = Math::NaN();
      if (outmask & AREA)
        S12 = Math::NaN();
    }
    if (outmask & LATITUDE) lat2 = lat2x;
    if (outmask & LONGITUDE) lon2 = lon2x;
  }

} // namespace GeographicLib
