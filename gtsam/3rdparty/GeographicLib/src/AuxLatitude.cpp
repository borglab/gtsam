/**
 * \file AuxLatitude.cpp
 * \brief Implementation for the GeographicLib::AuxLatitude class.
 *
 * This file is an implementation of the methods described in
 * - C. F. F. Karney,
 *   <a href="https://doi.org/10.1080/00396265.2023.2217604">
 *   On auxiliary latitudes,</a>
 *   Survey Review 56(395), 165--180 (2024);
 *   preprint
 *   <a href="https://arxiv.org/abs/2212.05818">arXiv:2212.05818</a>.
 * .
 * Copyright (c) Charles Karney (2022-2023) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <GeographicLib/AuxLatitude.hpp>
#include <GeographicLib/EllipticFunction.hpp>

namespace GeographicLib {

  using namespace std;

  AuxLatitude::AuxLatitude(real a, real f)
    : tol_( sqrt(numeric_limits<real>::epsilon()) )
    , bmin_( log2(numeric_limits<real>::min()) )
    , bmax_( log2(numeric_limits<real>::max()) )
    , _a(a)
    , _b(_a * (1 - f))
    , _f( f )
    , _fm1( 1 - _f )
    , _e2( _f * (2 - _f) )
    , _e2m1( _fm1 * _fm1 )
    , _e12( _e2/(1 - _e2) )
    , _e12p1( 1 / _e2m1 )
    , _n( _f/(2 - _f) )
    , _e( sqrt(fabs(_e2)) )
    , _e1( sqrt(fabs(_e12)) )
    , _n2( _n * _n )
    , _q( _e12p1 + (_f == 0 ? 1 : (_f > 0 ? asinh(_e1) : atan(_e)) / _e) )
  {
    if (!(isfinite(_a) && _a > 0))
      throw GeographicErr("Equatorial radius is not positive");
    if (!(isfinite(_b) && _b > 0))
      throw GeographicErr("Polar semi-axis is not positive");
    fill(_c, _c + Lmax * AUXNUMBER * AUXNUMBER,
         numeric_limits<real>::quiet_NaN());
  }

  /// \cond SKIP
  AuxLatitude::AuxLatitude(const pair<real, real>& axes)
    : tol_( sqrt(numeric_limits<real>::epsilon()) )
    , bmin_( log2(numeric_limits<real>::min()) )
    , bmax_( log2(numeric_limits<real>::max()) )
    , _a(axes.first)
    , _b(axes.second)
    , _f( (_a - _b) / _a )
    , _fm1( _b / _a )
    , _e2( ((_a - _b) * (_a + _b)) / (_a * _a) )
    , _e2m1( (_b * _b) / (_a * _a) )
    , _e12( ((_a - _b) * (_a + _b)) / (_b * _b) )
    , _e12p1( (_a * _a) / (_b * _b) )
    , _n( (_a - _b) / (_a + _b) )
    , _e( sqrt(fabs(_a - _b) * (_a + _b)) / _a )
    , _e1( sqrt(fabs(_a - _b) * (_a + _b)) / _b )
    , _n2( _n * _n )
    , _q( _e12p1 + (_f == 0 ? 1 : (_f > 0 ? asinh(_e1) : atan(_e)) / _e) )
  {
    if (!(isfinite(_a) && _a > 0))
      throw GeographicErr("Equatorial radius is not positive");
    if (!(isfinite(_b) && _b > 0))
      throw GeographicErr("Polar semi-axis is not positive");
    fill(_c, _c + Lmax * AUXNUMBER * AUXNUMBER,
         numeric_limits<real>::quiet_NaN());
  }
  /// \endcond

  const AuxLatitude& AuxLatitude::WGS84() {
    static const AuxLatitude wgs84(Constants::WGS84_a(), Constants::WGS84_f());
    return wgs84;
  }

  AuxAngle AuxLatitude::Parametric(const AuxAngle& phi, real* diff) const {
    if (diff) *diff = _fm1;
    return AuxAngle(phi.y() * _fm1, phi.x());
  }

  AuxAngle AuxLatitude::Geocentric(const AuxAngle& phi, real* diff) const {
    if (diff) *diff = _e2m1;
    return AuxAngle(phi.y() * _e2m1, phi.x());
  }

  AuxAngle AuxLatitude::Rectifying(const AuxAngle& phi, real* diff) const {
    using std::isinf;           // Needed for Centos 7, ubuntu 14
    AuxAngle beta(Parametric(phi).normalized());
    real sbeta = fabs(beta.y()), cbeta = fabs(beta.x());
    real a = 1, b = _fm1, ka = _e2, kb = -_e12, ka1 = _e2m1, kb1 = _e12p1,
      smu, cmu, mr;
    if (_f < 0) {
      swap(a, b); swap(ka, kb); swap(ka1, kb1); swap(sbeta, cbeta);
    }
    // now a,b = larger/smaller semiaxis
    // beta now measured from larger semiaxis
    // kb,ka = modulus-squared for distance from beta = 0,pi/2
    // NB kb <= 0; 0 <= ka <= 1
    // sa = b*E(beta,sqrt(kb)), sb = a*E(beta',sqrt(ka))
    //    1 - ka * (1 - sb2) = 1 -ka + ka*sb2
    real
      sb2 = sbeta * sbeta,
      cb2 = cbeta * cbeta,
      db2 = 1 - kb * sb2,
      da2 = ka1 + ka * sb2,
      // DLMF Eq. 19.25.9
      sa = b * sbeta * ( EllipticFunction::RF(cb2, db2, 1) -
                         kb * sb2 * EllipticFunction::RD(cb2, db2, 1)/3 ),
      // DLMF Eq. 19.25.10 with complementary angles
      sb = a * cbeta * ( ka1 * EllipticFunction::RF(sb2, da2, 1)
                         + ka * ka1 * cb2 * EllipticFunction::RD(sb2, 1, da2)/3
                         + ka * sbeta / sqrt(da2) );
    // sa + sb  = 2*EllipticFunction::RG(a*a, b*b) = a*E(e) = b*E(i*e')
    // mr = a*E(e)*(2/pi) = b*E(i*e')*(2/pi)
    mr = (2 * (sa + sb)) / Math::pi();
    smu = sin(sa / mr);
    cmu = sin(sb / mr);
    if (_f < 0) { swap(smu, cmu); swap(a, b); }
    // mu is normalized
    AuxAngle mu(AuxAngle(smu, cmu).copyquadrant(phi));
    if (diff) {
      real cphi = phi.normalized().x(), tphi = phi.tan();
      if (!isinf(tphi)) {
        cmu = mu.x(); cbeta = beta.x();
        *diff = _fm1 * b/mr * Math::sq(cbeta / cmu) * (cbeta / cphi);
      } else
        *diff = _fm1 * mr/a;
    }
    return mu;
  }

  AuxAngle AuxLatitude::Conformal(const AuxAngle& phi, real* diff) const {
    using std::isinf;           // Needed for Centos 7, ubuntu 14
    real tphi = fabs(phi.tan()), tchi = tphi;
    if ( !( !isfinite(tphi) || tphi == 0 || _f == 0 ) ) {
      real scphi = sc(tphi),
        sig = sinh(_e2 * atanhee(tphi) ),
        scsig = sc(sig);
      if (_f <= 0) {
        tchi = tphi * scsig - sig * scphi;
      } else {
        // The general expression for tchi is
        //   tphi * scsig - sig * scphi
        // This involves cancellation if f > 0, so change to
        //   (tphi - sig) * (tphi + sig) / (tphi * scsig + sig * scphi)
        // To control overflow, write as (sigtphi = sig / tphi)
        //   (tphi - sig) * (1 + sigtphi) / (scsig + sigtphi * scphi)
        real sigtphi = sig / tphi, tphimsig;
        if (sig < tphi / 2)
          tphimsig = tphi - sig;
        else {
          // Still have possibly dangerous cancellation in tphi - sig.
          //
          // Write tphi - sig = (1 - e) * Dg(1, e)
          //   Dg(x, y) = (g(x) - g(y)) / (x - y)
          //   g(x) = sinh(x * atanh(sphi * x))
          // Note sinh(atanh(sphi)) = tphi
          // Turn the crank on divided differences, substitute
          //   sphi = tphi/sc(tphi)
          //   atanh(x) = asinh(x/sqrt(1-x^2))
          real em1 = _e2m1 / (1 + _e),              // 1 - e
            atanhs = asinh(tphi),                // atanh(sphi)
            scbeta = sc(_fm1 * tphi),            // sec(beta)
            scphibeta = sc(tphi) / scbeta,       // sec(phi)/sec(beta)
            atanhes = asinh(_e * tphi / scbeta), // atanh(e * sphi)
            t1 = (atanhs - _e * atanhes)/2,
            t2 = asinh(em1 * (tphi * scphibeta)) / em1,
            Dg = cosh((atanhs + _e * atanhes)/2) * (sinh(t1) / t1)
            * ((atanhs + atanhes)/2 + (1 + _e)/2 * t2);
          tphimsig = em1 * Dg;  // tphi - sig
        }
        tchi = tphimsig * (1 + sigtphi) / (scsig + sigtphi * scphi);
      }
    }
    AuxAngle chi(AuxAngle(tchi).copyquadrant(phi));
    if (diff) {
      if (!isinf(tphi)) {
        real cchi = chi.normalized().x(),
          cphi = phi.normalized().x(),
          cbeta = Parametric(phi).normalized().x();
        *diff = _e2m1 * (cbeta / cchi) * (cbeta / cphi);
      } else {
        real ss = _f > 0 ? sinh(_e * asinh(_e1)) : sinh(-_e * atan(_e));
        *diff = _f > 0 ? 1/( sc(ss) + ss ) : sc(ss) - ss;
      }
    }
    return chi;
  }

  AuxAngle AuxLatitude::Authalic(const AuxAngle& phi, real* diff) const {
    using std::isnan;           // Needed for Centos 7, ubuntu 14
    real tphi = fabs(phi.tan());
    AuxAngle xi(phi), phin(phi.normalized());
    if ( !( !isfinite(tphi) || tphi == 0 || _f == 0 ) ) {
      real qv = q(tphi),
        Dqp = Dq(tphi),
        Dqm = (_q + qv) / (1 + fabs(phin.y())); // Dq(-tphi)
      xi = AuxAngle( copysign(qv, phi.y()), phin.x() * sqrt(Dqp * Dqm) );
    }
    if (diff) {
      if (!isnan(tphi)) {
        real cbeta = Parametric(phi).normalized().x(),
          cxi = xi.normalized().x();
        *diff =
          (2/_q) * Math::sq(cbeta / cxi) * (cbeta / cxi) * (cbeta / phin.x());
      } else
        *diff = _e2m1 * sqrt(_q/2);
    }
    return xi;
  }

  AuxAngle AuxLatitude::ToAuxiliary(int auxout, const AuxAngle& phi,
                                          real* diff) const {
    switch (auxout) {
    case GEOGRAPHIC: if (diff) *diff = 1; return phi; break;
    case PARAMETRIC: return Parametric(phi, diff); break;
    case GEOCENTRIC: return Geocentric(phi, diff); break;
    case RECTIFYING: return Rectifying(phi, diff); break;
    case CONFORMAL : return Conformal (phi, diff); break;
    case AUTHALIC  : return Authalic  (phi, diff); break;
    default:
      if (diff) *diff = numeric_limits<real>::quiet_NaN();
      return AuxAngle::NaN();
      break;
    }
  }

  AuxAngle AuxLatitude::FromAuxiliary(int auxin, const AuxAngle& zeta,
                                            int* niter) const {
    int n = 0; if (niter) *niter = n;
    real tphi = _fm1;
    switch (auxin) {
    case GEOGRAPHIC: return zeta; break;
      // case PARAMETRIC:                   break;
    case PARAMETRIC: return AuxAngle(zeta.y() / _fm1, zeta.x()); break;
      // case GEOCENTRIC: tphi *= _fm1  ; break;
    case GEOCENTRIC: return AuxAngle(zeta.y() / _e2m1, zeta.x()); break;
    case RECTIFYING: tphi *= sqrt(_fm1); break;
    case CONFORMAL : tphi *= _fm1  ; break;
    case AUTHALIC  : tphi *= cbrt(_fm1); break;
    default: return AuxAngle::NaN(); break;
    }

    // Drop through to solution by Newton's method
    real tzeta = fabs(zeta.tan()), ltzeta = log2(tzeta);
    if (!isfinite(ltzeta)) return zeta;
    tphi = tzeta / tphi;
    real ltphi = log2(tphi),
      bmin = fmin(ltphi, bmin_), bmax = fmax(ltphi, bmax_);
    for (int sign = 0, osign = 0, ntrip = 0; n < numit_;) {
      ++n;
      real diff;
      AuxAngle zeta1(ToAuxiliary(auxin, AuxAngle(tphi), &diff));
      real tzeta1 = zeta1.tan(), ltzeta1 = log2(tzeta1);
      // Convert derivative from dtan(zeta)/dtan(phi) to
      // dlog(tan(zeta))/dlog(tan(phi))
      diff *= tphi/tzeta1;
      osign = sign;
      if (tzeta1 == tzeta)
        break;
      else if (tzeta1 > tzeta) {
        sign = 1;
        bmax = ltphi;
      } else {
        sign = -1;
        bmin = ltphi;
      }
      real dltphi = -(ltzeta1 - ltzeta) / diff;
      ltphi += dltphi;
      tphi = exp2(ltphi);
      if (!(fabs(dltphi) >= tol_)) {
        ++n;
        // Final Newton iteration without the logs
        zeta1 = ToAuxiliary(auxin, AuxAngle(tphi), &diff);
        tphi -= (zeta1.tan() - tzeta) / diff;
        break;
      }
      if ((sign * osign < 0 && n - ntrip > 2) ||
          ltphi >= bmax || ltphi <= bmin) {
        sign = 0; ntrip = n;
        ltphi = (bmin + bmax) / 2;
        tphi = exp2(ltphi);
      }
    }
    if (niter) *niter = n;
    return AuxAngle(tphi).copyquadrant(zeta);
  }

  AuxAngle AuxLatitude::Convert(int auxin, int auxout, const AuxAngle& zeta,
                                bool exact) const {
    using std::isnan;           // Needed for Centos 7, ubuntu 14
    int k = ind(auxout, auxin);
    if (k < 0) return AuxAngle::NaN();
    if (auxin == auxout) return zeta;
    if (exact) {
      if (auxin < 3 && auxout < 3)
        // Need extra real because, since C++11, pow(float, int) returns double
        return AuxAngle(zeta.y() * real(pow(_fm1, auxout - auxin)), zeta.x());
      else
        return ToAuxiliary(auxout, FromAuxiliary(auxin, zeta));
    } else {
      if ( isnan(_c[Lmax * (k + 1) - 1]) ) fillcoeff(auxin, auxout, k);
      AuxAngle zetan(zeta.normalized());
      real d = Clenshaw(true, zetan.y(), zetan.x(), _c + Lmax * k, Lmax);
      zetan += AuxAngle::radians(d);
      return zetan;
    }
  }

  Math::real AuxLatitude::Convert(int auxin, int auxout, real zeta,
                                  bool exact) const {
    AuxAngle zetaa(AuxAngle::degrees(zeta));
    real m = round((zeta - zetaa.degrees()) / Math::td);
    return Math::td * m + Convert(auxin, auxout, zetaa, exact).degrees();
  }

  Math::real AuxLatitude::RectifyingRadius(bool exact) const {
    if (exact) {
      return EllipticFunction::RG(Math::sq(_a), Math::sq(_b)) * 4 / Math::pi();
    } else {
      // Maxima code for these coefficients:
      // df[i]:=if i<0 then df[i+2]/(i+2) else i!!$
      // R(Lmax):=sum((df[2*j-3]/df[2*j])^2*n^(2*j),j,0,floor(Lmax/2))$
      // cf(Lmax):=block([t:R(Lmax)],
      //  t:makelist(coeff(t,n,2*(floor(Lmax/2)-j)),j,0,floor(Lmax/2)),
      //  map(lambda([x],num(x)/
      //         (if denom(x) = 1 then 1 else real(denom(x)))),t))$
#if GEOGRAPHICLIB_AUXLATITUDE_ORDER == 4
      static const real coeff[] = {1/real(64), 1/real(4), 1};
#elif GEOGRAPHICLIB_AUXLATITUDE_ORDER == 6
      static const real coeff[] = {1/real(256), 1/real(64), 1/real(4), 1};
#elif GEOGRAPHICLIB_AUXLATITUDE_ORDER == 8
      static const real coeff[] = {
        25/real(16384), 1/real(256), 1/real(64), 1/real(4), 1
      };
#else
#error "Unsupported value for GEOGRAPHICLIB_AUXLATITUDE_ORDER"
#endif
      int m = Lmax/2;
      return (_a + _b) / 2 * Math::polyval(m, coeff, _n2);
    }
  }

  Math::real AuxLatitude::AuthalicRadiusSquared(bool exact) const {
    if (exact) {
      return Math::sq(_b) * _q / 2;
    } else {
      // Using a * (a + b) / 2 as the multiplying factor leads to a rapidly
      // converging series in n.  Of course, using this series isn't really
      // necessary, since the exact expression is simple to evaluate.  However,
      // we do it for consistency with RectifyingRadius; and, presumably, the
      // roundoff error is smaller compared to that for the exact expression.
      //
      // Maxima code for these coefficients:
      // c2:subst(e=2*sqrt(n)/(1+n),
      //          (atanh(e)/e * (1-n)^2 + (1+n)^2)/(2*(1+n)))$
      // cf(Lmax):=block([t:expand(ratdisrep(taylor(c2,n,0,Lmax)))],
      //  t:makelist(coeff(t,n,Lmax-j),j,0,Lmax),
      //  map(lambda([x],num(x)/
      //         (if denom(x) = 1 then 1 else real(denom(x)))),t))$
      // N.B. Coeff of n^j = 1                     for j = 0
      //                     -1/3                  for j = 1
      //                     4*(2*j-5)!!/(2*j+1)!! for j > 1
#if GEOGRAPHICLIB_AUXLATITUDE_ORDER == 4
      static const real coeff[] = {
        4/real(315), 4/real(105), 4/real(15), -1/real(3), 1
      };
#elif GEOGRAPHICLIB_AUXLATITUDE_ORDER == 6
      static const real coeff[] = {
        4/real(1287), 4/real(693), 4/real(315), 4/real(105), 4/real(15),
        -1/real(3), 1
      };
#elif GEOGRAPHICLIB_AUXLATITUDE_ORDER == 8
      static const real coeff[] = {
        4/real(3315), 4/real(2145), 4/real(1287), 4/real(693), 4/real(315),
        4/real(105), 4/real(15), -1/real(3), 1
      };
#else
#error "Unsupported value for GEOGRAPHICLIB_AUXLATITUDE_ORDER"
#endif
      int m = Lmax;
      return _a * (_a + _b) / 2 *  Math::polyval(m, coeff, _n);
    }
  }

  /// \cond SKIP
  Math::real AuxLatitude::atanhee(real tphi) const {
    real s = _f <= 0 ? sn(tphi) : sn(_fm1 * tphi);
    return _f == 0 ? s :
      // atanh(e * sphi) = asinh(e' * sbeta)
      (_f < 0 ? atan( _e * s ) : asinh( _e1 * s )) / _e;
  }
  /// \endcond

  Math::real AuxLatitude::q(real tphi) const {
    real scbeta = sc(_fm1 * tphi);
    return atanhee(tphi) + (tphi / scbeta) * (sc(tphi) / scbeta);
  }

  Math::real AuxLatitude::Dq(real tphi) const {
    real scphi = sc(tphi), sphi = sn(tphi),
      // d = (1 - sphi) can underflow to zero for large tphi
      d = tphi > 0 ? 1 / (scphi * scphi * (1 + sphi)) : 1 - sphi;
    if (tphi <= 0)
      // This branch is not reached; this case is open-coded in Authalic.
      return (_q - q(tphi)) / d;
    else if (d == 0)
      return 2 / Math::sq(_e2m1);
    else {
      // General expression for Dq(1, sphi) is
      // atanh(e * d / (1 - e2 * sphi)) / (e * d) +
      //   (1 + e2 * sphi) / ((1 - e2 * sphi * sphi) * e2m1);
      // atanh( e * d / (1 - e2 * sphi))
      // = atanh( e * d * scphi/(scphi - e2 * tphi))
      // =
      real scbeta = sc(_fm1 * tphi);
      return (_f == 0 ? 1 :
              (_f > 0 ? asinh(_e1 * d * scphi / scbeta) :
               atan(_e * d / (1 - _e2 * sphi))) / (_e * d) ) +
        (_f  > 0 ?
         ((scphi + _e2 * tphi) / (_e2m1 * scbeta)) * (scphi / scbeta) :
        (1 + _e2 * sphi) / ((1 - _e2 * sphi*sphi) * _e2m1) );
    }
  }

  /// \cond SKIP
  void AuxLatitude::fillcoeff(int auxin, int auxout, int k) const {
#if GEOGRAPHICLIB_AUXLATITUDE_ORDER == 4
    static const real coeffs[] = {
      // C[phi,phi] skipped
      // C[phi,beta]; even coeffs only
      0, 1,
      0, 1/real(2),
      1/real(3),
      1/real(4),
      // C[phi,theta]; even coeffs only
      -2, 2,
      -4, 2,
      8/real(3),
      4,
      // C[phi,mu]; even coeffs only
      -27/real(32), 3/real(2),
      -55/real(32), 21/real(16),
      151/real(96),
      1097/real(512),
      // C[phi,chi]
      116/real(45), -2, -2/real(3), 2,
      -227/real(45), -8/real(5), 7/real(3),
      -136/real(35), 56/real(15),
      4279/real(630),
      // C[phi,xi]
      -2582/real(14175), -16/real(35), 4/real(45), 4/real(3),
      -11966/real(14175), 152/real(945), 46/real(45),
      3802/real(14175), 3044/real(2835),
      6059/real(4725),
      // C[beta,phi]; even coeffs only
      0, -1,
      0, 1/real(2),
      -1/real(3),
      1/real(4),
      // C[beta,beta] skipped
      // C[beta,theta]; even coeffs only
      0, 1,
      0, 1/real(2),
      1/real(3),
      1/real(4),
      // C[beta,mu]; even coeffs only
      -9/real(32), 1/real(2),
      -37/real(96), 5/real(16),
      29/real(96),
      539/real(1536),
      // C[beta,chi]
      38/real(45), -1/real(3), -2/real(3), 1,
      -7/real(9), -14/real(15), 5/real(6),
      -34/real(21), 16/real(15),
      2069/real(1260),
      // C[beta,xi]
      -1082/real(14175), -46/real(315), 4/real(45), 1/real(3),
      -338/real(2025), 68/real(945), 17/real(90),
      1102/real(14175), 461/real(2835),
      3161/real(18900),
      // C[theta,phi]; even coeffs only
      2, -2,
      -4, 2,
      -8/real(3),
      4,
      // C[theta,beta]; even coeffs only
      0, -1,
      0, 1/real(2),
      -1/real(3),
      1/real(4),
      // C[theta,theta] skipped
      // C[theta,mu]; even coeffs only
      -23/real(32), -1/real(2),
      -5/real(96), 5/real(16),
      1/real(32),
      283/real(1536),
      // C[theta,chi]
      4/real(9), -2/real(3), -2/real(3), 0,
      -23/real(45), -4/real(15), 1/real(3),
      -24/real(35), 2/real(5),
      83/real(126),
      // C[theta,xi]
      -2102/real(14175), -158/real(315), 4/real(45), -2/real(3),
      934/real(14175), -16/real(945), 16/real(45),
      922/real(14175), -232/real(2835),
      719/real(4725),
      // C[mu,phi]; even coeffs only
      9/real(16), -3/real(2),
      -15/real(32), 15/real(16),
      -35/real(48),
      315/real(512),
      // C[mu,beta]; even coeffs only
      3/real(16), -1/real(2),
      1/real(32), -1/real(16),
      -1/real(48),
      -5/real(512),
      // C[mu,theta]; even coeffs only
      13/real(16), 1/real(2),
      33/real(32), -1/real(16),
      -5/real(16),
      -261/real(512),
      // C[mu,mu] skipped
      // C[mu,chi]
      41/real(180), 5/real(16), -2/real(3), 1/real(2),
      557/real(1440), -3/real(5), 13/real(48),
      -103/real(140), 61/real(240),
      49561/real(161280),
      // C[mu,xi]
      -1609/real(28350), 121/real(1680), 4/real(45), -1/real(6),
      16463/real(453600), 26/real(945), -29/real(720),
      449/real(28350), -1003/real(45360),
      -40457/real(2419200),
      // C[chi,phi]
      -82/real(45), 4/real(3), 2/real(3), -2,
      -13/real(9), -16/real(15), 5/real(3),
      34/real(21), -26/real(15),
      1237/real(630),
      // C[chi,beta]
      -16/real(45), 0, 2/real(3), -1,
      19/real(45), -2/real(5), 1/real(6),
      16/real(105), -1/real(15),
      17/real(1260),
      // C[chi,theta]
      -2/real(9), 2/real(3), 2/real(3), 0,
      43/real(45), 4/real(15), -1/real(3),
      2/real(105), -2/real(5),
      -55/real(126),
      // C[chi,mu]
      1/real(360), -37/real(96), 2/real(3), -1/real(2),
      437/real(1440), -1/real(15), -1/real(48),
      37/real(840), -17/real(480),
      -4397/real(161280),
      // C[chi,chi] skipped
      // C[chi,xi]
      -2312/real(14175), -88/real(315), 34/real(45), -2/real(3),
      6079/real(14175), -184/real(945), 1/real(45),
      772/real(14175), -106/real(2835),
      -167/real(9450),
      // C[xi,phi]
      538/real(4725), 88/real(315), -4/real(45), -4/real(3),
      -2482/real(14175), 8/real(105), 34/real(45),
      -898/real(14175), -1532/real(2835),
      6007/real(14175),
      // C[xi,beta]
      34/real(675), 32/real(315), -4/real(45), -1/real(3),
      74/real(2025), -4/real(315), -7/real(90),
      2/real(14175), -83/real(2835),
      -797/real(56700),
      // C[xi,theta]
      778/real(4725), 62/real(105), -4/real(45), 2/real(3),
      12338/real(14175), -32/real(315), 4/real(45),
      -1618/real(14175), -524/real(2835),
      -5933/real(14175),
      // C[xi,mu]
      1297/real(18900), -817/real(10080), -4/real(45), 1/real(6),
      -29609/real(453600), -2/real(35), 49/real(720),
      -2917/real(56700), 4463/real(90720),
      331799/real(7257600),
      // C[xi,chi]
      2458/real(4725), 46/real(315), -34/real(45), 2/real(3),
      3413/real(14175), -256/real(315), 19/real(45),
      -15958/real(14175), 248/real(567),
      16049/real(28350),
      // C[xi,xi] skipped
    };
    static const int ptrs[] = {
      0, 0, 6, 12, 18, 28, 38, 44, 44, 50, 56, 66, 76, 82, 88, 88, 94, 104,
      114, 120, 126, 132, 132, 142, 152, 162, 172, 182, 192, 192, 202, 212,
      222, 232, 242, 252, 252,
    };
#elif GEOGRAPHICLIB_AUXLATITUDE_ORDER == 6
    static const real coeffs[] = {
      // C[phi,phi] skipped
      // C[phi,beta]; even coeffs only
      0, 0, 1,
      0, 0, 1/real(2),
      0, 1/real(3),
      0, 1/real(4),
      1/real(5),
      1/real(6),
      // C[phi,theta]; even coeffs only
      2, -2, 2,
      6, -4, 2,
      -8, 8/real(3),
      -16, 4,
      32/real(5),
      32/real(3),
      // C[phi,mu]; even coeffs only
      269/real(512), -27/real(32), 3/real(2),
      6759/real(4096), -55/real(32), 21/real(16),
      -417/real(128), 151/real(96),
      -15543/real(2560), 1097/real(512),
      8011/real(2560),
      293393/real(61440),
      // C[phi,chi]
      -2854/real(675), 26/real(45), 116/real(45), -2, -2/real(3), 2,
      2323/real(945), 2704/real(315), -227/real(45), -8/real(5), 7/real(3),
      73814/real(2835), -1262/real(105), -136/real(35), 56/real(15),
      -399572/real(14175), -332/real(35), 4279/real(630),
      -144838/real(6237), 4174/real(315),
      601676/real(22275),
      // C[phi,xi]
      28112932/real(212837625), 60136/real(467775), -2582/real(14175),
      -16/real(35), 4/real(45), 4/real(3),
      251310128/real(638512875), -21016/real(51975), -11966/real(14175),
      152/real(945), 46/real(45),
      -8797648/real(10945935), -94388/real(66825), 3802/real(14175),
      3044/real(2835),
      -1472637812/real(638512875), 41072/real(93555), 6059/real(4725),
      455935736/real(638512875), 768272/real(467775),
      4210684958LL/real(1915538625),
      // C[beta,phi]; even coeffs only
      0, 0, -1,
      0, 0, 1/real(2),
      0, -1/real(3),
      0, 1/real(4),
      -1/real(5),
      1/real(6),
      // C[beta,beta] skipped
      // C[beta,theta]; even coeffs only
      0, 0, 1,
      0, 0, 1/real(2),
      0, 1/real(3),
      0, 1/real(4),
      1/real(5),
      1/real(6),
      // C[beta,mu]; even coeffs only
      205/real(1536), -9/real(32), 1/real(2),
      1335/real(4096), -37/real(96), 5/real(16),
      -75/real(128), 29/real(96),
      -2391/real(2560), 539/real(1536),
      3467/real(7680),
      38081/real(61440),
      // C[beta,chi]
      -3118/real(4725), -1/real(3), 38/real(45), -1/real(3), -2/real(3), 1,
      -247/real(270), 50/real(21), -7/real(9), -14/real(15), 5/real(6),
      17564/real(2835), -5/real(3), -34/real(21), 16/real(15),
      -49877/real(14175), -28/real(9), 2069/real(1260),
      -28244/real(4455), 883/real(315),
      797222/real(155925),
      // C[beta,xi]
      7947332/real(212837625), 11824/real(467775), -1082/real(14175),
      -46/real(315), 4/real(45), 1/real(3),
      39946703/real(638512875), -16672/real(155925), -338/real(2025),
      68/real(945), 17/real(90),
      -255454/real(1563705), -101069/real(467775), 1102/real(14175),
      461/real(2835),
      -189032762/real(638512875), 1786/real(18711), 3161/real(18900),
      80274086/real(638512875), 88868/real(467775),
      880980241/real(3831077250LL),
      // C[theta,phi]; even coeffs only
      -2, 2, -2,
      6, -4, 2,
      8, -8/real(3),
      -16, 4,
      -32/real(5),
      32/real(3),
      // C[theta,beta]; even coeffs only
      0, 0, -1,
      0, 0, 1/real(2),
      0, -1/real(3),
      0, 1/real(4),
      -1/real(5),
      1/real(6),
      // C[theta,theta] skipped
      // C[theta,mu]; even coeffs only
      499/real(1536), -23/real(32), -1/real(2),
      6565/real(12288), -5/real(96), 5/real(16),
      -77/real(128), 1/real(32),
      -4037/real(7680), 283/real(1536),
      1301/real(7680),
      17089/real(61440),
      // C[theta,chi]
      -3658/real(4725), 2/real(9), 4/real(9), -2/real(3), -2/real(3), 0,
      61/real(135), 68/real(45), -23/real(45), -4/real(15), 1/real(3),
      9446/real(2835), -46/real(35), -24/real(35), 2/real(5),
      -34712/real(14175), -80/real(63), 83/real(126),
      -2362/real(891), 52/real(45),
      335882/real(155925),
      // C[theta,xi]
      216932/real(2627625), 109042/real(467775), -2102/real(14175),
      -158/real(315), 4/real(45), -2/real(3),
      117952358/real(638512875), -7256/real(155925), 934/real(14175),
      -16/real(945), 16/real(45),
      -7391576/real(54729675), -25286/real(66825), 922/real(14175),
      -232/real(2835),
      -67048172/real(638512875), 268/real(18711), 719/real(4725),
      46774256/real(638512875), 14354/real(467775),
      253129538/real(1915538625),
      // C[mu,phi]; even coeffs only
      -3/real(32), 9/real(16), -3/real(2),
      135/real(2048), -15/real(32), 15/real(16),
      105/real(256), -35/real(48),
      -189/real(512), 315/real(512),
      -693/real(1280),
      1001/real(2048),
      // C[mu,beta]; even coeffs only
      -1/real(32), 3/real(16), -1/real(2),
      -9/real(2048), 1/real(32), -1/real(16),
      3/real(256), -1/real(48),
      3/real(512), -5/real(512),
      -7/real(1280),
      -7/real(2048),
      // C[mu,theta]; even coeffs only
      -15/real(32), 13/real(16), 1/real(2),
      -1673/real(2048), 33/real(32), -1/real(16),
      349/real(256), -5/real(16),
      963/real(512), -261/real(512),
      -921/real(1280),
      -6037/real(6144),
      // C[mu,mu] skipped
      // C[mu,chi]
      7891/real(37800), -127/real(288), 41/real(180), 5/real(16), -2/real(3),
      1/real(2),
      -1983433/real(1935360), 281/real(630), 557/real(1440), -3/real(5),
      13/real(48),
      167603/real(181440), 15061/real(26880), -103/real(140), 61/real(240),
      6601661/real(7257600), -179/real(168), 49561/real(161280),
      -3418889/real(1995840), 34729/real(80640),
      212378941/real(319334400),
      // C[mu,xi]
      12674323/real(851350500), -384229/real(14968800), -1609/real(28350),
      121/real(1680), 4/real(45), -1/real(6),
      -31621753811LL/real(1307674368000LL), -431/real(17325),
      16463/real(453600), 26/real(945), -29/real(720),
      -32844781/real(1751349600), 3746047/real(119750400), 449/real(28350),
      -1003/real(45360),
      10650637121LL/real(326918592000LL), 629/real(53460),
      -40457/real(2419200),
      205072597/real(20432412000LL), -1800439/real(119750400),
      -59109051671LL/real(3923023104000LL),
      // C[chi,phi]
      4642/real(4725), 32/real(45), -82/real(45), 4/real(3), 2/real(3), -2,
      -1522/real(945), 904/real(315), -13/real(9), -16/real(15), 5/real(3),
      -12686/real(2835), 8/real(5), 34/real(21), -26/real(15),
      -24832/real(14175), -12/real(5), 1237/real(630),
      109598/real(31185), -734/real(315),
      444337/real(155925),
      // C[chi,beta]
      -998/real(4725), 2/real(5), -16/real(45), 0, 2/real(3), -1,
      -2/real(27), -22/real(105), 19/real(45), -2/real(5), 1/real(6),
      116/real(567), -22/real(105), 16/real(105), -1/real(15),
      2123/real(14175), -8/real(105), 17/real(1260),
      128/real(4455), -1/real(105),
      149/real(311850),
      // C[chi,theta]
      1042/real(4725), -14/real(45), -2/real(9), 2/real(3), 2/real(3), 0,
      -712/real(945), -4/real(45), 43/real(45), 4/real(15), -1/real(3),
      274/real(2835), 124/real(105), 2/real(105), -2/real(5),
      21068/real(14175), -16/real(105), -55/real(126),
      -9202/real(31185), -22/real(45),
      -90263/real(155925),
      // C[chi,mu]
      -96199/real(604800), 81/real(512), 1/real(360), -37/real(96), 2/real(3),
      -1/real(2),
      1118711/real(3870720), -46/real(105), 437/real(1440), -1/real(15),
      -1/real(48),
      -5569/real(90720), 209/real(4480), 37/real(840), -17/real(480),
      830251/real(7257600), 11/real(504), -4397/real(161280),
      108847/real(3991680), -4583/real(161280),
      -20648693/real(638668800),
      // C[chi,chi] skipped
      // C[chi,xi]
      -55271278/real(212837625), 27128/real(93555), -2312/real(14175),
      -88/real(315), 34/real(45), -2/real(3),
      106691108/real(638512875), -65864/real(155925), 6079/real(14175),
      -184/real(945), 1/real(45),
      5921152/real(54729675), -14246/real(467775), 772/real(14175),
      -106/real(2835),
      75594328/real(638512875), -5312/real(467775), -167/real(9450),
      2837636/real(638512875), -248/real(13365),
      -34761247/real(1915538625),
      // C[xi,phi]
      -44732/real(2837835), 20824/real(467775), 538/real(4725), 88/real(315),
      -4/real(45), -4/real(3),
      -12467764/real(212837625), -37192/real(467775), -2482/real(14175),
      8/real(105), 34/real(45),
      100320856/real(1915538625), 54968/real(467775), -898/real(14175),
      -1532/real(2835),
      -5884124/real(70945875), 24496/real(467775), 6007/real(14175),
      -839792/real(19348875), -23356/real(66825),
      570284222/real(1915538625),
      // C[xi,beta]
      -70496/real(8513505), 2476/real(467775), 34/real(675), 32/real(315),
      -4/real(45), -1/real(3),
      53836/real(212837625), 3992/real(467775), 74/real(2025), -4/real(315),
      -7/real(90),
      -661844/real(1915538625), 7052/real(467775), 2/real(14175),
      -83/real(2835),
      1425778/real(212837625), 934/real(467775), -797/real(56700),
      390088/real(212837625), -3673/real(467775),
      -18623681/real(3831077250LL),
      // C[xi,theta]
      -4286228/real(42567525), -193082/real(467775), 778/real(4725),
      62/real(105), -4/real(45), 2/real(3),
      -61623938/real(70945875), 92696/real(467775), 12338/real(14175),
      -32/real(315), 4/real(45),
      427003576/real(1915538625), 612536/real(467775), -1618/real(14175),
      -524/real(2835),
      427770788/real(212837625), -8324/real(66825), -5933/real(14175),
      -9153184/real(70945875), -320044/real(467775),
      -1978771378/real(1915538625),
      // C[xi,mu]
      -9292991/real(302702400), 7764059/real(239500800), 1297/real(18900),
      -817/real(10080), -4/real(45), 1/real(6),
      36019108271LL/real(871782912000LL), 35474/real(467775),
      -29609/real(453600), -2/real(35), 49/real(720),
      3026004511LL/real(30648618000LL), -4306823/real(59875200),
      -2917/real(56700), 4463/real(90720),
      -368661577/real(4036032000LL), -102293/real(1871100),
      331799/real(7257600),
      -875457073/real(13621608000LL), 11744233/real(239500800),
      453002260127LL/real(7846046208000LL),
      // C[xi,chi]
      2706758/real(42567525), -55222/real(93555), 2458/real(4725),
      46/real(315), -34/real(45), 2/real(3),
      -340492279/real(212837625), 516944/real(467775), 3413/real(14175),
      -256/real(315), 19/real(45),
      4430783356LL/real(1915538625), 206834/real(467775), -15958/real(14175),
      248/real(567),
      62016436/real(70945875), -832976/real(467775), 16049/real(28350),
      -651151712/real(212837625), 15602/real(18711),
      2561772812LL/real(1915538625),
      // C[xi,xi] skipped
    };
    static const int ptrs[] = {
      0, 0, 12, 24, 36, 57, 78, 90, 90, 102, 114, 135, 156, 168, 180, 180, 192,
      213, 234, 246, 258, 270, 270, 291, 312, 333, 354, 375, 396, 396, 417,
      438, 459, 480, 501, 522, 522,
    };
#elif GEOGRAPHICLIB_AUXLATITUDE_ORDER == 8
    static const real coeffs[] = {
      // C[phi,phi] skipped
      // C[phi,beta]; even coeffs only
      0, 0, 0, 1,
      0, 0, 0, 1/real(2),
      0, 0, 1/real(3),
      0, 0, 1/real(4),
      0, 1/real(5),
      0, 1/real(6),
      1/real(7),
      1/real(8),
      // C[phi,theta]; even coeffs only
      -2, 2, -2, 2,
      -8, 6, -4, 2,
      16, -8, 8/real(3),
      40, -16, 4,
      -32, 32/real(5),
      -64, 32/real(3),
      128/real(7),
      32,
      // C[phi,mu]; even coeffs only
      -6607/real(24576), 269/real(512), -27/real(32), 3/real(2),
      -155113/real(122880), 6759/real(4096), -55/real(32), 21/real(16),
      87963/real(20480), -417/real(128), 151/real(96),
      2514467/real(245760), -15543/real(2560), 1097/real(512),
      -69119/real(6144), 8011/real(2560),
      -5962461/real(286720), 293393/real(61440),
      6459601/real(860160),
      332287993/real(27525120),
      // C[phi,chi]
      189416/real(99225), 16822/real(4725), -2854/real(675), 26/real(45),
      116/real(45), -2, -2/real(3), 2,
      141514/real(8505), -31256/real(1575), 2323/real(945), 2704/real(315),
      -227/real(45), -8/real(5), 7/real(3),
      -2363828/real(31185), 98738/real(14175), 73814/real(2835),
      -1262/real(105), -136/real(35), 56/real(15),
      14416399/real(935550), 11763988/real(155925), -399572/real(14175),
      -332/real(35), 4279/real(630),
      258316372/real(1216215), -2046082/real(31185), -144838/real(6237),
      4174/real(315),
      -2155215124LL/real(14189175), -115444544/real(2027025),
      601676/real(22275),
      -170079376/real(1216215), 38341552/real(675675),
      1383243703/real(11351340),
      // C[phi,xi]
      -1683291094/real(37574026875LL), 22947844/real(1915538625),
      28112932/real(212837625), 60136/real(467775), -2582/real(14175),
      -16/real(35), 4/real(45), 4/real(3),
      -14351220203LL/real(488462349375LL), 1228352/real(3007125),
      251310128/real(638512875), -21016/real(51975), -11966/real(14175),
      152/real(945), 46/real(45),
      505559334506LL/real(488462349375LL), 138128272/real(147349125),
      -8797648/real(10945935), -94388/real(66825), 3802/real(14175),
      3044/real(2835),
      973080708361LL/real(488462349375LL), -45079184/real(29469825),
      -1472637812/real(638512875), 41072/real(93555), 6059/real(4725),
      -1385645336626LL/real(488462349375LL), -550000184/real(147349125),
      455935736/real(638512875), 768272/real(467775),
      -2939205114427LL/real(488462349375LL), 443810768/real(383107725),
      4210684958LL/real(1915538625),
      101885255158LL/real(54273594375LL), 387227992/real(127702575),
      1392441148867LL/real(325641566250LL),
      // C[beta,phi]; even coeffs only
      0, 0, 0, -1,
      0, 0, 0, 1/real(2),
      0, 0, -1/real(3),
      0, 0, 1/real(4),
      0, -1/real(5),
      0, 1/real(6),
      -1/real(7),
      1/real(8),
      // C[beta,beta] skipped
      // C[beta,theta]; even coeffs only
      0, 0, 0, 1,
      0, 0, 0, 1/real(2),
      0, 0, 1/real(3),
      0, 0, 1/real(4),
      0, 1/real(5),
      0, 1/real(6),
      1/real(7),
      1/real(8),
      // C[beta,mu]; even coeffs only
      -4879/real(73728), 205/real(1536), -9/real(32), 1/real(2),
      -86171/real(368640), 1335/real(4096), -37/real(96), 5/real(16),
      2901/real(4096), -75/real(128), 29/real(96),
      1082857/real(737280), -2391/real(2560), 539/real(1536),
      -28223/real(18432), 3467/real(7680),
      -733437/real(286720), 38081/real(61440),
      459485/real(516096),
      109167851/real(82575360),
      // C[beta,chi]
      -25666/real(99225), 4769/real(4725), -3118/real(4725), -1/real(3),
      38/real(45), -1/real(3), -2/real(3), 1,
      193931/real(42525), -14404/real(4725), -247/real(270), 50/real(21),
      -7/real(9), -14/real(15), 5/real(6),
      -1709614/real(155925), -36521/real(14175), 17564/real(2835), -5/real(3),
      -34/real(21), 16/real(15),
      -637699/real(85050), 2454416/real(155925), -49877/real(14175),
      -28/real(9), 2069/real(1260),
      48124558/real(1216215), -20989/real(2835), -28244/real(4455),
      883/real(315),
      -16969807/real(1091475), -2471888/real(184275), 797222/real(155925),
      -1238578/real(42525), 2199332/real(225225),
      87600385/real(4540536),
      // C[beta,xi]
      -5946082372LL/real(488462349375LL), 9708931/real(1915538625),
      7947332/real(212837625), 11824/real(467775), -1082/real(14175),
      -46/real(315), 4/real(45), 1/real(3),
      190673521/real(69780335625LL), 164328266/real(1915538625),
      39946703/real(638512875), -16672/real(155925), -338/real(2025),
      68/real(945), 17/real(90),
      86402898356LL/real(488462349375LL), 236067184/real(1915538625),
      -255454/real(1563705), -101069/real(467775), 1102/real(14175),
      461/real(2835),
      110123070361LL/real(488462349375LL), -98401826/real(383107725),
      -189032762/real(638512875), 1786/real(18711), 3161/real(18900),
      -200020620676LL/real(488462349375LL), -802887278/real(1915538625),
      80274086/real(638512875), 88868/real(467775),
      -296107325077LL/real(488462349375LL), 66263486/real(383107725),
      880980241/real(3831077250LL),
      4433064236LL/real(18091198125LL), 37151038/real(127702575),
      495248998393LL/real(1302566265000LL),
      // C[theta,phi]; even coeffs only
      2, -2, 2, -2,
      -8, 6, -4, 2,
      -16, 8, -8/real(3),
      40, -16, 4,
      32, -32/real(5),
      -64, 32/real(3),
      -128/real(7),
      32,
      // C[theta,beta]; even coeffs only
      0, 0, 0, -1,
      0, 0, 0, 1/real(2),
      0, 0, -1/real(3),
      0, 0, 1/real(4),
      0, -1/real(5),
      0, 1/real(6),
      -1/real(7),
      1/real(8),
      // C[theta,theta] skipped
      // C[theta,mu]; even coeffs only
      -14321/real(73728), 499/real(1536), -23/real(32), -1/real(2),
      -201467/real(368640), 6565/real(12288), -5/real(96), 5/real(16),
      2939/real(4096), -77/real(128), 1/real(32),
      1155049/real(737280), -4037/real(7680), 283/real(1536),
      -19465/real(18432), 1301/real(7680),
      -442269/real(286720), 17089/real(61440),
      198115/real(516096),
      48689387/real(82575360),
      // C[theta,chi]
      64424/real(99225), 76/real(225), -3658/real(4725), 2/real(9), 4/real(9),
      -2/real(3), -2/real(3), 0,
      2146/real(1215), -2728/real(945), 61/real(135), 68/real(45),
      -23/real(45), -4/real(15), 1/real(3),
      -95948/real(10395), 428/real(945), 9446/real(2835), -46/real(35),
      -24/real(35), 2/real(5),
      29741/real(85050), 4472/real(525), -34712/real(14175), -80/real(63),
      83/real(126),
      280108/real(13365), -17432/real(3465), -2362/real(891), 52/real(45),
      -48965632/real(4729725), -548752/real(96525), 335882/real(155925),
      -197456/real(15795), 51368/real(12285),
      1461335/real(174636),
      // C[theta,xi]
      -230886326/real(6343666875LL), -189115382/real(1915538625),
      216932/real(2627625), 109042/real(467775), -2102/real(14175),
      -158/real(315), 4/real(45), -2/real(3),
      -11696145869LL/real(69780335625LL), 288456008/real(1915538625),
      117952358/real(638512875), -7256/real(155925), 934/real(14175),
      -16/real(945), 16/real(45),
      91546732346LL/real(488462349375LL), 478700902/real(1915538625),
      -7391576/real(54729675), -25286/real(66825), 922/real(14175),
      -232/real(2835),
      218929662961LL/real(488462349375LL), -67330724/real(383107725),
      -67048172/real(638512875), 268/real(18711), 719/real(4725),
      -129039188386LL/real(488462349375LL), -117954842/real(273648375),
      46774256/real(638512875), 14354/real(467775),
      -178084928947LL/real(488462349375LL), 2114368/real(34827975),
      253129538/real(1915538625),
      6489189398LL/real(54273594375LL), 13805944/real(127702575),
      59983985827LL/real(325641566250LL),
      // C[mu,phi]; even coeffs only
      57/real(2048), -3/real(32), 9/real(16), -3/real(2),
      -105/real(4096), 135/real(2048), -15/real(32), 15/real(16),
      -105/real(2048), 105/real(256), -35/real(48),
      693/real(16384), -189/real(512), 315/real(512),
      693/real(2048), -693/real(1280),
      -1287/real(4096), 1001/real(2048),
      -6435/real(14336),
      109395/real(262144),
      // C[mu,beta]; even coeffs only
      19/real(2048), -1/real(32), 3/real(16), -1/real(2),
      7/real(4096), -9/real(2048), 1/real(32), -1/real(16),
      -3/real(2048), 3/real(256), -1/real(48),
      -11/real(16384), 3/real(512), -5/real(512),
      7/real(2048), -7/real(1280),
      9/real(4096), -7/real(2048),
      -33/real(14336),
      -429/real(262144),
      // C[mu,theta]; even coeffs only
      509/real(2048), -15/real(32), 13/real(16), 1/real(2),
      2599/real(4096), -1673/real(2048), 33/real(32), -1/real(16),
      -2989/real(2048), 349/real(256), -5/real(16),
      -43531/real(16384), 963/real(512), -261/real(512),
      5545/real(2048), -921/real(1280),
      16617/real(4096), -6037/real(6144),
      -19279/real(14336),
      -490925/real(262144),
      // C[mu,mu] skipped
      // C[mu,chi]
      -18975107/real(50803200), 72161/real(387072), 7891/real(37800),
      -127/real(288), 41/real(180), 5/real(16), -2/real(3), 1/real(2),
      148003883/real(174182400), 13769/real(28800), -1983433/real(1935360),
      281/real(630), 557/real(1440), -3/real(5), 13/real(48),
      79682431/real(79833600), -67102379/real(29030400), 167603/real(181440),
      15061/real(26880), -103/real(140), 61/real(240),
      -40176129013LL/real(7664025600LL), 97445/real(49896),
      6601661/real(7257600), -179/real(168), 49561/real(161280),
      2605413599LL/real(622702080), 14644087/real(9123840),
      -3418889/real(1995840), 34729/real(80640),
      175214326799LL/real(58118860800LL), -30705481/real(10378368),
      212378941/real(319334400),
      -16759934899LL/real(3113510400LL), 1522256789/real(1383782400),
      1424729850961LL/real(743921418240LL),
      // C[mu,xi]
      -375027460897LL/real(125046361440000LL),
      7183403063LL/real(560431872000LL), 12674323/real(851350500),
      -384229/real(14968800), -1609/real(28350), 121/real(1680), 4/real(45),
      -1/real(6),
      30410873385097LL/real(2000741783040000LL),
      1117820213/real(122594472000LL), -31621753811LL/real(1307674368000LL),
      -431/real(17325), 16463/real(453600), 26/real(945), -29/real(720),
      151567502183LL/real(17863765920000LL),
      -116359346641LL/real(3923023104000LL), -32844781/real(1751349600),
      3746047/real(119750400), 449/real(28350), -1003/real(45360),
      -317251099510901LL/real(8002967132160000LL), -13060303/real(766215450),
      10650637121LL/real(326918592000LL), 629/real(53460),
      -40457/real(2419200),
      -2105440822861LL/real(125046361440000LL),
      146875240637LL/real(3923023104000LL), 205072597/real(20432412000LL),
      -1800439/real(119750400),
      91496147778023LL/real(2000741783040000LL), 228253559/real(24518894400LL),
      -59109051671LL/real(3923023104000LL),
      126430355893LL/real(13894040160000LL),
      -4255034947LL/real(261534873600LL),
      -791820407649841LL/real(42682491371520000LL),
      // C[chi,phi]
      1514/real(1323), -8384/real(4725), 4642/real(4725), 32/real(45),
      -82/real(45), 4/real(3), 2/real(3), -2,
      142607/real(42525), -2288/real(1575), -1522/real(945), 904/real(315),
      -13/real(9), -16/real(15), 5/real(3),
      120202/real(51975), 44644/real(14175), -12686/real(2835), 8/real(5),
      34/real(21), -26/real(15),
      -1097407/real(187110), 1077964/real(155925), -24832/real(14175),
      -12/real(5), 1237/real(630),
      -12870194/real(1216215), 1040/real(567), 109598/real(31185),
      -734/real(315),
      -126463/real(72765), -941912/real(184275), 444337/real(155925),
      3463678/real(467775), -2405834/real(675675),
      256663081/real(56756700),
      // C[chi,beta]
      1384/real(11025), -34/real(4725), -998/real(4725), 2/real(5),
      -16/real(45), 0, 2/real(3), -1,
      -12616/real(42525), 1268/real(4725), -2/real(27), -22/real(105),
      19/real(45), -2/real(5), 1/real(6),
      1724/real(51975), -1858/real(14175), 116/real(567), -22/real(105),
      16/real(105), -1/real(15),
      115249/real(935550), -26836/real(155925), 2123/real(14175), -8/real(105),
      17/real(1260),
      140836/real(1216215), -424/real(6237), 128/real(4455), -1/real(105),
      210152/real(4729725), -31232/real(2027025), 149/real(311850),
      30208/real(6081075), -499/real(225225),
      -68251/real(113513400),
      // C[chi,theta]
      -1738/real(11025), 18/real(175), 1042/real(4725), -14/real(45),
      -2/real(9), 2/real(3), 2/real(3), 0,
      23159/real(42525), 332/real(945), -712/real(945), -4/real(45),
      43/real(45), 4/real(15), -1/real(3),
      13102/real(31185), -1352/real(945), 274/real(2835), 124/real(105),
      2/real(105), -2/real(5),
      -2414843/real(935550), 1528/real(4725), 21068/real(14175), -16/real(105),
      -55/real(126),
      60334/real(93555), 20704/real(10395), -9202/real(31185), -22/real(45),
      40458083/real(14189175), -299444/real(675675), -90263/real(155925),
      -3818498/real(6081075), -8962/real(12285),
      -4259027/real(4365900),
      // C[chi,mu]
      -7944359/real(67737600), 5406467/real(38707200), -96199/real(604800),
      81/real(512), 1/real(360), -37/real(96), 2/real(3), -1/real(2),
      -24749483/real(348364800), -51841/real(1209600), 1118711/real(3870720),
      -46/real(105), 437/real(1440), -1/real(15), -1/real(48),
      6457463/real(17740800), -9261899/real(58060800), -5569/real(90720),
      209/real(4480), 37/real(840), -17/real(480),
      -324154477/real(7664025600LL), -466511/real(2494800),
      830251/real(7257600), 11/real(504), -4397/real(161280),
      -22894433/real(124540416), 8005831/real(63866880), 108847/real(3991680),
      -4583/real(161280),
      2204645983LL/real(12915302400LL), 16363163/real(518918400),
      -20648693/real(638668800),
      497323811/real(12454041600LL), -219941297/real(5535129600LL),
      -191773887257LL/real(3719607091200LL),
      // C[chi,chi] skipped
      // C[chi,xi]
      -17451293242LL/real(488462349375LL), 308365186/real(1915538625),
      -55271278/real(212837625), 27128/real(93555), -2312/real(14175),
      -88/real(315), 34/real(45), -2/real(3),
      -101520127208LL/real(488462349375LL), 149984636/real(1915538625),
      106691108/real(638512875), -65864/real(155925), 6079/real(14175),
      -184/real(945), 1/real(45),
      10010741462LL/real(37574026875LL), -99534832/real(383107725),
      5921152/real(54729675), -14246/real(467775), 772/real(14175),
      -106/real(2835),
      1615002539/real(75148053750LL), -35573728/real(273648375),
      75594328/real(638512875), -5312/real(467775), -167/real(9450),
      -3358119706LL/real(488462349375LL), 130601488/real(1915538625),
      2837636/real(638512875), -248/real(13365),
      46771947158LL/real(488462349375LL), -3196/real(3553875),
      -34761247/real(1915538625),
      -18696014/real(18091198125LL), -2530364/real(127702575),
      -14744861191LL/real(651283132500LL),
      // C[xi,phi]
      -88002076/real(13956067125LL), -86728/real(16372125),
      -44732/real(2837835), 20824/real(467775), 538/real(4725), 88/real(315),
      -4/real(45), -4/real(3),
      -2641983469LL/real(488462349375LL), -895712/real(147349125),
      -12467764/real(212837625), -37192/real(467775), -2482/real(14175),
      8/real(105), 34/real(45),
      8457703444LL/real(488462349375LL), 240616/real(4209975),
      100320856/real(1915538625), 54968/real(467775), -898/real(14175),
      -1532/real(2835),
      -4910552477LL/real(97692469875LL), -4832848/real(147349125),
      -5884124/real(70945875), 24496/real(467775), 6007/real(14175),
      9393713176LL/real(488462349375LL), 816824/real(13395375),
      -839792/real(19348875), -23356/real(66825),
      -4532926649LL/real(97692469875LL), 1980656/real(54729675),
      570284222/real(1915538625),
      -14848113968LL/real(488462349375LL), -496894276/real(1915538625),
      224557742191LL/real(976924698750LL),
      // C[xi,beta]
      29232878/real(97692469875LL), -18484/real(4343625), -70496/real(8513505),
      2476/real(467775), 34/real(675), 32/real(315), -4/real(45), -1/real(3),
      -324943819/real(488462349375LL), -4160804/real(1915538625),
      53836/real(212837625), 3992/real(467775), 74/real(2025), -4/real(315),
      -7/real(90),
      -168643106/real(488462349375LL), 237052/real(383107725),
      -661844/real(1915538625), 7052/real(467775), 2/real(14175),
      -83/real(2835),
      113042383/real(97692469875LL), -2915326/real(1915538625),
      1425778/real(212837625), 934/real(467775), -797/real(56700),
      -558526274/real(488462349375LL), 6064888/real(1915538625),
      390088/real(212837625), -3673/real(467775),
      155665021/real(97692469875LL), 41288/real(29469825),
      -18623681/real(3831077250LL),
      504234982/real(488462349375LL), -6205669/real(1915538625),
      -8913001661LL/real(3907698795000LL),
      // C[xi,theta]
      182466964/real(8881133625LL), 53702182/real(212837625),
      -4286228/real(42567525), -193082/real(467775), 778/real(4725),
      62/real(105), -4/real(45), 2/real(3),
      367082779691LL/real(488462349375LL), -32500616/real(273648375),
      -61623938/real(70945875), 92696/real(467775), 12338/real(14175),
      -32/real(315), 4/real(45),
      -42668482796LL/real(488462349375LL), -663111728/real(383107725),
      427003576/real(1915538625), 612536/real(467775), -1618/real(14175),
      -524/real(2835),
      -327791986997LL/real(97692469875LL), 421877252/real(1915538625),
      427770788/real(212837625), -8324/real(66825), -5933/real(14175),
      74612072536LL/real(488462349375LL), 6024982024LL/real(1915538625),
      -9153184/real(70945875), -320044/real(467775),
      489898512247LL/real(97692469875LL), -46140784/real(383107725),
      -1978771378/real(1915538625),
      -42056042768LL/real(488462349375LL), -2926201612LL/real(1915538625),
      -2209250801969LL/real(976924698750LL),
      // C[xi,mu]
      39534358147LL/real(2858202547200LL),
      -25359310709LL/real(1743565824000LL), -9292991/real(302702400),
      7764059/real(239500800), 1297/real(18900), -817/real(10080), -4/real(45),
      1/real(6),
      -13216941177599LL/real(571640509440000LL),
      -14814966289LL/real(245188944000LL), 36019108271LL/real(871782912000LL),
      35474/real(467775), -29609/real(453600), -2/real(35), 49/real(720),
      -27782109847927LL/real(250092722880000LL),
      99871724539LL/real(1569209241600LL), 3026004511LL/real(30648618000LL),
      -4306823/real(59875200), -2917/real(56700), 4463/real(90720),
      168979300892599LL/real(1600593426432000LL),
      2123926699/real(15324309000LL), -368661577/real(4036032000LL),
      -102293/real(1871100), 331799/real(7257600),
      1959350112697LL/real(9618950880000LL),
      -493031379277LL/real(3923023104000LL), -875457073/real(13621608000LL),
      11744233/real(239500800),
      -145659994071373LL/real(800296713216000LL),
      -793693009/real(9807557760LL), 453002260127LL/real(7846046208000LL),
      -53583096419057LL/real(500185445760000LL),
      103558761539LL/real(1426553856000LL),
      real(12272105438887727LL)/real(128047474114560000LL),
      // C[xi,chi]
      -64724382148LL/real(97692469875LL), 16676974/real(30405375),
      2706758/real(42567525), -55222/real(93555), 2458/real(4725),
      46/real(315), -34/real(45), 2/real(3),
      85904355287LL/real(37574026875LL), 158999572/real(1915538625),
      -340492279/real(212837625), 516944/real(467775), 3413/real(14175),
      -256/real(315), 19/real(45),
      2986003168LL/real(37574026875LL), -7597644214LL/real(1915538625),
      4430783356LL/real(1915538625), 206834/real(467775), -15958/real(14175),
      248/real(567),
      -375566203/real(39037950), 851209552/real(174139875),
      62016436/real(70945875), -832976/real(467775), 16049/real(28350),
      5106181018156LL/real(488462349375LL), 3475643362LL/real(1915538625),
      -651151712/real(212837625), 15602/real(18711),
      34581190223LL/real(8881133625LL), -10656173804LL/real(1915538625),
      2561772812LL/real(1915538625),
      -5150169424688LL/real(488462349375LL), 873037408/real(383107725),
      7939103697617LL/real(1953849397500LL),
      // C[xi,xi] skipped
    };
    static const int ptrs[] = {
      0, 0, 20, 40, 60, 96, 132, 152, 152, 172, 192, 228, 264, 284, 304, 304,
      324, 360, 396, 416, 436, 456, 456, 492, 528, 564, 600, 636, 672, 672,
      708, 744, 780, 816, 852, 888, 888,
    };
#else
#error "Unsupported value for GEOGRAPHICLIB_AUXLATITUDE_ORDER"
#endif

    static_assert(sizeof(ptrs) / sizeof(int) == AUXNUMBER*AUXNUMBER+1,
                  "Mismatch in size of ptrs array");
    static_assert(sizeof(coeffs) / sizeof(real) ==
                  (RECTIFYING+1)*RECTIFYING *
                  (Lmax * (Lmax + 3) - 2*(Lmax/2))/4 // Even only arrays
                  + (AUXNUMBER*(AUXNUMBER-1) - (RECTIFYING+1)*RECTIFYING) *
                  (Lmax * (Lmax + 1))/2,
                  "Mismatch in size of coeffs array");

    if (k < 0) return;          // auxout or auxin out of range
    if (auxout == auxin)
      fill(_c + Lmax * k, _c + Lmax * (k + 1), 0);
    else {
      int o = ptrs[k];
      real d = _n;
      if (auxin <= RECTIFYING && auxout <= RECTIFYING) {
        for (int l = 0; l < Lmax; ++l) {
          int m = (Lmax - l - 1) / 2; // order of polynomial in n^2
          _c[Lmax * k + l] = d * Math::polyval(m, coeffs + o, _n2);
          o += m + 1;
          d *= _n;
        }
      } else {
        for (int l = 0; l < Lmax; ++l) {
          int m = (Lmax - l - 1); // order of polynomial in n
          _c[Lmax * k + l] = d * Math::polyval(m, coeffs + o, _n);
          o += m + 1;
          d *= _n;
        }
      }
      // assert (o == ptrs[AUXNUMBER * auxout + auxin + 1])
    }
  }

  Math::real AuxLatitude::Clenshaw(bool sinp, real szeta, real czeta,
                                   const real c[], int K) {
    // Evaluate
    // y = sum(c[k] * sin( (2*k+2) * zeta), k, 0, K-1) if  sinp
    // y = sum(c[k] * cos( (2*k+2) * zeta), k, 0, K-1) if !sinp
    // Approx operation count = (K + 5) mult and (2 * K + 2) add
    int k = K;
    real u0 = 0, u1 = 0,        // accumulators for sum
      x = 2 * (czeta - szeta) * (czeta + szeta); // 2 * cos(2*zeta)
    for (; k > 0;) {
      real t = x * u0 - u1 + c[--k];
      u1 = u0; u0 = t;
    }
    // u0*f0(zeta) - u1*fm1(zeta)
    // f0 = sinp ? sin(2*zeta) : cos(2*zeta)
    // fm1 = sinp ? 0 : 1
    real f0 = sinp ? 2 * szeta * czeta : x / 2, fm1 = sinp ? 0 : 1;
    return f0 * u0 - fm1 * u1;
  }
  /// \endcond

} // namespace GeographicLib
