/**
 * \file DAuxLatitude.cpp
 * \brief Implementation for the GeographicLib::DAuxLatitude class.
 *
 * This file is an implementation of the methods described in
 * - C. F. F. Karney,
 *   <a href="https://doi.org/10.1080/00396265.2023.2217604">
 *   On auxiliary latitudes,</a>
 *   Survey Review 56(395), 165--180 (2024);
 *   preprint
 *   <a href="https://arxiv.org/abs/2212.05818">arXiv:2212.05818</a>.
 * .
 * Copyright (c) Charles Karney (2022-2024) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <GeographicLib/DAuxLatitude.hpp>
#include <GeographicLib/EllipticFunction.hpp>

namespace GeographicLib {

  using namespace std;

  Math::real DAuxLatitude::DRectifying(const AuxAngle& phi1,
                                       const AuxAngle& phi2)
    const {
    // Stipulate that phi1 and phi2 are in [-90d, 90d]
    real x = phi1.radians(), y = phi2.radians();
    if (x == y) {
      real d;
      AuxAngle mu1(base::Rectifying(phi1, &d));
      real tphi1 = phi1.tan(), tmu1 = mu1.tan();
      return
        isfinite(tphi1) ? d * Math::sq(base::sc(tphi1)/base::sc(tmu1)) : 1/d;
    } else if (x * y < 0)
      return (base::Rectifying(phi2).radians() -
              base::Rectifying(phi1).radians()) / (y - x);
    else {
      AuxAngle bet1(base::Parametric(phi1)), bet2(base::Parametric(phi2));
      real dEdbet = DE(bet1, bet2), dbetdphi = DParametric(phi1, phi2);
      return base::_b * dEdbet / base::RectifyingRadius(true) * dbetdphi;
    }
  }

  Math::real DAuxLatitude::DParametric(const AuxAngle& phi1,
                                       const AuxAngle& phi2)
    const {
    real tx = phi1.tan(), ty = phi2.tan(), r;
    // DbetaDphi = Datan(fm1*tx, fm1*ty) * fm1 / Datan(tx, ty)
    // Datan(x, y) = 1/(1 + x^2),                       for x = y
    //             = (atan(y) - atan(x)) / (y-x),       for x*y < 0
    //             = atan( (y-x) / (1 + x*y) ) / (y-x), for x*y > 0
    if (!(tx * ty >= 0))        // This includes, e.g., tx = 0, ty = inf
      r = (atan(base::_fm1 * ty) - atan(base::_fm1 * tx)) /
        (atan(ty) - atan(tx));
    else if (tx == ty) {        // This includes the case tx = ty = inf
      tx *= tx;
      if (tx <= 1)
        r = base::_fm1 * (1 + tx) / (1 + base::_e2m1 * tx);
      else {
        tx = 1/tx;
        r = base::_fm1 * (1 + tx) / (base::_e2m1 + tx);
      }
    } else {
      if (tx * ty <= 1)
        r = atan2(base::_fm1 * (ty - tx), 1 + base::_e2m1 * tx * ty)
          / atan2(        ty - tx , 1 +         tx * ty);
      else {
        tx = 1/tx; ty = 1/ty;
        r = atan2(base::_fm1 * (ty - tx), base::_e2m1 + tx * ty)
          / atan2(        ty - tx ,   1   + tx * ty);
      }
    }
    return r;
  }

  Math::real DAuxLatitude::DE(const AuxAngle& X, const AuxAngle& Y) const {
    AuxAngle Xn(X.normalized()), Yn(Y.normalized());
    // We assume that X and Y are in [-90d, 90d] and have the same sign
    // If not we would include
    //    if (Xn.y() * Yn.y() < 0)
    //      return d != 0 ? (E(X) - E(Y)) / d : 1;

    // The general formula fails for x = y = 0d and x = y = 90d.  Probably this
    // is fixable (the formula works for other x = y.  But let's also stipulate
    // that x != y .

    // Make both positive, so we can do the swap a <-> b trick
    Xn.y() = fabs(Xn.y()); Yn.y() = fabs(Yn.y());
    real k2 = -base::_e12;
    bool flip = base::_f < 0;
    // Switch prolate to oblate; we then can use the formulas for k2 < 0
    if (flip) {
      swap(Xn.x(), Xn.y());
      swap(Yn.x(), Yn.y());
      k2 = base::_e2;
    }
    real x = Xn.radians(), y = Yn.radians(), d = y - x,
      sx = Xn.y(), sy = Yn.y(), cx = Xn.x(), cy = Yn.x();
    // See DLMF: Eqs (19.11.2) and (19.11.4) letting
    // theta -> x, phi -> -y, psi -> z
    //
    // (E(y) - E(x)) / d = E(z)/d - k2 * sin(x) * sin(y) * sin(z)/d
    //                   = (E(z)/sin(z) - k2 * sin(x) * sin(y)) * sin(z)/d
    // tan(z/2) = (sin(x)*Delta(y) - sin(y)*Delta(x)) / (cos(x) + cos(y))
    //          = d * Dsin(x,y) * (sin(x) + sin(y))/(cos(x) + cos(y)) /
    //             (sin(x)*Delta(y) + sin(y)*Delta(x))
    //          = t = d * Dt
    // Delta(x) = sqrt(1 - k2 * sin(x)^2)
    // sin(z) = 2*t/(1+t^2); cos(z) = (1-t^2)/(1+t^2)
    real Dt = Dsin(x, y) * (sx + sy) /
      ((cx + cy) * (sx * sqrt(1 - k2 * sy*sy) + sy * sqrt(1 - k2 * sx*sx))),
      t = d * Dt, Dsz = 2 * Dt / (1 + t*t),
      sz = d * Dsz, cz = (1 - t) * (1 + t) / (1 + t*t),
      sz2 = sz*sz, cz2 = cz*cz, dz2 = 1 - k2 * sz2,
      // E(z)/sin(z)
      Ezbsz = (EllipticFunction::RF(cz2, dz2, 1)
               - k2 * sz2 * EllipticFunction::RD(cz2, dz2, 1) / 3);
    return (Ezbsz - k2 * sx * sy) * Dsz / (flip ? 1 - base::_f : 1);
  }

  /// \cond SKIP
  Math::real DAuxLatitude::Dsn(real x, real y) {
    real sc1 = base::sc(x);
    if (x == y) return 1 / (sc1 * (1 + x*x));
    real sc2 = base::sc(y), sn1 = base::sn(x), sn2 = base::sn(y);
    return x * y > 0 ?
      (sn1/sc2 + sn2/sc1) / ((sn1 + sn2) * sc1 * sc2) :
      (sn2 - sn1) / (y - x);
  }
  Math::real DAuxLatitude::Datan(real x, real y) {
    using std::isinf;           // Needed for Centos 7, ubuntu 14
    real d = y - x, xy = x*y;
    return x == y ? 1 / (1 + xy) :
      (isinf(xy) && xy > 0 ? 0 :
       (2 * xy > -1 ? atan( d / (1 + xy) ) : atan(y) - atan(x)) / d);
  }
  Math::real DAuxLatitude::Dasinh(real x, real y) {
    using std::isinf;           // Needed for Centos 7, ubuntu 14
    real d = y - x, xy = x*y, hx = base::sc(x), hy = base::sc(y);
    // KF formula for x*y < 0 is asinh(y*hx - x*hy) / (y - x)
    // but this has problem if x*y overflows to -inf
    return x == y ? 1 / hx :
      (isinf(d) ? 0 :
       (xy > 0 ? asinh(d * (x*y < 1 ? (x + y) / (x*hy + y*hx) :
                            (1/x + 1/y) / (hy/y + hx/x))) :
        asinh(y) - asinh(x)) / d);
  }
  Math::real DAuxLatitude::Dh(real x, real y) {
    using std::isnan; using std::isinf; // Needed for Centos 7, ubuntu 14
    if (isnan(x + y))
      return x + y;           // N.B. nan for inf-inf
    if (isinf(x))
      return copysign(1/real(2), x);
    if (isinf(y))
      return copysign(1/real(2), y);
    real sx = base::sn(x), sy = base::sn(y), d = sx*x + sy*y;
    if (d / 2 == 0)
      return (x + y) / 2;     // Handle underflow
    if (x * y <= 0)
      return (h(y) - h(x)) / (y - x); // Does not include x = y = 0
    real scx = base::sc(x), scy = base::sc(y);
    return ((x + y) / (2 * d)) *
      (Math::sq(sx*sy) + Math::sq(sy/scx) + Math::sq(sx/scy));
  }
  Math::real DAuxLatitude::Datanhee(real x, real y) const {
    // atan(e*sn(tphi))/e:
    //  Datan(e*sn(x),e*sn(y))*Dsn(x,y)/Datan(x,y)
    // asinh(e1*sn(fm1*tphi)):
    //  Dasinh(e1*sn(fm1*x)), e1*sn(fm1*y)) *
    // e1 * Dsn(fm1*x, fm1*y) *fm1 / (e * Datan(x,y))
    // = Dasinh(e1*sn(fm1*x)), e1*sn(fm1*y)) *
    //  Dsn(fm1*x, fm1*y) / Datan(x,y)
    return base::_f < 0 ?
      Datan(base::_e * base::sn(x), base::_e * base::sn(y)) * Dsn(x, y) :
      Dasinh(base::_e1 * base::sn(base::_fm1 * x),
             base::_e1 * base::sn(base::_fm1 * y)) *
      Dsn(base::_fm1 * x, base::_fm1 * y);
  }
  /// \endcond

  Math::real DAuxLatitude::DIsometric(const AuxAngle& phi1,
                                      const AuxAngle& phi2)
    const {
    // psi = asinh(tan(phi)) - e^2 * atanhee(tan(phi))
    using std::isnan; using std::isinf; // Needed for Centos 7, ubuntu 14
    real tphi1 = phi1.tan(), tphi2 = phi2.tan();
    return isnan(tphi1) || isnan(tphi2) ? numeric_limits<real>::quiet_NaN() :
      (isinf(tphi1) || isinf(tphi2) ? numeric_limits<real>::infinity() :
       (Dasinh(tphi1, tphi2) - base::_e2 * Datanhee(tphi1, tphi2)) /
       Datan(tphi1, tphi2));
  }

  Math::real DAuxLatitude::DConvert(int auxin, int auxout,
                                    const AuxAngle& zeta1,
                                    const AuxAngle& zeta2)
    const {
    using std::isnan;           // Needed for Centos 7, ubuntu 14
    int k = base::ind(auxout, auxin);
    if (k < 0) return numeric_limits<real>::quiet_NaN();
    if (auxin == auxout) return 1;
    if ( isnan(base::_c[base::Lmax * (k + 1) - 1]) )
      base::fillcoeff(auxin, auxout, k);
    AuxAngle zeta1n(zeta1.normalized()), zeta2n(zeta2.normalized());
    return 1 + DClenshaw(true, zeta2n.radians() - zeta1n.radians(),
                         zeta1n.y(), zeta1n.x(), zeta2n.y(), zeta2n.x(),
                         base::_c + base::Lmax * k, base::Lmax);
  }

  Math::real DAuxLatitude::DClenshaw(bool sinp, real Delta,
                                     real szeta1, real czeta1,
                                     real szeta2, real czeta2,
                                     const real c[], int K) {
    // Evaluate
    // (Clenshaw(sinp, szeta2, czeta2, c, K) -
    //  Clenshaw(sinp, szeta1, czeta1, c, K)) / Delta
    // or
    // sum(c[k] * (sin( (2*k+2) * zeta2) - sin( (2*k+2) * zeta2)), i, 0, K-1)
    //   / Delta
    // (if !sinp, then change sin->cos here.)
    //
    // Delta is EITHER 1, giving the plain difference OR (zeta2 - zeta1) in
    // radians, giving the divided difference.  Other values will give
    // nonsense.
    //
    int k = K;
    // suffices a b denote [1,1], [2,1] elements of matrix/vector
    real D2 = Delta * Delta,
      czetap = czeta2 * czeta1 - szeta2 * szeta1,
      szetap = szeta2 * czeta1 + czeta2 * szeta1,
      czetam = czeta2 * czeta1 + szeta2 * szeta1,
      // sin(zetam) / Delta
      szetamd =  (Delta == 1 ? szeta2 * czeta1 - czeta2 * szeta1 :
                 (Delta != 0 ? sin(Delta) / Delta : 1)),
      Xa =  2 * czetap * czetam,
      Xb = -2 * szetap * szetamd,
      u0a = 0, u0b = 0, u1a = 0, u1b = 0; // accumulators for sum
    for (--k; k >= 0; --k) {
      // temporary real = X . U0 - U1 + c[k] * I
      real ta = Xa * u0a + D2 * Xb * u0b - u1a + c[k],
        tb = Xb * u0a +      Xa * u0b - u1b;
      // U1 = U0; U0 = real
      u1a = u0a; u0a = ta;
      u1b = u0b; u0b = tb;
    }
    // P = U0 . F[0] - U1 . F[-1]
    // if sinp:
    //   F[0] = [ sin(2*zeta2) + sin(2*zeta1),
    //           (sin(2*zeta2) - sin(2*zeta1)) / Delta]
    //        = 2 * [ szetap * czetam, czetap * szetamd ]
    //   F[-1] = [0, 0]
    // else:
    //   F[0] = [ cos(2*zeta2) + cos(2*zeta1),
    //           (cos(2*zeta2) - cos(2*zeta1)) / Delta]
    //        = 2 * [ czetap * czetam, -szetap * szetamd ]
    //   F[-1] = [2, 0]
    real F0a = (sinp ? szetap :  czetap) * czetam,
      F0b = (sinp ? czetap : -szetap) * szetamd,
      Fm1a = sinp ? 0 : 1;  // Fm1b = 0;
    // Don't both to compute sum...
    // divided difference (or difference if Delta == 1)
    return 2 * (F0a * u0b + F0b * u0a  - Fm1a * u1b);
  }

} // namespace GeographicLib
