/**
 * \file Geodesic.cpp
 * \brief Implementation for GeographicLib::Geodesic class
 *
 * Copyright (c) Charles Karney (2009-2017) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 *
 * This is a reformulation of the geodesic problem.  The notation is as
 * follows:
 * - at a general point (no suffix or 1 or 2 as suffix)
 *   - phi = latitude
 *   - beta = latitude on auxiliary sphere
 *   - omega = longitude on auxiliary sphere
 *   - lambda = longitude
 *   - alpha = azimuth of great circle
 *   - sigma = arc length along great circle
 *   - s = distance
 *   - tau = scaled distance (= sigma at multiples of pi/2)
 * - at northwards equator crossing
 *   - beta = phi = 0
 *   - omega = lambda = 0
 *   - alpha = alpha0
 *   - sigma = s = 0
 * - a 12 suffix means a difference, e.g., s12 = s2 - s1.
 * - s and c prefixes mean sin and cos
 **********************************************************************/

#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/GeodesicLine.hpp>

#if defined(_MSC_VER)
// Squelch warnings about potentially uninitialized local variables and
// constant conditional expressions
#  pragma warning (disable: 4701 4127)
#endif

namespace GeographicLib {

  using namespace std;

  Geodesic::Geodesic(real a, real f)
    : maxit2_(maxit1_ + Math::digits() + 10)
      // Underflow guard.  We require
      //   tiny_ * epsilon() > 0
      //   tiny_ + epsilon() == epsilon()
    , tiny_(sqrt(numeric_limits<real>::min()))
    , tol0_(numeric_limits<real>::epsilon())
      // Increase multiplier in defn of tol1_ from 100 to 200 to fix inverse
      // case 52.784459512564 0 -52.784459512563990912 179.634407464943777557
      // which otherwise failed for Visual Studio 10 (Release and Debug)
    , tol1_(200 * tol0_)
    , tol2_(sqrt(tol0_))
    , tolb_(tol0_ * tol2_)      // Check on bisection interval
    , xthresh_(1000 * tol2_)
    , _a(a)
    , _f(f)
    , _f1(1 - _f)
    , _e2(_f * (2 - _f))
    , _ep2(_e2 / Math::sq(_f1)) // e2 / (1 - e2)
    , _n(_f / ( 2 - _f))
    , _b(_a * _f1)
    , _c2((Math::sq(_a) + Math::sq(_b) *
           (_e2 == 0 ? 1 :
            Math::eatanhe(real(1), (_f < 0 ? -1 : 1) * sqrt(abs(_e2))) / _e2))
          / 2) // authalic radius squared
      // The sig12 threshold for "really short".  Using the auxiliary sphere
      // solution with dnm computed at (bet1 + bet2) / 2, the relative error in
      // the azimuth consistency check is sig12^2 * abs(f) * min(1, 1-f/2) / 2.
      // (Error measured for 1/100 < b/a < 100 and abs(f) >= 1/1000.  For a
      // given f and sig12, the max error occurs for lines near the pole.  If
      // the old rule for computing dnm = (dn1 + dn2)/2 is used, then the error
      // increases by a factor of 2.)  Setting this equal to epsilon gives
      // sig12 = etol2.  Here 0.1 is a safety factor (error decreased by 100)
      // and max(0.001, abs(f)) stops etol2 getting too large in the nearly
      // spherical case.
    , _etol2(real(0.1) * tol2_ /
             sqrt( max(real(0.001), abs(_f)) * min(real(1), 1 - _f/2) / 2 ))
  {
    if (!(Math::isfinite(_a) && _a > 0))
      throw GeographicErr("Equatorial radius is not positive");
    if (!(Math::isfinite(_b) && _b > 0))
      throw GeographicErr("Polar semi-axis is not positive");
    A3coeff();
    C3coeff();
    C4coeff();
  }

  const Geodesic& Geodesic::WGS84() {
    static const Geodesic wgs84(Constants::WGS84_a(), Constants::WGS84_f());
    return wgs84;
  }

  Math::real Geodesic::SinCosSeries(bool sinp,
                                    real sinx, real cosx,
                                    const real c[], int n) {
    // Evaluate
    // y = sinp ? sum(c[i] * sin( 2*i    * x), i, 1, n) :
    //            sum(c[i] * cos((2*i+1) * x), i, 0, n-1)
    // using Clenshaw summation.  N.B. c[0] is unused for sin series
    // Approx operation count = (n + 5) mult and (2 * n + 2) add
    c += (n + sinp);            // Point to one beyond last element
    real
      ar = 2 * (cosx - sinx) * (cosx + sinx), // 2 * cos(2 * x)
      y0 = n & 1 ? *--c : 0, y1 = 0;          // accumulators for sum
    // Now n is even
    n /= 2;
    while (n--) {
      // Unroll loop x 2, so accumulators return to their original role
      y1 = ar * y0 - y1 + *--c;
      y0 = ar * y1 - y0 + *--c;
    }
    return sinp
      ? 2 * sinx * cosx * y0    // sin(2 * x) * y0
      : cosx * (y0 - y1);       // cos(x) * (y0 - y1)
  }

  GeodesicLine Geodesic::Line(real lat1, real lon1, real azi1,
                              unsigned caps) const {
    return GeodesicLine(*this, lat1, lon1, azi1, caps);
  }

  Math::real Geodesic::GenDirect(real lat1, real lon1, real azi1,
                                 bool arcmode, real s12_a12, unsigned outmask,
                                 real& lat2, real& lon2, real& azi2,
                                 real& s12, real& m12, real& M12, real& M21,
                                 real& S12) const {
    // Automatically supply DISTANCE_IN if necessary
    if (!arcmode) outmask |= DISTANCE_IN;
    return GeodesicLine(*this, lat1, lon1, azi1, outmask)
      .                         // Note the dot!
      GenPosition(arcmode, s12_a12, outmask,
                  lat2, lon2, azi2, s12, m12, M12, M21, S12);
  }

  GeodesicLine Geodesic::GenDirectLine(real lat1, real lon1, real azi1,
                                       bool arcmode, real s12_a12,
                                       unsigned caps) const {
    azi1 = Math::AngNormalize(azi1);
    real salp1, calp1;
    // Guard against underflow in salp0.  Also -0 is converted to +0.
    Math::sincosd(Math::AngRound(azi1), salp1, calp1);
    // Automatically supply DISTANCE_IN if necessary
    if (!arcmode) caps |= DISTANCE_IN;
    return GeodesicLine(*this, lat1, lon1, azi1, salp1, calp1,
                        caps, arcmode, s12_a12);
  }

  GeodesicLine Geodesic::DirectLine(real lat1, real lon1, real azi1, real s12,
                                    unsigned caps) const {
    return GenDirectLine(lat1, lon1, azi1, false, s12, caps);
  }

  GeodesicLine Geodesic::ArcDirectLine(real lat1, real lon1, real azi1,
                                       real a12, unsigned caps) const {
    return GenDirectLine(lat1, lon1, azi1, true, a12, caps);
  }

  Math::real Geodesic::GenInverse(real lat1, real lon1, real lat2, real lon2,
                                  unsigned outmask, real& s12,
                                  real& salp1, real& calp1,
                                  real& salp2, real& calp2,
                                  real& m12, real& M12, real& M21,
                                  real& S12) const {
    // Compute longitude difference (AngDiff does this carefully).  Result is
    // in [-180, 180] but -180 is only for west-going geodesics.  180 is for
    // east-going and meridional geodesics.
    real lon12s, lon12 = Math::AngDiff(lon1, lon2, lon12s);
    // Make longitude difference positive.
    int lonsign = lon12 >= 0 ? 1 : -1;
    // If very close to being on the same half-meridian, then make it so.
    lon12 = lonsign * Math::AngRound(lon12);
    lon12s = Math::AngRound((180 - lon12) - lonsign * lon12s);
    real
      lam12 = lon12 * Math::degree(),
      slam12, clam12;
    if (lon12 > 90) {
      Math::sincosd(lon12s, slam12, clam12);
      clam12 = -clam12;
    } else
      Math::sincosd(lon12, slam12, clam12);

    // If really close to the equator, treat as on equator.
    lat1 = Math::AngRound(Math::LatFix(lat1));
    lat2 = Math::AngRound(Math::LatFix(lat2));
    // Swap points so that point with higher (abs) latitude is point 1
    // If one latitude is a nan, then it becomes lat1.
    int swapp = abs(lat1) < abs(lat2) ? -1 : 1;
    if (swapp < 0) {
      lonsign *= -1;
      swap(lat1, lat2);
    }
    // Make lat1 <= 0
    int latsign = lat1 < 0 ? 1 : -1;
    lat1 *= latsign;
    lat2 *= latsign;
    // Now we have
    //
    //     0 <= lon12 <= 180
    //     -90 <= lat1 <= 0
    //     lat1 <= lat2 <= -lat1
    //
    // longsign, swapp, latsign register the transformation to bring the
    // coordinates to this canonical form.  In all cases, 1 means no change was
    // made.  We make these transformations so that there are few cases to
    // check, e.g., on verifying quadrants in atan2.  In addition, this
    // enforces some symmetries in the results returned.

    real sbet1, cbet1, sbet2, cbet2, s12x, m12x;

    Math::sincosd(lat1, sbet1, cbet1); sbet1 *= _f1;
    // Ensure cbet1 = +epsilon at poles; doing the fix on beta means that sig12
    // will be <= 2*tiny for two points at the same pole.
    Math::norm(sbet1, cbet1); cbet1 = max(tiny_, cbet1);

    Math::sincosd(lat2, sbet2, cbet2); sbet2 *= _f1;
    // Ensure cbet2 = +epsilon at poles
    Math::norm(sbet2, cbet2); cbet2 = max(tiny_, cbet2);

    // If cbet1 < -sbet1, then cbet2 - cbet1 is a sensitive measure of the
    // |bet1| - |bet2|.  Alternatively (cbet1 >= -sbet1), abs(sbet2) + sbet1 is
    // a better measure.  This logic is used in assigning calp2 in Lambda12.
    // Sometimes these quantities vanish and in that case we force bet2 = +/-
    // bet1 exactly.  An example where is is necessary is the inverse problem
    // 48.522876735459 0 -48.52287673545898293 179.599720456223079643
    // which failed with Visual Studio 10 (Release and Debug)

    if (cbet1 < -sbet1) {
      if (cbet2 == cbet1)
        sbet2 = sbet2 < 0 ? sbet1 : -sbet1;
    } else {
      if (abs(sbet2) == -sbet1)
        cbet2 = cbet1;
    }

    real
      dn1 = sqrt(1 + _ep2 * Math::sq(sbet1)),
      dn2 = sqrt(1 + _ep2 * Math::sq(sbet2));

    real a12, sig12;
    // index zero element of this array is unused
    real Ca[nC_];

    bool meridian = lat1 == -90 || slam12 == 0;

    if (meridian) {

      // Endpoints are on a single full meridian, so the geodesic might lie on
      // a meridian.

      calp1 = clam12; salp1 = slam12; // Head to the target longitude
      calp2 = 1; salp2 = 0;           // At the target we're heading north

      real
        // tan(bet) = tan(sig) * cos(alp)
        ssig1 = sbet1, csig1 = calp1 * cbet1,
        ssig2 = sbet2, csig2 = calp2 * cbet2;

      // sig12 = sig2 - sig1
      sig12 = atan2(max(real(0), csig1 * ssig2 - ssig1 * csig2),
                                 csig1 * csig2 + ssig1 * ssig2);
      {
        real dummy;
        Lengths(_n, sig12, ssig1, csig1, dn1, ssig2, csig2, dn2, cbet1, cbet2,
                outmask | DISTANCE | REDUCEDLENGTH,
                s12x, m12x, dummy, M12, M21, Ca);
      }
      // Add the check for sig12 since zero length geodesics might yield m12 <
      // 0.  Test case was
      //
      //    echo 20.001 0 20.001 0 | GeodSolve -i
      //
      // In fact, we will have sig12 > pi/2 for meridional geodesic which is
      // not a shortest path.
      if (sig12 < 1 || m12x >= 0) {
        // Need at least 2, to handle 90 0 90 180
        if (sig12 < 3 * tiny_)
          sig12 = m12x = s12x = 0;
        m12x *= _b;
        s12x *= _b;
        a12 = sig12 / Math::degree();
      } else
        // m12 < 0, i.e., prolate and too close to anti-podal
        meridian = false;
    }

    // somg12 > 1 marks that it needs to be calculated
    real omg12 = 0, somg12 = 2, comg12 = 0;
    if (!meridian &&
        sbet1 == 0 &&   // and sbet2 == 0
        (_f <= 0 || lon12s >= _f * 180)) {

      // Geodesic runs along equator
      calp1 = calp2 = 0; salp1 = salp2 = 1;
      s12x = _a * lam12;
      sig12 = omg12 = lam12 / _f1;
      m12x = _b * sin(sig12);
      if (outmask & GEODESICSCALE)
        M12 = M21 = cos(sig12);
      a12 = lon12 / _f1;

    } else if (!meridian) {

      // Now point1 and point2 belong within a hemisphere bounded by a
      // meridian and geodesic is neither meridional or equatorial.

      // Figure a starting point for Newton's method
      real dnm;
      sig12 = InverseStart(sbet1, cbet1, dn1, sbet2, cbet2, dn2,
                           lam12, slam12, clam12,
                           salp1, calp1, salp2, calp2, dnm,
                           Ca);

      if (sig12 >= 0) {
        // Short lines (InverseStart sets salp2, calp2, dnm)
        s12x = sig12 * _b * dnm;
        m12x = Math::sq(dnm) * _b * sin(sig12 / dnm);
        if (outmask & GEODESICSCALE)
          M12 = M21 = cos(sig12 / dnm);
        a12 = sig12 / Math::degree();
        omg12 = lam12 / (_f1 * dnm);
      } else {

        // Newton's method.  This is a straightforward solution of f(alp1) =
        // lambda12(alp1) - lam12 = 0 with one wrinkle.  f(alp) has exactly one
        // root in the interval (0, pi) and its derivative is positive at the
        // root.  Thus f(alp) is positive for alp > alp1 and negative for alp <
        // alp1.  During the course of the iteration, a range (alp1a, alp1b) is
        // maintained which brackets the root and with each evaluation of
        // f(alp) the range is shrunk, if possible.  Newton's method is
        // restarted whenever the derivative of f is negative (because the new
        // value of alp1 is then further from the solution) or if the new
        // estimate of alp1 lies outside (0,pi); in this case, the new starting
        // guess is taken to be (alp1a + alp1b) / 2.
        //
        // initial values to suppress warnings (if loop is executed 0 times)
        real ssig1 = 0, csig1 = 0, ssig2 = 0, csig2 = 0, eps = 0, domg12 = 0;
        unsigned numit = 0;
        // Bracketing range
        real salp1a = tiny_, calp1a = 1, salp1b = tiny_, calp1b = -1;
        for (bool tripn = false, tripb = false;
             numit < maxit2_ || GEOGRAPHICLIB_PANIC;
             ++numit) {
          // the WGS84 test set: mean = 1.47, sd = 1.25, max = 16
          // WGS84 and random input: mean = 2.85, sd = 0.60
          real dv;
          real v = Lambda12(sbet1, cbet1, dn1, sbet2, cbet2, dn2, salp1, calp1,
                            slam12, clam12,
                            salp2, calp2, sig12, ssig1, csig1, ssig2, csig2,
                            eps, domg12, numit < maxit1_, dv, Ca);
          // Reversed test to allow escape with NaNs
          if (tripb || !(abs(v) >= (tripn ? 8 : 1) * tol0_)) break;
          // Update bracketing values
          if (v > 0 && (numit > maxit1_ || calp1/salp1 > calp1b/salp1b))
            { salp1b = salp1; calp1b = calp1; }
          else if (v < 0 && (numit > maxit1_ || calp1/salp1 < calp1a/salp1a))
            { salp1a = salp1; calp1a = calp1; }
          if (numit < maxit1_ && dv > 0) {
            real
              dalp1 = -v/dv;
            real
              sdalp1 = sin(dalp1), cdalp1 = cos(dalp1),
              nsalp1 = salp1 * cdalp1 + calp1 * sdalp1;
            if (nsalp1 > 0 && abs(dalp1) < Math::pi()) {
              calp1 = calp1 * cdalp1 - salp1 * sdalp1;
              salp1 = nsalp1;
              Math::norm(salp1, calp1);
              // In some regimes we don't get quadratic convergence because
              // slope -> 0.  So use convergence conditions based on epsilon
              // instead of sqrt(epsilon).
              tripn = abs(v) <= 16 * tol0_;
              continue;
            }
          }
          // Either dv was not positive or updated value was outside legal
          // range.  Use the midpoint of the bracket as the next estimate.
          // This mechanism is not needed for the WGS84 ellipsoid, but it does
          // catch problems with more eccentric ellipsoids.  Its efficacy is
          // such for the WGS84 test set with the starting guess set to alp1 =
          // 90deg:
          // the WGS84 test set: mean = 5.21, sd = 3.93, max = 24
          // WGS84 and random input: mean = 4.74, sd = 0.99
          salp1 = (salp1a + salp1b)/2;
          calp1 = (calp1a + calp1b)/2;
          Math::norm(salp1, calp1);
          tripn = false;
          tripb = (abs(salp1a - salp1) + (calp1a - calp1) < tolb_ ||
                   abs(salp1 - salp1b) + (calp1 - calp1b) < tolb_);
        }
        {
          real dummy;
          // Ensure that the reduced length and geodesic scale are computed in
          // a "canonical" way, with the I2 integral.
          unsigned lengthmask = outmask |
            (outmask & (REDUCEDLENGTH | GEODESICSCALE) ? DISTANCE : NONE);
          Lengths(eps, sig12, ssig1, csig1, dn1, ssig2, csig2, dn2,
                  cbet1, cbet2, lengthmask, s12x, m12x, dummy, M12, M21, Ca);
        }
        m12x *= _b;
        s12x *= _b;
        a12 = sig12 / Math::degree();
        if (outmask & AREA) {
          // omg12 = lam12 - domg12
          real sdomg12 = sin(domg12), cdomg12 = cos(domg12);
          somg12 = slam12 * cdomg12 - clam12 * sdomg12;
          comg12 = clam12 * cdomg12 + slam12 * sdomg12;
        }
      }
    }

    if (outmask & DISTANCE)
      s12 = 0 + s12x;           // Convert -0 to 0

    if (outmask & REDUCEDLENGTH)
      m12 = 0 + m12x;           // Convert -0 to 0

    if (outmask & AREA) {
      real
        // From Lambda12: sin(alp1) * cos(bet1) = sin(alp0)
        salp0 = salp1 * cbet1,
        calp0 = Math::hypot(calp1, salp1 * sbet1); // calp0 > 0
      real alp12;
      if (calp0 != 0 && salp0 != 0) {
        real
          // From Lambda12: tan(bet) = tan(sig) * cos(alp)
          ssig1 = sbet1, csig1 = calp1 * cbet1,
          ssig2 = sbet2, csig2 = calp2 * cbet2,
          k2 = Math::sq(calp0) * _ep2,
          eps = k2 / (2 * (1 + sqrt(1 + k2)) + k2),
          // Multiplier = a^2 * e^2 * cos(alpha0) * sin(alpha0).
          A4 = Math::sq(_a) * calp0 * salp0 * _e2;
        Math::norm(ssig1, csig1);
        Math::norm(ssig2, csig2);
        C4f(eps, Ca);
        real
          B41 = SinCosSeries(false, ssig1, csig1, Ca, nC4_),
          B42 = SinCosSeries(false, ssig2, csig2, Ca, nC4_);
        S12 = A4 * (B42 - B41);
      } else
        // Avoid problems with indeterminate sig1, sig2 on equator
        S12 = 0;

      if (!meridian && somg12 > 1) {
        somg12 = sin(omg12); comg12 = cos(omg12);
      }

      if (!meridian &&
          // omg12 < 3/4 * pi
          comg12 > -real(0.7071) &&     // Long difference not too big
          sbet2 - sbet1 < real(1.75)) { // Lat difference not too big
        // Use tan(Gamma/2) = tan(omg12/2)
        // * (tan(bet1/2)+tan(bet2/2))/(1+tan(bet1/2)*tan(bet2/2))
        // with tan(x/2) = sin(x)/(1+cos(x))
        real domg12 = 1 + comg12, dbet1 = 1 + cbet1, dbet2 = 1 + cbet2;
        alp12 = 2 * atan2( somg12 * ( sbet1 * dbet2 + sbet2 * dbet1 ),
                           domg12 * ( sbet1 * sbet2 + dbet1 * dbet2 ) );
      } else {
        // alp12 = alp2 - alp1, used in atan2 so no need to normalize
        real
          salp12 = salp2 * calp1 - calp2 * salp1,
          calp12 = calp2 * calp1 + salp2 * salp1;
        // The right thing appears to happen if alp1 = +/-180 and alp2 = 0, viz
        // salp12 = -0 and alp12 = -180.  However this depends on the sign
        // being attached to 0 correctly.  The following ensures the correct
        // behavior.
        if (salp12 == 0 && calp12 < 0) {
          salp12 = tiny_ * calp1;
          calp12 = -1;
        }
        alp12 = atan2(salp12, calp12);
      }
      S12 += _c2 * alp12;
      S12 *= swapp * lonsign * latsign;
      // Convert -0 to 0
      S12 += 0;
    }

    // Convert calp, salp to azimuth accounting for lonsign, swapp, latsign.
    if (swapp < 0) {
      swap(salp1, salp2);
      swap(calp1, calp2);
      if (outmask & GEODESICSCALE)
        swap(M12, M21);
    }

    salp1 *= swapp * lonsign; calp1 *= swapp * latsign;
    salp2 *= swapp * lonsign; calp2 *= swapp * latsign;

    // Returned value in [0, 180]
    return a12;
  }

  Math::real Geodesic::GenInverse(real lat1, real lon1, real lat2, real lon2,
                                  unsigned outmask,
                                  real& s12, real& azi1, real& azi2,
                                  real& m12, real& M12, real& M21,
                                  real& S12) const {
    outmask &= OUT_MASK;
    real salp1, calp1, salp2, calp2,
      a12 =  GenInverse(lat1, lon1, lat2, lon2,
                        outmask, s12, salp1, calp1, salp2, calp2,
                        m12, M12, M21, S12);
    if (outmask & AZIMUTH) {
      azi1 = Math::atan2d(salp1, calp1);
      azi2 = Math::atan2d(salp2, calp2);
    }
    return a12;
  }

  GeodesicLine Geodesic::InverseLine(real lat1, real lon1,
                                     real lat2, real lon2,
                                     unsigned caps) const {
    real t, salp1, calp1, salp2, calp2,
      a12 = GenInverse(lat1, lon1, lat2, lon2,
                       // No need to specify AZIMUTH here
                       0u, t, salp1, calp1, salp2, calp2,
                       t, t, t, t),
      azi1 = Math::atan2d(salp1, calp1);
    // Ensure that a12 can be converted to a distance
    if (caps & (OUT_MASK & DISTANCE_IN)) caps |= DISTANCE;
    return
      GeodesicLine(*this, lat1, lon1, azi1, salp1, calp1, caps, true, a12);
  }

  void Geodesic::Lengths(real eps, real sig12,
                         real ssig1, real csig1, real dn1,
                         real ssig2, real csig2, real dn2,
                         real cbet1, real cbet2, unsigned outmask,
                         real& s12b, real& m12b, real& m0,
                         real& M12, real& M21,
                         // Scratch area of the right size
                         real Ca[]) const {
    // Return m12b = (reduced length)/_b; also calculate s12b = distance/_b,
    // and m0 = coefficient of secular term in expression for reduced length.

    outmask &= OUT_MASK;
    // outmask & DISTANCE: set s12b
    // outmask & REDUCEDLENGTH: set m12b & m0
    // outmask & GEODESICSCALE: set M12 & M21

    real m0x = 0, J12 = 0, A1 = 0, A2 = 0;
    real Cb[nC2_ + 1];
    if (outmask & (DISTANCE | REDUCEDLENGTH | GEODESICSCALE)) {
      A1 = A1m1f(eps);
      C1f(eps, Ca);
      if (outmask & (REDUCEDLENGTH | GEODESICSCALE)) {
        A2 = A2m1f(eps);
        C2f(eps, Cb);
        m0x = A1 - A2;
        A2 = 1 + A2;
      }
      A1 = 1 + A1;
    }
    if (outmask & DISTANCE) {
      real B1 = SinCosSeries(true, ssig2, csig2, Ca, nC1_) -
        SinCosSeries(true, ssig1, csig1, Ca, nC1_);
      // Missing a factor of _b
      s12b = A1 * (sig12 + B1);
      if (outmask & (REDUCEDLENGTH | GEODESICSCALE)) {
        real B2 = SinCosSeries(true, ssig2, csig2, Cb, nC2_) -
          SinCosSeries(true, ssig1, csig1, Cb, nC2_);
        J12 = m0x * sig12 + (A1 * B1 - A2 * B2);
      }
    } else if (outmask & (REDUCEDLENGTH | GEODESICSCALE)) {
      // Assume here that nC1_ >= nC2_
      for (int l = 1; l <= nC2_; ++l)
        Cb[l] = A1 * Ca[l] - A2 * Cb[l];
      J12 = m0x * sig12 + (SinCosSeries(true, ssig2, csig2, Cb, nC2_) -
                           SinCosSeries(true, ssig1, csig1, Cb, nC2_));
    }
    if (outmask & REDUCEDLENGTH) {
      m0 = m0x;
      // Missing a factor of _b.
      // Add parens around (csig1 * ssig2) and (ssig1 * csig2) to ensure
      // accurate cancellation in the case of coincident points.
      m12b = dn2 * (csig1 * ssig2) - dn1 * (ssig1 * csig2) -
        csig1 * csig2 * J12;
    }
    if (outmask & GEODESICSCALE) {
      real csig12 = csig1 * csig2 + ssig1 * ssig2;
      real t = _ep2 * (cbet1 - cbet2) * (cbet1 + cbet2) / (dn1 + dn2);
      M12 = csig12 + (t * ssig2 - csig2 * J12) * ssig1 / dn1;
      M21 = csig12 - (t * ssig1 - csig1 * J12) * ssig2 / dn2;
    }
  }

  Math::real Geodesic::Astroid(real x, real y) {
    // Solve k^4+2*k^3-(x^2+y^2-1)*k^2-2*y^2*k-y^2 = 0 for positive root k.
    // This solution is adapted from Geocentric::Reverse.
    real k;
    real
      p = Math::sq(x),
      q = Math::sq(y),
      r = (p + q - 1) / 6;
    if ( !(q == 0 && r <= 0) ) {
      real
        // Avoid possible division by zero when r = 0 by multiplying equations
        // for s and t by r^3 and r, resp.
        S = p * q / 4,            // S = r^3 * s
        r2 = Math::sq(r),
        r3 = r * r2,
        // The discriminant of the quadratic equation for T3.  This is zero on
        // the evolute curve p^(1/3)+q^(1/3) = 1
        disc = S * (S + 2 * r3);
      real u = r;
      if (disc >= 0) {
        real T3 = S + r3;
        // Pick the sign on the sqrt to maximize abs(T3).  This minimizes loss
        // of precision due to cancellation.  The result is unchanged because
        // of the way the T is used in definition of u.
        T3 += T3 < 0 ? -sqrt(disc) : sqrt(disc); // T3 = (r * t)^3
        // N.B. cbrt always returns the real root.  cbrt(-8) = -2.
        real T = Math::cbrt(T3); // T = r * t
        // T can be zero; but then r2 / T -> 0.
        u += T + (T != 0 ? r2 / T : 0);
      } else {
        // T is complex, but the way u is defined the result is real.
        real ang = atan2(sqrt(-disc), -(S + r3));
        // There are three possible cube roots.  We choose the root which
        // avoids cancellation.  Note that disc < 0 implies that r < 0.
        u += 2 * r * cos(ang / 3);
      }
      real
        v = sqrt(Math::sq(u) + q),    // guaranteed positive
        // Avoid loss of accuracy when u < 0.
        uv = u < 0 ? q / (v - u) : u + v, // u+v, guaranteed positive
        w = (uv - q) / (2 * v);           // positive?
      // Rearrange expression for k to avoid loss of accuracy due to
      // subtraction.  Division by 0 not possible because uv > 0, w >= 0.
      k = uv / (sqrt(uv + Math::sq(w)) + w);   // guaranteed positive
    } else {               // q == 0 && r <= 0
      // y = 0 with |x| <= 1.  Handle this case directly.
      // for y small, positive root is k = abs(y)/sqrt(1-x^2)
      k = 0;
    }
    return k;
  }

  Math::real Geodesic::InverseStart(real sbet1, real cbet1, real dn1,
                                    real sbet2, real cbet2, real dn2,
                                    real lam12, real slam12, real clam12,
                                    real& salp1, real& calp1,
                                    // Only updated if return val >= 0
                                    real& salp2, real& calp2,
                                    // Only updated for short lines
                                    real& dnm,
                                    // Scratch area of the right size
                                    real Ca[]) const {
    // Return a starting point for Newton's method in salp1 and calp1 (function
    // value is -1).  If Newton's method doesn't need to be used, return also
    // salp2 and calp2 and function value is sig12.
    real
      sig12 = -1,               // Return value
      // bet12 = bet2 - bet1 in [0, pi); bet12a = bet2 + bet1 in (-pi, 0]
      sbet12 = sbet2 * cbet1 - cbet2 * sbet1,
      cbet12 = cbet2 * cbet1 + sbet2 * sbet1;
#if defined(__GNUC__) && __GNUC__ == 4 && \
  (__GNUC_MINOR__ < 6 || defined(__MINGW32__))
    // Volatile declaration needed to fix inverse cases
    // 88.202499451857 0 -88.202499451857 179.981022032992859592
    // 89.262080389218 0 -89.262080389218 179.992207982775375662
    // 89.333123580033 0 -89.333123580032997687 179.99295812360148422
    // which otherwise fail with g++ 4.4.4 x86 -O3 (Linux)
    // and g++ 4.4.0 (mingw) and g++ 4.6.1 (tdm mingw).
    real sbet12a;
    {
      GEOGRAPHICLIB_VOLATILE real xx1 = sbet2 * cbet1;
      GEOGRAPHICLIB_VOLATILE real xx2 = cbet2 * sbet1;
      sbet12a = xx1 + xx2;
    }
#else
    real sbet12a = sbet2 * cbet1 + cbet2 * sbet1;
#endif
    bool shortline = cbet12 >= 0 && sbet12 < real(0.5) &&
      cbet2 * lam12 < real(0.5);
    real somg12, comg12;
    if (shortline) {
      real sbetm2 = Math::sq(sbet1 + sbet2);
      // sin((bet1+bet2)/2)^2
      // =  (sbet1 + sbet2)^2 / ((sbet1 + sbet2)^2 + (cbet1 + cbet2)^2)
      sbetm2 /= sbetm2 + Math::sq(cbet1 + cbet2);
      dnm = sqrt(1 + _ep2 * sbetm2);
      real omg12 = lam12 / (_f1 * dnm);
      somg12 = sin(omg12); comg12 = cos(omg12);
    } else {
      somg12 = slam12; comg12 = clam12;
    }

    salp1 = cbet2 * somg12;
    calp1 = comg12 >= 0 ?
      sbet12 + cbet2 * sbet1 * Math::sq(somg12) / (1 + comg12) :
      sbet12a - cbet2 * sbet1 * Math::sq(somg12) / (1 - comg12);

    real
      ssig12 = Math::hypot(salp1, calp1),
      csig12 = sbet1 * sbet2 + cbet1 * cbet2 * comg12;

    if (shortline && ssig12 < _etol2) {
      // really short lines
      salp2 = cbet1 * somg12;
      calp2 = sbet12 - cbet1 * sbet2 *
        (comg12 >= 0 ? Math::sq(somg12) / (1 + comg12) : 1 - comg12);
      Math::norm(salp2, calp2);
      // Set return value
      sig12 = atan2(ssig12, csig12);
    } else if (abs(_n) > real(0.1) || // Skip astroid calc if too eccentric
               csig12 >= 0 ||
               ssig12 >= 6 * abs(_n) * Math::pi() * Math::sq(cbet1)) {
      // Nothing to do, zeroth order spherical approximation is OK
    } else {
      // Scale lam12 and bet2 to x, y coordinate system where antipodal point
      // is at origin and singular point is at y = 0, x = -1.
      real y, lamscale, betscale;
      // Volatile declaration needed to fix inverse case
      // 56.320923501171 0 -56.320923501171 179.664747671772880215
      // which otherwise fails with g++ 4.4.4 x86 -O3
      GEOGRAPHICLIB_VOLATILE real x;
      real lam12x = atan2(-slam12, -clam12); // lam12 - pi
      if (_f >= 0) {            // In fact f == 0 does not get here
        // x = dlong, y = dlat
        {
          real
            k2 = Math::sq(sbet1) * _ep2,
            eps = k2 / (2 * (1 + sqrt(1 + k2)) + k2);
          lamscale = _f * cbet1 * A3f(eps) * Math::pi();
        }
        betscale = lamscale * cbet1;

        x = lam12x / lamscale;
        y = sbet12a / betscale;
      } else {                  // _f < 0
        // x = dlat, y = dlong
        real
          cbet12a = cbet2 * cbet1 - sbet2 * sbet1,
          bet12a = atan2(sbet12a, cbet12a);
        real m12b, m0, dummy;
        // In the case of lon12 = 180, this repeats a calculation made in
        // Inverse.
        Lengths(_n, Math::pi() + bet12a,
                sbet1, -cbet1, dn1, sbet2, cbet2, dn2,
                cbet1, cbet2,
                REDUCEDLENGTH, dummy, m12b, m0, dummy, dummy, Ca);
        x = -1 + m12b / (cbet1 * cbet2 * m0 * Math::pi());
        betscale = x < -real(0.01) ? sbet12a / x :
          -_f * Math::sq(cbet1) * Math::pi();
        lamscale = betscale / cbet1;
        y = lam12x / lamscale;
      }

      if (y > -tol1_ && x > -1 - xthresh_) {
        // strip near cut
        // Need real(x) here to cast away the volatility of x for min/max
        if (_f >= 0) {
          salp1 = min(real(1), -real(x)); calp1 = - sqrt(1 - Math::sq(salp1));
        } else {
          calp1 = max(real(x > -tol1_ ? 0 : -1), real(x));
          salp1 = sqrt(1 - Math::sq(calp1));
        }
      } else {
        // Estimate alp1, by solving the astroid problem.
        //
        // Could estimate alpha1 = theta + pi/2, directly, i.e.,
        //   calp1 = y/k; salp1 = -x/(1+k);  for _f >= 0
        //   calp1 = x/(1+k); salp1 = -y/k;  for _f < 0 (need to check)
        //
        // However, it's better to estimate omg12 from astroid and use
        // spherical formula to compute alp1.  This reduces the mean number of
        // Newton iterations for astroid cases from 2.24 (min 0, max 6) to 2.12
        // (min 0 max 5).  The changes in the number of iterations are as
        // follows:
        //
        // change percent
        //    1       5
        //    0      78
        //   -1      16
        //   -2       0.6
        //   -3       0.04
        //   -4       0.002
        //
        // The histogram of iterations is (m = number of iterations estimating
        // alp1 directly, n = number of iterations estimating via omg12, total
        // number of trials = 148605):
        //
        //  iter    m      n
        //    0   148    186
        //    1 13046  13845
        //    2 93315 102225
        //    3 36189  32341
        //    4  5396      7
        //    5   455      1
        //    6    56      0
        //
        // Because omg12 is near pi, estimate work with omg12a = pi - omg12
        real k = Astroid(x, y);
        real
          omg12a = lamscale * ( _f >= 0 ? -x * k/(1 + k) : -y * (1 + k)/k );
        somg12 = sin(omg12a); comg12 = -cos(omg12a);
        // Update spherical estimate of alp1 using omg12 instead of lam12
        salp1 = cbet2 * somg12;
        calp1 = sbet12a - cbet2 * sbet1 * Math::sq(somg12) / (1 - comg12);
      }
    }
    // Sanity check on starting guess.  Backwards check allows NaN through.
    if (!(salp1 <= 0))
      Math::norm(salp1, calp1);
    else {
      salp1 = 1; calp1 = 0;
    }
    return sig12;
  }

  Math::real Geodesic::Lambda12(real sbet1, real cbet1, real dn1,
                                real sbet2, real cbet2, real dn2,
                                real salp1, real calp1,
                                real slam120, real clam120,
                                real& salp2, real& calp2,
                                real& sig12,
                                real& ssig1, real& csig1,
                                real& ssig2, real& csig2,
                                real& eps, real& domg12,
                                bool diffp, real& dlam12,
                                // Scratch area of the right size
                                real Ca[]) const {

    if (sbet1 == 0 && calp1 == 0)
      // Break degeneracy of equatorial line.  This case has already been
      // handled.
      calp1 = -tiny_;

    real
      // sin(alp1) * cos(bet1) = sin(alp0)
      salp0 = salp1 * cbet1,
      calp0 = Math::hypot(calp1, salp1 * sbet1); // calp0 > 0

    real somg1, comg1, somg2, comg2, somg12, comg12, lam12;
    // tan(bet1) = tan(sig1) * cos(alp1)
    // tan(omg1) = sin(alp0) * tan(sig1) = tan(omg1)=tan(alp1)*sin(bet1)
    ssig1 = sbet1; somg1 = salp0 * sbet1;
    csig1 = comg1 = calp1 * cbet1;
    Math::norm(ssig1, csig1);
    // Math::norm(somg1, comg1); -- don't need to normalize!

    // Enforce symmetries in the case abs(bet2) = -bet1.  Need to be careful
    // about this case, since this can yield singularities in the Newton
    // iteration.
    // sin(alp2) * cos(bet2) = sin(alp0)
    salp2 = cbet2 != cbet1 ? salp0 / cbet2 : salp1;
    // calp2 = sqrt(1 - sq(salp2))
    //       = sqrt(sq(calp0) - sq(sbet2)) / cbet2
    // and subst for calp0 and rearrange to give (choose positive sqrt
    // to give alp2 in [0, pi/2]).
    calp2 = cbet2 != cbet1 || abs(sbet2) != -sbet1 ?
      sqrt(Math::sq(calp1 * cbet1) +
           (cbet1 < -sbet1 ?
            (cbet2 - cbet1) * (cbet1 + cbet2) :
            (sbet1 - sbet2) * (sbet1 + sbet2))) / cbet2 :
      abs(calp1);
    // tan(bet2) = tan(sig2) * cos(alp2)
    // tan(omg2) = sin(alp0) * tan(sig2).
    ssig2 = sbet2; somg2 = salp0 * sbet2;
    csig2 = comg2 = calp2 * cbet2;
    Math::norm(ssig2, csig2);
    // Math::norm(somg2, comg2); -- don't need to normalize!

    // sig12 = sig2 - sig1, limit to [0, pi]
    sig12 = atan2(max(real(0), csig1 * ssig2 - ssig1 * csig2),
                               csig1 * csig2 + ssig1 * ssig2);

    // omg12 = omg2 - omg1, limit to [0, pi]
    somg12 = max(real(0), comg1 * somg2 - somg1 * comg2);
    comg12 =              comg1 * comg2 + somg1 * somg2;
    // eta = omg12 - lam120
    real eta = atan2(somg12 * clam120 - comg12 * slam120,
                     comg12 * clam120 + somg12 * slam120);
    real B312;
    real k2 = Math::sq(calp0) * _ep2;
    eps = k2 / (2 * (1 + sqrt(1 + k2)) + k2);
    C3f(eps, Ca);
    B312 = (SinCosSeries(true, ssig2, csig2, Ca, nC3_-1) -
            SinCosSeries(true, ssig1, csig1, Ca, nC3_-1));
    domg12 = -_f * A3f(eps) * salp0 * (sig12 + B312);
    lam12 = eta + domg12;

    if (diffp) {
      if (calp2 == 0)
        dlam12 = - 2 * _f1 * dn1 / sbet1;
      else {
        real dummy;
        Lengths(eps, sig12, ssig1, csig1, dn1, ssig2, csig2, dn2,
                cbet1, cbet2, REDUCEDLENGTH,
                dummy, dlam12, dummy, dummy, dummy, Ca);
        dlam12 *= _f1 / (calp2 * cbet2);
      }
    }

    return lam12;
  }

  Math::real Geodesic::A3f(real eps) const {
    // Evaluate A3
    return Math::polyval(nA3_ - 1, _A3x, eps);
  }

  void Geodesic::C3f(real eps, real c[]) const {
    // Evaluate C3 coeffs
    // Elements c[1] thru c[nC3_ - 1] are set
    real mult = 1;
    int o = 0;
    for (int l = 1; l < nC3_; ++l) { // l is index of C3[l]
      int m = nC3_ - l - 1;          // order of polynomial in eps
      mult *= eps;
      c[l] = mult * Math::polyval(m, _C3x + o, eps);
      o += m + 1;
    }
    // Post condition: o == nC3x_
  }

  void Geodesic::C4f(real eps, real c[]) const {
    // Evaluate C4 coeffs
    // Elements c[0] thru c[nC4_ - 1] are set
    real mult = 1;
    int o = 0;
    for (int l = 0; l < nC4_; ++l) { // l is index of C4[l]
      int m = nC4_ - l - 1;          // order of polynomial in eps
      c[l] = mult * Math::polyval(m, _C4x + o, eps);
      o += m + 1;
      mult *= eps;
    }
    // Post condition: o == nC4x_
  }

  // The static const coefficient arrays in the following functions are
  // generated by Maxima and give the coefficients of the Taylor expansions for
  // the geodesics.  The convention on the order of these coefficients is as
  // follows:
  //
  //   ascending order in the trigonometric expansion,
  //   then powers of eps in descending order,
  //   finally powers of n in descending order.
  //
  // (For some expansions, only a subset of levels occur.)  For each polynomial
  // of order n at the lowest level, the (n+1) coefficients of the polynomial
  // are followed by a divisor which is applied to the whole polynomial.  In
  // this way, the coefficients are expressible with no round off error.  The
  // sizes of the coefficient arrays are:
  //
  //   A1m1f, A2m1f            = floor(N/2) + 2
  //   C1f, C1pf, C2f, A3coeff = (N^2 + 7*N - 2*floor(N/2)) / 4
  //   C3coeff       = (N - 1) * (N^2 + 7*N - 2*floor(N/2)) / 8
  //   C4coeff       = N * (N + 1) * (N + 5) / 6
  //
  // where N = GEOGRAPHICLIB_GEODESIC_ORDER
  //         = nA1 = nA2 = nC1 = nC1p = nA3 = nC4

  // The scale factor A1-1 = mean value of (d/dsigma)I1 - 1
  Math::real Geodesic::A1m1f(real eps) {
    // Generated by Maxima on 2015-05-05 18:08:12-04:00
#if GEOGRAPHICLIB_GEODESIC_ORDER/2 == 1
    static const real coeff[] = {
      // (1-eps)*A1-1, polynomial in eps2 of order 1
      1, 0, 4,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER/2 == 2
    static const real coeff[] = {
      // (1-eps)*A1-1, polynomial in eps2 of order 2
      1, 16, 0, 64,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER/2 == 3
    static const real coeff[] = {
      // (1-eps)*A1-1, polynomial in eps2 of order 3
      1, 4, 64, 0, 256,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER/2 == 4
    static const real coeff[] = {
      // (1-eps)*A1-1, polynomial in eps2 of order 4
      25, 64, 256, 4096, 0, 16384,
    };
#else
#error "Bad value for GEOGRAPHICLIB_GEODESIC_ORDER"
#endif
    GEOGRAPHICLIB_STATIC_ASSERT(sizeof(coeff) / sizeof(real) == nA1_/2 + 2,
                                "Coefficient array size mismatch in A1m1f");
    int m = nA1_/2;
    real t = Math::polyval(m, coeff, Math::sq(eps)) / coeff[m + 1];
    return (t + eps) / (1 - eps);
  }

  // The coefficients C1[l] in the Fourier expansion of B1
  void Geodesic::C1f(real eps, real c[]) {
    // Generated by Maxima on 2015-05-05 18:08:12-04:00
#if GEOGRAPHICLIB_GEODESIC_ORDER == 3
    static const real coeff[] = {
      // C1[1]/eps^1, polynomial in eps2 of order 1
      3, -8, 16,
      // C1[2]/eps^2, polynomial in eps2 of order 0
      -1, 16,
      // C1[3]/eps^3, polynomial in eps2 of order 0
      -1, 48,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 4
    static const real coeff[] = {
      // C1[1]/eps^1, polynomial in eps2 of order 1
      3, -8, 16,
      // C1[2]/eps^2, polynomial in eps2 of order 1
      1, -2, 32,
      // C1[3]/eps^3, polynomial in eps2 of order 0
      -1, 48,
      // C1[4]/eps^4, polynomial in eps2 of order 0
      -5, 512,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 5
    static const real coeff[] = {
      // C1[1]/eps^1, polynomial in eps2 of order 2
      -1, 6, -16, 32,
      // C1[2]/eps^2, polynomial in eps2 of order 1
      1, -2, 32,
      // C1[3]/eps^3, polynomial in eps2 of order 1
      9, -16, 768,
      // C1[4]/eps^4, polynomial in eps2 of order 0
      -5, 512,
      // C1[5]/eps^5, polynomial in eps2 of order 0
      -7, 1280,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 6
    static const real coeff[] = {
      // C1[1]/eps^1, polynomial in eps2 of order 2
      -1, 6, -16, 32,
      // C1[2]/eps^2, polynomial in eps2 of order 2
      -9, 64, -128, 2048,
      // C1[3]/eps^3, polynomial in eps2 of order 1
      9, -16, 768,
      // C1[4]/eps^4, polynomial in eps2 of order 1
      3, -5, 512,
      // C1[5]/eps^5, polynomial in eps2 of order 0
      -7, 1280,
      // C1[6]/eps^6, polynomial in eps2 of order 0
      -7, 2048,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 7
    static const real coeff[] = {
      // C1[1]/eps^1, polynomial in eps2 of order 3
      19, -64, 384, -1024, 2048,
      // C1[2]/eps^2, polynomial in eps2 of order 2
      -9, 64, -128, 2048,
      // C1[3]/eps^3, polynomial in eps2 of order 2
      -9, 72, -128, 6144,
      // C1[4]/eps^4, polynomial in eps2 of order 1
      3, -5, 512,
      // C1[5]/eps^5, polynomial in eps2 of order 1
      35, -56, 10240,
      // C1[6]/eps^6, polynomial in eps2 of order 0
      -7, 2048,
      // C1[7]/eps^7, polynomial in eps2 of order 0
      -33, 14336,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 8
    static const real coeff[] = {
      // C1[1]/eps^1, polynomial in eps2 of order 3
      19, -64, 384, -1024, 2048,
      // C1[2]/eps^2, polynomial in eps2 of order 3
      7, -18, 128, -256, 4096,
      // C1[3]/eps^3, polynomial in eps2 of order 2
      -9, 72, -128, 6144,
      // C1[4]/eps^4, polynomial in eps2 of order 2
      -11, 96, -160, 16384,
      // C1[5]/eps^5, polynomial in eps2 of order 1
      35, -56, 10240,
      // C1[6]/eps^6, polynomial in eps2 of order 1
      9, -14, 4096,
      // C1[7]/eps^7, polynomial in eps2 of order 0
      -33, 14336,
      // C1[8]/eps^8, polynomial in eps2 of order 0
      -429, 262144,
    };
#else
#error "Bad value for GEOGRAPHICLIB_GEODESIC_ORDER"
#endif
    GEOGRAPHICLIB_STATIC_ASSERT(sizeof(coeff) / sizeof(real) ==
                                (nC1_*nC1_ + 7*nC1_ - 2*(nC1_/2)) / 4,
                                "Coefficient array size mismatch in C1f");
    real
      eps2 = Math::sq(eps),
      d = eps;
    int o = 0;
    for (int l = 1; l <= nC1_; ++l) { // l is index of C1p[l]
      int m = (nC1_ - l) / 2;         // order of polynomial in eps^2
      c[l] = d * Math::polyval(m, coeff + o, eps2) / coeff[o + m + 1];
      o += m + 2;
      d *= eps;
    }
    // Post condition: o == sizeof(coeff) / sizeof(real)
  }

  // The coefficients C1p[l] in the Fourier expansion of B1p
  void Geodesic::C1pf(real eps, real c[]) {
    // Generated by Maxima on 2015-05-05 18:08:12-04:00
#if GEOGRAPHICLIB_GEODESIC_ORDER == 3
    static const real coeff[] = {
      // C1p[1]/eps^1, polynomial in eps2 of order 1
      -9, 16, 32,
      // C1p[2]/eps^2, polynomial in eps2 of order 0
      5, 16,
      // C1p[3]/eps^3, polynomial in eps2 of order 0
      29, 96,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 4
    static const real coeff[] = {
      // C1p[1]/eps^1, polynomial in eps2 of order 1
      -9, 16, 32,
      // C1p[2]/eps^2, polynomial in eps2 of order 1
      -37, 30, 96,
      // C1p[3]/eps^3, polynomial in eps2 of order 0
      29, 96,
      // C1p[4]/eps^4, polynomial in eps2 of order 0
      539, 1536,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 5
    static const real coeff[] = {
      // C1p[1]/eps^1, polynomial in eps2 of order 2
      205, -432, 768, 1536,
      // C1p[2]/eps^2, polynomial in eps2 of order 1
      -37, 30, 96,
      // C1p[3]/eps^3, polynomial in eps2 of order 1
      -225, 116, 384,
      // C1p[4]/eps^4, polynomial in eps2 of order 0
      539, 1536,
      // C1p[5]/eps^5, polynomial in eps2 of order 0
      3467, 7680,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 6
    static const real coeff[] = {
      // C1p[1]/eps^1, polynomial in eps2 of order 2
      205, -432, 768, 1536,
      // C1p[2]/eps^2, polynomial in eps2 of order 2
      4005, -4736, 3840, 12288,
      // C1p[3]/eps^3, polynomial in eps2 of order 1
      -225, 116, 384,
      // C1p[4]/eps^4, polynomial in eps2 of order 1
      -7173, 2695, 7680,
      // C1p[5]/eps^5, polynomial in eps2 of order 0
      3467, 7680,
      // C1p[6]/eps^6, polynomial in eps2 of order 0
      38081, 61440,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 7
    static const real coeff[] = {
      // C1p[1]/eps^1, polynomial in eps2 of order 3
      -4879, 9840, -20736, 36864, 73728,
      // C1p[2]/eps^2, polynomial in eps2 of order 2
      4005, -4736, 3840, 12288,
      // C1p[3]/eps^3, polynomial in eps2 of order 2
      8703, -7200, 3712, 12288,
      // C1p[4]/eps^4, polynomial in eps2 of order 1
      -7173, 2695, 7680,
      // C1p[5]/eps^5, polynomial in eps2 of order 1
      -141115, 41604, 92160,
      // C1p[6]/eps^6, polynomial in eps2 of order 0
      38081, 61440,
      // C1p[7]/eps^7, polynomial in eps2 of order 0
      459485, 516096,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 8
    static const real coeff[] = {
      // C1p[1]/eps^1, polynomial in eps2 of order 3
      -4879, 9840, -20736, 36864, 73728,
      // C1p[2]/eps^2, polynomial in eps2 of order 3
      -86171, 120150, -142080, 115200, 368640,
      // C1p[3]/eps^3, polynomial in eps2 of order 2
      8703, -7200, 3712, 12288,
      // C1p[4]/eps^4, polynomial in eps2 of order 2
      1082857, -688608, 258720, 737280,
      // C1p[5]/eps^5, polynomial in eps2 of order 1
      -141115, 41604, 92160,
      // C1p[6]/eps^6, polynomial in eps2 of order 1
      -2200311, 533134, 860160,
      // C1p[7]/eps^7, polynomial in eps2 of order 0
      459485, 516096,
      // C1p[8]/eps^8, polynomial in eps2 of order 0
      109167851, 82575360,
    };
#else
#error "Bad value for GEOGRAPHICLIB_GEODESIC_ORDER"
#endif
    GEOGRAPHICLIB_STATIC_ASSERT(sizeof(coeff) / sizeof(real) ==
                                (nC1p_*nC1p_ + 7*nC1p_ - 2*(nC1p_/2)) / 4,
                                "Coefficient array size mismatch in C1pf");
    real
      eps2 = Math::sq(eps),
      d = eps;
    int o = 0;
    for (int l = 1; l <= nC1p_; ++l) { // l is index of C1p[l]
      int m = (nC1p_ - l) / 2;         // order of polynomial in eps^2
      c[l] = d * Math::polyval(m, coeff + o, eps2) / coeff[o + m + 1];
      o += m + 2;
      d *= eps;
    }
    // Post condition: o == sizeof(coeff) / sizeof(real)
  }

  // The scale factor A2-1 = mean value of (d/dsigma)I2 - 1
  Math::real Geodesic::A2m1f(real eps) {
    // Generated by Maxima on 2015-05-29 08:09:47-04:00
#if GEOGRAPHICLIB_GEODESIC_ORDER/2 == 1
    static const real coeff[] = {
      // (eps+1)*A2-1, polynomial in eps2 of order 1
      -3, 0, 4,
    };  // count = 3
#elif GEOGRAPHICLIB_GEODESIC_ORDER/2 == 2
    static const real coeff[] = {
      // (eps+1)*A2-1, polynomial in eps2 of order 2
      -7, -48, 0, 64,
    };  // count = 4
#elif GEOGRAPHICLIB_GEODESIC_ORDER/2 == 3
    static const real coeff[] = {
      // (eps+1)*A2-1, polynomial in eps2 of order 3
      -11, -28, -192, 0, 256,
    };  // count = 5
#elif GEOGRAPHICLIB_GEODESIC_ORDER/2 == 4
    static const real coeff[] = {
      // (eps+1)*A2-1, polynomial in eps2 of order 4
      -375, -704, -1792, -12288, 0, 16384,
    };  // count = 6
#else
#error "Bad value for GEOGRAPHICLIB_GEODESIC_ORDER"
#endif
    GEOGRAPHICLIB_STATIC_ASSERT(sizeof(coeff) / sizeof(real) == nA2_/2 + 2,
                                "Coefficient array size mismatch in A2m1f");
    int m = nA2_/2;
    real t = Math::polyval(m, coeff, Math::sq(eps)) / coeff[m + 1];
    return (t - eps) / (1 + eps);
  }

  // The coefficients C2[l] in the Fourier expansion of B2
  void Geodesic::C2f(real eps, real c[]) {
    // Generated by Maxima on 2015-05-05 18:08:12-04:00
#if GEOGRAPHICLIB_GEODESIC_ORDER == 3
    static const real coeff[] = {
      // C2[1]/eps^1, polynomial in eps2 of order 1
      1, 8, 16,
      // C2[2]/eps^2, polynomial in eps2 of order 0
      3, 16,
      // C2[3]/eps^3, polynomial in eps2 of order 0
      5, 48,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 4
    static const real coeff[] = {
      // C2[1]/eps^1, polynomial in eps2 of order 1
      1, 8, 16,
      // C2[2]/eps^2, polynomial in eps2 of order 1
      1, 6, 32,
      // C2[3]/eps^3, polynomial in eps2 of order 0
      5, 48,
      // C2[4]/eps^4, polynomial in eps2 of order 0
      35, 512,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 5
    static const real coeff[] = {
      // C2[1]/eps^1, polynomial in eps2 of order 2
      1, 2, 16, 32,
      // C2[2]/eps^2, polynomial in eps2 of order 1
      1, 6, 32,
      // C2[3]/eps^3, polynomial in eps2 of order 1
      15, 80, 768,
      // C2[4]/eps^4, polynomial in eps2 of order 0
      35, 512,
      // C2[5]/eps^5, polynomial in eps2 of order 0
      63, 1280,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 6
    static const real coeff[] = {
      // C2[1]/eps^1, polynomial in eps2 of order 2
      1, 2, 16, 32,
      // C2[2]/eps^2, polynomial in eps2 of order 2
      35, 64, 384, 2048,
      // C2[3]/eps^3, polynomial in eps2 of order 1
      15, 80, 768,
      // C2[4]/eps^4, polynomial in eps2 of order 1
      7, 35, 512,
      // C2[5]/eps^5, polynomial in eps2 of order 0
      63, 1280,
      // C2[6]/eps^6, polynomial in eps2 of order 0
      77, 2048,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 7
    static const real coeff[] = {
      // C2[1]/eps^1, polynomial in eps2 of order 3
      41, 64, 128, 1024, 2048,
      // C2[2]/eps^2, polynomial in eps2 of order 2
      35, 64, 384, 2048,
      // C2[3]/eps^3, polynomial in eps2 of order 2
      69, 120, 640, 6144,
      // C2[4]/eps^4, polynomial in eps2 of order 1
      7, 35, 512,
      // C2[5]/eps^5, polynomial in eps2 of order 1
      105, 504, 10240,
      // C2[6]/eps^6, polynomial in eps2 of order 0
      77, 2048,
      // C2[7]/eps^7, polynomial in eps2 of order 0
      429, 14336,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 8
    static const real coeff[] = {
      // C2[1]/eps^1, polynomial in eps2 of order 3
      41, 64, 128, 1024, 2048,
      // C2[2]/eps^2, polynomial in eps2 of order 3
      47, 70, 128, 768, 4096,
      // C2[3]/eps^3, polynomial in eps2 of order 2
      69, 120, 640, 6144,
      // C2[4]/eps^4, polynomial in eps2 of order 2
      133, 224, 1120, 16384,
      // C2[5]/eps^5, polynomial in eps2 of order 1
      105, 504, 10240,
      // C2[6]/eps^6, polynomial in eps2 of order 1
      33, 154, 4096,
      // C2[7]/eps^7, polynomial in eps2 of order 0
      429, 14336,
      // C2[8]/eps^8, polynomial in eps2 of order 0
      6435, 262144,
    };
#else
#error "Bad value for GEOGRAPHICLIB_GEODESIC_ORDER"
#endif
    GEOGRAPHICLIB_STATIC_ASSERT(sizeof(coeff) / sizeof(real) ==
                                (nC2_*nC2_ + 7*nC2_ - 2*(nC2_/2)) / 4,
                                "Coefficient array size mismatch in C2f");
    real
      eps2 = Math::sq(eps),
      d = eps;
    int o = 0;
    for (int l = 1; l <= nC2_; ++l) { // l is index of C2[l]
      int m = (nC2_ - l) / 2;         // order of polynomial in eps^2
      c[l] = d * Math::polyval(m, coeff + o, eps2) / coeff[o + m + 1];
      o += m + 2;
      d *= eps;
    }
    // Post condition: o == sizeof(coeff) / sizeof(real)
  }

  // The scale factor A3 = mean value of (d/dsigma)I3
  void Geodesic::A3coeff() {
    // Generated by Maxima on 2015-05-05 18:08:13-04:00
#if GEOGRAPHICLIB_GEODESIC_ORDER == 3
    static const real coeff[] = {
      // A3, coeff of eps^2, polynomial in n of order 0
      -1, 4,
      // A3, coeff of eps^1, polynomial in n of order 1
      1, -1, 2,
      // A3, coeff of eps^0, polynomial in n of order 0
      1, 1,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 4
    static const real coeff[] = {
      // A3, coeff of eps^3, polynomial in n of order 0
      -1, 16,
      // A3, coeff of eps^2, polynomial in n of order 1
      -1, -2, 8,
      // A3, coeff of eps^1, polynomial in n of order 1
      1, -1, 2,
      // A3, coeff of eps^0, polynomial in n of order 0
      1, 1,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 5
    static const real coeff[] = {
      // A3, coeff of eps^4, polynomial in n of order 0
      -3, 64,
      // A3, coeff of eps^3, polynomial in n of order 1
      -3, -1, 16,
      // A3, coeff of eps^2, polynomial in n of order 2
      3, -1, -2, 8,
      // A3, coeff of eps^1, polynomial in n of order 1
      1, -1, 2,
      // A3, coeff of eps^0, polynomial in n of order 0
      1, 1,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 6
    static const real coeff[] = {
      // A3, coeff of eps^5, polynomial in n of order 0
      -3, 128,
      // A3, coeff of eps^4, polynomial in n of order 1
      -2, -3, 64,
      // A3, coeff of eps^3, polynomial in n of order 2
      -1, -3, -1, 16,
      // A3, coeff of eps^2, polynomial in n of order 2
      3, -1, -2, 8,
      // A3, coeff of eps^1, polynomial in n of order 1
      1, -1, 2,
      // A3, coeff of eps^0, polynomial in n of order 0
      1, 1,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 7
    static const real coeff[] = {
      // A3, coeff of eps^6, polynomial in n of order 0
      -5, 256,
      // A3, coeff of eps^5, polynomial in n of order 1
      -5, -3, 128,
      // A3, coeff of eps^4, polynomial in n of order 2
      -10, -2, -3, 64,
      // A3, coeff of eps^3, polynomial in n of order 3
      5, -1, -3, -1, 16,
      // A3, coeff of eps^2, polynomial in n of order 2
      3, -1, -2, 8,
      // A3, coeff of eps^1, polynomial in n of order 1
      1, -1, 2,
      // A3, coeff of eps^0, polynomial in n of order 0
      1, 1,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 8
    static const real coeff[] = {
      // A3, coeff of eps^7, polynomial in n of order 0
      -25, 2048,
      // A3, coeff of eps^6, polynomial in n of order 1
      -15, -20, 1024,
      // A3, coeff of eps^5, polynomial in n of order 2
      -5, -10, -6, 256,
      // A3, coeff of eps^4, polynomial in n of order 3
      -5, -20, -4, -6, 128,
      // A3, coeff of eps^3, polynomial in n of order 3
      5, -1, -3, -1, 16,
      // A3, coeff of eps^2, polynomial in n of order 2
      3, -1, -2, 8,
      // A3, coeff of eps^1, polynomial in n of order 1
      1, -1, 2,
      // A3, coeff of eps^0, polynomial in n of order 0
      1, 1,
    };
#else
#error "Bad value for GEOGRAPHICLIB_GEODESIC_ORDER"
#endif
    GEOGRAPHICLIB_STATIC_ASSERT(sizeof(coeff) / sizeof(real) ==
                                (nA3_*nA3_ + 7*nA3_ - 2*(nA3_/2)) / 4,
                                "Coefficient array size mismatch in A3f");
    int o = 0, k = 0;
    for (int j = nA3_ - 1; j >= 0; --j) { // coeff of eps^j
      int m = min(nA3_ - j - 1, j);       // order of polynomial in n
      _A3x[k++] = Math::polyval(m, coeff + o, _n) / coeff[o + m + 1];
      o += m + 2;
    }
    // Post condition: o == sizeof(coeff) / sizeof(real) && k == nA3x_
  }

  // The coefficients C3[l] in the Fourier expansion of B3
  void Geodesic::C3coeff() {
    // Generated by Maxima on 2015-05-05 18:08:13-04:00
#if GEOGRAPHICLIB_GEODESIC_ORDER == 3
    static const real coeff[] = {
    // C3[1], coeff of eps^2, polynomial in n of order 0
    1, 8,
    // C3[1], coeff of eps^1, polynomial in n of order 1
    -1, 1, 4,
    // C3[2], coeff of eps^2, polynomial in n of order 0
    1, 16,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 4
    static const real coeff[] = {
    // C3[1], coeff of eps^3, polynomial in n of order 0
    3, 64,
    // C3[1], coeff of eps^2, polynomial in n of order 1
    // This is a case where a leading 0 term has been inserted to maintain the
    // pattern in the orders of the polynomials.
    0, 1, 8,
    // C3[1], coeff of eps^1, polynomial in n of order 1
    -1, 1, 4,
    // C3[2], coeff of eps^3, polynomial in n of order 0
    3, 64,
    // C3[2], coeff of eps^2, polynomial in n of order 1
    -3, 2, 32,
    // C3[3], coeff of eps^3, polynomial in n of order 0
    5, 192,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 5
    static const real coeff[] = {
    // C3[1], coeff of eps^4, polynomial in n of order 0
    5, 128,
    // C3[1], coeff of eps^3, polynomial in n of order 1
    3, 3, 64,
    // C3[1], coeff of eps^2, polynomial in n of order 2
    -1, 0, 1, 8,
    // C3[1], coeff of eps^1, polynomial in n of order 1
    -1, 1, 4,
    // C3[2], coeff of eps^4, polynomial in n of order 0
    3, 128,
    // C3[2], coeff of eps^3, polynomial in n of order 1
    -2, 3, 64,
    // C3[2], coeff of eps^2, polynomial in n of order 2
    1, -3, 2, 32,
    // C3[3], coeff of eps^4, polynomial in n of order 0
    3, 128,
    // C3[3], coeff of eps^3, polynomial in n of order 1
    -9, 5, 192,
    // C3[4], coeff of eps^4, polynomial in n of order 0
    7, 512,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 6
    static const real coeff[] = {
    // C3[1], coeff of eps^5, polynomial in n of order 0
    3, 128,
    // C3[1], coeff of eps^4, polynomial in n of order 1
    2, 5, 128,
    // C3[1], coeff of eps^3, polynomial in n of order 2
    -1, 3, 3, 64,
    // C3[1], coeff of eps^2, polynomial in n of order 2
    -1, 0, 1, 8,
    // C3[1], coeff of eps^1, polynomial in n of order 1
    -1, 1, 4,
    // C3[2], coeff of eps^5, polynomial in n of order 0
    5, 256,
    // C3[2], coeff of eps^4, polynomial in n of order 1
    1, 3, 128,
    // C3[2], coeff of eps^3, polynomial in n of order 2
    -3, -2, 3, 64,
    // C3[2], coeff of eps^2, polynomial in n of order 2
    1, -3, 2, 32,
    // C3[3], coeff of eps^5, polynomial in n of order 0
    7, 512,
    // C3[3], coeff of eps^4, polynomial in n of order 1
    -10, 9, 384,
    // C3[3], coeff of eps^3, polynomial in n of order 2
    5, -9, 5, 192,
    // C3[4], coeff of eps^5, polynomial in n of order 0
    7, 512,
    // C3[4], coeff of eps^4, polynomial in n of order 1
    -14, 7, 512,
    // C3[5], coeff of eps^5, polynomial in n of order 0
    21, 2560,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 7
    static const real coeff[] = {
    // C3[1], coeff of eps^6, polynomial in n of order 0
    21, 1024,
    // C3[1], coeff of eps^5, polynomial in n of order 1
    11, 12, 512,
    // C3[1], coeff of eps^4, polynomial in n of order 2
    2, 2, 5, 128,
    // C3[1], coeff of eps^3, polynomial in n of order 3
    -5, -1, 3, 3, 64,
    // C3[1], coeff of eps^2, polynomial in n of order 2
    -1, 0, 1, 8,
    // C3[1], coeff of eps^1, polynomial in n of order 1
    -1, 1, 4,
    // C3[2], coeff of eps^6, polynomial in n of order 0
    27, 2048,
    // C3[2], coeff of eps^5, polynomial in n of order 1
    1, 5, 256,
    // C3[2], coeff of eps^4, polynomial in n of order 2
    -9, 2, 6, 256,
    // C3[2], coeff of eps^3, polynomial in n of order 3
    2, -3, -2, 3, 64,
    // C3[2], coeff of eps^2, polynomial in n of order 2
    1, -3, 2, 32,
    // C3[3], coeff of eps^6, polynomial in n of order 0
    3, 256,
    // C3[3], coeff of eps^5, polynomial in n of order 1
    -4, 21, 1536,
    // C3[3], coeff of eps^4, polynomial in n of order 2
    -6, -10, 9, 384,
    // C3[3], coeff of eps^3, polynomial in n of order 3
    -1, 5, -9, 5, 192,
    // C3[4], coeff of eps^6, polynomial in n of order 0
    9, 1024,
    // C3[4], coeff of eps^5, polynomial in n of order 1
    -10, 7, 512,
    // C3[4], coeff of eps^4, polynomial in n of order 2
    10, -14, 7, 512,
    // C3[5], coeff of eps^6, polynomial in n of order 0
    9, 1024,
    // C3[5], coeff of eps^5, polynomial in n of order 1
    -45, 21, 2560,
    // C3[6], coeff of eps^6, polynomial in n of order 0
    11, 2048,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 8
    static const real coeff[] = {
    // C3[1], coeff of eps^7, polynomial in n of order 0
    243, 16384,
    // C3[1], coeff of eps^6, polynomial in n of order 1
    10, 21, 1024,
    // C3[1], coeff of eps^5, polynomial in n of order 2
    3, 11, 12, 512,
    // C3[1], coeff of eps^4, polynomial in n of order 3
    -2, 2, 2, 5, 128,
    // C3[1], coeff of eps^3, polynomial in n of order 3
    -5, -1, 3, 3, 64,
    // C3[1], coeff of eps^2, polynomial in n of order 2
    -1, 0, 1, 8,
    // C3[1], coeff of eps^1, polynomial in n of order 1
    -1, 1, 4,
    // C3[2], coeff of eps^7, polynomial in n of order 0
    187, 16384,
    // C3[2], coeff of eps^6, polynomial in n of order 1
    69, 108, 8192,
    // C3[2], coeff of eps^5, polynomial in n of order 2
    -2, 1, 5, 256,
    // C3[2], coeff of eps^4, polynomial in n of order 3
    -6, -9, 2, 6, 256,
    // C3[2], coeff of eps^3, polynomial in n of order 3
    2, -3, -2, 3, 64,
    // C3[2], coeff of eps^2, polynomial in n of order 2
    1, -3, 2, 32,
    // C3[3], coeff of eps^7, polynomial in n of order 0
    139, 16384,
    // C3[3], coeff of eps^6, polynomial in n of order 1
    -1, 12, 1024,
    // C3[3], coeff of eps^5, polynomial in n of order 2
    -77, -8, 42, 3072,
    // C3[3], coeff of eps^4, polynomial in n of order 3
    10, -6, -10, 9, 384,
    // C3[3], coeff of eps^3, polynomial in n of order 3
    -1, 5, -9, 5, 192,
    // C3[4], coeff of eps^7, polynomial in n of order 0
    127, 16384,
    // C3[4], coeff of eps^6, polynomial in n of order 1
    -43, 72, 8192,
    // C3[4], coeff of eps^5, polynomial in n of order 2
    -7, -40, 28, 2048,
    // C3[4], coeff of eps^4, polynomial in n of order 3
    -7, 20, -28, 14, 1024,
    // C3[5], coeff of eps^7, polynomial in n of order 0
    99, 16384,
    // C3[5], coeff of eps^6, polynomial in n of order 1
    -15, 9, 1024,
    // C3[5], coeff of eps^5, polynomial in n of order 2
    75, -90, 42, 5120,
    // C3[6], coeff of eps^7, polynomial in n of order 0
    99, 16384,
    // C3[6], coeff of eps^6, polynomial in n of order 1
    -99, 44, 8192,
    // C3[7], coeff of eps^7, polynomial in n of order 0
    429, 114688,
    };
#else
#error "Bad value for GEOGRAPHICLIB_GEODESIC_ORDER"
#endif
    GEOGRAPHICLIB_STATIC_ASSERT(sizeof(coeff) / sizeof(real) ==
                                ((nC3_-1)*(nC3_*nC3_ + 7*nC3_ - 2*(nC3_/2)))/8,
                                "Coefficient array size mismatch in C3coeff");
    int o = 0, k = 0;
    for (int l = 1; l < nC3_; ++l) {        // l is index of C3[l]
      for (int j = nC3_ - 1; j >= l; --j) { // coeff of eps^j
        int m = min(nC3_ - j - 1, j);       // order of polynomial in n
        _C3x[k++] = Math::polyval(m, coeff + o, _n) / coeff[o + m + 1];
        o += m + 2;
      }
    }
    // Post condition: o == sizeof(coeff) / sizeof(real) && k == nC3x_
  }

  void Geodesic::C4coeff() {
    // Generated by Maxima on 2015-05-05 18:08:13-04:00
#if GEOGRAPHICLIB_GEODESIC_ORDER == 3
    static const real coeff[] = {
      // C4[0], coeff of eps^2, polynomial in n of order 0
      -2, 105,
      // C4[0], coeff of eps^1, polynomial in n of order 1
      16, -7, 35,
      // C4[0], coeff of eps^0, polynomial in n of order 2
      8, -28, 70, 105,
      // C4[1], coeff of eps^2, polynomial in n of order 0
      -2, 105,
      // C4[1], coeff of eps^1, polynomial in n of order 1
      -16, 7, 315,
      // C4[2], coeff of eps^2, polynomial in n of order 0
      4, 525,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 4
    static const real coeff[] = {
      // C4[0], coeff of eps^3, polynomial in n of order 0
      11, 315,
      // C4[0], coeff of eps^2, polynomial in n of order 1
      -32, -6, 315,
      // C4[0], coeff of eps^1, polynomial in n of order 2
      -32, 48, -21, 105,
      // C4[0], coeff of eps^0, polynomial in n of order 3
      4, 24, -84, 210, 315,
      // C4[1], coeff of eps^3, polynomial in n of order 0
      -1, 105,
      // C4[1], coeff of eps^2, polynomial in n of order 1
      64, -18, 945,
      // C4[1], coeff of eps^1, polynomial in n of order 2
      32, -48, 21, 945,
      // C4[2], coeff of eps^3, polynomial in n of order 0
      -8, 1575,
      // C4[2], coeff of eps^2, polynomial in n of order 1
      -32, 12, 1575,
      // C4[3], coeff of eps^3, polynomial in n of order 0
      8, 2205,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 5
    static const real coeff[] = {
      // C4[0], coeff of eps^4, polynomial in n of order 0
      4, 1155,
      // C4[0], coeff of eps^3, polynomial in n of order 1
      -368, 121, 3465,
      // C4[0], coeff of eps^2, polynomial in n of order 2
      1088, -352, -66, 3465,
      // C4[0], coeff of eps^1, polynomial in n of order 3
      48, -352, 528, -231, 1155,
      // C4[0], coeff of eps^0, polynomial in n of order 4
      16, 44, 264, -924, 2310, 3465,
      // C4[1], coeff of eps^4, polynomial in n of order 0
      4, 1155,
      // C4[1], coeff of eps^3, polynomial in n of order 1
      80, -99, 10395,
      // C4[1], coeff of eps^2, polynomial in n of order 2
      -896, 704, -198, 10395,
      // C4[1], coeff of eps^1, polynomial in n of order 3
      -48, 352, -528, 231, 10395,
      // C4[2], coeff of eps^4, polynomial in n of order 0
      -8, 1925,
      // C4[2], coeff of eps^3, polynomial in n of order 1
      384, -88, 17325,
      // C4[2], coeff of eps^2, polynomial in n of order 2
      320, -352, 132, 17325,
      // C4[3], coeff of eps^4, polynomial in n of order 0
      -16, 8085,
      // C4[3], coeff of eps^3, polynomial in n of order 1
      -256, 88, 24255,
      // C4[4], coeff of eps^4, polynomial in n of order 0
      64, 31185,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 6
    static const real coeff[] = {
      // C4[0], coeff of eps^5, polynomial in n of order 0
      97, 15015,
      // C4[0], coeff of eps^4, polynomial in n of order 1
      1088, 156, 45045,
      // C4[0], coeff of eps^3, polynomial in n of order 2
      -224, -4784, 1573, 45045,
      // C4[0], coeff of eps^2, polynomial in n of order 3
      -10656, 14144, -4576, -858, 45045,
      // C4[0], coeff of eps^1, polynomial in n of order 4
      64, 624, -4576, 6864, -3003, 15015,
      // C4[0], coeff of eps^0, polynomial in n of order 5
      100, 208, 572, 3432, -12012, 30030, 45045,
      // C4[1], coeff of eps^5, polynomial in n of order 0
      1, 9009,
      // C4[1], coeff of eps^4, polynomial in n of order 1
      -2944, 468, 135135,
      // C4[1], coeff of eps^3, polynomial in n of order 2
      5792, 1040, -1287, 135135,
      // C4[1], coeff of eps^2, polynomial in n of order 3
      5952, -11648, 9152, -2574, 135135,
      // C4[1], coeff of eps^1, polynomial in n of order 4
      -64, -624, 4576, -6864, 3003, 135135,
      // C4[2], coeff of eps^5, polynomial in n of order 0
      8, 10725,
      // C4[2], coeff of eps^4, polynomial in n of order 1
      1856, -936, 225225,
      // C4[2], coeff of eps^3, polynomial in n of order 2
      -8448, 4992, -1144, 225225,
      // C4[2], coeff of eps^2, polynomial in n of order 3
      -1440, 4160, -4576, 1716, 225225,
      // C4[3], coeff of eps^5, polynomial in n of order 0
      -136, 63063,
      // C4[3], coeff of eps^4, polynomial in n of order 1
      1024, -208, 105105,
      // C4[3], coeff of eps^3, polynomial in n of order 2
      3584, -3328, 1144, 315315,
      // C4[4], coeff of eps^5, polynomial in n of order 0
      -128, 135135,
      // C4[4], coeff of eps^4, polynomial in n of order 1
      -2560, 832, 405405,
      // C4[5], coeff of eps^5, polynomial in n of order 0
      128, 99099,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 7
    static const real coeff[] = {
      // C4[0], coeff of eps^6, polynomial in n of order 0
      10, 9009,
      // C4[0], coeff of eps^5, polynomial in n of order 1
      -464, 291, 45045,
      // C4[0], coeff of eps^4, polynomial in n of order 2
      -4480, 1088, 156, 45045,
      // C4[0], coeff of eps^3, polynomial in n of order 3
      10736, -224, -4784, 1573, 45045,
      // C4[0], coeff of eps^2, polynomial in n of order 4
      1664, -10656, 14144, -4576, -858, 45045,
      // C4[0], coeff of eps^1, polynomial in n of order 5
      16, 64, 624, -4576, 6864, -3003, 15015,
      // C4[0], coeff of eps^0, polynomial in n of order 6
      56, 100, 208, 572, 3432, -12012, 30030, 45045,
      // C4[1], coeff of eps^6, polynomial in n of order 0
      10, 9009,
      // C4[1], coeff of eps^5, polynomial in n of order 1
      112, 15, 135135,
      // C4[1], coeff of eps^4, polynomial in n of order 2
      3840, -2944, 468, 135135,
      // C4[1], coeff of eps^3, polynomial in n of order 3
      -10704, 5792, 1040, -1287, 135135,
      // C4[1], coeff of eps^2, polynomial in n of order 4
      -768, 5952, -11648, 9152, -2574, 135135,
      // C4[1], coeff of eps^1, polynomial in n of order 5
      -16, -64, -624, 4576, -6864, 3003, 135135,
      // C4[2], coeff of eps^6, polynomial in n of order 0
      -4, 25025,
      // C4[2], coeff of eps^5, polynomial in n of order 1
      -1664, 168, 225225,
      // C4[2], coeff of eps^4, polynomial in n of order 2
      1664, 1856, -936, 225225,
      // C4[2], coeff of eps^3, polynomial in n of order 3
      6784, -8448, 4992, -1144, 225225,
      // C4[2], coeff of eps^2, polynomial in n of order 4
      128, -1440, 4160, -4576, 1716, 225225,
      // C4[3], coeff of eps^6, polynomial in n of order 0
      64, 315315,
      // C4[3], coeff of eps^5, polynomial in n of order 1
      1792, -680, 315315,
      // C4[3], coeff of eps^4, polynomial in n of order 2
      -2048, 1024, -208, 105105,
      // C4[3], coeff of eps^3, polynomial in n of order 3
      -1792, 3584, -3328, 1144, 315315,
      // C4[4], coeff of eps^6, polynomial in n of order 0
      -512, 405405,
      // C4[4], coeff of eps^5, polynomial in n of order 1
      2048, -384, 405405,
      // C4[4], coeff of eps^4, polynomial in n of order 2
      3072, -2560, 832, 405405,
      // C4[5], coeff of eps^6, polynomial in n of order 0
      -256, 495495,
      // C4[5], coeff of eps^5, polynomial in n of order 1
      -2048, 640, 495495,
      // C4[6], coeff of eps^6, polynomial in n of order 0
      512, 585585,
    };
#elif GEOGRAPHICLIB_GEODESIC_ORDER == 8
    static const real coeff[] = {
      // C4[0], coeff of eps^7, polynomial in n of order 0
      193, 85085,
      // C4[0], coeff of eps^6, polynomial in n of order 1
      4192, 850, 765765,
      // C4[0], coeff of eps^5, polynomial in n of order 2
      20960, -7888, 4947, 765765,
      // C4[0], coeff of eps^4, polynomial in n of order 3
      12480, -76160, 18496, 2652, 765765,
      // C4[0], coeff of eps^3, polynomial in n of order 4
      -154048, 182512, -3808, -81328, 26741, 765765,
      // C4[0], coeff of eps^2, polynomial in n of order 5
      3232, 28288, -181152, 240448, -77792, -14586, 765765,
      // C4[0], coeff of eps^1, polynomial in n of order 6
      96, 272, 1088, 10608, -77792, 116688, -51051, 255255,
      // C4[0], coeff of eps^0, polynomial in n of order 7
      588, 952, 1700, 3536, 9724, 58344, -204204, 510510, 765765,
      // C4[1], coeff of eps^7, polynomial in n of order 0
      349, 2297295,
      // C4[1], coeff of eps^6, polynomial in n of order 1
      -1472, 510, 459459,
      // C4[1], coeff of eps^5, polynomial in n of order 2
      -39840, 1904, 255, 2297295,
      // C4[1], coeff of eps^4, polynomial in n of order 3
      52608, 65280, -50048, 7956, 2297295,
      // C4[1], coeff of eps^3, polynomial in n of order 4
      103744, -181968, 98464, 17680, -21879, 2297295,
      // C4[1], coeff of eps^2, polynomial in n of order 5
      -1344, -13056, 101184, -198016, 155584, -43758, 2297295,
      // C4[1], coeff of eps^1, polynomial in n of order 6
      -96, -272, -1088, -10608, 77792, -116688, 51051, 2297295,
      // C4[2], coeff of eps^7, polynomial in n of order 0
      464, 1276275,
      // C4[2], coeff of eps^6, polynomial in n of order 1
      -928, -612, 3828825,
      // C4[2], coeff of eps^5, polynomial in n of order 2
      64256, -28288, 2856, 3828825,
      // C4[2], coeff of eps^4, polynomial in n of order 3
      -126528, 28288, 31552, -15912, 3828825,
      // C4[2], coeff of eps^3, polynomial in n of order 4
      -41472, 115328, -143616, 84864, -19448, 3828825,
      // C4[2], coeff of eps^2, polynomial in n of order 5
      160, 2176, -24480, 70720, -77792, 29172, 3828825,
      // C4[3], coeff of eps^7, polynomial in n of order 0
      -16, 97461,
      // C4[3], coeff of eps^6, polynomial in n of order 1
      -16384, 1088, 5360355,
      // C4[3], coeff of eps^5, polynomial in n of order 2
      -2560, 30464, -11560, 5360355,
      // C4[3], coeff of eps^4, polynomial in n of order 3
      35840, -34816, 17408, -3536, 1786785,
      // C4[3], coeff of eps^3, polynomial in n of order 4
      7168, -30464, 60928, -56576, 19448, 5360355,
      // C4[4], coeff of eps^7, polynomial in n of order 0
      128, 2297295,
      // C4[4], coeff of eps^6, polynomial in n of order 1
      26624, -8704, 6891885,
      // C4[4], coeff of eps^5, polynomial in n of order 2
      -77824, 34816, -6528, 6891885,
      // C4[4], coeff of eps^4, polynomial in n of order 3
      -32256, 52224, -43520, 14144, 6891885,
      // C4[5], coeff of eps^7, polynomial in n of order 0
      -6784, 8423415,
      // C4[5], coeff of eps^6, polynomial in n of order 1
      24576, -4352, 8423415,
      // C4[5], coeff of eps^5, polynomial in n of order 2
      45056, -34816, 10880, 8423415,
      // C4[6], coeff of eps^7, polynomial in n of order 0
      -1024, 3318315,
      // C4[6], coeff of eps^6, polynomial in n of order 1
      -28672, 8704, 9954945,
      // C4[7], coeff of eps^7, polynomial in n of order 0
      1024, 1640925,
    };
#else
#error "Bad value for GEOGRAPHICLIB_GEODESIC_ORDER"
#endif
    GEOGRAPHICLIB_STATIC_ASSERT(sizeof(coeff) / sizeof(real) ==
                                (nC4_ * (nC4_ + 1) * (nC4_ + 5)) / 6,
                                "Coefficient array size mismatch in C4coeff");
    int o = 0, k = 0;
    for (int l = 0; l < nC4_; ++l) {        // l is index of C4[l]
      for (int j = nC4_ - 1; j >= l; --j) { // coeff of eps^j
        int m = nC4_ - j - 1;               // order of polynomial in n
        _C4x[k++] = Math::polyval(m, coeff + o, _n) / coeff[o + m + 1];
        o += m + 2;
      }
    }
    // Post condition: o == sizeof(coeff) / sizeof(real) && k == nC4x_
  }

} // namespace GeographicLib
