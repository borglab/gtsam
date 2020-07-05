/**
 * \file GeodesicLine.cpp
 * \brief Implementation for GeographicLib::GeodesicLine class
 *
 * Copyright (c) Charles Karney (2009-2016) <charles@karney.com> and licensed
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

#include <GeographicLib/GeodesicLine.hpp>

namespace GeographicLib {

  using namespace std;

  void GeodesicLine::LineInit(const Geodesic& g,
                              real lat1, real lon1,
                              real azi1, real salp1, real calp1,
                              unsigned caps) {
    tiny_ = g.tiny_;
    _lat1 = Math::LatFix(lat1);
    _lon1 = lon1;
    _azi1 = azi1;
    _salp1 = salp1;
    _calp1 = calp1;
    _a = g._a;
    _f = g._f;
    _b = g._b;
    _c2 = g._c2;
    _f1 = g._f1;
    // Always allow latitude and azimuth and unrolling of longitude
    _caps = caps | LATITUDE | AZIMUTH | LONG_UNROLL;

    real cbet1, sbet1;
    Math::sincosd(Math::AngRound(_lat1), sbet1, cbet1); sbet1 *= _f1;
    // Ensure cbet1 = +epsilon at poles
    Math::norm(sbet1, cbet1); cbet1 = max(tiny_, cbet1);
    _dn1 = sqrt(1 + g._ep2 * Math::sq(sbet1));

    // Evaluate alp0 from sin(alp1) * cos(bet1) = sin(alp0),
    _salp0 = _salp1 * cbet1; // alp0 in [0, pi/2 - |bet1|]
    // Alt: calp0 = hypot(sbet1, calp1 * cbet1).  The following
    // is slightly better (consider the case salp1 = 0).
    _calp0 = Math::hypot(_calp1, _salp1 * sbet1);
    // Evaluate sig with tan(bet1) = tan(sig1) * cos(alp1).
    // sig = 0 is nearest northward crossing of equator.
    // With bet1 = 0, alp1 = pi/2, we have sig1 = 0 (equatorial line).
    // With bet1 =  pi/2, alp1 = -pi, sig1 =  pi/2
    // With bet1 = -pi/2, alp1 =  0 , sig1 = -pi/2
    // Evaluate omg1 with tan(omg1) = sin(alp0) * tan(sig1).
    // With alp0 in (0, pi/2], quadrants for sig and omg coincide.
    // No atan2(0,0) ambiguity at poles since cbet1 = +epsilon.
    // With alp0 = 0, omg1 = 0 for alp1 = 0, omg1 = pi for alp1 = pi.
    _ssig1 = sbet1; _somg1 = _salp0 * sbet1;
    _csig1 = _comg1 = sbet1 != 0 || _calp1 != 0 ? cbet1 * _calp1 : 1;
    Math::norm(_ssig1, _csig1); // sig1 in (-pi, pi]
    // Math::norm(_somg1, _comg1); -- don't need to normalize!

    _k2 = Math::sq(_calp0) * g._ep2;
    real eps = _k2 / (2 * (1 + sqrt(1 + _k2)) + _k2);

    if (_caps & CAP_C1) {
      _A1m1 = Geodesic::A1m1f(eps);
      Geodesic::C1f(eps, _C1a);
      _B11 = Geodesic::SinCosSeries(true, _ssig1, _csig1, _C1a, nC1_);
      real s = sin(_B11), c = cos(_B11);
      // tau1 = sig1 + B11
      _stau1 = _ssig1 * c + _csig1 * s;
      _ctau1 = _csig1 * c - _ssig1 * s;
      // Not necessary because C1pa reverts C1a
      //    _B11 = -SinCosSeries(true, _stau1, _ctau1, _C1pa, nC1p_);
    }

    if (_caps & CAP_C1p)
      Geodesic::C1pf(eps, _C1pa);

    if (_caps & CAP_C2) {
      _A2m1 = Geodesic::A2m1f(eps);
      Geodesic::C2f(eps, _C2a);
      _B21 = Geodesic::SinCosSeries(true, _ssig1, _csig1, _C2a, nC2_);
    }

    if (_caps & CAP_C3) {
      g.C3f(eps, _C3a);
      _A3c = -_f * _salp0 * g.A3f(eps);
      _B31 = Geodesic::SinCosSeries(true, _ssig1, _csig1, _C3a, nC3_-1);
    }

    if (_caps & CAP_C4) {
      g.C4f(eps, _C4a);
      // Multiplier = a^2 * e^2 * cos(alpha0) * sin(alpha0)
      _A4 = Math::sq(_a) * _calp0 * _salp0 * g._e2;
      _B41 = Geodesic::SinCosSeries(false, _ssig1, _csig1, _C4a, nC4_);
    }

    _a13 = _s13 = Math::NaN();
  }

  GeodesicLine::GeodesicLine(const Geodesic& g,
                             real lat1, real lon1, real azi1,
                             unsigned caps) {
    azi1 = Math::AngNormalize(azi1);
    real salp1, calp1;
    // Guard against underflow in salp0.  Also -0 is converted to +0.
    Math::sincosd(Math::AngRound(azi1), salp1, calp1);
    LineInit(g, lat1, lon1, azi1, salp1, calp1, caps);
  }

  GeodesicLine::GeodesicLine(const Geodesic& g,
                             real lat1, real lon1,
                             real azi1, real salp1, real calp1,
                             unsigned caps, bool arcmode, real s13_a13) {
    LineInit(g, lat1, lon1, azi1, salp1, calp1, caps);
    GenSetDistance(arcmode, s13_a13);
  }

  Math::real GeodesicLine::GenPosition(bool arcmode, real s12_a12,
                                       unsigned outmask,
                                       real& lat2, real& lon2, real& azi2,
                                       real& s12, real& m12,
                                       real& M12, real& M21,
                                       real& S12) const {
    outmask &= _caps & OUT_MASK;
    if (!( Init() && (arcmode || (_caps & (OUT_MASK & DISTANCE_IN))) ))
      // Uninitialized or impossible distance calculation requested
      return Math::NaN();

    // Avoid warning about uninitialized B12.
    real sig12, ssig12, csig12, B12 = 0, AB1 = 0;
    if (arcmode) {
      // Interpret s12_a12 as spherical arc length
      sig12 = s12_a12 * Math::degree();
      Math::sincosd(s12_a12, ssig12, csig12);
    } else {
      // Interpret s12_a12 as distance
      real
        tau12 = s12_a12 / (_b * (1 + _A1m1)),
        s = sin(tau12),
        c = cos(tau12);
      // tau2 = tau1 + tau12
      B12 = - Geodesic::SinCosSeries(true,
                                     _stau1 * c + _ctau1 * s,
                                     _ctau1 * c - _stau1 * s,
                                     _C1pa, nC1p_);
      sig12 = tau12 - (B12 - _B11);
      ssig12 = sin(sig12); csig12 = cos(sig12);
      if (abs(_f) > 0.01) {
        // Reverted distance series is inaccurate for |f| > 1/100, so correct
        // sig12 with 1 Newton iteration.  The following table shows the
        // approximate maximum error for a = WGS_a() and various f relative to
        // GeodesicExact.
        //     erri = the error in the inverse solution (nm)
        //     errd = the error in the direct solution (series only) (nm)
        //     errda = the error in the direct solution
        //             (series + 1 Newton) (nm)
        //
        //       f     erri  errd errda
        //     -1/5    12e6 1.2e9  69e6
        //     -1/10  123e3  12e6 765e3
        //     -1/20   1110 108e3  7155
        //     -1/50  18.63 200.9 27.12
        //     -1/100 18.63 23.78 23.37
        //     -1/150 18.63 21.05 20.26
        //      1/150 22.35 24.73 25.83
        //      1/100 22.35 25.03 25.31
        //      1/50  29.80 231.9 30.44
        //      1/20   5376 146e3  10e3
        //      1/10  829e3  22e6 1.5e6
        //      1/5   157e6 3.8e9 280e6
        real
          ssig2 = _ssig1 * csig12 + _csig1 * ssig12,
          csig2 = _csig1 * csig12 - _ssig1 * ssig12;
        B12 = Geodesic::SinCosSeries(true, ssig2, csig2, _C1a, nC1_);
        real serr = (1 + _A1m1) * (sig12 + (B12 - _B11)) - s12_a12 / _b;
        sig12 = sig12 - serr / sqrt(1 + _k2 * Math::sq(ssig2));
        ssig12 = sin(sig12); csig12 = cos(sig12);
        // Update B12 below
      }
    }

    real ssig2, csig2, sbet2, cbet2, salp2, calp2;
    // sig2 = sig1 + sig12
    ssig2 = _ssig1 * csig12 + _csig1 * ssig12;
    csig2 = _csig1 * csig12 - _ssig1 * ssig12;
    real dn2 = sqrt(1 + _k2 * Math::sq(ssig2));
    if (outmask & (DISTANCE | REDUCEDLENGTH | GEODESICSCALE)) {
      if (arcmode || abs(_f) > 0.01)
        B12 = Geodesic::SinCosSeries(true, ssig2, csig2, _C1a, nC1_);
      AB1 = (1 + _A1m1) * (B12 - _B11);
    }
    // sin(bet2) = cos(alp0) * sin(sig2)
    sbet2 = _calp0 * ssig2;
    // Alt: cbet2 = hypot(csig2, salp0 * ssig2);
    cbet2 = Math::hypot(_salp0, _calp0 * csig2);
    if (cbet2 == 0)
      // I.e., salp0 = 0, csig2 = 0.  Break the degeneracy in this case
      cbet2 = csig2 = tiny_;
    // tan(alp0) = cos(sig2)*tan(alp2)
    salp2 = _salp0; calp2 = _calp0 * csig2; // No need to normalize

    if (outmask & DISTANCE)
      s12 = arcmode ? _b * ((1 + _A1m1) * sig12 + AB1) : s12_a12;

    if (outmask & LONGITUDE) {
      // tan(omg2) = sin(alp0) * tan(sig2)
      real somg2 = _salp0 * ssig2, comg2 = csig2,  // No need to normalize
        E = Math::copysign(real(1), _salp0);       // east-going?
      // omg12 = omg2 - omg1
      real omg12 = outmask & LONG_UNROLL
        ? E * (sig12
               - (atan2(    ssig2, csig2) - atan2(    _ssig1, _csig1))
               + (atan2(E * somg2, comg2) - atan2(E * _somg1, _comg1)))
        : atan2(somg2 * _comg1 - comg2 * _somg1,
                comg2 * _comg1 + somg2 * _somg1);
      real lam12 = omg12 + _A3c *
        ( sig12 + (Geodesic::SinCosSeries(true, ssig2, csig2, _C3a, nC3_-1)
                   - _B31));
      real lon12 = lam12 / Math::degree();
      lon2 = outmask & LONG_UNROLL ? _lon1 + lon12 :
        Math::AngNormalize(Math::AngNormalize(_lon1) +
                           Math::AngNormalize(lon12));
    }

    if (outmask & LATITUDE)
      lat2 = Math::atan2d(sbet2, _f1 * cbet2);

    if (outmask & AZIMUTH)
      azi2 = Math::atan2d(salp2, calp2);

    if (outmask & (REDUCEDLENGTH | GEODESICSCALE)) {
      real
        B22 = Geodesic::SinCosSeries(true, ssig2, csig2, _C2a, nC2_),
        AB2 = (1 + _A2m1) * (B22 - _B21),
        J12 = (_A1m1 - _A2m1) * sig12 + (AB1 - AB2);
      if (outmask & REDUCEDLENGTH)
        // Add parens around (_csig1 * ssig2) and (_ssig1 * csig2) to ensure
        // accurate cancellation in the case of coincident points.
        m12 = _b * ((dn2 * (_csig1 * ssig2) - _dn1 * (_ssig1 * csig2))
                    - _csig1 * csig2 * J12);
      if (outmask & GEODESICSCALE) {
        real t = _k2 * (ssig2 - _ssig1) * (ssig2 + _ssig1) / (_dn1 + dn2);
        M12 = csig12 + (t *  ssig2 -  csig2 * J12) * _ssig1 / _dn1;
        M21 = csig12 - (t * _ssig1 - _csig1 * J12) *  ssig2 /  dn2;
      }
    }

    if (outmask & AREA) {
      real
        B42 = Geodesic::SinCosSeries(false, ssig2, csig2, _C4a, nC4_);
      real salp12, calp12;
      if (_calp0 == 0 || _salp0 == 0) {
        // alp12 = alp2 - alp1, used in atan2 so no need to normalize
        salp12 = salp2 * _calp1 - calp2 * _salp1;
        calp12 = calp2 * _calp1 + salp2 * _salp1;
        // We used to include here some patch up code that purported to deal
        // with nearly meridional geodesics properly.  However, this turned out
        // to be wrong once _salp1 = -0 was allowed (via
        // Geodesic::InverseLine).  In fact, the calculation of {s,c}alp12
        // was already correct (following the IEEE rules for handling signed
        // zeros).  So the patch up code was unnecessary (as well as
        // dangerous).
      } else {
        // tan(alp) = tan(alp0) * sec(sig)
        // tan(alp2-alp1) = (tan(alp2) -tan(alp1)) / (tan(alp2)*tan(alp1)+1)
        // = calp0 * salp0 * (csig1-csig2) / (salp0^2 + calp0^2 * csig1*csig2)
        // If csig12 > 0, write
        //   csig1 - csig2 = ssig12 * (csig1 * ssig12 / (1 + csig12) + ssig1)
        // else
        //   csig1 - csig2 = csig1 * (1 - csig12) + ssig12 * ssig1
        // No need to normalize
        salp12 = _calp0 * _salp0 *
          (csig12 <= 0 ? _csig1 * (1 - csig12) + ssig12 * _ssig1 :
           ssig12 * (_csig1 * ssig12 / (1 + csig12) + _ssig1));
        calp12 = Math::sq(_salp0) + Math::sq(_calp0) * _csig1 * csig2;
      }
      S12 = _c2 * atan2(salp12, calp12) + _A4 * (B42 - _B41);
    }

    return arcmode ? s12_a12 : sig12 / Math::degree();
  }

  void GeodesicLine::SetDistance(real s13) {
    _s13 = s13;
    real t;
    // This will set _a13 to NaN if the GeodesicLine doesn't have the
    // DISTANCE_IN capability.
    _a13 = GenPosition(false, _s13, 0u, t, t, t, t, t, t, t, t);
  }

  void GeodesicLine::SetArc(real a13) {
    _a13 = a13;
    // In case the GeodesicLine doesn't have the DISTANCE capability.
    _s13 = Math::NaN();
    real t;
    GenPosition(true, _a13, DISTANCE, t, t, t, _s13, t, t, t, t);
  }

  void GeodesicLine::GenSetDistance(bool arcmode, real s13_a13) {
    arcmode ? SetArc(s13_a13) : SetDistance(s13_a13);
  }

} // namespace GeographicLib
