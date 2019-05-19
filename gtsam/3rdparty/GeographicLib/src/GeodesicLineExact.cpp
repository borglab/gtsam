/**
 * \file GeodesicLineExact.cpp
 * \brief Implementation for GeographicLib::GeodesicLineExact class
 *
 * Copyright (c) Charles Karney (2012-2016) <charles@karney.com> and licensed
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

#include <GeographicLib/GeodesicLineExact.hpp>

namespace GeographicLib {

  using namespace std;

  void GeodesicLineExact::LineInit(const GeodesicExact& g,
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
    _e2 = g._e2;
    // Always allow latitude and azimuth and unrolling of longitude
    _caps = caps | LATITUDE | AZIMUTH | LONG_UNROLL;

    real cbet1, sbet1;
    Math::sincosd(Math::AngRound(_lat1), sbet1, cbet1); sbet1 *= _f1;
    // Ensure cbet1 = +epsilon at poles
    Math::norm(sbet1, cbet1); cbet1 = max(tiny_, cbet1);
    _dn1 = (_f >= 0 ? sqrt(1 + g._ep2 * Math::sq(sbet1)) :
            sqrt(1 - _e2 * Math::sq(cbet1)) / _f1);

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
    // Without normalization we have schi1 = somg1.
    _cchi1 = _f1 * _dn1 * _comg1;
    Math::norm(_ssig1, _csig1); // sig1 in (-pi, pi]
    // Math::norm(_somg1, _comg1); -- don't need to normalize!
    // Math::norm(_schi1, _cchi1); -- don't need to normalize!

    _k2 = Math::sq(_calp0) * g._ep2;
    _E.Reset(-_k2, -g._ep2, 1 + _k2, 1 + g._ep2);

    if (_caps & CAP_E) {
      _E0 = _E.E() / (Math::pi() / 2);
      _E1 = _E.deltaE(_ssig1, _csig1, _dn1);
      real s = sin(_E1), c = cos(_E1);
      // tau1 = sig1 + B11
      _stau1 = _ssig1 * c + _csig1 * s;
      _ctau1 = _csig1 * c - _ssig1 * s;
      // Not necessary because Einv inverts E
      //    _E1 = -_E.deltaEinv(_stau1, _ctau1);
    }

    if (_caps & CAP_D) {
      _D0 = _E.D() / (Math::pi() / 2);
      _D1 = _E.deltaD(_ssig1, _csig1, _dn1);
    }

    if (_caps & CAP_H) {
      _H0 = _E.H() / (Math::pi() / 2);
      _H1 = _E.deltaH(_ssig1, _csig1, _dn1);
    }

    if (_caps & CAP_C4) {
      real eps = _k2 / (2 * (1 + sqrt(1 + _k2)) + _k2);
      g.C4f(eps, _C4a);
      // Multiplier = a^2 * e^2 * cos(alpha0) * sin(alpha0)
      _A4 = Math::sq(_a) * _calp0 * _salp0 * _e2;
      _B41 = GeodesicExact::CosSeries(_ssig1, _csig1, _C4a, nC4_);
    }

    _a13 = _s13 = Math::NaN();
  }

  GeodesicLineExact::GeodesicLineExact(const GeodesicExact& g,
                                       real lat1, real lon1, real azi1,
                                       unsigned caps) {
    azi1 = Math::AngNormalize(azi1);
    real salp1, calp1;
    // Guard against underflow in salp0.  Also -0 is converted to +0.
    Math::sincosd(Math::AngRound(azi1), salp1, calp1);
    LineInit(g, lat1, lon1, azi1, salp1, calp1, caps);
  }

  GeodesicLineExact::GeodesicLineExact(const GeodesicExact& g,
                                       real lat1, real lon1,
                                       real azi1, real salp1, real calp1,
                                       unsigned caps,
                                       bool arcmode, real s13_a13) {
    LineInit(g, lat1, lon1, azi1, salp1, calp1, caps);
    GenSetDistance(arcmode, s13_a13);
  }

  Math::real GeodesicLineExact::GenPosition(bool arcmode, real s12_a12,
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
    real sig12, ssig12, csig12, E2 = 0, AB1 = 0;
    if (arcmode) {
      // Interpret s12_a12 as spherical arc length
      sig12 = s12_a12 * Math::degree();
      real s12a = abs(s12_a12);
      s12a -= 180 * floor(s12a / 180);
      ssig12 = s12a ==  0 ? 0 : sin(sig12);
      csig12 = s12a == 90 ? 0 : cos(sig12);
    } else {
      // Interpret s12_a12 as distance
      real
        tau12 = s12_a12 / (_b * _E0),
        s = sin(tau12),
        c = cos(tau12);
      // tau2 = tau1 + tau12
      E2 = - _E.deltaEinv(_stau1 * c + _ctau1 * s, _ctau1 * c - _stau1 * s);
      sig12 = tau12 - (E2 - _E1);
      ssig12 = sin(sig12);
      csig12 = cos(sig12);
    }

    real ssig2, csig2, sbet2, cbet2, salp2, calp2;
    // sig2 = sig1 + sig12
    ssig2 = _ssig1 * csig12 + _csig1 * ssig12;
    csig2 = _csig1 * csig12 - _ssig1 * ssig12;
    real dn2 = _E.Delta(ssig2, csig2);
    if (outmask & (DISTANCE | REDUCEDLENGTH | GEODESICSCALE)) {
      if (arcmode) {
        E2 = _E.deltaE(ssig2, csig2, dn2);
      }
      AB1 = _E0 * (E2 - _E1);
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
      s12 = arcmode ? _b * (_E0 * sig12 + AB1) : s12_a12;

    if (outmask & LONGITUDE) {
      real somg2 = _salp0 * ssig2, comg2 = csig2,  // No need to normalize
        E = Math::copysign(real(1), _salp0);       // east-going?
      // Without normalization we have schi2 = somg2.
      real cchi2 =  _f1 * dn2 *  comg2;
      real chi12 = outmask & LONG_UNROLL
        ? E * (sig12
               - (atan2(    ssig2, csig2) - atan2(    _ssig1, _csig1))
               + (atan2(E * somg2, cchi2) - atan2(E * _somg1, _cchi1)))
        : atan2(somg2 * _cchi1 - cchi2 * _somg1,
                cchi2 * _cchi1 + somg2 * _somg1);
      real lam12 = chi12 -
        _e2/_f1 * _salp0 * _H0 *
        (sig12 + (_E.deltaH(ssig2, csig2, dn2) - _H1));
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
      real J12 = _k2 * _D0 * (sig12 + (_E.deltaD(ssig2, csig2, dn2) - _D1));
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
        B42 = GeodesicExact::CosSeries(ssig2, csig2, _C4a, nC4_);
      real salp12, calp12;
      if (_calp0 == 0 || _salp0 == 0) {
        // alp12 = alp2 - alp1, used in atan2 so no need to normalize
        salp12 = salp2 * _calp1 - calp2 * _salp1;
        calp12 = calp2 * _calp1 + salp2 * _salp1;
        // We used to include here some patch up code that purported to deal
        // with nearly meridional geodesics properly.  However, this turned out
        // to be wrong once _salp1 = -0 was allowed (via
        // GeodesicExact::InverseLine).  In fact, the calculation of {s,c}alp12
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

  void GeodesicLineExact::SetDistance(real s13) {
    _s13 = s13;
    real t;
    // This will set _a13 to NaN if the GeodesicLineExact doesn't have the
    // DISTANCE_IN capability.
    _a13 = GenPosition(false, _s13, 0u, t, t, t, t, t, t, t, t);
  }

  void GeodesicLineExact::SetArc(real a13) {
    _a13 = a13;
    // In case the GeodesicLineExact doesn't have the DISTANCE capability.
    _s13 = Math::NaN();
    real t;
    GenPosition(true, _a13, DISTANCE, t, t, t, _s13, t, t, t, t);
  }

  void GeodesicLineExact::GenSetDistance(bool arcmode, real s13_a13) {
    arcmode ? SetArc(s13_a13) : SetDistance(s13_a13);
  }

} // namespace GeographicLib
