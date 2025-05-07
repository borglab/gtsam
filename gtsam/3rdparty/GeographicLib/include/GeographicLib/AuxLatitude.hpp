/**
 * \file AuxLatitude.hpp
 * \brief Header for the GeographicLib::AuxLatitude class
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

#if !defined(GEOGRAPHICLIB_AUXLATITUDE_HPP)
#define GEOGRAPHICLIB_AUXLATITUDE_HPP 1

#include <utility>
#include <GeographicLib/Math.hpp>
#include <GeographicLib/AuxAngle.hpp>

#if !defined(GEOGRAPHICLIB_AUXLATITUDE_ORDER)
/**
 * The order of the series approximation used in AuxLatitude.
 * GEOGRAPHICLIB_AUXLATITUDE_ORDER can be set to one of [4, 6, 8].  Use order
 * appropriate for double precision, 6, also for GEOGRAPHICLIB_PRECISION == 5
 * to enable truncation errors to be measured easily.
 **********************************************************************/
#  define GEOGRAPHICLIB_AUXLATITUDE_ORDER \
  (GEOGRAPHICLIB_PRECISION == 2 || GEOGRAPHICLIB_PRECISION == 5 ? 6 : \
   (GEOGRAPHICLIB_PRECISION == 1 ? 4 : 8))
#endif

namespace GeographicLib {

  /**
   * \brief Conversions between auxiliary latitudes.
   *
   * This class is an implementation of the methods described in
   * - C. F. F. Karney,
   *   <a href="https://doi.org/10.1080/00396265.2023.2217604">
   *   On auxiliary latitudes,</a>
   *   Survey Review 56(395), 165--180 (2024);
   *   preprint
   *   <a href="https://arxiv.org/abs/2212.05818">arXiv:2212.05818</a>.
   *
   * The provides accurate conversions between geographic (\e phi, &phi;),
   * parametric (\e beta, &beta;), geocentric (\e theta, &theta;), rectifying
   * (\e mu, &mu;), conformal (\e chi, &chi;), and authalic (\e xi, &xi;)
   * latitudes for an ellipsoid of revolution.  A latitude is represented by
   * the class AuxAngle in order to maintain precision close to the poles.
   *
   * The class implements two methods for the conversion:
   * - Direct evaluation of the defining equations, the \e exact method.  These
   *   equations are formulated so as to preserve relative accuracy of the
   *   tangent of the latitude, ensuring high accuracy near the equator and the
   *   poles.  Newton's method is used for those conversions that can't be
   *   expressed in closed form.
   * - Expansions in powers of \e n, the third flattening, the \e series
   *   method.  This delivers full accuracy for abs(\e f) &le; 1/150.  Here, \e
   *   f is the flattening of the ellipsoid.
   *
   * The series method is the preferred method of conversion for any conversion
   * involving &mu;, &chi;, or &xi;, with abs(\e f) &le; 1/150.  The equations
   * for the conversions between &phi;, &beta;, and &theta; are sufficiently
   * simple that the exact method should be used for such conversions and also
   * for conversions with with abs(\e f) &gt; 1/150.
   *
   * Example of use:
   * \include example-AuxLatitude.cpp
   **********************************************************************/
  class GEOGRAPHICLIB_EXPORT AuxLatitude {
    typedef Math::real real;
    AuxLatitude(const std::pair<real, real>& axes);
  public:
    /**
     * The floating-point type for real numbers.  This just connects to the
     * template parameters for the class.
     **********************************************************************/
    /**
     * The different auxiliary latitudes.
     **********************************************************************/
    enum aux {
      /**
       * Geographic latitude, \e phi, &phi;
       * @hideinitializer
       **********************************************************************/
      GEOGRAPHIC = 0,
      /**
       * Parametric latitude, \e beta, &beta;
       * @hideinitializer
       **********************************************************************/
      PARAMETRIC = 1,
      /**
       * %Geocentric latitude, \e theta, &theta;
       * @hideinitializer
       **********************************************************************/
      GEOCENTRIC = 2,
      /**
       * Rectifying latitude, \e mu, &mu;
       * @hideinitializer
       **********************************************************************/
      RECTIFYING = 3,
      /**
       * Conformal latitude, \e chi, &chi;
       * @hideinitializer
       **********************************************************************/
      CONFORMAL  = 4,
      /**
       * Authalic latitude, \e xi, &xi;
       * @hideinitializer
       **********************************************************************/
      AUTHALIC   = 5,
      /**
       * The total number of auxiliary latitudes
       * @hideinitializer
       **********************************************************************/
      AUXNUMBER  = 6,
      /**
       * An alias for GEOGRAPHIC
       * @hideinitializer
       **********************************************************************/
      PHI = GEOGRAPHIC,
      /**
       * An alias for PARAMETRIC
       * @hideinitializer
       **********************************************************************/
      BETA = PARAMETRIC,
      /**
       * An alias for GEOCENTRIC
       * @hideinitializer
       **********************************************************************/
      THETA = GEOCENTRIC,
      /**
       * An alias for RECTIFYING
       * @hideinitializer
       **********************************************************************/
      MU = RECTIFYING,
      /**
       * An alias for CONFORMAL
       * @hideinitializer
       **********************************************************************/
      CHI = CONFORMAL,
      /**
       * An alias for AUTHALIC
       * @hideinitializer
       **********************************************************************/
      XI = AUTHALIC,
      /**
       * An alias for GEOGRAPHIC
       * @hideinitializer
       **********************************************************************/
      COMMON = GEOGRAPHIC,
      /**
       * An alias for GEOGRAPHIC
       * @hideinitializer
       **********************************************************************/
      GEODETIC = GEOGRAPHIC,
      /**
       * An alias for PARAMETRIC
       * @hideinitializer
       **********************************************************************/
      REDUCED = PARAMETRIC,
    };
    /**
     * Constructor
     *
     * @param[in] a equatorial radius.
     * @param[in] f flattening of ellipsoid.  Setting \e f = 0 gives a sphere.
     *   Negative \e f gives a prolate ellipsoid.
     * @exception GeographicErr if \e a or (1 &minus; \e f) \e a is not
     *   positive.
     *
     * \note the constructor does not precompute the coefficients for the
     * Fourier series for the series conversions.  These are computed and saved
     * when first needed.
     **********************************************************************/
    AuxLatitude(real a, real f);
    /**
     * Construct and return an AuxLatitude object specified in terms of the
     * semi-axes
     *
     * @param[in] a equatorial radius.
     * @param[in] b polar semi-axis.
     * @exception GeographicErr if \e a or \e b is not positive.
     *
     * This allows a new AuxAngle to be initialized as an angle in radians with
     * @code
     *   AuxLatitude aux(AuxLatitude::axes(a, b));
     * @endcode
     **********************************************************************/
    static AuxLatitude axes(real a, real b) {
      return AuxLatitude(std::pair<real, real>(a, b));
    }
    /**
     * Convert between any two auxiliary latitudes specified as AuxAngle.
     *
     * @param[in] auxin an AuxLatitude::aux indicating the type of
     *   auxiliary latitude \e zeta.
     * @param[in] auxout an AuxLatitude::aux indicating the type of
     *   auxiliary latitude \e eta.
     * @param[in] zeta the input auxiliary latitude as an AuxAngle
     * @param[in] exact if true use the exact equations instead of the Taylor
     *   series [default false].
     * @return the output auxiliary latitude \e eta as an AuxAngle.
     *
     * With \e exact = false, the Fourier coefficients for a specific \e auxin
     * and \e auxout are computed and saved on the first call; the saved
     * coefficients are used on subsequent calls.  The series method is
     * accurate for abs(\e f) &le; 1/150; for other \e f, the exact method
     * should be used.
     **********************************************************************/
    AuxAngle Convert(int auxin, int auxout, const AuxAngle& zeta,
                     bool exact = false) const;
    /**
     * Convert between any two auxiliary latitudes specified in degrees.
     *
     * @param[in] auxin an AuxLatitude::aux indicating the type of
     *   auxiliary latitude \e zeta.
     * @param[in] auxout an AuxLatitude::aux indicating the type of
     *   auxiliary latitude \e eta.
     * @param[in] zeta the input auxiliary latitude in degrees.
     * @param[in] exact if true use the exact equations instead of the Taylor
     *   series [default false].
     * @return the output auxiliary latitude \e eta in degrees.
     *
     * With \e exact = false, the Fourier coefficients for a specific \e auxin
     * and \e auxout are computed and saved on the first call; the saved
     * coefficients are used on subsequent calls.  The series method is
     * accurate for abs(\e f) &le; 1/150; for other \e f, the exact method
     * should be used.
     **********************************************************************/
    Math::real Convert(int auxin, int auxout, real zeta, bool exact = false)
      const;
    /**
     * Convert geographic latitude to an auxiliary latitude \e eta.
     *
     * @param[in] auxout an AuxLatitude::aux indicating the auxiliary
     *   latitude returned.
     * @param[in] phi the geographic latitude.
     * @param[out] diff optional pointer to the derivative d tan(\e eta) / d
     *   tan(\e phi).
     * @return the auxiliary latitude \e eta.
     *
     * This uses the exact equations.
     **********************************************************************/
    AuxAngle ToAuxiliary(int auxout, const AuxAngle& phi, real* diff = nullptr)
      const;
    /**
     * Convert an auxiliary latitude \e zeta to geographic latitude.
     *
     * @param[in] auxin an AuxLatitude::aux indicating the type of
     *   auxiliary latitude \e zeta.
     * @param[in] zeta the input auxiliary latitude.
     * @param[out] niter optional pointer to the number of iterations.
     * @return the geographic latitude \e phi.
     *
     * This uses the exact equations.
     **********************************************************************/
    AuxAngle FromAuxiliary(int auxin, const AuxAngle& zeta,
                           int* niter = nullptr) const;
    /**
     * Return the rectifying radius.
     *
     * @param[in] exact if true use the exact expression instead of the Taylor
     *   series [default false].
     * @return the rectifying radius in the same units as \e a.
     **********************************************************************/
    Math::real RectifyingRadius(bool exact = false) const;
    /**
     * Return the authalic radius squared.
     *
     * @param[in] exact if true use the exact expression instead of the Taylor
     *   series [default false].
     * @return the authalic radius squared in the same units as \e a.
     **********************************************************************/
    Math::real AuthalicRadiusSquared(bool exact = false) const;
    /**
     * @return \e a the equatorial radius of the ellipsoid (meters).
     **********************************************************************/
    Math::real EquatorialRadius() const { return _a; }
    /**
     * @return \e b the polar semi-axis of the ellipsoid (meters).
     **********************************************************************/
    Math::real PolarSemiAxis() const { return _b; }
    /**
     * @return \e f, the flattening of the ellipsoid.
     **********************************************************************/
    Math::real Flattening() const { return _f; }
    /**
     * Use Clenshaw to sum a Fouier series.
     *
     * @param[in] sinp if true sum the sine series, else sum the cosine series.
     * @param[in] szeta sin(\e zeta).
     * @param[in] czeta cos(\e zeta).
     * @param[in] c the array of coefficients.
     * @param[in] K the number of coefficients.
     * @return the Clenshaw sum.
     *
     * The result returned is \f$ \sum_0^{K-1} c_k \sin (2k+2)\zeta \f$, if \e
     * sinp is true; with \e sinp false, replace sin by cos.
     **********************************************************************/
    // Clenshaw applied to sum(c[k] * sin( (2*k+2) * zeta), i, 0, K-1);
    // if !sinp then subst sine->cosine.
    static Math::real Clenshaw(bool sinp, real szeta, real czeta,
                         const real c[], int K);
    /**
     * The order of the series expansions.  This is set at compile time to
     * either 4, 6, or 8, by the preprocessor macro
     * GEOGRAPHICLIB_AUXLATITUDE_ORDER.
     * @hideinitializer
     **********************************************************************/
    static const int Lmax = GEOGRAPHICLIB_AUXLATITUDE_ORDER;
    /**
     * A global instantiation of Ellipsoid with the parameters for the WGS84
     * ellipsoid.
     **********************************************************************/
    static const AuxLatitude& WGS84();
  private:
    // Maximum number of iterations for Newton's method
    static const int numit_ = 1000;
    real tol_, bmin_, bmax_;       // Static consts for Newton's method
    // the function atanh(e * sphi)/e + sphi / (1 - (e * sphi)^2);
  protected:
    /**
     * Convert geographic latitude to parametric latitude
     *
     * @param[in] phi geographic latitude.
     * @param[out] diff optional pointer to the derivative d tan(\e beta) / d
     *   tan(\e phi).
     * @return \e beta, the parametric latitude
     **********************************************************************/
    AuxAngle Parametric(const AuxAngle& phi, real* diff = nullptr) const;
    /**
     * Convert geographic latitude to geocentric latitude
     *
     * @param[in] phi geographic latitude.
     * @param[out] diff optional pointer to the derivative d tan(\e theta) / d
     *   tan(\e phi).
     * @return \e theta, the geocentric latitude.
     **********************************************************************/
    AuxAngle Geocentric(const AuxAngle& phi, real* diff = nullptr) const;
    /**
     * Convert geographic latitude to rectifying latitude
     *
     * @param[in] phi geographic latitude.
     * @param[out] diff optional pointer to the derivative d tan(\e mu) / d
     *   tan(\e phi).
     * @return \e mu, the rectifying latitude.
     **********************************************************************/
    AuxAngle Rectifying(const AuxAngle& phi, real* diff = nullptr) const;
    /**
     * Convert geographic latitude to conformal latitude
     *
     * @param[in] phi geographic latitude.
     * @param[out] diff optional pointer to the derivative d tan(\e chi) / d
     *   tan(\e phi).
     * @return \e chi, the conformal latitude.
     **********************************************************************/
    AuxAngle Conformal(const AuxAngle& phi, real* diff = nullptr) const;
    /**
     * Convert geographic latitude to authalic latitude
     *
     * @param[in] phi geographic latitude.
     * @param[out] diff optional pointer to the derivative d tan(\e xi) / d
     *   tan(\e phi).
     * @return \e xi, the authalic latitude.
     **********************************************************************/
    AuxAngle Authalic(const AuxAngle& phi, real* diff = nullptr) const;
    /// \cond SKIP
    // Ellipsoid parameters
    real _a, _b, _f, _fm1, _e2, _e2m1, _e12, _e12p1, _n, _e, _e1, _n2, _q;
    // To hold computed Fourier coefficients
    mutable real _c[Lmax * AUXNUMBER * AUXNUMBER];
    // 1d index into AUXNUMBER x AUXNUMBER data
    static int ind(int auxout, int auxin) {
      return (auxout >= 0 && auxout < AUXNUMBER &&
              auxin  >= 0 && auxin  < AUXNUMBER) ?
        AUXNUMBER * auxout + auxin : -1;
    }
    // the function sqrt(1 + tphi^2), convert tan to sec
    static real sc(real tphi)
    { using std::hypot; return hypot(real(1), tphi); }
    // the function tphi / sqrt(1 + tphi^2), convert tan to sin
    static real sn(real tphi) {
      using std::isinf; using std::copysign;
      return isinf(tphi) ? copysign(real(1), tphi) : tphi / sc(tphi);
    }
    // Populate [_c[Lmax * k], _c[Lmax * (k + 1)])
    void fillcoeff(int auxin, int auxout, int k) const;
    // the function atanh(e * sphi)/e; works for e^2 = 0 and e^2 < 0
    real atanhee(real tphi) const;
    /// \endcond
  private:
    // the function atanh(e * sphi)/e + sphi / (1 - (e * sphi)^2);
    real q(real tphi) const;
    // The divided difference of (q(1) - q(sphi)) / (1 - sphi)
    real Dq(real tphi) const;
  };

} // namespace GeographicLib

#endif  // GEOGRAPHICLIB_AUXLATITUDE_HPP
