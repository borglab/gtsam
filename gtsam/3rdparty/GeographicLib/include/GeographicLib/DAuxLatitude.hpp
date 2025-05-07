/**
 * \file DAuxLatitude.hpp
 * \brief Header for the GeographicLib::DAuxLatitude class
 *
 * Copyright (c) Charles Karney (2022-2023) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_DAUXLATITUDE_HPP)
#define GEOGRAPHICLIB_DAUXLATITUDE_HPP 1

#include <GeographicLib/AuxLatitude.hpp>

namespace GeographicLib {

  /**
   * \brief Divided differences of auxiliary latitudes.
   *
   * This class computed the divided differences of auxiliary latitudes and
   * some other divided differences needed to support rhumb line calculations.
   **********************************************************************/
  class GEOGRAPHICLIB_EXPORT DAuxLatitude : public AuxLatitude {
  private:
    typedef Math::real real;
    typedef AuxLatitude base;
  public:
    /**
     * Constructor
     *
     * @param[in] a equatorial radius.
     * @param[in] f flattening of ellipsoid.  Setting \e f = 0 gives a sphere.
     *   Negative \e f gives a prolate ellipsoid.
     * @exception GeographicErr if \e a or (1 &minus; \e f) \e a is not
     *   positive.
     **********************************************************************/
    DAuxLatitude(real a, real f) : AuxLatitude(a, f) {}
    /**
     * The divided difference of one auxiliary latitude with respect to
     * another.
     *
     * @param[in] auxin an AuxLatitude::aux indicating the type of
     *   auxiliary latitude \e zeta.
     * @param[in] auxout an AuxLatitude::aux indicating the type of
     *   auxiliary latitude \e eta.
     * @param[in] zeta1 the first of the input auxiliary latitudeas.
     * @param[in] zeta2 the second of the input auxiliary latitude.
     * @return the divided difference (\e eta2 - \e eta1) / (\e zeta2 - \e
     *   zeta1).
     *
     * \note This routine uses the series method.
     *
     * In the expression for the divided difference above, the angle quantities
     * should be understood as the conventional measure of angle (either in
     * radians or in degrees).
     *
     * The Fourier coefficients for a specific \e auxin and \e auxout are
     * computed and saved on the first call; the saved coefficients are used on
     * subsequent calls.  The series method is accurate for abs(\e f) &le;
     * 1/150.
     **********************************************************************/
    Math::real DConvert(int auxin, int auxout,
                        const AuxAngle& zeta1, const AuxAngle& zeta2) const;
    /**
     * The divided difference of the parametric latitude with respect to the
     * geographic latitude.
     *
     * @param[in] phi1 the first geographic latitude as an AuxAngle.
     * @param[in] phi2 the second geographic latitude as an AuxAngle.
     * @return the divided difference (\e beta2 - \e beta1) / (\e phi2 - \e
     *   phi1), where \e beta is the parametric latitude.
     *
     * \note This routine uses the exact formulas and is valid for arbitrary
     * latitude.
     **********************************************************************/
    Math::real DParametric(const AuxAngle& phi1, const AuxAngle& phi2) const;
    /**
     * The divided difference of the rectifying latitude with respect to the
     * geographic latitude.
     *
     * @param[in] phi1 the first geographic latitude as an AuxAngle.
     * @param[in] phi2 the second geographic latitude as an AuxAngle.
     * @return the divided difference (\e mu2 - \e mu1) / (\e phi2 - \e
     *   phi1), where \e mu is the rectifying latitude.
     *
     * \note This routine uses the exact formulas and is valid for arbitrary
     * latitude.
     **********************************************************************/
    Math::real DRectifying(const AuxAngle& phi1, const AuxAngle& phi2) const;
    /**
     * The divided difference of the isometric latitude with respect to the
     * geographic latitude.
     *
     * @param[in] phi1 the first geographic latitude as an AuxAngle.
     * @param[in] phi2 the second geographic latitude as an AuxAngle.
     * @return the divided difference (\e psi2 - \e psi1) / (\e phi2 - \e
     *   phi1), where \e psi = asinh( tan(\e chi) ) is the isometric latitude
     *   and \e chi is the conformal latitude.
     *
     * \note This routine uses the exact formulas and is valid for arbitrary
     * latitude.
     **********************************************************************/
    Math::real DIsometric(const AuxAngle& phi1, const AuxAngle& phi2) const;
    /**
     * The divided difference of AuxLatitude::Clenshaw.
     *
     * @param[in] sinp if true sum the sine series, else sum the cosine series.
     * @param[in] Delta either 1 \e or (zeta2 - zeta1) in radians.
     * @param[in] szeta1 sin(\e zeta1).
     * @param[in] czeta1 cos(\e zeta1).
     * @param[in] szeta2 sin(\e zeta2).
     * @param[in] czeta2 cos(\e zeta2).
     * @param[in] c the array of coefficients.
     * @param[in] K the number of coefficients.
     * @return the divided difference.
     *
     * The result is
     *  <pre>
     *    ( AuxLatitude::Clenshaw(sinp, szeta2, czeta2, c, K) -
     *      AuxLatitude::Clenshaw(sinp, szeta1, czeta1, c, K) ) / Delta
     * </pre>
     *
     * \warning \e Delta **must** be either 1 or (\e zeta2 - \e zeta1);
     * other values will return nonsense.
     **********************************************************************/
    static Math::real DClenshaw(bool sinp, real Delta,
                                real szeta1, real czeta1,
                                real szeta2, real czeta2,
                                const real c[], int K);
    /**
     * The divided difference of the isometric latitude with respect to the
     * conformal latitude.
     *
     * @param[in] x tan(\e chi1).
     * @param[in] y tan(\e chi2).
     * @return the divided difference (\e psi2 - \e psi1) / (\e chi2 - \e
     *   chi1), where \e psi = asinh( tan(\e chi) ).
     *
     * \note This parameters for this routine are the \e tangents of conformal
     * latitude.
     *
     * This routine computes Dasinh(x, y) / Datan(x, y).
     **********************************************************************/
    static Math::real Dlam(real x, real y) {
      using std::isnan; using std::isinf;
      return x == y ? base::sc(x) :
        (isnan(x) || isnan(y) ? std::numeric_limits<real>::quiet_NaN() :
         (isinf(x) || isinf(y) ? std::numeric_limits<real>::infinity() :
          Dasinh(x, y) / Datan(x, y)));
    }
    // Dp0Dpsi in terms of chi
    /**
     * The divided difference of the spherical rhumb area term with respect to
     * the isometric latitude.
     *
     * @param[in] x tan(\e chi1).
     * @param[in] y tan(\e chi2).
     * @return the divided difference (p0(\e chi2) - p0(\e chi1)) / (\e psi2 -
     *   \e psi1), where p0(\e chi) = log( sec(\e chi) ) and \e psi = asinh(
     *   tan(\e chi) ).
     *
     * \note This parameters for this routine are the \e tangents of conformal
     * latitude.
     **********************************************************************/
    static Math::real Dp0Dpsi(real x, real y) {
      using std::isnan; using std::isinf; using std::copysign;
      return x == y ? base::sn(x) :
        (isnan(x + y) ? x + y : // N.B. nan for inf-inf
         (isinf(x) ? copysign(real(1), x) :
          (isinf(y) ? copysign(real(1), y) :
           Dasinh(h(x), h(y)) * Dh(x, y) / Dasinh(x, y))));
    }
  protected:                    // so TestAux can access these functions
    /// \cond SKIP
    // (sn(y) - sn(x)) / (y - x)
    static real Dsn(real x, real y);
    static real Datan(real x, real y);
    static real Dasinh(real x, real y);
    // h(tan(x)) = tan(x) * sin(x) / 2
    static real h(real x) { return x * base::sn(x) / 2; }
    static real Dh(real x, real y);
    real Datanhee(real tphi1, real tphi2) const;
    /// \endcond
  private:
    static real Dsin(real x, real y) {
      using std::sin; using std::cos;
      real d = (x - y) / 2;
      return cos((x + y) / 2) * (d != 0 ? sin(d) / d : 1);
    }
    // (E(x) - E(y)) / (x - y)
    real DE(const AuxAngle& X, const AuxAngle& Y) const;
  };

} // namespace GeographicLib

#endif  // GEOGRAPHICLIB_DAUXLATITUDE_HPP
