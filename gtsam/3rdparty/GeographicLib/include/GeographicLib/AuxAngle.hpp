/**
 * \file AuxAngle.hpp
 * \brief Header for the GeographicLib::AuxAngle class
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

#if !defined(GEOGRAPHICLIB_AUXANGLE_HPP)
#define GEOGRAPHICLIB_AUXANGLE_HPP 1

#include <GeographicLib/Math.hpp>

namespace GeographicLib {

  /**
   * \brief An accurate representation of angles.
   *
   * This class is an implementation of the methods described in
   * - C. F. F. Karney,
   *   <a href="https://doi.org/10.1080/00396265.2023.2217604">
   *   On auxiliary latitudes,</a>
   *   Survey Review 56(395), 165--180 (2024);
   *   preprint
   *   <a href="https://arxiv.org/abs/2212.05818">arXiv:2212.05818</a>.
   *
   * An angle is represented be the \e y and \e x coordinates of a point in the
   * 2d plane.  The two coordinates are proportional to the sine and cosine of
   * the angle.  This allows angles close to the cardinal points to be
   * represented accurately.  It also saves on unnecessary recomputations of
   * trigonometric functions of the angle.  Only angles in [&minus;180&deg;,
   * 180&deg;] can be represented.  (A possible extension would be to keep
   * count of the number of turns.)
   *
   * Example of use:
   * \include example-AuxAngle.cpp
   **********************************************************************/
  class GEOGRAPHICLIB_EXPORT AuxAngle {
  private:
    typedef Math::real real;
    real _y, _x;
  public:
    /**
     * The constructor.
     *
     * @param[in] y the \e y coordinate.
     * @param[in] x the \e x coordinate.
     *
     * \note the \e y coordinate is specified \e first.
     * \warning either \e x or \e y can be infinite, but not both.
     *
     * The defaults (\e x = 1 and \e y = 0) are such that
     * + no arguments gives an angle of 0;
     * + 1 argument specifies the tangent of the angle.
     **********************************************************************/
    explicit AuxAngle(real y = 0, real x = 1) : _y(y), _x(x) {}
    /**
     * @return the \e y component.  This is the sine of the angle if the
     *   AuxAngle has been normalized.
     **********************************************************************/
    Math::real y() const { return _y; }
    /**
     * @return the \e x component.  This is the cosine of the angle if the
     *   AuxAngle has been normalized.
     **********************************************************************/
    Math::real x() const { return _x; }
    /**
     * @return a reference to the \e y component.  This allows this component
     *   to be altered.
     **********************************************************************/
    Math::real& y() { return _y; }
    /**
     * @return a reference to the \e x component.  This allows this component
     *   to be altered.
     **********************************************************************/
    Math::real& x() { return _x; }
    /**
     * @return the AuxAngle converted to the conventional angle measured in
     *   degrees.
     **********************************************************************/
    Math::real degrees() const;
    /**
     * @return the AuxAngle converted to the conventional angle measured in
     *   radians.
     **********************************************************************/
    Math::real radians() const;
    /**
     * @return the lambertian of the AuxAngle.
     *
     * \note the lambertian of an angle &chi; is
     * lam(&chi;) = asinh(tan(&chi;)).
     **********************************************************************/
    Math::real lam() const;
    /**
     * @return the lambertian of the AuxAngle in degrees.
     *
     * \note the lambertian of an angle &chi; is
     * lam(&chi;) = asinh(tan(&chi;)).
     **********************************************************************/
    Math::real lamd() const;
    /**
     * @return the tangent of the angle.
     **********************************************************************/
    Math::real tan() const { return _y / _x; }
    /**
     * @return a new normalized AuxAngle with the point lying on the unit
     *   circle and the \e y and \e x components are equal to the sine and
     *   cosine of the angle.
     **********************************************************************/
    AuxAngle normalized() const;
    /**
     * Normalize the AuxAngle in place so that the \e y and \e x components are
     *   equal to the sine and cosine of the angle.
     **********************************************************************/
    void normalize() { *this = normalized(); }
    /**
     * Set the quadrant for the AuxAngle.
     *
     * @param[in] p the AuxAngle from which the quadrant information is taken.
     * @return the new AuxAngle in the same quadrant as \e p.
     **********************************************************************/
    AuxAngle copyquadrant(const AuxAngle& p) const;
    /**
     * Add an AuxAngle.
     *
     * @param[in] p the AuxAngle to be added.
     * @return a reference to the new AuxAngle.
     *
     * The addition is done in place, altering the current AuxAngle.
     *
     * \warning Neither *this nor \e p should have an infinite component.  If
     * necessary, invoke AuxAngle::normalize on these angles first.
     **********************************************************************/
    AuxAngle& operator+=(const AuxAngle& p);
    /**
     * Construct and return an AuxAngle specied as an angle in degrees.
     *
     * @param[in] d the angle measured in degrees.
     * @return the corresponding AuxAngle.
     *
     * This allows a new AuxAngle to be initialized as an angle in degrees with
     * @code
     *   AuxAngle phi(AuxAngle::degrees(d));
     * @endcode
     **********************************************************************/
    static AuxAngle degrees(real d);
    /**
     * Construct and return an AuxAngle specied as an angle in radians.
     *
     * @param[in] r the angle measured in radians.
     * @return the corresponding AuxAngle.
     *
     * This allows a new AuxAngle to be initialized as an angle in radians with
     * @code
     *   AuxAngle phi(AuxAngle::radians(r));
     * @endcode
     **********************************************************************/
    static AuxAngle radians(real r);
    /**
     * Construct and return an AuxAngle specied by the lambertian of the angle.
     *
     * @param[in] psi the lambertian of the angle.
     * @return the corresponding AuxAngle.
     *
     * This allows a new AuxAngle to be initialized given the lambertian with
     * @code
     *   AuxAngle chi(AuxAngle::lam(psi));
     * @endcode
     *
     * \note this sets the angle &chi; to gd(&psi;) = atan(sinh(&psi;)).
     **********************************************************************/
    static AuxAngle lam(real psi);
    /**
     * Construct and return an AuxAngle specied by the lambertian of the angle
     * in degrees.
     *
     * @param[in] psid the lambertian of the angle in degrees.
     * @return the corresponding AuxAngle.
     *
     * This allows a new AuxAngle to be initialized given the lambertian with
     * @code
     *   AuxAngle chi(AuxAngle::lamd(psid));
     * @endcode
     *
     * \note this sets the angle &chi; to gd(&psi;) = atan(sinh(&psi;)).
     **********************************************************************/
    static AuxAngle lamd(real psid);
    /**
     * @return a "NaN" AuxAngle.
     **********************************************************************/
    static AuxAngle NaN();
  };

  inline AuxAngle AuxAngle::degrees(real d) {
    real y, x;
    Math::sincosd(d, y, x);
    return AuxAngle(y, x);
  }

  inline AuxAngle AuxAngle::radians(real r) {
    using std::sin; using std::cos;
    return AuxAngle(sin(r), cos(r));
  }

  inline AuxAngle AuxAngle::lam(real psi) {
    using std::sinh;
    return AuxAngle(sinh(psi));
  }

  inline AuxAngle AuxAngle::lamd(real psid) {
    using std::sinh;
    return AuxAngle(sinh(psid * Math::degree()));
  }

  inline Math::real AuxAngle::degrees() const {
    return Math::atan2d(_y, _x);
  }

  inline Math::real AuxAngle::radians() const {
    using std::atan2; return atan2(_y, _x);
  }

  inline Math::real AuxAngle::lam() const {
    using std::asinh; return asinh( tan() );
  }

  inline Math::real AuxAngle::lamd() const {
    using std::asinh; return asinh( tan() ) / Math::degree();
  }

} // namespace GeographicLib

#endif  // GEOGRAPHICLIB_AUXANGLE_HPP
