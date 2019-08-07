/**
 * \file Ellipsoid.hpp
 * \brief Header for GeographicLib::Ellipsoid class
 *
 * Copyright (c) Charles Karney (2012-2017) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_ELLIPSOID_HPP)
#define GEOGRAPHICLIB_ELLIPSOID_HPP 1

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/TransverseMercator.hpp>
#include <GeographicLib/EllipticFunction.hpp>
#include <GeographicLib/AlbersEqualArea.hpp>

namespace GeographicLib {

  /**
   * \brief Properties of an ellipsoid
   *
   * This class returns various properties of the ellipsoid and converts
   * between various types of latitudes.  The latitude conversions are also
   * possible using the various projections supported by %GeographicLib; but
   * Ellipsoid provides more direct access (sometimes using private functions
   * of the projection classes).  Ellipsoid::RectifyingLatitude,
   * Ellipsoid::InverseRectifyingLatitude, and Ellipsoid::MeridianDistance
   * provide functionality which can be provided by the Geodesic class.
   * However Geodesic uses a series approximation (valid for abs \e f < 1/150),
   * whereas Ellipsoid computes these quantities using EllipticFunction which
   * provides accurate results even when \e f is large.  Use of this class
   * should be limited to &minus;3 < \e f < 3/4 (i.e., 1/4 < b/a < 4).
   *
   * Example of use:
   * \include example-Ellipsoid.cpp
   **********************************************************************/

  class GEOGRAPHICLIB_EXPORT Ellipsoid {
  private:
    typedef Math::real real;
    static const int numit_ = 10;
    real stol_;
    real _a, _f, _f1, _f12, _e2, _es, _e12, _n, _b;
    TransverseMercator _tm;
    EllipticFunction _ell;
    AlbersEqualArea _au;

    // These are the alpha and beta coefficients in the Krueger series from
    // TransverseMercator.  Thy are used by RhumbSolve to compute
    // (psi2-psi1)/(mu2-mu1).
    const Math::real* ConformalToRectifyingCoeffs() const { return _tm._alp; }
    const Math::real* RectifyingToConformalCoeffs() const { return _tm._bet; }
    friend class Rhumb; friend class RhumbLine;
  public:
    /** \name Constructor
     **********************************************************************/
    ///@{

    /**
     * Constructor for a ellipsoid with
     *
     * @param[in] a equatorial radius (meters).
     * @param[in] f flattening of ellipsoid.  Setting \e f = 0 gives a sphere.
     *   Negative \e f gives a prolate ellipsoid.
     * @exception GeographicErr if \e a or (1 &minus; \e f) \e a is not
     *   positive.
     **********************************************************************/
    Ellipsoid(real a, real f);
    ///@}

    /** \name %Ellipsoid dimensions.
     **********************************************************************/
    ///@{

    /**
     * @return \e a the equatorial radius of the ellipsoid (meters).  This is
     *   the value used in the constructor.
     **********************************************************************/
    Math::real MajorRadius() const { return _a; }

    /**
     * @return \e b the polar semi-axis (meters).
     **********************************************************************/
    Math::real MinorRadius() const { return _b; }

    /**
     * @return \e L the distance between the equator and a pole along a
     *   meridian (meters).  For a sphere \e L = (&pi;/2) \e a.  The radius
     *   of a sphere with the same meridian length is \e L / (&pi;/2).
     **********************************************************************/
    Math::real QuarterMeridian() const;

    /**
     * @return \e A the total area of the ellipsoid (meters<sup>2</sup>).  For
     *   a sphere \e A = 4&pi; <i>a</i><sup>2</sup>.  The radius of a sphere
     *   with the same area is sqrt(\e A / (4&pi;)).
     **********************************************************************/
    Math::real Area() const;

    /**
     * @return \e V the total volume of the ellipsoid (meters<sup>3</sup>).
     *   For a sphere \e V = (4&pi; / 3) <i>a</i><sup>3</sup>.  The radius of
     *   a sphere with the same volume is cbrt(\e V / (4&pi;/3)).
     **********************************************************************/
    Math::real Volume() const
    { return (4 * Math::pi()) * Math::sq(_a) * _b / 3; }
    ///@}

    /** \name %Ellipsoid shape
     **********************************************************************/
    ///@{

    /**
     * @return \e f = (\e a &minus; \e b) / \e a, the flattening of the
     *   ellipsoid.  This is the value used in the constructor.  This is zero,
     *   positive, or negative for a sphere, oblate ellipsoid, or prolate
     *   ellipsoid.
     **********************************************************************/
    Math::real Flattening() const { return _f; }

    /**
     * @return \e f ' = (\e a &minus; \e b) / \e b, the second flattening of
     *   the ellipsoid.  This is zero, positive, or negative for a sphere,
     *   oblate ellipsoid, or prolate ellipsoid.
     **********************************************************************/
    Math::real SecondFlattening() const { return _f / (1 - _f); }

    /**
     * @return \e n = (\e a &minus; \e b) / (\e a + \e b), the third flattening
     *   of the ellipsoid.  This is zero, positive, or negative for a sphere,
     *   oblate ellipsoid, or prolate ellipsoid.
     **********************************************************************/
    Math::real ThirdFlattening() const { return _n; }

    /**
     * @return <i>e</i><sup>2</sup> = (<i>a</i><sup>2</sup> &minus;
     *   <i>b</i><sup>2</sup>) / <i>a</i><sup>2</sup>, the eccentricity squared
     *   of the ellipsoid.  This is zero, positive, or negative for a sphere,
     *   oblate ellipsoid, or prolate ellipsoid.
     **********************************************************************/
    Math::real EccentricitySq() const { return _e2; }

    /**
     * @return <i>e'</i> <sup>2</sup> = (<i>a</i><sup>2</sup> &minus;
     *   <i>b</i><sup>2</sup>) / <i>b</i><sup>2</sup>, the second eccentricity
     *   squared of the ellipsoid.  This is zero, positive, or negative for a
     *   sphere, oblate ellipsoid, or prolate ellipsoid.
     **********************************************************************/
    Math::real SecondEccentricitySq() const { return _e12; }

    /**
     * @return <i>e''</i> <sup>2</sup> = (<i>a</i><sup>2</sup> &minus;
     *   <i>b</i><sup>2</sup>) / (<i>a</i><sup>2</sup> + <i>b</i><sup>2</sup>),
     *   the third eccentricity squared of the ellipsoid.  This is zero,
     *   positive, or negative for a sphere, oblate ellipsoid, or prolate
     *   ellipsoid.
     **********************************************************************/
    Math::real ThirdEccentricitySq() const { return _e2 / (2 - _e2); }
    ///@}

    /** \name Latitude conversion.
     **********************************************************************/
    ///@{

    /**
     * @param[in] phi the geographic latitude (degrees).
     * @return &beta; the parametric latitude (degrees).
     *
     * The geographic latitude, &phi;, is the angle beween the equatorial
     * plane and a vector normal to the surface of the ellipsoid.
     *
     * The parametric latitude (also called the reduced latitude), &beta;,
     * allows the cartesian coordinated of a meridian to be expressed
     * conveniently in parametric form as
     * - \e R = \e a cos &beta;
     * - \e Z = \e b sin &beta;
     * .
     * where \e a and \e b are the equatorial radius and the polar semi-axis.
     * For a sphere &beta; = &phi;.
     *
     * &phi; must lie in the range [&minus;90&deg;, 90&deg;]; the
     * result is undefined if this condition does not hold.  The returned value
     * &beta; lies in [&minus;90&deg;, 90&deg;].
     **********************************************************************/
    Math::real ParametricLatitude(real phi) const;

    /**
     * @param[in] beta the parametric latitude (degrees).
     * @return &phi; the geographic latitude (degrees).
     *
     * &beta; must lie in the range [&minus;90&deg;, 90&deg;]; the
     * result is undefined if this condition does not hold.  The returned value
     * &phi; lies in [&minus;90&deg;, 90&deg;].
     **********************************************************************/
    Math::real InverseParametricLatitude(real beta) const;

    /**
     * @param[in] phi the geographic latitude (degrees).
     * @return &theta; the geocentric latitude (degrees).
     *
     * The geocentric latitude, &theta;, is the angle beween the equatorial
     * plane and a line between the center of the ellipsoid and a point on the
     * ellipsoid.  For a sphere &theta; = &phi;.
     *
     * &phi; must lie in the range [&minus;90&deg;, 90&deg;]; the
     * result is undefined if this condition does not hold.  The returned value
     * &theta; lies in [&minus;90&deg;, 90&deg;].
     **********************************************************************/
    Math::real GeocentricLatitude(real phi) const;

    /**
     * @param[in] theta the geocentric latitude (degrees).
     * @return &phi; the geographic latitude (degrees).
     *
     * &theta; must lie in the range [&minus;90&deg;, 90&deg;]; the
     * result is undefined if this condition does not hold.  The returned value
     * &phi; lies in [&minus;90&deg;, 90&deg;].
     **********************************************************************/
    Math::real InverseGeocentricLatitude(real theta) const;

    /**
     * @param[in] phi the geographic latitude (degrees).
     * @return &mu; the rectifying latitude (degrees).
     *
     * The rectifying latitude, &mu;, has the property that the distance along
     * a meridian of the ellipsoid between two points with rectifying latitudes
     * &mu;<sub>1</sub> and &mu;<sub>2</sub> is equal to
     * (&mu;<sub>2</sub> - &mu;<sub>1</sub>) \e L / 90&deg;,
     * where \e L = QuarterMeridian().  For a sphere &mu; = &phi;.
     *
     * &phi; must lie in the range [&minus;90&deg;, 90&deg;]; the
     * result is undefined if this condition does not hold.  The returned value
     * &mu; lies in [&minus;90&deg;, 90&deg;].
     **********************************************************************/
    Math::real RectifyingLatitude(real phi) const;

    /**
     * @param[in] mu the rectifying latitude (degrees).
     * @return &phi; the geographic latitude (degrees).
     *
     * &mu; must lie in the range [&minus;90&deg;, 90&deg;]; the
     * result is undefined if this condition does not hold.  The returned value
     * &phi; lies in [&minus;90&deg;, 90&deg;].
     **********************************************************************/
    Math::real InverseRectifyingLatitude(real mu) const;

    /**
     * @param[in] phi the geographic latitude (degrees).
     * @return &xi; the authalic latitude (degrees).
     *
     * The authalic latitude, &xi;, has the property that the area of the
     * ellipsoid between two circles with authalic latitudes
     * &xi;<sub>1</sub> and &xi;<sub>2</sub> is equal to (sin
     * &xi;<sub>2</sub> - sin &xi;<sub>1</sub>) \e A / 2, where \e A
     * = Area().  For a sphere &xi; = &phi;.
     *
     * &phi; must lie in the range [&minus;90&deg;, 90&deg;]; the
     * result is undefined if this condition does not hold.  The returned value
     * &xi; lies in [&minus;90&deg;, 90&deg;].
     **********************************************************************/
    Math::real AuthalicLatitude(real phi) const;

    /**
     * @param[in] xi the authalic latitude (degrees).
     * @return &phi; the geographic latitude (degrees).
     *
     * &xi; must lie in the range [&minus;90&deg;, 90&deg;]; the
     * result is undefined if this condition does not hold.  The returned value
     * &phi; lies in [&minus;90&deg;, 90&deg;].
     **********************************************************************/
    Math::real InverseAuthalicLatitude(real xi) const;

    /**
     * @param[in] phi the geographic latitude (degrees).
     * @return &chi; the conformal latitude (degrees).
     *
     * The conformal latitude, &chi;, gives the mapping of the ellipsoid to a
     * sphere which which is conformal (angles are preserved) and in which the
     * equator of the ellipsoid maps to the equator of the sphere.  For a
     * sphere &chi; = &phi;.
     *
     * &phi; must lie in the range [&minus;90&deg;, 90&deg;]; the
     * result is undefined if this condition does not hold.  The returned value
     * &chi; lies in [&minus;90&deg;, 90&deg;].
     **********************************************************************/
    Math::real ConformalLatitude(real phi) const;

    /**
     * @param[in] chi the conformal latitude (degrees).
     * @return &phi; the geographic latitude (degrees).
     *
     * &chi; must lie in the range [&minus;90&deg;, 90&deg;]; the
     * result is undefined if this condition does not hold.  The returned value
     * &phi; lies in [&minus;90&deg;, 90&deg;].
     **********************************************************************/
    Math::real InverseConformalLatitude(real chi) const;

    /**
     * @param[in] phi the geographic latitude (degrees).
     * @return &psi; the isometric latitude (degrees).
     *
     * The isometric latitude gives the mapping of the ellipsoid to a plane
     * which which is conformal (angles are preserved) and in which the equator
     * of the ellipsoid maps to a straight line of constant scale; this mapping
     * defines the Mercator projection.  For a sphere &psi; =
     * sinh<sup>&minus;1</sup> tan &phi;.
     *
     * &phi; must lie in the range [&minus;90&deg;, 90&deg;]; the result is
     * undefined if this condition does not hold.  The value returned for &phi;
     * = &plusmn;90&deg; is some (positive or negative) large but finite value,
     * such that InverseIsometricLatitude returns the original value of &phi;.
     **********************************************************************/
    Math::real IsometricLatitude(real phi) const;

    /**
     * @param[in] psi the isometric latitude (degrees).
     * @return &phi; the geographic latitude (degrees).
     *
     * The returned value &phi; lies in [&minus;90&deg;, 90&deg;].  For a
     * sphere &phi; = tan<sup>&minus;1</sup> sinh &psi;.
     **********************************************************************/
    Math::real InverseIsometricLatitude(real psi) const;
    ///@}

    /** \name Other quantities.
     **********************************************************************/
    ///@{

    /**
     * @param[in] phi the geographic latitude (degrees).
     * @return \e R = \e a cos &beta; the radius of a circle of latitude
     *   &phi; (meters).  \e R (&pi;/180&deg;) gives meters per degree
     *   longitude measured along a circle of latitude.
     *
     * &phi; must lie in the range [&minus;90&deg;, 90&deg;]; the
     * result is undefined if this condition does not hold.
     **********************************************************************/
    Math::real CircleRadius(real phi) const;

    /**
     * @param[in] phi the geographic latitude (degrees).
     * @return \e Z = \e b sin &beta; the distance of a circle of latitude
     *   &phi; from the equator measured parallel to the ellipsoid axis
     *   (meters).
     *
     * &phi; must lie in the range [&minus;90&deg;, 90&deg;]; the
     * result is undefined if this condition does not hold.
     **********************************************************************/
    Math::real CircleHeight(real phi) const;

    /**
     * @param[in] phi the geographic latitude (degrees).
     * @return \e s the distance along a meridian
     *   between the equator and a point of latitude &phi; (meters).  \e s is
     *   given by \e s = &mu; \e L / 90&deg;, where \e L =
     *   QuarterMeridian()).
     *
     * &phi; must lie in the range [&minus;90&deg;, 90&deg;]; the
     * result is undefined if this condition does not hold.
     **********************************************************************/
    Math::real MeridianDistance(real phi) const;

    /**
     * @param[in] phi the geographic latitude (degrees).
     * @return &rho; the meridional radius of curvature of the ellipsoid at
     *   latitude &phi; (meters); this is the curvature of the meridian.  \e
     *   rho is given by &rho; = (180&deg;/&pi;) d\e s / d&phi;,
     *   where \e s = MeridianDistance(); thus &rho; (&pi;/180&deg;)
     *   gives meters per degree latitude measured along a meridian.
     *
     * &phi; must lie in the range [&minus;90&deg;, 90&deg;]; the
     * result is undefined if this condition does not hold.
     **********************************************************************/
    Math::real MeridionalCurvatureRadius(real phi) const;

    /**
     * @param[in] phi the geographic latitude (degrees).
     * @return &nu; the transverse radius of curvature of the ellipsoid at
     *   latitude &phi; (meters); this is the curvature of a curve on the
     *   ellipsoid which also lies in a plane perpendicular to the ellipsoid
     *   and to the meridian.  &nu; is related to \e R = CircleRadius() by \e
     *   R = &nu; cos &phi;.
     *
     * &phi; must lie in the range [&minus;90&deg;, 90&deg;]; the
     * result is undefined if this condition does not hold.
     **********************************************************************/
    Math::real TransverseCurvatureRadius(real phi) const;

    /**
     * @param[in] phi the geographic latitude (degrees).
     * @param[in] azi the angle between the meridian and the normal section
     *   (degrees).
     * @return the radius of curvature of the ellipsoid in the normal
     *   section at latitude &phi; inclined at an angle \e azi to the
     *   meridian (meters).
     *
     * &phi; must lie in the range [&minus;90&deg;, 90&deg;]; the result is
     * undefined this condition does not hold.
     **********************************************************************/
    Math::real NormalCurvatureRadius(real phi, real azi) const;
    ///@}

    /** \name Eccentricity conversions.
     **********************************************************************/
    ///@{

    /**
     * @param[in] fp = \e f ' = (\e a &minus; \e b) / \e b, the second
     *   flattening.
     * @return \e f = (\e a &minus; \e b) / \e a, the flattening.
     *
     * \e f ' should lie in (&minus;1, &infin;).
     * The returned value \e f lies in (&minus;&infin;, 1).
     **********************************************************************/
    static Math::real SecondFlatteningToFlattening(real fp)
    { return fp / (1 + fp); }

    /**
     * @param[in] f = (\e a &minus; \e b) / \e a, the flattening.
     * @return \e f ' = (\e a &minus; \e b) / \e b, the second flattening.
     *
     * \e f should lie in (&minus;&infin;, 1).
     * The returned value \e f ' lies in (&minus;1, &infin;).
     **********************************************************************/
    static Math::real FlatteningToSecondFlattening(real f)
    { return f / (1 - f); }

    /**
     * @param[in] n = (\e a &minus; \e b) / (\e a + \e b), the third
     *   flattening.
     * @return \e f = (\e a &minus; \e b) / \e a, the flattening.
     *
     * \e n should lie in (&minus;1, 1).
     * The returned value \e f lies in (&minus;&infin;, 1).
     **********************************************************************/
    static Math::real ThirdFlatteningToFlattening(real n)
    { return 2 * n / (1 + n); }

    /**
     * @param[in] f = (\e a &minus; \e b) / \e a, the flattening.
     * @return \e n = (\e a &minus; \e b) / (\e a + \e b), the third
     *   flattening.
     *
     * \e f should lie in (&minus;&infin;, 1).
     * The returned value \e n lies in (&minus;1, 1).
     **********************************************************************/
    static Math::real FlatteningToThirdFlattening(real f)
    { return f / (2 - f); }

    /**
     * @param[in] e2 = <i>e</i><sup>2</sup> = (<i>a</i><sup>2</sup> &minus;
     *   <i>b</i><sup>2</sup>) / <i>a</i><sup>2</sup>, the eccentricity
     *   squared.
     * @return \e f = (\e a &minus; \e b) / \e a, the flattening.
     *
     * <i>e</i><sup>2</sup> should lie in (&minus;&infin;, 1).
     * The returned value \e f lies in (&minus;&infin;, 1).
     **********************************************************************/
    static Math::real EccentricitySqToFlattening(real e2)
    { using std::sqrt; return e2 / (sqrt(1 - e2) + 1); }

    /**
     * @param[in] f = (\e a &minus; \e b) / \e a, the flattening.
     * @return <i>e</i><sup>2</sup> = (<i>a</i><sup>2</sup> &minus;
     *   <i>b</i><sup>2</sup>) / <i>a</i><sup>2</sup>, the eccentricity
     *   squared.
     *
     * \e f should lie in (&minus;&infin;, 1).
     * The returned value <i>e</i><sup>2</sup> lies in (&minus;&infin;, 1).
     **********************************************************************/
    static Math::real FlatteningToEccentricitySq(real f)
    { return f * (2 - f); }

    /**
     * @param[in] ep2 = <i>e'</i> <sup>2</sup> = (<i>a</i><sup>2</sup> &minus;
     *   <i>b</i><sup>2</sup>) / <i>b</i><sup>2</sup>, the second eccentricity
     *   squared.
     * @return \e f = (\e a &minus; \e b) / \e a, the flattening.
     *
     * <i>e'</i> <sup>2</sup> should lie in (&minus;1, &infin;).
     * The returned value \e f lies in (&minus;&infin;, 1).
     **********************************************************************/
    static Math::real SecondEccentricitySqToFlattening(real ep2)
    { using std::sqrt; return ep2 / (sqrt(1 + ep2) + 1 + ep2); }

    /**
     * @param[in] f = (\e a &minus; \e b) / \e a, the flattening.
     * @return <i>e'</i> <sup>2</sup> = (<i>a</i><sup>2</sup> &minus;
     *   <i>b</i><sup>2</sup>) / <i>b</i><sup>2</sup>, the second eccentricity
     *   squared.
     *
     * \e f should lie in (&minus;&infin;, 1).
     * The returned value <i>e'</i> <sup>2</sup> lies in (&minus;1, &infin;).
     **********************************************************************/
    static Math::real FlatteningToSecondEccentricitySq(real f)
    { return f * (2 - f) / Math::sq(1 - f); }

    /**
     * @param[in] epp2 = <i>e''</i> <sup>2</sup> = (<i>a</i><sup>2</sup>
     *   &minus; <i>b</i><sup>2</sup>) / (<i>a</i><sup>2</sup> +
     *   <i>b</i><sup>2</sup>), the third eccentricity squared.
     * @return \e f = (\e a &minus; \e b) / \e a, the flattening.
     *
     * <i>e''</i> <sup>2</sup> should lie in (&minus;1, 1).
     * The returned value \e f lies in (&minus;&infin;, 1).
     **********************************************************************/
    static Math::real ThirdEccentricitySqToFlattening(real epp2) {
      using std::sqrt;
      return 2 * epp2 / (sqrt((1 - epp2) * (1 + epp2)) + 1 + epp2);
    }

    /**
     * @param[in] f = (\e a &minus; \e b) / \e a, the flattening.
     * @return <i>e''</i> <sup>2</sup> = (<i>a</i><sup>2</sup> &minus;
     *   <i>b</i><sup>2</sup>) / (<i>a</i><sup>2</sup> + <i>b</i><sup>2</sup>),
     *   the third eccentricity squared.
     *
     * \e f should lie in (&minus;&infin;, 1).
     * The returned value <i>e''</i> <sup>2</sup> lies in (&minus;1, 1).
     **********************************************************************/
    static Math::real FlatteningToThirdEccentricitySq(real f)
    { return f * (2 - f) / (1 + Math::sq(1 - f)); }

    ///@}

    /**
     * A global instantiation of Ellipsoid with the parameters for the WGS84
     * ellipsoid.
     **********************************************************************/
    static const Ellipsoid& WGS84();
  };

} // namespace GeographicLib

#endif  // GEOGRAPHICLIB_ELLIPSOID_HPP
