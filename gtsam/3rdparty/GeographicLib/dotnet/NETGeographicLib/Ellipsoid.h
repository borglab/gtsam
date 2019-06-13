/**
 * \file NETGeographicLib/Ellipsoid.h
 * \brief Header for NETGeographicLib::Ellipsoid class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#pragma once

namespace NETGeographicLib
{
  /**
   * \brief .NET wrapper for GeographicLib::Ellipsoid.
   *
   * This class allows .NET applications to access GeographicLib::Ellipsoid.
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
   * C# Example:
   * \include example-Ellipsoid.cs
   * Managed C++ Example:
   * \include example-Ellipsoid.cpp
   * Visual Basic Example:
   * \include example-Ellipsoid.vb
   *
   * <B>INTERFACE DIFFERENCES:</B><BR>
   * A default constructor has been provided that assumes a WGS84 ellipsoid.
   *
   * The following functions are implemented as properties:
   * MajorRadius, MinorRadius, QuarterMeridian, Area, Volume, Flattening,
   * SecondFlattening, ThirdFlattening, EccentricitySq, SecondEccentricitySq,
   * and ThirdEccentricitySq.
   **********************************************************************/
    public ref class Ellipsoid
    {
        private:
        // A pointer to the unmanaged GeographicLib::Ellipsoid
        GeographicLib::Ellipsoid* m_pEllipsoid;

        // The finalizer frees the unmanaged memory when the object is destroyed.
        !Ellipsoid();
    public:
        /** \name Constructor
         **********************************************************************/
        ///@{

        /**
         * Constructor for a WGS84 ellipsoid
         **********************************************************************/
        Ellipsoid();

        /**
         * Constructor for a ellipsoid with
         *
         * @param[in] a equatorial radius (meters).
         * @param[in] f flattening of ellipsoid.  Setting \e f = 0 gives a sphere.
         *   Negative \e f gives a prolate ellipsoid.
         * @exception GeographicErr if \e a or (1 &minus; \e f ) \e a is not
         *   positive.
         **********************************************************************/
        Ellipsoid(double a, double f);
        ///@}

        /** The destructor calls the finalizer.
         **********************************************************************/
        ~Ellipsoid()
        { this->!Ellipsoid(); }

        /** \name %Ellipsoid dimensions.
         **********************************************************************/
        ///@{

        /**
         * @return \e a the equatorial radius of the ellipsoid (meters).  This is
         *   the value used in the constructor.
         **********************************************************************/
        property double MajorRadius { double get(); }

        /**
         * @return \e b the polar semi-axis (meters).
         **********************************************************************/
        property double MinorRadius { double get(); }

        /**
         * @return \e L the distance between the equator and a pole along a
         *   meridian (meters).  For a sphere \e L = (&pi;/2) \e a.  The radius
         *   of a sphere with the same meridian length is \e L / (&pi;/2).
         **********************************************************************/
        property double QuarterMeridian { double get(); }

        /**
         * @return \e A the total area of the ellipsoid (meters<sup>2</sup>).  For
         *   a sphere \e A = 4&pi; <i>a</i><sup>2</sup>.  The radius of a sphere
         *   with the same area is sqrt(\e A / (4&pi;)).
         **********************************************************************/
        property double Area { double get(); }

        /**
         * @return \e V the total volume of the ellipsoid (meters<sup>3</sup>).
         *   For a sphere \e V = (4&pi; / 3) <i>a</i><sup>3</sup>.  The radius of
         *   a sphere with the same volume is cbrt(\e V / (4&pi;/3)).
         **********************************************************************/
        property double Volume { double get(); }
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
        property double Flattening { double get(); }

        /**
         * @return \e f ' = (\e a &minus; \e b) / \e b, the second flattening of
         *   the ellipsoid.  This is zero, positive, or negative for a sphere,
         *   oblate ellipsoid, or prolate ellipsoid.
         **********************************************************************/
        property double SecondFlattening { double get(); }

        /**
         * @return \e n = (\e a &minus; \e b) / (\e a + \e b), the third flattening
         *   of the ellipsoid.  This is zero, positive, or negative for a sphere,
         *   oblate ellipsoid, or prolate ellipsoid.
         **********************************************************************/
        property double ThirdFlattening { double get(); }

        /**
         * @return <i>e</i><sup>2</sup> = (<i>a</i><sup>2</sup> &minus;
         *   <i>b</i><sup>2</sup>) / <i>a</i><sup>2</sup>, the eccentricity squared
         *   of the ellipsoid.  This is zero, positive, or negative for a sphere,
         *   oblate ellipsoid, or prolate ellipsoid.
         **********************************************************************/
        property double EccentricitySq { double get(); }

        /**
         * @return <i>e'</i> <sup>2</sup> = (<i>a</i><sup>2</sup> &minus;
         *   <i>b</i><sup>2</sup>) / <i>b</i><sup>2</sup>, the second eccentricity
         *   squared of the ellipsoid.  This is zero, positive, or negative for a
         *   sphere, oblate ellipsoid, or prolate ellipsoid.
         **********************************************************************/
        property double SecondEccentricitySq { double get(); }

        /**
         * @return <i>e''</i> <sup>2</sup> = (<i>a</i><sup>2</sup> &minus;
         *   <i>b</i><sup>2</sup>) / (<i>a</i><sup>2</sup> + <i>b</i><sup>2</sup>),
         *   the third eccentricity squared of the ellipsoid.  This is zero,
         *   positive, or negative for a sphere, oblate ellipsoid, or prolate
         *   ellipsoid.
         **********************************************************************/
        property double ThirdEccentricitySq { double get(); }
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
        double ParametricLatitude(double phi);

        /**
         * @param[in] beta the parametric latitude (degrees).
         * @return &phi; the geographic latitude (degrees).
         *
         * &beta; must lie in the range [&minus;90&deg;, 90&deg;]; the
         * result is undefined if this condition does not hold.  The returned value
         * &phi; lies in [&minus;90&deg;, 90&deg;].
         **********************************************************************/
        double InverseParametricLatitude(double beta);

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
        double GeocentricLatitude(double phi);

        /**
         * @param[in] theta the geocentric latitude (degrees).
         * @return &phi; the geographic latitude (degrees).
         *
         * &theta; must lie in the range [&minus;90&deg;, 90&deg;]; the
         * result is undefined if this condition does not hold.  The returned value
         * &phi; lies in [&minus;90&deg;, 90&deg;].
         **********************************************************************/
        double InverseGeocentricLatitude(double theta);

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
        double RectifyingLatitude(double phi);

        /**
         * @param[in] mu the rectifying latitude (degrees).
         * @return &phi; the geographic latitude (degrees).
         *
         * &mu; must lie in the range [&minus;90&deg;, 90&deg;]; the
         * result is undefined if this condition does not hold.  The returned value
         * &phi; lies in [&minus;90&deg;, 90&deg;].
         **********************************************************************/
        double InverseRectifyingLatitude(double mu);

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
        double AuthalicLatitude(double phi);

        /**
         * @param[in] xi the authalic latitude (degrees).
         * @return &phi; the geographic latitude (degrees).
         *
         * &xi; must lie in the range [&minus;90&deg;, 90&deg;]; the
         * result is undefined if this condition does not hold.  The returned value
         * &phi; lies in [&minus;90&deg;, 90&deg;].
         **********************************************************************/
        double InverseAuthalicLatitude(double xi);

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
        double ConformalLatitude(double phi);

        /**
         * @param[in] chi the conformal latitude (degrees).
         * @return &phi; the geographic latitude (degrees).
         *
         * &chi; must lie in the range [&minus;90&deg;, 90&deg;]; the
         * result is undefined if this condition does not hold.  The returned value
         * &phi; lies in [&minus;90&deg;, 90&deg;].
         **********************************************************************/
        double InverseConformalLatitude(double chi);

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
         * &phi; must lie in the range [&minus;90&deg;, 90&deg;]; the
         * result is undefined if this condition does not hold.
         **********************************************************************/
        double IsometricLatitude(double phi);

        /**
         * @param[in] psi the isometric latitude (degrees).
         * @return &phi; the geographic latitude (degrees).
         *
         * The returned value &phi; lies in [&minus;90&deg;, 90&deg;].
         **********************************************************************/
        double InverseIsometricLatitude(double psi);
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
        double CircleRadius(double phi);

        /**
         * @param[in] phi the geographic latitude (degrees).
         * @return \e Z = \e b sin &beta; the distance of a circle of latitude
         *   &phi; from the equator measured parallel to the ellipsoid axis
         *   (meters).
         *
         * &phi; must lie in the range [&minus;90&deg;, 90&deg;]; the
         * result is undefined if this condition does not hold.
         **********************************************************************/
        double CircleHeight(double phi);

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
        double MeridianDistance(double phi);

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
        double MeridionalCurvatureRadius(double phi);

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
        double TransverseCurvatureRadius(double phi);

        /**
         * @param[in] phi the geographic latitude (degrees).
         * @param[in] azi the angle between the meridian and the normal section
         *   (degrees).
         * @return the radius of curvature of the ellipsoid in the normal
         *   section at latitude &phi; inclined at an angle \e azi to the
         *   meridian (meters).
         *
         * &phi; must lie in the range [&minus;90&deg;, 90&deg;]; the
         * result is undefined if this condition does not hold.
         **********************************************************************/
        double NormalCurvatureRadius(double phi, double azi);
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
        static double SecondFlatteningToFlattening(double fp);

        /**
         * @param[in] f = (\e a &minus; \e b) / \e a, the flattening.
         * @return \e f ' = (\e a &minus; \e b) / \e b, the second flattening.
         *
         * \e f should lie in (&minus;&infin;, 1).
         * The returned value \e f ' lies in (&minus;1, &infin;).
         **********************************************************************/
        static double FlatteningToSecondFlattening(double f);

        /**
         * @param[in] n = (\e a &minus; \e b) / (\e a + \e b), the third
         *   flattening.
         * @return \e f = (\e a &minus; \e b) / \e a, the flattening.
         *
         * \e n should lie in (&minus;1, 1).
         * The returned value \e f lies in (&minus;&infin;, 1).
         **********************************************************************/
        static double ThirdFlatteningToFlattening(double n);

        /**
         * @param[in] f = (\e a &minus; \e b) / \e a, the flattening.
         * @return \e n = (\e a &minus; \e b) / (\e a + \e b), the third
         *   flattening.
         *
         * \e f should lie in (&minus;&infin;, 1).
         * The returned value \e n lies in (&minus;1, 1).
         **********************************************************************/
        static double FlatteningToThirdFlattening(double f);

        /**
         * @param[in] e2 = <i>e</i><sup>2</sup> = (<i>a</i><sup>2</sup> &minus;
         *   <i>b</i><sup>2</sup>) / <i>a</i><sup>2</sup>, the eccentricity
         *   squared.
         * @return \e f = (\e a &minus; \e b) / \e a, the flattening.
         *
         * <i>e</i><sup>2</sup> should lie in (&minus;&infin;, 1).
         * The returned value \e f lies in (&minus;&infin;, 1).
         **********************************************************************/
        static double EccentricitySqToFlattening(double e2);

        /**
         * @param[in] f = (\e a &minus; \e b) / \e a, the flattening.
         * @return <i>e</i><sup>2</sup> = (<i>a</i><sup>2</sup> &minus;
         *   <i>b</i><sup>2</sup>) / <i>a</i><sup>2</sup>, the eccentricity
         *   squared.
         *
         * \e f should lie in (&minus;&infin;, 1).
         * The returned value <i>e</i><sup>2</sup> lies in (&minus;&infin;, 1).
         **********************************************************************/
        static double FlatteningToEccentricitySq(double f);

        /**
         * @param[in] ep2 = <i>e'</i> <sup>2</sup> = (<i>a</i><sup>2</sup> &minus;
         *   <i>b</i><sup>2</sup>) / <i>b</i><sup>2</sup>, the second eccentricity
         *   squared.
         * @return \e f = (\e a &minus; \e b) / \e a, the flattening.
         *
         * <i>e'</i> <sup>2</sup> should lie in (&minus;1, &infin;).
         * The returned value \e f lies in (&minus;&infin;, 1).
         **********************************************************************/
        static double SecondEccentricitySqToFlattening(double ep2);

        /**
         * @param[in] f = (\e a &minus; \e b) / \e a, the flattening.
         * @return <i>e'</i> <sup>2</sup> = (<i>a</i><sup>2</sup> &minus;
         *   <i>b</i><sup>2</sup>) / <i>b</i><sup>2</sup>, the second eccentricity
         *   squared.
         *
         * \e f should lie in (&minus;&infin;, 1).
         * The returned value <i>e'</i> <sup>2</sup> lies in (&minus;1, &infin;).
         **********************************************************************/
        static double FlatteningToSecondEccentricitySq(double f);

        /**
         * @param[in] epp2 = <i>e''</i> <sup>2</sup> = (<i>a</i><sup>2</sup>
         *   &minus; <i>b</i><sup>2</sup>) / (<i>a</i><sup>2</sup> +
         *   <i>b</i><sup>2</sup>), the third eccentricity squared.
         * @return \e f = (\e a &minus; \e b) / \e a, the flattening.
         *
         * <i>e''</i> <sup>2</sup> should lie in (&minus;1, 1).
         * The returned value \e f lies in (&minus;&infin;, 1).
         **********************************************************************/
        static double ThirdEccentricitySqToFlattening(double epp2);

        /**
         * @param[in] f = (\e a &minus; \e b) / \e a, the flattening.
         * @return <i>e''</i> <sup>2</sup> = (<i>a</i><sup>2</sup> &minus;
         *   <i>b</i><sup>2</sup>) / (<i>a</i><sup>2</sup> + <i>b</i><sup>2</sup>),
         *   the third eccentricity squared.
         *
         * \e f should lie in (&minus;&infin;, 1).
         * The returned value <i>e''</i> <sup>2</sup> lies in (&minus;1, 1).
         **********************************************************************/
        static double FlatteningToThirdEccentricitySq(double f);
    };
} // namespace NETGeographicLib
