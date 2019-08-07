#pragma once
/**
 * \file NETGeographicLib/PolarStereographic.h
 * \brief Header for NETGeographicLib::PolarStereographic class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

namespace NETGeographicLib
{
    /*!
    \brief .NET wrapper for GeographicLib::PolarStereographic.

    This class allows .NET applications to access GeographicLib::PolarStereographic.
    */
  /**
   * \brief .NET wrapper for GeographicLib::PolarStereographic.
   *
   * This class allows .NET applications to access GeographicLib::PolarStereographic.
   *
   * Implementation taken from the report,
   * - J. P. Snyder,
   *   <a href="http://pubs.er.usgs.gov/usgspubs/pp/pp1395"> Map Projections: A
   *   Working Manual</a>, USGS Professional Paper 1395 (1987),
   *   pp. 160--163.
   *
   * This is a straightforward implementation of the equations in Snyder except
   * that Newton's method is used to invert the projection.
   *
   * C# Example:
   * \include example-PolarStereographic.cs
   * Managed C++ Example:
   * \include example-PolarStereographic.cpp
   * Visual Basic Example:
   * \include example-PolarStereographic.vb
   *
   * <B>INTERFACE DIFFERENCES:</B><BR>
   * A default constructor is provided that assumes WGS84 parameters and
   * a UPS scale factor.
   *
   * The MajorRadius, Flattening, and CentralScale functions are
   * implemented as properties.
   **********************************************************************/
    public ref class PolarStereographic
    {
        private:
        // pointer to the unmanaged GeographicLib::PolarStereographic
        GeographicLib::PolarStereographic* m_pPolarStereographic;

        // the finalizer frees the unmanaged memory when the object is destroyed.
        !PolarStereographic(void);
    public:

        /**
         * Constructor for a ellipsoid with
         *
         * @param[in] a equatorial radius (meters).
         * @param[in] f flattening of ellipsoid.  Setting \e f = 0 gives a sphere.
         *   Negative \e f gives a prolate ellipsoid.
         * @param[in] k0 central scale factor.
         * @exception GeographicErr if \e a, (1 &minus; \e f ) \e a, or \e k0 is
         *   not positive.
         **********************************************************************/
        PolarStereographic(double a, double f, double k0);

        /**
         * An instantiation of PolarStereographic with the WGS84 ellipsoid
         * and the UPS scale factor.
         **********************************************************************/
        PolarStereographic();

        /**
         * The destructor calls the finalizer.
         **********************************************************************/
        ~PolarStereographic()
        { this->!PolarStereographic(); }

        /**
         * Set the scale for the projection.
         *
         * @param[in] lat (degrees) assuming \e northp = true.
         * @param[in] k scale at latitude \e lat
         * @exception GeographicErr \e k is not positive.
         * @exception GeographicErr if \e lat is not in (&minus;90&deg;,
         *   90&deg;] or this object was created with the default constructor.
         **********************************************************************/
        void SetScale(double lat, double k);

        /**
         * Forward projection, from geographic to polar stereographic.
         *
         * @param[in] northp the pole which is the center of projection (true means
         *   north, false means south).
         * @param[in] lat latitude of point (degrees).
         * @param[in] lon longitude of point (degrees).
         * @param[out] x easting of point (meters).
         * @param[out] y northing of point (meters).
         * @param[out] gamma meridian convergence at point (degrees).
         * @param[out] k scale of projection at point.
         *
         * No false easting or northing is added.  \e lat should be in the range
         * (&minus;90&deg;, 90&deg;] for \e northp = true and in the range
         * [&minus;90&deg;, 90&deg;) for \e northp = false.
         **********************************************************************/
        void Forward(bool northp, double lat, double lon,
                     [System::Runtime::InteropServices::Out] double% x,
                     [System::Runtime::InteropServices::Out] double% y,
                     [System::Runtime::InteropServices::Out] double% gamma,
                     [System::Runtime::InteropServices::Out] double% k);

        /**
         * Reverse projection, from polar stereographic to geographic.
         *
         * @param[in] northp the pole which is the center of projection (true means
         *   north, false means south).
         * @param[in] x easting of point (meters).
         * @param[in] y northing of point (meters).
         * @param[out] lat latitude of point (degrees).
         * @param[out] lon longitude of point (degrees).
         * @param[out] gamma meridian convergence at point (degrees).
         * @param[out] k scale of projection at point.
         *
         * No false easting or northing is added.  The value of \e lon returned is
         * in the range [&minus;180&deg;, 180&deg;).
         **********************************************************************/
        void Reverse(bool northp, double x, double y,
                     [System::Runtime::InteropServices::Out] double% lat,
                     [System::Runtime::InteropServices::Out] double% lon,
                     [System::Runtime::InteropServices::Out] double% gamma,
                     [System::Runtime::InteropServices::Out] double% k);

        /**
         * PolarStereographic::Forward without returning the convergence and scale.
         **********************************************************************/
        void Forward(bool northp, double lat, double lon,
                     [System::Runtime::InteropServices::Out] double% x,
                     [System::Runtime::InteropServices::Out] double% y);

        /**
         * PolarStereographic::Reverse without returning the convergence and scale.
         **********************************************************************/
        void Reverse(bool northp, double x, double y,
                     [System::Runtime::InteropServices::Out] double% lat,
                     [System::Runtime::InteropServices::Out] double% lon);

        /** \name Inspector functions
         **********************************************************************/
        ///@{
        /**
         * @return \e a the equatorial radius of the ellipsoid (meters).  This is
         *   the value used in the constructor.
         **********************************************************************/
        property double MajorRadius { double get(); }

        /**
         * @return \e f the flattening of the ellipsoid.  This is the value used in
         *   the constructor.
         **********************************************************************/
        property double Flattening { double get(); }

        /**
         * The central scale for the projection.  This is the value of \e k0 used
         * in the constructor and is the scale at the pole unless overridden by
         * PolarStereographic::SetScale.
         **********************************************************************/
        property double CentralScale { double get(); }
        ///@}
    };
} //namespace NETGeographicLib
