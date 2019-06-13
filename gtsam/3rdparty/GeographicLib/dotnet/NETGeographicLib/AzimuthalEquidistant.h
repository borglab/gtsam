/**
 * \file NETGeographicLib/AzimuthalEquidistant.h
 * \brief Header for NETGeographicLib::AzimuthalEquidistant class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#pragma once
#include "Geodesic.h"

namespace NETGeographicLib
{
  /**
   * \brief .NET wrapper for GeographicLib::AzimuthalEquidistant.
   *
   * This class allows .NET applications to access GeographicLib::AzimuthalEquidistant.
   *
   * Azimuthal equidistant projection centered at an arbitrary position on the
   * ellipsoid.  For a point in projected space (\e x, \e y), the geodesic
   * distance from the center position is hypot(\e x, \e y) and the azimuth of
   * the geodesic from the center point is atan2(\e x, \e y).  The Forward and
   * Reverse methods also return the azimuth \e azi of the geodesic at (\e x,
   * \e y) and reciprocal scale \e rk in the azimuthal direction which,
   * together with the basic properties of the projection, serve to specify
   * completely the local affine transformation between geographic and
   * projected coordinates.
   *
   * The conversions all take place using a Geodesic object (by default
   * Geodesic::WGS84).  For more information on geodesics see \ref geodesic.
   *
   * C# Example:
   * \include example-AzimuthalEquidistant.cs
   * Managed C++ Example:
   * \include example-AzimuthalEquidistant.cpp
   * Visual Basic Example:
   * \include example-AzimuthalEquidistant.vb
   *
   * <B>INTERFACE DIFFERENCES:</B><BR>
   * A default constructor is provided that assumes a WGS84 ellipsoid.
   *
   * The MajorRadius and Flattening functions are implemented as
   * properties.
   **********************************************************************/
    public ref class AzimuthalEquidistant
    {
    private:
        // Pointer to the unmanaged GeographicLib::AzimuthalEquidistant
        const GeographicLib::AzimuthalEquidistant* m_pAzimuthalEquidistant;

        // Frees the unmanaged memory when the object is destroyed.
        !AzimuthalEquidistant();
    public:
        /**
         * Default Constructor for AzimuthalEquidistant.
         * Assumes WGS84 Geodesic
         **********************************************************************/
        AzimuthalEquidistant(void);

        /**
         * Constructor for AzimuthalEquidistant.
         *
         * @param[in] earth the Geodesic object to use for geodesic calculations.
         **********************************************************************/
        AzimuthalEquidistant( Geodesic^ earth );

        /**
         * Destructor
         *
         * frees unmanaged memory.
         **********************************************************************/
        ~AzimuthalEquidistant()
        { this->!AzimuthalEquidistant(); }

        /**
         * Forward projection, from geographic to azimuthal equidistant.
         *
         * @param[in] lat0 latitude of center point of projection (degrees).
         * @param[in] lon0 longitude of center point of projection (degrees).
         * @param[in] lat latitude of point (degrees).
         * @param[in] lon longitude of point (degrees).
         * @param[out] x easting of point (meters).
         * @param[out] y northing of point (meters).
         * @param[out] azi azimuth of geodesic at point (degrees).
         * @param[out] rk reciprocal of azimuthal scale at point.
         *
         * \e lat0 and \e lat should be in the range [&minus;90&deg;, 90&deg;].
         * The scale of the projection is 1 in the "radial" direction, \e azi
         * clockwise from true north, and is 1/\e rk in the direction
         * perpendicular to this.  A call to Forward followed by a call to
         * Reverse will return the original (\e lat, \e lon) (to within
         * roundoff).
         **********************************************************************/
        void Forward(double lat0, double lon0, double lat, double lon,
                     [System::Runtime::InteropServices::Out] double% x,
                     [System::Runtime::InteropServices::Out] double% y,
                     [System::Runtime::InteropServices::Out] double% azi,
                     [System::Runtime::InteropServices::Out] double% rk);

        /**
         * Reverse projection, from azimuthal equidistant to geographic.
         *
         * @param[in] lat0 latitude of center point of projection (degrees).
         * @param[in] lon0 longitude of center point of projection (degrees).
         * @param[in] x easting of point (meters).
         * @param[in] y northing of point (meters).
         * @param[out] lat latitude of point (degrees).
         * @param[out] lon longitude of point (degrees).
         * @param[out] azi azimuth of geodesic at point (degrees).
         * @param[out] rk reciprocal of azimuthal scale at point.
         *
         * \e lat0 should be in the range [&minus;90&deg;, 90&deg;].  \e lat
         * will be in the range [&minus;90&deg;, 90&deg;] and \e lon will be in
         * the range [&minus;180&deg;, 180&deg;).  The scale of the projection
         * is 1 in the "radial" direction, \e azi clockwise from true north,
         * and is 1/\e rk in the direction perpendicular to this.  A call to
         * Reverse followed by a call to Forward will return the original (\e
         * x, \e y) (to roundoff) only if the geodesic to (\e x, \e y) is a
         * shortest path.
         **********************************************************************/
        void Reverse(double lat0, double lon0, double x, double y,
                     [System::Runtime::InteropServices::Out] double% lat,
                     [System::Runtime::InteropServices::Out] double% lon,
                     [System::Runtime::InteropServices::Out] double% azi,
                     [System::Runtime::InteropServices::Out] double% rk);

        /**
         * AzimuthalEquidistant::Forward without returning the azimuth and scale.
         **********************************************************************/
        void Forward(double lat0, double lon0, double lat, double lon,
                     [System::Runtime::InteropServices::Out] double% x,
                     [System::Runtime::InteropServices::Out] double% y);

        /**
         * AzimuthalEquidistant::Reverse without returning the azimuth and scale.
         **********************************************************************/
        void Reverse(double lat0, double lon0, double x, double y,
                     [System::Runtime::InteropServices::Out] double% lat,
                     [System::Runtime::InteropServices::Out] double% lon);

        /** \name Inspector functions
         **********************************************************************/
        ///@{
        /**
         * @return \e a the equatorial radius of the ellipsoid (meters).  This is
         *   the value inherited from the Geodesic object used in the constructor.
         **********************************************************************/
        property double MajorRadius { double get(); }

        /**
         * @return \e f the flattening of the ellipsoid.  This is the value
         *   inherited from the Geodesic object used in the constructor.
         **********************************************************************/
        property double Flattening { double get(); }
        ///@}
    };
} // namespace NETGeographicLib
