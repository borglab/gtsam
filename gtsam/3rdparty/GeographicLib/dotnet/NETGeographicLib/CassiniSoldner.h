/**
 * \file NETGeographicLib/CassiniSoldner.h
 * \brief Header for NETGeographicLib::CassiniSoldner class
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
   * \brief .NET wrapper for GeographicLib::CassiniSoldner.
   *
   * This class allows .NET applications to access GeographicLib::CassiniSoldner.
   *
   * Cassini-Soldner projection centered at an arbitrary position, \e lat0, \e
   * lon0, on the ellipsoid.  This projection is a transverse cylindrical
   * equidistant projection.  The projection from (\e lat, \e lon) to easting
   * and northing (\e x, \e y) is defined by geodesics as follows.  Go north
   * along a geodesic a distance \e y from the central point; then turn
   * clockwise 90&deg; and go a distance \e x along a geodesic.
   * (Although the initial heading is north, this changes to south if the pole
   * is crossed.)  This procedure uniquely defines the reverse projection.  The
   * forward projection is constructed as follows.  Find the point (\e lat1, \e
   * lon1) on the meridian closest to (\e lat, \e lon).  Here we consider the
   * full meridian so that \e lon1 may be either \e lon0 or \e lon0 +
   * 180&deg;.  \e x is the geodesic distance from (\e lat1, \e lon1) to
   * (\e lat, \e lon), appropriately signed according to which side of the
   * central meridian (\e lat, \e lon) lies.  \e y is the shortest distance
   * along the meridian from (\e lat0, \e lon0) to (\e lat1, \e lon1), again,
   * appropriately signed according to the initial heading.  [Note that, in the
   * case of prolate ellipsoids, the shortest meridional path from (\e lat0, \e
   * lon0) to (\e lat1, \e lon1) may not be the shortest path.]  This procedure
   * uniquely defines the forward projection except for a small class of points
   * for which there may be two equally short routes for either leg of the
   * path.
   *
   * Because of the properties of geodesics, the (\e x, \e y) grid is
   * orthogonal.  The scale in the easting direction is unity.  The scale, \e
   * k, in the northing direction is unity on the central meridian and
   * increases away from the central meridian.  The projection routines return
   * \e azi, the true bearing of the easting direction, and \e rk = 1/\e k, the
   * reciprocal of the scale in the northing direction.
   *
   * The conversions all take place using a Geodesic object (by default
   * Geodesic::WGS84).  For more information on geodesics see \ref geodesic.
   * The determination of (\e lat1, \e lon1) in the forward projection is by
   * solving the inverse geodesic problem for (\e lat, \e lon) and its twin
   * obtained by reflection in the meridional plane.  The scale is found by
   * determining where two neighboring geodesics intersecting the central
   * meridian at \e lat1 and \e lat1 + \e dlat1 intersect and taking the ratio
   * of the reduced lengths for the two geodesics between that point and,
   * respectively, (\e lat1, \e lon1) and (\e lat, \e lon).
   *
   * C# Example:
   * \include example-CassiniSoldner.cs
   * Managed C++ Example:
   * \include example-CassiniSoldner.cpp
   * Visual Basic Example:
   * \include example-CassiniSoldner.vb
   *
   * <B>INTERFACE DIFFERENCES:</B><BR>
   * The LatitudeOrigin, LongitudeOrigin, MajorRadius and Flattening
   * functions are implimented as properties.
   **********************************************************************/
    public ref class CassiniSoldner
    {
        private:
        // A pointer to the unmanaged GeographicLib::CassiniSoldner
        GeographicLib::CassiniSoldner* m_pCassiniSoldner;

        // The finalizer frees the unmanaged memory when the object is destroyed.
        !CassiniSoldner();
    public:
        /**
         * Constructor for CassiniSoldner specifying a center point and
         * assuming the WGS84 ellipsoid.
         *
         * @param[in] lat0 latitude of center point of projection (degrees).
         * @param[in] lon0 longitude of center point of projection (degrees).
         **********************************************************************/
        CassiniSoldner(double lat0, double lon0);

        /**
         * Constructor for CassiniSoldner specifying a center point.
         *
         * @param[in] lat0 latitude of center point of projection (degrees).
         * @param[in] lon0 longitude of center point of projection (degrees).
         * @param[in] earth the Geodesic object to use for geodesic calculations.
         *   By default this uses the WGS84 ellipsoid.
         *
         * \e lat0 should be in the range [&minus;90&deg;, 90&deg;].
         **********************************************************************/
        CassiniSoldner(double lat0, double lon0, Geodesic^ earth );

        /**
         * The destructor calls the finalizer.
         **********************************************************************/
        ~CassiniSoldner()
        { this->!CassiniSoldner(); }

        /**
         * Set the central point of the projection
         *
         * @param[in] lat0 latitude of center point of projection (degrees).
         * @param[in] lon0 longitude of center point of projection (degrees).
         *
         * \e lat0 should be in the range [&minus;90&deg;, 90&deg;].
         **********************************************************************/
        void Reset(double lat0, double lon0);

        /**
         * Forward projection, from geographic to Cassini-Soldner.
         *
         * @param[in] lat latitude of point (degrees).
         * @param[in] lon longitude of point (degrees).
         * @param[out] x easting of point (meters).
         * @param[out] y northing of point (meters).
         * @param[out] azi azimuth of easting direction at point (degrees).
         * @param[out] rk reciprocal of azimuthal northing scale at point.
         *
         * \e lat should be in the range [&minus;90&deg;, 90&deg;].  A call to
         * Forward followed by a call to Reverse will return the original (\e
         * lat, \e lon) (to within roundoff).  The routine does nothing if the
         * origin has not been set.
         **********************************************************************/
        void Forward(double lat, double lon,
                     [System::Runtime::InteropServices::Out] double% x,
                     [System::Runtime::InteropServices::Out] double% y,
                     [System::Runtime::InteropServices::Out] double% azi,
                     [System::Runtime::InteropServices::Out] double% rk);

        /**
         * Reverse projection, from Cassini-Soldner to geographic.
         *
         * @param[in] x easting of point (meters).
         * @param[in] y northing of point (meters).
         * @param[out] lat latitude of point (degrees).
         * @param[out] lon longitude of point (degrees).
         * @param[out] azi azimuth of easting direction at point (degrees).
         * @param[out] rk reciprocal of azimuthal northing scale at point.
         *
         * A call to Reverse followed by a call to Forward will return the original
         * (\e x, \e y) (to within roundoff), provided that \e x and \e y are
         * sufficiently small not to "wrap around" the earth.  The routine does
         * nothing if the origin has not been set.
         **********************************************************************/
        void Reverse(double x, double y,
                     [System::Runtime::InteropServices::Out] double% lat,
                     [System::Runtime::InteropServices::Out] double% lon,
                     [System::Runtime::InteropServices::Out] double% azi,
                     [System::Runtime::InteropServices::Out] double% rk);

        /**
         * CassiniSoldner::Forward without returning the azimuth and scale.
         **********************************************************************/
        void Forward(double lat, double lon,
            [System::Runtime::InteropServices::Out] double% x,
            [System::Runtime::InteropServices::Out] double% y);

        /**
         * CassiniSoldner::Reverse without returning the azimuth and scale.
         **********************************************************************/
        void Reverse(double x, double y,
            [System::Runtime::InteropServices::Out] double% lat,
            [System::Runtime::InteropServices::Out] double% lon);

        /** \name Inspector functions
         **********************************************************************/
        ///@{
        /**
         * @return \e lat0 the latitude of origin (degrees).
         **********************************************************************/
        property double LatitudeOrigin { double get(); }

        /**
         * @return \e lon0 the longitude of origin (degrees).
         **********************************************************************/
        property double LongitudeOrigin { double get(); }

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
