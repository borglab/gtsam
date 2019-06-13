/**
 * \file AzimuthalEquidistant.hpp
 * \brief Header for GeographicLib::AzimuthalEquidistant class
 *
 * Copyright (c) Charles Karney (2009-2016) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_AZIMUTHALEQUIDISTANT_HPP)
#define GEOGRAPHICLIB_AZIMUTHALEQUIDISTANT_HPP 1

#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/Constants.hpp>

namespace GeographicLib {

  /**
   * \brief Azimuthal equidistant projection
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
   * Geodesic::WGS84()).  For more information on geodesics see \ref geodesic.
   *
   * Example of use:
   * \include example-AzimuthalEquidistant.cpp
   *
   * <a href="GeodesicProj.1.html">GeodesicProj</a> is a command-line utility
   * providing access to the functionality of AzimuthalEquidistant, Gnomonic,
   * and CassiniSoldner.
   **********************************************************************/

  class GEOGRAPHICLIB_EXPORT AzimuthalEquidistant {
  private:
    typedef Math::real real;
    real eps_;
    Geodesic _earth;
  public:

    /**
     * Constructor for AzimuthalEquidistant.
     *
     * @param[in] earth the Geodesic object to use for geodesic calculations.
     *   By default this uses the WGS84 ellipsoid.
     **********************************************************************/
    explicit AzimuthalEquidistant(const Geodesic& earth = Geodesic::WGS84());

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
     * clockwise from true north, and is 1/\e rk in the direction perpendicular
     * to this.  A call to Forward followed by a call to Reverse will return
     * the original (\e lat, \e lon) (to within roundoff).
     **********************************************************************/
    void Forward(real lat0, real lon0, real lat, real lon,
                 real& x, real& y, real& azi, real& rk) const;

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
     * \e lat0 should be in the range [&minus;90&deg;, 90&deg;].  \e lat will
     * be in the range [&minus;90&deg;, 90&deg;] and \e lon will be in the
     * range [&minus;180&deg;, 180&deg;].  The scale of the projection is 1 in
     * the "radial" direction, \e azi clockwise from true north, and is 1/\e rk
     * in the direction perpendicular to this.  A call to Reverse followed by a
     * call to Forward will return the original (\e x, \e y) (to roundoff) only
     * if the geodesic to (\e x, \e y) is a shortest path.
     **********************************************************************/
    void Reverse(real lat0, real lon0, real x, real y,
                 real& lat, real& lon, real& azi, real& rk) const;

    /**
     * AzimuthalEquidistant::Forward without returning the azimuth and scale.
     **********************************************************************/
    void Forward(real lat0, real lon0, real lat, real lon,
                 real& x, real& y) const {
      real azi, rk;
      Forward(lat0, lon0, lat, lon, x, y, azi, rk);
    }

    /**
     * AzimuthalEquidistant::Reverse without returning the azimuth and scale.
     **********************************************************************/
    void Reverse(real lat0, real lon0, real x, real y,
                 real& lat, real& lon) const {
      real azi, rk;
      Reverse(lat0, lon0, x, y, lat, lon, azi, rk);
    }

    /** \name Inspector functions
     **********************************************************************/
    ///@{
    /**
     * @return \e a the equatorial radius of the ellipsoid (meters).  This is
     *   the value inherited from the Geodesic object used in the constructor.
     **********************************************************************/
    Math::real MajorRadius() const { return _earth.MajorRadius(); }

    /**
     * @return \e f the flattening of the ellipsoid.  This is the value
     *   inherited from the Geodesic object used in the constructor.
     **********************************************************************/
    Math::real Flattening() const { return _earth.Flattening(); }
    ///@}

  };

} // namespace GeographicLib

#endif  // GEOGRAPHICLIB_AZIMUTHALEQUIDISTANT_HPP
