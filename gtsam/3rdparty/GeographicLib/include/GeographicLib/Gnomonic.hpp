/**
 * \file Gnomonic.hpp
 * \brief Header for GeographicLib::Gnomonic class
 *
 * Copyright (c) Charles Karney (2010-2016) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_GNOMONIC_HPP)
#define GEOGRAPHICLIB_GNOMONIC_HPP 1

#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/GeodesicLine.hpp>
#include <GeographicLib/Constants.hpp>

namespace GeographicLib {

  /**
   * \brief %Gnomonic projection
   *
   * %Gnomonic projection centered at an arbitrary position \e C on the
   * ellipsoid.  This projection is derived in Section 8 of
   * - C. F. F. Karney,
   *   <a href="https://doi.org/10.1007/s00190-012-0578-z">
   *   Algorithms for geodesics</a>,
   *   J. Geodesy <b>87</b>, 43--55 (2013);
   *   DOI: <a href="https://doi.org/10.1007/s00190-012-0578-z">
   *   10.1007/s00190-012-0578-z</a>;
   *   addenda:
   *   <a href="https://geographiclib.sourceforge.io/geod-addenda.html">
   *   geod-addenda.html</a>.
   * .
   * The projection of \e P is defined as follows: compute the geodesic line
   * from \e C to \e P; compute the reduced length \e m12, geodesic scale \e
   * M12, and &rho; = <i>m12</i>/\e M12; finally \e x = &rho; sin \e azi1; \e
   * y = &rho; cos \e azi1, where \e azi1 is the azimuth of the geodesic at \e
   * C.  The Gnomonic::Forward and Gnomonic::Reverse methods also return the
   * azimuth \e azi of the geodesic at \e P and reciprocal scale \e rk in the
   * azimuthal direction.  The scale in the radial direction if
   * 1/<i>rk</i><sup>2</sup>.
   *
   * For a sphere, &rho; is reduces to \e a tan(<i>s12</i>/<i>a</i>), where \e
   * s12 is the length of the geodesic from \e C to \e P, and the gnomonic
   * projection has the property that all geodesics appear as straight lines.
   * For an ellipsoid, this property holds only for geodesics interesting the
   * centers.  However geodesic segments close to the center are approximately
   * straight.
   *
   * Consider a geodesic segment of length \e l.  Let \e T be the point on the
   * geodesic (extended if necessary) closest to \e C the center of the
   * projection and \e t be the distance \e CT.  To lowest order, the maximum
   * deviation (as a true distance) of the corresponding gnomonic line segment
   * (i.e., with the same end points) from the geodesic is<br>
   * <br>
   * (<i>K</i>(<i>T</i>) - <i>K</i>(<i>C</i>))
   * <i>l</i><sup>2</sup> \e t / 32.<br>
   * <br>
   * where \e K is the Gaussian curvature.
   *
   * This result applies for any surface.  For an ellipsoid of revolution,
   * consider all geodesics whose end points are within a distance \e r of \e
   * C.  For a given \e r, the deviation is maximum when the latitude of \e C
   * is 45&deg;, when endpoints are a distance \e r away, and when their
   * azimuths from the center are &plusmn; 45&deg; or &plusmn; 135&deg;.
   * To lowest order in \e r and the flattening \e f, the deviation is \e f
   * (<i>r</i>/2<i>a</i>)<sup>3</sup> \e r.
   *
   * The conversions all take place using a Geodesic object (by default
   * Geodesic::WGS84()).  For more information on geodesics see \ref geodesic.
   *
   * \warning The definition of this projection for a sphere is
   * standard.  However, there is no standard for how it should be extended to
   * an ellipsoid.  The choices are:
   * - Declare that the projection is undefined for an ellipsoid.
   * - Project to a tangent plane from the center of the ellipsoid.  This
   *   causes great ellipses to appear as straight lines in the projection;
   *   i.e., it generalizes the spherical great circle to a great ellipse.
   *   This was proposed by independently by Bowring and Williams in 1997.
   * - Project to the conformal sphere with the constant of integration chosen
   *   so that the values of the latitude match for the center point and
   *   perform a central projection onto the plane tangent to the conformal
   *   sphere at the center point.  This causes normal sections through the
   *   center point to appear as straight lines in the projection; i.e., it
   *   generalizes the spherical great circle to a normal section.  This was
   *   proposed by I. G. Letoval'tsev, Generalization of the gnomonic
   *   projection for a spheroid and the principal geodetic problems involved
   *   in the alignment of surface routes, Geodesy and Aerophotography (5),
   *   271--274 (1963).
   * - The projection given here.  This causes geodesics close to the center
   *   point to appear as straight lines in the projection; i.e., it
   *   generalizes the spherical great circle to a geodesic.
   *
   * Example of use:
   * \include example-Gnomonic.cpp
   *
   * <a href="GeodesicProj.1.html">GeodesicProj</a> is a command-line utility
   * providing access to the functionality of AzimuthalEquidistant, Gnomonic,
   * and CassiniSoldner.
   **********************************************************************/

  class GEOGRAPHICLIB_EXPORT Gnomonic {
  private:
    typedef Math::real real;
    real eps0_, eps_;
    Geodesic _earth;
    real _a, _f;
    static const int numit_ = 10;
  public:

    /**
     * Constructor for Gnomonic.
     *
     * @param[in] earth the Geodesic object to use for geodesic calculations.
     *   By default this uses the WGS84 ellipsoid.
     **********************************************************************/
    explicit Gnomonic(const Geodesic& earth = Geodesic::WGS84());

    /**
     * Forward projection, from geographic to gnomonic.
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
     * The scale of the projection is 1/<i>rk</i><sup>2</sup> in the "radial"
     * direction, \e azi clockwise from true north, and is 1/\e rk in the
     * direction perpendicular to this.  If the point lies "over the horizon",
     * i.e., if \e rk &le; 0, then NaNs are returned for \e x and \e y (the
     * correct values are returned for \e azi and \e rk).  A call to Forward
     * followed by a call to Reverse will return the original (\e lat, \e lon)
     * (to within roundoff) provided the point in not over the horizon.
     **********************************************************************/
    void Forward(real lat0, real lon0, real lat, real lon,
                 real& x, real& y, real& azi, real& rk) const;

    /**
     * Reverse projection, from gnomonic to geographic.
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
     * range [&minus;180&deg;, 180&deg;].  The scale of the projection is
     * 1/<i>rk</i><sup>2</sup> in the "radial" direction, \e azi clockwise from
     * true north, and is 1/\e rk in the direction perpendicular to this.  Even
     * though all inputs should return a valid \e lat and \e lon, it's possible
     * that the procedure fails to converge for very large \e x or \e y; in
     * this case NaNs are returned for all the output arguments.  A call to
     * Reverse followed by a call to Forward will return the original (\e x, \e
     * y) (to roundoff).
     **********************************************************************/
    void Reverse(real lat0, real lon0, real x, real y,
                 real& lat, real& lon, real& azi, real& rk) const;

    /**
     * Gnomonic::Forward without returning the azimuth and scale.
     **********************************************************************/
    void Forward(real lat0, real lon0, real lat, real lon,
                 real& x, real& y) const {
      real azi, rk;
      Forward(lat0, lon0, lat, lon, x, y, azi, rk);
    }

    /**
     * Gnomonic::Reverse without returning the azimuth and scale.
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

#endif  // GEOGRAPHICLIB_GNOMONIC_HPP
