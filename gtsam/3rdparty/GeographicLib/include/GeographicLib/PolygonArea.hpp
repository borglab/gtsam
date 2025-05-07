/**
 * \file PolygonArea.hpp
 * \brief Header for GeographicLib::PolygonAreaT class
 *
 * Copyright (c) Charles Karney (2010-2023) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_POLYGONAREA_HPP)
#define GEOGRAPHICLIB_POLYGONAREA_HPP 1

#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/GeodesicExact.hpp>
#include <GeographicLib/Rhumb.hpp>
#include <GeographicLib/Accumulator.hpp>

namespace GeographicLib {

  /**
   * \brief Polygon areas
   *
   * This computes the area of a polygon whose edges are geodesics using the
   * method given in Section 6 of
   * - C. F. F. Karney,
   *   <a href="https://doi.org/10.1007/s00190-012-0578-z">
   *   Algorithms for geodesics</a>,
   *   J. Geodesy <b>87</b>, 43--55 (2013);
   *   DOI: <a href="https://doi.org/10.1007/s00190-012-0578-z">
   *   10.1007/s00190-012-0578-z</a>;
   *   addenda:
   *   <a href="https://geographiclib.sourceforge.io/geod-addenda.html">
   *   geod-addenda.html</a>.
   *
   * Arbitrarily complex polygons are allowed.  In the case self-intersecting
   * of polygons the area is accumulated "algebraically", e.g., the areas of
   * the 2 loops in a figure-8 polygon will partially cancel.
   *
   * This class lets you add vertices and edges one at a time to the polygon.
   * The sequence must start with a vertex and thereafter vertices and edges
   * can be added in any order.  Any vertex after the first creates a new edge
   * which is the \e shortest geodesic from the previous vertex.  In some
   * cases there may be two or many such shortest geodesics and the area is
   * then not uniquely defined.  In this case, either add an intermediate
   * vertex or add the edge \e as an edge (by defining its direction and
   * length).
   *
   * The area and perimeter are accumulated at two times the standard floating
   * point precision to guard against the loss of accuracy with many-sided
   * polygons.  At any point you can ask for the perimeter and area so far.
   * There's an option to treat the points as defining a polyline instead of a
   * polygon; in that case, only the perimeter is computed.
   *
   * This is a templated class to allow it to be used with Geodesic,
   * GeodesicExact, and Rhumb.  GeographicLib::PolygonArea,
   * GeographicLib::PolygonAreaExact, and GeographicLib::PolygonAreaRhumb are
   * typedefs for these cases.
   *
   * For GeographicLib::PolygonArea (edges defined by Geodesic), an upper bound
   * on the error is about 0.1 m<sup>2</sup> per vertex.  However this is a
   * wildly pessimistic estimate in most cases.  A more realistic estimate of
   * the error is given by a test involving 10<sup>7</sup> approximately
   * regular polygons on the WGS84 ellipsoid.  The centers and the orientations
   * of the polygons were uniformly distributed, the number of vertices was
   * log-uniformly distributed in [3, 300], and the center to vertex distance
   * log-uniformly distributed in [0.1 m, 9000 km].
   *
   * Using double precision (the standard precision for GeographicLib), the
   * maximum error in the perimeter was 200 nm, and the maximum error in the
   * area was<pre>
   *     0.0013 m^2 for perimeter < 10 km
   *     0.0070 m^2 for perimeter < 100 km
   *     0.070 m^2 for perimeter < 1000 km
   *     0.11 m^2 for all perimeters
   * </pre>
   * The errors are given in terms of the perimeter, because it is expected
   * that the errors depend mainly on the number of edges and the edge lengths.
   *
   * Using long doubles (GEOGRPAHICLIB_PRECISION = 3), the maximum error in the
   * perimeter was 200 pm, and the maximum error in the area was<pre>
   *     0.7 mm^2 for perim < 10 km
   *     3.2 mm^2 for perimeter < 100 km
   *     21 mm^2 for perimeter < 1000 km
   *     45 mm^2 for all perimeters
   * </pre>
   *
   * @tparam GeodType the geodesic class to use.
   *
   * Example of use:
   * \include example-PolygonArea.cpp
   *
   * <a href="Planimeter.1.html">Planimeter</a> is a command-line utility
   * providing access to the functionality of PolygonAreaT.
   **********************************************************************/

  template<class GeodType = Geodesic>
  class PolygonAreaT {
  private:
    typedef Math::real real;
    GeodType _earth;
    real _area0;                // Full ellipsoid area
    bool _polyline;             // Assume polyline (don't close and skip area)
    unsigned _mask;
    unsigned _num;
    int _crossings;
    Accumulator<> _areasum, _perimetersum;
    real _lat0, _lon0, _lat1, _lon1;
    static int transit(real lon1, real lon2);
    // an alternate version of transit to deal with longitudes in the direct
    // problem.
    static int transitdirect(real lon1, real lon2);
    void Remainder(Accumulator<>& a) const { a.remainder(_area0); }
    void Remainder(real& a) const {
      using std::remainder;
      a = remainder(a, _area0);
    }
    template<typename T>
    void AreaReduce(T& area, int crossings, bool reverse, bool sign) const;
  public:

    /**
     * Constructor for PolygonAreaT.
     *
     * @param[in] earth the Geodesic object to use for geodesic calculations.
     * @param[in] polyline if true that treat the points as defining a polyline
     *   instead of a polygon (default = false).
     **********************************************************************/
    PolygonAreaT(const GeodType& earth, bool polyline = false)
      : _earth(earth)
      , _area0(_earth.EllipsoidArea())
      , _polyline(polyline)
      , _mask(GeodType::LATITUDE | GeodType::LONGITUDE | GeodType::DISTANCE |
              (_polyline ? GeodType::NONE :
               GeodType::AREA | GeodType::LONG_UNROLL))
    { Clear(); }

    /**
     * Clear PolygonAreaT, allowing a new polygon to be started.
     **********************************************************************/
    void Clear() {
      _num = 0;
      _crossings = 0;
      _areasum = 0;
      _perimetersum = 0;
      _lat0 = _lon0 = _lat1 = _lon1 = Math::NaN();
    }

    /**
     * Add a point to the polygon or polyline.
     *
     * @param[in] lat the latitude of the point (degrees).
     * @param[in] lon the longitude of the point (degrees).
     *
     * \e lat should be in the range [&minus;90&deg;, 90&deg;].
     **********************************************************************/
    void AddPoint(real lat, real lon);

    /**
     * Add an edge to the polygon or polyline.
     *
     * @param[in] azi azimuth at current point (degrees).
     * @param[in] s distance from current point to next point (meters).
     *
     * This does nothing if no points have been added yet.  Use
     * PolygonAreaT::CurrentPoint to determine the position of the new vertex.
     **********************************************************************/
    void AddEdge(real azi, real s);

    /**
     * Return the results so far.
     *
     * @param[in] reverse if true then clockwise (instead of counter-clockwise)
     *   traversal counts as a positive area.
     * @param[in] sign if true then return a signed result for the area if
     *   the polygon is traversed in the "wrong" direction instead of returning
     *   the area for the rest of the earth.
     * @param[out] perimeter the perimeter of the polygon or length of the
     *   polyline (meters).
     * @param[out] area the area of the polygon (meters<sup>2</sup>); only set
     *   if \e polyline is false in the constructor.
     * @return the number of points.
     *
     * More points can be added to the polygon after this call.
     **********************************************************************/
    unsigned Compute(bool reverse, bool sign,
                     real& perimeter, real& area) const;

    /**
     * Return the results assuming a tentative final test point is added;
     * however, the data for the test point is not saved.  This lets you report
     * a running result for the perimeter and area as the user moves the mouse
     * cursor.  Ordinary floating point arithmetic is used to accumulate the
     * data for the test point; thus the area and perimeter returned are less
     * accurate than if PolygonAreaT::AddPoint and PolygonAreaT::Compute are
     * used.
     *
     * @param[in] lat the latitude of the test point (degrees).
     * @param[in] lon the longitude of the test point (degrees).
     * @param[in] reverse if true then clockwise (instead of counter-clockwise)
     *   traversal counts as a positive area.
     * @param[in] sign if true then return a signed result for the area if
     *   the polygon is traversed in the "wrong" direction instead of returning
     *   the area for the rest of the earth.
     * @param[out] perimeter the approximate perimeter of the polygon or length
     *   of the polyline (meters).
     * @param[out] area the approximate area of the polygon
     *   (meters<sup>2</sup>); only set if polyline is false in the
     *   constructor.
     * @return the number of points.
     *
     * \e lat should be in the range [&minus;90&deg;, 90&deg;].
     **********************************************************************/
    unsigned TestPoint(real lat, real lon, bool reverse, bool sign,
                       real& perimeter, real& area) const;

    /**
     * Return the results assuming a tentative final test point is added via an
     * azimuth and distance; however, the data for the test point is not saved.
     * This lets you report a running result for the perimeter and area as the
     * user moves the mouse cursor.  Ordinary floating point arithmetic is used
     * to accumulate the data for the test point; thus the area and perimeter
     * returned are less accurate than if PolygonAreaT::AddEdge and
     * PolygonAreaT::Compute are used.
     *
     * @param[in] azi azimuth at current point (degrees).
     * @param[in] s distance from current point to final test point (meters).
     * @param[in] reverse if true then clockwise (instead of counter-clockwise)
     *   traversal counts as a positive area.
     * @param[in] sign if true then return a signed result for the area if
     *   the polygon is traversed in the "wrong" direction instead of returning
     *   the area for the rest of the earth.
     * @param[out] perimeter the approximate perimeter of the polygon or length
     *   of the polyline (meters).
     * @param[out] area the approximate area of the polygon
     *   (meters<sup>2</sup>); only set if polyline is false in the
     *   constructor.
     * @return the number of points.
     **********************************************************************/
    unsigned TestEdge(real azi, real s, bool reverse, bool sign,
                      real& perimeter, real& area) const;

    /** \name Inspector functions
     **********************************************************************/
    ///@{
    /**
     * @return \e a the equatorial radius of the ellipsoid (meters).  This is
     *   the value inherited from the Geodesic object used in the constructor.
     **********************************************************************/

    Math::real EquatorialRadius() const { return _earth.EquatorialRadius(); }

    /**
     * @return \e f the flattening of the ellipsoid.  This is the value
     *   inherited from the Geodesic object used in the constructor.
     **********************************************************************/
    Math::real Flattening() const { return _earth.Flattening(); }

    /**
     * Report the previous vertex added to the polygon or polyline.
     *
     * @param[out] lat the latitude of the point (degrees).
     * @param[out] lon the longitude of the point (degrees).
     *
     * If no points have been added, then NaNs are returned.  Otherwise, \e lon
     * will be in the range [&minus;180&deg;, 180&deg;].
     **********************************************************************/
    void CurrentPoint(real& lat, real& lon) const
    { lat = _lat1; lon = _lon1; }

    /**
     * Report the number of points currently in the polygon or polyline.
     *
     * @return the number of points.
     *
     * If no points have been added, then 0 is returned.
     **********************************************************************/
    unsigned NumberPoints() const { return _num; }

    /**
     * Report whether the current object is a polygon or a polyline.
     *
     * @return true if the object is a polyline.
     **********************************************************************/
    bool Polyline() const { return _polyline; }
    ///@}
  };

  /**
   * @relates PolygonAreaT
   *
   * Polygon areas using Geodesic.  This should be used if the flattening is
   * small.
   **********************************************************************/
  typedef PolygonAreaT<Geodesic> PolygonArea;

  /**
   * @relates PolygonAreaT
   *
   * \deprecated Polygon areas using GeodesicExact.  Instead use PolygonArea
   *   with a Geodesic object specified with \e exact = true.
   **********************************************************************/
  typedef PolygonAreaT<GeodesicExact> PolygonAreaExact;

  /**
   * @relates PolygonAreaT
   *
   * Polygon areas using Rhumb.
   **********************************************************************/
  typedef PolygonAreaT<Rhumb> PolygonAreaRhumb;

} // namespace GeographicLib

#endif  // GEOGRAPHICLIB_POLYGONAREA_HPP
