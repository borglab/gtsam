/**
 * \file Intersect.hpp
 * \brief Header for GeographicLib::Intersect class
 *
 * Copyright (c) Charles Karney (2023-2024) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_INTERSECT_HPP)
#define GEOGRAPHICLIB_INTERSECT_HPP 1

#include <vector>
#include <set>
#include <GeographicLib/Math.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/GeodesicLine.hpp>

namespace GeographicLib {

  /**
   * \brief %Geodesic intersections
   *
   * Find the intersections of two geodesics \e X and \e Y.  Four calling
   * sequences are supported.
   * - The geodesics are defined by a position (latitude and longitude) and an
   *   azimuth.  In this case the \e closest intersection is found.
   * - The geodesics are defined by two endpoints.  The intersection of the two
   *   segments is found.  If they don't intersect, then the closest
   *   intersection is returned.
   * - The geodesics are defined as an intersection point, a single position
   *   and two azimuths.  In this case, the next closest intersection is found.
   * - The geodesics are defined as in the first case and all intersection
   *   within a specified distance are returned.
   * .
   * In all cases the position of the intersection is given by the signed
   * displacements \e x and \e y along the geodesics from the starting point
   * (the first point in the case of a geodesic segment).  The closest
   * itersection is defined as the one that minimizes the L1 distance,
   * Intersect::Dist([<i>x</i>, <i>y</i>) = |<i>x</i>| + |<i>y</i>|.
   *
   * The routines also optionally return a coincidence indicator \e c.  This is
   * typically 0.  However if the geodesics lie on top of one another at the
   * point of intersection, then \e c is set to +1, if they are parallel, and
   * &minus;1, if they are antiparallel.
   *
   * Example of use:
   * \include example-Intersect.cpp
   *
   * <a href="IntersectTool.1.html">IntersectTool</a> is a command-line utility
   * providing access to the functionality of this class.
   *
   * This solution for intersections is described in
   * - C. F. F. Karney,<br>
   *   <a href="https://doi.org/10.1061/JSUED2.SUENG-1483">
   *   Geodesic intersections</a>,
   *   J. Surveying Eng. <b>150</b>(3), 04024005:1--9 (2024);
   *   preprint
   *   <a href="https://arxiv.org/abs/2308.00495">arxiv:2308.00495</a>.
   * .
   * It is based on the work of
   * - S. Baseldga and J. C. Martinez-Llario,
   *   <a href="https://doi.org/10.1007/s11200-017-1020-z">
   *   Intersection and point-to-line solutions for geodesics
   *   on the ellipsoid</a>,
   *   Stud. Geophys. Geod. <b>62</b>, 353--363 (2018);
   *   DOI: <a href="https://doi.org/10.1007/s11200-017-1020-z">
   *   10.1007/s11200-017-1020-z</a>.
   **********************************************************************/

  class GEOGRAPHICLIB_EXPORT Intersect {
  private:
    typedef Math::real real;
  public:
    /**
     * The type used to hold the two displacement along the geodesics.  This is
     * just a std::pair with \e x = \e first and \e y = \e second.
     **********************************************************************/
    typedef std::pair<Math::real, Math::real> Point;
    /**
     * The minimum capabilities for GeodesicLine objects which are passed to
     * this class.
     **********************************************************************/
    static const unsigned LineCaps = Geodesic::LATITUDE | Geodesic::LONGITUDE |
      Geodesic::AZIMUTH | Geodesic::REDUCEDLENGTH | Geodesic::GEODESICSCALE |
      Geodesic::DISTANCE_IN;
  private:
    static const int numit_ = 100;
    const Geodesic _geod;
    real _a, _f,                // equatorial radius, flattening
      _rR,                      // authalic radius
      _d,                       // pi*_rR
      _eps,                     // criterion for intersection + coincidence
      _tol,                     // convergence for Newton in Solve1
      _delta,                   // for equality tests, safety margin for tiling
      _t1,                      // min distance between intersections
      _t2,                      // furthest dist to closest intersection
      _t3,                      // 1/2 furthest min dist to next intersection
      _t4,                      // capture radius for spherical sol in Solve0
      _t5,                      // longest shortest geodesic
      _d1,                      // tile spacing for Closest
      _d2,                      // tile spacing for Next
      _d3;                      // tile spacing for All
    // The L1 distance
    static Math::real d1(Math::real x, Math::real y)
    { using std::fabs; return fabs(x) + fabs(y); }
    // An internal version of Point with a little more functionality
    class XPoint {
    public:
      real x, y;
      int c;
      XPoint(Math::real x, Math::real y, int c = 0)
        : x(x), y(y), c(c)
      {}
      XPoint()
        : x(Math::NaN()), y(Math::NaN()), c(0)
      {}
      XPoint(const Point& p)
        : x(p.first), y(p.second), c(0)
      {}
      XPoint& operator+=(const XPoint& p) {
        x += p.x; y += p.y;
        if (p.c) c = p.c;       // pass along a nonzero c from either operand
        return *this;
      }
      XPoint operator+(const XPoint& p) const {
        XPoint t = *this; t += p; return t;
      }
      Math::real Dist() const { return d1(x, y); }
      Math::real Dist(const XPoint& p) const { return d1(x - p.x, y - p.y); }
      Point data() const { return Point(x, y); }
    };
    // Comparing XPoints for insertions into sets, but ensure that close
    // XPoints test equal.
    class GEOGRAPHICLIB_EXPORT SetComp {
    private:
      const real _delta;
    public:
      SetComp(Math::real delta) : _delta(delta) {}
      bool eq(const XPoint& p, const XPoint& q) const {
        return d1(p.x - q.x, p.y - q.y) <= _delta;
      }
      bool operator()(const XPoint& p, const XPoint& q) const {
        return !eq(p, q) && ( p.x != q.x ? p.x < q.x : p.y < q.y );
      }
    };
    SetComp _comp;
    // For ranking XPoints by closeness
    class RankPoint {
    private:
      const real _x, _y;
    public:
      RankPoint(const Point& p0) : _x(p0.first), _y(p0.second) {}
      RankPoint(const XPoint& p0) : _x(p0.x), _y(p0.y) {}
      bool operator()(const XPoint& p, const XPoint& q) const {
        real dp = d1(p.x - _x, p.y - _y),
          dq = d1(q.x - _x, q.y - _y);
        return dp != dq ? (dp < dq) :
          (p.x != q.x ? (p.x < q.x) : (p.y < q.y));
      }
    };
    // The spherical solution
    XPoint Spherical(const GeodesicLine& lineX, const GeodesicLine& lineY,
                     const XPoint& p) const;
    // The basic algorithm
    XPoint Basic(const GeodesicLine& lineX, const GeodesicLine& lineY,
                 const XPoint& p0) const;
    // The closest intersecton
    XPoint ClosestInt(const GeodesicLine& lineX, const GeodesicLine& lineY,
                  const XPoint& p0) const;
    // The next intersecton
    XPoint NextInt(const GeodesicLine& lineX, const GeodesicLine& lineY) const;
    // Segment intersecton
    XPoint SegmentInt(const GeodesicLine& lineX, const GeodesicLine& lineY,
                      int& segmode) const;
    // All intersectons
    std::vector<XPoint>
    AllInt0(const GeodesicLine& lineX, const GeodesicLine& lineY,
           Math::real maxdist, const XPoint& p0) const;
    std::vector<Point>
    AllInternal(const GeodesicLine& lineX, const GeodesicLine& lineY,
                Math::real maxdist, const Point& p0,
                std::vector<int>& c, bool cp) const;
    // Find {semi-,}conjugate point which is close to s3.  Optional m12, M12,
    // M21 use {semi-,}conjugacy relative to point 2
    Math::real ConjugateDist(const GeodesicLine& line, Math::real s3, bool semi,
                             Math::real m12 = 0, Math::real M12 = 1,
                             Math::real M21 = 1) const;
    Math::real distpolar(Math::real lat1, Math::real* lat2 = nullptr) const;
    Math::real polarb(Math::real* lata = nullptr, Math::real* latb = nullptr)
      const;
    Math::real conjdist(Math::real azi, Math::real* ds = nullptr,
                        Math::real* sp = nullptr, Math::real* sm = nullptr)
      const;
    Math::real distoblique(Math::real* azi = nullptr, Math::real* sp = nullptr,
                           Math::real* sm = nullptr) const;
    // p is intersection point on coincident lines orientation = c; p0 is
    // origin point.  Change p to center point wrt p0, i.e, abs((p-p0)_x) =
    // abs((p-p0)_y)
    static XPoint fixcoincident(const XPoint& p0, const XPoint& p);
    static XPoint fixcoincident(const XPoint& p0, const XPoint& p, int c);
    static XPoint fixsegment(Math::real sx, Math::real sy, const XPoint& p);
    static int segmentmode(Math::real sx, Math::real sy, const XPoint& p) {
      return (p.x < 0 ? -1 : p.x <= sx ? 0 : 1) * 3
        + (p.y < 0 ? -1 : p.y <= sy ? 0 : 1);
    }
    mutable long long _cnt0, _cnt1, _cnt2, _cnt3, _cnt4;
  public:
    /** \name Constructor
     **********************************************************************/
    ///@{
    /**
     * Constructor for an ellipsoid with
     *
     * @param[in] geod a Geodesic object.  This sets the parameters \e a and \e
     *   f for the ellipsoid.
     * @exception GeographicErr if the eccentricity of the elliposdoid is too
     *   large.
     *
     * \note This class has been validated for -1/4 &le; \e f &le; 1/5.  It may
     * give satisfactory results slightly outside this range; however
     * sufficient far outside the range, some internal checks will fail and an
     * exception thrown.
     *
     * \note If |<i>f</i>| > 1/50, then the Geodesic object should be
     * constructed with \e exact = true.
     **********************************************************************/
    Intersect(const Geodesic& geod);
    ///@}

    /** \name Finding intersections
     **********************************************************************/
    ///@{
    /**
     * Find the closest intersection point, with each geodesic specified by
     *   position and azimuth.
     *
     * @param[in] latX latitude of starting point for geodesic \e X (degrees).
     * @param[in] lonX longitude of starting point for geodesic \e X  (degrees).
     * @param[in] aziX azimuth at starting point for geodesic \e X (degrees).
     * @param[in] latY latitude of starting point for geodesic \e Y (degrees).
     * @param[in] lonY longitude of starting point for geodesic \e Y  (degrees).
     * @param[in] aziY azimuth at starting point for geodesic \e Y (degrees).
     * @param[in] p0 an optional offset for the starting points (meters),
     *   default = [0,0].
     * @param[out] c optional pointer to an integer coincidence indicator.
     * @return \e p the intersection point closest to \e p0.
     *
     * The returned intersection minimizes Intersect::Dist(\e p, \e p0).
     **********************************************************************/
    Point Closest(Math::real latX, Math::real lonX, Math::real aziX,
                  Math::real latY, Math::real lonY, Math::real aziY,
                  const Point& p0 = Point(0, 0), int* c = nullptr) const;
    /**
     * Find the closest intersection point, with each geodesic given as a
     *   GeodesicLine.
     *
     * @param[in] lineX geodesic \e X.
     * @param[in] lineY geodesic \e Y.
     * @param[in] p0 an optional offset for the starting points (meters),
     *   default = [0,0].
     * @param[out] c optional pointer to an integer coincidence indicator.
     * @return \e p the intersection point closest to \e p0.
     *
     * The returned intersection minimizes Intersect::Dist(\e p, \e p0).
     *
     * \note \e lineX and \e lineY should be created with minimum capabilities
     * Intersect::LineCaps.  The methods for creating a GeodesicLine include
     * all these capabilities by default.
     **********************************************************************/
    Point Closest(const GeodesicLine& lineX, const GeodesicLine& lineY,
                  const Point& p0 = Point(0, 0), int* c = nullptr) const;
    /**
     * Find the intersection of two geodesic segments defined by their
     *   endpoints.
     *
     * @param[in] latX1 latitude of first point for segment \e X (degrees).
     * @param[in] lonX1 longitude of first point for segment \e X (degrees).
     * @param[in] latX2 latitude of second point for segment \e X (degrees).
     * @param[in] lonX2 longitude of second point for segment \e X (degrees).
     * @param[in] latY1 latitude of first point for segment \e Y (degrees).
     * @param[in] lonY1 longitude of first point for segment \e Y (degrees).
     * @param[in] latY2 latitude of second point for segment \e Y (degrees).
     * @param[in] lonY2 longitude of second point for segment \e Y (degrees).
     * @param[out] segmode an indicator equal to zero if the segments
     *   intersect (see below).
     * @param[out] c optional pointer to an integer coincidence indicator.
     * @return \e p the intersection point if the segments intersect, otherwise
     *   the intersection point closest to the midpoints of the two
     *   segments.
     *
     * \warning The results are only well defined if there's a \e unique
     * shortest geodesic between the endpoints of the two segments.
     *
     * \e segmode codes up information about the closest intersection in the
     * case where the segments intersect.  Let <i>x</i><sub>12</sub> be the
     * length of the segment \e X and \e x = <i>p</i>.first, the position of
     * the intersection on segment \e X.  Define
     * - \e k<sub><i>x</i></sub> = &minus;1, if \e x < 0,
     * - \e k<sub><i>x</i></sub> = 0,
     *   if 0 &le; \e x &le; <i>x</i><sub>12</sub>,
     * - \e k<sub><i>x</i></sub> = 1, if <i>x</i><sub>12</sub> < \e x.
     * .
     * and similarly for segment \e Y.  Then
     * \e segmode = 3 \e k<sub><i>x</i></sub> + \e k<sub><i>y</i></sub>.
     **********************************************************************/
    Point Segment(Math::real latX1, Math::real lonX1,
                  Math::real latX2, Math::real lonX2,
                  Math::real latY1, Math::real lonY1,
                  Math::real latY2, Math::real lonY2,
                  int& segmode, int* c = nullptr) const;
    /**
     * Find the intersection of two geodesic segments each defined by a
     *   GeodesicLine.
     *
     * @param[in] lineX segment \e X.
     * @param[in] lineY segment \e Y.
     * @param[out] segmode an indicator equal to zero if the segments
     *   intersect (see below).
     * @param[out] c optional pointer to an integer coincidence indicator.
     * @return \e p the intersection point if the segments intersect, otherwise
     *   the intersection point closest to the midpoints of the two
     *   segments.
     *
     * \warning \e lineX and \e lineY must represent shortest geodesics, e.g.,
     * they can be created by Geodesic::InverseLine.  The results are only well
     * defined if there's a \e unique shortest geodesic between the endpoints
     * of the two segments.
     *
     * \note \e lineX and \e lineY should be created with minimum capabilities
     * Intersect::LineCaps.  The methods for creating a GeodesicLine include
     * all these capabilities by default.
     *
     * See previous definition of Intersect::Segment for more information on \e
     * segmode.
     **********************************************************************/
    Point Segment(const GeodesicLine& lineX, const GeodesicLine& lineY,
                  int& segmode, int* c = nullptr) const;
    /**
     * Find the next closest intersection point to a given intersection,
     *   specified by position and two azimuths.
     *
     * @param[in] latX latitude of starting points for geodesics \e X and \e Y
     *   (degrees).
     * @param[in] lonX longitude of starting points for geodesics \e X and \e Y
     *   (degrees).
     * @param[in] aziX azimuth at starting point for geodesic \e X (degrees).
     * @param[in] aziY azimuth at starting point for geodesic \e Y (degrees).
     * @param[out] c optional pointer to an integer coincidence indicator.
     * @return \e p the next closest intersection point.
     *
     * The returned intersection minimizes Intersect::Dist(\e p) (excluding \e
     * p = [0,0]).
     *
     * \note Equidistant closest intersections are surprisingly common.  If
     * this may be a problem, use Intersect::All with a sufficiently large \e
     * maxdist to capture close intersections.
     **********************************************************************/
    Point Next(Math::real latX, Math::real lonX,
               Math::real aziX, Math::real aziY, int* c = nullptr) const;
    /**
     * Find the next closest intersection point to a given intersection,
     *   with each geodesic specified a GeodesicLine.
     *
     * @param[in] lineX geodesic \e X.
     * @param[in] lineY geodesic \e Y.
     * @param[out] c optional pointer to an integer coincidence indicator.
     * @return \e p the next closest intersection point.
     *
     * \warning \e lineX and \e lineY must both have the same starting point,
     * i.e., the distance between [<i>lineX</i>.Latitude(),
     * <i>lineX</i>.Longitude()] and [<i>lineY</i>.Latitude(),
     * <i>lineY</i>.Longitude()] must be zero.
     *
     * \note \e lineX and \e lineY should be created with minimum capabilities
     * Intersect::LineCaps.  The methods for creating a GeodesicLine include
     * all these capabilities by default.
     *
     * \note Equidistant closest intersections are surprisingly common.  If
     * this may be a problem, use Intersect::All with a sufficiently large \e
     * maxdist to capture close intersections.
     **********************************************************************/
    Point Next(const GeodesicLine& lineX, const GeodesicLine& lineY,
               int* c = nullptr) const;
    ///@}

    /** \name Finding all intersections
     **********************************************************************/
    ///@{
    /**
     * Find all intersections within a certain distance, with each geodesic
     *   specified by position and azimuth.
     *
     * @param[in] latX latitude of starting point for geodesic \e X (degrees).
     * @param[in] lonX longitude of starting point for geodesic \e X  (degrees).
     * @param[in] aziX azimuth at starting point for geodesic \e X (degrees).
     * @param[in] latY latitude of starting point for geodesic \e Y (degrees).
     * @param[in] lonY longitude of starting point for geodesic \e Y  (degrees).
     * @param[in] aziY azimuth at starting point for geodesic \e Y (degrees).
     * @param[in] maxdist the maximum distance for the returned intersections
     *   (meters).
     * @param[out] c vector of coincidences.
     * @param[in] p0 an optional offset for the starting points (meters),
     *   default = [0,0].
     * @return \e plist a vector for the intersections closest to \e p0.
     *
     * Each intersection point satisfies Intersect::Dist(\e p, \e p0) &le; \e
     * maxdist.  The vector of returned intersections is sorted on the distance
     * from \e p0.
     **********************************************************************/
    std::vector<Point> All(Math::real latX, Math::real lonX, Math::real aziX,
                           Math::real latY, Math::real lonY, Math::real aziY,
                           Math::real maxdist, std::vector<int>& c,
                           const Point& p0 = Point(0, 0))
      const;
    /**
     * Find all intersections within a certain distance, with each geodesic
     *   specified by position and azimuth.  Don't return vector of
     *   coincidences.
     *
     * @param[in] latX latitude of starting point for geodesic \e X (degrees).
     * @param[in] lonX longitude of starting point for geodesic \e X  (degrees).
     * @param[in] aziX azimuth at starting point for geodesic \e X (degrees).
     * @param[in] latY latitude of starting point for geodesic \e Y (degrees).
     * @param[in] lonY longitude of starting point for geodesic \e Y  (degrees).
     * @param[in] aziY azimuth at starting point for geodesic \e Y (degrees).
     * @param[in] maxdist the maximum distance for the returned intersections
     *   (meters).
     * @param[in] p0 an optional offset for the starting points (meters),
     *   default = [0,0].
     * @return \e plist a vector for the intersections closest to \e p0.
     *
     * Each intersection point satisfies Intersect::Dist(\e p, \e p0) &le; \e
     * maxdist.  The vector of returned intersections is sorted on the distance
     * from \e p0.
     **********************************************************************/
    std::vector<Point> All(Math::real latX, Math::real lonX, Math::real aziX,
                           Math::real latY, Math::real lonY, Math::real aziY,
                           Math::real maxdist, const Point& p0 = Point(0, 0))
      const;
    /**
     * Find all intersections within a certain distance, with each geodesic
     *   specified by a GeodesicLine.
     *
     * @param[in] lineX geodesic \e X.
     * @param[in] lineY geodesic \e Y.
     * @param[in] maxdist the maximum distance for the returned intersections
     *   (meters).
     * @param[out] c vector of coincidences.
     * @param[in] p0 an optional offset for the starting points (meters),
     *   default = [0,0].
     * @return \e plist a vector for the intersections closest to \e p0.
     *
     * Each intersection point satisfies Intersect::Dist(\e p, \e p0) &le; \e
     * maxdist.  The vector of returned intersections is sorted on the distance
     * from \e p0.
     *
     * \note \e lineX and \e lineY should be created with minimum capabilities
     * Intersect::LineCaps.  The methods for creating a GeodesicLine include
     * all these capabilities by default.
     **********************************************************************/
    std::vector<Point> All(const GeodesicLine& lineX, const GeodesicLine& lineY,
                           Math::real maxdist, std::vector<int>& c,
                           const Point& p0 = Point(0, 0))
      const;
    /**
     * Find all intersections within a certain distance, with each geodesic
     *   specified by a GeodesicLine.  Don't return vector or coincidences.
     *
     * @param[in] lineX geodesic \e X.
     * @param[in] lineY geodesic \e Y.
     * @param[in] maxdist the maximum distance for the returned intersections
     *   (meters).
     * @param[in] p0 an optional offset for the starting points (meters),
     *   default = [0,0].
     * @return \e plist a vector for the intersections closest to \e p0.
     *
     * Each intersection point satisfies Intersect::Dist(\e p, \e p0) &le; \e
     * maxdist.  The vector of returned intersections is sorted on the distance
     * from \e p0.
     *
     * \note \e lineX and \e lineY should be created with minimum capabilities
     * Intersect::LineCaps.  The methods for creating a GeodesicLine include
     * all these capabilities by default.
     **********************************************************************/
    std::vector<Point> All(const GeodesicLine& lineX, const GeodesicLine& lineY,
                           Math::real maxdist, const Point& p0 = Point(0, 0))
      const;
    ///@}

    /** \name Diagnostic counters
     **********************************************************************/
    ///@{
    /**
     * @return the cumulative number of invocations of **h**.
     *
     * This is a count of the number of times the spherical triangle needs to
     * be solved.  Each involves a call to Geodesic::Inverse and this is a good
     * metric for the overall cost. This counter is set to zero by the
     * constructor.
     *
     * \warning The counter is a mutable variable and so is not thread safe.
     **********************************************************************/
    long long NumInverse() const { return _cnt0; }
    /**
     * @return the cumulative number of invocations of **b**.
     *
     * This is a count of the number of invocations of the basic algorithm,
     * which is used by all the intersection methods.  This counter is set to
     * zero by the constructor.
     *
     * \warning The counter is a mutable variable and so is not thread safe.
     **********************************************************************/
    long long NumBasic() const { return _cnt1; }
    /**
     * @return the number of times intersection point was changed in
     *   Intersect::Closest and Intersect::Next.
     *
     * If this counter is incremented by just 1 in Intersect::Closest, then the
     * initial result of the basic algorithm was eventually accepted.  This
     * counter is set to zero by the constructor.
     *
     * \note This counter is also incremented by Intersect::Segment, which
     * calls Intersect::Closest.
     *
     * \warning The counter is a mutable variable and so is not thread safe.
     **********************************************************************/
    long long NumChange() const { return _cnt2; }
    /**
     * @return the number of times a corner point is checked in
     *   Intersect::Segment.
     *
     * This counter is set to zero by the constructor.
     *
     * \warning The counter is a mutable variable and so is not thread safe.
     **********************************************************************/
    long long NumCorner() const { return _cnt3; }
    /**
     * @return the number of times a corner point is returned by
     *   Intersect::Segment.
     *
     * This counter is set to zero by the constructor.
     *
     * \note A conjecture is that a corner point never results in an
     * intersection that overrides the intersection closest to the midpoints of
     * the segments; i.e., NumCorner() always returns 0.
     *
     * \warning The counter is a mutable variable and so is not thread safe.
     **********************************************************************/
    long long NumOverride() const { return _cnt4; }
    ///@}

    /** \name Insepctor function
     **********************************************************************/
    ///@{
    /**
     * @return \e geod the Geodesic object used in the constructor.
     *
     * This can be used to query Geodesic::EquatorialRadius(),
     * Geodesic::Flattening(), Geodesic::Exact(), and
     * Geodesic::EllipsoidArea().
     **********************************************************************/
    const Geodesic& GeodesicObject() const  { return _geod; }
    ///@}

    /**
     * The L1 distance.
     *
     * @param[in] p the position along geodesics \e X and \e Y.
     * @param[in] p0 [optional] the reference position, default = [0, 0].

     * @return the L1 distance of \e p from \e p0, i.e.,
     *  |<i>p</i><sub><i>x</i></sub> &minus; <i>p0</i><sub><i>x</i></sub>| +
     *  |<i>p</i><sub><i>y</i></sub> &minus; <i>p0</i><sub><i>y</i></sub>|.
     **********************************************************************/
    static Math::real Dist(const Point& p, const Point& p0 = Point(0, 0)) {
      using std::fabs;
      return fabs(p.first - p0.first) + fabs(p.second - p0.second);
    }
  };

} // namespace GeographicLib

#endif  // GEOGRAPHICLIB_INTERSECT_HPP
