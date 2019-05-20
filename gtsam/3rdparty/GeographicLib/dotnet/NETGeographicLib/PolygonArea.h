#pragma once
/**
 * \file NETGeographicLib/PolygonArea.h
 * \brief Header for NETGeographicLib::PolygonArea class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

namespace NETGeographicLib
{
    ref class Geodesic;
  /**
   * \brief .NET wrapper for GeographicLib::PolygonArea and PolygonAreaExact.
   *
   * This class allows .NET applications to access GeographicLib::PolygonArea.
   *
   * This computes the area of a geodesic polygon using the method given
   * Section 6 of
   * - C. F. F. Karney,
   *   <a href="https://doi.org/10.1007/s00190-012-0578-z">
   *   Algorithms for geodesics</a>,
   *   J. Geodesy <b>87</b>, 43--55 (2013);
   *   DOI: <a href="https://doi.org/10.1007/s00190-012-0578-z">
   *   10.1007/s00190-012-0578-z</a>;
   *   addenda: <a href="https://geographiclib.sourceforge.io/geod-addenda.html">
   *   geod-addenda.html</a>.
   *
   * This class lets you add vertices one at a time to the polygon.  The area
   * and perimeter are accumulated in two times the standard floating point
   * precision to guard against the loss of accuracy with many-sided polygons.
   * At any point you can ask for the perimeter and area so far.  There's an
   * option to treat the points as defining a polyline instead of a polygon; in
   * that case, only the perimeter is computed.
   *
   * C# Example:
   * \include example-PolygonArea.cs
   * Managed C++ Example:
   * \include example-PolygonArea.cpp
   * Visual Basic Example:
   * \include example-PolygonArea.vb
   *
   * <B>INTERFACE DIFFERENCES:</B><BR>
   * The MajorRadius and Flattening functions are implemented as properties.
   **********************************************************************/
    public ref class PolygonArea
    {
        private:
        // a pointer to the unmanaged GeographicLib::PolygonArea
        GeographicLib::PolygonArea* m_pPolygonArea;

        // the finalize frees the unmanaged memory when the object is destroyed.
        !PolygonArea(void);
    public:

        /**
         * Constructor for PolygonArea.
         *
         * @param[in] earth the Geodesic object to use for geodesic calculations.
         * @param[in] polyline if true that treat the points as defining a polyline
         *   instead of a polygon.
         **********************************************************************/
        PolygonArea(Geodesic^ earth, bool polyline );

        /**
         * Constructor for PolygonArea that assumes a WGS84 ellipsoid.
         *
         * @param[in] polyline if true that treat the points as defining a polyline
         *   instead of a polygon.
         **********************************************************************/
        PolygonArea(const bool polyline );

        /**
         * The destructor calls the finalizer.
         **********************************************************************/
        ~PolygonArea()
        { this->!PolygonArea(); }

        /**
         * Clear PolygonArea, allowing a new polygon to be started.
         **********************************************************************/
        void Clear();

        /**
         * Add a point to the polygon or polyline.
         *
         * @param[in] lat the latitude of the point (degrees).
         * @param[in] lon the longitude of the point (degrees).
         *
         * \e lat should be in the range [&minus;90&deg;, 90&deg;].
         **********************************************************************/
        void AddPoint(double lat, double lon);

        /**
         * Add an edge to the polygon or polyline.
         *
         * @param[in] azi azimuth at current point (degrees).
         * @param[in] s distance from current point to next point (meters).
         *
         * This does nothing if no points have been added yet.  Use
         * PolygonArea::CurrentPoint to determine the position of the new
         * vertex.
         **********************************************************************/
        void AddEdge(double azi, double s);

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
         **********************************************************************/
        unsigned Compute(bool reverse, bool sign,
                [System::Runtime::InteropServices::Out] double% perimeter,
                [System::Runtime::InteropServices::Out] double% area);

        /**
         * Return the results assuming a tentative final test point is added;
         * however, the data for the test point is not saved.  This lets you report
         * a running result for the perimeter and area as the user moves the mouse
         * cursor.  Ordinary floating point arithmetic is used to accumulate the
         * data for the test point; thus the area and perimeter returned are less
         * accurate than if PolygonArea::AddPoint and PolygonArea::Compute are
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
        unsigned TestPoint(double lat, double lon, bool reverse, bool sign,
                [System::Runtime::InteropServices::Out] double% perimeter,
                [System::Runtime::InteropServices::Out] double% area);

        /**
         * Return the results assuming a tentative final test point is added via an
         * azimuth and distance; however, the data for the test point is not saved.
         * This lets you report a running result for the perimeter and area as the
         * user moves the mouse cursor.  Ordinary floating point arithmetic is used
         * to accumulate the data for the test point; thus the area and perimeter
         * returned are less accurate than if PolygonArea::AddEdge and
         * PolygonArea::Compute are used.
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
        unsigned TestEdge(double azi, double s, bool reverse, bool sign,
                [System::Runtime::InteropServices::Out] double% perimeter,
                [System::Runtime::InteropServices::Out] double% area);

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

        /**
         * Report the previous vertex added to the polygon or polyline.
         *
         * @param[out] lat the latitude of the point (degrees).
         * @param[out] lon the longitude of the point (degrees).
         *
         * If no points have been added, then NaNs are returned.  Otherwise, \e lon
         * will be in the range [&minus;180&deg;, 180&deg;).
         **********************************************************************/
        void CurrentPoint([System::Runtime::InteropServices::Out] double% lat,
                          [System::Runtime::InteropServices::Out] double% lon);
        ///@}
    };

    //*************************************************************************
    // PolygonAreaExact
    //*************************************************************************
    ref class GeodesicExact;

    public ref class PolygonAreaExact
    {
        private:
        // a pointer to the unmanaged GeographicLib::PolygonArea
        GeographicLib::PolygonAreaExact* m_pPolygonArea;

        // the finalize frees the unmanaged memory when the object is destroyed.
        !PolygonAreaExact(void);
    public:

        /**
         * Constructor for PolygonArea.
         *
         * @param[in] earth the Geodesic object to use for geodesic calculations.
         * @param[in] polyline if true that treat the points as defining a polyline
         *   instead of a polygon.
         **********************************************************************/
        PolygonAreaExact(GeodesicExact^ earth, bool polyline );

        /**
         * Constructor for PolygonArea that assumes a WGS84 ellipsoid.
         *
         * @param[in] polyline if true that treat the points as defining a polyline
         *   instead of a polygon.
         **********************************************************************/
        PolygonAreaExact(const bool polyline );

        /**
         * The destructor calls the finalizer.
         **********************************************************************/
        ~PolygonAreaExact()
        { this->!PolygonAreaExact(); }

        /**
         * Clear PolygonArea, allowing a new polygon to be started.
         **********************************************************************/
        void Clear();

        /**
         * Add a point to the polygon or polyline.
         *
         * @param[in] lat the latitude of the point (degrees).
         * @param[in] lon the longitude of the point (degrees).
         *
         * \e lat should be in the range [&minus;90&deg;, 90&deg;].
         **********************************************************************/
        void AddPoint(double lat, double lon);

        /**
         * Add an edge to the polygon or polyline.
         *
         * @param[in] azi azimuth at current point (degrees).
         * @param[in] s distance from current point to next point (meters).
         *
         * This does nothing if no points have been added yet.  Use
         * PolygonArea::CurrentPoint to determine the position of the new
         * vertex.
         **********************************************************************/
        void AddEdge(double azi, double s);

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
         **********************************************************************/
        unsigned Compute(bool reverse, bool sign,
                [System::Runtime::InteropServices::Out] double% perimeter,
                [System::Runtime::InteropServices::Out] double% area);

        /**
         * Return the results assuming a tentative final test point is added;
         * however, the data for the test point is not saved.  This lets you report
         * a running result for the perimeter and area as the user moves the mouse
         * cursor.  Ordinary floating point arithmetic is used to accumulate the
         * data for the test point; thus the area and perimeter returned are less
         * accurate than if PolygonArea::AddPoint and PolygonArea::Compute are
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
        unsigned TestPoint(double lat, double lon, bool reverse, bool sign,
                [System::Runtime::InteropServices::Out] double% perimeter,
                [System::Runtime::InteropServices::Out] double% area);

        /**
         * Return the results assuming a tentative final test point is added via an
         * azimuth and distance; however, the data for the test point is not saved.
         * This lets you report a running result for the perimeter and area as the
         * user moves the mouse cursor.  Ordinary floating point arithmetic is used
         * to accumulate the data for the test point; thus the area and perimeter
         * returned are less accurate than if PolygonArea::AddEdge and
         * PolygonArea::Compute are used.
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
        unsigned TestEdge(double azi, double s, bool reverse, bool sign,
                [System::Runtime::InteropServices::Out] double% perimeter,
                [System::Runtime::InteropServices::Out] double% area);

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

        /**
         * Report the previous vertex added to the polygon or polyline.
         *
         * @param[out] lat the latitude of the point (degrees).
         * @param[out] lon the longitude of the point (degrees).
         *
         * If no points have been added, then NaNs are returned.  Otherwise, \e lon
         * will be in the range [&minus;180&deg;, 180&deg;).
         **********************************************************************/
        void CurrentPoint([System::Runtime::InteropServices::Out] double% lat,
                          [System::Runtime::InteropServices::Out] double% lon);
        ///@}
    };

    //*************************************************************************
    // PolygonAreaRhumb
    //*************************************************************************
    ref class Rhumb;

    public ref class PolygonAreaRhumb
    {
        private:
        // a pointer to the unmanaged GeographicLib::PolygonArea
        GeographicLib::PolygonAreaRhumb* m_pPolygonArea;

        // the finalize frees the unmanaged memory when the object is destroyed.
        !PolygonAreaRhumb(void);
    public:

        /**
         * Constructor for PolygonArea.
         *
         * @param[in] earth the Geodesic object to use for geodesic calculations.
         * @param[in] polyline if true that treat the points as defining a polyline
         *   instead of a polygon.
         **********************************************************************/
        PolygonAreaRhumb(Rhumb^ earth, bool polyline );

        /**
         * Constructor for PolygonArea that assumes a WGS84 ellipsoid.
         *
         * @param[in] polyline if true that treat the points as defining a polyline
         *   instead of a polygon.
         **********************************************************************/
        PolygonAreaRhumb(const bool polyline );

        /**
         * The destructor calls the finalizer.
         **********************************************************************/
        ~PolygonAreaRhumb()
        { this->!PolygonAreaRhumb(); }

        /**
         * Clear PolygonArea, allowing a new polygon to be started.
         **********************************************************************/
        void Clear();

        /**
         * Add a point to the polygon or polyline.
         *
         * @param[in] lat the latitude of the point (degrees).
         * @param[in] lon the longitude of the point (degrees).
         *
         * \e lat should be in the range [&minus;90&deg;, 90&deg;].
         **********************************************************************/
        void AddPoint(double lat, double lon);

        /**
         * Add an edge to the polygon or polyline.
         *
         * @param[in] azi azimuth at current point (degrees).
         * @param[in] s distance from current point to next point (meters).
         *
         * This does nothing if no points have been added yet.  Use
         * PolygonArea::CurrentPoint to determine the position of the new
         * vertex.
         **********************************************************************/
        void AddEdge(double azi, double s);

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
         **********************************************************************/
        unsigned Compute(bool reverse, bool sign,
                [System::Runtime::InteropServices::Out] double% perimeter,
                [System::Runtime::InteropServices::Out] double% area);

        /**
         * Return the results assuming a tentative final test point is added;
         * however, the data for the test point is not saved.  This lets you report
         * a running result for the perimeter and area as the user moves the mouse
         * cursor.  Ordinary floating point arithmetic is used to accumulate the
         * data for the test point; thus the area and perimeter returned are less
         * accurate than if PolygonArea::AddPoint and PolygonArea::Compute are
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
        unsigned TestPoint(double lat, double lon, bool reverse, bool sign,
                [System::Runtime::InteropServices::Out] double% perimeter,
                [System::Runtime::InteropServices::Out] double% area);

        /**
         * Return the results assuming a tentative final test point is added via an
         * azimuth and distance; however, the data for the test point is not saved.
         * This lets you report a running result for the perimeter and area as the
         * user moves the mouse cursor.  Ordinary floating point arithmetic is used
         * to accumulate the data for the test point; thus the area and perimeter
         * returned are less accurate than if PolygonArea::AddEdge and
         * PolygonArea::Compute are used.
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
        unsigned TestEdge(double azi, double s, bool reverse, bool sign,
                [System::Runtime::InteropServices::Out] double% perimeter,
                [System::Runtime::InteropServices::Out] double% area);

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

        /**
         * Report the previous vertex added to the polygon or polyline.
         *
         * @param[out] lat the latitude of the point (degrees).
         * @param[out] lon the longitude of the point (degrees).
         *
         * If no points have been added, then NaNs are returned.  Otherwise, \e lon
         * will be in the range [&minus;180&deg;, 180&deg;).
         **********************************************************************/
        void CurrentPoint([System::Runtime::InteropServices::Out] double% lat,
                          [System::Runtime::InteropServices::Out] double% lon);
        ///@}
    };
} // namespace NETGeographicLib
