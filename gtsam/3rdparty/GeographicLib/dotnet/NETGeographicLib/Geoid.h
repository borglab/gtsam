#pragma once
/**
 * \file NETGeographicLib/Geoid.h
 * \brief Header for NETGeographicLib::Geoid class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

namespace NETGeographicLib
{
  /**
   * \brief .NET wrapper for GeographicLib::Geoid.
   *
   * This class allows .NET applications to access GeographicLib::Geoid.
   *
   * This class evaluated the height of one of the standard geoids, EGM84,
   * EGM96, or EGM2008 by bilinear or cubic interpolation into a rectangular
   * grid of data.  These geoid models are documented in
   * - EGM84:
   *   http://earth-info.nga.mil/GandG/wgs84/gravitymod/wgs84_180/wgs84_180.html
   * - EGM96:
   *   http://earth-info.nga.mil/GandG/wgs84/gravitymod/egm96/egm96.html
   * - EGM2008:
   *   http://earth-info.nga.mil/GandG/wgs84/gravitymod/egm2008
   *
   * The geoids are defined in terms of spherical harmonics.  However in order
   * to provide a quick and flexible method of evaluating the geoid heights,
   * this class evaluates the height by interpolation into a grid of
   * precomputed values.
   *
   * The geoid height, \e N, can be used to convert a height above the
   * ellipsoid, \e h, to the corresponding height above the geoid (roughly the
   * height above mean sea level), \e H, using the relations
   *
   * &nbsp;&nbsp;&nbsp;\e h = \e N + \e H;
   * &nbsp;&nbsp;\e H = &minus;\e N + \e h.
   *
   * See \ref geoid for details of how to install the data sets, the data
   * format, estimates of the interpolation errors, and how to use caching.
   *
   * This class is typically \e not thread safe in that a single instantiation
   * cannot be safely used by multiple threads because of the way the object
   * reads the data set and because it maintains a single-cell cache.  If
   * multiple threads need to calculate geoid heights they should all construct
   * thread-local instantiations.  Alternatively, set the optional \e
   * threadsafe parameter to true in the constructor.  This causes the
   * constructor to read all the data into memory and to turn off the
   * single-cell caching which results in a Geoid object which \e is thread
   * safe.
   *
   * C# Example:
   * \include example-Geoid.cs
   * Managed C++ Example:
   * \include example-Geoid.cpp
   * Visual Basic Example:
   * \include example-Geoid.vb
   *
   * <B>INTERFACE DIFFERENCES:</B><BR>
   * The () operator has been replaced with Height method.
   *
   * The following functions are implemented as properties:
   * Description, DateTime, GeoidFile, GeoidName, GeoidDirectory,
   * Interpolation, MaxError, RMSError, Offset, Scale, ThreadSafe,
   * Cache, CacheWest, CacheEast, CacheNorth, CacheSouth, MajorRadius,
   * and Flattening.
   **********************************************************************/
    public ref class Geoid
    {
        private:
        // a pointer to the unmanaged GeographicLib::Geoid.
        const GeographicLib::Geoid* m_pGeoid;

        // the finalizer frees hthe unmanaged memory when the object is destroyed.
        !Geoid(void);
    public:
        /**
         * Flags indicating conversions between heights above the geoid and heights
         * above the ellipsoid.
         **********************************************************************/
        enum class ConvertFlag {
          /**
           * The multiplier for converting from heights above the geoid to heights
           * above the ellipsoid.
           **********************************************************************/
          ELLIPSOIDTOGEOID = -1,
          /**
           * No conversion.
           **********************************************************************/
          NONE = 0,
          /**
           * The multiplier for converting from heights above the ellipsoid to
           * heights above the geoid.
           **********************************************************************/
          GEOIDTOELLIPSOID = 1,
        };

        /** \name Setting up the geoid
         **********************************************************************/
        ///@{
        /**
         * Construct a geoid.
         *
         * @param[in] name the name of the geoid.
         * @param[in] path (optional) directory for data file.
         * @param[in] cubic (optional) interpolation method; false means bilinear,
         *   true (the default) means cubic.
         * @param[in] threadsafe (optional), if true, construct a thread safe
         *   object.  The default is false
         * @exception GeographicErr if the data file cannot be found, is
         *   unreadable, or is corrupt.
         * @exception GeographicErr if \e threadsafe is true but the memory
         *   necessary for caching the data can't be allocated.
         *
         * The data file is formed by appending ".pgm" to the name.  If \e path is
         * specified (and is non-empty), then the file is loaded from directory, \e
         * path.  Otherwise the path is given by DefaultGeoidPath().  If the \e
         * threadsafe parameter is true, the data set is read into memory, the data
         * file is closed, and single-cell caching is turned off; this results in a
         * Geoid object which \e is thread safe.
         **********************************************************************/
        Geoid(System::String^ name, System::String^ path,
                       bool cubic, bool threadsafe);
        /**
         * The destructor calls the finalizer.
         **********************************************************************/
        ~Geoid()
        { this->!Geoid(); }

        /**
         * Set up a cache.
         *
         * @param[in] south latitude (degrees) of the south edge of the cached area.
         * @param[in] west longitude (degrees) of the west edge of the cached area.
         * @param[in] north latitude (degrees) of the north edge of the cached area.
         * @param[in] east longitude (degrees) of the east edge of the cached area.
         * @exception GeographicErr if the memory necessary for caching the data
         *   can't be allocated (in this case, you will have no cache and can try
         *   again with a smaller area).
         * @exception GeographicErr if there's a problem reading the data.
         * @exception GeographicErr if this is called on a threadsafe Geoid.
         *
         * Cache the data for the specified "rectangular" area bounded by the
         * parallels \e south and \e north and the meridians \e west and \e east.
         * \e east is always interpreted as being east of \e west, if necessary by
         * adding 360&deg; to its value.  \e south and \e north should be in
         * the range [&minus;90&deg;, 90&deg;].
         **********************************************************************/
        void CacheArea(double south, double west, double north, double east);

        /**
         * Cache all the data.
         *
         * @exception GeographicErr if the memory necessary for caching the data
         *   can't be allocated (in this case, you will have no cache and can try
         *   again with a smaller area).
         * @exception GeographicErr if there's a problem reading the data.
         * @exception GeographicErr if this is called on a threadsafe Geoid.
         *
         * On most computers, this is fast for data sets with grid resolution of 5'
         * or coarser.  For a 1' grid, the required RAM is 450MB; a 2.5' grid needs
         * 72MB; and a 5' grid needs 18MB.
         **********************************************************************/
        void CacheAll();

        /**
         * Clear the cache.  This never throws an error.  (This does nothing with a
         * thread safe Geoid.)
         **********************************************************************/
        void CacheClear();

        ///@}

        /** \name Compute geoid heights
         **********************************************************************/
        ///@{
        /**
         * Compute the geoid height at a point
         *
         * @param[in] lat latitude of the point (degrees).
         * @param[in] lon longitude of the point (degrees).
         * @exception GeographicErr if there's a problem reading the data; this
         *   never happens if (\e lat, \e lon) is within a successfully cached area.
         * @return geoid height (meters).
         *
         * The latitude should be in [&minus;90&deg;, 90&deg;].
         **********************************************************************/
        double Height(double lat, double lon);

        /**
         * Convert a height above the geoid to a height above the ellipsoid and
         * vice versa.
         *
         * @param[in] lat latitude of the point (degrees).
         * @param[in] lon longitude of the point (degrees).
         * @param[in] h height of the point (degrees).
         * @param[in] d a Geoid::convertflag specifying the direction of the
         *   conversion; Geoid::GEOIDTOELLIPSOID means convert a height above the
         *   geoid to a height above the ellipsoid; Geoid::ELLIPSOIDTOGEOID means
         *   convert a height above the ellipsoid to a height above the geoid.
         * @exception GeographicErr if there's a problem reading the data; this
         *   never happens if (\e lat, \e lon) is within a successfully cached area.
         * @return converted height (meters).
         **********************************************************************/
        double ConvertHeight(double lat, double lon, double h,
                                 ConvertFlag d);

        ///@}

        /** \name Inspector functions
         **********************************************************************/
        ///@{
        /**
         * @return geoid description, if available, in the data file; if
         *   absent, return "NONE".
         **********************************************************************/
        property System::String^ Description { System::String^ get(); }

        /**
         * @return date of the data file; if absent, return "UNKNOWN".
         **********************************************************************/
        property System::String^ DateTime { System::String^ get(); }

        /**
         * @return full file name used to load the geoid data.
         **********************************************************************/
        property System::String^ GeoidFile { System::String^ get(); }

        /**
         * @return "name" used to load the geoid data (from the first argument of
         *   the constructor).
         **********************************************************************/
        property System::String^ GeoidName { System::String^ get(); }

        /**
         * @return directory used to load the geoid data.
         **********************************************************************/
        property System::String^ GeoidDirectory { System::String^ get(); }

        /**
         * @return interpolation method ("cubic" or "bilinear").
         **********************************************************************/
        property System::String^ Interpolation { System::String^ get(); }

        /**
         * @return estimate of the maximum interpolation and quantization error
         *   (meters).
         *
         * This relies on the value being stored in the data file.  If the value is
         * absent, return &minus;1.
         **********************************************************************/
        property double MaxError { double get(); }

        /**
         * @return estimate of the RMS interpolation and quantization error
         *   (meters).
         *
         * This relies on the value being stored in the data file.  If the value is
         * absent, return &minus;1.
         **********************************************************************/
        property double RMSError { double get(); }

        /**
         * @return offset (meters).
         *
         * This in used in converting from the pixel values in the data file to
         * geoid heights.
         **********************************************************************/
        property double Offset { double get(); }

        /**
         * @return scale (meters).
         *
         * This in used in converting from the pixel values in the data file to
         * geoid heights.
         **********************************************************************/
        property double Scale { double get(); }

        /**
         * @return true if the object is constructed to be thread safe.
         **********************************************************************/
        property bool ThreadSafe { bool get(); }

        /**
         * @return true if a data cache is active.
         **********************************************************************/
        property bool Cache { bool get(); }

        /**
         * @return west edge of the cached area; the cache includes this edge.
         **********************************************************************/
        property double CacheWest { double get(); }

        /**
         * @return east edge of the cached area; the cache excludes this edge.
         **********************************************************************/
        property double CacheEast { double get(); }

        /**
         * @return north edge of the cached area; the cache includes this edge.
         **********************************************************************/
        property double CacheNorth { double get(); }

        /**
         * @return south edge of the cached area; the cache excludes this edge
         *   unless it's the south pole.
         **********************************************************************/
        property double CacheSouth { double get(); }

        /**
         * @return \e a the equatorial radius of the WGS84 ellipsoid (meters).
         *
         * (The WGS84 value is returned because the supported geoid models are all
         * based on this ellipsoid.)
         **********************************************************************/
        property double MajorRadius { double get(); }

        /**
         * @return \e f the flattening of the WGS84 ellipsoid.
         *
         * (The WGS84 value is returned because the supported geoid models are all
         * based on this ellipsoid.)
         **********************************************************************/
        property double Flattening { double get(); }
        ///@}

        /**
         * @return the default path for geoid data files.
         *
         * This is the value of the environment variable
         * GEOGRAPHICLIB_GEOID_PATH, if set; otherwise, it is
         * $GEOGRAPHICLIB_DATA/geoids if the environment variable
         * GEOGRAPHICLIB_DATA is set; otherwise, it is a compile-time default
         * (/usr/local/share/GeographicLib/geoids on non-Windows systems and
         * C:/ProgramData/GeographicLib/geoids on Windows systems).
         **********************************************************************/
        static System::String^ DefaultGeoidPath();

        /**
         * @return the default name for the geoid.
         *
         * This is the value of the environment variable
         * GEOGRAPHICLIB_GEOID_NAME, if set, otherwise, it is "egm96-5".  The
         * Geoid class does not use this function; it is just provided as a
         * convenience for a calling program when constructing a Geoid object.
         **********************************************************************/
        static System::String^ DefaultGeoidName();
    };
} // namespace NETGeographicLib
