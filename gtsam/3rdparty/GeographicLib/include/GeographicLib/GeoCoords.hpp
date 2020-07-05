/**
 * \file GeoCoords.hpp
 * \brief Header for GeographicLib::GeoCoords class
 *
 * Copyright (c) Charles Karney (2008-2016) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_GEOCOORDS_HPP)
#define GEOGRAPHICLIB_GEOCOORDS_HPP 1

#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/Constants.hpp>

namespace GeographicLib {

  /**
   * \brief Conversion between geographic coordinates
   *
   * This class stores a geographic position which may be set via the
   * constructors or Reset via
   * - latitude and longitude
   * - UTM or UPS coordinates
   * - a string representation of these or an MGRS coordinate string
   *
   * The state consists of the latitude and longitude and the supplied UTM or
   * UPS coordinates (possibly derived from the MGRS coordinates).  If latitude
   * and longitude were given then the UTM/UPS coordinates follows the standard
   * conventions.
   *
   * The mutable state consists of the UTM or UPS coordinates for a alternate
   * zone.  A method SetAltZone is provided to set the alternate UPS/UTM zone.
   *
   * Methods are provided to return the geographic coordinates, the input UTM
   * or UPS coordinates (and associated meridian convergence and scale), or
   * alternate UTM or UPS coordinates (and their associated meridian
   * convergence and scale).
   *
   * Once the input string has been parsed, you can print the result out in any
   * of the formats, decimal degrees, degrees minutes seconds, MGRS, UTM/UPS.
   *
   * Example of use:
   * \include example-GeoCoords.cpp
   *
   * <a href="GeoConvert.1.html">GeoConvert</a> is a command-line utility
   * providing access to the functionality of GeoCoords.
   **********************************************************************/
  class GEOGRAPHICLIB_EXPORT GeoCoords {
  private:
    typedef Math::real real;
    real _lat, _long, _easting, _northing, _gamma, _k;
    bool _northp;
    int _zone;                  // See UTMUPS::zonespec
    mutable real _alt_easting, _alt_northing, _alt_gamma, _alt_k;
    mutable int _alt_zone;

    void CopyToAlt() const {
      _alt_easting = _easting;
      _alt_northing = _northing;
      _alt_gamma = _gamma;
      _alt_k = _k;
      _alt_zone = _zone;
    }
    static void UTMUPSString(int zone, bool northp,
                             real easting, real northing,
                             int prec, bool abbrev, std::string& utm);
    void FixHemisphere();
  public:

    /** \name Initializing the GeoCoords object
     **********************************************************************/
    ///@{
    /**
     * The default constructor sets the coordinate as undefined.
     **********************************************************************/
    GeoCoords()
      : _lat(Math::NaN())
      , _long(Math::NaN())
      , _easting(Math::NaN())
      , _northing(Math::NaN())
      , _gamma(Math::NaN())
      , _k(Math::NaN())
      , _northp(false)
      , _zone(UTMUPS::INVALID)
    { CopyToAlt(); }

    /**
     * Construct from a string.
     *
     * @param[in] s 1-element, 2-element, or 3-element string representation of
     *   the position.
     * @param[in] centerp governs the interpretation of MGRS coordinates (see
     *   below).
     * @param[in] longfirst governs the interpretation of geographic
     *   coordinates (see below).
     * @exception GeographicErr if the \e s is malformed (see below).
     *
     * Parse as a string and interpret it as a geographic position.  The input
     * string is broken into space (or comma) separated pieces and Basic
     * decision on which format is based on number of components
     * -# MGRS
     * -# "Lat Long" or "Long Lat"
     * -# "Zone Easting Northing" or "Easting Northing Zone"
     *
     * The following inputs are approximately the same (Ar Ramadi Bridge, Iraq)
     * - Latitude and Longitude
     *   -  33.44      43.27
     *   -  N33d26.4'  E43d16.2'
     *   -  43d16'12&quot;E 33d26'24&quot;N
     *   -  43:16:12E  33:26:24
     * - MGRS
     *   -  38SLC30
     *   -  38SLC391014
     *   -  38SLC3918701405
     *   -  37SHT9708
     * - UTM
     *   -  38n 339188 3701405
     *   -  897039 3708229 37n
     *
     * <b>Latitude and Longitude parsing</b>: Latitude precedes longitude,
     * unless a N, S, E, W hemisphere designator is used on one or both
     * coordinates.  If \e longfirst = true (default is false), then
     * longitude precedes latitude in the absence of a hemisphere designator.
     * Thus (with \e longfirst = false)
     * - 40 -75
     * - N40 W75
     * - -75 N40
     * - 75W 40N
     * - E-75 -40S
     * .
     * are all the same position.  The coordinates may be given in
     * decimal degrees, degrees and decimal minutes, degrees, minutes,
     * seconds, etc.  Use d, ', and &quot; to mark off the degrees,
     * minutes and seconds.  Various alternative symbols for degrees, minutes,
     * and seconds are allowed.  Alternatively, use : to separate these
     * components.  A single addition or subtraction is allowed.  (See
     * DMS::Decode for details.)  Thus
     * - 40d30'30&quot;
     * - 40d30'30
     * - 40&deg;30'30
     * - 40d30.5'
     * - 40d30.5
     * - 40:30:30
     * - 40:30.5
     * - 40.508333333
     * - 40:30+0:0:30
     * - 40:31-0:0.5
     * .
     * all specify the same angle.  The leading sign applies to the following
     * components so -1d30 is -(1+30/60) = &minus;1.5.  However, note
     * that -1:30-0:0:15 is parsed as (-1:30) + (-0:0:15) = &minus;(1+30/60)
     * &minus; (15/3600).  Latitudes must be in the range [&minus;90&deg;,
     * 90&deg;].  Internally longitudes are reduced to the range
     * [&minus;180&deg;, 180&deg;].
     *
     * <b>UTM/UPS parsing</b>: For UTM zones (&minus;80&deg; &le; Lat <
     * 84&deg;), the zone designator is made up of a zone number (for 1 to 60)
     * and a hemisphere letter (n or s), e.g., 38n (38north can also be used).
     * The latitude band designer ([C--M] in the southern hemisphere and [N--X]
     * in the northern) should NOT be used.  (This is part of the MGRS
     * coordinate.)  The zone designator for the poles (where UPS is employed)
     * is a hemisphere letter by itself, i.e., n or s (north or south can also
     * be used).
     *
     * <b>MGRS parsing</b> interprets the grid references as square area at the
     * specified precision (1m, 10m, 100m, etc.).  If \e centerp = true (the
     * default), the center of this square is then taken to be the precise
     * position; thus:
     * - 38SMB           = 38n 450000 3650000
     * - 38SMB4484       = 38n 444500 3684500
     * - 38SMB44148470   = 38n 444145 3684705
     * .
     * Otherwise, the "south-west" corner of the square is used, i.e.,
     * - 38SMB           = 38n 400000 3600000
     * - 38SMB4484       = 38n 444000 3684000
     * - 38SMB44148470   = 38n 444140 3684700
     **********************************************************************/
    explicit GeoCoords(const std::string& s,
                       bool centerp = true, bool longfirst = false)
    { Reset(s, centerp, longfirst); }

    /**
     * Construct from geographic coordinates.
     *
     * @param[in] latitude (degrees).
     * @param[in] longitude (degrees).
     * @param[in] zone if specified, force the UTM/UPS representation to use a
     *   specified zone using the rules given in UTMUPS::zonespec.
     * @exception GeographicErr if \e latitude is not in [&minus;90&deg;,
     *   90&deg;].
     * @exception GeographicErr if \e zone cannot be used for this location.
     **********************************************************************/
    GeoCoords(real latitude, real longitude, int zone = UTMUPS::STANDARD) {
      Reset(latitude, longitude, zone);
    }

    /**
     * Construct from UTM/UPS coordinates.
     *
     * @param[in] zone UTM zone (zero means UPS).
     * @param[in] northp hemisphere (true means north, false means south).
     * @param[in] easting (meters).
     * @param[in] northing (meters).
     * @exception GeographicErr if \e zone, \e easting, or \e northing is
     *   outside its allowed range.
     **********************************************************************/
    GeoCoords(int zone, bool northp, real easting, real northing) {
      Reset(zone, northp, easting, northing);
    }

    /**
     * Reset the location from a string.  See
     * GeoCoords(const std::string& s, bool centerp, bool longfirst).
     *
     * @param[in] s 1-element, 2-element, or 3-element string representation of
     *   the position.
     * @param[in] centerp governs the interpretation of MGRS coordinates.
     * @param[in] longfirst governs the interpretation of geographic
     *   coordinates.
     * @exception GeographicErr if the \e s is malformed.
     **********************************************************************/
    void Reset(const std::string& s,
               bool centerp = true, bool longfirst = false);

    /**
     * Reset the location in terms of geographic coordinates.  See
     * GeoCoords(real latitude, real longitude, int zone).
     *
     * @param[in] latitude (degrees).
     * @param[in] longitude (degrees).
     * @param[in] zone if specified, force the UTM/UPS representation to use a
     *   specified zone using the rules given in UTMUPS::zonespec.
     * @exception GeographicErr if \e latitude is not in [&minus;90&deg;,
     *   90&deg;].
     * @exception GeographicErr if \e zone cannot be used for this location.
     **********************************************************************/
    void Reset(real latitude, real longitude, int zone = UTMUPS::STANDARD) {
      UTMUPS::Forward(latitude, longitude,
                      _zone, _northp, _easting, _northing, _gamma, _k,
                      zone);
      _lat = latitude;
      _long = longitude;
      if (_long >= 180) _long -= 360;
      else if (_long < -180) _long += 360;
      CopyToAlt();
    }

    /**
     * Reset the location in terms of UPS/UPS coordinates.  See
     * GeoCoords(int zone, bool northp, real easting, real northing).
     *
     * @param[in] zone UTM zone (zero means UPS).
     * @param[in] northp hemisphere (true means north, false means south).
     * @param[in] easting (meters).
     * @param[in] northing (meters).
     * @exception GeographicErr if \e zone, \e easting, or \e northing is
     *   outside its allowed range.
     **********************************************************************/
    void Reset(int zone, bool northp, real easting, real northing) {
      UTMUPS::Reverse(zone, northp, easting, northing,
                      _lat, _long, _gamma, _k);
      _zone = zone;
      _northp = northp;
      _easting = easting;
      _northing = northing;
      FixHemisphere();
      CopyToAlt();
    }
    ///@}

    /** \name Querying the GeoCoords object
     **********************************************************************/
    ///@{
    /**
     * @return latitude (degrees)
     **********************************************************************/
    Math::real Latitude() const { return _lat; }

    /**
     * @return longitude (degrees)
     **********************************************************************/
    Math::real Longitude() const { return _long; }

    /**
     * @return easting (meters)
     **********************************************************************/
    Math::real Easting() const { return _easting; }

    /**
     * @return northing (meters)
     **********************************************************************/
    Math::real Northing() const { return _northing; }

    /**
     * @return meridian convergence (degrees) for the UTM/UPS projection.
     **********************************************************************/
    Math::real Convergence() const { return _gamma; }

    /**
     * @return scale for the UTM/UPS projection.
     **********************************************************************/
    Math::real Scale() const { return _k; }

    /**
     * @return hemisphere (false means south, true means north).
     **********************************************************************/
    bool Northp() const { return _northp; }

    /**
     * @return hemisphere letter n or s.
     **********************************************************************/
    char Hemisphere() const { return _northp ? 'n' : 's'; }

    /**
     * @return the zone corresponding to the input (return 0 for UPS).
     **********************************************************************/
    int Zone() const { return _zone; }

    ///@}

    /** \name Setting and querying the alternate zone
     **********************************************************************/
    ///@{
    /**
     * Specify alternate zone number.
     *
     * @param[in] zone zone number for the alternate representation.
     * @exception GeographicErr if \e zone cannot be used for this location.
     *
     * See UTMUPS::zonespec for more information on the interpretation of \e
     * zone.  Note that \e zone == UTMUPS::STANDARD (the default) use the
     * standard UPS or UTM zone, UTMUPS::MATCH does nothing retaining the
     * existing alternate representation.  Before this is called the alternate
     * zone is the input zone.
     **********************************************************************/
    void SetAltZone(int zone = UTMUPS::STANDARD) const {
      if (zone == UTMUPS::MATCH)
        return;
      zone = UTMUPS::StandardZone(_lat, _long, zone);
      if (zone == _zone)
        CopyToAlt();
      else {
        bool northp;
        UTMUPS::Forward(_lat, _long,
                        _alt_zone, northp,
                        _alt_easting, _alt_northing, _alt_gamma, _alt_k,
                        zone);
      }
    }

    /**
     * @return current alternate zone (return 0 for UPS).
     **********************************************************************/
    int AltZone() const { return _alt_zone; }

    /**
     * @return easting (meters) for alternate zone.
     **********************************************************************/
    Math::real AltEasting() const { return _alt_easting; }

    /**
     * @return northing (meters) for alternate zone.
     **********************************************************************/
    Math::real AltNorthing() const { return _alt_northing; }

    /**
     * @return meridian convergence (degrees) for alternate zone.
     **********************************************************************/
    Math::real AltConvergence() const { return _alt_gamma; }

    /**
     * @return scale for alternate zone.
     **********************************************************************/
    Math::real AltScale() const { return _alt_k; }
    ///@}

    /** \name String representations of the GeoCoords object
     **********************************************************************/
    ///@{
    /**
     * String representation with latitude and longitude as signed decimal
     * degrees.
     *
     * @param[in] prec precision (relative to about 1m).
     * @param[in] longfirst if true give longitude first (default = false)
     * @exception std::bad_alloc if memory for the string can't be allocated.
     * @return decimal latitude/longitude string representation.
     *
     * Precision specifies accuracy of representation as follows:
     * - prec = &minus;5 (min), 1&deg;
     * - prec = 0, 10<sup>&minus;5</sup>&deg; (about 1m)
     * - prec = 3, 10<sup>&minus;8</sup>&deg;
     * - prec = 9 (max), 10<sup>&minus;14</sup>&deg;
     **********************************************************************/
    std::string GeoRepresentation(int prec = 0, bool longfirst = false) const;

    /**
     * String representation with latitude and longitude as degrees, minutes,
     * seconds, and hemisphere.
     *
     * @param[in] prec precision (relative to about 1m)
     * @param[in] longfirst if true give longitude first (default = false)
     * @param[in] dmssep if non-null, use as the DMS separator character
     *   (instead of d, ', &quot; delimiters).
     * @exception std::bad_alloc if memory for the string can't be allocated.
     * @return DMS latitude/longitude string representation.
     *
     * Precision specifies accuracy of representation as follows:
     * - prec = &minus;5 (min), 1&deg;
     * - prec = &minus;4, 0.1&deg;
     * - prec = &minus;3, 1'
     * - prec = &minus;2, 0.1'
     * - prec = &minus;1, 1&quot;
     * - prec = 0, 0.1&quot; (about 3m)
     * - prec = 1, 0.01&quot;
     * - prec = 10 (max), 10<sup>&minus;11</sup>&quot;
     **********************************************************************/
    std::string DMSRepresentation(int prec = 0, bool longfirst = false,
                                  char dmssep = char(0))
      const;

    /**
     * MGRS string.
     *
     * @param[in] prec precision (relative to about 1m).
     * @exception std::bad_alloc if memory for the string can't be allocated.
     * @return MGRS string.
     *
     * This gives the coordinates of the enclosing grid square with size given
     * by the precision.  Thus 38n 444180 3684790 converted to a MGRS
     * coordinate at precision &minus;2 (100m) is 38SMB441847 and not
     * 38SMB442848.  \e prec specifies the precision of the MGRS string as
     * follows:
     * - prec = &minus;6 (min), only the grid zone is returned, e.g., 38S
     * - prec = &minus;5, 100km, e.g., 38SMB
     * - prec = &minus;4, 10km
     * - prec = &minus;3, 1km
     * - prec = &minus;2, 100m
     * - prec = &minus;1, 10m
     * - prec = 0, 1m
     * - prec = 1, 0.1m
     * - prec = 6 (max), 1&mu;m
     **********************************************************************/
    std::string MGRSRepresentation(int prec = 0) const;

    /**
     * UTM/UPS string.
     *
     * @param[in] prec precision (relative to about 1m)
     * @param[in] abbrev if true (the default) use abbreviated (n/s) notation
     *   for hemisphere; otherwise spell out the hemisphere (north/south)
     * @exception std::bad_alloc if memory for the string can't be allocated.
     * @return UTM/UPS string representation: zone designator, easting, and
     *   northing.
     *
     * Precision specifies accuracy of representation as follows:
     * - prec = &minus;5 (min), 100km
     * - prec = &minus;3, 1km
     * - prec = 0, 1m
     * - prec = 3, 1mm
     * - prec = 6, 1&mu;m
     * - prec = 9 (max), 1nm
     **********************************************************************/
    std::string UTMUPSRepresentation(int prec = 0, bool abbrev = true) const;

    /**
     * UTM/UPS string with hemisphere override.
     *
     * @param[in] northp hemisphere override
     * @param[in] prec precision (relative to about 1m)
     * @param[in] abbrev if true (the default) use abbreviated (n/s) notation
     *   for hemisphere; otherwise spell out the hemisphere (north/south)
     * @exception GeographicErr if the hemisphere override attempts to change
     *   UPS N to UPS S or vice versa.
     * @exception std::bad_alloc if memory for the string can't be allocated.
     * @return UTM/UPS string representation: zone designator, easting, and
     *   northing.
     **********************************************************************/
    std::string UTMUPSRepresentation(bool northp, int prec = 0,
                                     bool abbrev = true) const;

    /**
     * MGRS string for the alternate zone.  See GeoCoords::MGRSRepresentation.
     *
     * @param[in] prec precision (relative to about 1m).
     * @exception std::bad_alloc if memory for the string can't be allocated.
     * @return MGRS string.
     **********************************************************************/
    std::string AltMGRSRepresentation(int prec = 0) const;

    /**
     * UTM/UPS string for the alternate zone.  See
     * GeoCoords::UTMUPSRepresentation.
     *
     * @param[in] prec precision (relative to about 1m)
     * @param[in] abbrev if true (the default) use abbreviated (n/s) notation
     *   for hemisphere; otherwise spell out the hemisphere (north/south)
     * @exception std::bad_alloc if memory for the string can't be allocated.
     * @return UTM/UPS string representation: zone designator, easting, and
     *   northing.
     **********************************************************************/
    std::string AltUTMUPSRepresentation(int prec = 0, bool abbrev = true)
      const;

    /**
     * UTM/UPS string for the alternate zone, with hemisphere override.
     *
     * @param[in] northp hemisphere override
     * @param[in] prec precision (relative to about 1m)
     * @param[in] abbrev if true (the default) use abbreviated (n/s) notation
     *   for hemisphere; otherwise spell out the hemisphere (north/south)
     * @exception GeographicErr if the hemisphere override attempts to change
     *   UPS n to UPS s or vice verse.
     * @exception std::bad_alloc if memory for the string can't be allocated.
     * @return UTM/UPS string representation: zone designator, easting, and
     *   northing.
     **********************************************************************/
    std::string AltUTMUPSRepresentation(bool northp, int prec = 0,
                                        bool abbrev = true) const;
    ///@}

    /** \name Inspector functions
     **********************************************************************/
    ///@{
    /**
     * @return \e a the equatorial radius of the WGS84 ellipsoid (meters).
     *
     * (The WGS84 value is returned because the UTM and UPS projections are
     * based on this ellipsoid.)
     **********************************************************************/
    Math::real MajorRadius() const { return UTMUPS::MajorRadius(); }

    /**
     * @return \e f the flattening of the WGS84 ellipsoid.
     *
     * (The WGS84 value is returned because the UTM and UPS projections are
     * based on this ellipsoid.)
     **********************************************************************/
    Math::real Flattening() const { return UTMUPS::Flattening(); }
    ///@}

  };

} // namespace GeographicLib

#endif  // GEOGRAPHICLIB_GEOCOORDS_HPP
