/**
 * \file NETGeographicLib/GeoCoords.h
 * \brief Header for NETGeographicLib::GeoCoords class
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
   * \brief .NET wrapper for GeographicLib::GeoCoords.
   *
   * This class allows .NET applications to access GeographicLib::GeoCoords.
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
   * C# Example:
   * \include example-GeoCoords.cs
   * Managed C++ Example:
   * \include example-GeoCoords.cpp
   * Visual Basic Example:
   * \include example-GeoCoords.vb
   *
   * <B>INTERFACE DIFFERENCES:</B><BR>
   * The following functions are implemented as properties: MajorRadius,
   * Flattening, Latitude, Longitude, Easting, Northing, Convergence,
   * Scale, Northp, Hemisphere, Zone, AltZone, AltEasting, AltNorthing,
   * AltConvergence, and AltScale.
   **********************************************************************/
    public ref class GeoCoords
    {
        private:
        // pointer to the unmanaged  GeographicLib::GeoCoords
        GeographicLib::GeoCoords* m_pGeoCoords;

        // The finalizer frees unmanaged memory when the object is destroyed.
        !GeoCoords();
    public:
        /** \name Initializing the GeoCoords object
         **********************************************************************/
        ///@{
        /**
         * The default constructor sets the coordinate as undefined.
         **********************************************************************/
        GeoCoords();

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
         *   -  38N 339188 3701405
         *   -  897039 3708229 37N
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
         * components.  (See DMS::Decode for details.)  Thus
         * - 40d30'30&quot;
         * - 40d30'30
         * - 40&deg;30'30
         * - 40d30.5'
         * - 40d30.5
         * - 40:30:30
         * - 40:30.5
         * - 40.508333333
         * .
         * all specify the same angle.  The leading sign applies to all
         * components so -1d30 is -(1+30/60) = -1.5.  Latitudes must be in the
         * range [&minus;90&deg;, 90&deg;].  Internally longitudes are reduced
         * to the range [&minus;180&deg;, 180&deg;).
         *
         * <b>UTM/UPS parsing</b>: For UTM zones (&minus;80&deg; &le; Lat <
         * 84&deg;), the zone designator is made up of a zone number (for 1 to 60)
         * and a hemisphere letter (N or S), e.g., 38N.  The latitude zone designer
         * ([C--M] in the southern hemisphere and [N--X] in the northern) should
         * NOT be used.  (This is part of the MGRS coordinate.)  The zone
         * designator for the poles (where UPS is employed) is a hemisphere letter
         * by itself, i.e., N or S.
         *
         * <b>MGRS parsing</b> interprets the grid references as square area at the
         * specified precision (1m, 10m, 100m, etc.).  If \e centerp = true (the
         * default), the center of this square is then taken to be the precise
         * position; thus:
         * - 38SMB           = 38N 450000 3650000
         * - 38SMB4484       = 38N 444500 3684500
         * - 38SMB44148470   = 38N 444145 3684705
         * .
         * Otherwise, the "south-west" corner of the square is used, i.e.,
         * - 38SMB           = 38N 400000 3600000
         * - 38SMB4484       = 38N 444000 3684000
         * - 38SMB44148470   = 38N 444140 3684700
         **********************************************************************/
        GeoCoords(System::String^ s, bool centerp, bool longfirst );

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
        GeoCoords(double latitude, double longitude, int zone );

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
        GeoCoords(int zone, bool northp, double easting, double northing);

        /**
         * The destructor calls the finalizer.
         **********************************************************************/
        ~GeoCoords() { this->!GeoCoords(); }

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
        void Reset( System::String^ s, bool centerp, bool longfirst);

        /**
         * Reset the location in terms of geographic coordinates.  See
         * GeoCoords(double latitude, double longitude, int zone).
         *
         * @param[in] latitude (degrees).
         * @param[in] longitude (degrees).
         * @param[in] zone if specified, force the UTM/UPS representation to use a
         *   specified zone using the rules given in UTMUPS::zonespec.
         * @exception GeographicErr if \e latitude is not in [&minus;90&deg;,
         *   90&deg;].
         * @exception GeographicErr if \e zone cannot be used for this location.
         **********************************************************************/
        void Reset(double latitude, double longitude, int zone ) ;

        /**
         * Reset the location in terms of UPS/UPS coordinates.  See
         * GeoCoords(int zone, bool northp, double easting, double northing).
         *
         * @param[in] zone UTM zone (zero means UPS).
         * @param[in] northp hemisphere (true means north, false means south).
         * @param[in] easting (meters).
         * @param[in] northing (meters).
         * @exception GeographicErr if \e zone, \e easting, or \e northing is
         *   outside its allowed range.
         **********************************************************************/
        void Reset(int zone, bool northp, double easting, double northing);
        ///@}

        /** \name Querying the GeoCoords object
         **********************************************************************/
        ///@{
        /**
         * @return latitude (degrees)
         **********************************************************************/
        property double Latitude { double get(); }

        /**
         * @return longitude (degrees)
         **********************************************************************/
        property double Longitude { double get(); }

        /**
         * @return easting (meters)
         **********************************************************************/
        property double Easting { double get(); }

        /**
         * @return northing (meters)
         **********************************************************************/
        property double Northing { double get(); }

        /**
         * @return meridian convergence (degrees) for the UTM/UPS projection.
         **********************************************************************/
        property double Convergence { double get(); }

        /**
         * @return scale for the UTM/UPS projection.
         **********************************************************************/
        property double Scale { double get(); }

        /**
         * @return hemisphere (false means south, true means north).
         **********************************************************************/
        property bool Northp { bool get(); }

        /**
         * @return hemisphere letter N or S.
         **********************************************************************/
        property char Hemisphere { char get(); }

        /**
         * @return the zone corresponding to the input (return 0 for UPS).
         **********************************************************************/
        property int Zone { int get(); }

        /**
         * Gets/Sets the current alternate zone (0 = UPS).
         * @exception GeographicErr if \e zone cannot be used for this location.
         *
         * See UTMUPS::zonespec for more information on the interpretation of \e
         * zone.  Note that \e zone == UTMUPS::STANDARD (the default) use the
         * standard UPS or UTM zone, UTMUPS::MATCH does nothing retaining the
         * existing alternate representation.  Before this is called the alternate
         * zone is the input zone.
         **********************************************************************/
        property int AltZone
        {
            int get();
            void set( int zone );
        }
        ///@}

        /**
         * @return easting (meters) for alternate zone.
         **********************************************************************/
        property double AltEasting { double get(); }

        /**
         * @return northing (meters) for alternate zone.
         **********************************************************************/
        property double AltNorthing { double get(); }

        /**
         * @return meridian convergence (degrees) for alternate zone.
         **********************************************************************/
        property double AltConvergence { double get(); }

        /**
         * @return scale for alternate zone.
         **********************************************************************/
        property double AltScale { double get(); }
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
        System::String^ GeoRepresentation(int prec, bool longfirst );

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
        System::String^ DMSRepresentation(int prec, bool longfirst,
                                      char dmssep );

        /**
         * MGRS string.
         *
         * @param[in] prec precision (relative to about 1m).
         * @exception std::bad_alloc if memory for the string can't be allocated.
         * @return MGRS string.
         *
         * This gives the coordinates of the enclosing grid square with size given
         * by the precision.  Thus 38N 444180 3684790 converted to a MGRS
         * coordinate at precision &minus;2 (100m) is 38SMB441847 and not
         * 38SMB442848.  \e prec specifies the precision of the MGRS string as
         * follows:
         * - prec = &minus;5 (min), 100km
         * - prec = &minus;4, 10km
         * - prec = &minus;3, 1km
         * - prec = &minus;2, 100m
         * - prec = &minus;1, 10m
         * - prec = 0, 1m
         * - prec = 1, 0.1m
         * - prec = 6 (max), 1&mu;m
         **********************************************************************/
        System::String^ MGRSRepresentation(int prec);

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
        System::String^ UTMUPSRepresentation(int prec, bool abbrev);

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
        System::String^ UTMUPSRepresentation(bool northp, int prec, bool abbrev);

        /**
         * MGRS string for the alternate zone.  See GeoCoords::MGRSRepresentation.
         *
         * @param[in] prec precision (relative to about 1m).
         * @exception std::bad_alloc if memory for the string can't be allocated.
         * @return MGRS string.
         **********************************************************************/
        System::String^ AltMGRSRepresentation(int prec);

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
        System::String^ AltUTMUPSRepresentation(int prec, bool abbrev);

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
        System::String^ AltUTMUPSRepresentation(bool northp, int prec, bool abbrev);
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
        property double MajorRadius { double get(); }

        /**
         * @return \e f the flattening of the WGS84 ellipsoid.
         *
         * (The WGS84 value is returned because the UTM and UPS projections are
         * based on this ellipsoid.)
         **********************************************************************/
        property double Flattening { double get(); }
        ///@}
    };
} //namespace NETGeographicLib
