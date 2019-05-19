/**
 * \file NETGeographicLib/DMS.h
 * \brief Header for NETGeographicLib::DMS class
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
   * \brief .NET wrapper for GeographicLib::DMS.
   *
   * Parse a string representing degree, minutes, and seconds and return the
   * angle in degrees and format an angle in degrees as degree, minutes, and
   * seconds.  In addition, handle NANs and infinities on input and output.
   *
   * C# Example:
   * \include example-DMS.cs
   * Managed C++ Example:
   * \include example-DMS.cpp
   * Visual Basic Example:
   * \include example-DMS.vb
   **********************************************************************/
public ref class DMS
{
public:
    /**
     * Indicator for presence of hemisphere indicator (N/S/E/W) on latitudes
     * and longitudes.
     **********************************************************************/
    enum class Flag {
      /**
       * No indicator present.
       * @hideinitializer
       **********************************************************************/
      NONE = 0,
      /**
       * Latitude indicator (N/S) present.
       * @hideinitializer
       **********************************************************************/
      LATITUDE = 1,
      /**
       * Longitude indicator (E/W) present.
       * @hideinitializer
       **********************************************************************/
      LONGITUDE = 2,
      /**
       * Used in Encode to indicate output of an azimuth in [000, 360) with no
       * letter indicator.
       * @hideinitializer
       **********************************************************************/
      AZIMUTH = 3,
      /**
       * Used in Encode to indicate output of a plain number.
       * @hideinitializer
       **********************************************************************/
      NUMBER = 4,
    };

    /**
     * Indicator for trailing units on an angle.
     **********************************************************************/
    enum class Component {
      /**
       * Trailing unit is degrees.
       * @hideinitializer
       **********************************************************************/
      DEGREE = 0,
      /**
       * Trailing unit is arc minutes.
       * @hideinitializer
       **********************************************************************/
      MINUTE = 1,
      /**
       * Trailing unit is arc seconds.
       * @hideinitializer
       **********************************************************************/
      SECOND = 2,
    };

    /**
     * Convert a string in DMS to an angle.
     *
     * @param[in] dms string input.
     * @param[out] ind a DMS::flag value signaling the presence of a
     *   hemisphere indicator.
     * @exception GeographicErr if \e dms is malformed (see below).
     * @return angle (degrees).
     *
     * Degrees, minutes, and seconds are indicated by the characters d, '
     * (single quote), &quot; (double quote), and these components may only be
     * given in this order.  Any (but not all) components may be omitted and
     * other symbols (e.g., the &deg; symbol for degrees and the unicode prime
     * and double prime symbols for minutes and seconds) may be substituted;
     * two single quotes can be used instead of &quot;.  The last component
     * indicator may be omitted and is assumed to be the next smallest unit
     * (thus 33d10 is interpreted as 33d10').  The final component may be a
     * decimal fraction but the non-final components must be integers.  Instead
     * of using d, ', and &quot; to indicate degrees, minutes, and seconds, :
     * (colon) may be used to <i>separate</i> these components (numbers must
     * appear before and after each colon); thus 50d30'10.3&quot; may be
     * written as 50:30:10.3, 5.5' may be written 0:5.5, and so on.  The
     * integer parts of the minutes and seconds components must be less
     * than 60.  A single leading sign is permitted.  A hemisphere designator
     * (N, E, W, S) may be added to the beginning or end of the string.  The
     * result is multiplied by the implied sign of the hemisphere designator
     * (negative for S and W).  In addition \e ind is set to DMS::LATITUDE if N
     * or S is present, to DMS::LONGITUDE if E or W is present, and to
     * DMS::NONE otherwise.  Throws an error on a malformed string.  No check
     * is performed on the range of the result.  Examples of legal and illegal
     * strings are
     * - <i>LEGAL</i> (all the entries on each line are equivalent)
     *   - -20.51125, 20d30'40.5&quot;S, -20&deg;30'40.5, -20d30.675,
     *     N-20d30'40.5&quot;, -20:30:40.5
     *   - 4d0'9, 4d9&quot;, 4d9'', 4:0:9, 004:00:09, 4.0025, 4.0025d, 4d0.15,
     *     04:.15
     *   - 4:59.99999999999999, 4:60.0, 4:59:59.9999999999999, 4:59:60.0, 5
     * - <i>ILLEGAL</i> (the exception thrown explains the problem)
     *   - 4d5&quot;4', 4::5, 4:5:, :4:5, 4d4.5'4&quot;, -N20.5, 1.8e2d, 4:60,
     *     4:59:60
     *
     * The decoding operation can also perform addition and subtraction
     * operations.  If the string includes <i>internal</i> signs (i.e., not at
     * the beginning nor immediately after an initial hemisphere designator),
     * then the string is split immediately before such signs and each piece is
     * decoded according to the above rules and the results added; thus
     * <code>S3-2.5+4.1N</code> is parsed as the sum of <code>S3</code>,
     * <code>-2.5</code>, <code>+4.1N</code>.  Any piece can include a
     * hemisphere designator; however, if multiple designators are given, they
     * must compatible; e.g., you cannot mix N and E.  In addition, the
     * designator can appear at the beginning or end of the first piece, but
     * must be at the end of all subsequent pieces (a hemisphere designator is
     * not allowed after the initial sign).  Examples of legal and illegal
     * combinations are
     * - <i>LEGAL</i> (these are all equivalent)
     *   - 070:00:45, 70:01:15W+0:0.5, 70:01:15W-0:0:30W, W70:01:15+0:0:30E
     * - <i>ILLEGAL</i> (the exception thrown explains the problem)
     *   - 70:01:15W+0:0:15N, W70:01:15+W0:0:15
     *
     * <b>WARNING:</b> "Exponential" notation is not recognized.  Thus
     * <code>7.0E1</code> is illegal, while <code>7.0E+1</code> is parsed as
     * <code>(7.0E) + (+1)</code>, yielding the same result as
     * <code>8.0E</code>.
     *
     * <b>NOTE:</b> At present, all the string handling in the C++
     * implementation %GeographicLib is with 8-bit characters.  The support for
     * unicode symbols for degrees, minutes, and seconds is therefore via the
     * <a href="https://en.wikipedia.org/wiki/UTF-8">UTF-8</a> encoding.  (The
     * JavaScript implementation of this class uses unicode natively, of
     * course.)
     *
     * Here is the list of Unicode symbols supported for degrees, minutes,
     * seconds:
     * - degrees:
     *   - d, D lower and upper case letters
     *   - U+00b0 degree symbol (&deg;)
     *   - U+00ba masculine ordinal indicator
     *   - U+2070 superscript zero
     *   - U+02da ring above
     * - minutes:
     *   - ' apostrophe
     *   - U+2032 prime (&prime;)
     *   - U+00b4 acute accent
     *   - U+2019 right single quote (&rsquo;)
     * - seconds:
     *   - &quot; quotation mark
     *   - U+2033 double prime (&Prime;)
     *   - U+201d right double quote (&rdquo;)
     *   - '&nbsp;' any two consecutive symbols for minutes
     * .
     * The codes with a leading zero byte, e.g., U+00b0, are accepted in their
     * UTF-8 coded form 0xc2 0xb0 and as a single byte 0xb0.
     **********************************************************************/
    static double Decode(System::String^ dms,
        [System::Runtime::InteropServices::Out] Flag% ind);

    /**
     * Convert DMS to an angle.
     *
     * @param[in] d degrees.
     * @param[in] m arc minutes.
     * @param[in] s arc seconds.
     * @return angle (degrees)
     *
     * This does not propagate the sign on \e d to the other components, so
     * -3d20' would need to be represented as - DMS::Decode(3.0, 20.0) or
     * DMS::Decode(-3.0, -20.0).
     **********************************************************************/
    static double Decode(double d, double m, double s )
    { return d + (m + s/double(60))/double(60); }

    /**
     * Convert a pair of strings to latitude and longitude.
     *
     * @param[in] dmsa first string.
     * @param[in] dmsb second string.
     * @param[out] lat latitude (degrees).
     * @param[out] lon longitude (degrees).
     * @param[in] longfirst if true assume longitude is given before latitude
     *   in the absence of hemisphere designators (default false).
     * @exception GeographicErr if \e dmsa or \e dmsb is malformed.
     * @exception GeographicErr if \e dmsa and \e dmsb are both interpreted as
     *   latitudes.
     * @exception GeographicErr if \e dmsa and \e dmsb are both interpreted as
     *   longitudes.
     * @exception GeographicErr if decoded latitude is not in [&minus;90&deg;,
     *   90&deg;].
     *
     * By default, the \e lat (resp., \e lon) is assigned to the results of
     * decoding \e dmsa (resp., \e dmsb).  However this is overridden if either
     * \e dmsa or \e dmsb contain a latitude or longitude hemisphere designator
     * (N, S, E, W).  If an exception is thrown, \e lat and \e lon are
     * unchanged.
     **********************************************************************/
    static void DecodeLatLon(System::String^ dmsa, System::String^ dmsb,
                     [System::Runtime::InteropServices::Out] double% lat,
                     [System::Runtime::InteropServices::Out] double% lon,
                     bool longfirst );

    /**
     * Convert a string to an angle in degrees.
     *
     * @param[in] angstr input string.
     * @exception GeographicErr if \e angstr is malformed.
     * @exception GeographicErr if \e angstr includes a hemisphere designator.
     * @return angle (degrees)
     *
     * No hemisphere designator is allowed and no check is done on the range of
     * the result.
     **********************************************************************/
    static double DecodeAngle(System::String^ angstr);

    /**
     * Convert a string to an azimuth in degrees.
     *
     * @param[in] azistr input string.
     * @exception GeographicErr if \e azistr is malformed.
     * @exception GeographicErr if \e azistr includes a N/S designator.
     * @return azimuth (degrees) reduced to the range [&minus;180&deg;,
     *   180&deg;).
     *
     * A hemisphere designator E/W can be used; the result is multiplied by
     * &minus;1 if W is present.
     **********************************************************************/
    static double DecodeAzimuth(System::String^ azistr);

    /**
     * Convert angle (in degrees) into a DMS string (using d, ', and &quot;).
     *
     * @param[in] angle input angle (degrees)
     * @param[in] trailing DMS::component value indicating the trailing units
     *   on the string and this is given as a decimal number if necessary.
     * @param[in] prec the number of digits after the decimal point for the
     *   trailing component.
     * @param[in] ind DMS::flag value indicated additional formatting.
     * @param[in] dmssep if non-null, use as the DMS separator character
     *   (instead of d, ', &quot; delimiters).
     * @exception GeographicErr if memory for the string can't be allocated.
     * @return formatted string
     *
     * The interpretation of \e ind is as follows:
     * - ind == DMS::NONE, signed result no leading zeros on degrees except in
     *   the units place, e.g., -8d03'.
     * - ind == DMS::LATITUDE, trailing N or S hemisphere designator, no sign,
     *   pad degrees to 2 digits, e.g., 08d03'S.
     * - ind == DMS::LONGITUDE, trailing E or W hemisphere designator, no
     *   sign, pad degrees to 3 digits, e.g., 008d03'W.
     * - ind == DMS::AZIMUTH, convert to the range [0, 360&deg;), no
     *   sign, pad degrees to 3 digits, , e.g., 351d57'.
     * .
     * The integer parts of the minutes and seconds components are always given
     * with 2 digits.
     **********************************************************************/
    static System::String^ Encode(double angle, Component trailing, unsigned prec,
                              Flag ind, char dmssep );

    /**
     * Convert angle into a DMS string (using d, ', and &quot;) selecting the
     * trailing component based on the precision.
     *
     * @param[in] angle input angle (degrees)
     * @param[in] prec the precision relative to 1 degree.
     * @param[in] ind DMS::flag value indicated additional formatting.
     * @param[in] dmssep if non-null, use as the DMS separator character
     *   (instead of d, ', &quot; delimiters).
     * @exception std::bad_alloc if memory for the string can't be allocated.
     * @return formatted string
     *
     * \e prec indicates the precision relative to 1 degree, e.g., \e prec = 3
     * gives a result accurate to 0.1' and \e prec = 4 gives a result accurate
     * to 1&quot;.  \e ind is interpreted as in DMS::Encode with the additional
     * facility that DMS::NUMBER represents \e angle as a number in fixed
     * format with precision \e prec.
     **********************************************************************/
    static System::String^ Encode(double angle, unsigned prec, Flag ind,
                              char dmssep );

    /**
     * Split angle into degrees and minutes
     *
     * @param[in] ang angle (degrees)
     * @param[out] d degrees (an integer returned as a double)
     * @param[out] m arc minutes.
     **********************************************************************/
    static void Encode(double ang,
        [System::Runtime::InteropServices::Out] double% d,
        [System::Runtime::InteropServices::Out] double% m)
    {
      d = int(ang); m = 60 * (ang - d);
    }

    /**
     * Split angle into degrees and minutes and seconds.
     *
     * @param[in] ang angle (degrees)
     * @param[out] d degrees (an integer returned as a double)
     * @param[out] m arc minutes (an integer returned as a double)
     * @param[out] s arc seconds.
     **********************************************************************/
    static void Encode(double ang,
        [System::Runtime::InteropServices::Out] double% d,
        [System::Runtime::InteropServices::Out] double% m,
        [System::Runtime::InteropServices::Out] double% s)
    {
      d = int(ang); ang = 60 * (ang - d);
      m = int(ang); s = 60 * (ang - m);
    }
};
} // namespace NETGeographicLib
