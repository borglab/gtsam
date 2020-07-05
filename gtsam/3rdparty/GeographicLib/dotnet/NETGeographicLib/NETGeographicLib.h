#pragma once
/**
 * \file NETGeographicLib/NETGeographicLib.h
 * \brief Header for NETGeographicLib::NETGeographicLib objects
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include <string>

using namespace System;

namespace NETGeographicLib
{
    enum class captype {
      CAP_NONE = 0U,
      CAP_C1   = 1U<<0,
      CAP_C1p  = 1U<<1,
      CAP_C2   = 1U<<2,
      CAP_C3   = 1U<<3,
      CAP_C4   = 1U<<4,
      CAP_ALL  = 0x1FU,
      CAP_MASK = CAP_ALL,
      OUT_ALL  = 0x7F80U,
      OUT_MASK = 0xFF80U,
    };

    /**
     * Bit masks for what calculations to do.  These masks do double duty.
     * They signify to the GeodesicLine::GeodesicLine constructor and to
     * Geodesic::Line what capabilities should be included in the GeodesicLine
     * object.  They also specify which results to return in the general
     * routines Geodesic::GenDirect and Geodesic::GenInverse routines.
     **********************************************************************/
    public enum class Mask {
      /**
       * No capabilities, no output.
       * @hideinitializer
       **********************************************************************/
      NONE          = 0U,
      /**
       * Calculate latitude \e lat2.  (It's not necessary to include this as a
       * capability to GeodesicLine because this is included by default.)
       * @hideinitializer
       **********************************************************************/
      LATITUDE      = 1U<<7  | unsigned(captype::CAP_NONE),
      /**
       * Calculate longitude \e lon2.
       * @hideinitializer
       **********************************************************************/
      LONGITUDE     = 1U<<8  | unsigned(captype::CAP_C3),
      /**
       * Calculate azimuths \e azi1 and \e azi2.  (It's not necessary to
       * include this as a capability to GeodesicLine because this is included
       * by default.)
       * @hideinitializer
       **********************************************************************/
      AZIMUTH       = 1U<<9  | unsigned(captype::CAP_NONE),
      /**
       * Calculate distance \e s12.
       * @hideinitializer
       **********************************************************************/
      DISTANCE      = 1U<<10 | unsigned(captype::CAP_C1),
      /**
       * Allow distance \e s12 to be used as input in the direct geodesic
       * problem.
       * @hideinitializer
       **********************************************************************/
      DISTANCE_IN   = 1U<<11 | unsigned(captype::CAP_C1) |
                          unsigned(captype::CAP_C1p),
      /**
       * Calculate reduced length \e m12.
       * @hideinitializer
       **********************************************************************/
      REDUCEDLENGTH = 1U<<12 | unsigned(captype::CAP_C1) |
                          unsigned(captype::CAP_C2),
      /**
       * Calculate geodesic scales \e M12 and \e M21.
       * @hideinitializer
       **********************************************************************/
      GEODESICSCALE = 1U<<13 | unsigned(captype::CAP_C1) |
                          unsigned(captype::CAP_C2),
      /**
       * Calculate area \e S12.
       * @hideinitializer
       **********************************************************************/
      AREA          = 1U<<14 | unsigned(captype::CAP_C4),
      /**
       * Do not wrap the \e lon2 in the direct calculation.
       * @hideinitializer
       **********************************************************************/
      LONG_UNROLL   = 1U<<15,
      /**
       * All capabilities, calculate everything.
       * @hideinitializer
       **********************************************************************/
      ALL           = unsigned(captype::OUT_ALL) | unsigned(captype::CAP_ALL),
    };

    /**
     * @brief The version information.
     **********************************************************************/
    public ref class VersionInfo
    {
    private:
        VersionInfo() {}
    public:
        /**
         * @return The version string.
         *******************************************************************/
        static System::String^ GetString();
        /**
         * @return The major version.
         *******************************************************************/
        static int MajorVersion();
        /**
         * @return The minor version.
         *******************************************************************/
        static int MinorVersion();
        /**
         * @return The patch number.
         *******************************************************************/
        static int Patch();
    };

    /**
     * @brief Exception class for NETGeographicLib
     **********************************************************************/
    public ref class GeographicErr : public System::Exception
    {
    public:
        /**
         * @brief Creates an exception using an unmanaged string.
         * @param[in] msg The error string.
         ******************************************************************/
        GeographicErr( const char* msg ) :
            System::Exception( gcnew System::String( msg ) ) {}
        /**
         * @brief Creates an exception using a managed string.
         * @param[in] msg The error string.
         ******************************************************************/
        GeographicErr( System::String^ msg ) : System::Exception( msg ) {}
    };

    ref class StringConvert
    {
        StringConvert() {}
    public:
        static std::string ManagedToUnmanaged( System::String^ s );
        static System::String^ UnmanagedToManaged( const std::string& s )
        {   return gcnew System::String( s.c_str() ); }
    };

    /**
     * @brief Physical constants
     *
     * References:<br>
     * http://www.orekit.org/static/apidocs/org/orekit/utils/Constants.html<br>
     * A COMPENDIUM OF EARTH CONSTANTS RELEVANT TO AUSTRALIAN GEODETIC SCIENCE<br>
     * http://espace.library.curtin.edu.au/R?func=dbin-jump-full&local_base=gen01-era02&object_id=146669
     **********************************************************************/
    public ref class Constants
    {
    private:
        Constants() {}
    public:

        /**
         * @brief WGS72 Parameters
         **********************************************************************/
        ref class WGS72
        {
        private:
            WGS72() {}
            // The equatorial radius in meters.
            static const double m_MajorRadius = 6378135.0;
            // The flattening of the ellipsoid
            static const double m_Flattening = 1.0 / 298.26;
            // The gravitational constant in meters<sup>3</sup>/second<sup>2</sup>.
            static const double m_GravitationalConstant = 3.986008e+14;
            // The spin rate of the Earth in radians/second.
            static const double m_EarthRate = 7.292115147e-5;
            // dynamical form factor
            static const double m_J2 = 1.0826158e-3;
        public:
            //! The equatorial radius in meters.
            static property double MajorRadius { double get() { return m_MajorRadius; } }
            //! The flattening of the ellipsoid
            static property double Flattening { double get() { return m_Flattening; } }
            //! The gravitational constant in meters<sup>3</sup>/second<sup>2</sup>.
            static property double GravitationalConstant { double get() { return m_GravitationalConstant; } }
            //! The spin rate of the Earth in radians/second.
            static property double EarthRate { double get() { return m_EarthRate; } }
            //! The dynamical form factor (J2).
            static property double J2 { double get() { return m_J2; } }
        };

        /**
         * @brief WGS84 Parameters
         **********************************************************************/
        ref class WGS84
        {
        private:
            WGS84() {}
            // The equatorial radius in meters.
            static const double m_MajorRadius = 6378137.0;
            // The flattening of the ellipsoid
            static const double m_Flattening = 1.0 / 298.257223563;
            // The gravitational constant in meters<sup>3</sup>/second<sup>2</sup>.
            // I have also seen references that set this value to 3.986004418e+14.
            // The following value is used to maintain consistency with GeographicLib.
            static const double m_GravitationalConstant = 3.986005e+14;
            // The spin rate of the Earth in radians/second.
            static const double m_EarthRate = 7.292115e-5;
            // dynamical form factor
            static const double m_J2 = 1.08263e-3;
        public:
            //! The equatorial radius in meters.
            static property double MajorRadius { double get() { return m_MajorRadius; } }
            //! The flattening of the ellipsoid
            static property double Flattening { double get() { return m_Flattening; } }
            //! The gravitational constant in meters<sup>3</sup>/second<sup>2</sup>.
            static property double GravitationalConstant { double get() { return m_GravitationalConstant; } }
            //! The spin rate of the Earth in radians/second.
            static property double EarthRate { double get() { return m_EarthRate; } }
            //! The dynamical form factor (J2).
            static property double J2 { double get() { return m_J2; } }
        };

        /**
         * @brief GRS80 Parameters
         **********************************************************************/
        ref class GRS80
        {
        private:
            GRS80() {}
            // The equatorial radius in meters.
            static const double m_MajorRadius = 6378137.0;
            // The flattening of the ellipsoid
            static const double m_Flattening = 1.0 / 298.257222100882711;
            // The gravitational constant in meters<sup>3</sup>/second<sup>2</sup>.
            static const double m_GravitationalConstant = 3.986005e+14;
            // The spin rate of the Earth in radians/second.
            static const double m_EarthRate = 7.292115e-5;
            // dynamical form factor
            static const double m_J2 = 1.08263e-3;
        public:
            //! The equatorial radius in meters.
            static property double MajorRadius { double get() { return m_MajorRadius; } }
            //! The flattening of the ellipsoid
            static property double Flattening { double get() { return m_Flattening; } }
            //! The gravitational constant in meters<sup>3</sup>/second<sup>2</sup>.
            static property double GravitationalConstant { double get() { return m_GravitationalConstant; } }
            //! The spin rate of the Earth in radians/second.
            static property double EarthRate { double get() { return m_EarthRate; } }
            //! The dynamical form factor (J2).
            static property double J2 { double get() { return m_J2; } }
        };
    };

    /**
     * @brief Utility library.
     *
     * This class only exposes the GeographicLib::Utility::fractionalyear
     * function.
     **********************************************************************/
    public ref class Utility
    {
    private:
        // hide the constructor since all members of this class are static
        Utility() {}
    public:
    /**
     * Convert a string representing a date to a fractional year.
     *
     * @param[in] s the string to be converted.
     * @exception GeographicErr if \e s can't be interpreted as a date.
     * @return the fractional year.
     *
     * The string is first read as an ordinary number (e.g., 2010 or 2012.5);
     * if this is successful, the value is returned.  Otherwise the string
     * should be of the form yyyy-mm or yyyy-mm-dd and this is converted to a
     * number with 2010-01-01 giving 2010.0 and 2012-07-03 giving 2012.5.
     **********************************************************************/
        static double FractionalYear( System::String^ s );
    };
}  // namespace NETGeographicLib
