/**
 * \file Constants.hpp
 * \brief Header for GeographicLib::Constants class
 *
 * Copyright (c) Charles Karney (2008-2011) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * http://geographiclib.sourceforge.net/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_CONSTANTS_HPP)
#define GEOGRAPHICLIB_CONSTANTS_HPP 1

#include <GeographicLib/Config.h>

/**
 * A compile-time assert.  Use C++11 static_assert, if available.
 **********************************************************************/
#if !defined(STATIC_ASSERT)
#  if __cplusplus >= 201103
#    define STATIC_ASSERT static_assert
#  elif defined(__GXX_EXPERIMENTAL_CXX0X__)
#    define STATIC_ASSERT static_assert
#  elif defined(_MSC_VER) && _MSC_VER >= 1600
// For reference, here is a table of Visual Studio and _MSC_VER
// correspondences:
//
// _MSC_VER  Visual Studio
//   1300     vc7
//   1311     vc7.1 (2003)
//   1400     vc8   (2005)
//   1500     vc9   (2008)
//   1600     vc10  (2010)
//   1700     vc11  (2012)
//   1800     vc12  (2013)
#    define STATIC_ASSERT static_assert
#  else
#    define STATIC_ASSERT(cond,reason) \
            { enum{ STATIC_ASSERT_ENUM = 1/int(cond) }; }
#  endif
#endif

#if defined(_MSC_VER) && defined(GEOGRAPHICLIB_SHARED_LIB) && \
  GEOGRAPHICLIB_SHARED_LIB
#  if GEOGRAPHICLIB_SHARED_LIB > 1
#    error GEOGRAPHICLIB_SHARED_LIB must be 0 or 1
#  elif defined(GeographicLib_EXPORTS)
#    define GEOGRAPHICLIB_EXPORT __declspec(dllexport)
#  else
#    define GEOGRAPHICLIB_EXPORT __declspec(dllimport)
#  endif
#else
#  define GEOGRAPHICLIB_EXPORT
#endif

#include <stdexcept>
#include <string>
#include <GeographicLib/Math.hpp>

/**
 * \brief Namespace for %GeographicLib
 *
 * All of %GeographicLib is defined within the GeographicLib namespace.  In
 * addition all the header files are included via %GeographicLib/Class.hpp.
 * This minimizes the likelihood of conflicts with other packages.
 **********************************************************************/
namespace GeographicLib {

  /**
   * \brief %Constants needed by %GeographicLib
   *
   * Define constants specifying the WGS84 ellipsoid, the UTM and UPS
   * projections, and various unit conversions.
   *
   * Example of use:
   * \include example-Constants.cpp
   **********************************************************************/
  class GEOGRAPHICLIB_EXPORT Constants {
  private:
    typedef Math::real real;
    Constants();                // Disable constructor

  public:
    /**
     * A synonym for Math::degree<real>().
     **********************************************************************/
    static inline Math::real degree() throw() { return Math::degree<real>(); }
    /**
     * @return the number of radians in an arcminute.
     **********************************************************************/
    static inline Math::real arcminute() throw()
    { return Math::degree<real>() / 60; }
    /**
     * @return the number of radians in an arcsecond.
     **********************************************************************/
    static inline Math::real arcsecond() throw()
    { return Math::degree<real>() / 3600; }

    /** \name Ellipsoid parameters
     **********************************************************************/
    ///@{
    /**
     * @tparam T the type of the returned value.
     * @return the equatorial radius of WGS84 ellipsoid (6378137 m).
     **********************************************************************/
    template<typename T> static inline T WGS84_a() throw()
    { return T(6378137) * meter<T>(); }
    /**
     * A synonym for WGS84_a<real>().
     **********************************************************************/
    static inline Math::real WGS84_a() throw() { return WGS84_a<real>(); }
    /**
     * @tparam T the type of the returned value.
     * @return the flattening of WGS84 ellipsoid (1/298.257223563).
     **********************************************************************/
    template<typename T> static inline T WGS84_f() throw()
    { return T(1) / ( T(298) + T(257223563) / T(1000000000) ); }
    /**
     * A synonym for WGS84_f<real>().
     **********************************************************************/
    static inline Math::real WGS84_f() throw() { return WGS84_f<real>(); }
    /**
     * @tparam T the type of the returned value.
     * @return the gravitational constant of the WGS84 ellipsoid, \e GM, in
     *   m<sup>3</sup> s<sup>&minus;2</sup>.
     **********************************************************************/
    template<typename T> static inline T WGS84_GM() throw()
    { return T(3986004) * T(100000000) + T(41800000); }
    /**
     * @tparam T the type of the returned value.
     * @return the angular velocity of the WGS84 ellipsoid, &omega;, in rad
     *   s<sup>&minus;1</sup>.
     **********************************************************************/
    template<typename T> static inline T WGS84_omega() throw()
    { return T(7292115) / (T(1000000) * T(100000)); }
    /// \cond SKIP
    /**
     * <b>DEPRECATED</b>
     * @return the reciprocal flattening of WGS84 ellipsoid.
     **********************************************************************/
    template<typename T> static inline T WGS84_r() throw()
    { return 1/WGS84_f<T>(); }
    /**
     * <b>DEPRECATED</b>
     * A synonym for WGS84_r<real>().
     **********************************************************************/
    static inline Math::real WGS84_r() throw() { return WGS84_r<real>(); }
    /// \endcond
    /**
     * @tparam T the type of the returned value.
     * @return the equatorial radius of GRS80 ellipsoid, \e a, in m.
     **********************************************************************/
    template<typename T> static inline T GRS80_a() throw()
    { return T(6378137); }
    /**
     * @tparam T the type of the returned value.
     * @return the gravitational constant of the GRS80 ellipsoid, \e GM, in
     *   m<sup>3</sup> s<sup>&minus;2</sup>.
     **********************************************************************/
    template<typename T> static inline T GRS80_GM() throw()
    { return T(3986005) * T(100000000); }
    /**
     * @tparam T the type of the returned value.
     * @return the angular velocity of the GRS80 ellipsoid, &omega;, in rad
     *   s<sup>&minus;1</sup>.
     *
     * This is about 2 &pi; 366.25 / (365.25 &times; 24 &times; 3600) rad
     * s<sup>&minus;1</sup>.  365.25 is the number of days in a Julian year and
     * 365.35/366.25 converts from solar days to sidereal days.  Using the
     * number of days in a Gregorian year (365.2425) results in a worse
     * approximation (because the Gregorian year includes the precession of the
     * earth's axis).
     **********************************************************************/
    template<typename T> static inline T GRS80_omega() throw()
    { return T(7292115) / (T(1000000) * T(100000)); }
    /**
     * @tparam T the type of the returned value.
     * @return the dynamical form factor of the GRS80 ellipsoid,
     *   <i>J</i><sub>2</sub>.
     **********************************************************************/
    template<typename T> static inline T GRS80_J2() throw()
    { return T(108263) / T(100000000); }
    /**
     * @tparam T the type of the returned value.
     * @return the central scale factor for UTM (0.9996).
     **********************************************************************/
    template<typename T> static inline T UTM_k0() throw()
    {return T(9996) / T(10000); }
    /**
     * A synonym for UTM_k0<real>().
     **********************************************************************/
    static inline Math::real UTM_k0() throw() { return UTM_k0<real>(); }
    /**
     * @tparam T the type of the returned value.
     * @return the central scale factor for UPS (0.994).
     **********************************************************************/
    template<typename T> static inline T UPS_k0() throw()
    { return T(994) / T(1000); }
    /**
     * A synonym for UPS_k0<real>().
     **********************************************************************/
    static inline Math::real UPS_k0() throw() { return UPS_k0<real>(); }
    ///@}

    /** \name SI units
     **********************************************************************/
    ///@{
    /**
     * @tparam T the type of the returned value.
     * @return the number of meters in a meter.
     *
     * This is unity, but this lets the internal system of units be changed if
     * necessary.
     **********************************************************************/
    template<typename T> static inline T meter() throw() { return T(1); }
    /**
     * A synonym for meter<real>().
     **********************************************************************/
    static inline Math::real meter() throw() { return meter<real>(); }
    /**
     * @return the number of meters in a kilometer.
     **********************************************************************/
    static inline Math::real kilometer() throw()
    { return 1000 * meter<real>(); }
    /**
     * @return the number of meters in a nautical mile (approximately 1 arc
     *   minute)
     **********************************************************************/
    static inline Math::real nauticalmile() throw()
    { return 1852 * meter<real>(); }

    /**
     * @tparam T the type of the returned value.
     * @return the number of square meters in a square meter.
     *
     * This is unity, but this lets the internal system of units be changed if
     * necessary.
     **********************************************************************/
    template<typename T> static inline T square_meter() throw()
    { return meter<real>() * meter<real>(); }
    /**
     * A synonym for square_meter<real>().
     **********************************************************************/
    static inline Math::real square_meter() throw()
    { return square_meter<real>(); }
    /**
     * @return the number of square meters in a hectare.
     **********************************************************************/
    static inline Math::real hectare() throw()
    { return 10000 * square_meter<real>(); }
    /**
     * @return the number of square meters in a square kilometer.
     **********************************************************************/
    static inline Math::real square_kilometer() throw()
    { return kilometer() * kilometer(); }
    /**
     * @return the number of square meters in a square nautical mile.
     **********************************************************************/
    static inline Math::real square_nauticalmile() throw()
    { return nauticalmile() * nauticalmile(); }
    ///@}

    /** \name Anachronistic British units
     **********************************************************************/
    ///@{
    /**
     * @return the number of meters in an international foot.
     **********************************************************************/
    static inline Math::real foot() throw()
    { return real(0.0254L) * 12 * meter<real>(); }
    /**
     * @return the number of meters in a yard.
     **********************************************************************/
    static inline Math::real yard() throw() { return 3 * foot(); }
    /**
     * @return the number of meters in a fathom.
     **********************************************************************/
    static inline Math::real fathom() throw() { return 2 * yard(); }
    /**
     * @return the number of meters in a chain.
     **********************************************************************/
    static inline Math::real chain() throw() { return 22 * yard(); }
    /**
     * @return the number of meters in a furlong.
     **********************************************************************/
    static inline Math::real furlong() throw() { return 10 * chain(); }
    /**
     * @return the number of meters in a statute mile.
     **********************************************************************/
    static inline Math::real mile() throw() { return 8 * furlong(); }
    /**
     * @return the number of square meters in an acre.
     **********************************************************************/
    static inline Math::real acre() throw() { return chain() * furlong(); }
    /**
     * @return the number of square meters in a square statute mile.
     **********************************************************************/
    static inline Math::real square_mile() throw() { return mile() * mile(); }
    ///@}

    /** \name Anachronistic US units
     **********************************************************************/
    ///@{
    /**
     * @return the number of meters in a US survey foot.
     **********************************************************************/
    static inline Math::real surveyfoot() throw()
    { return real(1200) / real(3937) * meter<real>(); }
    ///@}
  };

  /**
   * \brief Exception handling for %GeographicLib
   *
   * A class to handle exceptions.  It's derived from std::runtime_error so it
   * can be caught by the usual catch clauses.
   *
   * Example of use:
   * \include example-GeographicErr.cpp
   **********************************************************************/
  class GeographicErr : public std::runtime_error {
  public:

    /**
     * Constructor
     *
     * @param[in] msg a string message, which is accessible in the catch
     *   clause via what().
     **********************************************************************/
    GeographicErr(const std::string& msg) : std::runtime_error(msg) {}
  };

} // namespace GeographicLib

#endif  // GEOGRAPHICLIB_CONSTANTS_HPP
