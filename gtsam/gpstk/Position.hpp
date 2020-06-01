#pragma ident "$Id$"



/**
 * @file Position.hpp
 * class gpstk::Position encapsulates 3-D positions, including geographic positions,
 *    expressed as geodetic (with respect to an ellipsoid), geocentric or
 *    Earth-centered, Earth-fixed (cartesian) coordinates, as well as ordinary
 *    positions defined by spherical or cartesian coordinates. Position inherits
 *    from class Triple.
 */

#ifndef GPSTK_POSITION_HPP
#define GPSTK_POSITION_HPP

//============================================================================
//
//  This file is part of GPSTk, the GPS Toolkit.
//
//  The GPSTk is free software; you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published
//  by the Free Software Foundation; either version 2.1 of the License, or
//  any later version.
//
//  The GPSTk is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with GPSTk; if not, write to the Free Software Foundation,
//  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
//  
//  Copyright 2004, The University of Texas at Austin
//
//============================================================================

#include "Exception.hpp"
#include "StringUtils.hpp"
#include "Triple.hpp"
#include "EllipsoidModel.hpp"
#include "ReferenceFrame.hpp"
#include "Xvt.hpp"

namespace gpstk
{
      /** @addtogroup geodeticgroup */
      //@{

      // forward declarations
   class Position;
   double range(const Position& A, const Position& B) throw(GeometryException);
   
      /**
       * A position representation class for common 3D geographic position formats,
       * including geodetic (geodetic latitude, longitude, and height above the ellipsoid)
       * geocentric (geocentric latitude, longitude, and radius from Earth's center),
       * cartesian (Earth-centered, Earth-fixed) and spherical (theta,phi,radius).
       *
       * Internally, the representation of Position consists of three coordinate
       * values (double), two doubles from a ellipsoid model (see below, storing these
       * doubles is preferred over adding EllipsoidModel to calling arguments everywhere),
       * a flag of type 'enum CoordinateSystem' giving the coordinate system, and a
       * tolerance for use in comparing Positions. Class Position inherits from class
       * Triple, which is how the coordinate values are stored (Triple actually uses
       * std::valarray<double> of length 3). It is important to note that
       * Triple:: routines are properly used by Positions ONLY in the Cartesian
       * coordinate system.
       *
       * Only geodetic coordinates depend on a ellipsoid, and then
       * only on the semi-major axis of the Earth and the square of its
       * eccentricity. Input of this ellipsoid information (usually a pointer to a
       * EllipsoidModel) is required by functions involving constructors of, or
       * transformation to or from, Geodetic coordinates. However since a default
       * is supplied (WGS84), the user need never deal with geiods unless desired.
       * In fact, if the geodetic coordinate system is avoided, the Position class
       * can be interpreted simply as 3D vectors in any context, particularly since
       * the class inherits from Triple, which includes many vector manipulation
       * routines (although the Triple:: routines assume Cartesian coordinates).
       * Even the requirement that lengths (radius, height and the cartesian
       * coordinates) have units of meters is required only if geodetic coordinates
       * are used (because the semi-major axis in EllipsoidModel is in meters);
       * without using Geodetic one could apply the class using any units for
       * length as long as setTolerance() is called appropriately.
       *
       * Position relies on a series of fundamental routines to transform from
       * one coordinate system to another, these include, for example
       * void Position::convertGeodeticToCartesian(const Triple& llh, Triple& xyz,
       *    const double A, const double eccSq);
       * void Position::convertSphericalToCartesian(const Triple& tpr, Triple& xyz);
       * These functions use Triple in the calling arguments.
       *
       * Position will throw exceptions (gpstk::GeometryException) on bad input
       * (e.g. negative radius or latitude > 90 degrees); otherwise the class
       * attempts to handle all points, even the pole and the origin, consistently
       * and without throwing exceptions.
       * At or very near the poles, the transformation routines will set
       * latitude = +/-90 degrees, which is theta = 0 or 180, and (arbitrarily)
       * longitude = 0. At or very near the origin, the transformation routines
       * will set latitude = 0, which is theta = 90, and (arbitrarily) longitude = 0;
       * radius will be set to zero and geodetic height will be set to
       * -radius(Earth) (= -6378137.0 in WGS84). The tolerance used in testing
       * 'at or near the pole or origin' is radius < POSITION_TOLERANCE/5.
       * Note that this implies that a Position that is very near the origin may
       * be SET to the exact origin by the transformation routines, and that
       * thereby information about direction (e.g. latitude and longitude)
       * may be LOST. The user is warned to be very careful when working
       * near either the pole or the origin.
       *
       * Position includes setToString() and printf() functions similar to those
       * in gpstk::CommonTime; this allows flexible and powerful I/O of Position to
       * strings and streams.
       *
       * @sa positiontest.cpp for examples.
       */
   class Position : public Triple
   {
   public:
      // ----------- Part  1: coordinate systems --------------------------------
      //
         /// The coordinate systems supported by Position
      enum CoordinateSystem
      {
         Unknown=0,  ///< unknown coordinate system
         Geodetic,   ///< geodetic latitude, longitude, and height above ellipsoid
         Geocentric, ///< geocentric (regular spherical coordinates)
         Cartesian,  ///< cartesian (Earth-centered, Earth-fixed)
         Spherical   ///< spherical coordinates (theta,phi,radius)
      };

         /// return string giving name of coordinate system
      std::string getSystemName()
         throw();

      // ----------- Part  2: member functions: tolerance -----------------------
      //
         /// One millimeter tolerance.
      static const double ONE_MM_TOLERANCE;
         /// One centimeter tolerance.
      static const double ONE_CM_TOLERANCE;
         /// One micron tolerance.
      static const double ONE_UM_TOLERANCE;
      
         /// Default tolerance for time equality in days.
      static double POSITION_TOLERANCE;

         /// Changes the POSITION_TOLERANCE for all Position objects
      static double setPositionTolerance(const double tol)
         { POSITION_TOLERANCE = tol;  return POSITION_TOLERANCE; }

         /// Returns the current POSITION_TOLERANCE.
      static double getPositionTolerance()
         { return POSITION_TOLERANCE; }
   
         /**
          * Sets the tolerance for output and comparisons, for this object only.
          * See the constants in this file (e.g. ONE_MM_TOLERANCE)
          * for some easy to use tolerance values.
          * @param tol Tolerance in meters to be used by comparison operators.
          * @sa Position-Specific Definitions
          */
      Position& setTolerance(const double tol)
         throw();

      // ----------- Part  3: member functions: constructors --------------------
      //
         /**
          * Default constructor.
          * Initializes to zero, Unknown coordinates
          */
      Position()
         throw();

         /**
          * Explicit constructor. Coordinate system may be specified on input,
          * but defaults to Cartesian. Pointer to EllipsoidModel may be specified,
          * but default is NULL (in which case WGS84 values will be used).
          * @param a first coordinate [ X(m), or latitude (degrees N) ]
          * @param b second coordinate [ Y(m), or longitude (degrees E) ]
          * @param c third coordinate [ Z, height above ellipsoid or radius, in m ]
          * @param s coordinate system
          * @param ell pointer to EllipsoidModel
          * @throw GeometryException on invalid input.
          */
      Position(const double& a,
               const double& b,
               const double& c,
               CoordinateSystem s = Cartesian,
               EllipsoidModel *ell = NULL,
               ReferenceFrame frame = ReferenceFrame::Unknown)
         throw(GeometryException);

         /**
          * Explicit constructor. Coordinate system may be specified on input,
          * but defaults to Cartesian. Pointer to EllipsoidModel may be specified,
          * but default is NULL (in which case WGS84 values will be used).
          * @param ABC double array[3] coordinate values
          * @param s CoordinateSystem
          * @param ell pointer to EllipsoidModel
          * @throw GeometryException on invalid input.
          */
      Position(const double ABC[3],
               CoordinateSystem s = Cartesian,
               EllipsoidModel *ell = NULL,
               ReferenceFrame frame = ReferenceFrame::Unknown)
         throw(GeometryException);

         /**
          * Explicit constructor. Coordinate system may be specified on input,
          * but defaults to Cartesian. Pointer to EllipsoidModel may be specified,
          * but default is NULL (in which case WGS84 values will be used).
          * @param ABC coordinate values
          * @param s CoordinateSystem
          * @param ell pointer to EllipsoidModel
          * @throw GeometryException on invalid input.
          */
      Position(const Triple& ABC,
               CoordinateSystem s = Cartesian,
               EllipsoidModel *ell = NULL,
               ReferenceFrame frame = ReferenceFrame::Unknown)
         throw(GeometryException);

         /**
          * Explicit constructor from Xvt. The coordinate system is Cartesian,
          * and the velocity and time information in the input is ignored.
          * @param xvt Input Xvt object, xvt.x contains the Cartesian coordinates
          */
      Position(const Xvt& xvt)
         throw();

         /// Destructor.
      ~Position()
         throw()
         {}

      // ----------- Part  4: member functions: arithmetic ----------------------
      //
         /** Subtract a Position from this Position. Perform the subtraction in
          * Cartesian coordinates, but return this Position to the system it
          * had originally.
          * @param right Position to subtract from this one.
          * @return new Position, in the original system.
          */
      Position& operator-=(const Position& right)
         throw();

         /** Add a Position to this Position. Perform the addition in
          * Cartesian coordinates, but return this Position to the system it
          * had originally.
          * @param right Position to add to this one.
          * @return new Position, in the original system.
          */
      Position& operator+=(const Position& right)
         throw();

         /**
          * Difference two Positions, returning result as a Position in Cartesian
          * coordinates, the only system in which a position difference makes sense.
          * @param right Position to subtract from this one.
          * @return difference as Position.
          */
      friend Position operator-(const Position& left,
                                      const Position& right)
         throw();

         /**
          * Add two Positions, returning result as a Position in Cartesian
          * coordinates, the only system in which a position sum makes sense.
          * @param right Position to add to this one.
          * @return The new Position.
          */
      friend Position operator+(const Position& left,
                                      const Position& right)
         throw();

         /** Multiply a Position by a double scalar on the left.
          * @param right Position to be multiplied by the scalar
          * @param scale the (double) scalar
          * @return The new Position.
          */
      friend Position operator*(const double& scale,
                                const Position& right)
         {
            Position tmp(right);
            tmp.theArray *= scale;
            return tmp;
         }

         /** Multiply a Position by a double scalar on the right.
          * @param left Position to be multiplied by the scalar
          * @param scale the (double) scalar
          * @return The new Position.
          */
      friend Position operator*(const Position& left,
                                const double& scale)
         {
            return operator*(scale, left);
         }

         /** Multiply a Position by an integer scalar on the left.
          * @param right Position to be multiplied by the scalar
          * @param scale the (int) scalar
          * @return The new Position.
          */
      friend Position operator*(const int& scale,
                                const Position& right)
         {
            return operator*(double(scale), right);
         }

         /** Multiply a Position by an integer scalar on the right.
          * @param left Position to be multiplied by the scalar
          * @param scale the (int) scalar
          * @return The new Position.
          */
      friend Position operator*(const Position& left,
                                const int& scale)
         {
            return operator*(double(scale), left);
         }

      // ----------- Part  5: member functions: comparisons ---------------------
      //
         /// Equality operator. Return true if range between this Position and
         /// the input Position is less than tolerance. Return false if ellipsoid
         /// values differ.
         /// @param right Position to be compared to this Position
      bool operator==(const Position &right) const
         throw();

         /// Inequality operator. Return true if range between this Position and
         /// the input Position is greater than tolerance. Return true if ellipsoid
         /// values differ.
         /// @param right Position to be compared to this Position
      bool operator!=(const Position &right) const
         throw();

      // ----------- Part  6: member functions: coordinate transformations ------
      //
         /**
          * Transform coordinate system. Does nothing if sys already matches the
          * current value of member CoordinateSystem 'system'.
          * @param sys CoordinateSystem into which this Position is transformed.
          */
      Position transformTo(CoordinateSystem sys)
         throw();
  
         /// Convert to geodetic coordinates (does nothing if
         /// system == Geodetic already).
      Position asGeodetic()
         throw()
      { transformTo(Geodetic); return *this; }

         /// Convert to another ell, then to geodetic coordinates.
         /// @return a reference to this.
         /// @throw GeometryException if input is NULL.
      Position asGeodetic(EllipsoidModel *ell)
         throw(GeometryException)
      {
         try { setEllipsoidModel(ell); }
         catch(GeometryException& ge) { GPSTK_RETHROW(ge); }
         transformTo(Geodetic);
         return *this;
      }

         /// Convert to cartesian coordinates (does nothing if
         /// system == Cartesian already).
      Position asECEF()
         throw()
      { transformTo(Cartesian); return *this; }


      // ----------- Part  7: member functions: get -----------------------------
      // 
      // These routines retrieve coordinate values in all coordinate systems.
      //

         /// return coordinate ReferenceFrame
      const ReferenceFrame& getReferenceFrame() const
         throw();
      
         /// return X coordinate (meters)
      double X() const
         throw();

         /// return Y coordinate (meters)
      double Y() const
         throw();

         /// return Z coordinate (meters)
      double Z() const
         throw();

         /// return geodetic latitude (degrees North).
      double geodeticLatitude() const
         throw();

         /// return geocentric latitude (degrees North);
         /// equal to 90 degress - theta in regular spherical coordinates.
      double geocentricLatitude() const
         throw();

         /// return spherical coordinate theta in degrees
      double theta() const
         throw();

         /// return spherical coordinate phi in degrees
      double phi() const
         throw();

         /// return longitude (degrees East);
         /// equal to phi in regular spherical coordinates.
      double longitude() const
         throw();

         /// return distance from the center of Earth (meters),
         /// Same as radius in spherical coordinates.
      double radius() const
         throw();

         /// return height above ellipsoid (meters) (Geodetic).
      double height() const
         throw();

         /// return the coordinate system for this Position
      CoordinateSystem getCoordinateSystem() const
         throw() 
      { return system; };

         /// return geodetic latitude (deg N)
      double getGeodeticLatitude() const
         throw()
      { return geodeticLatitude(); }

         /// return geocentric latitude (deg N)
      double getGeocentricLatitude() const
         throw()
      { return geocentricLatitude(); }

         /// return longitude (deg E) (either geocentric or geodetic)
      double getLongitude() const
         throw()
      { return longitude(); }

         /// return height above ellipsoid (meters)
      double getAltitude() const
         throw()
      { return height(); }

         /// return height above ellipsoid (meters)
      double getHeight() const
         throw()
      { return height(); }

         /// return ECEF X coordinate (meters)
      double getX() const
         throw()
      { return X(); }

         /// return ECEF Y coordinate (meters)
      double getY() const
         throw()
      { return Y(); }

         /// return ECEF Z coordinate (meters)
      double getZ() const
         throw()
      { return Z(); }

         /// return spherical coordinate angle theta (deg) (90 - geocentric latitude)
      double getTheta() const
         throw()
      { return theta(); }

         /// return spherical coordinate angle phi (deg) (same as longitude)
      double getPhi() const
         throw()
      { return phi(); }

         /// return radius
      double getRadius() const
         throw()
      { return radius(); }

      // ----------- Part  8: member functions: set -----------------------------
      //
         /**
          * Set the ReferenceFrame that this position is in.
          * @param frame The ReferenceFrame to set to.
          */
      void setReferenceFrame(const ReferenceFrame& frame)
         throw();

         /**
          * Set the ellipsoid values for this Position given a ellipsoid.
          * @param ell  Pointer to the EllipsoidModel.
          * @throw      GeometryException if input is NULL.
          */
      void setEllipsoidModel(const EllipsoidModel *ell)
         throw(GeometryException);

         /**
          * Set the Position given geodetic coordinates; system is set to Geodetic.
          * @param lat geodetic latitude in degrees North
          * @param lon geodetic longitude in degrees East
          * @param ht height above the ellipsoid in meters
          * @return a reference to this object.
          * @throw GeometryException on invalid input
          */
      Position& setGeodetic(const double lat,
                            const double lon,
                            const double ht,
                            const EllipsoidModel *ell = NULL)
         throw(GeometryException);

         /**
          * Set the Position given geocentric coordinates; system is set to Geocentric
          * @param lat geocentric latitude in degrees North
          * @param lon geocentric longitude in degrees East
          * @param rad radius from the Earth's center in meters
          * @return a reference to this object.
          * @throw GeometryException on invalid input
          */
      Position& setGeocentric(const double lat,
                              const double lon,
                              const double rad)
         throw(GeometryException);

         /**
          * Set the Position given spherical coordinates; system is set to Spherical
          * @param theta angle from the Z-axis (degrees)
          * @param phi angle from the X-axis in the XY plane (degrees)
          * @param rad radius from the center in meters
          * @return a reference to this object.
          * @throw GeometryException on invalid input
          */
      Position& setSpherical(const double theta,
                             const double phi,
                             const double rad)
         throw(GeometryException);

         /**
          * Set the Position given ECEF coordinates; system is set to Cartesian.
          * @param X ECEF X coordinate in meters.
          * @param Y ECEF Y coordinate in meters.
          * @param Z ECEF Z coordinate in meters.
          * @return a reference to this object.
          */
      Position& setECEF(const double X,
                        const double Y,
                        const double Z)
         throw();

         /**
          * Set the Position given an array of ECEF coordinates;
          * system is set to Cartesian.
          * @param XYZ array[3] ECEF X,Y,Z coordinate in meters.
          * @return a reference to this object.
          */
      Position& setECEF(const double XYZ[3])
         throw()
      { return setECEF(XYZ[0],XYZ[1],XYZ[2]); }

         /**
          * Set the Position given ECEF coordinates; system is set to Cartesian.
          * @param XYZ ECEF X,Y,Z coordinates in meters.
          * @return a reference to this object.
          */
      Position& setECEF(const Triple& XYZ)
         throw()
      { return setECEF(XYZ[0],XYZ[1],XYZ[2]); }

      // ----------- Part 9: member functions: setToString, printf -------------
      //
         /**
          * setToString, similar to scanf, this function takes a string and a
          * format describing string in order to define Position
          * values.  The parameters it can take are listed below and
          * described above with the printf() function.
          *
          * The specification must be sufficient to define a Position.
          * The following table lists combinations that give valid
          * Positions. Anything more or other combinations will give
          * unknown (read as: "bad") results so don't try it.  Anything
          * less will throw an exception.
          *
          * @code
          *  %X %Y %Z  (cartesian or ECEF in kilometers)
          *  %x %y %z  (cartesian or ECEF in meters)
          *  %a %l %r  (geocentric lat,lon,radius, longitude E, radius in meters)
          *  %A %L %h  (geodetic lat,lon,height, longitude E, height in meters)
          *  %a %w %R  (geocentric lat,lon,radius, longitude W, radius in kilometers)
          *  %A %W %H  (geodetic lat,lon,height, longitude W, height in kilometers)
          *  %t %p %r  (spherical theta, phi, radius, degrees and meters)
          *  %T %P %R  (spherical theta, phi, radius, radians and kilometers)
          * @endcode
          *
          * So
          * @code
          * pos.setToString("123.4342,9328.1982,-128987.399", "%X,%Y,%Z");
          * @endcode
          *
          * works but 
          *
          * @code
          * pos.setToString("123.4342,9328.1982", "%X,%Y");
          * @endcode
          * doesn't work (incomplete specification because it doesn't specify
          * a Position).
          *
          * Whitespace is unimportant here; the function will handle it.
          * The caller must ensure that that the extra characters in
          * the format string (ie '.' ',') are in the same relative
          * location as they are in the actual string; see the example above.
          *
          * @param str string from which to get the Position coordinates
          * @param fmt format to use to parse \c str.
          * @throw GeometryException if \c fmt is an incomplete or invalid
          *    specification
          * @throw StringException if an error occurs manipulating the
          * \c str or \c fmt strings.
          * @return a reference to this object.
          */
      Position& setToString(const std::string& str,
                            const std::string& fmt)
         throw(GeometryException,
               StringUtils::StringException);


         // if you can see this, ignore the \'s below, as they are for
         // the nasty html-ifying of doxygen.  Browsers try to
         // interpret the % and they get all messed up.
         /**
          * Format this Position into a string.
          *
          * Generate and return a string containing formatted
          * Position coordinates, formatted by the specification \c fmt.
          *
          * \li \%x   X() (meters)
          * \li \%y   Y() (meters)
          * \li \%z   Z() (meters)
          * \li \%X   X()/1000 (kilometers)
          * \li \%Y   Y()/1000 (kilometers)
          * \li \%Z   Z()/1000 (kilometers)
          * \li \%A   geodeticLatitude() (degrees North)
          * \li \%a   geocentricLatitude() (degrees North)
          * \li \%L   longitude() (degrees East)
          * \li \%l   longitude() (degrees East)
          * \li \%w   longitude() (degrees West)
          * \li \%W   longitude() (degrees West)
          * \li \%t   theta() (degrees)
          * \li \%T   theta() (radians)
          * \li \%p   phi() (degrees)
          * \li \%P   phi() (radians)
          * \li \%r   radius() meters
          * \li \%R   radius()/1000 kilometers
          * \li \%h   height() meters
          * \li \%H   height()/1000 kilometers
          *
          * @param fmt format to use for this time.
          * @return a string containing this Position in the
          * representation specified by \c fmt.
          */
      std::string printf(const char *fmt) const
         throw(StringUtils::StringException);

         /// Format this time into a string.
         /// @see printf(const char*)
      std::string printf(const std::string& fmt) const
         throw(StringUtils::StringException) 
      { return printf(fmt.c_str()); }

         /// Returns the string that operator<<() would print.
      std::string asString() const
         throw(StringUtils::StringException);

      // ----------- Part 10: functions: fundamental conversions ---------------
      // 
         /** Fundamental conversion from spherical to cartesian coordinates.
          * @param trp (input): theta, phi (degrees), radius
          * @param xyz (output): X,Y,Z in units of radius
          * Algorithm references: standard geometry.
          */
      static void convertSphericalToCartesian(const Triple& tpr,
                                              Triple& xyz)
         throw();

         /** Fundamental routine to convert cartesian to spherical coordinates.
          * The zero vector is converted to (90,0,0).
          * @param xyz (input): X,Y,Z
          * @param trp (output): theta, phi (degrees), radius (units of input)
          * Algorithm references: standard geometry.
          */
      static void convertCartesianToSpherical(const Triple& xyz,
                                              Triple& tpr)
         throw();


         /** Fundamental routine to convert ECEF (cartesian) to geodetic coordinates,
          * (Ellipsoid specified by semi-major axis and eccentricity squared).
          * The zero vector is converted to (90,0,-R(earth)).
          * @param xyz (input): X,Y,Z in meters
          * @param llh (output): geodetic lat(deg N), lon(deg E),
          *                             height above ellipsoid (meters)
          * @param A (input) Earth semi-major axis
          * @param eccSq (input) square of Earth eccentricity
          * Algorithm references: Leick, "GPS Satellite Surveying," 2nd edition.
          */
      static void convertCartesianToGeodetic(const Triple& xyz,
                                             Triple& llh,
                                             const double A,
                                             const double eccSq)
         throw();

         /** Fundamental routine to convert geodetic to ECEF (cartesian) coordinates,
          * (Ellipsoid specified by semi-major axis and eccentricity squared).
          * @param llh (input): geodetic lat(deg N), lon(deg E),
          *                             height above ellipsoid (meters)
          * @param A (input) Earth semi-major axis
          * @param xyz (output): X,Y,Z in meters
          * @param eccSq (input) square of Earth eccentricity
          * Algorithm references: Leick, "GPS Satellite Surveying," 2nd edition.
          */
      static void convertGeodeticToCartesian(const Triple& llh,
                                             Triple& xyz,
                                             const double A,
                                             const double eccSq)
         throw();


         /** Fundamental routine to convert cartesian (ECEF) to geocentric
          * The zero vector is converted to (0,0,0).
          * @param xyz (input): X,Y,Z
          * @param llr (output): geocentric lat(deg N), lon(deg E),
          *                              radius (units of input)
          */
      static void convertCartesianToGeocentric(const Triple& xyz,
                                               Triple& llr)
         throw();

         /** Fundamental routine to convert geocentric to cartesian (ECEF)
          * @param llr (input): geocentric lat(deg N),lon(deg E),radius
          * @param xyz (output): X,Y,Z (units of radius)
          */
      static void convertGeocentricToCartesian(const Triple& llr,
                                               Triple& xyz)
         throw();


         /** Fundamental routine to convert geocentric to geodetic
          * @param llr (input): geocentric lat(deg N),lon(deg E),radius (meters)
          * @param geodeticllh (output): geodetic latitude (deg N),
          *            longitude (deg E), and height above ellipsoid (meters)
          * @param A (input) Earth semi-major axis
          * @param eccSq (input) square of Earth eccentricity
          */
      static void convertGeocentricToGeodetic(const Triple& llr,
                                              Triple& geodeticllh,
                                              const double A,
                                              const double eccSq)
         throw();

         /** Fundamental routine to convert geodetic to geocentric 
          * @param geodeticllh (input): geodetic latitude (deg N),
          *            longitude (deg E), and height above ellipsoid (meters)
          * @param llr (output): geocentric lat (deg N),lon (deg E),radius (meters)
          * @param A (input) Earth semi-major axis
          * @param eccSq (input) square of Earth eccentricity
          */
      static void convertGeodeticToGeocentric(const Triple& geodeticllh,
                                              Triple& llr,
                                              const double A,
                                              const double eccSq)
         throw();

      // ----------- Part 11: operator<< and other useful functions -------------
      //
         /**
         * Stream output for Position objects.
         * @param s stream to append formatted Position to.
         * @param t Position to append to stream \c s.
         * @return reference to \c s.
         */
      friend std::ostream& operator<<(std::ostream& s,
                                      const Position& p);

         /**
         * Compute the range in meters between two Positions.
         * Input Positions are not modified.
         * @param A,B Positions between which to find the range
         * @return the range (in meters)
         * @throw GeometryException if ellipsoid values differ.
         *        or if transformTo(Cartesian) fails
         */
      friend double range(const Position& A,
                          const Position& B)
         throw(GeometryException);

         /**
         * Compute the radius of the ellipsoidal Earth, given the geodetic latitude.
         * @param geolat geodetic latitude in degrees
         * @return the Earth radius (in meters)
         */
      static double radiusEarth(const double geolat,
                                const double A,
                                const double eccSq)
         throw();

         /**
         * A member function that calls the non-member radiusEarth() for
         * this Position.
         * @return the Earth radius (in meters)
         */
      double radiusEarth() const
         throw()
      {
         Position p(*this);
         p.transformTo(Position::Geodetic);
         return Position::radiusEarth(p.theArray[0], p.AEarth, p.eccSquared);
      }

         /**
         * A member function that computes the elevation of the input
         * (Target) position as seen from this Position.
         * @param Target the Position which is observed to have the
         *        computed elevation, as seen from this Position.
         * @return the elevation in degrees
         */
      double elevation(const Position& Target) const
         throw(GeometryException);

         /**
         * A member function that computes the elevation of the input
         * (Target) position as seen from this Position, using a Geodetic
         * (ellipsoidal) system.
         * @param Target the Position which is observed to have the
         *        computed elevation, as seen from this Position.
         * @return the elevation in degrees
         */
      double elevationGeodetic(const Position& Target) const
         throw(GeometryException);

         /**
         * A member function that computes the azimuth of the input
         * (Target) position as seen from this Position.
         * @param Target the Position which is observed to have the
         *        computed azimuth, as seen from this Position.
         * @return the azimuth in degrees
         */
      double azimuth(const Position& Target) const
         throw(GeometryException);

         /**
         * A member function that computes the azimuth of the input
         * (Target) position as seen from this Position, using a Geodetic
         * (ellipsoidal) system.
         * @param Target the Position which is observed to have the
         *        computed azimuth, as seen from this Position.
         * @return the azimuth in degrees
         */
      double azimuthGeodetic(const Position& Target) const
         throw(GeometryException);

         /**
         * A member function that computes the position at which a signal, which
         * is received at this Position and there is observed at the (input)
         * azimuth and elevation angles, crosses a model ionosphere that is
         * taken to be a thin shell at constant (input) height.
         * This function will not transform this Position, and it will return
         * a Position in the same system; the algorithm itself is done in the
         * geocentric coordinate system.
         * @param elev elevation angle in degrees of the signal at reception
         * @param azim azimuth angle in degrees of the signal at reception
         * @param ionoht height of the ionosphere, in meters
         * @return Position IPP the position of the ionospheric pierce point,
         *     in the same coordinate system as *this; *this is not modified.
         */
      Position getIonosphericPiercePoint(const double elev,
                                         const double azim,
                                         const double ionoht) const
         throw();

         /**
         * A member function that computes the radius of curvature of the 
         * meridian (Rm) corresponding to this Position.
         * @return radius of curvature of the meridian (in meters)
         */
      double getCurvMeridian() const
         throw();

         /**
         * A member function that computes the radius of curvature in the 
         * prime vertical (Rn) corresponding to this Position.
         * @return radius of curvature in the prime vertical (in meters)
         */
      double getCurvPrimeVertical() const
         throw();

      // ----------- Part 12: private functions and member data -----------------
      //
   private:

         /** Initialization function, used by the constructors.
          * @param a coordinate [ X(m), or latitude (degrees N) ]
          * @param b coordinate [ Y(m), or longitude (degrees E) ]
          * @param c coordinate [ Z, height above ellipsoid or radius, in m ]
          * @param s CoordinateSystem, defaults to Cartesian
          * @param geiod pointer to a EllipsoidModel, default NULL (WGS84)
          * @throw GeometryException on invalid input.
          */
      void initialize(const double a,
                     const double b,
                     const double c,
                     CoordinateSystem s = Cartesian,
                     EllipsoidModel *ell = NULL,
                     ReferenceFrame frame = ReferenceFrame::Unknown)
         throw(GeometryException);

         /* Values of the coordinates, defined for each system as follows;
         *    Cartesian  : X,Y,Z in meters
         *    Geocentric : Latitude(degrees N), Longitude(degrees E),
         *                    Radius (meters)
         *    Geodetic   : Latitude(degrees N), Longitude(degrees E),
         *                    Height above ellipsoid (meters)
         *    Spherical  : theta (degrees) - angle from the z axis
         *                 phi (degrees) - angle in xy plane from x axis toward
         *                                     y axis (same as longitude)
         *                 radius (meters?) - distance from origin
         */
      // use std::valarray<double> theArray;  -- inherit from Triple

         /// semi-major axis of Earth (meters)
      double AEarth;

         /// square of ellipsoid eccentricity
      double eccSquared;

         /// see #CoordinateSystem
      CoordinateSystem system;

         /// tolerance used in comparisons
      double tolerance;
      
      ReferenceFrame refFrame;

   };   // end class Position

   //@}

}  // namespace gpstk

#endif   // GPSTK_POSITION_HPP
