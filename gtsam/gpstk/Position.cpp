#pragma ident "$Id$"



/**
 * @file Position.cpp
 * class gpstk::Position encapsulates 3-D positions, both geographic positions,
 *    expressed as geodetic (with respect to any geoid), geocentric or
 *    Earth-centered, Earth-fixed (cartesian) coordinates, as well as ordinary
 *    positions defined by spherical or cartesian coordinates. Position inherits
 *    from class Triple.
 */

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

#include "Position.hpp"
#include "WGS84Ellipsoid.hpp"
#include "GNSSconstants.hpp"    // for TWO_PI, etc
#include "geometry.hpp"             // for RAD_TO_DEG, etc
#include "MiscMath.hpp"             // for RSS, SQRT

namespace gpstk
{

   using namespace std;
   using namespace StringUtils;

   // ----------- Part  1: coordinate systems --------------------------------
      // Labels for coordinate systems supported by Position
   static const char *SystemNames[] = {
      "Unknown",
      "Geodetic",
      "Geocentric",
      "Cartesian",
      "Spherical"};

      // return string giving name of coordinate system
   string Position::getSystemName()
      throw()
   { return SystemNames[system]; }

   // ----------- Part  2: tolerance -----------------------------------------
      // One millimeter tolerance.
   const double Position::ONE_MM_TOLERANCE = 0.001;
      // One centimeter tolerance.
   const double Position::ONE_CM_TOLERANCE = 0.01;
      // One micron tolerance.
   const double Position::ONE_UM_TOLERANCE = 0.000001;

      // Default tolerance for time equality in meters.
   double Position::POSITION_TOLERANCE = Position::ONE_MM_TOLERANCE;

      // Sets the tolerance for output and comparisons, for this object only.
      // See the constants in this file (e.g. ONE_MM_TOLERANCE)
      // for some easy to use tolerance values.
      // @param tol Tolerance in meters to be used by comparison operators.
   Position& Position::setTolerance(const double tol)
      throw()
   {
      tolerance = tol;
      return *this;
   }

   // ----------- Part  3: member functions: constructors --------------------
   //
      // Default constructor.
   Position::Position()
      throw()
   {
      WGS84Ellipsoid WGS84;
      initialize(0.0,0.0,0.0,Unknown,&WGS84);
   }

   Position::Position(const double& a,
                      const double& b,
                      const double& c,
                      Position::CoordinateSystem s,
                      EllipsoidModel *ell,
                      ReferenceFrame frame)
      throw(GeometryException)
   {
      try {
         initialize(a,b,c,s,ell,frame);
      }
      catch(GeometryException& ge) {
         GPSTK_RETHROW(ge);
      }
   }

   Position::Position(const double ABC[3],
                      CoordinateSystem s,
                      EllipsoidModel *ell,
                      ReferenceFrame frame)
      throw(GeometryException)
   {
      double a=ABC[0];
      double b=ABC[1];
      double c=ABC[2];
      try {
         initialize(a,b,c,s,ell,frame);
      }
      catch(GeometryException& ge) {
         GPSTK_RETHROW(ge);
      }
   }

   Position::Position(const Triple& ABC,
                      CoordinateSystem s,
                      EllipsoidModel *ell,
                      ReferenceFrame frame)
      throw(GeometryException)
   {
      double a=ABC[0];
      double b=ABC[1];
      double c=ABC[2];
      try {
         initialize(a,b,c,s,ell,frame);
      }
      catch(GeometryException& ge) {
         GPSTK_RETHROW(ge);
      }
   }

   Position::Position(const Xvt& xvt)
      throw()
   {
      double a=xvt.x[0];
      double b=xvt.x[1];
      double c=xvt.x[2];
      initialize(a,b,c,Cartesian, NULL, xvt.frame);
   }

   // ----------- Part  4: member functions: arithmetic ----------------------
   //
   Position& Position::operator-=(const Position& right)
      throw()
   {
      Position r(right);
      CoordinateSystem savesys=system;    // save the original system

         // convert to cartestian and difference there
      transformTo(Cartesian);
      r.transformTo(Cartesian);

      for(int i=0; i<3; i++)
         theArray[i] -= r.theArray[i];

      transformTo(savesys);               // transform back to the original system
      return *this;
   }

   Position& Position::operator+=(const Position& right)
      throw()
   {
      Position r(right);
      CoordinateSystem savesys=system;    // save the original system

         // convert to cartestian and difference there
      transformTo(Cartesian);
      r.transformTo(Cartesian);

      for(int i=0; i<3; i++)
         theArray[i] += r.theArray[i];

      transformTo(savesys);               // transform back to the original system
      return *this;
   }

   Position operator-(const Position& left,
                            const Position& right)
      throw()
   {
      Position l(left),r(right);
         // convert both to Cartesian
      l.transformTo(Position::Cartesian);
      r.transformTo(Position::Cartesian);
         // difference
      l -= r;

      return l;
   }

   Position operator+(const Position& left,
                            const Position& right)
      throw()
   {
      Position l(left),r(right);
         // convert both to Cartesian
      l.transformTo(Position::Cartesian);
      r.transformTo(Position::Cartesian);
         // add
      l += r;

      return l;
   }

   // ----------- Part  5: member functions: comparisons ---------------------
   //
      // Equality operator. Returns false if ell values differ.
   bool Position::operator==(const Position &right) const
      throw()
   {
      if(AEarth != right.AEarth || eccSquared != right.eccSquared)
         return false;
      if(right.getReferenceFrame() != refFrame)
         return false;   //Unknown frames are considered the same.
      if(range(*this,right) < tolerance)
         return true;
      else
         return false;
   }

      // Inequality operator. Returns true if ell values differ.
   bool Position::operator!=(const Position &right) const
      throw()
   {
      return !(operator==(right));
   }

   // ----------- Part  6: member functions: coordinate transformations ------
   //
      // Transform coordinate system. Does nothing if sys already matches the
      // current value of member CoordinateSystem 'system'.
      // @param sys coordinate system into which *this is to be transformed.
      // @return *this
   Position Position::transformTo(CoordinateSystem sys)
      throw()
   {
      if(sys == Unknown || sys == system) return *this;

      // this copies geoid information and tolerance
      Position target(*this);

      // transform target.theArray and set target.system
      switch(system) {
         case Unknown:
            return *this;
         case Geodetic:
            // --------------- Geodetic to ... ------------------------
            switch(sys) {
               case Unknown: case Geodetic: return *this;
               case Geocentric:
                  convertGeodeticToGeocentric(*this,target,AEarth,eccSquared);
                  target.system = Geocentric;
                  break;
               case Cartesian:
                  convertGeodeticToCartesian(*this,target,AEarth,eccSquared);
                  target.system = Cartesian;
                  break;
               case Spherical:
                  convertGeodeticToGeocentric(*this,target,AEarth,eccSquared);
                  target.theArray[0] = 90 - target.theArray[0];   // geocen -> sph
                  target.system = Spherical;
                  break;
            }
            break;
         case Geocentric:
            // --------------- Geocentric to ... ----------------------
            switch(sys) {
               case Unknown: case Geocentric: return *this;
               case Geodetic:
                  convertGeocentricToGeodetic(*this,target,AEarth,eccSquared);
                  target.system = Geodetic;
                  break;
               case Cartesian:
                  convertGeocentricToCartesian(*this,target);
                  target.system = Cartesian;
                  break;
               case Spherical:
                  target.theArray[0] = 90 - target.theArray[0];   // geocen -> sph
                  target.system = Spherical;
                  break;
            }
            break;
         case Cartesian:
            // --------------- Cartesian to ... -----------------------
            switch(sys) {
               case Unknown: case Cartesian: return *this;
               case Geodetic:
                  convertCartesianToGeodetic(*this,target,AEarth,eccSquared);
                  target.system = Geodetic;
                  break;
               case Geocentric:
                  convertCartesianToGeocentric(*this,target);
                  target.system = Geocentric;
                  break;
               case Spherical:
                  convertCartesianToSpherical(*this,target);
                  target.system = Spherical;
                  break;
            }
            break;
         case Spherical:
            // --------------- Spherical to ... -----------------------
            switch(sys) {
               case Unknown: case Spherical: return *this;
               case Geodetic:
                  theArray[0] = 90 - theArray[0];   // sph -> geocen
                  convertGeocentricToGeodetic(*this,target,AEarth,eccSquared);
                  target.system = Geodetic;
                  break;
               case Geocentric:
                  target.theArray[0] = 90 - target.theArray[0];   // sph -> geocen
                  target.system = Geocentric;
                  break;
               case Cartesian:
                  convertSphericalToCartesian(*this,target);
                  target.system = Cartesian;
                  break;
            }
            break;
      }  // end switch(system)

      *this = target;
      return *this;
   }

   // ----------- Part  7: member functions: get -----------------------------
   //
   // These routines retrieve coordinate values in all coordinate systems.
   // Note that calling these will transform the Position to another coordinate
   // system if that is required.
   //
   
   const ReferenceFrame& Position::getReferenceFrame() const
      throw()
   {   
      return refFrame;
   }
   
      // Get X coordinate (meters)
   double Position::X() const
      throw()
   {
      if(system == Cartesian)
         return theArray[0];
      Position t(*this);
      t.transformTo(Cartesian);
      return t.theArray[0];
   }

      // Get Y coordinate (meters)
   double Position::Y() const
      throw()
   {
      if(system == Cartesian)
         return theArray[1];
      Position t(*this);
      t.transformTo(Cartesian);
      return t.theArray[1];
   }

      // Get Z coordinate (meters)
   double Position::Z() const
      throw()
   {
      if(system == Cartesian)
         return theArray[2];
      Position t(*this);
      t.transformTo(Cartesian);
      return t.theArray[2];
   }

      // Get geodetic latitude (degrees North).
   double Position::geodeticLatitude() const
      throw()
   {
      if(system == Geodetic)
         return theArray[0];
      Position t(*this);
      t.transformTo(Geodetic);
      return t.theArray[0];
   }

      // Get geocentric latitude (degrees North),
      // equal to 90 degress - theta in regular spherical coordinates.
   double Position::geocentricLatitude() const
      throw()
   {
      if(system == Geocentric)
         return theArray[0];
      Position t(*this);
      t.transformTo(Geocentric);
      return t.theArray[0];
   }

      // Get spherical coordinate theta in degrees
   double Position::theta() const
      throw()
   {
      if(system == Spherical)
         return theArray[0];
      Position t(*this);
      t.transformTo(Spherical);
      return t.theArray[0];
   }

      // Get spherical coordinate phi in degrees
   double Position::phi() const
      throw()
   {
      if(system == Spherical)
         return theArray[1];
      Position t(*this);
      t.transformTo(Spherical);
      return t.theArray[1];
   }

      // Get longitude (degrees East),
      // equal to phi in regular spherical coordinates.
   double Position::longitude() const
      throw()
   {
      if(system != Cartesian)
         return theArray[1];
      Position t(*this);
      t.transformTo(Spherical);
      return t.theArray[1];
   }

      // Get radius or distance from the center of Earth (meters),
      // Same as radius in spherical coordinates.
   double Position::radius() const
      throw()
   {
      if(system == Spherical || system == Geocentric)
         return theArray[2];
      Position t(*this);
      t.transformTo(Spherical);
      return t.theArray[2];
   }

      // Get height above ellipsoid (meters) (Geodetic).
   double Position::height() const
      throw()
   {
      if(system == Geodetic)
         return theArray[2];
      Position t(*this);
      t.transformTo(Geodetic);
      return t.theArray[2];
   }

   // ----------- Part  8: member functions: set -----------------------------
   //
   void Position::setReferenceFrame(const ReferenceFrame& frame)
      throw()
   {
      refFrame = frame;
   }
   
      /**
      * Set the ellipsoid values for this Position given a ellipsoid.
      * @param ell  Pointer to the EllipsoidModel.
      * @throw      GeometryException if input is NULL.
      */
   void Position::setEllipsoidModel(const EllipsoidModel *ell)
      throw(GeometryException)
   {
      if(!ell)
      {
         GeometryException ge("Given EllipsoidModel pointer is NULL.");
         GPSTK_THROW(ge);
      }
      AEarth = ell->a();
      eccSquared = ell->eccSquared();
   }

      // Set the Position given geodetic coordinates, system is set to Geodetic.
      // @param lat geodetic latitude in degrees North
      // @param lon geodetic longitude in degrees East
      // @param ht height above the ellipsoid in meters
      // @return a reference to this object.
      // @throw GeometryException on invalid input
   Position& Position::setGeodetic(const double lat,
                                   const double lon,
                                   const double ht,
                                   const EllipsoidModel *ell)
      throw(GeometryException)
   {
      if(lat > 90 || lat < -90)
      {
         GeometryException ge("Invalid latitude in setGeodetic: "
                                 + StringUtils::asString(lat));
         GPSTK_THROW(ge);
      }
      theArray[0] = lat;

      theArray[1] = lon;
      if(theArray[1] < 0)
         theArray[1] += 360*(1+(unsigned long)(theArray[1]/360));
      else if(theArray[1] >= 360)
         theArray[1] -= 360*(unsigned long)(theArray[1]/360);

      theArray[2] = ht;

      if(ell) {
         AEarth = ell->a();
         eccSquared = ell->eccSquared();
      }
      system = Geodetic;

      return *this;
   }

      // Set the Position given geocentric coordinates, system is set to Geocentric
      // @param lat geocentric latitude in degrees North
      // @param lon geocentric longitude in degrees East
      // @param rad radius from the Earth's center in meters
      // @return a reference to this object.
      // @throw GeometryException on invalid input
   Position& Position::setGeocentric(const double lat,
                                     const double lon,
                                     const double rad)
      throw(GeometryException)
   {
      if(lat > 90 || lat < -90)
      {
         GeometryException ge("Invalid latitude in setGeocentric: "
                                 + StringUtils::asString(lat));
         GPSTK_THROW(ge);
      }
      if(rad < 0)
      {
         GeometryException ge("Invalid radius in setGeocentric: "
                                          + StringUtils::asString(rad));
         GPSTK_THROW(ge);
      }
      theArray[0] = lat;
      theArray[1] = lon;
      theArray[2] = rad;

      if(theArray[1] < 0)
         theArray[1] += 360*(1+(unsigned long)(theArray[1]/360));
      else if(theArray[1] >= 360)
         theArray[1] -= 360*(unsigned long)(theArray[1]/360);
      system = Geocentric;

      return *this;
   }

      // Set the Position given spherical coordinates, system is set to Spherical
      // @param theta angle from the Z-axis (degrees)
      // @param phi angle from the X-axis in the XY plane (degrees)
      // @param rad radius from the center in meters
      // @return a reference to this object.
      // @throw GeometryException on invalid input
   Position& Position::setSpherical(const double theta,
                                    const double phi,
                                    const double rad)
      throw(GeometryException)
   {
      if(theta < 0 || theta > 180)
      {
         GeometryException ge("Invalid theta in setSpherical: "
                                 + StringUtils::asString(theta));
         GPSTK_THROW(ge);
      }
      if(rad < 0)
      {
         GeometryException ge("Invalid radius in setSpherical: "
                                          + StringUtils::asString(rad));
         GPSTK_THROW(ge);
      }

      theArray[0] = theta;
      theArray[1] = phi;
      theArray[2] = rad;

      if(theArray[1] < 0)
         theArray[1] += 360*(1+(unsigned long)(theArray[1]/360));
      else if(theArray[1] >= 360)
         theArray[1] -= 360*(unsigned long)(theArray[1]/360);
      system = Spherical;

      return *this;
   }

      // Set the Position given ECEF coordinates, system is set to Cartesian.
      // @param X ECEF X coordinate in meters.
      // @param Y ECEF Y coordinate in meters.
      // @param Z ECEF Z coordinate in meters.
      // @return a reference to this object.
   Position& Position::setECEF(const double X,
                               const double Y,
                               const double Z)
      throw()
   {
      theArray[0] = X;
      theArray[1] = Y;
      theArray[2] = Z;
      system = Cartesian;
      return *this;
   }

   // ----------- Part 9: member functions: setToString, printf -------------
   //
      // setToString, similar to scanf, this function takes a string and a
      // format describing string in order to define Position
      // values.  The parameters it can take are listed below and
      // described above with the printf() function.
      //
      // The specification must be sufficient to define a Position.
      // The following table lists combinations that give valid
      // Positions. Anything more or other combinations will give
      // unknown (read as: "bad") results so don't try it.  Anything
      // less will throw an exception.
      //
      // @code
      //  %X %Y %Z  (cartesian or ECEF)
      //  %x %y %z  (cartesian or ECEF)
      //  %a %l %r  (geocentric)
      //  %A %L %h  (geodetic)
      //  %t %p %r  (spherical)
      // @endcode
      //
      // So
      // @code
      // pos.setToString("123.4342,9328.1982,-128987.399", "%X,%Y,%Z");
      // @endcode
      //
      // works but
      //
      // @code
      // pos.setToString("123.4342,9328.1982", "%X,%Y");
      // @endcode
      // doesn't work (incomplete specification because it doesn't specify
      // a Position).
      //
      // Whitespace is unimportant here, the function will handle it.
      // The caller must ensure that that the extra characters in
      // the format string (ie '.' ',') are in the same relative
      // location as they are in the actual string, see the example above.
      //
      // @param str string from which to get the Position coordinates
      // @param fmt format to use to parse \c str.
      // @throw GeometryException if \c fmt is an incomplete or invalid specification
      // @throw StringException if an error occurs manipulating the
      // \c str or \c fmt strings.
      // @return a reference to this object.
   Position& Position::setToString(const std::string& str,
                                   const std::string& fmt)
      throw(GeometryException,StringUtils::StringException)
   {
      try {
            // make an object to return (so we don't fiddle with *this
            // until it's necessary)
         Position toReturn;

            // flags indicated these defined
         bool hX=false, hY=false, hZ=false;
         bool hglat=false, hlon=false, hht=false;
         bool hclat=false, hrad=false;
         bool htheta=false, hphi=false;
            // store input values
         double x,y,z,glat,lon,ht,clat,rad,theta,phi;
            // copy format and input string to parse
         string f = fmt;
         string s = str;
         stripLeading(s);
         stripTrailing(f);

            // parse strings...  As we process each part, it's removed from both
            // strings so when we reach 0, we're done
         while ( (s.size() > 0) && (f.size() > 0) )
         {
               // remove everything in f and s up to the first % in f
               // (these parts of the strings must be identical or this will break
               // after it tries to remove it!)
            while ( (s.length() != 0) && (f.length() != 0) && (f[0] != '%') )
            {
                  // remove that character now and other whitespace
               s.erase(0,1);
               f.erase(0,1);
               stripLeading(s);
               stripLeading(f);
            }

               // check just in case we hit the end of either string...
            if ( (s.length() == 0) || (f.length() == 0) )
               break;

               // lose the '%' in f...
            f.erase(0,1);

               // if the format string is like %03f, get '3' as the field
               // length.
            string::size_type fieldLength = string::npos;

            if (!isalpha(f[0]))
            {
               fieldLength = asInt(f);

                  // remove everything else up to the next character
                  // (in "%03f", that would be 'f')
               while ((!f.empty()) && (!isalpha(f[0])))
                  f.erase(0,1);
               if (f.empty())
                  break;
            }

               // finally, get the character that should end this field, if any
            char delimiter = 0;
            if (f.size() > 1)
            {
               if (f[1] != '%')
               {
                  delimiter = f[1];

                  if (fieldLength == string::npos)
                     fieldLength = s.find(delimiter,0);
               }
                  // if the there is no delimiter character and the next field
                  // is another part of the time to parse, assume the length
                  // of this field is 1
               else if (fieldLength == string::npos)
               {
                  fieldLength = 1;
               }
            }

               // figure out the next string to be removed.  if there is a field
               // length, use that first
            string toBeRemoved = s.substr(0, fieldLength);

               // based on char at f[0], we know what to do...
            switch (f[0])
            {
          //%x   X() (meters)
          //%y   Y() (meters)
          //%z   Z() (meters)
          //%X   X()/1000 (kilometers)
          //%Y   Y()/1000 (kilometers)
          //%Z   Z()/1000 (kilometers)
               case 'X':
                  x = asDouble(toBeRemoved) * 1000;
                  hX = true;
                  break;
               case 'x':
                  x = asDouble(toBeRemoved);
                  hX = true;
                  break;
               case 'Y':
                  y = asDouble(toBeRemoved) * 1000;
                  hY = true;
                  break;
               case 'y':
                  y = asDouble(toBeRemoved);
                  hY = true;
                  break;
               case 'Z':
                  z = asDouble(toBeRemoved) * 1000;
                  hZ = true;
                  break;
               case 'z':
                  z = asDouble(toBeRemoved);
                  hZ = true;
                  break;
          //%A   geodeticLatitude() (degrees North)
          //%a   geocentricLatitude() (degrees North)
               case 'A':
                  glat = asDouble(toBeRemoved);
                  if(glat > 90. || glat < -90.) {
                     InvalidRequest f(
                           "Invalid geodetic latitude for setTostring: "
                           + toBeRemoved);
                     GPSTK_THROW(f);
                  }
                  hglat = true;
                  break;
               case 'a':
                  clat = asDouble(toBeRemoved);
                  if(clat > 90. || clat < -90.) {
                     InvalidRequest f(
                           "Invalid geocentric latitude for setTostring: "
                           + toBeRemoved);
                     GPSTK_THROW(f);
                  }
                  hclat = true;
                  break;
          //%L   longitude() (degrees East)
          //%l   longitude() (degrees East)
          //%w   longitude() (degrees West)
          //%W   longitude() (degrees West)
               case 'L':
               case 'l':
                  lon = asDouble(toBeRemoved);
                  if(lon < 0)
                     lon += 360*(1+(unsigned long)(lon/360));
                  else if(lon >= 360)
                     lon -= 360*(unsigned long)(lon/360);
                  hlon = true;
                  break;
               case 'w':
               case 'W':
                  lon = 360.0 - asDouble(toBeRemoved);
                  if(lon < 0)
                     lon += 360*(1+(unsigned long)(lon/360));
                  else if(lon >= 360)
                     lon -= 360*(unsigned long)(lon/360);
                  hlon = true;
                  break;
          //%t   theta() (degrees)
          //%T   theta() (radians)
               case 't':
                  theta = asDouble(toBeRemoved);
                  if(theta > 180. || theta < 0.) {
                     InvalidRequest f("Invalid theta for setTostring: "
                                                + toBeRemoved);
                     GPSTK_THROW(f);
                  }
                  htheta = true;
                  break;
               case 'T':
                  theta = asDouble(toBeRemoved) * RAD_TO_DEG;
                  if(theta > 90. || theta < -90.) {
                     InvalidRequest f("Invalid theta for setTostring: "
                                                + toBeRemoved);
                     GPSTK_THROW(f);
                  }
                  htheta = true;
                  break;
          //%p   phi() (degrees)
          //%P   phi() (radians)
               case 'p':
                  phi = asDouble(toBeRemoved);
                  if(phi < 0)
                     phi += 360*(1+(unsigned long)(phi/360));
                  else if(phi >= 360)
                     phi -= 360*(unsigned long)(phi/360);
                  hphi = true;
                  break;
               case 'P':
                  phi = asDouble(toBeRemoved) * RAD_TO_DEG;
                  if(phi < 0)
                     phi += 360*(1+(unsigned long)(phi/360));
                  else if(phi >= 360)
                     phi -= 360*(unsigned long)(phi/360);
                  hphi = true;
                  break;
          //%r   radius() meters
          //%R   radius()/1000 kilometers
          //%h   height() meters
          //%H   height()/1000 kilometers
               case 'r':
                  rad = asDouble(toBeRemoved);
                  if(rad < 0.0) {
                     InvalidRequest f("Invalid radius for setTostring: "
                                                + toBeRemoved);
                     GPSTK_THROW(f);
                  }
                  hrad = true;
                  break;
               case 'R':
                  rad = asDouble(toBeRemoved) * 1000;
                  if(rad < 0.0) {
                     InvalidRequest f("Invalid radius for setTostring: "
                                                + toBeRemoved);
                     GPSTK_THROW(f);
                  }
                  hrad = true;
                  break;
               case 'h':
                  ht = asDouble(toBeRemoved);
                  hht = true;
                  break;
               case 'H':
                  ht = asDouble(toBeRemoved) * 1000;
                  hht = true;
                  break;
               default: // do nothing
                  break;
            }
               // remove the part of s that we processed
            stripLeading(s,toBeRemoved,1);

               // remove the character we processed from f
            f.erase(0,1);

               // check for whitespace again...
            stripLeading(f);
            stripLeading(s);

         }

         if ( s.length() != 0  )
         {
               // throw an error - something didn't get processed in the strings
            InvalidRequest fe(
               "Processing error - parts of strings left unread - " + s);
            GPSTK_THROW(fe);
         }

         if (f.length() != 0)
         {
               // throw an error - something didn't get processed in the strings
            InvalidRequest fe(
               "Processing error - parts of strings left unread - " + f);
            GPSTK_THROW(fe);
         }

            // throw if the specification is incomplete
         if ( !(hX && hY && hZ) && !(hglat && hlon && hht) &&
              !(hclat && hlon && hrad) && !(htheta && hphi && hrad)) {
            InvalidRequest fe("Incomplete specification for setToString");
            GPSTK_THROW(fe);
         }

            // define the Position toReturn
         if(hX && hY && hZ)
            toReturn.setECEF(x,y,z);
         else if(hglat && hlon && hht)
            toReturn.setGeodetic(glat,lon,ht);
         else if(hclat && hlon && hrad)
            toReturn.setGeocentric(clat,lon,rad);
         else if(htheta && hphi && hrad)
            toReturn.setSpherical(theta,phi,rad);

         *this = toReturn;
         return *this;
      }
      catch(gpstk::Exception& exc)
      {
         GeometryException ge(exc);
         ge.addText("Failed to convert string to Position");
         GPSTK_THROW(ge);
      }
      catch(std::exception& exc)
      {
         GeometryException ge(exc.what());
         ge.addText("Failed to convert string to Position");
         GPSTK_THROW(ge);
      }
   }

      // Format this Position into a string.
      //
      // Generate and return a string containing formatted
      // Position coordinates, formatted by the specification \c fmt.
      //
      // \li \%x   X() (meters)
      // \li \%y   Y() (meters)
      // \li \%z   Z() (meters)
      // \li \%X   X()/1000 (kilometers)
      // \li \%Y   Y()/1000 (kilometers)
      // \li \%Z   Z()/1000 (kilometers)
      // \li \%A   geodeticLatitude() (degrees North)
      // \li \%a   geocentricLatitude() (degrees North)
      // \li \%L   longitude() (degrees East)
      // \li \%l   longitude() (degrees East)
      // \li \%w   longitude() (degrees West)
      // \li \%W   longitude() (degrees West)
      // \li \%t   theta() (degrees)
      // \li \%T   theta() (radians)
      // \li \%p   phi() (degrees)
      // \li \%P   phi() (radians)
      // \li \%r   radius() meters
      // \li \%R   radius()/1000 kilometers
      // \li \%h   height() meters
      // \li \%H   height()/1000 kilometers
      //
      // @param fmt format to use for this time.
      // @return a string containing this Position in the
      // representation specified by \c fmt.
   std::string Position::printf(const char *fmt) const
      throw(StringUtils::StringException)
   {
      string rv = fmt;
      rv = formattedPrint(rv, string("%[ 0-]?[[:digit:]]*(\\.[[:digit:]]+)?x"),
                          string("xf"), X());
      rv = formattedPrint(rv, string("%[ 0-]?[[:digit:]]*(\\.[[:digit:]]+)?y"),
                          string("yf"), Y());
      rv = formattedPrint(rv, string("%[ 0-]?[[:digit:]]*(\\.[[:digit:]]+)?z"),
                          string("zf"), Z());
      rv = formattedPrint(rv, string("%[ 0-]?[[:digit:]]*(\\.[[:digit:]]+)?X"),
                          string("Xf"), X()/1000);
      rv = formattedPrint(rv, string("%[ 0-]?[[:digit:]]*(\\.[[:digit:]]+)?Y"),
                          string("Yf"), Y()/1000);
      rv = formattedPrint(rv, string("%[ 0-]?[[:digit:]]*(\\.[[:digit:]]+)?Z"),
                          string("Zf"), Z()/1000);

      rv = formattedPrint(rv, string("%[ 0-]?[[:digit:]]*(\\.[[:digit:]]+)?A"),
                          string("Af"), geodeticLatitude());
      rv = formattedPrint(rv, string("%[ 0-]?[[:digit:]]*(\\.[[:digit:]]+)?a"),
                          string("af"), geocentricLatitude());
      rv = formattedPrint(rv, string("%[ 0-]?[[:digit:]]*(\\.[[:digit:]]+)?L"),
                          string("Lf"), longitude());
      rv = formattedPrint(rv, string("%[ 0-]?[[:digit:]]*(\\.[[:digit:]]+)?l"),
                          string("lf"), longitude());
      rv = formattedPrint(rv, string("%[ 0-]?[[:digit:]]*(\\.[[:digit:]]+)?w"),
                          string("wf"), 360-longitude());
      rv = formattedPrint(rv, string("%[ 0-]?[[:digit:]]*(\\.[[:digit:]]+)?W"),
                          string("Wf"), 360-longitude());

      rv = formattedPrint(rv, string("%[ 0-]?[[:digit:]]*(\\.[[:digit:]]+)?t"),
                          string("tf"), theta());
      rv = formattedPrint(rv, string("%[ 0-]?[[:digit:]]*(\\.[[:digit:]]+)?T"),
                          string("Tf"), theta()*DEG_TO_RAD);
      rv = formattedPrint(rv, string("%[ 0-]?[[:digit:]]*(\\.[[:digit:]]+)?p"),
                          string("pf"), phi());
      rv = formattedPrint(rv, string("%[ 0-]?[[:digit:]]*(\\.[[:digit:]]+)?P"),
                          string("Pf"), phi()*DEG_TO_RAD);
      rv = formattedPrint(rv, string("%[ 0-]?[[:digit:]]*(\\.[[:digit:]]+)?r"),
                          string("rf"), radius());
      rv = formattedPrint(rv, string("%[ 0-]?[[:digit:]]*(\\.[[:digit:]]+)?R"),
                          string("Rf"), radius()/1000);
      rv = formattedPrint(rv, string("%[ 0-]?[[:digit:]]*(\\.[[:digit:]]+)?h"),
                          string("hf"), height());
      rv = formattedPrint(rv, string("%[ 0-]?[[:digit:]]*(\\.[[:digit:]]+)?H"),
                          string("Hf"), height()/1000);
      return rv;
   }

      // Returns the string that operator<<() would print.
   string Position::asString() const
      throw(StringUtils::StringException)
   {
      ostringstream o;
      o << *this;
      return o.str();
   }

   // ----------- Part 10: functions: fundamental conversions ---------------
   //
      // Fundamental conversion from spherical to cartesian coordinates.
      // @param trp (input): theta, phi, radius
      // @param xyz (output): X,Y,Z in units of radius
      // Algorithm references: standard geometry.
   void Position::convertSphericalToCartesian(const Triple& tpr,
                                              Triple& xyz)
      throw()
   {
      double st=::sin(tpr[0]*DEG_TO_RAD);
      xyz[0] = tpr[2]*st*::cos(tpr[1]*DEG_TO_RAD);
      xyz[1] = tpr[2]*st*::sin(tpr[1]*DEG_TO_RAD);
      xyz[2] = tpr[2]*::cos(tpr[0]*DEG_TO_RAD);
   }

      // Fundamental routine to convert cartesian to spherical coordinates.
      // @param xyz (input): X,Y,Z
      // @param trp (output): theta, phi (deg), radius in units of input
      // Algorithm references: standard geometry.
   void Position::convertCartesianToSpherical(const Triple& xyz,
                                              Triple& tpr)
      throw()
   {
      tpr[2] = RSS(xyz[0],xyz[1],xyz[2]);
      if(tpr[2] <= Position::POSITION_TOLERANCE/5) { // zero-length Cartesian vector
         tpr[0] = 90;
         tpr[1] = 0;
         return;
      }
      tpr[0] = ::acos(xyz[2]/tpr[2]);
      tpr[0] *= RAD_TO_DEG;
      if(RSS(xyz[0],xyz[1]) < Position::POSITION_TOLERANCE/5) {       // pole
         tpr[1] = 0;
         return;
      }
      tpr[1] = ::atan2(xyz[1],xyz[0]);
      tpr[1] *= RAD_TO_DEG;
      if(tpr[1] < 0) tpr[1] += 360;
   }

      // Fundamental routine to convert cartesian (ECEF) to geodetic coordinates,
      // (Geoid specified by semi-major axis and eccentricity squared).
      // @param xyz (input): X,Y,Z in meters
      // @param llh (output): geodetic lat(deg N), lon(deg E),
      //                             height above ellipsoid (meters)
      // @param A (input) Earth semi-major axis
      // @param eccSq (input) square of Earth eccentricity
      // Algorithm references:
   void Position::convertCartesianToGeodetic(const Triple& xyz,
                                             Triple& llh,
                                             const double A,
                                             const double eccSq)
      throw()
   {
      double p,slat,N,htold,latold;
      p = SQRT(xyz[0]*xyz[0]+xyz[1]*xyz[1]);
      if(p < Position::POSITION_TOLERANCE/5) {  // pole or origin
         llh[0] = (xyz[2] > 0 ? 90.0: -90.0);
         llh[1] = 0;                            // lon undefined, really
         llh[2] = ::fabs(xyz[2]) - A*SQRT(1.0-eccSq);
         return;
      }
      llh[0] = ::atan2(xyz[2], p*(1.0-eccSq));
      llh[2] = 0;
      for(int i=0; i<5; i++) {
         slat = ::sin(llh[0]);
         N = A / SQRT(1.0 - eccSq*slat*slat);
         htold = llh[2];
         llh[2] = p/::cos(llh[0]) - N;
         latold = llh[0];
         llh[0] = ::atan2(xyz[2], p*(1.0-eccSq*(N/(N+llh[2]))));
         if(::fabs(llh[0]-latold) < 1.0e-9 && fabs(llh[2]-htold) < 1.0e-9 * A) break;
      }
      llh[1] = ::atan2(xyz[1],xyz[0]);
      if(llh[1] < 0.0) llh[1] += TWO_PI;
      llh[0] *= RAD_TO_DEG;
      llh[1] *= RAD_TO_DEG;
   }

      // Fundamental routine to convert geodetic to cartesian (ECEF) coordinates,
      // (Geoid specified by semi-major axis and eccentricity squared).
      // @param llh (input): geodetic lat(deg N), lon(deg E),
      //            height above ellipsoid (meters)
      // @param xyz (output): X,Y,Z in meters
      // @param A (input) Earth semi-major axis
      // @param eccSq (input) square of Earth eccentricity
      // Algorithm references:
   void Position::convertGeodeticToCartesian(const Triple& llh,
                                             Triple& xyz,
                                             const double A,
                                             const double eccSq)
      throw()
   {
      double slat = ::sin(llh[0]*DEG_TO_RAD);
      double clat = ::cos(llh[0]*DEG_TO_RAD);
      double N = A/SQRT(1.0-eccSq*slat*slat);
      xyz[0] = (N+llh[2])*clat*::cos(llh[1]*DEG_TO_RAD);
      xyz[1] = (N+llh[2])*clat*::sin(llh[1]*DEG_TO_RAD);
      xyz[2] = (N*(1.0-eccSq)+llh[2])*slat;
   }

      // Fundamental routine to convert cartesian (ECEF) to geocentric coordinates.
      // @param xyz (input): X,Y,Z in meters
      // @param llr (output):
      //            geocentric lat(deg N),lon(deg E),radius (units of input)
   void Position::convertCartesianToGeocentric(const Triple& xyz,
                                               Triple& llr)
      throw()
   {
      convertCartesianToSpherical(xyz, llr);
      llr[0] = 90 - llr[0];         // convert theta to latitude
   }

      // Fundamental routine to convert geocentric to cartesian (ECEF) coordinates.
      // @param llr (input): geocentric lat(deg N),lon(deg E),radius
      // @param xyz (output): X,Y,Z (units of radius)
   void Position::convertGeocentricToCartesian(const Triple& llr,
                                               Triple& xyz)
      throw()
   {
      Triple llh(llr);
      llh[0] = 90 - llh[0];         // convert latitude to theta
      convertSphericalToCartesian(llh, xyz);
   }

      // Fundamental routine to convert geocentric to geodetic coordinates.
      // @param llr (input): geocentric Triple: lat(deg N),lon(deg E),radius (meters)
      // @param llh (output): geodetic latitude (deg N),
      //            longitude (deg E), and height above ellipsoid (meters)
      // @param A (input) Earth semi-major axis
      // @param eccSq (input) square of Earth eccentricity
   void Position::convertGeocentricToGeodetic(const Triple& llr,
                                               Triple& llh,
                                               const double A,
                                               const double eccSq)
      throw()
   {
      double cl,p,sl,slat,N,htold,latold;
      llh[1] = llr[1];   // longitude is unchanged
      cl = ::sin((90-llr[0])*DEG_TO_RAD);
      sl = ::cos((90-llr[0])*DEG_TO_RAD);
      if(llr[2] <= Position::POSITION_TOLERANCE/5) {
         // radius is below tolerance, hence assign zero-length
         // arbitrarily set latitude = longitude = 0
         llh[0] = llh[1] = 0;
         llh[2] = -A;
         return;
      }
      else if(cl < 1.e-10) {
         // near pole ... note that 1mm/radius(Earth) = 1.5e-10
         if(llr[0] < 0) llh[0] = -90;
         else           llh[0] =  90;
         llh[1] = 0;
         llh[2] = llr[2] - A*SQRT(1-eccSq);
         return;
      }
      llh[0] = ::atan2(sl, cl*(1.0-eccSq));
      p = cl*llr[2];
      llh[2] = 0;
      for(int i=0; i<5; i++) {
         slat = ::sin(llh[0]);
         N = A / SQRT(1.0 - eccSq*slat*slat);
         htold = llh[2];
         llh[2] = p/::cos(llh[0]) - N;
         latold = llh[0];
         llh[0] = ::atan2(sl, cl*(1.0-eccSq*(N/(N+llh[2]))));
         if(fabs(llh[0]-latold) < 1.0e-9 && ::fabs(llh[2]-htold) < 1.0e-9 * A) break;
      }
      llh[0] *= RAD_TO_DEG;
   }

      // Fundamental routine to convert geodetic to geocentric coordinates.
      // @param geodeticllh (input): geodetic latitude (deg N),
      //            longitude (deg E), and height above ellipsoid (meters)
      // @param llr (output): geocentric lat (deg N),lon (deg E),radius (meters)
      // @param A (input) Earth semi-major axis
      // @param eccSq (input) square of Earth eccentricity
   void Position::convertGeodeticToGeocentric(const Triple& llh,
                                              Triple& llr,
                                              const double A,
                                              const double eccSq)
      throw()
   {
      double slat = ::sin(llh[0]*DEG_TO_RAD);
      double N = A/SQRT(1.0-eccSq*slat*slat);
      // longitude is unchanged
      llr[1] = llh[1];
      // radius
      llr[2] = SQRT((N+llh[2])*(N+llh[2]) + N*eccSq*(N*eccSq-2*(N+llh[2]))*slat*slat);
      if(llr[2] <= Position::POSITION_TOLERANCE/5) {
         // radius is below tolerance, hence assign zero-length
         // arbitrarily set latitude = longitude = 0
         llr[0] = llr[1] = llr[2] = 0;
         return;
      }
      if(1-::fabs(slat) < 1.e-10) {             // at the pole
         if(slat < 0) llr[0] = -90;
         else         llr[0] =  90;
         llr[1] = 0.0;
         return;
      }
      // theta
      llr[0] = ::acos((N*(1-eccSq)+llh[2])*slat/llr[2]);
      llr[0] *= RAD_TO_DEG;
      llr[0] = 90 - llr[0];
   }

   // ----------- Part 11: operator<< and other useful functions -------------
   //
     // Stream output for Position objects.
     // @param s stream to append formatted Position to.
     // @param t Position to append to stream \c s.
     // @return reference to \c s.
   ostream& operator<<(ostream& s, const Position& p)
   {
      if(p.system == Position::Cartesian)
         s << p.printf("%.4x m %.4y m %.4z m");
      else if(p.system == Position::Geodetic)
         s << p.printf("%.8A degN %.8L degE %.4h m");
      else if(p.system == Position::Geocentric)
         s << p.printf("%.8a degN %.8L degE %.4r m");
      else if(p.system == Position::Spherical)
         s << p.printf("%.8t deg %.8p deg %.4r m");
      else
         s << " Unknown system! : " << p[0] << " " << p[1] << " " << p[2];

      return s;
   }

      // Compute the range in meters between this Position and
      // the Position passed as input.
      // @param right Position to which to find the range
      // @return the range (in meters)
      // @throw GeometryException if ell values differ
   double range(const Position& A,
                const Position& B)
      throw(GeometryException)
   {
      if(A.AEarth != B.AEarth || A.eccSquared != B.eccSquared)
      {
         GeometryException ge("Unequal geoids");
         GPSTK_THROW(ge);
      }

         Position L(A),R(B);
         L.transformTo(Position::Cartesian);
         R.transformTo(Position::Cartesian);
         double dif = RSS(L.X()-R.X(),L.Y()-R.Y(),L.Z()-R.Z());
         return dif;
      }

     // Compute the radius of the ellipsoidal Earth, given the geodetic latitude.
     // @param geolat geodetic latitude in degrees
     // @return the Earth radius (in meters)
   double Position::radiusEarth(const double geolat,
                                const double A,
                                const double eccSq)
      throw()
   {
      double slat=::sin(DEG_TO_RAD*geolat);
      double e=(1.0-eccSq);
      double f=(1.0+(e*e-1.0)*slat*slat)/(1.0-eccSq*slat*slat);
      return (A * SQRT(f));
   }

      // A member function that computes the elevation of the input
      // (Target) position as seen from this Position.
      // @param Target the Position which is observed to have the
      //        computed elevation, as seen from this Position.
      // @return the elevation in degrees
   double Position::elevation(const Position& Target) const
      throw(GeometryException)
   {
      Position R(*this),S(Target);
      R.transformTo(Cartesian);
      S.transformTo(Cartesian);
      // use Triple:: functions in cartesian coordinates (only)
      double elevation;
      try {
         elevation = R.elvAngle(S);
      }
      catch(GeometryException& ge)
      {
         GPSTK_RETHROW(ge);
      }
      return elevation;
   }

      // A member function that computes the elevation of the input
      // (Target) position as seen from this Position, using a Geodetic
      // (i.e. ellipsoidal) system.
      // @param Target the Position which is observed to have the
      //        computed elevation, as seen from this Position.
      // @return the elevation in degrees
   double Position::elevationGeodetic(const Position& Target) const
      throw(GeometryException)
   {
      Position R(*this),S(Target);
      double latGeodetic = R.getGeodeticLatitude()*DEG_TO_RAD;
      double longGeodetic = R.getLongitude()*DEG_TO_RAD;
      double localUp;
      double cosUp;
      R.transformTo(Cartesian);
      S.transformTo(Cartesian);
      Triple z;
      // Let's get the slant vector
      z = S.theArray - R.theArray;

      if (z.mag()<=1e-4) // if the positions are within .1 millimeter
      {
         GeometryException ge("Positions are within .1 millimeter");
         GPSTK_THROW(ge);
      }

      // Compute k vector in local North-East-Up (NEU) system
      Triple kVector(::cos(latGeodetic)*::cos(longGeodetic), ::cos(latGeodetic)*::sin(longGeodetic), ::sin(latGeodetic));
      // Take advantage of dot method to get Up coordinate in local NEU system
      localUp = z.dot(kVector);
      // Let's get cos(z), being z the angle with respect to local vertical (Up);
      cosUp = localUp/z.mag();

      return 90.0 - ((::acos(cosUp))*RAD_TO_DEG);
   }

      // A member function that computes the azimuth of the input
      // (Target) position as seen from this Position.
      // @param Target the Position which is observed to have the
      //        computed azimuth, as seen from this Position.
      // @return the azimuth in degrees
   double Position::azimuth(const Position& Target) const
      throw(GeometryException)
   {
      Position R(*this),S(Target);
      R.transformTo(Cartesian);
      S.transformTo(Cartesian);
      // use Triple:: functions in cartesian coordinates (only)
      double az;
      try
      {
         az = R.azAngle(S);

      }
      catch(GeometryException& ge)
      {
         GPSTK_RETHROW(ge);
      }

      return az;
   }

      // A member function that computes the azimuth of the input
      // (Target) position as seen from this Position, using a Geodetic
      // (i.e. ellipsoidal) system.
      // @param Target the Position which is observed to have the
      //        computed azimuth, as seen from this Position.
      // @return the azimuth in degrees
   double Position::azimuthGeodetic(const Position& Target) const
      throw(GeometryException)
   {
      Position R(*this),S(Target);
      double latGeodetic = R.getGeodeticLatitude()*DEG_TO_RAD;
      double longGeodetic = R.getLongitude()*DEG_TO_RAD;
      double localN, localE;
      R.transformTo(Cartesian);
      S.transformTo(Cartesian);
      Triple z;
      // Let's get the slant vector
      z = S.theArray - R.theArray;

      if (z.mag()<=1e-4) // if the positions are within .1 millimeter
      {
         GeometryException ge("Positions are within .1 millimeter");
         GPSTK_THROW(ge);
      }

      // Compute i vector in local North-East-Up (NEU) system
      Triple iVector(-::sin(latGeodetic)*::cos(longGeodetic), -::sin(latGeodetic)*::sin(longGeodetic), ::cos(latGeodetic));
      // Compute j vector in local North-East-Up (NEU) system
      Triple jVector(-::sin(longGeodetic), ::cos(longGeodetic), 0);

      // Now, let's use dot product to get localN and localE unitary vectors
      localN = (z.dot(iVector))/z.mag();
      localE = (z.dot(jVector))/z.mag();

      // Let's test if computing azimuth has any sense
      double test = fabs(localN) + fabs(localE);

      // Warning: If elevation is very close to 90 degrees, we will return azimuth = 0.0
      if (test < 1.0e-16) return 0.0;

      double alpha = ((::atan2(localE, localN)) * RAD_TO_DEG);
      if (alpha < 0.0)
      {
         return alpha + 360.0;
      }
      else
      {
         return alpha;
      }
   }

     // A member function that computes the point at which a signal, which
     // is received at *this Position and there is observed at the input
     // azimuth and elevation, crosses a model ionosphere that is taken to
     // be a uniform thin shell at the input height. This algorithm is done
     // in geocentric coordinates.
     // A member function that computes the point at which a signal, which
     // is received at *this Position and there is observed at the input
     // azimuth and elevation, crosses a model ionosphere that is taken to
     // be a uniform thin shell at the input height. This algorithm is done
     // in geocentric coordinates.
     // @param elev elevation angle of the signal at reception, in degrees
     // @param azim azimuth angle of the signal at reception, in degrees
     // @param ionoht height of the ionosphere, in meters
     // @return Position IPP the position of the ionospheric pierce point,
     //     in the same coordinate system as *this; *this is not modified.
   Position Position::getIonosphericPiercePoint(const double elev,
                                                const double azim,
                                                const double ionoht) const
      throw()
   {
      Position Rx(*this);

      // convert to Geocentric
      Rx.transformTo(Geocentric);

      // compute the geographic pierce point
      Position IPP(Rx);                   // copy system and geoid
      double el = elev * DEG_TO_RAD;
      // p is the angle subtended at Earth center by Rx and the IPP
      double p = PI/2.0 - el - ::asin(AEarth*::cos(el)/(AEarth+ionoht));
      double lat = Rx.theArray[0] * DEG_TO_RAD;
      double az = azim * DEG_TO_RAD;
      IPP.theArray[0] = std::asin(std::sin(lat)*std::cos(p) + std::cos(lat)*std::sin(p)*std::cos(az));
      IPP.theArray[1] = Rx.theArray[1]*DEG_TO_RAD
         + ::asin(::sin(p)*::sin(az)/::cos(IPP.theArray[0]));

      IPP.theArray[0] *= RAD_TO_DEG;
      IPP.theArray[1] *= RAD_TO_DEG;
      IPP.theArray[2] = AEarth + ionoht;

      // transform back
      IPP.transformTo(system);

      return IPP;
   }


        /**
        * A member function that computes the radius of curvature of the 
        * meridian (Rm) corresponding to this Position.
        * @return radius of curvature of the meridian (in meters)
        */
    double Position::getCurvMeridian() const
        throw()
    {
    
        double slat = ::sin(geodeticLatitude()*DEG_TO_RAD);
        double W = 1.0/SQRT(1.0-eccSquared*slat*slat);
        
        return AEarth*(1.0-eccSquared)*W*W*W;
        
    }

        /**
        * A member function that computes the radius of curvature in the 
        * prime vertical (Rn) corresponding to this Position.
        * @return radius of curvature in the prime vertical (in meters)
        */
    double Position::getCurvPrimeVertical() const
        throw()
    {
    
        double slat = ::sin(geodeticLatitude()*DEG_TO_RAD);
        
        return AEarth/SQRT(1.0-eccSquared*slat*slat);
        
    }

   // ----------- Part 12: private functions and member data -----------------
   //
      // Initialization function, used by the constructors.
      // @param a coordinate [ X(m), or latitude (degrees N) ]
      // @param b coordinate [ Y(m), or longitude (degrees E) ]
      // @param c coordinate [ Z, height above ellipsoid or radius, in m ]
      // @param s CoordinateSystem, defaults to Cartesian
      // @param geiod pointer to a GeoidModel, default NULL (WGS84)
      // @throw GeometryException on invalid input.
   void Position::initialize(const double a,
                  const double b,
                  const double c,
                  Position::CoordinateSystem s,
                  EllipsoidModel *ell,
                  ReferenceFrame frame)
      throw(GeometryException)
   {
      double bb(b);
      if(s == Geodetic || s==Geocentric)
      {
         if(a > 90 || a < -90)
         {
            GeometryException ge("Invalid latitude in constructor: "
                                    + StringUtils::asString(a));
            GPSTK_THROW(ge);
         }
         if(bb < 0)
            bb += 360*(1+(unsigned long)(bb/360));
         else if(bb >= 360)
            bb -= 360*(unsigned long)(bb/360);
      }
      if(s==Geocentric || s==Spherical)
      {
         if(c < 0)
         {
            GeometryException ge("Invalid radius in constructor: "
                                           + StringUtils::asString(c));
            GPSTK_THROW(ge);
         }
      }
      if(s==Spherical)
      {
         if(a < 0 || a > 180)
         {
            GeometryException ge("Invalid theta in constructor: "
                                    + StringUtils::asString(a));
            GPSTK_THROW(ge);
         }
         if(bb < 0)
            bb += 360*(1+(unsigned long)(bb/360));
         else if(bb >= 360)
            bb -= 360*(unsigned long)(bb/360);
      }

      theArray[0] = a;
      theArray[1] = bb;
      theArray[2] = c;

      if(ell) {
         AEarth = ell->a();
         eccSquared = ell->eccSquared();
      }
      else {
         WGS84Ellipsoid WGS84;
         AEarth = WGS84.a();
         eccSquared = WGS84.eccSquared();
      }
      system = s;
      tolerance = POSITION_TOLERANCE;
      refFrame = frame;
   }

}  // namespace gpstk
