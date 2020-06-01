#pragma ident "$Id$"

#ifndef GPSTK_SATID_HPP
#define GPSTK_SATID_HPP

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

//============================================================================
//
//This software developed by Applied Research Laboratories at the University of
//Texas at Austin, under contract to an agency or agencies within the U.S. 
//Department of Defense. The U.S. Government retains all rights to use,
//duplicate, distribute, disclose, or release this software. 
//
//Pursuant to DoD Directive 523024 
//
// DISTRIBUTION STATEMENT A: This software has been approved for public 
//                           release, distribution is unlimited.
//
//=============================================================================

#include <iostream>
#include <iomanip>
#include <sstream>
#include "gps_constants.hpp"

/**
 * @file SatID.hpp
 * gpstk::SatID - navigation system-independent representation of a satellite.
 */

namespace gpstk
{
   // forward declarations
   class SatID;
//   std::istream& operator<<(std::istream& s, SatID& p);

   /// Satellite identifier consisting of a satellite number (PRN, etc.)
   /// and a satellite system
   class SatID
   {
   public:
      /// Supported satellite systems
      enum SatelliteSystem
      {
         systemGPS = 1,
         systemGalileo,
         systemGlonass,
         systemGeosync,
         systemLEO,
         systemTransit,
         systemBeiDou,
         systemQZSS,
         systemMixed,
         systemUserDefined,
         systemUnknown
      };

      /// empty constructor, creates an invalid object
      SatID() { id=-1; system=systemGPS; }

      /// explicit constructor, no defaults
      /// @note if s is given a default value here,
      /// some compilers will silently cast int to SatID.
      SatID(int p, SatelliteSystem s) { id=p; system=s; }

      // operator=, copy constructor and destructor built by compiler

      /// Convenience method used by dump().
      static std::string convertSatelliteSystemToString(SatelliteSystem s)
      {
         switch(s)
         {
            case systemGPS:         return "GPS";           break;
            case systemGalileo:     return "Galileo";       break;
            case systemGlonass:     return "GLONASS";       break;
            case systemGeosync:     return "Geostationary"; break;
            case systemLEO:         return "LEO";           break;
            case systemTransit:     return "Transit";       break;
            case systemBeiDou:      return "BeiDou";        break;
            case systemQZSS:        return "QZSS";          break;
            case systemMixed:       return "Mixed";         break;
            case systemUserDefined: return "UserDefined";   break;
            case systemUnknown:     return "Unknown";       break;
            default:                return "??";            break;
         };
      }

         /// Convenience output method.
      void dump(std::ostream& s) const
      {
         s << convertSatelliteSystemToString(system) << " " << id;
      }

      /// operator == for SatID
      bool operator==(const SatID& right) const
      { return ((system == right.system) && (id == right.id)); }

      /// operator != for SatID
      bool operator!=(const SatID& right) const
      { return !(operator==(right)); }

      /// operator < for SatID : order by system, then number
      bool operator<(const SatID& right) const
      {
         if (system==right.system)
            return (id<right.id);
         return (system<right.system);
      }

      /// operator > for SatID
      bool operator>(const SatID& right) const
      {  return (!operator<(right) && !operator==(right)); }

      /// operator <= for SatID
      bool operator<=(const SatID& right) const
      { return (operator<(right) || operator==(right)); }

      /// operator >= for SatID
      bool operator>=(const SatID& right) const
      { return !(operator<(right)); }

      /// return true if this is a valid SatID
      /// @note assumes all id's are positive and less than 100;
      ///     plus GPS id's are less than or equal to MAX_PRN (32).
      /// @note this is not used internally in the gpstk library
      bool isValid() const
      {
         switch(system)
         {
            case systemGPS: return (id > 0 && id <= MAX_PRN);
            //case systemGalileo:
            //case systemGlonass:
            //case systemGeosync:
            //case systemLEO:
            //case systemTransit:
            default: return (id > 0 && id < 100);
         }
      }

      int id;                   ///< satellite identifier, e.g. PRN
      SatelliteSystem system;   ///< system for this satellite

   }; // class SatID

   namespace StringUtils
   {
      inline std::string asString(const SatID& p)
      {
         std::ostringstream oss;
         p.dump(oss);
         return oss.str();
      }
   }

      /// stream output for SatID
   inline std::ostream& operator<<(std::ostream& s, const SatID& p)
   {
      p.dump(s);
      return s;
   }

} // namespace gpstk

#endif
