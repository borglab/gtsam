#pragma ident "$Id$"

#ifndef GPSTK_RINEX_SATID_HPP
#define GPSTK_RINEX_SATID_HPP

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
// This software developed by Applied Research Laboratories at the University
// of Texas at Austin, under contract to an agency or agencies within the U.S. 
// Department of Defense. The U.S. Government retains all rights to use,
// duplicate, distribute, disclose, or release this software. 
//
// Pursuant to DoD Directive 523024 
//
// DISTRIBUTION STATEMENT A: This software has been approved for public 
//                           release, distribution is unlimited.
//
//=============================================================================

#include <iostream>
#include <sstream>
#include <iomanip>

#include "Exception.hpp"
#include "SatID.hpp"

/**
 * @file RinexSatID.hpp
 * gpstk::RinexSatID - Navigation system-independent representation of a
 *                     satellite, as defined by the RINEX specification.
 */

namespace gpstk
{
   class RinexSatID : public SatID
   {

   public:

      /// Empty constructor; creates an invalid object (Unknown, ID = -1).

      RinexSatID()
         throw()
      { id = -1; system = systemUnknown; }


      /// Explicit constructor, no defaults, RINEX systems only.

      RinexSatID(int p, SatelliteSystem s)
         throw()
      {
         id = p; system = s;
         switch(s)
         {
            case systemGPS:
            case systemGalileo:
            case systemGlonass:
            case systemGeosync:
            case systemTransit:
            case systemQZSS:
            case systemBeiDou:
            case systemMixed:
               break;
            // Invalidate anything non-RINEX.
            default:
               system = systemUnknown;
               id = -1;
         }
      }


      /// Constructor from a string.

      RinexSatID(const std::string& str)
         throw(Exception)
      {
         try { fromString(str); }
         catch(Exception& e) { GPSTK_RETHROW(e); }
      }


      /// Cast a SatID to a RinexSatID.

      RinexSatID(const SatID& sat)
         throw()
      { *this = RinexSatID(sat.id,sat.system); }


      /// Set the fill character used in output and
      /// return the current fill character.

      char setfill(char c)
         throw()
      { char csave = fillchar; fillchar = c; return csave; }


      /// Get the fill character used in output.

      char getfill()
         throw()
      { return fillchar; }


      // operator=, copy constructor and destructor built by compiler


      /// Return the single-character system descriptor.
      /// @note return only RINEX types, for non-RINEX systems return '?'

      char systemChar() const
         throw()
      {
         switch(system)
         {
            case systemGPS:     return 'G';
            case systemGalileo: return 'E';
            case systemGlonass: return 'R';
            case systemGeosync: return 'S';
            case systemTransit: return 'T';
            case systemQZSS:    return 'J';
            case systemBeiDou:  return 'C';
            default:            return '?';
         }
      };


      /// Return the system name as a string.
      /// @note Return only RINEX types or 'Unknown'.
      std::string systemString() const
         throw()
      {
         switch(system)
         {
            case systemGPS:     return "GPS";
            case systemGalileo: return "Galileo";
            case systemGlonass: return "GLONASS";
            case systemGeosync: return "Geosync";
            case systemTransit: return "Transit";
            case systemQZSS:    return "QZSS";
            case systemBeiDou:  return "BeiDou";
            default:            return "Unknown";
         }
      };

      /// Return the system name as a string of length 3.
      /// @note Return only RINEX types or 'Unknown'.
      std::string systemString3() const
         throw()
      {
         switch(system)
         {
            case systemGPS:     return "GPS";
            case systemGalileo: return "GAL";
            case systemGlonass: return "GLO";
            case systemGeosync: return "GEO";
            case systemTransit: return "TRN";     // RINEX ver 2
            case systemQZSS:    return "QZS";
            case systemBeiDou:  return "BDS";
            default:            return "Unk";
         }
      };


      /// Set the RinexSatID from a string (1 character plus 2-digit integer).
      /// @note GPS is default system (no or unknown system char)

      void fromString(const std::string s)
         throw(Exception)
      {
         char c;
         std::istringstream iss(s);

         id = -1; system = systemGPS;  // default
         if(s.find_first_not_of(std::string(" \t\n"), 0) == std::string::npos)
            return;                    // all whitespace yields the default

         iss >> c;                     // read one character (non-whitespace)
         switch(c)
         {
                                       // no leading system character
            case '0': case '1': case '2': case '3': case '4':
            case '5': case '6': case '7': case '8': case '9':
               iss.putback(c);
               system = SatID::systemGPS;
               break;
            case 'R': case 'r':
               system = SatID::systemGlonass;
               break;
            case 'T': case 't':
               system = SatID::systemTransit;
               break;
            case 'S': case 's':
               system = SatID::systemGeosync;
               break;
            case 'E': case 'e':
               system = SatID::systemGalileo;
               break;
            case 'M': case 'm':
               system = SatID::systemMixed;
               break;
            case ' ': case 'G': case 'g':
               system = SatID::systemGPS;
               break;
            case 'J': case 'j':
               system = SatID::systemQZSS;
               break;
            case 'C': case 'c':
               system = SatID::systemBeiDou;
               break;
            default:                   // non-RINEX system character
               Exception e(std::string("Invalid system character \"")
                           + c + std::string("\""));
               GPSTK_THROW(e);
         }
         iss >> id;
         if(id <= 0) id = -1;
      }


      /// Convert the RinexSatID to string (1 character plus 2-digit integer).

      std::string toString() const
         throw()
      {
         std::ostringstream oss;
         oss.fill(fillchar);
         oss << systemChar() << std::setw(2) << id;
         return oss.str();
      }


   private:

      static char fillchar;  ///< Fill character used during stream output.

   }; // class RinexSatID

   /// Stream output for RinexSatID.

   inline std::ostream& operator<<(std::ostream& s, const RinexSatID& sat)
   {
      s << sat.toString();
      return s;
   }

} // namespace gpstk

#endif
