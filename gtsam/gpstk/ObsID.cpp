/// @file ObsID.cpp
/// gpstk::ObsID - Identifies types of observations

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

#include "ObsID.hpp"

namespace gpstk
{
   // descriptions (strings) of each code, carrier and type
   std::map< ObsID::TrackingCode,    std::string > ObsID::tcDesc;
   std::map< ObsID::CarrierBand,     std::string > ObsID::cbDesc;
   std::map< ObsID::ObservationType, std::string > ObsID::otDesc;
   // mappings between code, carrier, type and characters
   std::map< char, ObsID::ObservationType> ObsID::char2ot;
   std::map< char, ObsID::CarrierBand> ObsID::char2cb;
   std::map< char, ObsID::TrackingCode> ObsID::char2tc;
   std::map< ObsID::ObservationType, char > ObsID::ot2char;
   std::map< ObsID::CarrierBand, char > ObsID::cb2char;
   std::map< ObsID::TrackingCode, char> ObsID::tc2char;

   // map of valid RINEX tracking codes, systems and frequency
   std::map<char, std::map<char, std::string> > ObsID::validRinexTrackingCodes;

   // string containing the system characters for all valid RINEX systems.
   std::string ObsID::validRinexSystems;

   // maps between 1-char and 3-char system id
   std::map<std::string, std::string> ObsID::map1to3sys;
   std::map<std::string, std::string> ObsID::map3to1sys;

   // string containing the frequency digits for all valid RINEX systems.
   std::string ObsID::validRinexFrequencies;

   // object that forces initialization of the maps
   ObsIDInitializer singleton;

   // Construct this object from the string specifier
   ObsID::ObsID(const std::string& strID) throw(InvalidParameter)
   {
      int i = strID.length() - 3;
      if ( i < 0 || i > 1)
      {
         InvalidParameter e("identifier must be 3 or 4 characters long");
         GPSTK_THROW(e);
      }

      char sys = i ? strID[0] : 'G';
      char ot = strID[i];
      char cb = strID[i+1];
      char tc = strID[i+2];
      
      if (!char2ot.count(ot) || !char2cb.count(cb) || !char2tc.count(tc))
         idCreator(strID.substr(i,3));

      type = char2ot[ ot ];
      band = char2cb[ cb ];
      code = char2tc[ tc ];

      /// This next block takes care of fixing up the codes that are reused
      /// between the various signals
      if (sys == 'G') // GPS
      {
         if (tc=='X' && band==cbL5)
            code = tcIQ5;
      }
      if (sys == 'E') // Galileo
      {
         switch (code)
         {
            case tcCA: code = tcC; break;
            case tcI5: code = tcIE5; break;
            case tcQ5: code = tcQE5; break;
            default: break;
         }
         if (tc == 'X')
         {
            if (band == cbL1 || band == cbE6)
               code = tcBC;
            else if (band == cbL5 || band == cbE5b || band == cbE5ab)
               code = tcIQE5;
         }
      }
      else if (sys == 'R') // Glonass
      {
         switch (code)
         {
            case tcCA: code = tcGCA; break;
            case tcP: code = tcGP; break;
            case tcI5: code = tcIR3; break;
            case tcQ5: code = tcQR3; break;
            case tcC2LM: code = tcIQR3; break;
            default: break;
         }
         switch (band)
         {
            case cbL1: band = cbG1; break;
            case cbL2: band = cbG2; break;
            default: break;
         }
      }
      else if (sys == 'S') // SBAS or Geosync
      {
         switch (code)
         {
            case tcCA: code = tcSCA; break;     // 'C'
            case tcI5: code = tcSI5; break;     // 'I'
            case tcQ5: code = tcSQ5; break;     // 'Q'
            case tcC2LM: code = tcSIQ5; break;  // 'X'
            default: break;
         }
      }
      else if (sys == 'J') // QZSS
      {
         if(band == cbL1) switch (code)
         {
            case tcCA: code = tcJCA; break;     // 'C'
            case tcC2M: code = tcJD1; break;    // 'S'
            case tcC2L: code = tcJP1; break;    // 'L'
            case tcC2LM: code = tcJX1; break;   // 'X'
            case tcABC: code = tcJZ1; break;    // 'Z'
            default: break;
         }
         if(band == cbL2) switch (code)
         {
            case tcC2M: code = tcJM2; break;    // 'S'
            case tcC2L: code = tcJL2; break;    // 'L'
            case tcC2LM: code = tcJX2; break;   // 'X'
            default: break;
         }
         if(band == cbL5) switch (code)
         {
            case tcI5: code = tcJI5; break;     // 'I'
            case tcQ5: code = tcJQ5; break;     // 'Q'
            case tcC2LM: code = tcJIQ5; break;  // 'X'
            default: break;
         }
         if(band == cbE6) switch (code)
         {
            case tcC2M: code = tcJI6; break;    // 'S'
            case tcC2L: code = tcJQ6; break;    // 'L'
            case tcC2LM: code = tcJIQ6; break;  // 'X'
            default: break;
         }
      }
      else if (sys == 'C') // BeiDou
      {
         if(band == cbL1) band = cbB1;
         if(band == cbE6) band = cbB3;

         if(band == cbB1) {
            switch (code)
            {
               case tcI5: code = tcCI1; break;     // 'I'
               case tcQ5: code = tcCQ1; break;     // 'Q'
               case tcC2LM: code = tcCIQ1; break;  // 'X'
               default: break;
            }
         }
         if(band == cbB3) switch (code)
         {
            case tcI5: code = tcCI7; break;     // 'I'
            case tcQ5: code = tcCQ7; break;     // 'Q'
            case tcC2LM: code = tcCIQ7; break;  // 'X'
            default: break;
         }
         if(band == cbE5b) {
            switch (code)
            {
               case tcI5: code = tcCI6; break;     // 'I'
               case tcQ5: code = tcCQ6; break;     // 'Q'
               case tcC2LM: code = tcCIQ6; break;  // 'X'
            default: break;
            }
         }
      } // end of checking which GNSS system this obs is for
   }


   // Convenience output method
   std::ostream& ObsID::dump(std::ostream& s) const
   {
      s << ObsID::cbDesc[band] << " "
        << ObsID::tcDesc[code] << " "
        << ObsID::otDesc[type];
      return s;
   } // ObsID::dump()




   // This is used to register a new ObsID & Rinex 3 identifier.  The syntax for the
   // Rinex 3 identifier is the same as for the ObsID constructor. 
   // If there are spaces in the provided identifier, they are ignored
   ObsID ObsID::newID(const std::string& strID, const std::string& desc)
      throw(InvalidParameter)
   {
      if (char2ot.count(strID[0]) && 
          char2cb.count(strID[1]) && 
          char2tc.count(strID[2]))
          GPSTK_THROW(InvalidParameter("Identifier " + strID + " already defined."));

      return idCreator(strID, desc);
   }


   ObsID ObsID::idCreator(const std::string& strID, const std::string& desc)
   {
      char ot = strID[0];
      ObservationType type;
      if (!char2ot.count(ot))
      {
         type = (ObservationType)otDesc.size();
         otDesc[type] = desc;
         char2ot[ot] = type;
         ot2char[type] = ot;
      }
      else
         type = char2ot[ot];

      char cb = strID[1];
      CarrierBand band;
      if (!char2cb.count(cb))
      {
         band = (CarrierBand)cbDesc.size();
         cbDesc[band] = desc;
         char2cb[cb] = band;
         cb2char[band] = cb;
      }
      else
         band = char2cb[cb];

      char tc = strID[2];
      TrackingCode code;
      if (!char2tc.count(tc))
      {
         code = (TrackingCode) tcDesc.size();
         tcDesc[code] = desc;
         char2tc[tc] = code;
         tc2char[code] = tc;
      }
      else
         code = char2tc[tc];
      
      return ObsID(type, band, code);
   }


   // Equality requires all fields to be the same unless the field is unknown
   bool ObsID::operator==(const ObsID& right) const
   {
      bool ot = type == otAny || right.type == otAny || type == right.type;
      bool cb = band == cbAny || right.band == cbAny || band == right.band;
      bool tc = code == tcAny || right.code == tcAny || code == right.code;
      return ot && cb && tc;
   }


   // This ordering is somewhat arbitrary but is required to be able
   // to use an ObsID as an index to a std::map. If an application needs
   // some other ordering, inherit and override this function.
   bool ObsID::operator<(const ObsID& right) const
   {
      if (band == right.band)
         if (code == right.code)
            return type < right.type;
         else
            return code < right.code;
      else
         return band < right.band;

      // This should never be reached...
      return false;
   }


   namespace StringUtils
   {
      // convert this object to a string representation
      std::string asString(const ObsID& p)
      {
         std::ostringstream oss;
         p.dump(oss);
         return oss.str();
      }
   }


   // stream output for ObsID
   std::ostream& operator<<(std::ostream& s, const ObsID& p)
   {
      p.dump(s);
      return s;
   }

}
