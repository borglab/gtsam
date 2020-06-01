/// @file Rinex3NavHeader.hpp
/// Encapsulate header of RINEX 3 navigation file, including RINEX 2
/// compatibility.

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

#ifndef GPSTK_RINEX3NAVHEADER_HPP
#define GPSTK_RINEX3NAVHEADER_HPP

#include <iostream>
#include <string>
#include <vector>
#include <map>

#include "StringUtils.hpp"
#include "FFStream.hpp"
#include "Rinex3NavBase.hpp"
#include "RinexSatID.hpp"
#include "TimeSystemCorr.hpp"

namespace gpstk
{

   /** @addtogroup Rinex3Nav */
   //@{

   /// Ionospheric Corrections
class IonoCorr
{
public:
      /// Supported ionospheric correction types
   enum CorrType
   {
      GAL,     ///< Galileo
      GPSA,    ///< GPS alpha
      GPSB     ///< GPS beta
   };

      // Member data
   CorrType type;       ///< type of correction - enum CorrType
   double param[4];     ///< parameters ai0-ai2,0(GAL), alpha0-3 or beta0-3(GPS)

      /// Constructor
   IonoCorr() { }

      /// Constructor from string
   IonoCorr(std::string str) { this->fromString(str); }

      /// Return string version of CorrType
   std::string asString() const throw()
   {
      switch(type) {
         case GAL: return std::string("GAL"); break;
         case GPSA: return std::string("GPSA"); break;
         case GPSB: return std::string("GPSB"); break;
      }
      return std::string("ERROR");
   }

   void fromString(const std::string str) throw(Exception)
   {
      std::string STR(gpstk::StringUtils::upperCase(str));
           if(STR == std::string("GAL")) type = GAL;
      else if(STR == std::string("GPSA")) type = GPSA;
      else if(STR == std::string("GPSB")) type = GPSB;
      else {
         Exception e("Unknown IonoCorr type: " + str);
         GPSTK_THROW(e);
      }
   }

      /// Equal operator
   inline bool operator==(const IonoCorr& ic)
   { return ic.type == type; }

      /// Less than operator - required for map.find()
   inline bool operator<(const IonoCorr& ic)
   { return ic.type < type; }

}; // End of class 'IonoCorr'

   /// This class models the RINEX 3 Nav header for a RINEX 3 Nav file.
   /// \sa Rinex3NavData and Rinex3NavStream classes.
class Rinex3NavHeader : public Rinex3NavBase
{
   public:

      //// Public member functions

      /// Constructor
   Rinex3NavHeader(void)
      : valid(0), version(3.02)
   {}

      /// Destructor
   virtual ~Rinex3NavHeader()
   {}

      /// Rinex3NavHeader is a "header" so this function always returns true.
   virtual bool isHeader(void) const
   { return true; }

      /// This function dumps the contents of the header.
   virtual void dump(std::ostream& s) const;

      /// Change the file system, keeping fileType, fileSys, and fileSysSat
      /// consistent.
      /// @param string str beginning with system character or "M" for mixed
   void setFileSystem(const std::string& str) throw(Exception)
   {
      try {
         if(str[0] == 'M' || str[0] == 'm') {
            if(version < 3) {
               Exception e("RINEX version 2 'Mixed' Nav files do not exist");
               GPSTK_THROW(e);
            }
            fileType = "NAVIGATION";
            fileSys = "MIXED";
            fileSysSat = SatID(-1, SatID::systemMixed);
         }
         else {
            RinexSatID sat(std::string(1,str[0]));
            fileSysSat = SatID(sat);
            fileSys = StringUtils::asString(sat.systemChar())
                           + ": (" + sat.systemString3()+")";
            if(version >= 3) {
               fileType = "NAVIGATION";
            }
            else {            // RINEX 2
               if(sat.system == SatID::systemGPS)
                  fileType = "N (GPS Nav)";
               else if(sat.system == SatID::systemGlonass)
                  fileType = "G (GLO Nav)";
               else if(sat.system == SatID::systemGeosync)
                  fileType = "H (GEO Nav)";
               else {
                  Exception e( std::string("RINEX version 2 ") +
                               sat.systemString3() +
                               std::string(" Nav files do not exist") );
                  GPSTK_THROW(e);
               }
            }
         }
      }
      catch(Exception& e) { GPSTK_RETHROW(e); }
   }

      //// Member data
      /// All 'valid..' bits found in this header
   unsigned long valid;

      /// These are validity bits used in checking the RINEX NAV header.
   enum validBits
   {
      validVersion     = 0x01,   ///< Set if RINEX version is valid.
      validRunBy       = 0x02,   ///< Set if Run-by value is valid.
      validComment     = 0x04,   ///< Set if Comments are valid
      validIonoCorrGPS = 0x08,   ///< Set if GPS Iono Correction data is valid.
      validIonoCorrGal = 0x010,  ///< Set if Gal Iono Correction data is valid.
      validTimeSysCorr = 0x020,  ///< Set if Time System Correction is valid.
      validLeapSeconds = 0x040,  ///< Set if the Leap Seconds value is valid.
      validEoH         = 0x080000000,  ///< Set if the End of Header is valid.

         /// This bitset checks that all required header items are available
         /// for a Rinex (2 or 3) version file - only Version, RunBy, EOH
         /// are required.
      allValid3 = 0x080000003,
         // the only changes 3->3.01 in optional rec. (Leap) allValid301 =
         // 0x080000003,
      allValid2 = 0x080000003
   };



      /** @name HeaderValues */
      //@{

   double version;                 ///< RINEX Version
   std::string fileType;           ///< File type "N...."
   std::string fileSys;            ///< File system string
   SatID fileSysSat;               ///< File system as a SatID
   std::string fileProgram;        ///< Program string
   std::string fileAgency;         ///< Agency string
   std::string date;               ///< Date string; includes "UTC" at the end
   std::vector<std::string> commentList;  ///< Comment list
      /// map of label: GAUT, GPUT, etc, and TimeCorr
   std::map<std::string,TimeSystemCorrection> mapTimeCorr;
      /// map of label : GAL, GPSA or GPSB, and IONO CORRs
   std::map<std::string,IonoCorr> mapIonoCorr;
   long leapSeconds;               ///< Leap seconds
   long leapDelta;                 ///< Change in Leap seconds at ref time
   long leapWeek;                  ///< Week number of ref time
   long leapDay;                   ///< Day of week of ref time

      //@}


      /** @name HeaderStrings */
      //@{

   static const std::string stringVersion;      // "RINEX VERSION / TYPE"
   static const std::string stringRunBy;        // "PGM / RUN BY / DATE"
   static const std::string stringComment;      // "COMMENT"
   static const std::string stringIonoCorr;     // "IONOSPHERIC CORR"
   static const std::string stringTimeSysCorr;  // "TIME SYSTEM CORR"
   static const std::string stringLeapSeconds;  // "LEAP SECONDS"
   static const std::string stringDeltaUTC;     // "DELTA-UTC: A0,A1,T,W" // R2.11 GPS
   static const std::string stringCorrSysTime;  // "CORR TO SYSTEM TIME"  // R2.10 GLO
   static const std::string stringDUTC;         // "D-UTC A0,A1,T,W,S,U"  // R2.11 GEO
   static const std::string stringIonAlpha;     // "ION ALPHA"            // R2.11
   static const std::string stringIonBeta;      // "ION BETA"             // R2.11
   static const std::string stringEoH;          // "END OF HEADER"

      //@}


   protected:


      //// Protected member functions
      /// Write this header to stream \a s.
   virtual void reallyPutRecord(FFStream& s) const
      throw( std::exception,
             FFStreamError,
             gpstk::StringUtils::StringException );


      /// This function reads the RINEX Nav header from the given FFStream.
      /// If an error is encountered in reading from the stream, the stream
      /// is reset to its original position and its fail-bit is set.
      /// @throws StringException when a StringUtils function fails
      /// @throws FFStreamError when exceptions(failbit) is set and a read
      ///         or formatting error occurs.  This also resets the stream
      ///         to its pre-read position.
   virtual void reallyGetRecord(FFStream& s)
      throw( std::exception,
             FFStreamError,
             gpstk::StringUtils::StringException );

}; // End of class 'Rinex3NavHeader'

   //@}

}  // End of namespace gpstk

#endif   // GPSTK_RINEX3NAVHEADER_HPP
