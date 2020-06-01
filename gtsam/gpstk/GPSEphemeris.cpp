/// @file GPSEphemeris.cpp Encapsulates the GPS legacy broadcast ephemeris and clock.
/// Inherits OrbitEph, which does most of the work; this class adds health and
/// accuracy information, fit interval, ionospheric correction terms and data
/// flags.

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

#include <string>
#include "Exception.hpp"
#include "SVNumXRef.hpp"
#include "GPSWeekSecond.hpp"
#include "CivilTime.hpp"
#include "TimeString.hpp"

#include "GPSEphemeris.hpp"

using namespace std;

namespace gpstk
{
   // Returns true if the time, ct, is within the period of validity of
   // this OrbitEph object.
   // @throw Invalid Request if the required data has not been stored.
   bool GPSEphemeris::isValid(const CommonTime& ct) const
   {
      try {
         if(ct >= beginValid && ct <= endValid) return true;
         return false;
      }
      catch(Exception& e) { GPSTK_RETHROW(e); }
   }

   // This function returns the health status of the SV.
   bool GPSEphemeris::isHealthy(void) const
   {
      try {
         OrbitEph::isHealthy();     // ignore the return value; for dataLoaded check
         if(health == 0) return true;
         return false;
      }
      catch(Exception& e) { GPSTK_RETHROW(e); }
   }

   // adjustBeginningValidity determines the beginValid and endValid times.
   // @throw Invalid Request if the required data has not been stored.
   void GPSEphemeris::adjustValidity(void)
   {
      try {
         OrbitEph::adjustValidity();   // for dataLoaded check
         beginValid = ctToe - fitDuration*1800.0;     // hours*3600/2
         endValid = ctToe + fitDuration*1800.0;
      }
      catch(Exception& e) { GPSTK_RETHROW(e); }
   }
      
   // Dump the overhead information as a string containing a single line.
   // @throw Invalid Request if the required data has not been stored.
   string GPSEphemeris::asString(void) const
   {
      if(!dataLoadedFlag)
         GPSTK_THROW(InvalidRequest("Data not loaded"));
      try {
         ostringstream os;
         CivilTime ct;
         os << "EPH G" << setfill('0') << setw(2) << satID.id << setfill(' ');
         ct = CivilTime(beginValid);
         os << printTime(ct," | %4Y %3j %02H:%02M:%02S |");
         ct = CivilTime(ctToe);
         os << printTime(ct," %3j %02H:%02M:%02S |");
         ct = CivilTime(ctToc);
         os << printTime(ct," %3j %02H:%02M:%02S |");
         ct = CivilTime(endValid);
         os << printTime(ct," %3j %02H:%02M:%02S |");
         ct = CivilTime(transmitTime);
         os << printTime(ct," %3j %02H:%02M:%02S | ");
         os << setw(3) << IODE << " | " << setw(3) << IODC << " | " << health;
         return os.str();
      }
      catch(Exception& e) { GPSTK_RETHROW(e);
      }
   }

   // Dump the overhead information to the given output stream.
   // @throw Invalid Request if the required data has not been stored.
   void GPSEphemeris::dumpHeader(std::ostream& os) const
   {
      try {
         // copy from OrbitEph::dumpHeader() ...
         if(!dataLoadedFlag)
            GPSTK_THROW(InvalidRequest("Data not loaded"));

         os << "****************************************************************"
            << "************" << endl
            << "Broadcast Orbit Ephemeris of class " << getName() << endl;
         os << "Satellite: " << SatID::convertSatelliteSystemToString(satID.system)
            << " " << setfill('0') << setw(2) << satID.id << setfill(' ');

         // ... and add this for GPS
         os << " SVN ";
         SVNumXRef svNumXRef; 
         try {
            os << svNumXRef.getNAVSTAR(satID.id, ctToe );
         }
         catch(NoNAVSTARNumberFound) {
            os << "Unknown";
         }
         os << endl;
      }
      catch(Exception& e) { GPSTK_RETHROW(e); }
   }

   // Dump the orbit, etc information to the given output stream.
   // @throw Invalid Request if the required data has not been stored.
   void GPSEphemeris::dumpBody(std::ostream& os) const
   {
      try {
         OrbitEph::dumpBody(os);

         os << "           GPS-SPECIFIC PARAMETERS\n"
            << scientific << setprecision(8)
            << "Tgd (L1/L2) : " << setw(16) << Tgd << " meters" << endl
            << "HOW time    : " << setw(6) << HOWtime << " (sec of GPS week "
               << setw(4) << static_cast<GPSWeekSecond>(ctToe).getWeek() << ")"
            << "   fitDuration: " << setw(2) << fitDuration << " hours" << endl
            << "TransmitTime: " << OrbitEph::timeDisplay(transmitTime) << endl
            << "Accuracy    : flag(URA): " << accuracyFlag << " => "
            << fixed << setprecision(2) << getAccuracy() << " meters" << endl
            << "IODC: " << IODC << "   IODE: " << IODE << "   health: " << health
            << " (0=good)   codeflags: " << codeflags << "   L2Pdata: " << L2Pdata
            << endl;
      }
      catch(Exception& e) { GPSTK_RETHROW(e); }
   }

   // Get the fit interval in hours from the fit interval flag and the IODC
   short GPSEphemeris::getFitInterval(const short IODC, const short fitIntFlag)
   {
      // TD This is for Block II/IIA, need to update for Block IIR and IIF

      if(IODC < 0 || IODC > 1023) // error in IODC, return minimum fit
         return 4;

      if((fitIntFlag == 0 && (IODC & 0xFF) < 240) || (IODC & 0xFF) > 255)
         return 4;

      else if(fitIntFlag == 1) {

         if(((IODC & 0xFF) < 240 || (IODC & 0xFF) > 255))
            return 6;
         else if(IODC >=240 && IODC <=247)
            return 8;
         else if((IODC >= 248 && IODC <= 255) || IODC == 496)
            return 14;

         // Revised in IS-GPS-200 Revision D for Block IIR/IIR-M/IIF
         else if((IODC >= 497 && IODC <=503) || (IODC >= 1021 && IODC <= 1023))
            return 26;
         else if(IODC >= 504 && IODC <=510)
            return 50;
         else if(IODC == 511 || (IODC >= 752 && IODC <= 756))
            return 74;

         // NOTE:
         // The following represents old fit intervals for Block II (not IIA)
         // and is present only in old versions of the ICD-GPS-200 Rev. C.
         // Please do not remove them as there are still people that may
         // want to process old Block II data and none of the IODC intervals
         // overlap (so far) so there is no need to remove them.
         else if(IODC >= 757 && IODC <= 763)
            return 98;
         else if((IODC >= 764 && IODC <=767) || (IODC >=1008 && IODC <=1010))
            return 122;
         else if(IODC >= 1011 && IODC <=1020)
            return 146;
         else              // error in the IODC or ephemeris, return minimum fit
            return 4;
      }
      else                 // error in ephemeris/IODC, return minimum fit
         return 4;

      return 0; // never reached
   }

} // end namespace
