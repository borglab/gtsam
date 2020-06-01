/// @file QZSEphemeris.cpp Encapsulates the QZSS broadcast ephemeris and clock.
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
#include "QZSWeekSecond.hpp"

#include "QZSEphemeris.hpp"

using namespace std;

namespace gpstk
{
   // Returns true if the time, ct, is within the period of validity of
   // this OrbitEph object.
   // @throw Invalid Request if the required data has not been stored.
   bool QZSEphemeris::isValid(const CommonTime& ct) const
   {
      try {
         if(ct >= beginValid && ct <= endValid) return true;
         return false;
      }
      catch(Exception& e) { GPSTK_RETHROW(e); }
   }

   // This function returns the health status of the SV.
   bool QZSEphemeris::isHealthy(void) const
   {
      try {
         OrbitEph::isHealthy();     // ignore the return value; for dataLoaded check

         // health is a bit map (0==good), 5 bits (MSB to LSB) applying to signals:
         //  (L1C/A)(L2C)(L5)(L1C)(LEX). Cf IS-QZSS 5.2.2.2.3 and Table 5.1.2-1 pg 50
         // Thus health == 1 means all are healthy except LEX
         //if((health & 0x1)==0) return true;    // LEX is healthy
         //if((health & 0x2)==0) return true;    // L1C is healthy
         //if((health & 0x4)==0) return true;    // L5 is healthy
         //if((health & 0x8)==0) return true;    // L2C is healthy
         //if((health & 0x10)==0) return true;   // L1C/A is healthy

         if((health & 0x1E)==0) return true;   // all but LEX
         return false;
      }
      catch(Exception& e) { GPSTK_RETHROW(e); }
   }

   // Determine the health by signal, where
   //    which = 5 4 3 2 1 as signal = L1C/A L2C L5 L1C LEX.
   // Cf IS-QZSS 5.2.2.2.3 and Table 5.1.2-1 pg 50
   bool QZSEphemeris::isHealthy(const int which) const
   {
      try {
         OrbitEph::isHealthy();     // ignore the return value; for dataLoaded check

         // health is a bit map (0==good), 5 bits (MSB to LSB) applying to signals:
         //  (L1C/A)(L2C)(L5)(L1C)(LEX). Cf IS-QZSS 5.2.2.2.3 and Table 5.1.2-1 pg 50
         // Thus health == 1 means all are healthy except LEX
         switch(which) {
            case 5:     // L1C/A
               if((health & 0x10)==0) return true;
               break;
            case 4:     // L2C
               if((health & 0x08)==0) return true;
               break;
            case 3:     // L5
               if((health & 0x04)==0) return true;
               break;
            case 2:     // L1C
               if((health & 0x02)==0) return true;
               break;
            case 1:     // LEX
               if((health & 0x01)==0) return true;
               break;
            default:
               break;
         }
         return false;
      }
      catch(Exception& e) { GPSTK_RETHROW(e); }
   }

   // adjustBeginningValidity determines the beginValid and endValid times.
   // @throw Invalid Request if the required data has not been stored.
   void QZSEphemeris::adjustValidity(void)
   {
      try {
         OrbitEph::adjustValidity();   // for dataLoaded check
         beginValid = ctToe - fitDuration*1800.0;     // hours*3600/2
         endValid = ctToe + fitDuration*1800.0;
      }
      catch(Exception& e) { GPSTK_RETHROW(e); }
   }
      
   // Dump the orbit, etc information to the given output stream.
   // @throw Invalid Request if the required data has not been stored.
   void QZSEphemeris::dumpBody(std::ostream& os) const
   {
      try {
         OrbitEph::dumpBody(os);

         os << "           QZSS-SPECIFIC PARAMETERS\n"
            << scientific << setprecision(8)
            << "Tgd (L1/L2) : " << setw(16) << Tgd << " meters" << endl
            << "HOW time    : " << setw(6) << HOWtime << " (sec of QZS week "
               << setw(4) << static_cast<QZSWeekSecond>(ctToe).getWeek() << ")"
            << "   fitDuration: " << setw(2) << fitDuration << " hours" << endl
            << "TransmitTime: " << OrbitEph::timeDisplay(transmitTime) << endl
            << "Accuracy    : " << fixed << setprecision(2)
            << getAccuracy() << " meters" << endl
            << "IODC: " << IODC << "   IODE: " << IODE << "   health: " << health
            << "   codeflags: " << codeflags << "   L2Pdata: " << L2Pdata
            << endl;
      }
      catch(Exception& e) { GPSTK_RETHROW(e); }
   }

} // end namespace
