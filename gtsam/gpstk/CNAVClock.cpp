#pragma ident "$Id$"

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

/**
 * @file CNAVClock.cpp
 * Ephemeris data encapsulated in engineering terms
 */
#include <iomanip>
#include <cmath>

#include "StringUtils.hpp"
#include "GNSSconstants.hpp"
#include "MathBase.hpp"
#include "CNAVClock.hpp"
#include "GPS_URA.hpp"

namespace gpstk
{
   using namespace std;

   CNAVClock::CNAVClock()
      throw()
   {
      dataLoaded = false;

      satSys = "";

      PRNID = TOWWeek = TOWCount = Alert = Top = 0;
   
      Toc = 0.0;
   }

   void CNAVClock::loadData( const std::string satSysArg, const ObsID obsIDArg,
                             const short PRNIDArg, const short AlertMsgArg,
                             const long TOWMsgArg, const short TOWWeekArg,
                             const long TopArg, const long TocArg,
	                          const double accuracyArg, const short URAocArg, 
                             const short URAoc1Arg, const short URAoc2Arg, 
                             const double af0Arg, const double af1Arg, 
                             const double af2Arg )
   {
      satSys       = satSysArg;
      obsID        = obsIDArg;
      PRNID        = PRNIDArg;
      Alert        = AlertMsgArg;
      TOWCount     = TOWMsgArg;
      TOWWeek      = TOWWeekArg;
      Top          = TopArg;
      Toc          = TocArg;
      short URAoc  = URAocArg;
      short URAoc1 = URAoc1Arg;
      short URAoc2 = URAoc2Arg;
      bool healthy = false;

      satSys = "G";
      double timeDiff = TocArg - TOWCount;
      short epochWeek = TOWWeek;
      if (timeDiff < -HALFWEEK) epochWeek++;
      else if (timeDiff > HALFWEEK) epochWeek--;

         // BrcClockCorrection takes Toc parameter as a CommonTime variable.
      CommonTime TocCT = GPSWeekSecond(epochWeek, Toc, TimeSystem::GPS);
      CommonTime TopCT = GPSWeekSecond(epochWeek, Top, TimeSystem::GPS);
   
         // The observation ID has a type of navigation. The code type could
         // be L2 or L5.
      ObsID obsID(ObsID::otNavMsg, obsIDArg.band, obsIDArg.code);

      bcClock.loadData( satSys, obsID, PRNID, TocCT, TopCT, URAoc,
                        URAoc1, URAoc2, healthy, af0Arg, af1Arg, af2Arg); 
      dataLoaded  = true;   
   }

   void CNAVClock::loadData( const ObsID obsIDArg, const short PRNIDArg,
                             const short TOWWeekArg, const PackedNavBits message3_)
      throw( InvalidParameter)
   {
      obsID        = obsIDArg;
      PRNID        = PRNIDArg;
      TOWWeek      = TOWWeekArg;
      satSys       = "G";
      Alert        = message3_.asUnsignedLong(37, 1, 1);
      TOWCount     = message3_.asUnsignedLong(20, 17, 300);
      Top          = message3_.asUnsignedLong(38, 11, 300);
      short URAoc  = message3_.asLong(49, 5, 1);
      short URAoc1 = message3_.asUnsignedLong(54, 3, 1);
      short URAoc2 = message3_.asUnsignedLong(57, 3, 1);
      Toc          = message3_.asUnsignedLong(60, 11, 300);
      double af0   = message3_.asSignedDouble(71, 26, -35);
      double af1   = message3_.asSignedDouble(97, 20, -48);
      double af2   = message3_.asSignedDouble(117, 10, -60);

      bool healthy    = false;
      double timeDiff = Toc - TOWCount;
      short epochWeek = TOWWeek;
      if (timeDiff < -HALFWEEK) epochWeek++;
      else if (timeDiff > HALFWEEK) epochWeek--;
   
      CommonTime TocCT = GPSWeekSecond(epochWeek, Toc, TimeSystem::GPS);
      CommonTime TopCT = GPSWeekSecond(epochWeek, Top, TimeSystem::GPS);

      bcClock.loadData( satSys, obsID, PRNID, TocCT, TopCT, URAoc,
                        URAoc1, URAoc2, healthy, af0, af1, af2); 
      dataLoaded = true;
   }

   bool CNAVClock::hasData() 
   {
      return(dataLoaded);
   }
      
   double CNAVClock::svClockBias(const CommonTime& t) const
      throw( InvalidRequest )
   {
      if(!dataLoaded)
      {
         InvalidRequest exc("svClockBias: Required data not stored.");
         GPSTK_THROW(exc);
      }
      return bcClock.svClockBias(t);
   }

   double CNAVClock::svClockDrift(const CommonTime& t) const
      throw( InvalidRequest )
   {
      if(!dataLoaded)
      {
         InvalidRequest exc("svClockDrift(): Required data not stored.");
         GPSTK_THROW(exc);
      }
      return bcClock.svClockDrift(t);
   }

   CommonTime CNAVClock::getClockEpoch() const
      throw(InvalidRequest)
   {
      CommonTime toReturn;
      double timeDiff = Toc - TOWCount;
      short epochWeek = TOWWeek;
      if (timeDiff < -HALFWEEK) epochWeek++;
      else if (timeDiff > HALFWEEK) epochWeek--;
      if (satSys == "G" )
         toReturn = GPSWeekSecond(epochWeek, Toc, TimeSystem::GPS);
      else if (satSys == "E" )
         toReturn = GPSWeekSecond(epochWeek, Toc, TimeSystem::GAL);
      else
      {
         InvalidRequest exc("Invalid Time System in CNAVClock::getClockEpoch()");
         GPSTK_THROW(exc);
      }
      return toReturn;
   }

   CommonTime CNAVClock::getTransmitTime() const
      throw(InvalidRequest)
   {
      if(!dataLoaded)
      {
         InvalidRequest exc("getTransmitTime: Required data not stored.");
         GPSTK_THROW(exc);
      }
      GPSWeekSecond gws(TOWWeek, TOWCount, TimeSystem::GPS);
      return (gws.convertToCommonTime());
   }

   CommonTime CNAVClock::getTimeOfPrediction() const
      throw( InvalidRequest )
   {
      if(!dataLoaded)
      {
         InvalidRequest exc("getTimeOfPrediction(): Required data not stored.");
         GPSTK_THROW(exc);
      }
      short week(TOWWeek);
      if( Top - TOWCount < -HALFWEEK)
      week++;
      else if ( Top - TOWCount > HALFWEEK)
      week--;
      CommonTime toReturn;
      toReturn = GPSWeekSecond(week, Top, TimeSystem::GPS);
      return toReturn;
   }

   short CNAVClock::getPRNID() const
      throw( InvalidRequest )
   {
      if(!dataLoaded)
      {
         InvalidRequest exc("getPRNID(): Required data not stored.");
         GPSTK_THROW(exc);
      }
      return PRNID;
   }

   short CNAVClock::getAlert()  const
      throw( InvalidRequest )
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("getAlert(): messageNum not stored.");
         GPSTK_THROW(exc);
      }
      return Alert;
   }

   double CNAVClock::getAccuracy(const CommonTime& t)  const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return bcClock.getAccuracy(t);
   }  

   short CNAVClock::getURAoc(const short ndx) const
      throw( InvalidRequest, InvalidParameter )
   {
      if(!dataLoaded)
      {
         InvalidRequest exc("getURAoc(): Required data not stored.");
         GPSTK_THROW(exc);
      }
      return bcClock.getURAoc(ndx);
   }

   long CNAVClock::getTop() const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return Top;
   }

   double CNAVClock::getToc() const
      throw(InvalidRequest)
   {
      if (!dataLoaded)
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      return Toc;
   }
  
   BrcClockCorrection CNAVClock::getClock() const
      throw(InvalidRequest )
   {
      if(!bcClock.hasData())
      {
         InvalidRequest exc("getClock(): Required Clock data not stored.");
         GPSTK_THROW(exc);
      }
      return (bcClock);
   }

   static void timeDisplay( ostream & os, const CommonTime& t )
   {
         // Convert to CommonTime struct from GPS wk,SOW to M/D/Y, H:M:S.
      GPSWeekSecond dummyTime;
      dummyTime = GPSWeekSecond(t);
      os << dec;
      os << setw(4) << dummyTime.week << "(";
      os << setw(4) << (dummyTime.week & 0x03FF) << ")  ";
      os << setw(6) << setfill(' ') << dummyTime.sow << "   ";

      switch (dummyTime.getDayOfWeek())
      {
         case 0: os << "Sun-0"; break;
         case 1: os << "Mon-1"; break;
         case 2: os << "Tue-2"; break;
         case 3: os << "Wed-3"; break;
         case 4: os << "Thu-4"; break;
         case 5: os << "Fri-5"; break;
         case 6: os << "Sat-6"; break;
         default: break;
      }
      os << "   " << (static_cast<YDSTime>(t)).printf("%3j   %5.0s  ") 
         << (static_cast<CivilTime>(t)).printf("%02m/%02d/%04Y   %02H:%02M:%02S");
   }

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-function"
   static void shortcut(ostream & os, const long HOW )
   {
      short DOW, hour, min, sec;
      long SOD, SOW;
      short SOH;
      
      SOW = static_cast<long>( HOW );
      DOW = static_cast<short>( SOW / SEC_PER_DAY );
      SOD = SOW - static_cast<long>( DOW * SEC_PER_DAY );
      hour = static_cast<short>( SOD/3600 );

      SOH = static_cast<short>( SOD - (hour*3600) );
      min = SOH/60;

      sec = SOH - min * 60;
      switch (DOW)
      {
         case 0: os << "Sun-0"; break;
         case 1: os << "Mon-1"; break;
         case 2: os << "Tue-2"; break;
         case 3: os << "Wed-3"; break;
         case 4: os << "Thu-4"; break;
         case 5: os << "Fri-5"; break;
         case 6: os << "Sat-6"; break;
         default: break;
      }

      os << ":" << setfill('0')
         << setw(2) << hour
         << ":" << setw(2) << min
         << ":" << setw(2) << sec
         << setfill(' ');
   }
#pragma clang diagnostic pop

   void CNAVClock :: dump(ostream& s) const
      throw()
   { 
      s.setf(ios::fixed, ios::floatfield);
      s.setf(ios::right, ios::adjustfield);
      s.setf(ios::uppercase);
      s.precision(0);
      s.fill(' ');
      
      s << "****************************************************************"
        << "************" << endl
        << "CNAV Message Type 3?" << endl
        << endl
        << "PRN: " << setw(2) << PRNID << "      "
        << "System: " << satSys << "      "
        << "Carrier: " << ObsID::cbDesc[obsID.band] << "      "
        << "Code: " << ObsID::tcDesc[obsID.code] << endl<<endl;  

      s << "                  Week        SOW     DOW   UTD     SOD"
        << "   MM/DD/YYYY   HH:MM:SS\n";
      s << "Clock Epoch:    ";
      timeDisplay(s, getClockEpoch());
      s << endl;
      s << "Transmit Time:  ";
      timeDisplay(s, getTransmitTime());
      s << endl;
      s << "Time of Predict:";
      timeDisplay(s, getTimeOfPrediction());
      s << endl;
     
      s << endl
      << "          ACCURACY PARAMETERS"
      << endl
      << endl
      << "URAoc index:  " <<setw(3) << getURAoc(0) << "    " << setw(3) << getURAoc(1) 
      << "    " << setw(3) << getURAoc(2) << endl;

      s.setf(ios::scientific, ios::floatfield);
      s.precision(11);
      
      s << endl
        << "           CLOCK"
        << endl
        << endl
        << "Bias T0:     " << setw(18) << bcClock.getAf0() << " sec" << endl
        << "Drift:       " << setw(18) << bcClock.getAf1() << " sec/sec" << endl
        << "Drift rate:  " << setw(18) << bcClock.getAf2() << " sec/(sec**2)" << endl;  
      
      s << "****************************************************************"
        << "************" << endl;
      
   } // end of CNAVClock::dump()

   ostream& operator<<(ostream& s, const CNAVClock& eph)
   {
      eph.dump(s);
      return s;
   } // end of operator<<

} // namespace
