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
//  Copyright 2013, The University of Texas at Austin
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
 * @file CNavDataElement.cpp
 */

#include "CNavDataElement.hpp"
#include "StringUtils.hpp"
#include "YDSTime.hpp"
#include "CivilTime.hpp"
#include "TimeSystem.hpp"
#include "TimeString.hpp"
#include "SVNumXRef.hpp"

namespace gpstk
{
   using namespace std;
   using namespace gpstk;

   CNavDataElement::CNavDataElement()
      :dataLoadedFlag(false),
       ctEpoch(CommonTime::BEGINNING_OF_TIME),
       ctXmit(CommonTime::BEGINNING_OF_TIME)
   {
      ctEpoch.setTimeSystem(TimeSystem::GPS);
      ctXmit.setTimeSystem(TimeSystem::GPS);
      dataLoadedFlag = false;
   }

   bool CNavDataElement::dataLoaded() const
   {
      return(dataLoadedFlag);
   }

   void CNavDataElement::shortcut(ostream & os, const long HOW )
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


   void CNavDataElement::timeDisplay( ostream & os, const CommonTime& t )
   {
      os.setf(ios::dec, ios::basefield);
         // Convert to CommonTime struct from GPS wk,SOW to M/D/Y, H:M:S.
      GPSWeekSecond dummyTime;
      dummyTime = GPSWeekSecond(t);
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
      os << printTime(t,"   %3j   %5.0s   %02m/%02d/%04Y   %02H:%02M:%02S");
   }

   void CNavDataElement::dump(ostream& s) const
      throw( InvalidRequest )
   {
      dumpHeader(s);
      dumpBody(s);
      dumpFooter(s);
   }

   void CNavDataElement::dumpHeader(ostream& s) const
      throw( InvalidRequest )
   {
      s << "****************************************************************"
        << "************" << endl
        << "Broadcast Data (Engineering Units) - " << getNameLong();
      s << endl;

      SVNumXRef svNumXRef;
      int NAVSTARNum = 0;

      s << endl;
      s << "PRN : " << setw(2) << satID.id << " / "
        << "SVN : " << setw(2);
      try
      {
         NAVSTARNum = svNumXRef.getNAVSTAR(satID.id, ctXmit );
         s << NAVSTARNum << "  ";
      }
      catch(NoNAVSTARNumberFound)
      {
         s << "XX";
      }
      s << endl
        << endl;
      ios::fmtflags oldFlags = s.flags();
#pragma unused(oldFlags)
       
      s.setf(ios::fixed, ios::floatfield);
      s.setf(ios::right, ios::adjustfield);
      s.setf(ios::uppercase);
      s.precision(0);
      s.fill(' ');

      s << endl;
      s << "           TIMES OF INTEREST"
        << endl << endl;
      s << "              Week(10bt)     SOW     DOW   UTD     SOD"
        << "   MM/DD/YYYY   HH:MM:SS\n";
      s << "Transmit Time:";
      timeDisplay(s, ctXmit);
      s << endl;

         // Special case for those data elements that do not possess an
         // epoch time.
      if (ctEpoch>CommonTime::BEGINNING_OF_TIME)
      {
         s << "Epoch Time:   ";
         timeDisplay(s, ctEpoch);
         s << endl;
      }
   }

   void CNavDataElement::dumpBody(ostream& s) const
      throw( InvalidRequest )
   {}

   void CNavDataElement::dumpFooter(ostream& s) const
      throw( InvalidRequest )
   {}

}  // End namespace gpstk

