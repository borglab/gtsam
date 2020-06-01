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
 * @file OrbElemLNav.cpp
 * OrbElemLNAV data encapsulated in engineering terms
 */
#include <iomanip>
#include <cmath>

#include "OrbElemLNav.hpp"
#include "StringUtils.hpp"
#include "GNSSconstants.hpp"
#include "GPS_URA.hpp"
#include "GPSWeekSecond.hpp"
#include "SVNumXRef.hpp"
#include "TimeString.hpp"
#include "EngNav.hpp"

namespace gpstk
{
   using namespace std;
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wreorder"
   OrbElemLNav::OrbElemLNav()
      :OrbElem(),
       codeflags(0), health(0), L2Pdata(0),
       accFlag(0), IODC(0), IODE(0),
       Tgd(0.0), fitint(0),
       AODO(0)
   {}
#pragma clang diagnostic pop
   OrbElemLNav::OrbElemLNav(  const long  SF1[10],
                              const long  SF2[10],
                              const long  SF3[10],
                              const SatID satIDArg,
                              const short XmitGPSWeek )
      throw(InvalidParameter)
   {
      loadData(SF1, SF3, SF3, satIDArg, XmitGPSWeek);
   }

   OrbElemLNav* OrbElemLNav::clone() const
   {
      return new OrbElemLNav (*this);
   }

   void OrbElemLNav::loadData(  const long  SF1[10],
                                const long  SF2[10],
                                const long  SF3[10],
                                const SatID satIDArg,
                                const short XmitGPSWeek )
      throw( InvalidParameter )
   {
      double ficOut[60];
      double ficTemp[60];
      EngNav engNavDummy; // Need this to initialize static data
      if(!EngNav::subframeConvert(SF1, XmitGPSWeek, ficTemp))
      {
         InvalidParameter exc("Invalid SF1 Data");
         GPSTK_THROW(exc);
      }
      for(int i = 0; i < 20; i++)
      {
         ficOut[i] = ficTemp[i];
      }
      if(!EngNav::subframeConvert(SF2, XmitGPSWeek, ficTemp))
      {
         InvalidParameter exc("Invalid SF2 Data");
         GPSTK_THROW(exc);
      }
      for(int i = 0; i < 20; i++)
      {
         ficOut[i+20] = ficTemp[i];
      }
      if(!EngNav::subframeConvert(SF3, XmitGPSWeek, ficTemp))
      {
         InvalidParameter exc("Invalid SF3 Data");
         GPSTK_THROW(exc);
      }
      for(int i = 0; i < 20; i++)
      {
         ficOut[i+40] = ficTemp[i];
      }
      //ficOut[19] = PRNID;

      // Fill in the variables unique to OrbElemLNav
      HOWtime[0] = static_cast<long>( ficOut[2] );
      ASalert[0] = static_cast<short>( ficOut[3] );
      codeflags  = static_cast<short>( ficOut[6] );
      accFlag    = static_cast<short>( ficOut[7] );
      health     = static_cast<short>( ficOut[8] );
      IODC       = static_cast<short>( ldexp( ficOut[9], -11 ) );
      L2Pdata    = static_cast<short>( ficOut[10] );
      Tgd        = ficOut[11];

      HOWtime[1]     = static_cast<long>( ficOut[22] );
      ASalert[1]     = static_cast<short>( ficOut[23] );
      IODE           = static_cast<short>( ldexp( ficOut[25], -11 ) );
      fitint         = static_cast<short>( ficOut[34] );

      HOWtime[2]       = static_cast<long>( ficOut[42] );
      ASalert[2]       = static_cast<short>( ficOut[43] );
      AODO             = static_cast<long>( ficOut[35] );

      short fullXmitWeekNum    = static_cast<short>( ficOut[5] );

         // Fill in the variables in the OrbElem parent
	 // - - - First the simple copies - - -
      double Toc     = ficOut[12];       // OrbElem only stores fully qualified times
                                         // see notes below.
      af2            = ficOut[13];
      af1            = ficOut[14];
      af0            = ficOut[15];

      Crs            = ficOut[26];
      dn             = ficOut[27];
      M0             = ficOut[28];
      Cuc            = ficOut[29];
      ecc            = ficOut[30];
      Cus            = ficOut[31];
      double AHalf   = ficOut[32];       // Not a member of OrbElem.  See notes below.
      double Toe     = ficOut[33];       // OrbElem only stores fully qualified times
                                         // see notes below.

      Cic            = ficOut[45];
      OMEGA0         = ficOut[46];
      Cis            = ficOut[47];
      i0             = ficOut[48];
      Crc            = ficOut[49];
      w              = ficOut[50];
      OMEGAdot       = ficOut[51];
      idot           = ficOut[53];

         // Store the satellite ID
      satID = satIDArg;

      // - - - Now work on the things that need to be calculated - - -
         // The observation ID has a type of navigation, but the
         // carrier and code types are undefined.  They could be
         // L1/L2 C/A, P, Y,.....
      obsID.type = ObsID::otNavMsg;
      obsID.band = ObsID::cbUndefined;
      obsID.code = ObsID::tcUndefined;

	 // Beginning of Validity
         // New concept.  Admit the following.
	 //  (a.) The collection system may not capture the data at earliest transmit.
	 //  (b.) The collection system may not capture the three SFs consecutively.
	 // Consider a couple of IS-GPS-200 promises,
	 //  (c.) By definition, beginning of validity == beginning of transmission.
	 //  (d.) Except for uploads, cutovers will only happen on hour boundaries
	 //  (e.) Cutovers can be detected by non-even Toc.
	 //  (f.) Even uploads will cutover on a frame (30s) boundary.
         // Therefore,
	 //   1.) If Toc is NOT even hour interval, pick lowest HOW time,
	 //   round back to even 30s.  That's the earliest Xmit time we can prove.
	 //   NOTE: For the case where this is the SECOND SF 1/2/3 after an upload,
	 //   this may yield a later time as such a SF 1/2/3 will be on a even
	 //   hour boundary.  Unfortunately, we have no way of knowing whether
	 //   this item is first or second after upload without additional information.
	 //   2.) If Toc IS even two hour interval, pick time from SF 1,
	 //   round back to nearest EVEN two hour boundary.  This assumes collection
	 //   SOMETIME in first hour of transmission.  Could be more
	 //   complete by looking at fit interval and IODC to more accurately
	 //   determine earliest transmission time.
      long longToc = (long) Toc;
      double XmitSOW = 0.0;
      if ( (longToc % 3600) != 0)     // NOT an even two hour change
      {
         long leastHOW = HOWtime[0];
         if (HOWtime[1]<leastHOW) leastHOW = HOWtime[1];
         if (HOWtime[2]<leastHOW) leastHOW = HOWtime[2];
         long Xmit = leastHOW - (leastHOW % 30);
         XmitSOW = (double) Xmit;
      }
      else
      {
            // This assumes the data are collected sometime in the
            // first two hours of transmission.  For extended ops
            // that might not be true.  Think about this.
         long Xmit = HOWtime[0] - HOWtime[0] % 7200;
         XmitSOW = (double) Xmit;
      }
      beginValid = GPSWeekSecond( fullXmitWeekNum, XmitSOW, TimeSystem::GPS );

      // Determine Transmit Time
      // Transmit time is the actual time this
      // SF 1/2/3 sample was collected
      long leastHOW = HOWtime[0];
      if (HOWtime[1]<leastHOW) leastHOW = HOWtime[1];
      if (HOWtime[2]<leastHOW) leastHOW = HOWtime[2];
      long Xmit = leastHOW - (leastHOW % 30);
      transmitTime = GPSWeekSecond( fullXmitWeekNum, (double)Xmit, TimeSystem::GPS );

         // Fully qualified Toe and Toc
	 // As broadcast, Toe and Toc are in GPS SOW and do not include
	 // the GPS week number.  OrbElem (rightly) insists on having a
	 // Toe and Toc in CommonTime objects which implies determining
	 // the week number.
      double timeDiff = Toe - XmitSOW;
      short epochWeek = fullXmitWeekNum;
      if (timeDiff < -HALFWEEK) epochWeek++;
      else if (timeDiff > HALFWEEK) epochWeek--;

      ctToc = GPSWeekSecond(epochWeek, Toc, TimeSystem::GPS);
      ctToe = GPSWeekSecond(epochWeek, Toe, TimeSystem::GPS);

	 // End of Validity.
	 // The end of validity is calculated from the fit interval
	 // and the Toe.  The fit interval is either trivial
	 // (if fit interval flag==0, fit interval is 4 hours)
	 // or a look-up table based on the IODC.
	 // Round the Toe value to the hour to elminate confusion
	 // due to possible "small offsets" indicating uploads
      short fitHours = getLegacyFitInterval(IODC, fitint);
      long  ToeOffset = (long) Toe % 3600;
      double adjToe = Toe;                  // Default case
      if (ToeOffset)
      {
         adjToe += 3600.0 - (double)ToeOffset;   // If offset, then adjust to remove it
      }
      long endFitSOW = adjToe + (fitHours/2)*3600;
      short endFitWk = epochWeek;
      if (endFitSOW >= FULLWEEK)
      {
         endFitSOW -= FULLWEEK;
         endFitWk++;
      }
      endValid = GPSWeekSecond(endFitWk, endFitSOW, TimeSystem::GPS);

	 // Semi-major axis and time-rate-of-change of semi-major axis
	 //    Note: Legacy navigation message (SF 1/2/3) used SQRT(A).
	 //    The CNAV and CNAV-2 formats use deltaA and Adot.  As a
	 //    result, OrbElem uses A and Adot and SQRT(A) and deltaA
	 //    are converted to A at runtime.
      A = AHalf * AHalf;
      Adot = 0.0;
         // Legacy nav doesn't have Rate of Chagen to corerction to mean motion,
	 // so set it to zero.
      dndot = 0.0;

         // Health
         // OrbElemLNav stores the full 8 bits health from the legacy
         // navigation message.  OrElem only stores the true/false,
         // use/don't use based on whether the 8 bit health is 0 or non-zero
      healthy = (health==0);

         // After all this is done, declare that data has been loaded
	 // into this object (so it may be used).
      dataLoadedFlag = true;

      return;
   }

   double OrbElemLNav::getAccuracy()  const
      throw(InvalidRequest)
   {
      if (!dataLoaded())
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }
      double accuracy = ura2accuracy( accFlag );
      return accuracy;
   }

   void OrbElemLNav::adjustBeginningValidity()
   {
      if (!dataLoaded()) return;

         // The nominal beginning of validity is calculated from
         // the fit interval and the Toe.  The fit interval is
         //  either trivial (if fit interval flag==0, fit interval is 4 hours)
         // or a look-up table based on the IODC.
      short fitHours = getLegacyFitInterval(IODC, fitint);
      long  oneHalfInterval = ((long)fitHours/2) * 3600;

         // If we assume this is the SECOND set of elements in a set
         // (which is an assumption of this function - see the .hpp) then
         // the "small offset in Toe" will actually push the Toe-oneHalfInterval
         // too early. For example, consider the following case.
         //         Toe : 19:59:44  (really near 20:00:00)
         //  first xMit : 18:00:00  (nominal)
         // Blindly setting beginValid top Toe - 1/2 fit interval will
         // result in 17:59:44.  But 18:00:00 actually is the right answer
         // because the -16 second offset is an artifact.
         //
         // Therefore, we are FIRST going to remove that offset,
         // THEN determine beginValid.
      long sow = (long) (static_cast<GPSWeekSecond>(ctToe)).sow;
      short week = (static_cast<GPSWeekSecond>(ctToe)).week;
      sow = sow + (3600 - (sow%3600));
      CommonTime adjustedToe = GPSWeekSecond(week, (double) sow);
      adjustedToe.setTimeSystem(TimeSystem::GPS);

      beginValid = adjustedToe - oneHalfInterval;
      return;
   }

   void OrbElemLNav::dumpHeader(ostream& s) const
      throw( InvalidRequest )
   {
      if (!dataLoaded())
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }

      ios::fmtflags oldFlags = s.flags();

      OrbElem::dumpHeader(s);

      s  << "           SUBFRAME OVERHEAD"
      	<< endl
      	<< endl
      	<< "               SOW    DOW:HH:MM:SS     IOD    ALERT   A-S\n";
      for (int i=0;i<3;i++)
     	 {
            s << "SF" << setw(1) << (i+1)
              << " HOW:   " << setw(7) << HOWtime[i]
              << "  ";

       	    shortcut( s, HOWtime[i]);
            if (i==0)
        	s << "   ";
            else
        	s << "    ";

            s << "0x" << setfill('0') << hex;

            if (i==0)
          	s << setw(3) << IODC;
            else
        	s << setw(2) << IODE;

            s << dec << "      " << setfill(' ');

            if (ASalert[i] & 0x0002)    // "Alert" bit handling
        	s << "1     ";
            else
        	s << "0     ";

            if (ASalert[i] & 0x0001)     // A-S flag handling
        	s << " on";
            else
        	s << "off";
            s << endl;
         }

      s << endl
        << "           SV STATUS"
        << endl
        << endl
        << "Health bits         :      0x" << setfill('0') << hex << setw(2)
        << health << dec << ", " <<health;
      s << endl
        << "Fit interval flag   :         " << setw(1) << fitint;
      s << endl
        << "URA index           :      " << setfill(' ')
        << setw(4) << accFlag << endl
        << "Code on L2          :   ";

      switch (codeflags)
      {
         case 0:
            s << "reserved ";
            break;

         case 1:
            s << " P only  ";
            break;

         case 2:
            s << " C/A only";
            break;

         case 3:
            s << " P & C/A ";
            break;

         default:
            break;

      }
      s << endl
        <<"L2 P Nav data       :        ";
      if (L2Pdata!=0)
        s << "off";
      else
         s << "on";

      s.setf(ios::uppercase);
      s << endl;
      s << "Tgd                 : " << setw(13) << setprecision(6) << scientific << Tgd << " sec";
      if(AODO!=0)
          s << endl << "AODO                :     " << setw(5) << fixed << AODO << " sec" << endl;
      s << endl;
      s.flags(oldFlags);
   } // end of dumpHeader()

   void OrbElemLNav :: dumpTerse(ostream& s) const
      throw(InvalidRequest )
   {

       // Check if the subframes have been loaded before attempting
       // to dump them.
      if (!dataLoaded())
      {
         InvalidRequest exc("No data in the object");
         GPSTK_THROW(exc);
      }

      ios::fmtflags oldFlags = s.flags();

      s.setf(ios::fixed, ios::floatfield);
      s.setf(ios::right, ios::adjustfield);
      s.setf(ios::uppercase);
      s.precision(0);
      s.fill(' ');

      SVNumXRef svNumXRef;
      int NAVSTARNum = 0;
      try
      {
         NAVSTARNum = svNumXRef.getNAVSTAR(satID.id, ctToe );
         s << setw(2) << " " << NAVSTARNum << "  ";
      }
      catch(NoNAVSTARNumberFound)
      {
         s << "  XX  ";
      }

      s << setw(2) << satID.id << " ! ";

      string tform = "%3j %02H:%02M:%02S";

      s << printTime(beginValid, tform) << " ! ";
      s << printTime(ctToe, tform) << " ! ";
      s << printTime(endValid, tform) << " !  ";

      s << setw(4) << setprecision(1) << getAccuracy() << "  ! ";
      s << "0x" << setfill('0') << hex << setw(3) << IODC << " ! ";
      s << "0x" << setfill('0')  << setw(2) << health;
      s << setfill(' ') << dec;
      s << "   " << setw(2) << health << " ! ";

      s << endl;
      s.flags(oldFlags);

   } // end of dumpTerse()

   ostream& operator<<(ostream& s, const OrbElemLNav& eph)
   {
      try
      {
         eph.dump(s);
      }
      catch(gpstk::Exception& ex)
      {
         GPSTK_RETHROW(ex);
      }
      return s;

   } // end of operator<<

} // namespace


