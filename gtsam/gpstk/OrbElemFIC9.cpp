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
 * @file OrbElemFIC9.cpp
 * OrbElemFIC9 data encapsulated in engineering terms
 */
#include <iomanip>
#include <cmath>

#include "OrbElemFIC9.hpp"
#include "StringUtils.hpp"
#include "GNSSconstants.hpp"
#include "GPS_URA.hpp"
#include "GPSWeekSecond.hpp"
#include "SVNumXRef.hpp"
#include "TimeString.hpp"


namespace gpstk
{
   using namespace std;
  

   OrbElemFIC9::OrbElemFIC9()
      :OrbElemLNav()
   {
      ASalert[0] = ASalert[1] = ASalert[2]  =
	   codeflags = health = L2Pdata = 0;

      HOWtime[0] = HOWtime[1] = HOWtime[2] = 0;

      IODC = IODE = 0;
      Tgd = 0.0;
      fitint = 0;
   }

   OrbElemFIC9::OrbElemFIC9( const FICData& fic9 )
      throw( InvalidParameter )
   {
      loadData( fic9 );
   }

   OrbElemFIC9* OrbElemFIC9::clone() const
   {
      return new OrbElemFIC9 (*this); 
   }


   void OrbElemFIC9::loadData( const FICData& fic9 )
      throw( InvalidParameter )
   {
      if (fic9.blockNum!=9)
      {
         InvalidParameter exc("Invalid FIC Block: "+StringUtils::asString(fic9.blockNum));
         GPSTK_THROW(exc);
      }

      // Fill in the variables unique to OrbElemFIC9
      HOWtime[0] = static_cast<long>( fic9.f[2] );
      ASalert[0] = static_cast<short>( fic9.f[3] );
      codeflags  = static_cast<short>( fic9.f[6] );
      accFlag    = static_cast<short>( fic9.f[7] ); 
      health     = static_cast<short>( fic9.f[8] );
      IODC       = static_cast<short>( ldexp( fic9.f[9], -11 ) );
      L2Pdata    = static_cast<short>( fic9.f[10] );
      Tgd        = fic9.f[11];

      HOWtime[1]     = static_cast<long>( fic9.f[22] );
      ASalert[1]     = static_cast<short>( fic9.f[23] );
      IODE           = static_cast<short>( ldexp( fic9.f[25], -11 ) );
      fitint         = static_cast<short>( fic9.f[34] );

      HOWtime[2]       = static_cast<long>( fic9.f[42] );
      ASalert[2]       = static_cast<short>( fic9.f[43] );

      short fullXmitWeekNum    = static_cast<short>( fic9.f[5] );

         // Fill in the variables in the OrbElem parent
	 // - - - First the simple copies - - -
      double Toc     = fic9.f[12];       // OrbElem only stores fully qualified times
                                         // see notes below.
      af2            = fic9.f[13];
      af1            = fic9.f[14];
      af0            = fic9.f[15];

      Crs            = fic9.f[26];
      dn             = fic9.f[27];
      M0             = fic9.f[28];
      Cuc            = fic9.f[29];
      ecc            = fic9.f[30];
      Cus            = fic9.f[31];
      double AHalf   = fic9.f[32];       // Not a member of OrbElem.  See notes below.
      double Toe     = fic9.f[33];       // OrbElem only stores fully qualified times
                                         // see notes below.

      Cic            = fic9.f[45];
      OMEGA0         = fic9.f[46];
      Cis            = fic9.f[47];
      i0             = fic9.f[48];
      Crc            = fic9.f[49];
      w              = fic9.f[50];
      OMEGAdot       = fic9.f[51];
      idot           = fic9.f[53];

      // - - - Now work on the things that need to be calculated - - -

	 // The system is assumed (legacy navigation message is from GPS)
      satID.id = static_cast<short>( fic9.f[19] );

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
	 //   1.) If Toc is NOT even two hour interval, pick lowest HOW time,
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
      if ( (longToc % 7200) != 0)     // NOT an even two hour change
      {
         long leastHOW = HOWtime[0];
         if (HOWtime[1]<leastHOW) leastHOW = HOWtime[1];
         if (HOWtime[2]<leastHOW) leastHOW = HOWtime[2];	 
         long Xmit = leastHOW - (leastHOW % 30);
	 XmitSOW = (double) Xmit;
      }
      else
      {
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
         // Legacy nav doesn't have Rate of Change to Correction to mean motion, 
	 // so set it to zero.
      dndot = 0.0;   

         // Health
         // OrbElemFIC9 stores the full 8 bits health from the legacy
	 // navigation message.  OrElemn only stores the true/false, 
	 // use/don't use based on whether the 8 bit health is 0 or non-zero
      healthy = (healthy==0);


         // URA Handling
      

         // After all this is done, declare that data has been loaded
	 // into this object (so it may be used). 
      dataLoadedFlag = true; 

      return;
   }
  
} // namespace


