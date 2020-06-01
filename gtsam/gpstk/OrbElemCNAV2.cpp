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
 * @file OrbElemCNAV2.cpp
 * OrbElemCNAV2 data encapsulated in engineering terms
 */
#include <iomanip>
#include <cmath>

#include "StringUtils.hpp"
#include "GNSSconstants.hpp"
#include "GPSWeekSecond.hpp"
#include "SVNumXRef.hpp"
#include "TimeString.hpp"
#include "OrbElemCNAV2.hpp"

namespace gpstk
{
   using namespace std;

   OrbElemCNAV2::OrbElemCNAV2()
      :OrbElemICE(),
       L1CHealth(0), ITOW(0),
       Tgd(0.0), ISCP(0.0), ISCD(0.0)
   {}

   OrbElemCNAV2::OrbElemCNAV2( const ObsID& obsIDArg,
                               const short PRNIDArg,
                               const int subframe1,
                               const PackedNavBits& subframe2) 
      throw( InvalidParameter)
   {
      loadData( obsIDArg, PRNIDArg, subframe1, subframe2 );
   }

   OrbElemCNAV2* OrbElemCNAV2::clone() const
   {
      return new OrbElemCNAV2 (*this); 
   }

          
   void OrbElemCNAV2::loadData( const ObsID& obsIDArg,
                                const short PRNIDArg,
                                const int subframe1,
                                const PackedNavBits& subframe2)
      throw( InvalidParameter )
   {
      obsID     = obsIDArg;
      satID.id  = PRNIDArg;
   //   satSys = "G";

      short TOWWeek        = subframe2.asUnsignedLong(0, 13, 1);
      ITOW                 = subframe2.asUnsignedLong(13, 8 ,1);
      double Top           = subframe2.asUnsignedLong(21, 11, 300);
      L1CHealth            = subframe2.asUnsignedLong(32, 1, 1);
      double TOWCount      = subframe1*18 + ITOW * 7200;
      URAed                = subframe2.asLong(33, 5, 1);
      double Toe           = subframe2.asUnsignedLong(38, 11, 300);
      double deltaA        = subframe2.asSignedDouble(49, 26, -9);
      Adot                 = subframe2.asSignedDouble(75, 25, -21);
      dn                   = subframe2.asDoubleSemiCircles(100, 17, -44);
      dndot                = subframe2.asDoubleSemiCircles(117, 23, -57);
      M0                   = subframe2.asDoubleSemiCircles(140, 33, -32);
      ecc                  = subframe2.asUnsignedDouble(173, 33, -34);
      w                    = subframe2.asDoubleSemiCircles(206, 33, -32);
      OMEGA0               = subframe2.asDoubleSemiCircles(239, 33, -32);
      i0                   = subframe2.asDoubleSemiCircles(272, 33, -32);
      double deltaOMEGAdot = subframe2.asDoubleSemiCircles(305, 17, -44);
      idot                 = subframe2.asDoubleSemiCircles(322, 15, -44);
      Cis                  = subframe2.asSignedDouble(337, 16, -30);
      Cic                  = subframe2.asSignedDouble(353, 16, -30);
      Crs                  = subframe2.asSignedDouble(369, 24, -8);
      Crc                  = subframe2.asSignedDouble(393, 24, -8);
      Cus                  = subframe2.asSignedDouble(417, 21, -30);
      Cuc                  = subframe2.asSignedDouble(438, 21, -30);
      URAned0              = subframe2.asLong(459, 5, 1);
      URAned1              = subframe2.asUnsignedLong(464, 3, 1);
      URAned2              = subframe2.asUnsignedLong(467, 3, 1);
      af0                  = subframe2.asSignedDouble(470, 26, -35);
      af1                  = subframe2.asSignedDouble(496, 20, -48);
      af2                  = subframe2.asSignedDouble(516, 10, -60);
      Tgd                  = subframe2.asSignedDouble(526, 13, -35);
      ISCP                 = subframe2.asSignedDouble(539, 13, -35);
      ISCD                 = subframe2.asSignedDouble(552, 13, -35);
      short sflag          = subframe2.asUnsignedLong(565, 1, 1);

      A        = A_REF_GPS + deltaA;
      OMEGAdot = OMEGADOT_REF_GPS + deltaOMEGAdot;

      healthy = (L1CHealth==0);
      double timeDiff = Toe - TOWCount;
      short epochWeek = TOWWeek;
      if (timeDiff < -HALFWEEK) epochWeek++;
      else if (timeDiff > HALFWEEK) epochWeek--;

         // Top must be before transmission.
         // Check for week rollover between Top and TOWCount.
      short TopWeek = TOWWeek;
      if(Top>TOWCount) TopWeek--; 
    
      long beginFitSOW = TOWCount;
      short beginFitWk = TOWWeek;
#pragma unused(sflag,beginFitSOW,beginFitWk)
       
         // NOTE: TOWCount actually points to the beginning time of
         // the next 18 second frame.  Therefore, to obtain the
         // transmit time of the beginning of THIS message, we subtract 18 sec.
      transmitTime = GPSWeekSecond(TOWWeek, TOWCount-18, TimeSystem::GPS);

         // If the Toe is an even-hour, then it may be assumed that this is
         // NOT a fresh upload (IS-GPS-200, 20.3.4.5).  Therefore, the 
         // transmission SHOULD have started on an even two-hour boundary 
         // and that is what will be assumed for purposes of the beginning
         // time of validity.
      long longToe = (long) Toe;
      long leastSOW = (static_cast<GPSWeekSecond>(transmitTime)).sow;
      long adjXmit = leastSOW;
      if((longToe % 7200) == 0)
      {
         adjXmit = leastSOW - (leastSOW % 7200); 
      }
      beginValid = GPSWeekSecond(TOWWeek, adjXmit, TimeSystem::GPS ); 

      ctTop = GPSWeekSecond(TopWeek, Top, TimeSystem::GPS);
      ctToe = GPSWeekSecond(epochWeek, Toe, TimeSystem::GPS); 
      ctToc = ctToe;
      
         // Speculation at this point. Need confirmation in form of
         // upated IS-GPS-200 Table 20-XIII that includes a 3 hour
         // fit interval. 
      endValid = ctToe + 3600;  

      dataLoadedFlag = true;   
   } // end of loadData()

   void OrbElemCNAV2::dumpHeader(ostream& s) const
      throw( InvalidRequest )
   {
      if (!dataLoaded())
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }

      ios::fmtflags oldFlags = s.flags();

      OrbElem::dumpHeader(s);
      //s << "Source : " << getNameLong() << endl;
  
      OrbElemICE::dumpHeader(s);
    
      s << endl
        << "           SV STATUS"
        << endl
        << endl
        << "Health bits                    :      0x" << setfill('0')  << setw(2) << L1CHealth;
      s.setf(ios::uppercase);
      s << setfill(' ') << endl;
      s << "Tgd                            : " << setw(15) << setprecision(8) << scientific
        << Tgd << " sec" << endl
        << "ISCP                           : " << setw(15) << setprecision(8) << scientific
        << ISCP << " sec" << endl
        << "ISCD                           : " << setw(15) << setprecision(8) << scientific
        << ISCD << " sec" << endl;

      s.setf(ios::fixed, ios::floatfield);
      s.setf(ios::right, ios::adjustfield);
      s.setf(ios::uppercase);
      s.precision(0);
      s.fill(' ');

      s << endl
        << endl;
      s << "              Week(10bt)  SOW      DOW     UTD   SOD"
        << "     MM/DD/YYYY   HH:MM:SS\n"; 
      s << "Transmit   :  ";
      timeDisplay(s, transmitTime);
      s << endl;
      s.flags(oldFlags);                
   } // end of dumpHeader()   

   ostream& operator<<(ostream& s, const OrbElemCNAV2& eph)
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

} // end namespace
