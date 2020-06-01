#pragma ident "$Id: $"

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
 * @file OrbElemCNAV.cpp
 * OrbElemCNAV data encapsulated in engineering terms
 */
#include <iomanip>
#include <cmath>

#include "StringUtils.hpp"
#include "GNSSconstants.hpp"
#include "GPSWeekSecond.hpp"
#include "SVNumXRef.hpp"
#include "TimeString.hpp"
#include "OrbElemCNAV.hpp"

namespace gpstk
{
   using namespace std;
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wreorder"
   OrbElemCNAV::OrbElemCNAV()
      :OrbElemICE(),
       L1Health(0), L2Health(0), ITOW(0),
       L5Health(0),
       ctMsg10(CommonTime::BEGINNING_OF_TIME),
       ctMsg11(CommonTime::BEGINNING_OF_TIME),
       ctMsgClk(CommonTime::BEGINNING_OF_TIME)
   {
      ctMsg10.setTimeSystem(TimeSystem::GPS);
      ctMsg11.setTimeSystem(TimeSystem::GPS);
      ctMsgClk.setTimeSystem(TimeSystem::GPS);
   }
#pragma clang diagnostic pop
   OrbElemCNAV::OrbElemCNAV( const ObsID& obsIDArg,
                             const SatID& satIDArg,
                             const PackedNavBits& message10,
                             const PackedNavBits& message11,
                             const PackedNavBits& messageClk)
      throw( InvalidParameter)
   {
      loadData( obsIDArg, satIDArg, message10, message11, messageClk);
   }

   OrbElemCNAV* OrbElemCNAV::clone() const
   {
      return new OrbElemCNAV (*this); 
   }

          
   void OrbElemCNAV::loadData( const ObsID& obsIDArg,
                               const SatID& satIDArg,
                               const PackedNavBits& message10,
                               const PackedNavBits& message11,
                               const PackedNavBits& messageClk)
      throw( InvalidParameter )
   {

      obsID     = obsIDArg;
      satID     = satIDArg;

           // Message Type 10 data
      unsigned long TOWCount10  = message10.asUnsignedLong(20, 17, 6);
      short TOWWeek             = message10.asUnsignedLong(38, 13, 1);
      L1Health                  = message10.asUnsignedLong(51, 1, 1);
      L2Health                  = message10.asUnsignedLong(52, 1, 1);
      L5Health                  = message10.asUnsignedLong(53, 1, 1);
      double Top                = message10.asUnsignedLong(54, 11, 300);
      URAed                     = message10.asLong(65, 5, 1);
      double Toe                = message10.asUnsignedLong(70, 11, 300);
      double deltaA             = message10.asSignedDouble(81, 26, -9);
      Adot                      = message10.asSignedDouble(107, 25, -21);
      dn                        = message10.asDoubleSemiCircles(132, 17, -44);
      dndot                     = message10.asDoubleSemiCircles(149, 23, -57);
      M0                        = message10.asDoubleSemiCircles(172, 33, -32);
      ecc                       = message10.asUnsignedDouble(205, 33, -34);
      w                         = message10.asDoubleSemiCircles(238, 33, -32);
      short ISFVal              = message10.asUnsignedLong(271, 1, 1);
      if (ISFVal==1) IntegrityStatusFlag = true;
      L2CPhasing                = message10.asUnsignedLong(272, 1, 1);    
       
         // Message Type 11 data
      unsigned long TOWCount11  = message11.asUnsignedLong(20, 17, 6);
      double Toe11              = message11.asUnsignedLong(38,11,300);
      OMEGA0                    = message11.asDoubleSemiCircles(49, 33, -32);
      i0                        = message11.asDoubleSemiCircles(82, 33, -32);
      double deltaOMEGAdot      = message11.asDoubleSemiCircles(115, 17, -44);
      idot                      = message11.asDoubleSemiCircles(132, 15, -44);
      Cis                       = message11.asSignedDouble(147, 16, -30);
      Cic                       = message11.asSignedDouble(163, 16, -30);
      Crs                       = message11.asSignedDouble(179, 24, -8);
      Crc                       = message11.asSignedDouble(203, 24, -8);
      Cus                       = message11.asSignedDouble(227, 21, -30);
      Cuc                       = message11.asSignedDouble(248, 21, -30);

         // Message Type Clock data
      unsigned long TOWCountClk = messageClk.asUnsignedLong(20, 17, 6);
      URAned0                   = messageClk.asLong(49, 5, 1);
      URAned1                   = messageClk.asUnsignedLong(54, 3, 1);
      URAned2                   = messageClk.asUnsignedLong(57, 3, 1);
      double Toc                = messageClk.asUnsignedLong(60, 11, 300);
      af0                       = messageClk.asSignedDouble(71, 26, -35);
      af1                       = messageClk.asSignedDouble(97, 20, -48);
      af2                       = messageClk.asSignedDouble(117, 10, -60);

      A        = A_REF_GPS + deltaA;
      OMEGAdot = OMEGADOT_REF_GPS + deltaOMEGAdot;

      if(Toe!=Toc ||
         Toe!=Toe11)
      {
       InvalidParameter exc("Toe(s) and Toc are not consistent.");
       GPSTK_THROW(exc);    
      } 

      healthy = false;
      if (obsIDArg.band == ObsID::cbL2 && L2Health == 0) healthy = true;
      if (obsIDArg.band == ObsID::cbL5 && L5Health == 0) healthy = true;

      double timeDiff = Toe - TOWCount10;
      short epochWeek = TOWWeek;
      if (timeDiff < -HALFWEEK) epochWeek++;
      else if (timeDiff > HALFWEEK) epochWeek--;

         // Note that TOW times are referenced to the beginning of 
         // the next message.  Therefore, the transmit time is actually
         // PRIOR to the TOW count in SOW.  However, the amount prior
         // is dependent on the signal.   12 sec for L2C, 6 sec for L5
      unsigned long SOWOffset = 6;
      if (obsID.code==ObsID::tcC2M ||
          obsID.code==ObsID::tcC2L ||
          obsID.code==ObsID::tcC2LM) SOWOffset=12;

         // Message Type 10  CONTAINS the week number.  Therefore, there
         // should not be any SOW rollover to deal with for Message Type 10. 
         // However, we don't know for certain that the 11 and the clock (31-37)
         // messages were collected in the same week (probably, but not certainly).
         // Therefore, we need to look for week rollovers.
      long SOW10 = TOWCount10 - SOWOffset;
      ctMsg10  =    GPSWeekSecond(TOWWeek, SOW10, TimeSystem::GPS);

         // Set up temp variables for MsgType 11 SOW and week.   
      short Week11 = TOWWeek;
      long SOW11 = TOWCount11-SOWOffset;

         // First, check to see that the TOW -> transmit time adjust did not 
         // cross the week boundary.
      if (SOW11<0) SOW11 += gpstk::FULLWEEK;

         // Then check against the MsgType 10 XMit time/week      
      long check = (SOW10-SOW11);
      if (check>gpstk::HALFWEEK) Week11++;
      else if (check<-gpstk::HALFWEEK) Week11--; 
      ctMsg11  =    GPSWeekSecond(Week11, SOW11, TimeSystem::GPS);

         // Now follow the same process for the clock message times.
      short WeekClk = TOWWeek;
      long SOWClk = TOWCountClk - SOWOffset;
      if (SOWClk<0) SOWClk += gpstk::FULLWEEK;
      check = (SOW10-SOW11);
      if (check>gpstk::HALFWEEK) WeekClk++;
      else if (check<-gpstk::HALFWEEK) WeekClk--; 
      ctMsgClk =    GPSWeekSecond(WeekClk, SOWClk, TimeSystem::GPS);

         // The fit interval is nominally the middle of the trasmission interval.
         // However, interval may "start late" for an upload.  Therefore, start
         // from the Toe (rounded to the middle) and work forward/backward from 
         // there.  The "beginning" is nominally the even two hour interval prior
         // to the Toe; however, if the Toe is offset from the middle, the 
         // beginning is the later of the beginning of fit and the transmission 
         // time. 
      long testSOW1 = (static_cast<GPSWeekSecond>(ctMsg10)).sow;
      long testSOW2 = (static_cast<GPSWeekSecond>(ctMsg11)).sow;
      long testSOW3 = (static_cast<GPSWeekSecond>(ctMsgClk)).sow;

      long midPointSOW = (long) Toe;
      long Xmit = 0;
      long offsetFromEven2Hours = midPointSOW % TWO_HOURS;
      bool smallOffset = false;
      if (offsetFromEven2Hours!=NINTY_MINUTES)
      {
         smallOffset = true; 
         
           // Compute the "small offset".   Negative value means the
           // midPoint is early.  
         unsigned long sizeOfOffset = offsetFromEven2Hours - NINTY_MINUTES; 

            // Apply the offset such that the midPointSOW is now the 
            // nominal midpoint of the tranmission interval.
         midPointSOW -= sizeOfOffset; 
      }

      if (smallOffset) // May not have been an even two hour change
      {
         long leastSOW = testSOW1;
         if(testSOW2<leastSOW) leastSOW = testSOW2;
         if(testSOW3<leastSOW) leastSOW = testSOW3;
         Xmit = leastSOW - (leastSOW % (4*SOWOffset));    // See -705B Table 20-XII
            // Still need a test to assure that the begin of fit doesn't'
            // exceed half the interval from the mid-point.  (During live
            // CNAV test of June 2013, Msg 11 was cutting over early.)
         long defBegFit = midPointSOW - NINTY_MINUTES;
         if (Xmit<defBegFit) Xmit = defBegFit; 
      }
      else
      {
         long leastSOW = testSOW1;
         if(testSOW2<leastSOW) leastSOW = testSOW2;
         if(testSOW3<leastSOW) leastSOW = testSOW3;
         Xmit = leastSOW - (leastSOW % TWO_HOURS);
         long defBegFit = midPointSOW - NINTY_MINUTES;
         if (Xmit<defBegFit) Xmit = defBegFit; 
      } 
      beginValid = GPSWeekSecond(TOWWeek, Xmit, TimeSystem::GPS);

         // Top must be before transmission.
         // Check for week rollover between Top and TOWCount.
      short TopWeek = TOWWeek;
      double testSOW = (static_cast<GPSWeekSecond>(beginValid)).sow;
      if(Top>testSOW) TopWeek--;

      ctTop = GPSWeekSecond(TopWeek, Top, TimeSystem::GPS);
      ctToe = GPSWeekSecond(epochWeek, Toe, TimeSystem::GPS);
      ctToc = GPSWeekSecond(epochWeek, Toc, TimeSystem::GPS);

         // The end of validity is 1.5 hours after the nominal midpoint
      double endSOW = midPointSOW + NINTY_MINUTES;
      short endWeek = TOWWeek;
      if (endSOW> gpstk::FULLWEEK)   
      {
         endSOW -= gpstk::FULLWEEK;
         endWeek++;
      }
      endValid = GPSWeekSecond(endWeek, endSOW, TimeSystem::GPS);

      dataLoadedFlag = true;   
   } // end of loadData()

   void OrbElemCNAV::dumpHeader(ostream& s) const
      throw( InvalidRequest )
   {
      if (!dataLoaded())
      {
         InvalidRequest exc("Required data not stored.");
         GPSTK_THROW(exc);
      }

    //  ios::fmtflags oldFlags = s.flags();
    
      OrbElem::dumpHeader(s);
      //s << "Source : " << getNameLong() << endl;
  
      OrbElemICE::dumpHeader(s);
    
      s << endl
        << "           SV STATUS"
        << endl
        << endl
        << "Health bits  L1, L2, L5        :     " << setfill('0') << setw(1)
        << L1Health << ",  " << L2Health << ",  " << L5Health
        << endl; 
      s << "L2C Phasing                    :     " << setfill(' ')
        << L2CPhasing << " (0=quadrature, 1=in-phase)" 
        << endl; 
     
      s.setf(ios::fixed, ios::floatfield);
      s.setf(ios::right, ios::adjustfield);
      s.setf(ios::uppercase);
      s.precision(0);
      s.fill(' ');

      s << endl
        << endl;
      s << "           TRANSMIT TIMES"
        << endl << endl;
      s << "              Week(10bt)     SOW     DOW   UTD     SOD"
        << "   MM/DD/YYYY   HH:MM:SS\n"; 
      s << "Message 10:   ";
      timeDisplay(s, ctMsg10);
      s << endl;
      s << "Message 11:   ";
      timeDisplay(s, ctMsg11);
      s << endl;
      s << "Clock:        ";
      timeDisplay(s, ctMsgClk);
      s << endl;
   } // end of dumpHeader()   

   ostream& operator<<(ostream& s, const OrbElemCNAV& eph)
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
