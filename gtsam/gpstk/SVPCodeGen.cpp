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


#include <iostream>
#include "SVPCodeGen.hpp"
#include "GPSWeekZcount.hpp"

using namespace std;
namespace gpstk
{
   const long LAST_6SEC_ZCOUNT_OF_WEEK = 403200 - 4;

   SVPCodeGen::SVPCodeGen( const int SVPRNID, const gpstk::CommonTime& dt )
   {
      if (SVPRNID < 1 || SVPRNID > 210)
      {
         gpstk::Exception e("Must provide a prn between 1 and 210");
         GPSTK_THROW(e);
      }
      currentZTime = dt;
      PRNID = SVPRNID;
   }

   void SVPCodeGen::getCurrentSixSeconds( CodeBuffer& pcb )
   {
         // Compute appropriate X2A offset
      int dayAdvance = (PRNID - 1) / 37;
      int EffPRNID = PRNID - dayAdvance * 37;
      long X1count = GPSWeekZcount(currentZTime + dayAdvance*86400.0).zcount;
      long X2count;
   
         /*
            Trivial, but special, case for beginning of week.  This
            can't be simplified into the general case due to the beginning
            of week chip delays that are equivalent to the PRNID.  These
            chips are stored at the beginning of the X2 chips sequence.  This
            is the only time the X2count should be "negative".  The offset is
            handled within the X2Sequence::operator[] method.
         */
      if (X1count==0 && PRNID <= 37) X2count = -PRNID;
   
         /*
            At the beginning of an X1 epoch, the previous X2 epoch
            will still be unfinished due to delays.  The accumulated
            delay is based on the PRNID and the delay per X1 epoch.
            Subtract this delay from the max length of the X2 sequence
            to determine the current chip within the X2 sequence.
         */
      else
      {
         long cumulativeX2Delay = X1count * X2A_EPOCH_DELAY + EffPRNID;
         X2count = MAX_X2_TEST - cumulativeX2Delay;
         if (X2count<0) X2count += MAX_X2_TEST;
      }

         /*
            If this if the final six-second interval of the week, 
            signal the X2 bit sequence generator to use the "end of week"
            sequence.  Otherwise, use the "regular" sequence.
         */
      if ( X1count==LAST_6SEC_ZCOUNT_OF_WEEK) X2Seq.setEOWX2Epoch(true);
       else X2Seq.setEOWX2Epoch(false);
   
         // Update the time and code state in the CodeBuffer object
      pcb.updateBufferStatus( currentZTime, P_CODE );
   
         // Starting at the beginning of the interval, step through
         // the six second period loading the code buffer as we go.
      for ( long i=0;i<NUM_6SEC_WORDS;++i )
      {
         pcb[i] = X1Seq[i] ^ X2Seq[X2count];
         X2count += MAX_BIT;
         if (X2count>=MAX_X2_TEST) X2count -= MAX_X2_TEST;
      }
   }

   void SVPCodeGen::increment4ZCounts( )
   {
      currentZTime += 6;    // 6 seconds == 4 Zcounts.
   }
   
   void SVPCodeGen::setCurrentZCount(const gpstk::GPSZcount& z)
   {
      GPSZcount z2 = z - z%4;
      currentZTime = GPSWeekZcount(z2.getWeek(), z2.getZcount());
   }
}     // end of namespace
