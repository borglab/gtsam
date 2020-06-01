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


#ifndef SVPCODEGEN_HPP
#define SVPCODEGEN_HPP

#include "CommonTime.hpp"
#include "PCodeConst.hpp"
#include "CodeBuffer.hpp"
#include "X1Sequence.hpp"
#include "X2Sequence.hpp"
#include "GPSZcount.hpp"

namespace gpstk 
{
/** @defgroup code    Code generator   */
//@{
   /**
    *     SV P-code Generator.
    *     Applied Research Laboratories, The University of Texas at Austin
    *     August 2003
    *
    *  SVPCodeGen handles the P-code generation for a particular SV.  Each object
    *  is initialized based on the particular satellite (identified by PRN code) 
    *  and the Z-count when generation is to begin.  The Z-count must be an even
    *  four Z-count (six second) interval.  Input values that do not align with four
    *  Z-count boundaries are rounded BACK to the next earliest 4 Z-count boundary.
    *
    *  After initializaiton, P-code is generated in 6-second "chunks".  The chunks
    *  are returned to the caller in CodeBuffer objects.  After each call to
    *  getNextSixSeconds( ), the application shall call increment4ZCounts( ) 
    *  to advance the time by four Z-counts.  This separation between getting the 
    *  bits and advancing the time is purposely done to allow for easier 
    *  coordination when generating code for multiple satellites and performing
    *  other operations on the data after generating the P-code and before moving
    *  on to the next increment.
    *
    *  A key aspect of this class is the use of the X1Sequence and X2Sequence
    *  classes.  Analysis of the P-code generation processes reveals that the 
    *  combined X1A/X1B bit stream repeats every 1.5s (1 Z-count) and is the
    *  same for all satellites.  At the end of 6s (4 Z-counts), the number of 
    *  bits is evenly divisable by 32 (which is the assumed wordsize for the 
    *  applicationn). Therefore, to obtain X1 bits, getNextSixSeconds( ) need only
    *  access the indiviual words from the X1Sequence starting at the beginning
    *  of the array and progressing to the end.  The restriction that the coder
    *  must start on an even 6-second (4 Z-count) epoch and generate intervals of 
    *  six seconds obviates the need to build code to compute bit positions
    *  within X1A cycles or any information on the X1A/X1B relationship.  See
    *  documentation on the class X1Sequence for more information.
    *
    *  The situation with respect to X2A/X2B is somewhat more complicated.  Once
    *  again, the X2A/X2B sequence is only 1.5s long.  However, the X2 sequence is
    *  "slipped" wrt the X1 sequence in two ways: (1.) once per week (at the 
    *  beginning of week), it is slipped by a number of bits corresponding to
    *  the PRN ID, (2.) slipped 37 bits for each X2A cycle.  This
    *  means that it is necessary to keep track of the bit position within the
    *  X2 sequence and "chop out" 32 bits at a time.  See documentation for the
    *  class X2Sequence for more information.
    */
   class SVPCodeGen
   {
   public:
      /**
       *  SVPCodeGen::SVPCodeGen( const int PRNID, const CommonTime ) - 
       *  Instantiate and initialize a SVPCodeGen object.  Based on the
       *  PRNID and the ZCount, determine the appropriate starting 
       *  location in the X2 sequence and set it up.  Set the current 
       *  time. 
       */
      SVPCodeGen( const int SVPRNID, const gpstk::CommonTime& dt );
      ~SVPCodeGen( ) {};
         
      /**
       *  Starting at the beginning of the X1 sequence and at the 
       *  appropriate location (as determined by time and PRN), advance
       *  through both the X1 sequence and X2 sequences combining the 
       *  sequences 32 bits at a time until the X1 sequence for this
       *  six second period has been exhausted.  Sometime within that 
       *  period, the X2 sequence will be exahusted  (including the
       *  appropriate delays) and will rollover to the beginning of 
       *  the sequence.
       */
      void getCurrentSixSeconds( CodeBuffer& pcb );
         
      /**
       * Generally, the only action is to increment the Z-count by 
       * 4 counts.  This function COULD be included at the end of 
       * getCurrentSixSeconds( ), however, it has been separated to
       * allow the calling application to have a consistent view of 
       * all the state conditions (for output and debug) before 
       * moving the time forward for the next generation.
       */
      void increment4ZCounts();
         
      /**
       * Returns the current time to the calling method.  
       */
      const gpstk::CommonTime& getCurrentZCount() const {return currentZTime;}

      /** Allows the user to set the current time. While a any time may
          be specified, this routine will take the Z % 4 for the actuall 
          time.
      **/
      void setCurrentZCount(const gpstk::GPSZcount& z);
     
   private:
      gpstk::X1Sequence X1Seq;
      gpstk::X2Sequence X2Seq;
      gpstk::CommonTime currentZTime;
      int PRNID;
   };
   //@}
}     // end of namespace
#endif // SVPCODEGEN_HPP
