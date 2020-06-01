#pragma ident "$Id$"


/*
*  X1Sequence.cpp
*
*     GPS X1 Sequencer.
*     Applied Research Laboratories, The University of Texas at Austin
*     August 2003
*/
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





   // Project headers
#include "Exception.hpp"
#include "X1Sequence.hpp"
#include "GenXSequence.hpp"

namespace gpstk
{
      //   Static Variable Definition
   bool X1Sequence::isInit = false;
   uint32_t* X1Sequence::X1Bits = 0;

   X1Sequence::X1Sequence(  )
   {
      if (isInit!=true)
      {
         gpstk::Exception e(
            "Must call X1Sequence::allocateMemory() before instantiating a X1Sequence object.");
         GPSTK_THROW(e);
      }
   }
   
   void X1Sequence::allocateMemory( )
   {
      int X1Aepoch;
      int X1Acount;
      int X1Bepoch;
      int X1Bcount;
      int X1epoch = 1;
      long X1Word = 0;
      int lengthOfX1BSequence;
   
      if (isInit==true)
      {
         gpstk::Exception e ("X1Sequence::allocateMemory() called multiple times");
         GPSTK_THROW(e);
      }
      
      X1Bits = new uint32_t[NUM_6SEC_WORDS];
      if (X1Bits==0) 
      {
         gpstk::Exception e ("X1Sequence::allocateMemory() - allocation failed.");
         GPSTK_THROW(e);
      }
   
         // Generate the X1A and X1B sequences.
      gpstk::GenXSequence X1A( X1A_INIT, X1A_TAPS, XA_COUNT, XA_EPOCH_DELAY);
      gpstk::GenXSequence X1B( X1B_INIT, X1B_TAPS, XB_COUNT, XB_EPOCH_DELAY);
   
         // Combination will be performed for four X1 epochs.
         // This will generate six seconds of X1 bits sequence
         // that will end on an even 32-bit boundary.
      uint32_t X1Abits;
      uint32_t X1Bbits;
      X1Aepoch = 1;
      X1Acount = 0;
      X1Bepoch = 1;
      X1Bcount = 0;
      lengthOfX1BSequence = XB_COUNT;
   
      while ( X1Word < NUM_6SEC_WORDS )
      {
            // Get 32 X1A bits.  Update counters and handle rollovers.
         X1Abits = X1A[X1Acount];
         X1Acount += MAX_BIT;
      
         if ( X1Acount >= XA_COUNT )
         {
            ++X1Aepoch;
            if (X1Aepoch>XA_MAX_EPOCH)
            {
               ++X1epoch;
               X1Aepoch = 1;
            }
            X1Acount = X1Acount - XA_COUNT;
         }
      
            // Get 32 X1B bits.  Update counters and handle rollovers
         X1Bbits = X1B[X1Bcount];
         X1Bcount += MAX_BIT;
         if (X1Bcount >= lengthOfX1BSequence )
         {
            X1Bcount = X1Bcount - lengthOfX1BSequence;
            ++X1Bepoch;
            if (X1Bepoch>XB_MAX_EPOCH) X1Bepoch = 1;
            if (X1Bepoch==XB_MAX_EPOCH) 
               lengthOfX1BSequence = XB_COUNT+XB_EPOCH_DELAY;
             else
               lengthOfX1BSequence = XB_COUNT;
            X1B.setLengthOfSequence( lengthOfX1BSequence );
         }
         
         X1Bits[X1Word++] = X1Abits ^ X1Bbits;
      }   
   
      isInit = true;
   }

   void X1Sequence::deAllocateMemory()
   {
      if (isInit!=true || X1Bits==0)
      {
         gpstk::Exception e("X1Sequence::deAllocateMemory() called when no memory allocated.");
         GPSTK_THROW(e);
      }
      delete [] X1Bits;
      isInit = false;
   }
   
}     // end of namespace

