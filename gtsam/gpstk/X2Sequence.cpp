#pragma ident "$Id$"


/*
*  X2Sequence.cpp
*     GPS X2 Sequencer.
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



   // Language headers
#include <cstring>
#include <stdio.h>
#include <string>

   // Project headers
#include "Exception.hpp"
#include "GenXSequence.hpp"
#include "X2Sequence.hpp"

namespace gpstk
{
      // Static Variable Definition
   bool X2Sequence::isInit = false;
   uint32_t* X2Sequence::X2Bits = 0;
   uint32_t* X2Sequence::X2BitsEOW = 0;

      // See program x2EOW.cpp for derivation of these values
   uint32_t X2Sequence::EOWEndOfSequence[LENGTH_OF_EOW_OVERLAP] =
   {
    0xFA5F8298, 0xB30C04D9, 0xD5CACBCA, 0x0ED47FFF, 0xFFFFFFFF, 0xFFFFFFFF,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF
   };

   X2Sequence::X2Sequence(  )
   {
      if (isInit!=true)
      {
         gpstk::Exception e(
            "Must call X2Sequence::allocateMemory() before instantiating a X2Sequence object.");
         GPSTK_THROW(e);
      }
      
         // This must be done for each object in order to initialize the
         // bitsP pointer to the correct buffer of bits.
      setEOWX2Epoch(false);
   }
   
   void X2Sequence::allocateMemory( )
   {
      int X2Aepoch;
      int X2Acount;
      int X2Bepoch;
      int X2Bcount;
      int X2epoch = 1;
      long X2Word = 0;
      long X2Count = 0;
      int lengthOfX2ASequence;
      int lengthOfX2BSequence;

      if (isInit==true)
      {
         gpstk::Exception e ("X2Sequence::allocateMemory() called multiple times");
         GPSTK_THROW(e);
      }
      
      X2Bits =    new uint32_t[NUM_X2_WORDS];
      X2BitsEOW = new uint32_t[NUM_X2_WORDS];
      if (X2Bits==0 || X2BitsEOW==0) 
      {
         gpstk::Exception e ("X2Sequence::allocateMemory() - allocation failed.");
         GPSTK_THROW(e);
      }
   
      for (long ndx = 0; ndx < NUM_X2_WORDS; ndx++)
      {
         X2Bits[ndx] = 0x00000000;
         X2BitsEOW[ndx] = 0x00000000;
      }

         // Last words of X2Bits and X2BitsEOW are only partially filled.
         // Initialize to 0 to avoid confusion.
      X2Bits[NUM_X2_WORDS-1] = 0x00000000;
      X2BitsEOW[NUM_X2_WORDS-1] = 0x00000000;
   
         // Generate the X2A and X2B sequences.
      gpstk::GenXSequence X2A( X2A_INIT, X2A_TAPS, XA_COUNT, 
                                         XA_EPOCH_DELAY+X2A_EPOCH_DELAY);
      gpstk::GenXSequence X2B( X2B_INIT, X2B_TAPS, XB_COUNT, 
                                         XB_EPOCH_DELAY+X2A_EPOCH_DELAY);

         /*
             In order to handle the beginning of week case, obtain the 
             initial X2 bit, then copy this bit into the first 37 bit 
             positions of the X2 sequence.
         */
      uint32_t firstTest = X2A[0] ^ X2B[0];
      if (firstTest & 0x80000000 ) 
         X2Bits[0] = 0xFFFFFFFF;
       else
         X2Bits[0] = 0x00000000;

      X2Bits[1] = firstTest >> 5;
      X2Bits[1] |= ( X2Bits[0] & 0xF8000000 );
   
         /*
            Previous section handled the beginning of week 37 chip delay 
            plus the first 27 chips (64 bits - 37 chip = 27) of the X2
            cycle.  Set the counters accordingly and start retrieving bits.
            The combination will be performed for four X2 epochs.
            This will generate six seconds+ of X2 bits sequence.
         */
      uint32_t X2Abits;
      uint32_t X2Bbits;
      X2Aepoch = 1;
      X2Acount = 27;
      X2Bepoch = 1;
      X2Bcount = 27;
      X2Word = 2;
      X2Count = X2Word * MAX_BIT;
   
      lengthOfX2ASequence = XA_COUNT;
      X2A.setLengthOfSequence( lengthOfX2ASequence );
   
      lengthOfX2BSequence = XB_COUNT;
      X2B.setLengthOfSequence( lengthOfX2BSequence );
   
      while ( X2Count < MAX_X2_COUNT )
      {
            // Get 32 X2A bits.  Update counters and handle rollovers.
         X2Abits = X2A[X2Acount];
         X2Acount += MAX_BIT;
      
         if ( X2Acount >= lengthOfX2ASequence )
         {
            X2Acount = X2Acount - lengthOfX2ASequence;
            ++X2Aepoch;
            if (X2Aepoch>XA_MAX_EPOCH)
            {
               ++X2epoch;
               X2Aepoch = 1;
            }
            if (X2Aepoch==XA_MAX_EPOCH)
               lengthOfX2ASequence = XA_COUNT+X2A_EPOCH_DELAY;
            else
               lengthOfX2ASequence = XA_COUNT;
            X2A.setLengthOfSequence( lengthOfX2ASequence );
         }
      
            // Get 32 X2B bits.  Update counters and handle rollovers
         X2Bbits = X2B[X2Bcount];
         X2Bcount += MAX_BIT;
         if (X2Bcount >= lengthOfX2BSequence )
         {  
            X2Bcount = X2Bcount - lengthOfX2BSequence;
            ++X2Bepoch;
            if (X2Bepoch>XB_MAX_EPOCH) X2Bepoch = 1;
            if (X2Bepoch==XB_MAX_EPOCH) 
               lengthOfX2BSequence = XB_COUNT+XB_EPOCH_DELAY+X2A_EPOCH_DELAY;
             else
               lengthOfX2BSequence = XB_COUNT;
            X2B.setLengthOfSequence( lengthOfX2BSequence );
         }
         
         X2Bits[X2Word++] = X2Abits ^ X2Bbits;
         X2Count += MAX_BIT;
      }   

         // At this point, the X2Bits array is complete.  Copy the entire
         // array into X2BitsEOW, then overlay the EOW section into the
         // appropriate place.
      const size_t numBytesPerWord = 4;
      size_t numBytes = NUM_X2_WORDS * numBytesPerWord;
      std::memcpy( X2BitsEOW, X2Bits, numBytes );
      numBytes = LENGTH_OF_EOW_OVERLAP * numBytesPerWord;
      memcpy( (void *) &X2BitsEOW[OVERLAP_WORD_POSITION], EOWEndOfSequence, numBytes );
   
      isInit = true;
   }

   void X2Sequence::deAllocateMemory()
   {
      if (isInit!=true || X2Bits==0 || X2BitsEOW==0)
      {
         gpstk::Exception e("X2Sequence::deAllocateMemory() called when no memory allocated.");
         GPSTK_THROW(e);
      }
      delete [] X2Bits;
      delete [] X2BitsEOW;
      isInit = false;
   }
   
   void X2Sequence::setEOWX2Epoch( const bool tf )
   {
      if (tf) bitsP = X2BitsEOW;
       else   bitsP = X2Bits;
   }

}  // end of namespace
