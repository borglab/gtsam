#pragma ident "$Id$"


//  X2Sequence.hpp - GPS X2 Sequencer

#ifndef X2SEQUENCE_HPP
#define X2SEQUENCE_HPP

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






   // Local headers
#include "gpstkplatform.h"
#include "PCodeConst.hpp"
#include "mergePCodeWords.h"

namespace gpstk
{
/** @addtogroup code   */
//@{
      
      /*
         The following constants are derived in x2EOW.cpp and
         used as literals here.
      */
   const long LENGTH_OF_EOW_OVERLAP =      34;
   const long OVERLAP_WORD_POSITION = 1451897;
      // Maximum number of X2 chips (exclusive of BOW delay chips)
   const long MAX_X2_TEST = 4 * ((XA_COUNT * XA_MAX_EPOCH) + X2A_EPOCH_DELAY); 
      // Maximum number of X2 chips
   const long MAX_X2_COUNT = X2A_EPOCH_DELAY + MAX_X2_TEST;

      /**
       *     GPS X2 Sequencer.
       *     Applied Research Laboratories, The University of Texas at Austin
       *     August 2003
       *
       *  X2Sequence is a static class (one instance per executable) that
       *  contains a little more than a six-second (four Z-count) sequence 
       *  of the combined X2A/X2B data for the GPS constellation.  The 
       *  X2 sequence repeats roughly every 1.5s (each X1 epoch) and is 
       *  identical for all SVs.  The progressive "slip" in the relative
       *  relationship between X1 and X2 is responsible for progressing the 
       *  P-code bitstream in time while a beginning of week offset is
       *  responsible to differentiating between unique satellite codes.
       *
       *  These two slips are responsbile for some special circumstances in
       *  the X2 bit sequence.  The satelite differentiation is handled by 
       *  "holding" the X2 state at the beginning of week bit for a delay 
       *  equivalent to the PRN ID of the satellite.  This is handled by 
       *  replicating the first bit of the combined X2A/X2B bitstream 37 
       *  times at the front of the bit sequence.  Therefore, a bit index
       *  of "1" will be internally interpreted as 38.  At the beginning
       *  of the week, the calling application shall coerce the beginning 
       *  bit of the first call to X2 sequence to be 1-PRNID.
       *
       *  The other delay is a 37 chip delay at the end of each X2A epoch.  
       *  During this delay, the X2 state is "held" in the final bit state.  
       *
       *  Given these considerations, the following caluclations derive the 
       *  number of bits that make up an X2Sequence and the number of unsigned 
       *  32-bit words required to hold these bits:
       *
       *  Base number of bits in a X2A epoch: 4092 chips
       *  Number of X2A epochs in an X2 Epoch: 3750 epochs (cycles)
       *  Delay at end of each X2 Epoch: 37 chips
       *  Number of bits required for a SINGLE X2 Epoch:
       *             4092 * 3750 + 37 = 15,345,037 bits.
       *  Number of bits in FOUR X2 Epochs (corresponding to FOUR X1 Epochs):
       *             4 * 15,345,037 = 61,380,148 bits.
       *  Number of PRN delay bits to be added to FRONT of array to be us
       *  at the beginning of week: 37.
       *
       *  Total number of bits required: 61,380,148 + 37 = 61,380,185 bits.
       *  Number of 32 bit words required:
       *     61,380,185 / 32 = 1,918,131 (1,918,130 Remainder of 25).
       *
       *  SPECIAL CONDITION
       *     The final X2 sequence of the week is both truncated and unique.
       *  X2Sequence makes special provision for handing this condition.  The
       *  final X2 epoch starts well into the final X1 epoch, consequently, 
       *  the final X2 epoch contains ?? X2A cycles at which point, the final 
       *  X2A chips is "held" until week rollover occurs.  The final X2 epoch 
       *  contains ?? X2B cycles at which point, the final X2B chip is "held"
       *  (repeated) until week rollover occurs.  This means bits ??-?? of the
       *  X2Sequence differ for this final cycle (and the end of this final X2 
       *  cycle is at bit ??). 
       *
       *  It's important to avoid having a conditional in the inline subscript
       *  operator.  Therefore, the design implements two buffers.  Each are 
       *  6+ seconds long.  The two buffers only differ in the 1069 chip span 
       *  (34 words) where there are  two different patterns. 
       *
       *  The internal points unsigned long *bitsP is used to track which buffer
       *  (X2bits[] or X2bitsEOW[] is in use at a particular time.  The method
       *  setEOWX2Epoch( bool tf ) is used to control the buffer to which *bitsP
       *  currently points.  X2Sequence initialize2 the bitsP pointer to point
       *  to the regular array.  The calling application will need to manage 
       *  the timing of calling setEOWX2Epoch.  This needs to be done for EACH
       *  X2Sequence object - X2Bits and X2BitsEOW are static, but the bitsP
       *  pointer is a member veriable of the class and NOT static.
       */
   class X2Sequence
   {
      public:
            /** Initialize the local variables associated with 
              * this object. In the case of this class, this is a significant
              * amount of work. The X2A/X2B process described in ICD-GPS-200B 
              * is followed in order to fill the X2Bits array.
              */
         X2Sequence();
         ~X2Sequence( ) {};
    
            /**  The X2 sequence requires two 6-second buffers of 10MBit/sec 
             *   samples.  This comes to approximately 4 million four-byte
             *   unsigned integers.  These data are the same for all PRN codes
             *   To minimize the memory footprint, these data are stored
             *   in a dynamically-allocated static array.  It is 
             *     - - - - NECESSARY - - - -
             *   that the calling method call X2Sequence::allocateMemory()
             *   PRIOR to instantiating the first X2Sequence object. 
             *   X2Sequence::allocateMemory() should only be called once.
             *   Violation of either condition will result in a 
             *   gpstk::Exception thrown from either X2Sequence::X2Sequence()
             *   or X2Sequence::allocateMemory().
             *
             *   The X2Sequence::deAllocateMemory() method may be called to
             *   release the memory (if desired) but it should only be called
             *   after all X2Sequence objects have been "destroyed".
             */
         static void allocateMemory( );
         static void deAllocateMemory( );
         
            /** Given a bit number from -37 to X2Length-37, stuff the 32 bits
             *  starting with that bit and continuing for the next 31 bits into
             *  an unsigned long and return this as the result.  Conditional 
             *  code (compiled only for debug) will confirm that the requested
             *  bit number is >=-37 and <(61,380,185-32) and will halt the
             *  program if this assertion is violated.  
             *  NOTE: operator[] should never have to worry about rollovers.
             *  The length of the X2 array shall be set such that the maximum
             *  number of X2 bits needed will be available.  The reset will be
             *  driven by the occurrence of the next X1 epoch and be tracked 
             *  one level up in the corresponding SVPcodeGen object.
             */
         uint32_t operator[]( long i );

            /**  Controls whether the X2 Epoch is set to EOW condition
             *   or normal condition.  Should only be set true for the final
             *   X2 epoch of the week.
             */
         void setEOWX2Epoch( const bool tf );

      private:
         uint32_t *bitsP;
         static uint32_t* X2Bits;
         static uint32_t* X2BitsEOW;
         static uint32_t EOWEndOfSequence[LENGTH_OF_EOW_OVERLAP];
         static bool isInit; 
   };

       /*
          Given a bit position within the X2 sequence (numbered starting at -37),
          return the next 32 bits.  Note: if there are insufficient bits left
          to fill the request, wrap around to the beginning of the sequence.
       */
   inline uint32_t X2Sequence::operator[] ( long i )
   {
      long adjustedCount = i + X2A_EPOCH_DELAY;
   
      uint32_t retArg = 0L;
      int ndx1 = adjustedCount / MAX_BIT;
      int offset = adjustedCount - (ndx1 * MAX_BIT);
      if ( (adjustedCount+MAX_BIT) <= MAX_X2_COUNT )
      {
         if (offset==0) retArg = bitsP[ndx1];
         else           retArg = merge( bitsP[ndx1], bitsP[ndx1+1], offset );
      }
         /*
            Complicated case when coming up to end of sequence.  May have to
            put together parts of up to three words to get 32 bits.  The problem 
            is complicated because the word at the end of the array is partial
            AND the beginning of sequence (BOW) occurs in mid-word due to the
            PRN offset.  Some numbers:
         
            Number of bits available in word N : 25
            Number of bits available in word at BOS : 27
                  
            Possible cases:
            1.) Combine bits from [n-1], [n], and [BOS] - word n will provide
                25 bits.  Therefore, some combination of (n-1,BOS) from the 
                choice of (1,4), (2,3), (3,2), (4,1).
             
            2.) Combine bits from [n-1] and [BOS] - word n-1 will provide 25-5 
                bits.  Therefore, BOS will provide 7-27 bits. 
         
            3.) Combine bits from [n-1], [BOS], and [BOS+1] - word n-1 provides
                4-1 bits.  BOS provides 27 bits (running total to 31-28 bits).
                BOS+1 provides 1-4 bits.
         */
      else
      {
         int numRemainingInSequence = MAX_X2_COUNT - adjustedCount;
         int numRemainingInWord;
         int numFilled = 0;

            // Handle word n-1
         if (ndx1==NUM_X2_WORDS-2)
         {
            numRemainingInWord = MAX_BIT - offset;
            retArg = bitsP[ndx1++] << offset;
            numFilled = numRemainingInWord;
            numRemainingInSequence -= numRemainingInWord;
         }
      
            // Handle word n
         uint32_t temp = bitsP[ndx1];
         numRemainingInWord = numRemainingInSequence;
         temp >>= (MAX_BIT-numRemainingInWord);
         temp <<= (MAX_BIT-(numRemainingInWord+numFilled));
         retArg |= temp;
         numFilled += numRemainingInSequence;
      
            //   Wrap to front.  Recall that "front" is actually bit
            //   37 in sequence due to "beginning of week" delay
         numRemainingInWord = (2 * MAX_BIT) - X2A_EPOCH_DELAY;
         int numNeeded = MAX_BIT - numFilled;
      
            // Case where all bits needed are in word 1 of sequence array
            //(which only has 27 "useful" bits)
         if (numNeeded <= numRemainingInWord)
         {
            temp = bitsP[1] << (MAX_BIT - numRemainingInWord);
            temp >>= (MAX_BIT - numNeeded);
            retArg |= temp;
         }
            // Case where all bits in word 1 are needed plus some bits in word 2.
         else
         {
               // Clearing high-order bits
            temp = bitsP[1] << (MAX_BIT - numRemainingInWord);
            temp >>= (MAX_BIT - numRemainingInWord);
            temp <<= (MAX_BIT - (numRemainingInWord+numFilled));
            retArg |= temp;
         
               // Fetch remaining bits from next word 
            numFilled += numRemainingInWord;
            numNeeded = MAX_BIT - numFilled;
            temp = bitsP[2] >> (MAX_BIT - numNeeded);
            retArg |= temp;
         }
      }
      return(retArg);
   }
   //@}
}  // end of namespace

#endif // X2SEQUENCE_HPP
