// $Id$

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






/*
 *  GenXSequence.cpp - Generate GPS X-register sequences
 */

   // Local headers
#include <string>
#include "GenXSequence.hpp"

using namespace std;
namespace gpstk
{
      // Constructor
   GenXSequence::GenXSequence( const unsigned int initialState, 
                               const unsigned int tapRegister,
                               const int initialLengthOfSequence,
                               const int maxDelay )
   {

         /*
         *  NOTE: the x register functions assume a LSB-to-MSB shift.  Therefore,
         *  the tap register definition and the initialization definition for the
         *  X register correspond to the diagrams in ICD-GPS-200, assuming 
         *  "stage 0" is the LSB.  That means the diagrams have to be read from
         *  LSB (LEFT) -> MSB (RIGHT).  The corresponding text doesn't have this
         *  problem.
         */  
      unsigned int mask12bits[12] = { 0x0001, 0x0002, 0x0004, 0x0008,
                                      0x0010, 0x0020, 0x0040, 0x0080,
                                      0x0100, 0x0200, 0x0400, 0x0800 };

      unsigned int reg = initialState;
      lengthOfSequence = initialLengthOfSequence;
      maxOfSequence = lengthOfSequence + maxDelay;
   
      uint32_t output;
      word_num = 0;
      bit_num = 0;
      int andBits;
      int i;
   
         // Clear the output array 
      for ( i=0; i<MAX_WORD; ++i ) bits[i] = 0x00000000;
      debugPrint = false;
      
      for ( i=0; i<lengthOfSequence ; ++i)
      {
            // Get current output and store it away 
         if ( (reg & 0x0800) !=  0 ) output = 0x00000001;
          else                       output = 0x00000000;
         addBitToSequence( output );
      
            // Calculate next input bit 
         andBits = reg & tapRegister;
         reg <<= 1;
         reg &= 0x0FFF;
         int cnt = 0;
         for ( int bit12cnt=0; bit12cnt<12; ++bit12cnt)
         {
            if (( andBits & mask12bits[bit12cnt] ) != 0) ++cnt;
         }
         int newBit = cnt % 2;
         reg |= newBit;
      }

         // Fill delay bits with copies of the last ouptut bit
      for ( i=0; i<maxDelay; ++i) addBitToSequence( output );
      
         //  When finished, make sure the last word (which is probably
         //  a partially-filled word) is left-justified.
      if (bit_num>0) bits[word_num] <<= (MAX_BIT-bit_num);
   }

      // Private helper method to avoid duplicate code.
   void GenXSequence::addBitToSequence( uint32_t newBit )
   {
         // Left shift any pre-existing data, then OR on the new 
         // data (assumed to be right-justified).
      bits[word_num] <<= 1;
      bits[word_num] |= newBit;

         /*
            Increment bit pointer and check for word overflow.
            NOTE: Overflow of the WORD pointer is a "programming problem"
            that is unrecoverable and should NEVER happen in production.
         */
      ++bit_num;
      if (bit_num>=MAX_BIT)
      {
         ++word_num;
         bit_num=0;
      }
   }

       /*
          Given a bit position within the X sequence (numbered starting at 0),
          return the next 32 bits.  Note: if there are insufficient bits left
          to fill the request, wrap around to the beginning of the sequence.
          Note that the location of the wrap around can be modified using the
          function GenXSequence::setEndOfSequence( int los );
       */
   uint32_t GenXSequence::operator[] ( int ia )
   {
      uint32_t retArg = 0x00000000;
      int i = ia;
      if (i >= lengthOfSequence) i = i % lengthOfSequence;
      int ndx1 = i / MAX_BIT;
      int offset = i % MAX_BIT;
      if ( (i+MAX_BIT) <= lengthOfSequence )
      {
         if (offset==0) retArg = bits[ndx1];
         else           retArg = merge( bits[ndx1], bits[ndx1+1], offset );
      }
         /*
            Complicated case when coming up to end of sequence.  May have to
            put together parts of up to three words to get 32 bits.  Two words
            at the end of sequence plus "wrap around" bits from beginning of 
            sequence.
              First: If end of sequence doesn't fall in current word, use up
              remaining bits in the current word.
              Second: use bits up to the end of sequence.
              Third: fill remaining bits from the beginning of the sequence.
         */
      else
      {
         int numRemainingInSequence = lengthOfSequence - i;
         int numRemainingInWord = MAX_BIT - offset;
         int numFilled = 0;
      
            /*
               Get bits (if any) from next-to-last word.
            */
         if (numRemainingInWord<numRemainingInSequence)
         {
            retArg = bits[ndx1++] << offset;
            numRemainingInSequence -= numRemainingInWord;
            numFilled = numRemainingInWord;
         }

         uint32_t temp = bits[ndx1];
            /*
               Get bits from last word
               Case 1: No bits from previous word, need only "middle" section
                       from last word.
               Case 2: Need all bits available from final word (may or
                       may not have bits from preceding word)
            */
         if (numFilled==0 && offset!=0)
         {
            temp <<= offset;     // Move to left to clear excess msb
            temp >>= (MAX_BIT-numRemainingInSequence);   // Shift right to clear excess lsb
            temp <<= (MAX_BIT - (numRemainingInSequence+numFilled) );
            retArg |= temp;
         }
         else
         {
            temp >>= (MAX_BIT-numRemainingInSequence);
            temp <<= (MAX_BIT-(numRemainingInSequence+numFilled));
            retArg |= temp;
         }

            // Finally, add bits from the "wraparound" word at the
            // beginning of the array.
         retArg |= bits[0] >> (numRemainingInSequence+numFilled);
      }
      return(retArg);
   }

   void GenXSequence::setLengthOfSequence( int los )
   {
      lengthOfSequence = los;
      return;
   }
}     //    end of namespace
