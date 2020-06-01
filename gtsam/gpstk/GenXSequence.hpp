#pragma ident "$Id$"



//  GenXSequence.hpp -  Generate X Sequence class.

#ifndef GENXSEQUENCE_HPP
#define GENXSEQUENCE_HPP

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



#include "gpstkplatform.h"
#include "PCodeConst.hpp"
#include "mergePCodeWords.h"

namespace gpstk
{
/** @addtogroup code  */
//@{
   
      // Derived from X2A maximum length (see xconst.hpp) 
   const int MAX_WORD = 140;

      /**
       *     Generate X Sequence for GPS
       *     Applied Research Laboratories, The University of Texas at Austin
       *     August 2003
       *
       *  GenXSequence holds a bit buffer containing one of the GPS code
       *  sequences: X1A, X1B, X2A, X2B.
       *
       *  The first 4092 or 4093 bits are constructed using the algorithms
       *  from ICD-GPS-200, section 3.3.2.  The process of generating the
       *  bits is identical for each of the four sequences. The process 
       *  consists of a 12-bit shift register which is initialized to a
       *  specific value and a specification of the "taps" on the shift 
       *  register. As a given bit is read off the high end of the register,
       *  the sum of the bits specified by the taps is used to determine
       *  is 0 or 1 is used as the next input to the shift register.  This
       *  process is defined in more detail in ICD-GPS-200 section 3.3.2.2.
       *
       *  Once the 4092 or 4093 bits are constructed, in the case of X1B, X2A,
       *  and X2B it is necesary to add copies of the last bit to account for 
       *  delay states in which the register is "held" in it's final position 
       *  for many counts while X1A completes a cycle.
       *
       *  Once the sequences are created, the array subscript operator is used 
       *  to access the sequences 32 bits at a time.  The index passed to the 
       *  array subscript operator is the bit position within the sequence 
       *  (first bit is equal to bit 0).  When the process reaches the end of
       *  the sequence, the operator will "wrap around" and restart at the 
       *  front of the array in order to fill the 32-bit return word.
       *
       *  For X1A, the length of the sequence will always be 4092 bits.  For 
       *  all other registers, the length of the sequence is a variable, 
       *  dependent on the epoch and the time of week.  That information is
       *  not available to GenXSequence; therefore, the method 
       *  GenXSequence::setEndOfSequence( los ) is provided to allow the 
       *  calling program to adjust the end of sequence accordingly. 
       *  NOTE: this means the calling program also needs to specify
       *  a maximum sequence length (see constructor) that is equal or greater
       *  than the maximum length the calling program will use.
       */
   class GenXSequence
   {
      public:
            /**
             *  The initial state of the register and the defintion of the
             *  taps are provided to the constructor.  In each case, the 
             *  12-bits of interest are right justified in the argument.  
             *  The lsb of the variables corresponds to stage 1 in figures
             *  3-3 through 3-6 of ICD-GPS-200.  The constructor runs the
             *  X generator process, creates 4096 bits of X-code, and stores
             *  the results internally in an array of 128 unsigned long 
             *  (32-bit) variables as packed integers.  Within the storage
             *  words the bits are ordered from word[0] to word[xxx] and from
             *  msb to lsb within the word. 
             * 
             *  initialEndOfSequence is the number of time the X-register 
             *  process should be incremented and should be either 4092 
             *  (X1A, X2A) or 4093 (X1B, X2B).
             *
             *  maxDelay represents the number of times the final bit of the
             *  sequence should be copied after initialEndOfSequence is reached.
             *  This should be equivalent (or greater than) the maximum number 
             *  of delay states that this variable can experience.  For X1B, 
             *  that's the end-of-X1A-epoch delay (349 counts).  For X2A and
             *  X2B the 37 chip end-of-X2-epoch and the end-of-week delays must
             *  be considered.
             */
         GenXSequence( const unsigned int initialState, 
                       const unsigned int taps,
                       const          int initialEndOfSequence,
                       const          int max_delay );
         ~GenXSequence( ) {};
         
         /**
          *  Fetch 32-bits from the sequence starting at the specified bit
          *  location.  Bits are counted started at 0 and run to 
          *  initialEndOfSequence+maxDelay.  Attempts to reference bits outside
          *  this range will generate exceptions and halt the program.  These 
          *  conditions shouldn't occur during operation, only during 
          *  programming.  If there are not 32 bits remaining prior to the 
          *  current value for lengthOfSequence, the function will wrap around
          *  to the beginning of sequence.
          */
         uint32_t operator[] ( int i );
         
         ///  Set the end of sequence for the current cycle.  
         void setLengthOfSequence( int i );
      
      private:
         void addBitToSequence( uint32_t newBit );
         uint32_t bits[MAX_WORD];
         int lengthOfSequence;
         int maxOfSequence;
         bool debugPrint;
         int word_num;
         int bit_num;
   };
   //@}
}     // end of namespace
#endif // GENXSEQUENCE_HPP
