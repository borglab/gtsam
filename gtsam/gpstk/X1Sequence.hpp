#pragma ident "$Id$"


// X1Sequence.cpp - GPS X1 Seqeuncer

#ifndef X1SEQUENCE_HPP
#define X1SEQUENCE_HPP

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
#include "gpstkplatform.h"
#include "PCodeConst.hpp"

namespace gpstk
{
/** @addtogroup code   */
//@{
      /**
        *     GPS X1 Sequencer.
        *     Applied Research Laboratories, The University of Texas at Austin
        *     August 2003
        *
        *  X1Sequence is contains
        *  a six-second (four Z-count) sequence of the combined X1A/X1B data
        *  for the GPS constellation.  The X1 sequence repeats every 1.5s 
        *  (each X1 epoch) and is identical for all SVs and all 1.5s epochs.
        *  In this case, 6 seconds of bitstream is generated and stored
        *  because this set of code "thinks" in term of 32-bit words and
        *  a 1.5s epoch doesn't contain an integer number of 32-bit words
        *  of packed bits) - there's a .25 word (8 bit) remainder.  
        *  Therefore, a six second seqeunce DOES include an even mulitple
        *  of 32 bits, which greatly simplifies handling of the data.
        *
        *  Six seconds of X1 bits is a significant amount of data:
        *    X1 Epoch = 4092 X1A bits * 3750 X1A cycles = 15,345,000 bits
        *    6 s = 4 X1 Epoch = 4 * bits = 4 * 15,345,000 = 61,380,000 bits.              
        *    The number of 32 bit words required equals
        *    61,380,000 bits / 32 bits/word = 1,918,125 words
        */
   class X1Sequence
   {
      public:
            /**
             *  Initialize the member variables associated with this object. 
             *  In the case of this class, this is a significant amount of 
             *  work. The X1A/X1B process described in ICD-GPS-200B is followed
             *  in order to fill the X1Bits array.
             */
         X1Sequence();
         ~X1Sequence( ) {};

            /**  The X1 sequence requires a 6-second buffer of 10MBit/sec 
             *   samples.  This comes to approximately 2 million four-byte
             *   unsigned integers.  These data are the same for all PRN codes
             *   To minimize the memory footprint, these data are stored
             *   in a dynamically-allocated static array.  It is 
             *     - - - - NECESSARY - - - -
             *   that the calling method call X1Sequence::allocateMemory()
             *   PRIOR to instantiating the first X1Sequence object. 
             *   X1Sequence::allocateMemory() should only be called once.
             *   Violation of either condition will result in a 
             *   gpstk::Exception thrown from either X1Sequence::X1Sequence()
             *   or X1Sequence::allocateMemory().
             *
             *   The X1Sequence::deAllocateMemory() method may be called to
             *   release the memory (if desired) but it should only be called
             *   after all X1Sequence objects have been "destroyed".
             */
         static void allocateMemory( );
         static void deAllocateMemory( );
         
         uint32_t & operator[]( int i );
            /**
             *  Given a word number from 0 to NUM_6SEC_WORDS, return the 
             *  requested word.
             */
         const uint32_t & operator[] ( int i ) const;
     
      private:
         static uint32_t* X1Bits;
         static bool isInit; 
   };

   inline uint32_t & X1Sequence::operator[] ( int i )
   {
      return(X1Bits[i]); 
   }

   inline const uint32_t & X1Sequence::operator[] ( int i ) const
   {
      return(X1Bits[i]);
   }

}  // end of namespace
#endif // X1SEQUENCE_HPP
