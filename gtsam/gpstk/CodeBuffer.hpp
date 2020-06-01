#pragma ident "$Id$"


//  CodeBuffer.hpp  

#ifndef CODEBUFFER_HPP
#define CODEBUFFER_HPP

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






   // Library headers
#include "CommonTime.hpp"

   // Project headers
#include "PCodeConst.hpp"

namespace gpstk
{
/** @addtogroup code    */
//@{
      /** P(Y)-code Buffer class.
       *  Applied Research Laboratories, The University of Texas at Austin
       *  August 2003
       *
       *  CodeBuffer is a helper class designed to store P-code information.
       *  Six seconds of code for a particular satellite is stored in each
       *  object.  The satellite is identified by PRNID and the beginning
       *  time is specified in a CommonTime object.  The code is stored in an
       *  array of unsigned long (assumed 32-bit) integers.  The time order 
       *  started with the MSB of first word of the buffer ([0]) and runs 
       *  through the LSB of the last word of the buffer.
       *
       *  The size of the buffer is probably the most notable feature of 
       *  this class. 1.5s (one Z-count) of P(Y)-code is 15,345,000 bits.  
       *  Therefore, 6 seconds is equal to 4 Z-counts or 4 * 15,345,000 bits
       *  = 61,380,000 bits.  These bits are stored in 1,918,125 unsigned 
       *  long integers.  The fact that 61,380,000 is evenly divisble by 32
       *  is not a coincidence, but part of the design.  The constant 
       *  NUM_6SEC_WORDS is used to hold the value 1,918,125 and located in
       *  PCodeConst.h.
       */
   class CodeBuffer
   {
      public:
           /// Defines the type of code held in the buffer 
         //enum codeType { P_CODE, Y_CODE, BOTH }; 
      
            /**
             * Instantiate and initialize a code buffer for a particular
             * satellite.
             */
         CodeBuffer( const int SVPRNID );
         ~CodeBuffer( );
         CodeBuffer& operator=( const CodeBuffer& c );
         
            /**
             * Update the time associated with the buffer and the designation
             * of the data contained in the buffer as P-code or Y-code.
             */
         void updateBufferStatus( const gpstk::CommonTime& dt, 
                                  const codeType PYFlag );

            /**
             * Update the time associated with the buffer and the designation
             * of the data contained in the buffer as P-code or Y-code.  In this
             * case, the time is not being changed, but the code flag may be 
             * changed.
             */
         void updateBufferStatus( const codeType PYFlag ) { POrYCode = PYFlag; }
         
            /// Accessor returning the current time.
         const gpstk::CommonTime& getCurrentTime( ) { return(currentTime); }
         
            /// Accessor returning the type of code in the buffer (P or Y)
         codeType getPYFlag( ) { return(POrYCode); } 
         
            /// Accessor returing the PRN ID of the buffer
         int getPRNID( ) { return(PRNID); }
            
            /// Set or return the designated word of the code buffer.
         unsigned long& operator[]( int i );
         const unsigned long& operator[]( const int i ) const;
         
            /** Given a bit number between 0 and (NUM_6SEC_WORDS * MAX_BIT) - 1,
             * return the value of the bit as a right justified unsigned 
             * long word (0x00000000 or 0x00000001)..
             */
         unsigned long getBit( const long i ) const;
         
            /** Perform an exclusive-or operation on the bits contained in 
             * this instance of CodeBuffer and the instance referenced by cb.
             */
         CodeBuffer& operator^=( const CodeBuffer& cb );
     
      protected:  
         CodeBuffer( const CodeBuffer& c );
         unsigned long * buffer;
         int PRNID;
         gpstk::CommonTime currentTime;
         codeType POrYCode;
   };

   inline  CodeBuffer::~CodeBuffer( ) { delete [] buffer; }
   inline CodeBuffer& CodeBuffer::operator^=( const CodeBuffer& cb )
   { 
      for (long i=0;i<NUM_6SEC_WORDS;++i) buffer[i] ^= cb.buffer[i];
      return( *this );
   }

   inline unsigned long& CodeBuffer::operator[]( int i )
   {
      return(buffer[i]);
   }

   inline const unsigned long& CodeBuffer::operator[]( int i ) const
   {
      return(buffer[i]);
   }

   inline unsigned long CodeBuffer::getBit( const long i ) const
   {
      unsigned long iret;
      long bNdx = i / MAX_BIT;
      long bitNum = i - (bNdx * MAX_BIT);
      iret = buffer[bNdx];
   
      // Shift LEFT to clear off msbs
      iret <<= bitNum;
      // Then shift RIGHT to clear off lsbs
      iret >>= (MAX_BIT-1);
   
      return iret;
   }
   //@}
}     // end of namespace
#endif // CODEBUFFER_HPP
