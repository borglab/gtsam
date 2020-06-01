#pragma ident "$Id$"

/**
 * @file PackedNavBits.hpp
 * Engineering units navigation message abstraction.
 */

#ifndef GPSTK_PACKEDNAVBITS_HPP
#define GPSTK_PACKEDNAVBITS_HPP

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

#include <vector>
#include <cstddef>
#include "gpstkplatform.h" //#include <stdint.h>
#include <string>

#include "ObsID.hpp"
#include "SatID.hpp"
#include "CommonTime.hpp"
#include "Exception.hpp"

namespace gpstk
{
   /** @addtogroup ephemcalc */
   //@{

   class PackedNavBits
   {
   public:
      /// empty constructor
      PackedNavBits();

      /// explicit constructor
      PackedNavBits(const SatID& satSysArg, 
                    const ObsID& obsIDArg,
                    const CommonTime& transmitTimeArg);

      PackedNavBits(const PackedNavBits& right);             // Copy constructor
//      PackedNavBits& operator=(const PackedNavBits& right); // Copy assignment

      PackedNavBits* clone() const;

      void setSatID(const SatID& satSysArg);
      void setObsID(const ObsID& obsIDArg);
      void setTime(const CommonTime& transmitTimeArg);
      void clearBits();

         /* Returnst the satellite system ID for a particular SV */
      SatID getsatSys() const;

         /* Returns Observation type, Carrier, and Tracking Code */
      ObsID getobsID() const;

         /* Returns time of transmission from SV */
      CommonTime getTransmitTime() const;

         /* Returns the number of bits */
      size_t getNumBits() const;

         /* Output the contents of this class to the given stream. */
      void dump(std::ostream& s = std::cout) const throw();
      
         /***    UNPACKING FUNCTIONS *********************************/
         /* Unpack an unsigned long integer */
      unsigned long asUnsignedLong(const int startBit, 
                                   const int numBits, 
                                   const int scale ) const;

         /* Unpack a signed long integer */
      long asLong(const int startBit, 
                  const int numBits, 
                  const int scale ) const;

         /* Unpack an unsigned double */
      double asUnsignedDouble( const int startBit, 
                               const int numBits, 
                               const int power2) const;

         /* Unpack a signed double */
      double asSignedDouble( const int startBit, 
                             const int numBits, 
                             const int power2) const;

         /* Unpack a double with units of semicircles */
      double asDoubleSemiCircles( const int startBit, 
                                  const int numBits, 
                                  const int power2) const;
     
         /* Unpack a string */
      std::string asString(const int startBit, 
                           const int numChars) const;

         /***    PACKING FUNCTIONS *********************************/
         /* Pack an unsigned long integer */
      void addUnsignedLong( const unsigned long value, 
                            const int numBits,  
                            const int scale ) 
         throw(InvalidParameter);
        
         /* Pack a signed long integer */                     
      void addLong( const long value, 
                    const int numBits, 
                    const int scale )
         throw(InvalidParameter);

         /* Pack an unsigned double */
      void addUnsignedDouble( const double value, 
                              const int numBits, 
                              const int power2)
         throw(InvalidParameter);

         /* Pack a signed double */
      void addSignedDouble( const double value, 
                            const int numBits, 
                            const int power2)
         throw(InvalidParameter);

         /* Pack a double with units of semicircles */
      void addDoubleSemiCircles( const double radians, 
                                 const int numBits, 
                                 const int power2)
         throw(InvalidParameter);

         /**
          * Pack a string.
          * Characters in String limited to those defined in IS-GPS-200 Section 20.3.3.5.1.8
          * numChars represents number of chars (8 bits each) to add to PackedBits.
          * If numChars < length of String only, chars 1..numChars will be added.
          * If numChars > length of String, blanks will be added at the end. */
      void addString(const std::string String, 
                     const int numChars)
         throw(InvalidParameter);
   
         /*
          * Output the packed bits as a set of 32 bit
          * hex values, four per line, without any
          * additional information. 
          * Returns the number of bits in the object.
          */
      int outputPackedBits(std::ostream& s = std::cout, 
		           short numPerLine=4,
			   char delimiter = ' ' ) const;

         /*
          * Return true if all bits between start and end are identical
          * between this object and right.  Default is to compare all
          * bits.  
          *
          * This method allows comparison of the "unchanging" data in 
          * nav messages while avoiding the time tags.
          */
      bool matchBits(const PackedNavBits& right, 
                     short startBit=1, short endBit=-1) const;
      
         /* Resize the vector holding the packed data. */
      void trimsize();

         /**
          * Raw bit input
          * This function is intended as a test-support function.
          * It assumes a string of the form
          *    ###  0xABCDABCD 0xABCDABCD 0xABCDABCD
          * where
          *    ### is the number of bits to expect in the remainder
          *        of the line.
          *    0xABCDABCD are each 32-bit unsigned hex numbers, left 
          *        justified.  The number of bits needs to match or
          *        exceed ###
          * The function returns if the read is succeessful.
          * Otherwise,the function throws an exception */
       void rawBitInput(const std::string inString )
          throw(InvalidParameter);       

   private:
      SatID satSys;            /**< System ID (based on RINEX defintions */
      ObsID obsID;             /**< Defines carrier and code tracked */
      CommonTime transmitTime; /**< Time nav message is trasnmitted */
      std::vector<bool> bits;  /**< Holds the packed data */
      int bits_used;

         /** Unpack the bits */
      uint64_t asUint64_t(const int startBit, const int numBits ) const 
         throw(InvalidParameter);

         /** Pack the bits */
      void addUint64_t( const uint64_t value, const int numBits );

         /** Extend the sign bit for signed values */
      int64_t SignExtend( const int startBit, const int numBits ) const;
   
         /** Scales doubles by their corresponding scale factor */
      double ScaleValue( const double value, const int power2) const;

   }; // class PackedNavBits

   //@}
   std::ostream& operator<<(std::ostream& s, const PackedNavBits& pnb);

} // namespace

#endif
