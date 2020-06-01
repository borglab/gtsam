#pragma ident "$Id$"



/**
 * @file BinUtils.hpp
 * Binary manipulation functions
 */

#ifndef BINUTILS_HPP
#define BINUTILS_HPP

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

#ifdef __sun
#include <arpa/nameser_compat.h>
#elif defined (__CYGWIN__)
#include <sys/param.h>
#elif defined (_AIX)
#include <sys/machine.h>
#endif

#include "Exception.hpp"

#ifndef BYTE_ORDER
#ifdef _MSC_VER
#define LITTLE_ENDIAN 1
#define BIG_ENDIAN 0
#define BYTE_ORDER LITTLE_ENDIAN
#else
#ifndef SWIG
#error "Platform doesn't support BYTE_ORDER?"
#endif
#endif
#endif

namespace gpstk
{
      /**
       * Binary data utilities.
       *
       * These functions and macros are to be used for manipulating
       * binary data.
       */
   namespace BinUtils
   {
         /** @defgroup binutilsgroup Binary Data Manipulation Tools */
         //@{
      
         /**
          * Reverse bytes.
          * This function will reverse the bytes in any type, though it
          * is typically meant to be used in atomic types like int and
          * double.
          * @param p object whose bytes are to be reversed.
          */
      template <class T> void twiddle(T& p)
         throw()
      {
         unsigned char *front = (unsigned char*)&p;
         unsigned char *end = front + sizeof(p) - 1;
         unsigned char temp;

         while (front<end)
         {
            temp = *front;
            *front = *end;
            *end = temp;
            front++;
            end--;
         }
      }

         /**
          * Converts Intel little-endian to host byte order, const version.
          * @param p the object whose bytes are to be modified.
          * @return a new object which is in Intel byte ordering.
          */
      template <class T> T intelToHost(const T& p)
         throw()
      {
         T temp(p);
#if BYTE_ORDER == BIG_ENDIAN
         twiddle(temp);
#endif
         return temp;
      }

         /** 
          * Converts host byte order to Intel little-endian, const version
          * @param p the object whose bytes are to be modified
          * @return a new object which is in host byte ordering.
          */
      template <class T> T hostToIntel(const T& p)
         throw()
      {
         T temp(p);
#if BYTE_ORDER == BIG_ENDIAN
         twiddle(temp);
#endif
         return temp;
      }      
      
         /**
          * Converts host byte order to network order, const version.
          * @param p the object whose bytes are to be modified.
          * @return a new object which is in network byte order.
          */
      template <class T> T netToHost(const T& p)
         throw()
      {
         T temp(p);
#if BYTE_ORDER == LITTLE_ENDIAN
         twiddle(temp);
#endif
         return temp;
      }

         /**
          * Converts network byte order to host order, const version.
          * @param p the object whose bytes are to be modified.
          * @return a new object which is in host byte order.
          */
      template <class T> T hostToNet(const T& p)
         throw()
      {
         T temp(p);
#if BYTE_ORDER == LITTLE_ENDIAN
         twiddle(temp);
#endif
         return temp;
      }

         /** 
          * Remove (optinally) the item specified from the string and convert it
          * from network byte order to host byte order.
          * @param str the string from which to obtain data.
          * @param pos an offset into the string to pull the data from. If this
          * value is specified, the item is not removed from the string.
          * @warn This function does not check for appropriate string length.
          */
      template <class T>
      T decodeVar( std::string& str, std::string::size_type pos = std::string::npos)
      {
         T t;
         char *cp = reinterpret_cast<char*>( &t );

         if (pos == std::string::npos)
         {
            str.copy( cp, sizeof(T) );
            t = gpstk::BinUtils::netToHost( t );
            str.erase( 0, sizeof(T) );
         }
         else
         {
            str.copy( cp, sizeof(T) , pos);
            t = gpstk::BinUtils::netToHost( t );
         }
         return t;
      }

         /** 
          * Add the network ordered binary representation of a var to the
          * the given string.
          * @param v the object of type T to convert to a string.
          */
      template<class T>
      std::string encodeVar( const T& v )
      {
         T tmp = v;
         tmp = gpstk::BinUtils::hostToNet( v );
         return std::string( reinterpret_cast<char*>( &tmp ), sizeof( tmp ) );
      }
      
         /// This is thrown when there is an error processing a CRC
         /// @ingroup exceptiongroup
      NEW_EXCEPTION_CLASS(CRCException, Exception);

         /// Reflects the lower \a bitnum bits of \a crc
      inline unsigned long reflect (unsigned long crc, 
                                    int bitnum)
      {
         unsigned long i, j = 1, crcout = 0;

         for (i = (unsigned long)1 << (bitnum - 1); i; i >>= 1)
         {
            if (crc & i)
            {
               crcout |= j;
            }
            j <<= 1;
         }
         return (crcout);
      }

         /// Encapsulate parameters for CRC computation
      class CRCParam
      {
      public:
         /// Constructor
         CRCParam(int o, 
                  unsigned long p, 
                  unsigned long i, 
                  unsigned long f,
                  bool d, 
                  bool ri, 
                  bool ro)
               : order(o), polynom(p), initial(i), final(f), direct(d),
                 refin(ri), refout(ro)
         {}

         int order;              ///< CRC polynomial order (without leading '1' bit).
         unsigned long polynom;  ///< CRC polynomial without the leading '1' bit.
         unsigned long initial;  ///< initial CRC initial value.
         unsigned long final;    ///< final final XOR value.
         bool direct;            ///< kind of algorithm, true = no augmented zero bits.
         bool refin;             ///< reflect the data bytes before processing.
         bool refout;            ///< reflect the CRC result before final XOR.
      };

      extern const CRCParam CRCCCITT;
      extern const CRCParam CRC16;
      extern const CRCParam CRC32;
      extern const CRCParam CRC24Q;


         /**
          * Compute CRC (suitable for polynomial orders from 1 to 32).
          * Does bit-by-bit computation (brute-force, no look-up
          * tables).  Default parameters are for CRC16.
          * \p
          * The following table lists parameters for common CRC
          * algorithms (order is decimal, the other parameters are
          * hex):
          * \li CRC-CCITT order=16 polynom=1021 initial=ffff final=0 direct=true refin=false refout=false
          * \li CRC-16 order=16 polynom=8005 initial=0 final=0 direct=true refin=true refout=true
          * \li CRC-32 order=32 polynom=4c11db7 initial=ffffffff final=ffffffff direct=true refin=true refout=true
          * @param data data to process CRC on.
          * @param len length of data to process.
          * @param params see documentation of CRCParam:w
          * @return the CRC value
          */
         // This code "stolen" from Sven Reifegerste (zorci@gmx.de).
         // Found at http://rcswww.urz.tu-dresden.de/~sr21/crctester.c
         // from link at http://rcswww.urz.tu-dresden.de/~sr21/crc.html
      inline unsigned long computeCRC(const unsigned char *data,
                                      unsigned long len,
                                      const CRCParam& params)
      {
         unsigned long i, j, c, bit;
         unsigned long crc = params.initial;

            // at first, compute constant bit masks for whole CRC and
            // CRC high bit
         unsigned long crcmask = 
            ((((unsigned long)1 << (params.order - 1)) - 1) << 1) | 1;
         unsigned long crchighbit = (unsigned long)1 << (params.order - 1);

         if (crc && params.direct)
         {
            for (i = 0; i < (unsigned long)params.order; i++)
            {
               bit = crc & 1;
               if (bit)
               {
                  crc ^= params.polynom;
               }
               crc >>= 1;
               if (bit)
               {
                  crc |= crchighbit;
               }
            }
         }


         for (i = 0; i < len; i++)
         {
            c = (unsigned long) * data++;
            if (params.refin)
            {
               c = reflect(c, 8);
            }

            for (j = 0x80; j; j >>= 1)
            {
               bit = crc & crchighbit;
               crc <<= 1;
               if (c & j)
               {
                  crc |= 1;
               }
               if (bit)
               {
                  crc ^= params.polynom;
               }
            }
         }

         for (i = 0; i < (unsigned long)params.order; i++)
         {
            bit = crc & crchighbit;
            crc <<= 1;
            if (bit)
            {
               crc ^= params.polynom;
            }
         }

         if (params.refout)
         {
            crc = reflect(crc, params.order);
         }
         crc ^= params.final;
         crc &= crcmask;

         return crc;
      }

         /**
          * Calculate an Exclusive-OR Checksum on the string /a str.
          * @return the calculated checksum.
          * @throws gpstk::InvalidParameter if there is a partial word at 
          *  the end of /a str.
          */
      template<class X>
      X xorChecksum(const std::string& str)
         throw(gpstk::InvalidParameter)
      {
         short wordSize = sizeof(X);
         short strSize = str.size();
         
         if(strSize % wordSize != 0)
         {
            gpstk::Exception ip("Incomplete word in string.");
            GPSTK_THROW(ip);
         }
         
         X temp, xc = 0;
         
         for(short i = 0; (i + wordSize - 1) < strSize; i += wordSize)
         {
            memcpy(&temp, &str[i], wordSize);
            xc ^= temp;
         }
         
         return xc;
      }
         //@}
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-function"
       
         /**
          * Count the set bits in an 32-bit unsigned integer.
          * Originated due to need in EngNav::checkParity
          */
      static unsigned short countBits(uint32_t v)
      {
            // Stolen from http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
         uint32_t c; // store the total here
         const int S[] = {1, 2, 4, 8, 16}; // Magic Binary Numbers
         const uint32_t B[] = {0x55555555, 0x33333333, 0x0F0F0F0F, 0x00FF00FF,
                               0x0000FFFF};

            // ...and if we were to turn this into a loop, it would
            // totally defeat the purpose.  The point here is to be
            // FAST.
         c = v;
         c = ((c >> S[0]) & B[0]) + (c & B[0]);
         c = ((c >> S[1]) & B[1]) + (c & B[1]);
         c = ((c >> S[2]) & B[2]) + (c & B[2]);
         c = ((c >> S[3]) & B[3]) + (c & B[3]);
         c = ((c >> S[4]) & B[4]) + (c & B[4]);

         return c;
      }
#pragma clang diagnostic pop
   } // end namespace BinUtils
} // end namespace gpstk

#endif
