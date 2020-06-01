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
//  Copyright 2008, The University of Texas at Austin
//
//============================================================================

/**
 * @file BinexData.cpp
 * Encapsulate BINEX file data, including I/O
 */

#include <cstring>
 
#include "BinexData.hpp"
#include "BinexStream.hpp"

using namespace std;

namespace gpstk
{
   // =========================================================================
   // BinexData::UBNXI Methods 
   // =========================================================================

   // -------------------------------------------------------------------------
   BinexData::UBNXI::UBNXI()
   {
      value = 0;
      size  = 1;
   }

   // -------------------------------------------------------------------------
   BinexData::UBNXI::UBNXI(unsigned long ul)
      throw(FFStreamError)
   {
      if (ul < 128)
      {
         size = 1;
      }
      else if (ul < 16384)
      {
         size = 2;
      }
      else if (ul < 2097152)
      {
         size = 3;
      }
      else if (ul < 536870912)
      {
         size = 4;
      }
      else
      {
         value = 0;
         size  = 0;

         std::ostringstream errStrm;
         errStrm << "BINEX UBNXI overflow: " << ul;
         FFStreamError err(errStrm.str() );
         GPSTK_THROW(err);
      }

      value = ul;
   }


   // -------------------------------------------------------------------------
   size_t
   BinexData::UBNXI::decode(
      const std::string&  inBuffer,
      size_t              offset,
      bool                littleEndian)
         throw(FFStreamError)
   {
      if (offset > inBuffer.size() )
      {
         std::ostringstream errStrm;
         errStrm << "Invalid offset into BINEX UBNXI input buffer: " << offset;
         FFStreamError err(errStrm.str() );
         GPSTK_THROW(err);
      }
      bool more = true;
      for (size = 0, value = 0L; (size < MAX_BYTES) && more; size++)
      {
         unsigned char mask = (size < 3) ? 0x7f : 0xff;
         if (littleEndian)
         {
            value |= ( (unsigned long)inBuffer[offset + size] & mask) << (7 * size);
         }
         else
         {
            value <<= (size < 3) ? 7 : 8;
            value |= ( (unsigned long)inBuffer[offset + size] & mask);
         }
         if ( (inBuffer[offset + size] & 0x80) != 0x80)
         {
            more = false;
         }
      }
      return size;
   }


   // -------------------------------------------------------------------------
   size_t
   BinexData::UBNXI::encode(
      std::string&  outBuffer,
      size_t        offset,
      bool          littleEndian) const
   {
      unsigned char  uc;
      unsigned short us, us1;
      unsigned long  ul, ul1, ul2, ul3;

      if (offset > outBuffer.size() )
      {
         std::ostringstream errStrm;
         errStrm << "Invalid offset into BINEX UBNXI output buffer: " << offset;
         FFStreamError err(errStrm.str() );
         GPSTK_THROW(err);
      }
      switch (size)
      {
         case 1:
            uc   = (unsigned char)value;
            outBuffer.replace(offset, size, reinterpret_cast<char*>(&uc), size);
            break;

         case 2:
            us   = (unsigned short)value;
            us1  = 0x7f00 & (us << 1);
            us  &= 0x007f;
            us  |= us1;

            if (littleEndian != nativeLittleEndian)
            {
                  // Allow encoding to non-native byte ordering
               reverseBuffer((unsigned char*)&us, 2);
            }
            us  |= nativeLittleEndian ? 0x0080 : 0x8000;

            outBuffer.replace(offset, size, reinterpret_cast<char*>(&us), size);
            break;

         case 3:
            ul   = (unsigned long)value;
            ul2  = 0x007f0000 & (ul << 2);
            ul1  = 0x00007f00 & (ul << 1);
            ul  &= 0x0000007f;
            ul  |= ul1 | ul2;
            
            if (littleEndian != nativeLittleEndian)
            {
                  // Allow encoding to non-native byte ordering
               reverseBuffer((unsigned char*)&ul, 4);
               ul >>= 8;
            }
            if (nativeLittleEndian)
            {
               ul  |= 0x00008080;
            }
            else
            {
               ul  |= 0x00808000;
               ul <<= 8;
            }

            outBuffer.replace(offset, size, reinterpret_cast<char*>(&ul), size);
            break;

         case 4:
            if (littleEndian)
            {
               ul   = (unsigned long)value;
               ul3  = 0xff000000 & (ul << 3);
               ul2  = 0x007f0000 & (ul << 2);
               ul1  = 0x00007f00 & (ul << 1);
               ul  &= 0x0000007f;
               ul  |= ul1 | ul2 | ul3;

               if (!nativeLittleEndian)
               {
                     // Allow encoding to non-native byte ordering
                  reverseBuffer((unsigned char*)&ul, 4);
               }
            }
            else
            {
               ul   = (unsigned long)value;
               ul2  = 0x7f000000 & (ul << 2);
               ul1  = 0x007f0000 & (ul << 1);
               ul  &= 0x00007fff;
               ul  |= ul1 | ul2;
   
               if (nativeLittleEndian)
               {
                     // Allow encoding to non-native byte ordering
                  reverseBuffer((unsigned char*)&ul, 4);
               }
            }
            ul  |= nativeLittleEndian ? 0x00808080 : 0x80808000;

            outBuffer.replace(offset, size, reinterpret_cast<char*>(&ul), size);
            break;

         default:
            std::ostringstream errStrm;
            errStrm << "BINEX UBNXI invalid size: " << size;
            FFStreamError err(errStrm.str() );
            GPSTK_THROW(err);
            break;
      }      
      return size;
   }


   // -------------------------------------------------------------------------
   size_t
   BinexData::UBNXI::read(
      istream&      strm,
      std::string   *outBuffer,
      size_t        offset,
      bool          reverseBytes,
      bool          littleEndian)
         throw(FFStreamError)
   {
      unsigned char mask = 0;
      char          buffer [MAX_BYTES];

      if (reverseBytes)
      {
         /// @TODO
      }
      else
      {
         bool more = true;
         for (size = 0, value = 0L; (size < MAX_BYTES) && more; size++)
         {
            mask = (size < 3) ? 0x7f : 0xff;
            
            strm.read(&buffer[size], 1);
            if (!strm.good() )
            {
               FFStreamError err("Error reading BINEX UBNXI");
               GPSTK_THROW(err);
            }            
            if (littleEndian)
            {
               value  |= ( (unsigned long)buffer[size] & mask) << (size * 7);
            }
            else
            {
               value <<= (size < 3) ? 7 : 8;
               value  |= (unsigned long)buffer[size] & mask;
            }
            if ( (buffer[size] & 0x80) != 0x80)
            {
               more = false;   // End of UBNXI detected - exit loop
            }
         }
         if (outBuffer)
         {
               // Store the as-read bytes
            if (offset <= outBuffer->size() )
            {
               outBuffer->replace(offset, size, buffer, size);
            }
            else
            {
               std::ostringstream errStrm;
               errStrm << "Invalid offset into BINEX UBNXI output buffer: " << offset;
               FFStreamError err(errStrm.str() );
               GPSTK_THROW(err);
            }
         }
      }
      return size;
   }


   // -------------------------------------------------------------------------
   size_t
   BinexData::UBNXI::write(
      ostream&       strm,
      std::string    *outBuffer,
      size_t         offset,
      bool           reverseBytes,
      bool           littleEndian) const
         throw(FFStreamError)
   {
      std::string  buffer;
      encode(buffer, 0, littleEndian);
      if (outBuffer)
      {
         if (offset <= outBuffer->size() )
         {
               // Store the as written bytes
            outBuffer->replace(offset, size, buffer);
         }
         else
         {
            std::ostringstream errStrm;
            errStrm << "Invalid offset into BINEX UBNXI output buffer: " << offset;
            FFStreamError err(errStrm.str() );
            GPSTK_THROW(err);
         }
      }
      if (reverseBytes)
      {
         reverseBuffer(buffer);
      }
      strm.write(buffer.data(), size);
      if (!strm.good() )
      {
         FFStreamError err("Error writing BINEX UBNXI");
         GPSTK_THROW(err);
      }
      return size;
   }

   // =========================================================================
   // BinexData::MGFZI Methods 
   // =========================================================================

   // -------------------------------------------------------------------------
   BinexData::MGFZI::MGFZI()
   {
      value = 0;
      size  = 1;
   }


   // -------------------------------------------------------------------------
   BinexData::MGFZI::MGFZI(long long ll)
      throw(FFStreamError)
   {

      value = ll;
      long long absValue = value;
      if (absValue < 0)
      {
         absValue = absValue*(-1);
      } 

      
      if (absValue < 16LL)
      {
         size = 1;
      }
      else if (absValue < 4110LL)
      {
         size = 2;
      }
      else if (absValue < 1052685LL)
      {
         size = 3;
      }
      else if (absValue < 269488140LL)
      {
         size = 4;
      }
      else if (absValue < 68988964875LL)
      {
         size = 5;
      }
      else if (absValue < 17661175009290LL)
      {
         size = 6;
      }
      else if (absValue < 4521260802379785LL)
      {
         size = 7;
      }
      else if (absValue <= BinexData::MGFZI::MAX_VALUE)
      {
         size = 8;
      }
      else
      {
         value = 0LL;
         size  = 0;

         std::ostringstream errStrm;
         errStrm << "BINEX MGFZI overflow: " << ll;
         FFStreamError err(errStrm.str() );
         GPSTK_THROW(err);
      }
   }


   // -------------------------------------------------------------------------
   size_t
   BinexData::MGFZI::decode(
      const std::string&  inBuffer,
      size_t              offset,
      bool                littleEndian)
         throw(FFStreamError)
   {
      long long          absValue = 0;
      unsigned char      flags;
      unsigned long long ull;
      short              sign;

      if (offset > inBuffer.size() )
      {
         std::ostringstream errStrm;
         errStrm << "Invalid offset into BINEX MGFZI input buffer: " << offset;
         FFStreamError err(errStrm.str() );
         GPSTK_THROW(err);
      }

      if (inBuffer.size() == 0)
      {
            // Nothing to decode
         size  = 0;
         value = 0;
         return 0;
      }
         // Isolate sign and byte-length flags
      flags = littleEndian
            ? inBuffer[offset] & 0x0f
            : (inBuffer[offset] >> 4) & 0x0f;

         // Determine whether the final value is positive or negative.
      sign = (flags & 0x08) ? -1 : 1;

         // Handle varying byte lengths
      size = (flags & 0x07) + 1;
      if (size > inBuffer.size() )
      {
         std::ostringstream errStrm;
         errStrm << "BINEX MGFZI is too large for the supplied decode buffer: "
                 << "MGFZI size = " << size << " , buffer size = " << inBuffer.size();
         FFStreamError err(errStrm.str() );
         GPSTK_THROW(err);
      }

      switch (size)
      {
         case 0x01:
            // Use 1 byte:
            //
            ull = parseBuffer(inBuffer, offset, 1);
            absValue = littleEndian
                     ? ull >> 4
                     : ull & 0x0000000fULL;
            if (sign == -1 && absValue == 0)
            {
               // "-0" reserved for "no data" indicator
               size = 0;
               // TODO - throw
            }
            else
            {
               value = sign * absValue;
            }
            break;

         case 0x02:
            // Use 2 bytes:
            //
            ull = parseBuffer(inBuffer, offset, 2);
            if (littleEndian != nativeLittleEndian)
            {
               reverseBuffer( (unsigned char*)&ull, 8);
               ull >>= 48;
            }
            absValue = littleEndian
                     ? ull >> 4
                     : ull & 0x0000000000000fffULL;
            value = sign * (14ULL + absValue);
            break;

         case 0x03:
            // Use 3 bytes:
            //
            ull = parseBuffer(inBuffer, offset, 3);
            if (littleEndian != nativeLittleEndian)
            {
               reverseBuffer( (unsigned char*)&ull, 8);
               ull >>= 40;
            }
            absValue = littleEndian
                     ? ull >> 4
                     : ull & 0x00000000000fffffULL;
            value = sign * (4109ULL + absValue);
            break;

         case 0x04:
            // Use 4 bytes:
            //
            ull = parseBuffer(inBuffer, offset, 4);
            if (littleEndian != nativeLittleEndian)
            {
               reverseBuffer( (unsigned char*)&ull, 8);
               ull >>= 32;
            }
            absValue = littleEndian
                     ? ull >> 4
                     : ull & 0x000000000fffffffULL;
            value = sign * (1052684ULL + absValue);
            break;

         case 0x05:
            // Use 5 bytes:
            //
            ull = parseBuffer(inBuffer, offset, 5);
            if (littleEndian != nativeLittleEndian)
            {
               reverseBuffer( (unsigned char*)&ull, 8);
               ull >>= 24;
            }
            absValue = littleEndian
                     ? ull >> 4
                     : ull & 0x0000000fffffffffULL;
            value = sign * (269488139ULL + absValue);
            break;

         case 0x06:
            // Use 6 bytes:
            //
            ull = parseBuffer(inBuffer, offset, 6);
            if (littleEndian != nativeLittleEndian)
            {
               reverseBuffer( (unsigned char*)&ull, 8);
               ull >>= 16;
            }                  
            absValue = littleEndian
                     ? ull >> 4
                     : ull & 0x00000fffffffffffULL;
            value = sign * (68988964874ULL + absValue);
            break;

         case 0x07:
            // Use 7 bytes:
            //
            ull = parseBuffer(inBuffer, offset, 7);
            if (littleEndian != nativeLittleEndian)
            {
               reverseBuffer( (unsigned char*)&ull, 8);
               ull >>= 8;
            }
            absValue = littleEndian
                     ? ull >> 4
                     : ull & 0x000fffffffffffffULL;
            value = sign * (17661175009289ULL + absValue);
            break;

         case 0x08:
            // Use 8 bytes:
            //
            ull = parseBuffer(inBuffer, offset, 8);
            if (littleEndian != nativeLittleEndian)
            {
               reverseBuffer( (unsigned char*)&ull, 8);
            }
            absValue = littleEndian
                     ? ull >> 4
                     : ull & 0x0fffffffffffffffULL;
            value = sign * (4521260802379784ULL + absValue);
            break;

         default:
               // Internal error
            std::ostringstream errStrm;
            errStrm << "BINEX MGFZI invalid size: " << size;
            FFStreamError err(errStrm.str() );
            size  = 0;
            GPSTK_THROW(err);
            break;

      } // switch (flag)
      return size;
   }


   // -------------------------------------------------------------------------
   size_t
   BinexData::MGFZI::encode(
      std::string&  outBuffer,
      size_t        offset,
      bool          littleEndian) const
   {
      char               buffer[MAX_BYTES];
      unsigned char      uc;
      unsigned short     us;
      unsigned long      ul;
      unsigned long long ull;

      if (offset > outBuffer.size() )
      {
         std::ostringstream errStrm;
         errStrm << "Invalid offset into BINEX MGFZI output buffer: " << offset;
         FFStreamError err(errStrm.str() );
         GPSTK_THROW(err);
      }
      
      long long absValue = value;
      if (absValue < 0)
      {
         absValue = absValue*(-1);
      } 
      
      unsigned char  signBit  = (value < 0) ? 0x01 : 0x00;

      size_t result = size;  // Default
      switch (size)
      {
         case 1:
            if (!littleEndian)
            {
               uc = (signBit << 7) | (0x0f & (unsigned char)absValue);
            }
            else
            {
               uc = (signBit << 3) | (0xf0 & ( (unsigned char)absValue << 4) );
            }
            if (uc == 0x08)
            {
                  // A valid value has been rounded to "-0"; convert to "+0".
               uc = 0x00;
            }
            buffer[0] = uc;
            break;

         case 2:
            absValue -= 14LL;
            if (littleEndian)
            {
               us = ( (unsigned short)signBit <<  3) | (0xfff1
                  & (0x0001 | (unsigned short)absValue << 4));
            }
            else
            {
               us = ((unsigned short)signBit << 15) | (0x1fff
                  & (0x1000 | (unsigned short)absValue));
            }
            if (littleEndian != nativeLittleEndian)
            {
               reverseBuffer( (unsigned char*)&us, 2);
            }
            std::memcpy((void*)buffer, (const void*)&us, 2);
            break;

         case 3:
            absValue -= 4109LL;   
            if (littleEndian)
            {
               ul = ((unsigned long)signBit << 3) | (0x00fffff2
                  & (0x00000002 | (unsigned long)absValue << 4));
            }
            else
            {
               ul = ((unsigned long)signBit << 31) | (0x2fffff00
                  & (0x20000000 | (unsigned long)absValue << 8));
            }
            if (littleEndian != nativeLittleEndian)
            {
               reverseBuffer( (unsigned char*)&ul, 4);
            }
            std::memcpy((void*)buffer, (const void*)&ul, 3);
            break;

         case 4:
            absValue -= 1052684LL;
            if (littleEndian)
            {
               ul = ((unsigned long)signBit << 3) | (0xfffffff3
                  & (0x00000003 | (unsigned long)absValue << 4));
            }
            else
            {
               ul = ((unsigned long)signBit << 31) | (0x3fffffff
                  & (0x30000000 | (unsigned long)absValue));
            }
            if (littleEndian != nativeLittleEndian)
            {
               reverseBuffer( (unsigned char*)&ul, 4);
            }
            memcpy((void*)buffer, (const void*)&ul, 4);
            break;

         case 5:
            absValue -= 269488139LL;
            if (littleEndian)
            {
               ull = ((unsigned long long)signBit << 3) | (0x000000fffffffff4ULL
                  & (0x0000000000000004ULL | (unsigned long long)absValue << 4));
            }
            else
            {
               ull = ((unsigned long long)signBit << 63) | (0x4fffffffff000000ULL
                  & (0x4000000000000000ULL | (unsigned long long)absValue << 24));
            }
            if (littleEndian != nativeLittleEndian)
            {
               reverseBuffer( (unsigned char*)&ull, 8);
            }
            memcpy((void*)buffer, (const void*)&ull, 5);
            break;

         case 6:
            absValue -= 68988964874LL;
            if (littleEndian)
            {
               ull = ((unsigned long long)signBit << 3) | (0x0000fffffffffff5ULL
                  & (0x0000000000000005ULL| (unsigned long long)absValue << 4));
            }
            else
            {
               ull = ((unsigned long long)signBit << 63) | (0x5fffffffffff0000ULL
                  & (0x5000000000000000ULL | (unsigned long long)absValue << 16));
            }
            if (littleEndian != nativeLittleEndian)
            {
               reverseBuffer( (unsigned char*)&ull, 8);
            }
            memcpy((void*)buffer, (const void*)&ull, 6);
            break;

         case 7:
            absValue -= 17661175009289LL;
            if (littleEndian)
            {
               ull = ((unsigned long long)signBit << 3) | (0x00fffffffffffff6ULL
                  & (0x0000000000000006ULL | (unsigned long long)absValue << 4));
            }
            else
            {
               ull = ((unsigned long long)signBit << 63) | (0x6fffffffffffff00ULL
                  & (0x6000000000000000ULL | (unsigned long long)absValue << 8));
            }
            if (littleEndian != nativeLittleEndian)
            {
               reverseBuffer( (unsigned char*)&ull, 8);
            }
            memcpy((void*)buffer, (const void*)&ull, 7);
            break;

         case 8:
            absValue -= 4521260802379784LL;
            if (littleEndian)
            {
               ull = ((unsigned long long)signBit << 3) | (0xfffffffffffffff7ULL
                  & (0x0000000000000007ULL | (unsigned long long)absValue << 4));
            }
            else
            {
               ull = ((unsigned long long)signBit << 63) | (0x7fffffffffffffffULL
                  & (0x7000000000000000ULL | (unsigned long long)absValue));
            }
            if (littleEndian != nativeLittleEndian)
            {
               reverseBuffer( (unsigned char*)&ull, 8);
            }
            memcpy((void*)buffer, (const void*)&ull, 8);
            break;

         default:
               // If the byte count is 0, store "no value"
            result    = 1;
            unsigned char  nv;
            nv = 0x80;
            buffer[0] = nv;

      } // switch
      outBuffer.replace(offset, size, buffer, size);
      return result;
   }


   // -------------------------------------------------------------------------
   size_t
   BinexData::MGFZI::read(
      istream&      strm,
      std::string   *outBuffer,
      size_t        offset,
      bool          reverseBytes,
      bool          littleEndian)
         throw(FFStreamError)
   {
      unsigned char  buffer   [MAX_BYTES];
      unsigned char  flags;
      short          sign;

         // Read the flags byte
      strm.read((char*)&buffer[0], 1);

         // Isolate sign and byte-length flags
      flags = reverseBytes
            ? buffer[0] & 0x0f
            : (buffer[0] >> 4) & 0x0f;

         // Determine whether the final value is positive or negative
         // and determine the number of bytes comprising the MGFZI.
      sign = (flags & 0x08) ? -1 : 1;      
      size = (flags & 0x07) + 1;

      if (size > 1)
      {
         strm.read((char*)&buffer[1], size - 1);
         if (!strm.good() || strm.gcount() != size - 1)
         {
            FFStreamError err("Error reading BINEX MGFZI");
            GPSTK_THROW(err);
         }
      }
      if (reverseBytes)
      {
         reverseBuffer(buffer, size);
      }
      if (outBuffer)
      {
            // Store the as-read bytes
         if (offset <= outBuffer->size() )
         {
            outBuffer->replace(offset, size, (char*)&buffer[0], size);
         }
         else
         {
            std::ostringstream errStrm;
            errStrm << "Invalid offset into BINEX MGFZI output buffer: " << offset;
            FFStreamError err(errStrm.str() );
            GPSTK_THROW(err);
         }
      }

         // Decode the entire buffer
      std::string  tmp((const char*)&buffer[0], size);
      decode(tmp, size, littleEndian);

      return size;
   }


   // -------------------------------------------------------------------------
   size_t
   BinexData::MGFZI::write(
      ostream&      strm,
      std::string   *outBuffer,
      size_t        offset,
      bool          reverseBytes,
      bool          littleEndian) const
         throw(FFStreamError)
   {
      std::string  buffer;
      encode(buffer, 0, littleEndian);
      if (outBuffer)
      {
            // Store the as-written bytes
         //memcpy(outBuffer, buffer, bufferSize);
         if (offset <= outBuffer->size() )
         {
            outBuffer->replace(offset, size, buffer);
         }
         else
         {
            std::ostringstream errStrm;
            errStrm << "Invalid offset into BINEX MGFZI output buffer: " << offset;
            FFStreamError err(errStrm.str() );
            GPSTK_THROW(err);
         }
      }
      if (reverseBytes)
      {
         reverseBuffer(buffer);
      }
      strm.write(buffer.data(), size);
      if (!strm.good() )
      {
         FFStreamError err("Error writing BINEX MGFZI");
         GPSTK_THROW(err);             
      }
      return size;
   }


   // =========================================================================
   // BinexData Methods 
   // =========================================================================

   // -------------------------------------------------------------------------
   BinexData::BinexData()
   {
      setRecordFlags();  // Establish defaults
      recID = INVALID_RECORD_ID;
   }


   // -------------------------------------------------------------------------
   BinexData::BinexData(const BinexData& other)
   {
      *this = other;
   }


   // -------------------------------------------------------------------------
   BinexData::BinexData(unsigned long recordID,
                        unsigned char recordFlags)
      throw()
   {
      setRecordFlags(recordFlags);
      setRecordID(recordID);
   }


   // -------------------------------------------------------------------------
   BinexData&
   BinexData::operator=(const BinexData& right)
   {
      recID    = right.recID;
      syncByte = right.syncByte;
      msg      = right.msg;
      return *this;
   }

   // -------------------------------------------------------------------------
   bool BinexData::operator==(const BinexData& b) const
   {
      return (  (recID     == b.recID)
             && (syncByte  == b.syncByte)
             && (msg.compare(b.msg) == 0) );
   }


   // -------------------------------------------------------------------------
   BinexData&
   BinexData::setRecordID(unsigned long id)
      throw(FFStreamError)
   {
      if (id > UBNXI::MAX_VALUE)
      {
         FFStreamError err("BINEX record ID overflow");
         GPSTK_THROW(err);
      }
         
      recID = id;
      return *this;
      
   } // BinexData::setRecordID()


   // -------------------------------------------------------------------------
   BinexData&
   BinexData::setRecordFlags(unsigned char flags)
   {
         // Set all unsupported bits to 0.
      syncByte = flags & VALID_RECORD_FLAGS;
      
         // Set "regular CRC" bit (0x02) to the opposite of the enhanced CRC bit
         // and set the two leading always-on bits (0xC0).
      syncByte |= (flags & eEnhancedCRC) ? 0xC0 : 0xC2;
      
      return *this;
   }

   
   // -------------------------------------------------------------------------
   size_t
   BinexData::getRecordSize() const
   {
      size_t recSize = 1;  // Start with sync byte
      
      UBNXI r(recID);             // Add record ID
      recSize += r.getSize();        
      UBNXI m(msg.size() );       // Add message length
      recSize += m.getSize();        
      recSize += msg.size();      // Add message

      size_t crcLen = getCRCLength(recSize - 1);

      recSize += crcLen;          // Add CRC length

      if (syncByte & eReverseReadable)
      {
         UBNXI t(1 + r.getSize() + m.getSize() + msg.size() + crcLen);
         
         recSize += t.getSize();  // Add total reverse length
         
         recSize += 1;            // Add sync byte
      }

      return recSize;
   }


   // -------------------------------------------------------------------------
   BinexData&
   BinexData::clearMessage()
   {
      msg.resize(0);
      return *this;

   } // BinexData::clearMessage()

   // -------------------------------------------------------------------------
   BinexData&
   BinexData::ensureMessageCapacity(size_t cap)
      throw(FFStreamError)
   {
      if (cap > UBNXI::MAX_VALUE)
      {
         std::ostringstream errStrm;
         errStrm << "Requested message length exceeds allowable maximum: " << cap;
         InvalidParameter ip(errStrm.str() );
         GPSTK_THROW(ip);
      }
      msg.reserve(cap);
      return *this;

   }  // BinexData::ensureMessageCapacity()

   // -------------------------------------------------------------------------
   BinexData&
   BinexData::updateMessageData(
      size_t&      offset,
      const UBNXI& data)
         throw(FFStreamError, InvalidParameter)
   {
      bool   littleEndian  = ( (syncByte & eBigEndian) == 0) ? true : false;
      ensureMessageCapacity(offset + data.getSize() );
      offset += data.encode(msg, offset, littleEndian);
      return *this;

   }  // BinexData::updateMessageData()

   // -------------------------------------------------------------------------
   BinexData&
   BinexData::updateMessageData(
      size_t&      offset,
      const MGFZI& data)
         throw(FFStreamError, InvalidParameter)
   {
      bool   littleEndian  = ( (syncByte & eBigEndian) == 0) ? true : false;
      ensureMessageCapacity(offset + data.getSize() );
      offset += data.encode(msg, offset, littleEndian);
      return *this;

   }  // BinexData::updateMessageData()

   // -------------------------------------------------------------------------
   BinexData&
   BinexData::updateMessageData(
      size_t&            offset,
      const std::string& data,
      size_t             size)
         throw(FFStreamError, InvalidParameter)
   {
      ensureMessageCapacity(offset + size);
      if (size > data.size() )
      {
         std::ostringstream errStrm;
         errStrm << "Invalid data size: " << size;
         InvalidParameter ip(errStrm.str() );
         GPSTK_THROW(ip);
      }
      msg.replace(offset, size, data, 0, size);
      offset += size;
      return *this;

   }  // BinexData::updateMessageData()

   // -------------------------------------------------------------------------
   BinexData&
   BinexData::updateMessageData(
      size_t&     offset,
      const char  *data,
      size_t      size)
         throw(FFStreamError, InvalidParameter)
   {
      ensureMessageCapacity(offset + size);
      msg.replace(offset, size, data, size);
      offset += size;
      return *this;
   }

   // -------------------------------------------------------------------------
   void
   BinexData::extractMessageData(
      size_t& offset,
      UBNXI&  data)
         throw(FFStreamError, InvalidParameter)
   {
      if (offset > msg.size() )
      {
         std::ostringstream errStrm;
         errStrm << "Message buffer offset invalid: " << offset;
         InvalidParameter ip(errStrm.str() );
         GPSTK_THROW(ip);
      }
      bool littleEndian  = ( (syncByte & eBigEndian) == 0) ? true : false;
      offset += data.decode(msg, offset, littleEndian);

   }  // BinexData::extractMessageData()

   // -------------------------------------------------------------------------
   void
   BinexData::extractMessageData(
      size_t& offset,
      MGFZI&  data)
         throw(FFStreamError, InvalidParameter)
   {
      if (offset > msg.size() )
      {
         std::ostringstream errStrm;
         errStrm << "Message buffer offset invalid: " << offset;
         InvalidParameter ip(errStrm.str() );
         GPSTK_THROW(ip);
      }
      bool littleEndian  = ( (syncByte & eBigEndian) == 0) ? true : false;
      offset += data.decode(msg, offset, littleEndian);

   }  // BinexData::extractMessageData()

   // -------------------------------------------------------------------------
   void
   BinexData::extractMessageData(
      size_t&      offset,
      std::string& data,
      size_t       size) const
         throw(InvalidParameter)
   {
      if (offset + size > msg.size() )
      {
         std::ostringstream errStrm;
         errStrm << "Message buffer offset invalid: " << offset;
         InvalidParameter ip(errStrm.str() );
         GPSTK_THROW(ip);
      } 
      data.assign(msg, offset, size);
      offset += size;

   }  // BinexData::extractMessageData()

   // -------------------------------------------------------------------------
   void
   BinexData::reallyPutRecord(FFStream& ffs) const
      throw(std::exception, FFStreamError, 
            StringUtils::StringException)
   {
      //std::cout << "BinexData::reallyPutRecord: ENTER" << std::endl;

      BinexStream* strm = dynamic_cast<BinexStream*>(&ffs);
      if (NULL == strm)
      {
         FFStreamError e("Attempt to read a BinexData object"
                         " from a non-BinexStream FFStream.");
         GPSTK_THROW(e);
      }

      try
      {
         unsigned char syncTail;
         if (!isHeadSyncByteValid(syncByte, syncTail) )
         {
            std::ostringstream errStrm;
            errStrm << "Invalid BINEX synchronization byte: "
                    << static_cast<uint16_t>(syncByte);
            FFStreamError err(errStrm.str() );
            GPSTK_THROW(err);
         }
            // Buffer, etc. for non-msg data
         std::string    headBuf;
         size_t         bufLen = 0;  // Total buffer data length
         size_t         crcLen = 0;  // Length of calculated CRC

         bool    littleEndian = (syncByte&eBigEndian)==0 ? true : false;

         headBuf.reserve(9);  // 1+4+4

            // Copy sync byte to header buffer
         headBuf.append(1, syncByte);
         bufLen += 1;

            // Copy record ID to header buffer
         UBNXI r(recID);
         bufLen += r.encode(headBuf, bufLen, littleEndian);

            // Copy message length to header buffer
         UBNXI m(msg.size() );
         bufLen += m.encode(headBuf, bufLen, littleEndian);

            // Write header buffer and message to the output stream
         strm->write(headBuf.data(), headBuf.size() );
         strm->write(msg.data(), msg.size() );

            // Calculate CRC and store it in the tail buffer
         std::string  tailBuf;
         tailBuf.reserve(21);  // 16+4+1
         headBuf.erase(0, 1);  // Exclude sync byte from CRC calculation
         getCRC(headBuf, msg, tailBuf);
         crcLen = tailBuf.size();

            // Copy enhanced CRC record items into the tail buffer
         if (syncByte & eReverseReadable)
         {
            UBNXI recLen(1 + r.getSize() + m.getSize() + msg.size() + crcLen);
            recLen.encode(tailBuf, crcLen, littleEndian);
            reverseBuffer(tailBuf, crcLen);
            tailBuf.append(1, syncTail);
         }

            // Write the tail buffer to the output stream.
         strm->write(tailBuf.data(), tailBuf.size() );
      }
      catch(std::exception& exc)
      {
         FFStreamError err(exc.what());
         GPSTK_THROW(err);
      }
      catch(...)
      {
         FFStreamError err("Unknown exception");
         GPSTK_THROW(err);
      }

      if (strm->fail() || strm->bad())
      {
         FFStreamError err("Error writing data");
         GPSTK_THROW(err);
      }

      //std::cout << "BinexData::reallyPutRecord: EXIT" << std::endl;
   }

   // -------------------------------------------------------------------------
   void BinexData::reallyGetRecord(FFStream& ffs) 
      throw(std::exception, FFStreamError, StringUtils::StringException)
   {
      //std::cout << "BinexData::reallyGetRecord: ENTER" << std::endl;

      BinexStream* strm = dynamic_cast<BinexStream*>(&ffs);
      if (NULL == strm)
      {
         FFStreamError e("Attempt to read a BinexData object"
                                " from a non-BinexStream FFStream.");
         GPSTK_THROW(e);
      }

      size_t        offset            = 0;
      
      std::string   crcBuf;
      size_t        crcBufLen = 0;
      
      unsigned char expectedSyncByte;
      std::string   expectedCrc;
      unsigned char crc               [16];
      size_t        crcLen            = 0;
      unsigned char syncBuf;

      try
      {
         strm->read((char*)&syncBuf, 1);  // Read synchronization byte
      }
      catch (std::exception &e)
      {
         if ( (strm->gcount() == 0) && strm->eof() )
         {
               // Process as EOF
            EndOfFile err("EOF encountered");
            GPSTK_THROW(err);
         }
         else
         {
               // Rethrow if not EOF
            FFStreamError err(e.what() );
            GPSTK_THROW(err);
         }
      }

      try
      {
         if (isHeadSyncByteValid(syncBuf, expectedSyncByte) )
         {
            syncByte = syncBuf;

               // Process the BINEX record forward (not in reverse)
            bool littleEndian = (syncByte & eBigEndian) == 0 ? true : false;

            UBNXI uRecID;
            crcBufLen += uRecID.read(*strm, &crcBuf, crcBufLen, false, littleEndian);
            setRecordID((unsigned long)uRecID);

            UBNXI uMsgLen;
            crcBufLen += uMsgLen.read(*strm, &crcBuf, crcBufLen, false, littleEndian);

            unsigned long msgLen  = (unsigned long)uMsgLen;

               // Read directly into the memory buffer of a vector.  This is OK
               // because, unlike a string, a vector is stored contiguously.
               // The vector's memory will be reclaimed in all exit conditions
               // since the vector is allocated on the stack.
            std::vector<char>  msgBuf(msgLen);
            strm->read((char*)&msgBuf[0], msgLen);
            if (!strm->good() || (strm->gcount() != msgLen) )
            {
               FFStreamError err("Incomplete BINEX record message");
               GPSTK_THROW(err);
            }
            msg.assign((char*)&msgBuf[0], msgLen);

               // Check CRC - first calculate expected, then read actual,
               // then compare.
            getCRC(crcBuf, msg, expectedCrc);
            crcLen = expectedCrc.size();

            strm->read( (char*)crc, crcLen);
            if (!strm->good() || (strm->gcount() != crcLen) )
            {
               FFStreamError err("Error reading BINEX CRC");
               GPSTK_THROW(err);
            }
            if (std::memcmp(crc, expectedCrc.data(), crcLen) )
            {
               FFStreamError err("Bad BINEX CRC");
               GPSTK_THROW(err);
            }
         }
         else if (isTailSyncByteValid(syncBuf, expectedSyncByte) )
         {
               // Process the BINEX record in reverse
            bool littleEndian = (expectedSyncByte & eBigEndian) == 0 ? true : false;

            UBNXI r, m, b;
            b.read(*strm, NULL, 0, false, littleEndian);
            unsigned long revRecSize = (unsigned long)b;

               // Read the entire remainder of the record into memory
            std::vector<char>  revRecVec(revRecSize);
            strm->read( (char*)&revRecVec[0], revRecSize);
            if (!strm->good() || (strm->gcount() != revRecSize) )
            {
               FFStreamError err("Incomplete BINEX record message");
               GPSTK_THROW(err);
            }
            std::string revRecBuf( (char*)&revRecVec[0], revRecSize);
            reverseBuffer(revRecBuf, revRecSize);

            if (revRecBuf[0] != expectedSyncByte)
            {
               FFStreamError err("BINEX head/tail synchronization byte mismatch");
               GPSTK_THROW(err);
            }
            syncByte = expectedSyncByte;
            offset += 1;

            offset += r.decode(revRecBuf, offset, littleEndian);
            setRecordID((unsigned long)r);
            // @TODO - Check against revRecSize
            offset += m.decode(revRecBuf, offset, littleEndian);
            // @TODO - Check against revRecSize

            msg.assign(revRecBuf, offset, (unsigned long)m);
            offset += msg.size();

               // Check CRC - first calculate expected, then compare to actual.
            getCRC(revRecBuf.substr(1, r.getSize() + m.getSize() ), msg, expectedCrc);
            crcLen = expectedCrc.size();
            
            if ( (offset + crcLen != revRecSize)
               || expectedCrc.compare(revRecBuf.substr(offset, crcLen) ) )
            {
               FFStreamError err("Bad BINEX CRC");
               GPSTK_THROW(err);
            }
         }
         else
         {
            std::ostringstream errStrm;
            errStrm << "Invalid BINEX synchronization byte: "
                    << static_cast<uint16_t>(syncBuf);
            FFStreamError err(errStrm.str() );
            GPSTK_THROW(err);
         }

      }
      catch(FFStreamError& exc)
      {
         GPSTK_RETHROW(exc);
      }
      catch(std::exception& exc)
      {
         FFStreamError err(exc.what() );
         GPSTK_THROW(err);
      }
      catch(...)
      {
         FFStreamError err("Unknown exception");
         GPSTK_THROW(err);
      }

      //std::cout << "BinexData::reallyGetRecord: EXIT" << std::endl;
   }

   void
   BinexData::getCRC(const std::string&  head,
                     const std::string&  message,
                     std::string&        crc) const
   {
      size_t crcDataLen = head.size() + message.size();
      size_t crcLen     = 0;
      unsigned long crcTmp = 0;

      if (crcDataLen >= 1048576)
      {
            // @TODO - Use 16-byte CRC (128-bit MD5 checksum)
         crcLen  = 16;
      }
      else // (crcLen < 1048576)
      {
         if (syncByte & eEnhancedCRC)
         {
            if (crcDataLen < 128)
            {
                  // Use 2-byte CRC (CRC16)
               BinUtils::CRCParam params(BinUtils::CRC16);
               crcTmp = BinUtils::computeCRC((const unsigned char*)head.data(),
                                             head.size(),
                                             params);
               params.initial = crcTmp;
               crcTmp = BinUtils::computeCRC((const unsigned char*)message.data(),
                                             message.size(),
                                             params);
               crcLen = 2;
            }
            else
            {
                  // Use 4-byte CRC (CRC32)
               BinUtils::CRCParam params(BinUtils::CRC32);
               crcTmp = BinUtils::computeCRC((const unsigned char*)head.data(),
                                             head.size(),
                                             params);
               params.initial = crcTmp;
               crcTmp = BinUtils::computeCRC((const unsigned char*)message.data(),
                                             message.size(),
                                             params);
               crcLen = 4;
            }
         }
         else // Regular CRC
         {
            if (crcDataLen < 128)
            {
                  // Use 1-byte checksum: 8-bit XOR of all bytes
               size_t b;
               const char *ptr;
               ptr = head.data();
               for (b = head.size(); b > 0 ; b--, ptr++)
               {
                  crcTmp ^= *ptr;
               }
               ptr = message.data();
               for (b = message.size(); b > 0 ; b--, ptr++)
               {
                  crcTmp ^= *ptr;
               }
               crcLen = 1;
            }
            else if (crcDataLen < 4096)
            {
                  // Use 2-byte CRC (CRC16)
               BinUtils::CRCParam params(BinUtils::CRC16);
               crcTmp = BinUtils::computeCRC((const unsigned char*)head.data(),
                                             head.size(),
                                             params);
               params.initial = crcTmp;
               crcTmp = BinUtils::computeCRC((const unsigned char*)message.data(),
                                             message.size(),
                                             params);
               crcLen = 2;
            }
            else
            {
                  // Use 4-byte CRC (CRC32)
               BinUtils::CRCParam params(BinUtils::CRC32);
               crcTmp = BinUtils::computeCRC((const unsigned char*)head.data(),
                                             head.size(),
                                             params);
               params.initial = crcTmp;
               crcTmp = BinUtils::computeCRC((const unsigned char*)message.data(),
                                             message.size(),
                                             params);
               crcLen = 4;
            }
         } // Regular CRC
         
            // Copy the CRC into the output
         if (!nativeLittleEndian)
         {
            BinUtils::twiddle(crcTmp);
         }
         crc.assign( (const char*)&crcTmp, crcLen);

      } // (crcLen < 1048576)

   }  // BinexData::getCRC()

   // -------------------------------------------------------------------------
   size_t
   BinexData::getCRCLength(size_t crcDataLen) const
   {
      size_t crcLen = 0;

      if (crcDataLen >= 1048576)
      {
         crcLen  = 16;
      }
      else // (crcLen < 1048576)
      {
         if (syncByte & eEnhancedCRC)
         {
            if (crcDataLen < 128)
            {
               crcLen = 2;
            }
            else
            {
               crcLen = 4;
            }
         }
         else // Regular CRC
         {
            if (crcDataLen < 128)
            {
               crcLen = 1;
            }
            else if (crcDataLen < 4096)
            {
               crcLen = 2;
            }
            else
            {
               crcLen = 4;
            }

         } // Regular CRC
         
      } // (crcLen < 1048576)

      return crcLen;
   }

   // -------------------------------------------------------------------------
   bool
   BinexData::isHeadSyncByteValid(unsigned char  headSync,
                                  unsigned char& expectedTailSync) const
   {
      switch (headSync)
      {
         case 0xC2: // Fall through ...
         case 0xE2: // Fall through ...
         case 0xC8: // Fall through ...
         case 0xE8:
            expectedTailSync = 0x00; // No tail sync byte
            break;

         case 0xD2:
            expectedTailSync = 0xB4;
            break;

         case 0xF2:
            expectedTailSync = 0xB0;
            break;

         case 0xD8:
            expectedTailSync = 0xE4;
            break;

         case 0xF8:
            expectedTailSync = 0xE0;
            break;

         default:
            return false; // Invalid head synchronization byte
      }
      return true;
   }

   // -------------------------------------------------------------------------
   bool
   BinexData::isTailSyncByteValid(unsigned char  tailSync,
                                  unsigned char& expectedHeadSync) const
   {
      switch (tailSync)
      {
         case 0xB4:
            expectedHeadSync = 0xD2;
            break;

         case 0xB0:
            expectedHeadSync = 0xF2;
            break;

         case 0xE4:
            expectedHeadSync = 0xD8;
            break;

         case 0xE0:
            expectedHeadSync = 0xF8;
            break;

         default:
            return false; // Invalid tail synchronization byte
      }
      return true;
   }

   // -------------------------------------------------------------------------
   unsigned long long
   BinexData::parseBuffer(const std::string&  buffer,
                          size_t              offset,
                          size_t              size)
      throw(FFStreamError)
   {
      unsigned long long value = 0;
      if (size > sizeof(value) )
      {
         FFStreamError err("Invalid data size parsing BINEX data buffer");
         GPSTK_THROW(err);
      }
      if (offset + size > buffer.size() )
      {
         FFStreamError err("Invalid offset parsing BINEX data buffer");
         GPSTK_THROW(err);
      }
      char* valp = (char*)&value;
      for (size_t i = 0; i < size; i++, valp++)
      {
         *valp = buffer[offset + i];
      }
      if (!nativeLittleEndian)
      {
         value >>= ( (sizeof(value) - size) << 3);
      }
      return value;
   }

   // -------------------------------------------------------------------------
   void
   BinexData::reverseBuffer(unsigned char  *buffer,
                            size_t         bufferLength)
   {
      unsigned char tmp;
      for (size_t i = 1; i <= (bufferLength >> 1); i++)
      {
         tmp = buffer[i - 1];
         buffer[i - 1] = buffer[bufferLength - i];
         buffer[bufferLength - i] = tmp;
      }
   }

   // -------------------------------------------------------------------------
   void
   BinexData::reverseBuffer(std::string& buffer, size_t offset, size_t n)
   {
      if ( offset > buffer.size() )
      {
         FFStreamError err("Invalid offset reversing BINEX data buffer");
         GPSTK_THROW(err);
      }
      size_t back = (n == std::string::npos) ? buffer.size() : offset + n;
      if ( back > buffer.size() )
      {
         FFStreamError err("Invalid size reversing BINEX data buffer");
         GPSTK_THROW(err);
      }
      --back;
      unsigned char tmp;
      while (offset < back)
      {
         tmp            = buffer[offset];
         buffer[offset] = buffer[back];
         buffer[back]   = tmp;
         ++offset;
         --back;
      }
   }

   // -------------------------------------------------------------------------
   void BinexData::dump(ostream& s) const
   {
      std::cout << "RecordID:    " << recID     << std::endl;
      std::cout << "SyncByte:    " << (unsigned short)syncByte << std::endl;
      std::cout << "MessageLen:  " << msg.size() << std::endl;
      std::cout << "MessageData: " << std::endl;
      for (size_t i = 0; i < msg.size(); i++)
      {
         unsigned short s = msg[i];
         std::cout << s << " ";
      }
      std::cout << std::endl;
   }


}  // namespace gpstk
