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
 * @file BinexData.hpp
 * Encapsulate BINEX file data, including I/O
 */

#ifndef GPSTK_BINEXDATA_HPP
#define GPSTK_BINEXDATA_HPP

#include "gpstkplatform.h"

#include "BinUtils.hpp"
#include "FFData.hpp"
#include "FFStream.hpp"

namespace gpstk
{
   /** @addtogroup Binex */

   //@{

      /** 
       * This class stores, reads, and writes BINEX records. 
       *
       * @sa binex_read_write.cpp for an example.
       * @sa binex_test.cpp for an example.
       * @sa BinexStream.
       */
   class BinexData : public FFData
   {
   public:

      // Establish the endianness of the native platform
   #if BYTE_ORDER == LITTLE_ENDIAN
      static const bool nativeLittleEndian = true;
   #else
      static const bool nativeLittleEndian = false;
   #endif

      static const unsigned long INVALID_RECORD_ID    = 0xFFFFFFFF;

      static const unsigned char DEFAULT_RECORD_FLAGS = 0x20;
      static const unsigned char VALID_RECORD_FLAGS   = 0x38;

         // Flags indicating whether a record is reversed, whether a record
         // is big endian, and whether a record contains an enhanced CRC.
         // Combining these flags (via bitwise-or) helps create the
         // synchronization byte(s) for a BINEX record.
      enum recordFlagsEnum
      {
         eReverseReadable = 0x10,
         eBigEndian       = 0x20,
         eEnhancedCRC     = 0x08
      };

         /// BINEX data types
         //@{

         /**
          * An unsigned integer stored using 1, 2, 3, or 4 bytes to represent
          * integers from 0 to 536870911; used to represent BINEX record IDs,
          * subrecord IDs, field IDs, and so on.
          */      
      class UBNXI
      {
      public:

         static const unsigned long  MIN_VALUE = 0;
         static const unsigned long  MAX_VALUE = 536870911;
         static const unsigned char  MAX_BYTES = 4;

            /**
             * Default constructor - sets value to 0.
             */      
         UBNXI();

            /**
             * Copy constructor.
             */
         UBNXI(const UBNXI& other)
         {
            *this = other;
         };

            /**
             * Constructor with unsigned long initialization value.
             */
         UBNXI(unsigned long ul)
            throw(FFStreamError);

            /**
             * Copies another UBNXI.
             */
         inline UBNXI&
         operator=(const UBNXI& right)
         {
            value = right.value;
            size  = right.size;
            return *this;
         };

            /**
             * Compares two UBNXI's for equality.
             */
         inline bool
         operator==(const UBNXI& other) const
         {
            return (value == other.value);
         };

            /**
             * Compares two UBNXI's for inequality.
             */
         inline bool
         operator!=(const UBNXI& other) const
         {
            return (value != other.value);
         };

            /**
             * Returns whether this UBNXI is less than the other.
             */
         inline bool
         operator<(const UBNXI& other) const
         {
            return (value < other.value);
         };

            /**
             * Returns whether this UBNXI is less than or equal to the other.
             */
         inline bool
         operator<=(const UBNXI& other) const
         {
            return (value <= other.value);
         };

            /**
             * Returns whether this UBNXI is greater than the other.
             */
         inline bool
         operator>(const UBNXI& other) const
         {
            return (value > other.value);
         };

            /**
             * Returns whether this UBNXI is greater than or equal to the other.
             */
         inline bool
         operator>=(const UBNXI& other) const
         {
            return (value >= other.value);
         };

            /**
             * Returns the value of the UBNXI as an unsigned long.
             */
         inline
         operator unsigned long() const
         {
            return value;
         };

            /**
             * Returns the number of bytes required to represent the UBNXI.
             * A size of 0 indicates an invalid or uninitialized UBNXI.
             */
         inline size_t
         getSize() const
         {
            return size;
         };

            /**
             * Attempts to decode a valid UBNXI from the contents of inBuffer.
             * The contents of inBuffer are assumed to be in normal order
             * (i.e. not reversed) but may be either big or little endian.
             * @param  inBuffer Sequence of bytes to decode
             * @param  offset Offset into inBuffer at which to decode
             * @param  littleEndian Byte order of the encoded bytes
             * @return Number of bytes decoded
             */
         size_t
         decode(const std::string& inBuffer,
                size_t             offset       = 0,
                bool               littleEndian = false)
             throw(FFStreamError);

            /**
             * Converts the UBNXI to a series of bytes placed in outBuffer.
             * The bytes are output in normal order (i.e. not reversed) but
             * may encode in either big or little endian format.
             * @param  outBuffer Sequence of encoded bytes
             * @param  offset Offset into outBuffer at which to encode
             * @param  littleEndian Optional flag indicating byte order of
             *                      the encoded bytes
             * @return Number of bytes used to encode
             */
         size_t
         encode(std::string& outBuffer,
                size_t       offset       = 0,
                bool         littleEndian = false) const;

            /**
             * Attempts to read a valid UBNXI from the specified input stream.
             * The stream can be in reverse order and can be
             * big or little endian.  If the method succeeds, the number
             * of bytes used to contruct the UBNXI can be determined
             * by calling the getSize() method.
             * @param strm Stream from which to read
             * @param outBuffer Optional buffer to receive copy of raw input
             * @param offset Offset into outBuffer at which to copy input
             * @param reverseBytes Optional flag indicating whether
             *                     the input bytes are reversed
             * @param littleEndian Optional flag indicating byte order of input
             * @return Number of bytes removed from the input stream
             */
         size_t
         read(std::istream& strm,
              std::string   *outBuffer   = NULL,
              size_t        offset       = 0,
              bool          reverseBytes = false,
              bool          littleEndian = false)
            throw(FFStreamError);

            /**
             * Attempts to write the UBNXI to the specified output stream.
             * The stream can be output in reverse order and can be
             * big or little endian.  The method fails if the entire
             * UBNXI cannot be written to the stream.
             * @param strm Stream in which to write
             * @param outBuffer Optional buffer to receive copy of raw ouput
             * @param offset Offset into outBuffer at which to copy output
             * @param reverseBytes Optional flag indicating whether
             *                     the ouput bytes should be reversed
             * @param littleEndian Optional flag indicating byte order of output
             * @return Number of bytes added to the output stream
             */
         size_t
         write(std::ostream& strm,
               std::string   *outBuffer   = NULL,
               size_t        offset       = 0,
               bool          reverseBytes = false,
               bool          littleEndian = false) const
            throw(FFStreamError);

      protected:

         unsigned long value;
         size_t        size;
      };

         /**
          * A signed integer stored using 1, 2, 3, 4, 5, 6, 7, or 8 bytes to
          * represent integers from about -1.15292e18 to +1.15292e18 using a
          * modified version of a compression scheme developed by GFZ, plus
          * using "special" numbers to flag certain conditions, such as using
          * the 1-byte MFGZI to store "-0" to indicate "no value."
          */      
      class MGFZI
      {
      public:

         static const long long      MIN_VALUE = -1157442765409226759LL;
         static const long long      MAX_VALUE =  1157442765409226759LL;
         static const unsigned char  MAX_BYTES =  8;

            /**
             * Default constructor - sets value to 0.
             */      
         MGFZI();

            /**
             * Copy constructor.
             */
         MGFZI(const MGFZI& other)
         {
            *this = other;
         };

            /**
             * Constructor with a long long initialization value.
             */
         MGFZI(long long ll)
            throw(FFStreamError);

            /**
             * Copies another MGFZI.
             */
         inline MGFZI&
         operator=(const MGFZI& right)
         {
            value = right.value;
            size  = right.size;
            return *this;
         };

            /**
             * Compares two MGFZI's for equality.
             */
         inline bool
         operator==(const MGFZI& other) const
         {
            return (value == other.value);
         };

            /**
             * Compares two MGFZI's for inequality.
             */
         inline bool
         operator!=(const MGFZI& other) const
         {
            return (value != other.value);
         };

            /**
             * Returns whether this MGFZI is less than the other.
             */
         inline bool
         operator<(const MGFZI& other) const
         {
            return (value < other.value);
         };

            /**
             * Returns whether this MGFZI is less than or equal to the other.
             */
         inline bool
         operator<=(const MGFZI& other) const
         {
            return (value <= other.value);
         };

            /**
             * Returns whether this MGFZI is greater than the other.
             */
         inline bool
         operator>(const MGFZI& other) const
         {
            return (value > other.value);
         };

            /**
             * Returns whether this MGFZI is greater than or equal to the other.
             */
         inline bool
         operator>=(const MGFZI& other) const
         {
            return (value >= other.value);
         };

            /**
             * Returns the value of the MGFZI as a long long.
             */
         inline
         operator long long() const
         {
            return value;
         };

            /**
             * Returns the number of bytes required to represent the MGFZI.
             * A size of 0 indicates an invalid or uninitialized MGFZI.
             */
         inline size_t
         getSize() const
         {
            return size;
         };

            /**
             * Attempts to decode a valid MGFZI from the contents of inBuffer.
             * The contents of inBuffer are assumed to be in normal order
             * (i.e. not reversed) but may be either big or little endian.
             * @param  inBuffer Sequence of bytes to decode
             * @param  offset Offset into inBuffer at which to decode
             * @param  littleEndian Byte order of the encoded bytes
             * @return Number of bytes decoded
             */
         size_t
         decode(const std::string& inBuffer,
                size_t             offset       = 0,
                bool               littleEndian = false)
            throw(FFStreamError);

            /**
             * Converts the MGFZI to a series of bytes placed in outBuffer.
             * The bytes are output in normal order (i.e. not reversed) but
             * may encode in either big or little endian format.
             * @param  outBuffer Sequence of encoded bytes
             * @param  offset Offset into outBuffer at which to encode
             * @param  littleEndian Byte order of the encoded bytes
             * @return Number of bytes used to encode
             */
         size_t
         encode(std::string& outBuffer,
                size_t       offset       = 0,
                bool         littleEndian = false) const;

            /**
             * Attempts to read a valid MGFZI from the specified input stream.
             * The stream can be in reverse order and can be
             * big or little endian.  If the method succeeds, the number
             * of bytes used to contruct the MGFZI can be determined
             * by calling the getSize() method.
             * @param strm Stream from which to read
             * @param outBuffer Optional buffer to receive copy of raw input
             * @param reverseBytes Optional flag indicating whether
             *                     the input bytes are reversed
             * @param littleEndian Optional flag indicating byte order of input
             */
         size_t
         read(std::istream& strm,
              std::string *outBuffer   = NULL,
              size_t      offset       = 0,
              bool        reverseBytes = false,
              bool        littleEndian = false)
            throw(FFStreamError);

            /**
             * Attempts to write the MGFZI to the specified output stream.
             * The stream can be output in reverse order and can be
             * big or little endian.  The method fails if the entire
             * MGFZI cannot be written to the stream.
             * @param strm Stream in which to write
             * @param outBuffer Optional buffer to receive copy of raw ouput
             * @param reverseBytes Optional flag indicating whether
             *                     the ouput bytes should be reversed
             * @param littleEndian Optional flag indicating byte order of output
             */
         size_t
         write(std::ostream& strm,
               std::string *outBuffer   = NULL,
               size_t      offset       = 0,
               bool        reverseBytes = false,
               bool        littleEndian = false) const
            throw(FFStreamError);

      protected:

         long long value;
         size_t    size;
      };

         //@}


         /**
          * Default constructor
          */
      BinexData();

         /**
          * Copy constructor
          */
      BinexData(const BinexData& other);

         /**
          * Convenience constructor
          */
      BinexData(unsigned long recordID,
                unsigned char recordFlags = DEFAULT_RECORD_FLAGS)
         throw();

         /**
          * Copies another BinexData object.
          */
      BinexData&
      operator=(const BinexData& right);

         /**
          * Destructor
          */
      virtual
      ~BinexData() {};

         /**
          * BinexData is "data" so this function always returns true.
          */
      virtual bool
      isData(void) const
      {
         return true;
      }

         /**
          * A debug output function.
          */
      virtual void
      dump(std::ostream& s) const;
      
         /**
          * Compares two BinexData objects.
          * 
          * @param b BinexData object to compare to this object
          */
      bool
      operator==(const BinexData& b) const;

         /**
          * Returns flags indicating endianness, reversability, and CRC-mode
          * of the current record.  The individual flags can be extracted from
          * the returned value by AND-ing with values from recordFlagsEnum.
          */
      inline unsigned char
      getRecordFlags() const
      {
            // Return only essential, valid flag bits listed in recordFlagMask
         return syncByte & VALID_RECORD_FLAGS;
      };

         /**
          * Sets the endianness, reversability, and CRC-mode of the record.
          * The "flags" paramater should be set by OR-ing together values
          * from recordFlagsEnum enumeration.  Invalid bits in "flag" are
          * silently ignored.
          *
          * WARNING: Since the record flags determine how data is stored in
          *          the record message buffer, altering the record flags
          *          after data has been placed in the message buffer
          *          could result in misinterpretation of that data.
          *          Doing so is therefore highly discouraged.
          */
      BinexData&
      setRecordFlags(unsigned char flags = DEFAULT_RECORD_FLAGS);

         /**
          * Returns the ID of this BINEX record.
          */
      inline unsigned long
      getRecordID() const
      {
         return recID;         
      };

         /**
          * Sets the ID of this BINEX record.
          */
      BinexData&
      setRecordID(unsigned long id)
         throw(FFStreamError);

         /**
          * Returns the number of bytes required to represent the entire record
          * (based on the record's current contents).
          */
      size_t
      getRecordSize() const;

         /**
          * Remove all data from the record message buffer.
          */
      BinexData&
      clearMessage();

         /**
          * Reserves a number of bytes for storage of the record message.
          * This number can grow as data is added to the message, but an
          * adequate initial number results in greater efficiency.  The
          * actual length of the data in the message buffer is a separate
          * and strictly smaller amount.
          */
      BinexData&
      ensureMessageCapacity(size_t cap)
         throw(FFStreamError);

         /**
          * Returns the length of the data in the record message buffer
          * (which is separate from the record message buffer's capacity).
          * 
          * @return Record message data length in bytes
          */
      inline size_t
      getMessageLength() const
      {
         return msg.size();
      };

         /**
          * Returns the capacity of the record message buffer (which is
          * separate from the lenth of the data in the buffer).
          *
          * @return Record message capacity in bytes
          */
      inline size_t
      getMessageCapacity() const
      {
         return msg.capacity();
      };

         /**
          * Returns a pointer to the raw message data.  Note that the format
          * of the data is dependent upon the record flags at the time the
          * data was added to the message.
          */
      //inline const char*
      inline const std::string&
      getMessageData() const
      {
         //return msg.data();
         return msg;
      };

         /**
          * Updates the message buffer with the specified UBNXI.  The location
          * within the message buffer is set by the offset parameter.
          * This method checks to ensure that all data fits within
          * the message buffer.  After updating the message buffer, the
          * value of the offset parameter is updated by size to reference
          * the next available byte in the message buffer.
          * 
          * @param offset Location within the message buffer at which to update
          * @param data   Data with which to update the message buffer
          */
      BinexData&
      updateMessageData(
         size_t&      offset,
         const UBNXI& data)
            throw(FFStreamError, InvalidParameter);

         /**
          * Updates the message buffer with the specified MGFZI.  The location
          * within the message buffer is set by the offset parameter.
          * This method checks to ensure that all data fits within
          * the message buffer.  After updating the message buffer, the
          * value of the offset parameter is updated by size to reference
          * the next available byte in the message buffer.
          * 
          * @param offset Location within the message buffer at which to update
          * @param data   Data with which to update the message buffer
          */
      BinexData&
      updateMessageData(
         size_t&      offset,
         const MGFZI& data)
            throw(FFStreamError, InvalidParameter);

         /**
          * Updates the message buffer with the specified raw data.  The
          * location within the message buffer is set by the offset parameter,
          * and the size of the data to copy is set by the size parameter.
          * After updating the message buffer, the value of the offset
          * parameter is updated by size to reference the next available byte
          * in the message buffer.
          * 
          * @param offset Location within the message buffer at which to update
          * @param data   Raw data with which to update the message buffer
          * @param size   Number of bytes of data to be copied
          */
      BinexData&
      updateMessageData(
         size_t&            offset,
         const std::string& data,
         size_t             size)
            throw(FFStreamError, InvalidParameter);

         /**
          * Updates the message buffer with the specified raw data.  The
          * location within the message buffer is set by the offset parameter,
          * and the size of the data to copy is set by the size parameter.
          * After updating the message buffer, the value of the offset
          * parameter is updated by size to reference the next available byte
          * in the message buffer.
          * 
          * @param offset Location within the message buffer at which to update
          * @param data   Raw data with which to update the message buffer
          * @param size   Number of bytes of data to be copied
          */
      BinexData&
      updateMessageData(
         size_t&     offset,
         const char  *data,
         size_t      size)
            throw(FFStreamError, InvalidParameter);

         /**
          * Updates the message buffer with the specified data.  The location
          * within the message buffer is set by the offset parameter,
          * and the size of the data to copy is set by the size parameter.
          * This method checks to ensure that the value of the size parameter
          * does not exceed sizeof(T) and that all data fits within
          * the message buffer.  After updating the message buffer, the
          * value of the offset parameter is updated by size to reference
          * the next available byte in the message buffer.
          * 
          * @param offset Location within the message buffer at which to update
          * @param data   Data with which to update the message buffer
          * @param size   Number of bytes of data to be copied
          */
      template<class T>
      BinexData&
      updateMessageData(
         size_t&      offset,
         const T&     data,
         size_t       size)
            throw(FFStreamError, InvalidParameter)
      {
         if (size > sizeof(T) )
         {
            std::ostringstream errStrm;
            errStrm << "Invalid data size: " << size;
            InvalidParameter ip(errStrm.str() );
            GPSTK_THROW(ip);
         }
         bool   littleEndian  = ( (syncByte & eBigEndian) == 0) ? true : false;
         if (littleEndian == nativeLittleEndian)
         {
            msg.replace(offset, size, reinterpret_cast<const char*>(&data), size);
         }
         else
         {
            T tmpData(data);
            BinUtils::twiddle(tmpData);
            msg.replace(offset, size, reinterpret_cast<const char*>(&tmpData), size);
         }
         offset += size;
         return *this;

      } // BinexData::updateMessageData()

         /**
          * Extacts a UBNXI from the message buffer.  The location within the
          * message buffer is set by the offset parameter.  After extracting
          * the UBNXI from the message buffer, the value of the offset parameter
          * is updated by the UBNXI's size to reference the next available byte
          * in the message buffer.
          * 
          * @param offset Location within the message buffer at which to extract
          * @param data   Location to store the extracted data
          */
      void
      extractMessageData(
         size_t& offset,
         UBNXI&  data)
            throw(FFStreamError, InvalidParameter);

         /**
          * Extacts a MGFZI from the message buffer.  The location within the
          * message buffer is set by the offset parameter.  After extracting
          * the MGFZI from the message buffer, the value of the offset parameter
          * is updated by the MGFZI's size to reference the next available byte
          * in the message buffer.
          * 
          * @param offset Location within the message buffer at which to extract
          * @param data   Location to store the extracted data
          */
      void
      extractMessageData(
         size_t& offset,
         MGFZI&  data)
            throw(FFStreamError, InvalidParameter);

         /**
          * Extacts raw data from the message buffer.  The location within the
          * message buffer is set by the offset parameter, and the size of the
          * data to extract is set by the size parameter.  This method checks
          * to ensure that all data is extracted from within the message
          * buffer.  After extracting data from the message buffer,
          * the value of the offset parameter is updated by size to reference
          * the next available byte in the message buffer.
          * 
          * @param offset Location within the message buffer at which to extract
          * @param data   Location to store the extracted data
          * @param size   Number of bytes of data to be extracted
          */
      void
      extractMessageData(
         size_t&      offset,
         std::string& data,
         size_t       size) const
            throw(InvalidParameter);

         /**
          * Extacts data from the message buffer.  The location within the
          * message buffer is set by the offset parameter, and the size of the
          * data to extract is set by the size parameter.  This method checks
          * to ensure that the value of the size parameter does not exceed
          * sizeof(T) and that all data is extracted from within the
          * message buffer.  After extracting data from the message buffer,
          * the value of the offset parameter is updated by size to reference
          * the next available byte in the message buffer.
          * 
          * @param offset Location within the message buffer at which to extract
          * @param data   Location to store the extracted data
          * @param size   Number of bytes of data to be extracted
          */
      template<class T>
      void
      extractMessageData(
         size_t&      offset,
         T&           data,
         size_t       size) const
            throw(FFStreamError, InvalidParameter)
      {
         if (size > sizeof(T) )
         {
            std::ostringstream errStrm;
            errStrm << "Data size invalid: " << size;
            InvalidParameter ip(errStrm.str() );
            GPSTK_THROW(ip);
         }  
         if (offset + size > msg.size() )
         {
            std::ostringstream errStrm;
            errStrm << "Message buffer offset invalid: " << offset;
            InvalidParameter ip(errStrm.str() );
            GPSTK_THROW(ip);
         }  
         bool littleEndian  = ( (syncByte & eBigEndian) == 0) ? true : false;
         msg.copy(reinterpret_cast<char*>(&data), size, offset);
         if (littleEndian != nativeLittleEndian)
         {
            BinUtils::twiddle(data);
         }
         offset += size;

      } // BinexData::extractMessageData()


   protected:

         /**
          * Writes the BINEX data to the file stream formatted correctly.
          */
      virtual void
      reallyPutRecord(FFStream& s) const
         throw(std::exception, FFStreamError, 
               StringUtils::StringException);     

         /** 
          * This function retrieves a BINEX record from the given FFStream.
          * If an error is encountered in reading from the stream, the stream
          * is returned to its original position and its fail-bit is set.
          * @throws StringException when a StringUtils function fails
          * @throws FFStreamError when exceptions(failbit) is set and
          *  a read or formatting error occurs.  This also resets the
          *  stream to its pre-read position.
          */
      virtual void
      reallyGetRecord(FFStream& s) 
         throw(std::exception, FFStreamError,
               StringUtils::StringException);

         /**
          * @param bufs    A NULL-terminated list of pointers to byte buffers
          * @param bufLens A list of lengths for the buffers specified by bufs
          * @param crc     A pointer to the buffer in which to store the CRC
          * @param crcLen  The number of bytes used to store the CRC
          */
      void getCRC(const std::string& head,
                  const std::string& message,
                  std::string&       crc) const;

         /**
          * Returns the number of bytes required to store the record's CRC
          * based on the record's current contents. 
          */
      size_t
      getCRCLength(size_t crcDataLen) const;

         /**
          * Determines whether the supplied head sync byte is valid an returns
          * an expected correosponding tail sync byte if appropriate.
          */
      bool
      isHeadSyncByteValid(unsigned char  headSync,
                          unsigned char& expectedTailSync) const;

         /**
          * Determines whether the supplied tail sync byte is valid an returns
          * an expected correosponding head sync byte.
          */
      bool
      isTailSyncByteValid(unsigned char  tailSync,
                          unsigned char& expectedHeadSync) const;
         /**
          * Converts a raw sequence of bytes into an unsigned long long integer.
          *
          * @param buffer  Raw bytes to convert
          * @param offset  Position at which to begin conversion
          * @param size    Number of bytes to convert
          * @return Result of converting raw bytes to an unsigned integer
          */
      static unsigned long long
      parseBuffer(const std::string&  buffer,
                  size_t              offset,
                  size_t              size)
         throw(FFStreamError);

         /**
          * Reverses the order of the first bufferLength bytes in the
          * specified buffer.
          * 
          * @param buffer       Pointer to the bytes
          * @param bufferLength Number of bytes to reverse
          */
      static void
      reverseBuffer(unsigned char *buffer,
                    size_t        bufferLength);

         /**
          * Reverses the order of the first bufferLength bytes in the
          * specified buffer.
          * 
          * @param buffer  String containing bytes to reverse
          * @param offset  Starting position of bytes to reverse
          * @param n       Number of bytes to reverse
          */
      static void
      reverseBuffer(std::string& buffer,
                    size_t       offset = 0,
                    size_t       n      = std::string::npos);

         /** @name Attributes
          */
         //@{
      unsigned char  syncByte;  ///< Flags for endianness, CRC, etc.
      unsigned long  recID;     ///< Record ID
      std::string    msg;       ///< Record message (opaque)
         //@}

   private:

   };  // class BinexData

   //@}

} // namespace gpstk


#endif // GPSTK_BINEXDATA_HPP
