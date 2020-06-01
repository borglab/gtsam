#pragma ident "$Id$"



/**
 * @file FFBinaryStream.hpp
 * An FFStream for binary file reading
 */

#ifndef GPSTK_FFBINARYSTREAM_HPP
#define GPSTK_FFBINARYSTREAM_HPP

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


#include <gtsam/gpstk/FFStream.hpp>

namespace gpstk
{
   /** @defgroup formattedfile Formatted file I/O */
   //@{
 
      /**
       * This is an FFStream that is required to be binary.  It also includes
       * functions for reading and writing binary file.  Otherwise, this
       * is the same as FFStream.
       */
   class FFBinaryStream : public FFStream
   {
   public:
         /// destructor
      virtual ~FFBinaryStream() {};
      
         /// Default constructor
      FFBinaryStream() {}

         /**
          * Constructor - opens the stream in binary mode if not set.
          * @param fn file name.
          * @param mode file open mode (std::ios)
          */
      FFBinaryStream(const char* fn, 
                     std::ios::openmode mode=std::ios::in|std::ios::binary)
         : FFStream(fn, mode|std::ios::binary) {}

         /// Overrides open to ensure binary mode opens
      virtual void open(const char* fn, std::ios::openmode mode)
         { FFStream::open(fn, mode|std::ios::binary); }

         /**
          * Reads a T-object directly from the stream
          * in binary form.
          * @throw FFStreamError when the size of the data read
          * from this stream doesn't match the size of a T-object.
          * @return a T-object
          */
      template <class T> T getData() throw(FFStreamError, EndOfFile)
      {
         T data;
         getData((char*)&data, sizeof(T));
         return data;
      } // end of getData(FFStream& strm)

      void getData(char* buff, size_t length) throw(FFStreamError, EndOfFile)
      {
         try
         {
            read(buff, length);
         }
         catch(std::exception& exc)
         {
            if (gcount() != length && eof())
            {
               EndOfFile err("EOF encountered");
               GPSTK_THROW(err);
            }
            else
            {
               FFStreamError err(exc.what());
               std::cout << err << std::endl;
               GPSTK_THROW(err);
            }
         }
         catch(...)
         {
            FFStreamError err("Unknown exception");
            GPSTK_THROW(err);
         }
      } // end of getData(char*, size_t))

         /**
          * Writes a T-object directly from the stream
          * in binary form.
          * @param data the data to be written.
          * @throw FFStreamError when the size of the data written
          * to this stream doesn't match the size of a T-object.
          * @return a T-object
          */
      template <class T> void writeData(const T& data)
         throw(FFStreamError)
      {
         T temp = data;
#pragma unused(temp)
          writeData((char*)&data, sizeof(T));
         return;
      } // end of writeData(FFStream& strm, const T& data)

      void writeData(const char* buff, size_t length)
         throw(FFStreamError)
      {
         try
         {
            write(buff, length);
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
      
         if (fail() || bad())
         {
            FFStreamError err("Error writing data");
            GPSTK_THROW(err);
         }
         return;
      } // end of writeData(const char*, size_t)

   };
   //@}
}
#endif
