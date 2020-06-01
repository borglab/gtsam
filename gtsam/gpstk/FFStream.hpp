#pragma ident "$Id$"

/**
 * @file FFStream.hpp
 * Formatted File Stream, root class to provide formatted I/O
 * operators ('<<' & '>>')
 */

#ifndef GPSTK_FFSTREAM_HPP
#define GPSTK_FFSTREAM_HPP

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



#include <iostream>
#include <fstream>
#include <string>
#include <typeinfo>

#include <gtsam/gpstk/FFStreamError.hpp>
#include <gtsam/gpstk/FFData.hpp>
#include <gtsam/gpstk/StringUtils.hpp>


namespace gpstk
{
   /** @addtogroup formattedfile */
   //@{

      /// This gets thrown if a valid EOF occurs on formattedGetLine.
      /// @ingroup exceptionclass
   NEW_EXCEPTION_CLASS(EndOfFile, gpstk::FFStreamError);

      /**
       * Formatted File Stream (FFStream).
       * This is just a root class to provide the single point formatted i/o
       * operators (such as '<<' & '>>' ).
       *
       * As a special design consideration,
       * all exceptions thrown are based on gpstk::Exception - all
       * std::exception throws are rethrown as gpstk::Exception.
       * Furthermore, exceptions will not be thrown unless exceptions
       * are set to be thrown:
       * @code
       * fs.exceptions(std::fstream::failbit);
       * @endcode
       * where \c fs is the name of your file stream.
       * Then when an exception occurs, conditionalThrow() will throw the
       * last thrown exception.
       * Otherwise when an exception occurs, the stream sets
       * \c ios::fail and will not read any more.  Exceptions for this
       * class store the record number of the file for when the exception
       * occurred as well as the file name and any detailed information
       * about the error.  For gpstk::FFTextStream, the line number
       * of the file where the error was found is also recorded, allowing
       * for easy location of file problems.
       *
       * When operating on the file, recordNumber will automatically increment
       * with each read and write operation. When a file is opened with the
       * constructor or with open(), all internal FFStream variables are
       * reset. Derived classes should make sure any of their internal
       * variables are reset when either of those function are called.
       *
       * Many file types have header data as part of the file format. When
       * reading the file, the reader is not required to explicitly read in
       * the header to access the data.  To facilitate this, each of these
       * stream classes has an internal header object that will store the
       * header. The stream keeps track of whether it read the
       * header or not, and reads the header if the internal state says
       * it hasn't been read.  When writing a file, the stream's
       * internal header is used for those formats which use header information
       * to determine what data is in the records.
       * See RinexObsHeader::reallyGetRecord() and
       * RinexObsData::reallyGetRecord()
       * for an example of this.
       *
       * \sa FFData for more information
       * \sa RinexObsData::reallyGetRecord() and
       *     RinexObsHeader::reallyGetRecord() for more information for files
       *     that read header data.
       *
       * @warning When using open(), the internal header data of the stream
       * is not guaranteed to be retained.
       */
   class FFStream : public std::fstream
   {
   public:

         /// Virtual destructor
      virtual ~FFStream(void) {};


         /**
          * Default constructor
          */
      FFStream()
            : recordNumber(0) {};


         /** Common constructor.
          *
          * @param fn file name.
          * @param mode file open mode (std::ios)
          */
      FFStream( const char* fn, std::ios::openmode mode=std::ios::in )
         : std::fstream(fn, mode), recordNumber(0), filename(fn)
      { clear(); }


         /** Common constructor.
          *
          * @param fn file name.
          * @param mode file open mode (std::ios)
          */
      FFStream( const std::string& fn, std::ios::openmode mode=std::ios::in )
         : std::fstream(fn.c_str(), mode), recordNumber(0), filename(fn)
      { clear(); };


         /**
          * Overrides fstream:open so derived classes can make appropriate
          * internal changes (line count, header info, etc).
          */
      virtual void open( const char* fn, std::ios::openmode mode );


         /**
          * Overrides fstream:open so derived classes can make appropriate
          * internal changes (line count, header info, etc).
          */
      virtual void open( const std::string& fn, std::ios::openmode mode )
      { open( fn.c_str(), mode ); };


         /// A function to help debug FFStreams
      void dumpState(std::ostream& s = std::cout) const;


         /**
          * Throws \a mostRecentException only if the stream is enabled
          * to throw exceptions when failbit is set.
          * You can set this behavior with the following line of code:
          * @code
          * ffstreamobject.exceptions(ifstream::failbit);
          * @endcode
          * where \a ffstreamobject is the name of your stream object.
          */
      inline void conditionalThrow(void) throw(FFStreamError)
      {

         if (exceptions() & std::fstream::failbit)
         {
            GPSTK_THROW(mostRecentException);
         }

      };

         /// Check if the input stream is the kind of RinexObsStream
      static bool IsFFStream(std::istream& i)
      {
         try
         { 
            FFStream& r = dynamic_cast<FFStream&>(i);
            #pragma unused(r)
         }
         catch(...)
         {
            return false;
         }

         return true;
      }


         ///@name Data members
         ///@{
         /// This stores the most recently thrown exception.
      FFStreamError mostRecentException;


         /// keeps track of the number of records read
      unsigned int recordNumber;


         /// file name
      std::string filename;

         //@}


         /// FFData is a friend so it can access the try* functions.
      friend class FFData;


   protected:


         /// Encapsulates shared try/catch blocks for all file types
         /// to hide std::exception.
      virtual void tryFFStreamGet(FFData& rec)
         throw(FFStreamError, gpstk::StringUtils::StringException);


         /// Encapsulates shared try/catch blocks for all file types
         /// to hide std::exception.
      virtual void tryFFStreamPut(const FFData& rec)
         throw(FFStreamError, gpstk::StringUtils::StringException);


   }; // End of class 'FFStream'

      //@}

}  // End of namespace gpstk
#endif   // GPSTK_FFSTREAM_HPP


