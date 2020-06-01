#pragma ident "$Id$"

/**
 * @file SP3Stream.hpp
 * gpstk::SP3Stream - SP3[abc] format file stream
 */

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

#ifndef SP3STREAM_INCLUDE
#define SP3STREAM_INCLUDE

#include "FFTextStream.hpp"
#include "SP3Header.hpp"

namespace gpstk
{
   /// @addtogroup SP3
   //@{

       /// This class performs file I/O on an SP3 file for the SP3Header
       /// and SP3Data classes.
       /// Note that the file format (a, b or c) is stored in the SP3Header (only).
       /// On input it is set by SP3Header::reallyGetRecord() by the file content;
       /// for output it may be set (SP3Header::setVersion()) before streaming.
   class SP3Stream : public FFTextStream
   {
   public:
         /// Default constructor
      SP3Stream() 
         : wroteEOF(false),
           writingMode(false),
           lastLine(std::string())
         {}
      
         /// Common constructor: open (default: to read)
         /// @param filename the name of the ASCII SP3 format file to be opened
         /// @param mode the ios::openmode to be used
      SP3Stream(const char* filename,
                std::ios::openmode mode=std::ios::in)
            : FFTextStream(filename, mode)
            { open(filename, mode); }

         /// destructor; override to force 'close'
      virtual ~SP3Stream()
      {
         if(writingMode && !wroteEOF) close();
      }

         /// override close() to write EOF line
      virtual void close(void) throw(Exception)
      {
         try {
            // if writing, add the final line
            if(writingMode && !wroteEOF) {
               (*this) << "EOF\n"; 
               wroteEOF = true;
            }
            FFTextStream::close();
         }
         catch(std::exception& e) {
            Exception ge(e.what());
            GPSTK_THROW(ge);
         }
      }

         /// override open() to reset the header
         /// @param filename the name of the ASCII SP3 format file to be opened
         /// @param mode the ios::openmode to be used
      virtual void open(const char* filename, std::ios::openmode mode)
      {
         FFTextStream::open(filename, mode);
         header = SP3Header();
         warnings.clear();

         // for close() later
         wroteEOF = writingMode = false;
         if( (mode & std::ios::out) && !(mode & std::ios::in) )
            writingMode = true;

         // this is necessary in order for SP3Data::reallyGetRecord() to
         // process the last line in the file when there is no EOF record...why?
         if(mode & std::ios::in) exceptions(std::ifstream::failbit);
      }

         ///@name data members
         //@{
      SP3Header header;     ///< SP3Header for this file
      bool wroteEOF;        ///< True if the final 'EOF' has been read.
      bool writingMode;     ///< True if the stream is open in 'out', not 'in', mode
      CommonTime currentEpoch;   ///< Time from last epoch record read
      std::string lastLine;      ///< Last line read, perhaps not yet processed
      std::vector<std::string> warnings; ///< warnings produced by reallyGetRecord()s
         //@}

   }; // class SP3Stream
   
   //@}
   
} // namespace gpstk

#endif // SP3STREAM_INCLUDE
