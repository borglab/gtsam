#pragma ident "$Id$"

/**
 * @file FFTextStream.hpp
 * An FFStream for text files
 */

#ifndef GPSTK_FFTEXTSTREAM_HPP
#define GPSTK_FFTEXTSTREAM_HPP

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

      /** @addtogroup formattedfile */
      //@{

      /**
       * An FFStream is meant for reading text.  This also includes an
       * internal line count and a read line method. When reading and
       * using the formattedGetLine() call, the lineNumber automatically
       * increments.  However, any other read and all write calls do not
       * update the line number - the derived class or programmer
       * needs to make sure that the reader or writer increments
       * lineNumber in these cases.
       */
   class FFTextStream : public FFStream
   {
   public:


         /// Destructor
      virtual ~FFTextStream() {};


         /// Default constructor
      FFTextStream()
            : lineNumber(0) {};


         /** Common constructor.
          *
          * @param fn file name.
          * @param mode file open mode (std::ios)
          */
      FFTextStream( const char* fn,
                    std::ios::openmode mode=std::ios::in )
         : FFStream(fn, mode), lineNumber(0)
      {};


         /** Common constructor.
          *
          * @param fn file name.
          * @param mode file open mode (std::ios)
          */
      FFTextStream( const std::string& fn,
                    std::ios::openmode mode=std::ios::in )
         : FFStream( fn.c_str(), mode ), lineNumber(0)
      {};


         /// Overrides open to reset the line number.
      virtual void open( const char* fn,
                         std::ios::openmode mode )
      { FFStream::open(fn, mode); lineNumber = 0; };


         /// Overrides open to reset the line number.
      virtual void open( const std::string& fn,
                         std::ios::openmode mode )
      { open(fn.c_str(), mode); };


         /// The internal line count. When writing, make sure
         /// to increment this.
      unsigned int lineNumber;


         /**
          * Like std::istream::getline but checks for EOF and removes '/r'.
          * Also increments lineNumber.  When \a expectEOF is true and EOF
          * is found, an gpstk::EndOfFile exception is thrown.  If
          * \a expectEOF is false and an EOF is encountered, an
          * gpstk::FFStreamError is thrown.
          * @param line is set to the value of the line read from the file.
          * @param expectEOF set true if finding EOF on this read is acceptable.
          * @throw EndOfFile if \a expectEOF is true and an EOF is encountered.
          * @throw FFStreamError if EOF is found and \a expectEOF is false
          * @throw gpstk::StringUtils::StringException when a string error occurs
          * or if any other error happens.
          * @warning There is a maximum line length of 1500 characters when
          * using this function.
          */
      inline void formattedGetLine( std::string& line,
                                    const bool expectEOF = false )
         throw(EndOfFile, FFStreamError, gpstk::StringUtils::StringException);


   protected:


         /// calls FFStream::tryFFStreamGet and adds line number information
      virtual void tryFFStreamGet(FFData& rec)
         throw(FFStreamError, gpstk::StringUtils::StringException)
      {

         unsigned int initialLineNumber = lineNumber;

         try
         {
            FFStream::tryFFStreamGet(rec);
         }
         catch(gpstk::Exception& e)
         {
            e.addText( std::string("Near file line ") +
                       gpstk::StringUtils::asString(lineNumber) );
            lineNumber = initialLineNumber;
            mostRecentException = e;
            conditionalThrow();
         }

       };


         /// calls FFStream::tryFFStreamPut and adds line number information
      virtual void tryFFStreamPut(const FFData& rec)
         throw(FFStreamError, gpstk::StringUtils::StringException)
      {

         unsigned int initialLineNumber = lineNumber;

         try
         {
            FFStream::tryFFStreamPut(rec);
         }
         catch(gpstk::Exception& e)
         {
            e.addText( std::string("Near file line ") +
                       gpstk::StringUtils::asString(lineNumber) );
            lineNumber = initialLineNumber;
            mostRecentException = e;
            conditionalThrow();
         }

      }

   }; // End of class 'FFTextStream'



      // the reason for checking ffs.eof() in the try AND catch block is
      // because if the user enabled exceptions on the stream with exceptions()
      // then eof could throw an exception, in which case we need to catch it
      // and rethrow an EOF or FFStream exception.  In any event, EndOfFile
      // gets thrown whenever there's an EOF and expectEOF is true
   void FFTextStream::formattedGetLine( std::string& line,
                                        const bool expectEOF )
         throw(EndOfFile, FFStreamError, gpstk::StringUtils::StringException)
   {

      try
      {
            // The following constant used to be 256, but with the change to
            // RINEX3 formats the possible length of a line increased
            // considerably. A RINEX3 observation file line for Galileo may
            // be 1277 characters long (taking into account all the possible
            // types of observations available, plus the end of line
            // characters), so this constant was conservatively set to
            // 1500 characters. Dagoberto Salazar.
         const int MAX_LINE_LENGTH = 1500;
         char templine[MAX_LINE_LENGTH + 1];
         getline(templine, MAX_LINE_LENGTH);
         lineNumber++;
            //check if line was longer than 256 characters, if so error
         if(fail() && !eof())
         {
            FFStreamError err("Line too long");
            GPSTK_THROW(err);
         }
         line = templine;
         gpstk::StringUtils::stripTrailing(line, '\r');
            // catch EOF when stream exceptions are disabled
         if ((gcount() == 0) && eof())
         {
            if (expectEOF)
            {
               EndOfFile err("EOF encountered");
               GPSTK_THROW(err);
            }
            else
            {
               FFStreamError err("Unexpected EOF encountered");
               GPSTK_THROW(err);
            }
         }
      }
      catch(std::exception &e)
      {

            // catch EOF when exceptions are enabled
         if ( (gcount() == 0) && eof())
         {
            if (expectEOF)
            {
               EndOfFile err("EOF encountered");
               GPSTK_THROW(err);
            }
            else
            {
               FFStreamError err("Unexpected EOF");
               GPSTK_THROW(err);
            }
         }
         else
         {
            FFStreamError err("Critical file error: " +
                              std::string(e.what()));
            GPSTK_THROW(err);
         }  // End of 'if ( (gcount() == 0) && eof())'

      }  // End of 'try-catch' block

   }  // End of method 'FFTextStream::formattedGetLine()'

      //@}

}  // End of namespace gpstk
#endif   // GPSTK_FFTEXTSTREAM_HPP
