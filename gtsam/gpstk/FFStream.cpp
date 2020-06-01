#pragma ident "$Id$"

/**
 * @file FFStream.cpp
 * Formatted File Stream base class
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



#include <gtsam/gpstk/FFStream.hpp>

namespace gpstk
{

      /*
       * Overrides fstream:open so derived classes can make appropriate
       * internal changes (line count, header info, etc).
       */
   void FFStream::open( const char* fn, std::ios::openmode mode )
   {
      std::fstream::open(fn, mode);

      filename = std::string(fn);
      recordNumber = 0;
      clear();

   }  // End of method 'FFStream::open()'



      // A function to help debug FFStreams
   void FFStream::dumpState(std::ostream& s) const
   {

      s << "filename:" << filename
        << ", recordNumber:" << recordNumber;
      s << ", exceptions:";

      if (exceptions() & std::ios::badbit)  s << "bad ";
      if (exceptions() & std::ios::failbit) s << "fail ";
      if (exceptions() & std::ios::eofbit)  s << "eof ";
      if (exceptions() == 0) s << "none";

      s << ", rdstate:";

      if (rdstate() & std::ios::badbit)  s << "bad ";
      if (rdstate() & std::ios::failbit) s << "fail ";
      if (rdstate() & std::ios::eofbit)  s << "eof ";
      if (rdstate() == 0)  s << "none";
      s << std::endl;

   }  // End of method 'FFStream::dumpState()'



      // the crazy double try block is so that no gpstk::Exception throws
      // get masked, allowing all exception information (line numbers, text,
      // etc) to be retained.
   void FFStream::tryFFStreamGet(FFData& rec)
      throw(FFStreamError, gpstk::StringUtils::StringException)
   {

         // Mark where we start in case there is an error.
      long initialPosition = tellg();
      unsigned long initialRecordNumber = recordNumber;
      clear();

      try
      {
         try
         {
            rec.reallyGetRecord(*this);
            recordNumber++;
         }
         catch (EndOfFile& e)
         {
            // EOF - do nothing - eof causes fail() to be set which
            // is handled by std::fstream
            e.addText("In record " +
               gpstk::StringUtils::asString(recordNumber));
            e.addText("In file " + filename);
            e.addLocation(FILE_LOCATION);
            mostRecentException = e;
         }
         catch (std::exception &e)
         {
            mostRecentException = FFStreamError("std::exception thrown: " +
                                                std::string(e.what()));
            mostRecentException.addText("In record " +
                  gpstk::StringUtils::asString(recordNumber));
            mostRecentException.addText("In file " + filename);
            mostRecentException.addLocation(FILE_LOCATION);
            clear();
            seekg(initialPosition);
            recordNumber = initialRecordNumber;
            setstate(std::ios::failbit);
            conditionalThrow();
         }
         catch (gpstk::StringUtils::StringException& e)
         {
            e.addText("In record " +
                      gpstk::StringUtils::asString(recordNumber));
            e.addText("In file " + filename);
            e.addLocation(FILE_LOCATION);
            mostRecentException = e;
            clear();
            seekg(initialPosition);
            recordNumber = initialRecordNumber;
            setstate(std::ios::failbit);
            conditionalThrow();
         }
            // catches some errors we can encounter
         catch (FFStreamError& e)
         {
            e.addText("In record " +
                      gpstk::StringUtils::asString(recordNumber));
            e.addText("In file " + filename);
            e.addLocation(FILE_LOCATION);
            mostRecentException = e;
            clear();
            seekg(initialPosition);
            recordNumber = initialRecordNumber;
            setstate(std::ios::failbit);
            conditionalThrow();
         }
      }
         // this is if you throw an FFStream error in the above catch
         // block because the catch(...) below will mask it otherwise.
         // This also takes care of catching StringExceptions
      catch (gpstk::Exception &e)
      {
         GPSTK_RETHROW(e);
      }
      catch (std::ifstream::failure &e)
      {
            // setting failbit when catching FFStreamError can cause
            // this exception to be thrown. in this case, we don't want
            // to lose the exception info so only make a new exception
            // if this isn't a fail() case
         if (!fail())
         {
            mostRecentException = FFStreamError("ifstream::failure thrown: " +
                                                std::string(e.what()));
            mostRecentException.addText("In file " + filename);
            mostRecentException.addLocation(FILE_LOCATION);
         }
         conditionalThrow();
      }
      catch (std::exception &e)
      {
         mostRecentException = FFStreamError("std::exception thrown: " +
                                             std::string(e.what()));
         mostRecentException.addText("In file " + filename);
         mostRecentException.addLocation(FILE_LOCATION);
         setstate(std::ios::failbit);
         conditionalThrow();
      }
      catch (...)
      {
         mostRecentException = FFStreamError("Unknown exception thrown");
         mostRecentException.addText("In file " + filename);
         mostRecentException.addLocation(FILE_LOCATION);
         setstate(std::ios::failbit);
         conditionalThrow();
      }

   }  // End of method 'FFStream::tryFFStreamGet()'



      // the crazy double try block is so that no gpstk::Exception throws 
      // get masked, allowing all exception information (line numbers, text,
      // etc) to be retained.
   void FFStream::tryFFStreamPut(const FFData& rec)
      throw(FFStreamError, gpstk::StringUtils::StringException)
   {
         // Mark where we start in case there is an error.
      long initialPosition = tellg();
      unsigned long initialRecordNumber = recordNumber;
      clear();

      try
      {
         try
         {
            rec.reallyPutRecord(*this);
            recordNumber++;
         }
         catch (std::exception &e)
         {
               // if this is a stream failure, don't mask it and let the
               // later catch block handle it
            if (dynamic_cast<std::ifstream::failure*>(&e))
               throw;

               // the catch(FFStreamError) below will add file information
               // to this exception
            mostRecentException = FFStreamError("std::exception thrown: " +
                                                std::string(e.what()));
            mostRecentException.addLocation(FILE_LOCATION);
            setstate(std::ios::failbit);
            conditionalThrow();
         }
         catch (gpstk::StringUtils::StringException& e)  
         {
            e.addText("In record " +
                      gpstk::StringUtils::asString(recordNumber));
            e.addText("In file " + filename);
            e.addLocation(FILE_LOCATION);
            mostRecentException = e;
            seekg(initialPosition);
            recordNumber = initialRecordNumber;
            setstate(std::ios::failbit);
            conditionalThrow();
         } 
            // catches some errors we can encounter
         catch (FFStreamError& e)
         {
            e.addText("In record " +
                      gpstk::StringUtils::asString(recordNumber));
            e.addText("In file " + filename);
            e.addLocation(FILE_LOCATION);
            mostRecentException = e;
            seekg(initialPosition);
            recordNumber = initialRecordNumber;
            setstate(std::ios::failbit);
            conditionalThrow();
         }
      }
         // this is if you throw an FFStream error in the above catch
         // block because the catch(...) below will mask it otherwise.
         // This also takes care of catching StringExceptions
      catch (gpstk::Exception &e)
      {
         GPSTK_RETHROW(e);
      }
      catch (std::ifstream::failure &e)
      {
            // setting failbit when catching FFStreamError can cause
            // this exception to be thrown. in this case, we don't want
            // to lose the exception info so only make a new exception
            // if this isn't a fail() case
         if (!fail())
         {
            mostRecentException = FFStreamError("ifstream::failure thrown: " +
                                                std::string(e.what()));
            mostRecentException.addText("In file " + filename);
            mostRecentException.addLocation(FILE_LOCATION);
         }
         conditionalThrow();
      }
      catch (std::exception &e)
      {
         mostRecentException = FFStreamError("std::exception thrown: " +
                                             std::string(e.what()));
         mostRecentException.addText("In file " + filename);
         mostRecentException.addLocation(FILE_LOCATION);
         setstate(std::ios::failbit);
         conditionalThrow();
      }
      catch (...)
      {
         mostRecentException = FFStreamError("Unknown exception thrown");
         mostRecentException.addText("In file " + filename);
         mostRecentException.addLocation(FILE_LOCATION);
         setstate(std::ios::failbit);
         conditionalThrow();
      }

   }  // End of method 'FFStream::tryFFStreamPut()'



}  // End of namespace gpstk


