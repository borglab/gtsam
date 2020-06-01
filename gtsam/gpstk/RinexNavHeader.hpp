#pragma ident "$Id$"



/**
 * @file RinexNavHeader.hpp
 * Encapsulate header of Rinex navigation file
 */

#ifndef GPSTK_RINEXNAVHEADER_HPP
#define GPSTK_RINEXNAVHEADER_HPP

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






#include "FFStream.hpp"
#include "RinexNavBase.hpp"

namespace gpstk
{
   /** @addtogroup RinexNav */
   //@{

      /**
       * This class models the RINEX NAV header for a RINEX NAV file.
       *
       * \sa rinex_nav_test.cpp and rinex_nav_read_write.cpp for examples.
       * \sa gpstk::RinexNavData and gpstk::RinexNavStream classes.
       */
   class RinexNavHeader : public RinexNavBase
   {
   public:
         /// Constructor
      RinexNavHeader(void) : valid(0), version(2.1) {}
      
         /// Destructor
      virtual ~RinexNavHeader() {}

         /// RinexNavHeader is a "header" so this function always returns true.
      virtual bool isHeader(void) const {return true;}

         /// This function dumps the contents of the header.
      virtual void dump(std::ostream& s) const;

         /// Tell me, Am I valid?
      unsigned long valid;

         /// These are validity bits used in checking the RINEX NAV header.
      enum validBits
      {
         versionValid = 0x01,        ///< Set if the RINEX version is valid.
         runByValid = 0x02,          ///< Set if the Run-by value is valid.
         commentValid = 0x04,        ///< Set if the comments are valid. Very subjective
         ionAlphaValid = 0x08,       ///< Set if the Ion Alpha value is valid.
         ionBetaValid = 0x010,       ///< Set if the Ion Beta value is valid.
         deltaUTCValid = 0x020,      ///< Set if the Delta UTC value is valid.
         leapSecondsValid = 0x040,   ///< Set if the Leap Seconds value is valid.
         endValid = 0x080000000,           ///< Set if the end value is valid.

            /// This bitset checks that all required header items are available
            /// for a Rinex 2.0 version file
         allValid20 = 0x080000003,
            /// This bitset checks that all required header items are available
            /// for a Rinex 2.1 version file
         allValid21 = 0x080000003,
            /// This bitset checks that all required header items are available
            /// for a Rinex 2.11 version file
         allValid211 = 0x080000003
      };

         /** @name HeaderValues
          */
         //@{
      double version;           ///< RINEX Version

      std::string fileType;            
      std::string fileProgram;
      std::string fileAgency;
      std::string date;
      std::vector<std::string> commentList;
      double ionAlpha[4];
      double ionBeta[4];
      double A0;
      double A1;
      long UTCRefTime;
      long UTCRefWeek;
      long leapSeconds;
         //@}

         /** @name FormattingStd::Strings
          */
         //@{
      static const std::string versionString; //"RINEX VERSION / TYPE"
      static const std::string runByString; //"PGM / RUN BY / DATE"
      static const std::string commentString; // "COMMENT"
      static const std::string ionAlphaString;  //"ION ALPHA"
      static const std::string ionBetaString;  //"ION BETA"
      static const std::string deltaUTCString; //"DELTA-UTC: A0,A1,T,W"
      static const std::string leapSecondsString; //"LEAP SECONDS"
      static const std::string endOfHeader;  //"END OF HEADER"
         //@}

   protected:
         /// Writes a correctly formatted record from this data to stream \a s.
      virtual void reallyPutRecord(FFStream& s) const
         throw(std::exception, FFStreamError, 
               gpstk::StringUtils::StringException);

         /**
          * This function reads the RINEX NAV header from the given FFStream.
          * If an error is encountered in reading from the stream, the stream
          * is reset to its original position and its fail-bit is set.
          * @throws StringException when a StringUtils function fails
          * @throws FFStreamError when exceptions(failbit) is set and
          *  a read or formatting error occurs.  This also resets the
          *  stream to its pre-read position.
          */
      virtual void reallyGetRecord(FFStream& s) 
         throw(std::exception, FFStreamError, 
               gpstk::StringUtils::StringException);
   }; // class RinexNavHeader

   //@}

} // namespace

#endif
