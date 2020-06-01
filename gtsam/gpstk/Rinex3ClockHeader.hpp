#pragma ident "$Id$"

/**
 * @file Rinex3ClockHeader.hpp
 * Encapsulate header of RINEX3 clock file, including I/O
 * See more at: ftp://igscb.jpl.nasa.gov/pub/data/format/rinex_clock.txt
 */

#ifndef GPSTK_RINEX3CLOCKHEADER_HPP
#define GPSTK_RINEX3CLOCKHEADER_HPP

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
//  Octavian Andrei - FGI ( http://www.fgi.fi ). 2008
//
//============================================================================

// system
#include <vector>
#include <list>
#include <map>
#include <iostream>
#include <iomanip>
// GPSTk
#include "CommonTime.hpp"
#include "FFStream.hpp"
#include "Rinex3ClockBase.hpp"
#include "Triple.hpp"
#include "RinexSatID.hpp"



namespace gpstk
{
      /** @addtogroup Rinex3Clock */
      //@{


      /// Holds the necessary data for one reference clock corresponding to
      /// one "ANALYSIS CLK REF" line
   struct RefClk
   {
         /// Name of the reference station
      std::string name;
            /// Constraints for the ref-clock (ms)
      double sigma;
   };


         /// Holds the data for the analysis clock references
         /// i.e., the "# OF CLK REF" lines
   struct RefClkRecord
   {
         /// Number of reference stations in file
      size_t nRef;
         /// List of reference clocks
      std::list<RefClk> clk;
         /// Ref-clock window in the file (sec)
      double refWin[2];

      RefClkRecord() : nRef(0)
      { refWin[0] = refWin[1] = 0.0; };
   };

      /// RINEX clock data types
   struct RinexClkType
   {
      std::string type;
      std::string description;
      RinexClkType() : type(std::string("UN")),
      description(std::string("Unknown or Invalid")) {}
      RinexClkType(std::string t, std::string d) :
      type(t),description(d) {}
   };


      /** This class models the header for a RINEX3 clock file.
       *
       * @sa gpstk::Rinex3ClockData and gpstk::Rinex3ClockStream.
       * @sa rinex_clk_test.cpp and rinex_clk_read_write.cpp for examples.
       */
   class Rinex3ClockHeader : public Rinex3ClockBase
   {
   public:

         /// A Simple Constructor.
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wreorder"
      Rinex3ClockHeader() :
         version(3.0), valid(false), timeFirst(CommonTime::BEGINNING_OF_TIME)
         {}
#pragma clang diagnostic pop

         /// Clear (empty out) header
      void clear(void);


         /**
          * @name Rinex3ClockHeaderFormatStrings
          * RINEX3 Clock Header Formatting Strings
          */
         //@{
      static const std::string versionString;       ///< "RINEX VERSION / TYPE"
      static const std::string runByString;         ///< "PGM / RUN BY / DATE"
      static const std::string commentString;       ///< "COMMENT"
      static const std::string numObsString;        ///< "SYS / # / OBS TYPES"
      static const std::string timeSystemString;    ///< "TIME SYSTEM ID"
      static const std::string leapSecondsString;   ///< "LEAP SECONDS"
      static const std::string sysDCBString;        ///< "SYS / DCBS APPLIED"
      static const std::string sysPCVString;        ///< "SYS / PCVS APPLIED"
      static const std::string dataTypesString;     ///< "# / TYPES OF DATA"
      static const std::string stationNameString;   ///< "STATION NAME / NUM"
      static const std::string calibrationClkString;///< "STATION CLK REF"
      static const std::string acNameString;        ///< "ANALYSIS CENTER"
      static const std::string numRefClkString;     ///< "# OF CLK REF"
      static const std::string analysisClkRefString;///< "ANALYSIS CLK REF"
      static const std::string numStationsString;   ///< "# OF SOLN STA / TRF"
      static const std::string solnStaNameString;   ///< "SOLN STA NAME / NUM"
      static const std::string numSatsString;       ///< "# OF SOLN SATS"
      static const std::string prnListString;       ///< "PRN LIST"
      static const std::string endOfHeader;         ///< "END OF HEADER"
         //@}

         /// Validity bits for the RINEX3 Clock Header
      enum validBits
      {
         versionValid         = 0x01,        ///< "RINEX VERSION / TYPE"
         runByValid           = 0x02,        ///< "PGM / RUN BY / DATE"
         commentValid         = 0x04,        ///< "COMMENT"                (optional)
         numObsValid          = 0x08,        ///< "SYS / # / OBS TYPES"
         timeSystemValid      = 0x010,       ///< "TIME SYSTEM ID"
         leapSecondsValid     = 0x020,       ///< "LEAP SECONDS"           (optional)
         sysDCBsValid         = 0x040,       ///< "SYS / DCBS APPLIED"
         sysPCVsValid         = 0x080,       ///< "SYS / PCVS APPLIED"
         dataTypesValid       = 0x0100,      ///< "# / TYPES OF DATA"
         stationNameValid     = 0x0200,      ///< "STATION NAME / NUM"
         calibrationClkValid  = 0x0400,      ///< "STATION CLK REF"
         acNameValid          = 0x0800,      ///< "ANALYSIS CENTER"
         numRefClkValid       = 0x01000,     ///< "# OF CLK REF"
         analysisClkRefValid  = 0x02000,     ///< "ANALYSIS CLK REF"
         numStationsValid     = 0x04000,     ///< "# OF SOLN STA / TRF"
         solnStaNameValid     = 0x08000,     ///< "SOLN STA NAME / NUM"
         numSatsValid         = 0x010000,    ///< "# OF SOLN SATS"
         prnListValid         = 0x020000,    ///< "PRN LIST"

         endValid = 0x080000000,          ///< "END OF HEADER"

            /// This mask is for all valid fields
         allValid = 0x08003FFFF

      };

         /** @name Standard RINEX clock data types
          */
         //@{
      static const RinexClkType UN;
      static const RinexClkType AR;
      static const RinexClkType AS;
      static const RinexClkType CR;
      static const RinexClkType DR;
      static const RinexClkType MS;
         //@}

      //static const std::vector<ObsID> StandardRinex3ObsIDs;
      //static std::vector<ObsID> RegisteredRinex3ObsIDs;


         /** @name Rinex3ClockHeaderValues
          */
         //@{
      double version;                        ///< RINEX3 VERSION & TYPE
      std::string fileType;                  ///< RINEX3 FILETYPE (Clock data etc)
      RinexSatID system;                     ///< The RINEX3 satellite system
      std::string fileProgram,               ///< The program used to generate this file
         fileAgency,                         ///< Who ran the program.
         date;                               ///< When the program was run.
      std::vector<std::string> commentList;  ///< Comments in header (optional)
      int numObsTyp;                         ///< Number of data types.
      std::vector<std::string> obsTypeList;  ///< List of observation types
      std::string timeSystem;                ///< Time system used
      int leapSeconds;                       ///< Leap second (optional)
      int numTyp;                            ///<
      std::vector<std::string> dataTypeList; ///< List of data types
      std::string clk0Name;                  ///< Name for the clocks
      std::string calName;                   ///< Calibration station identifier
      std::string ac;                        ///< Analysis center ID
      std::string acName;                    ///< Analysis center name
      CommonTime timeFirst;                  ///< First Epoch (CommonTime object)
      std::list<RefClkRecord> refClkList;    ///< List of all reference clock sets
      std::string trfName;                   ///< Terrestrial reference frame
      int numSta,                            ///< Number of station clocks
         numSVs;                             ///< Number of satellite clocks
      std::vector<std::string> clkNameList;  ///< Names of the sta/sat clocks
      std::vector<Triple> staCoordList;      ///< Coordinates of the stations (m)

      unsigned long valid; ///< Bits set when individual header members are present and valid

      bool isPGM;          ///< flag for PGM/RUN BY/DATE
      bool isAC;           ///< flag for AC/ACName

   //@}


         /// Destructor
      virtual ~Rinex3ClockHeader() {}

         // The next four lines is our common interface
         /// Rinex3ClockHeader is a "header" so this function
         /// always returns true.
      virtual bool isHeader() const {return true;}


         /**
          * This is a simple Debug output function.
          * It simply outputs the version, name and antenna number of this
          * RINEX3 clock header.
          */
      virtual void dump(std::ostream& s) const;


         /**
          * Parse a single header record, and modify valid accordingly.
          * Used by reallyGetRecord for both Rinex3ClockHeader and
          * Rinex3ClockData.
          */
      void ParseHeaderRecord(std::string& line)
         throw(FFStreamError);


         /// Return boolean : is this a valid Rinex clock header?
      bool isValid() const { return ((valid & allValid) == allValid); }


   protected:

         /**
          * outputs this record to the stream correctly formatted.
          */
      virtual void reallyPutRecord(FFStream& s) const
         throw(std::exception, FFStreamError, StringUtils::StringException);

         /**
          * This function retrieves the RINEX3 Clock Header from the given FFStream.
          * If an stream error is encountered, the stream is reset to its
          *  original position and its fail-bit is set.
          * @throws StringException when a StringUtils function fails
          * @throws FFStreamError when exceptions(failbit) is set and
          *  a read or formatting error occurs.  This also resets the
          *  stream to its pre-read position.
          */
      virtual void reallyGetRecord(FFStream& s)
         throw(std::exception, FFStreamError,StringUtils::StringException);

      friend class Rinex3ClockData;

   private:
         /// Converts the CommonTime \a dt into a Rinex3 Clock time
         /// string for the header
      std::string writeTime(const CommonTime& dt) const;

         /**
          * This function sets the time for this header.
          * It looks at \a line to obtain the needed information.
          */
      CommonTime parseTime(const std::string& line) const;

   }; // end class Rinex3ClockHeader

      //@}

}  // End of namespace gpstk

#endif   // GPSTK_RINEX3CLOCKHEADER_HPP
