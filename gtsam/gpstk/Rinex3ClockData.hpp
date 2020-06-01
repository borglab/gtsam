#pragma ident "$Id$"

/**
 * @file Rinex3Clockdata.hpp
 * Encapsulate RINEX3 clock data file, including I/O
 * See more at: ftp://igscb.jpl.nasa.gov/pub/data/format/rinex_clock.txt
 */

#ifndef GPSTK_RINEX3CLOCKDATA_HPP
#define GPSTK_RINEX3CLOCKDATA_HPP

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

//system
#include<map>
//GPSTk
#include "CommonTime.hpp"
#include "FFStream.hpp"
#include "Rinex3ClockBase.hpp"
#include "Rinex3ClockHeader.hpp"
#include "RinexSatID.hpp"

namespace gpstk
{
      /** @addtogroup Rinex3Clock */
      //@{

      /** This class models a RINEX3 clock data file.
       *
       * @sa gpstk::Rinex3ClockData and gpstk::Rinex3ClockStream.
       * @sa rinex_clk_test.cpp and rinex_clk_read_write.cpp for examples.
       */
   class Rinex3ClockData : public Rinex3ClockBase
   {
   public:

         /// A simple constructor
      Rinex3ClockData(): time(gpstk::CommonTime::BEGINNING_OF_TIME){}


         /// Destructor
      virtual ~Rinex3ClockData() {}

         ///< clock data type
      std::string type;
         ///< receiver or satellite name for which data are given
      std::string name;
         ///< the corresponding time to the clock data record
      CommonTime time;
         /// number of data values
      size_t numVal;
         ///< clock data
         ///< 0: clock bias (seconds).
         ///< 1: clock bias sigma[optional] (seconds).
         ///< 2: clock rate [optional] (dimensionless).
         ///< 3: clock rate sigma [optional] (dimensionless).
         ///< 4: clock acceleration [optional] (per second).
         ///< 5: clock acceleration sigma [optional] (per second).
      double data[6];

         // The next four lines is our common interface
         /// RinexObsData is a "data", so this function always returns true.
      virtual bool isData() const {return true;}

         /**
          * A Debug output function.
          * Dumps the time of observations and the IDs of the Sats
          * in the map.
          */
      virtual void dump(std::ostream& s) const;

   protected:
         /**
          * Writes a correctly formatted record from this data to stream \a s.
          * When printing comment records, you'll need to format them correctly
          * yourself.  This means making sure that "COMMENT" is at the end
          * of the line and that they're the correct length (<= 80 chrs).
          * Also make sure to correctly set the epochFlag to the correct
          * number for the type of header data you want to write.
          */
      virtual void reallyPutRecord(FFStream& s) const 
         throw(std::exception, FFStreamError,
               gpstk::StringUtils::StringException);
  
         /**
          * This functions obtains a RINEX Observation record from the given 
          * FFStream.
          * If there is an error in reading from the stream, it is reset
          * to its original position and its fail-bit is set.
          * Because of the Rinex Obs format, a RinexObsData record returned
          * might not have data in it.  Check the RinexSatMap for empty()
          * before using any data in it.
          * @throws StringException when a StringUtils function fails
          * @throws FFStreamError when exceptions(failbit) is set and
          *  a read or formatting error occurs.  This also resets the
          *  stream to its pre-read position.
          */
      virtual void reallyGetRecord(FFStream& s) 
         throw(std::exception, FFStreamError,
               gpstk::StringUtils::StringException);

   private:
         /// Writes the CommonTime object into RINEX format. If it's a bad time,
         /// it will return blanks.
      std::string writeTime(const CommonTime& dt) const
         throw(gpstk::StringUtils::StringException);

         /** This function constructs a CommonTime object from the given parameters.
          * @param line       the encoded time string found in the 
          *                   RINEX clock data record.
          */
      CommonTime parseTime(const std::string& line) const;

   }; // End of class Rinex3ClockData

      //@}

}  // End of namespace gpstk

#endif   // GPSTK_RINEX3CLOCKDATA_HPP
