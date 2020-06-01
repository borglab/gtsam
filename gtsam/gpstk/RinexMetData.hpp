#pragma ident "$Id$"

/**
 * @file RinexMetData.hpp
 * Encapsulates RINEX 2 & 3 Met file data, including I/O.
 */

#ifndef GPSTK_RINEXMETDATA_HPP
#define GPSTK_RINEXMETDATA_HPP

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

#include <map>

#include "CommonTime.hpp"
#include "FFStream.hpp"
#include "RinexMetBase.hpp"
#include "RinexMetHeader.hpp"

namespace gpstk
{
  /** @addtogroup RinexMet */
  //@{

  /**
   * This class stores, reads, and writes RINEX 2 & 3 Met records.
   * @warning When writing a RinexMetData, the RinexMetStream::headerData
   * must have the correct observation types set or else no data will
   * be written.
   *
   * @sa rinex_met_read_write.cpp for an example.
   * @sa rinex_met_test.cpp for an example.
   * @sa RinexMetStream.
   * @sa RinexMetHeader for information on writing RINEX 2 & 3 Met files.
   */

  class RinexMetData : public RinexMetBase
  {

  public:

    RinexMetData()
      : time(gpstk::CommonTime::BEGINNING_OF_TIME)
    {}

    /// RinexMetData is "data" so this function always returns true.
    virtual bool isData(void) const { return true; }

    /**
     * A debug output function.
     */
    virtual void dump(std::ostream& s) const;

    /// less-than operator, for use with STL sort()
    bool operator<(const RinexMetData& right) const
    { return (time < right.time); }

    /// A map for storing one line of observations,
    /// mapping the observation type to its value.
    typedef std::map<RinexMetHeader::RinexMetType, double> RinexMetMap;

    /** @name Rinex weather data
     */
    //@{
    CommonTime time;   ///< The time this data was recorded, in GPS time system.
    RinexMetMap data;  ///< The data itself in map form.
    //@}

    /// The maximum number of obs per line before you need a new line
    static const int maxObsPerLine;
    /// The max number of obs per continuation line before you need
    /// a new line.
    static const int maxObsPerContinuationLine;

  protected:

    /// Writes the met data to the file stream formatted correctly.
    void reallyPutRecord(FFStream& s) const
      throw(std::exception, FFStreamError,
            gpstk::StringUtils::StringException);

    /**
     * This function retrieves a RINEX 2 or 3 Met record from the given FFStream.
     * If an error is encountered reading from the stream, the stream is returned
     * to its original position and its fail-bit is set.
     * @throws StringException when a StringUtils function fails
     * @throws FFStreamError when exceptions(failbit) is set and a
     *         read or formatting error occurs.  This also resets the
     *         stream to its pre-read position.
     */
    virtual void reallyGetRecord(FFStream& s)
      throw(std::exception, FFStreamError,
            gpstk::StringUtils::StringException);

  private:

    /// Parses string \a line to get time and met data
    void processFirstLine(const std::string& line,
                          const RinexMetHeader& hdr)
      throw(FFStreamError);

    /// Parses string \a line to get data on continuation lines.
    void processContinuationLine(const std::string& line,
                                 const RinexMetHeader& hdr)
      throw(FFStreamError);

    /// Parses the time portion of a line into a CommonTime object.
    CommonTime parseTime(const std::string& line) const
      throw(FFStreamError);

  };  // class RinexMetData

  //@}

} // namespace


#endif
