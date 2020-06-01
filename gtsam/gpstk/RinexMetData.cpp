#pragma ident "$Id$"

/**
 * @file RinexMetData.cpp
 * Encapsulates RINEX 2 & 3 Met file data, including I/O.
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

#include "StringUtils.hpp"
#include "CivilTime.hpp"
#include "RinexMetHeader.hpp"
#include "RinexMetData.hpp"
#include "RinexMetStream.hpp"

using namespace gpstk::StringUtils;
using namespace std;

namespace gpstk
{
  const int RinexMetData::maxObsPerLine = 8;
  const int RinexMetData::maxObsPerContinuationLine = 10;

  void RinexMetData::reallyPutRecord(FFStream& ffs) const
    throw(std::exception, FFStreamError, 
          gpstk::StringUtils::StringException)
  {
    RinexMetStream& strm = dynamic_cast<RinexMetStream&>(ffs);
    string line;

    CivilTime civtime(time);

    // write the first line
    line += " ";
    line += rightJustify(asString<short>(civtime.year  ),2,'0');
    line += " ";
    line += rightJustify(asString<short>(civtime.month ),2);
    line += " ";
    line += rightJustify(asString<short>(civtime.day   ),2);
    line += " ";
    line += rightJustify(asString<short>(civtime.hour  ),2);
    line += " ";
    line += rightJustify(asString<short>(civtime.minute),2);
    line += " ";
    line += rightJustify(asString<short>(civtime.second),2);

    for (int i = 0;
         (i < int(strm.header.obsTypeList.size())) && (i < maxObsPerLine);
         i++)
    {
      RinexMetHeader::RinexMetType thistype = strm.header.obsTypeList[i];
      RinexMetMap::const_iterator itr = data.find(thistype);
      if (itr == data.end())
      {
        FFStreamError err("Couldn't find data for " + 
                          RinexMetHeader::convertObsType(strm.header.obsTypeList[i]));
        GPSTK_THROW(err);
      }
      line += rightJustify(asString((*itr).second,1),7);
    }

    // Do we need continuation lines?
    if (strm.header.obsTypeList.size() > maxObsPerLine)
    {
      for (size_t i = maxObsPerLine;
           i < strm.header.obsTypeList.size();
           i++)
      {
        if (((i - maxObsPerLine) % maxObsPerContinuationLine) == 0)
        {
          ffs << line << endl;
          strm.lineNumber++;
          line.clear();
          line += string(4,' ');
        }
        RinexMetHeader::RinexMetType thistype = strm.header.obsTypeList[i];
        RinexMetMap::const_iterator itr = data.find(thistype);
        if (itr == data.end())
        {
          FFStreamError err("Couldn't find data for " + 
                            RinexMetHeader::convertObsType(strm.header.obsTypeList[i]));
          GPSTK_THROW(err);
        }
        line += rightJustify(asString((*itr).second,1),7);
      }
    }

    ffs << line << endl;
    strm.lineNumber++;
  }

  void RinexMetData::reallyGetRecord(FFStream& ffs)
    throw(std::exception, FFStreamError, 
          gpstk::StringUtils::StringException)
  {
    RinexMetStream& strm = dynamic_cast<RinexMetStream&>(ffs);

    if(!strm.headerRead)
      strm >> strm.header;

    RinexMetHeader& hdr = strm.header;

    string line;
    data.clear();

    // this is to see whether or not we expect an EOF
    // when we read this next line
    if (hdr.obsTypeList.size() > maxObsPerLine)
      strm.formattedGetLine(line); 
    else
      strm.formattedGetLine(line, true);

    processFirstLine(line, hdr);

    time = parseTime(line);

    while (data.size() < hdr.obsTypeList.size())
    {
      if (hdr.obsTypeList.size() - data.size() < maxObsPerContinuationLine)
        strm.formattedGetLine(line, true);
      else
        strm.formattedGetLine(line);
      processContinuationLine(line, hdr);
    }

    if (data.size() != hdr.obsTypeList.size())
    {
      FFStreamError e("Incorrect number of records");
      GPSTK_THROW(e);
    }
  } 

  void RinexMetData::processFirstLine(const string& line,
                                      const RinexMetHeader& hdr)
    throw(FFStreamError)
  {
    try
    {
      for (int i = 0;
           (i < maxObsPerLine) && (i < int(hdr.obsTypeList.size()));
           i++)
      {
        int currPos = 7*i + 18;
        data[hdr.obsTypeList[i]] = asDouble(line.substr(currPos,7));
      }
    }
    catch (std::exception &e)
    {
      FFStreamError err("std::exception: " + string(e.what()));
      GPSTK_THROW(err);
    }
  }

  void RinexMetData::processContinuationLine(const string& line,
                                             const RinexMetHeader& hdr)
    throw(FFStreamError)
  {
    try
    {
      int currentElements = data.size();
      for (int i = currentElements;
           (i < (maxObsPerContinuationLine + currentElements)) &&
             (i < int(hdr.obsTypeList.size()));
           i++)
      {
        int currPos = 7*((i - maxObsPerLine) % maxObsPerContinuationLine) + 4;
        data[hdr.obsTypeList[i]] = asDouble(line.substr(currPos,7));
      }
    }
    catch (std::exception &e)
    {
      FFStreamError err("std::exception: " + string(e.what()));
      GPSTK_THROW(err);
    }
  }

  CommonTime RinexMetData::parseTime(const string& line) const
    throw(FFStreamError)
  {
    try
    {
      // According to the RINEX spec, any 2 digit year 80 or greater
      // is a year in the 1900s (1980-1999), under 80 is 2000s.
      const int YearRollover = 80;

      // check if the spaces are in the right place - an easy way to check
      // if there's corruption in the file
      if ( line.size() < 18  ||
           (line[0]  != ' ') ||
           (line[3]  != ' ') ||
           (line[6]  != ' ') ||
           (line[9]  != ' ') ||
           (line[12] != ' ') ||
           (line[15] != ' '))
      {
        FFStreamError e("Invalid time format");
        GPSTK_THROW(e);
      }

      int year, month, day, hour, min;
      double sec;

      year  = asInt(line.substr(1, 2));
      month = asInt(line.substr(3, 3));
      day   = asInt(line.substr(6, 3));
      hour  = asInt(line.substr(9, 3));
      min   = asInt(line.substr(12,3));
      sec   = asInt(line.substr(15,3));

      if (year < YearRollover)
      {
        year += 100;
      }
      year += 1900;

      CivilTime rv(year, month, day, hour, min, sec, TimeSystem::Any);
      return CommonTime(rv);
    }
    catch (std::exception &e)
    {
      FFStreamError err("std::exception: " + string(e.what()));
      GPSTK_THROW(err);
    }
  }

  void RinexMetData::dump(ostream& s) const
  {
    s << "  " << time << endl;

    RinexMetMap::const_iterator itr;
    for(itr = data.begin(); itr != data.end(); itr++)
    {
      s << "  " << RinexMetHeader::convertObsType((*itr).first)
        << " " << (*itr).second << endl;
    }
  }

}  // end of namespace
