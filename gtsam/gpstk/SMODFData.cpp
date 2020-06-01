#pragma ident "$Id$"

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

/**
 * @file SMODFData.cpp
 * smoothed measurement data file data
 */

#include "StringUtils.hpp"
#include "SMODFData.hpp"
#include "SMODFStream.hpp"
#include "YDSTime.hpp"
#include "TimeString.hpp"

using namespace gpstk::StringUtils;
using namespace std;

namespace gpstk
{
   const int SMODFData::SMO_LEN_ICD211 = 85;  ///< Length of an ICD-GPS-211 SMODF record
   const int SMODFData::SMO_LEN_LEGACY = 80;  ///< Length of a Legacy SMODF record
   const int SMODFData::BEGINGPS2DYEAR = 80;  ///< Beginning of the GPS Two Digit Year

   void SMODFData::reallyPutRecord(gpstk::FFStream& ffs) const 
      throw(std::exception, gpstk::FFStreamError,
            gpstk::StringUtils::StringException)
   {
      SMODFStream& strm = dynamic_cast<SMODFStream&>(ffs);
      
      // here's a hack - if you dont know what to write, assume ICD211
      if (strm.format == SMODFStream::undefined)
         strm.format = SMODFStream::icd211;
      
      string line;
      
      if (strm.format == SMODFStream::icd211)
      {
         line += rightJustify(asString<short>(static_cast<YDSTime>(time).year),4);
         line += rightJustify(asString<short>(static_cast<YDSTime>(time).doy),3,'0');
         line += rightJustify(asString(static_cast<YDSTime>(time).sod,7),13);
         line += rightJustify(asString<short>(PRNID),3);
         line += rightJustify(asString<long>(station),5);
         line += rightJustify(asString<short>(channel),2);
         line += rightJustify(asString<short>(type),1);
         
         if (type == 9)
            line += rightJustify(asString<short>(lol),1);
         else
            line += string(1, ' ');
         
         line += doub2funny(obs, 21, 2);
         line += doub2funny(stdDev, 12, 2);
         line += string(1, ' ');
         line += rightJustify(asString<short>(tempSource),1);
         line += rightJustify(asString<short>(pressSource),1);
         line += rightJustify(asString<short>(humidSource),1);
         line += rightJustify(asString(temp,1), 5);
         line += rightJustify(asString(pressure,1), 6);
         line += rightJustify(asString(humidity,1), 5);
         
      }
      else if (strm.format == SMODFStream::legacy)
      {
         line += printTime(time,"%02y%3j%12.6s");
         line += rightJustify(asString<short>(PRNID),3);
         line += rightJustify(asString<long>(station),5);
         line += rightJustify(asString<short>(channel),2);
         line += rightJustify(asString<short>(type),1);
         
         if (type == 9)
            line += rightJustify(asString<short>(lol),1);
         else
            line += string(1, ' ');
         
            // FIX this is actually D21.14, but because of the way
            // doub2for works, we prepend a space and use 20 characters
         line += string(1, ' ');
         line += doub2for(obs, 20, 2);
            // FIX same as above, only with D12.5
         line += string(1, ' ');
         line += doub2for(stdDev, 11, 2);
         line += string(1, ' ');

            // convert from 211-B/C to 211-A flags
         short wxsource = 0;
         if (tempSource == pressSource == humidSource)
            wxsource = tempSource;
         else if (tempSource && pressSource && humidSource)
         {
               // No missing data.  We can assume that there is some
               // default data at this point because the first test
               // eliminates the all-sources-same condition (all real,
               // all missing, all default), and this test guarantees
               // that there are no missing data mixed in with
               // real/default.  Only remaining possibility is some
               // real, some default.
            wxsource = 2;
         }
         else
         {
               // Mixed real and/or default data with missing.  We
               // can't really handle that.
            gpstk::FFStreamError err("Assertion failed: all weather data must be"
                              " either present or missing");
            if (tempSource == 0)
               err.addText("temperature data is missing");
            if (pressSource == 0)
               err.addText("pressure data is missing");
            if (humidSource == 0)
               err.addText("humidity data is missing");
            err.addText("unlisted weather measurements are present");
         }

         if (wxsource == 1)
            wxsource = 7;

         line += rightJustify(asString<short>(wxsource),1);
         line += rightJustify(asString(temp,1), 5);
         line += rightJustify(asString(pressure,1), 6);
         line += rightJustify(asString(humidity,1), 5);
      }
      else
      {
         gpstk::FFStreamError err("Unknown SMODF format: " + 
                           asString<unsigned long>(strm.format));
         err.addText("Make sure you specify the format of the data.");
         GPSTK_THROW(err);
      }
      
      ffs << line << endl;
      strm.lineNumber++;
   }

   void SMODFData::dump(ostream& s) const 
   {
      s << time << "  Station: " << station << "  Type: " << type 
        << "  PRN  " << PRNID  << endl;
   }

   void SMODFData::reallyGetRecord(gpstk::FFStream& ffs)
      throw(std::exception, gpstk::FFStreamError,
            gpstk::StringUtils::StringException)
   {
      SMODFStream& strm = dynamic_cast<SMODFStream&>(ffs);

      string str, currentLine;
      
      strm.formattedGetLine(currentLine, true);
      int len=currentLine.length();

         // determine the format of the ODBIF file by examining 
         // the record length
      if (strm.format == SMODFStream::undefined)
      {
         if (len == SMO_LEN_ICD211)
            strm.format = SMODFStream::icd211;
         else if (len == SMO_LEN_LEGACY)
            strm.format = SMODFStream::legacy;
         else
         {
            gpstk::FFStreamError e("Unreconized format");
            GPSTK_THROW(e);
         }
      }
      
      if (strm.format == SMODFStream::icd211)
      {
         if (len != SMO_LEN_ICD211)
         {
            gpstk::FFStreamError e("Bad 211 format line length: " + 
                            asString(len));
            GPSTK_THROW(e);
         }
         
            // blank out column 66 (in case this ODBIF file uses it 
            //   for some unauthorized purpose)
         currentLine[65] = ' ';
         
         if (currentLine[31]!='1')
            currentLine[31]='0';
         
            // Parse line and load apropriate values into ODBIF structure
         short year =     asInt(currentLine.substr( 0,  4));
         short DOY  =     asInt(currentLine.substr( 4,  3));
         double SOD =  asDouble(currentLine.substr( 7, 13));
         PRNID =          asInt(currentLine.substr(21,  2));
         station =        asInt(currentLine.substr(23,  5));
         channel =        asInt(currentLine.substr(28,  2));
         type =           asInt(currentLine.substr(30,  1));
         lol  =           asInt(currentLine.substr(31,  1));
         obs  =        for2doub(currentLine.substr(32, 21));
         stdDev =      for2doub(currentLine.substr(53, 12));
         tempSource =     asInt(currentLine.substr(66,  1));
         pressSource =    asInt(currentLine.substr(67,  1));
         humidSource =    asInt(currentLine.substr(68,  1));
         temp =        asDouble(currentLine.substr(69,  5));
         pressure =    asDouble(currentLine.substr(74,  6));
         humidity =    asDouble(currentLine.substr(80,  5));

         // Add some sanity checks on the data so we can detect if this is
         // *really* a SMOD file.
         if (DOY < 0 || DOY > 366 || SOD > 86400 || PRNID > 32 || 
             (type != 0 && type != 9) ||
             stdDev > 100 || stdDev <= 0)
         {
            gpstk::FFStreamError e(string("Bad 211 format data"));
            GPSTK_THROW(e);
         }

         
            // set the time
         time=YDSTime(year, DOY, SOD);
      }
      else if (strm.format == SMODFStream::legacy)
      {
         if (len != SMO_LEN_LEGACY)
         {
            gpstk::FFStreamError e("Bad legacy format line length: " + 
                            asString(len));
            GPSTK_THROW(e);
         }
         
            // blank out column 63 (in case this ODBIF file uses it 
            // for some unauthorized purpose)
         currentLine[62] = ' ';
         
         if (currentLine[28]!='1')
            currentLine[28]='0';
         
            // Parse line and load apropriate values into ODBIF structure
         short year =     asInt(currentLine.substr( 0,  2));
         short DOY  =     asInt(currentLine.substr( 2,  3));
         double SOD =  asDouble(currentLine.substr( 5, 12));
         PRNID =          asInt(currentLine.substr(17,  3));
         station =        asInt(currentLine.substr(20,  5));
         channel =        asInt(currentLine.substr(25,  2));
         type =           asInt(currentLine.substr(27,  1));
         lol  =           asInt(currentLine.substr(28,  1));
         obs  =        for2doub(currentLine.substr(29, 21));// len ??
         stdDev =      for2doub(currentLine.substr(50, 12));//start & 
         short src =      asInt(currentLine.substr(63,  1));
         temp =        asDouble(currentLine.substr(64,  5));
         pressure =    asDouble(currentLine.substr(69,  6));
         humidity =    asDouble(currentLine.substr(75,  5));

         // Add some sanity checks on the data so we can detect if this is
         // *really* a SMOD file.
         if (DOY < 0 || DOY > 366 || SOD > 86400 || PRNID > 32 || 
             (type != 0 && type != 9) ||
             stdDev > 100 || stdDev <= 0)
         {
            gpstk::FFStreamError e(string("Bad legacy format data"));
            GPSTK_THROW(e);
         }
         
            // set the time
         if ( year < BEGINGPS2DYEAR )
            year += 2000;
         else 
            year += 1900;
         time=YDSTime(year, DOY, SOD);
         
            /*
              Translate ODBIF (legacy) weather types to ICD-GPS-211 types
              Legacy   Meaning
              0     No met data
              1     OCS smoothed values
              2     OCS default values
              3     NIMA overridden at OCS (?)
              6     NIMA default weather values
              7     NIMA automated weather station (usual)
              8     Bad meteorological data
              other    Undefined
            */
         if ( src==1 || src==7 )
            tempSource = pressSource = humidSource = 1;
         else if ( src==2 || src==6 )
            tempSource = pressSource = humidSource = 2;
         else
            tempSource = pressSource = humidSource = 0;   
      }
   }   // end reallyGetRecord()

   string SMODFData::doub2funny(const double& num,
                                const std::string::size_type length,
                                const std::string::size_type expLen)
   {
         // Prepend a space if num > 0.
         // doub2sci() is supposed to do this, but it doesn't.
      string str((num >= 0 ? 1 : 0), ' ');
      str += doub2sci(num, length, expLen);
         // replace the 'e' or 'E' with 'D' as specified in the '211
      std::string::size_type idx = str.find_first_of("eE");
      if (idx != std::string::npos)
         str[idx] = 'D';
      return str;
   }

} // end namespace gpstk
