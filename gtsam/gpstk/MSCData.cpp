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
 * @file MSCData.cpp
 * Monitor station coordinate file data
 */

#include <math.h>
#include "MSCData.hpp"
#include "MSCStream.hpp"

using namespace gpstk::StringUtils;
using namespace std;

namespace gpstk
{

   static const unsigned long SEC_YEAR = 
   static_cast<unsigned long>(365.25 * gpstk::SEC_PER_DAY);

   void MSCData::reallyPutRecord(gpstk::FFStream & ffs) const
      throw(std::exception, gpstk::FFStreamError, StringException)
   {
      MSCStream& strm = dynamic_cast<MSCStream&>(ffs);
      
      string line;

      if ( time == YDSTime::BEGIN_TIME )
      {
         line += string(7, ' ');
      }
      else
      {
         line += rightJustify(asString<long>(time.year), 4);
         line += rightJustify(asString<long>(time.doy), 3 , '0');
      }
      line += rightJustify(asString<long>(station), 5);
      line += leftJustify(mnemonic, 7);
      if ( refepoch == YDSTime::BEGIN_TIME )
      {
         line += string(14, ' ');
      }
      else
      {
         line += rightJustify(asString<long>(refepoch.year), 4);
         line += " ";
         line += rightJustify(asString<long>(refepoch.doy), 3, '0');
         line += " ";
         line += rightJustify(asString<double>(refepoch.sod), 5, '0');
      }
      if ( effepoch == YDSTime::BEGIN_TIME )
      {
         line += string(14, ' ');
      }
      else
      {
         line += rightJustify(asString<long>(effepoch.year), 4);
         line += " ";
         line += rightJustify(asString<long>(effepoch.doy), 3, '0');
         line += " ";
         line += rightJustify(asString<double>(effepoch.sod), 5, '0');
      }
      line += rightJustify(asString(coordinates[0], 3), 12);
      line += rightJustify(asString(coordinates[1], 3), 12);
      line += rightJustify(asString(coordinates[2], 3), 12);
      line += rightJustify(asString(velocities[0], 4), 7);            
      line += rightJustify(asString(velocities[1], 4), 7);
      line += rightJustify(asString(velocities[2], 4), 7);
      
      ffs << line << endl;
      strm.lineNumber++;
   }

   void MSCData::reallyGetRecord(gpstk::FFStream& ffs)
      throw(std::exception, gpstk::FFStreamError,
            gpstk::StringUtils::StringException)
   {
      MSCStream& strm = dynamic_cast<MSCStream&>(ffs);

      string currentLine;

      // YDSTime version of BEGINNING_OF_TIME
  
      strm.formattedGetLine(currentLine, true);
      int len = currentLine.length(); //90 for old; 104 for new

      if(len == 90) //old format
      {
         long year = asInt(currentLine.substr(0, 4));
         long day =  asInt(currentLine.substr(4, 3));
         time.year = year;
         time.doy = day;
         time.sod = 0.0;

         station = asInt(currentLine.substr(7, 5));
         mnemonic = currentLine.substr(12, 7);

         double epoch, intg, frac, sod;
         long doyy;
         // can't have DOY 0, so use doy + 1 when generating times
         epoch = asDouble(currentLine.substr(19, 7));
         frac = modf(epoch, &intg);
         doyy = (long)(frac * SEC_YEAR / gpstk::SEC_PER_DAY);
         sod = (frac * SEC_YEAR) - (doyy * gpstk::SEC_PER_DAY);
         refepoch = gpstk::YDSTime((long)intg, doyy+1, sod);
         
         epoch = asDouble(currentLine.substr(26, 7));
         frac = modf(epoch, &intg);
         doyy = (long)(frac * SEC_YEAR / gpstk::SEC_PER_DAY);
         sod = (frac * SEC_YEAR) - (doyy * gpstk::SEC_PER_DAY);
         effepoch = gpstk::YDSTime((long)intg, doyy+1, sod);
         
         coordinates[0] = asDouble(currentLine.substr(33, 12));
         coordinates[1] = asDouble(currentLine.substr(45, 12));
         coordinates[2] = asDouble(currentLine.substr(57, 12));
         
         velocities[0] = asDouble(currentLine.substr(69, 7));
         velocities[1] = asDouble(currentLine.substr(76, 7));
         velocities[2] = asDouble(currentLine.substr(83, 7));
      }
      else if(len == 104) //new format
      {
         if  ( ( currentLine.substr(0, 4) == string(' ', 4) ) ||
               ( currentLine.substr(4, 3) == string(' ', 3) ) ||
               ( asInt(currentLine.substr(0, 4)) == 0 )       ||
               ( asInt(currentLine.substr(4, 3)) == 0 ) )
         {
            time = YDSTime::BEGIN_TIME;
         }
         else
         {
            time = YDSTime( (long)asInt(currentLine.substr(0, 4)),
                            (long)asInt(currentLine.substr(4, 3)),
                            (double)0.0 );
         }
         
         station = asInt(currentLine.substr(7, 5));
         mnemonic = currentLine.substr(12, 7);

         if ( ( currentLine.substr(19, 4) == string(' ', 4) ) ||
              ( currentLine.substr(24, 3) == string(' ', 3) ) ||
              ( asInt(currentLine.substr(19, 4)) == 0 )       ||
              ( asInt(currentLine.substr(24, 3)) == 0 ) )
         {
            refepoch = YDSTime::BEGIN_TIME;
         }
         else
         {
            refepoch = YDSTime( (long)asInt(currentLine.substr(19, 4)),
                                (long)asInt(currentLine.substr(24, 3)),
                                asDouble(currentLine.substr(28,5)) );
         }
         if ( ( currentLine.substr(33, 4) == string(' ', 4) ) ||
              ( currentLine.substr(38, 3) == string(' ', 3) ) ||
              ( asInt(currentLine.substr(33, 4)) == 0 )       ||
              ( asInt(currentLine.substr(38, 3)) == 0 ) )
         {
            effepoch = YDSTime::BEGIN_TIME;
         }
         else
         {
            effepoch = YDSTime( (long)asInt(currentLine.substr(33, 4)),
                                (long)asInt(currentLine.substr(38, 3)),
                                asDouble(currentLine.substr(42,5)) );
         }

         coordinates[0] = asDouble(currentLine.substr(47, 12));
         coordinates[1] = asDouble(currentLine.substr(59, 12));
         coordinates[2] = asDouble(currentLine.substr(71, 12));
            
         velocities[0] = asDouble(currentLine.substr(83, 7));
         velocities[1] = asDouble(currentLine.substr(90, 7));
         velocities[2] = asDouble(currentLine.substr(97, 7));
      }
      else //non-valid or unreadable coords file format
      {
      }
   }

   Xvt MSCData::getXvt(const YDSTime& t)
      const throw(InvalidRequest)
   {
      try
      {
         //
         // Calculate the elapsed time between the reference time
         // and the time of interest in order to determine the 
         // total station drift.
         double dt = (t.convertToCommonTime() - refepoch.convertToCommonTime()) / SEC_YEAR;
         Xvt xvt;
         xvt.x = coordinates;
         xvt.v = velocities;
         xvt.clkbias = 0.0;
         xvt.relcorr = 0.0;
         xvt.clkdrift = 0.0;
         const Triple& drift = velocities;
      
            // compute the position given the total drift vectors
         xvt.x[0] += drift[0] * dt;
         xvt.x[1] += drift[1] * dt;
         xvt.x[2] += drift[2] * dt;
         return( xvt );
      }
      catch(InvalidRequest& ir)
      {
         GPSTK_RETHROW(ir);
      }
   } // end of MSCData::getXvt()


}
