/// @file GPSWeekZcount.cpp

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

#include "GPSWeekZcount.hpp"
#include "TimeConstants.hpp"
#include "TimeConverters.hpp"

namespace gpstk
{
   GPSWeekZcount& GPSWeekZcount::operator=( const GPSWeekZcount& right )
   {
      GPSWeek::operator=(right);
      zcount = right.zcount;
      return *this;
   }

   CommonTime GPSWeekZcount::convertToCommonTime() const
   {
      try
      {
         int dow = zcount / ZCOUNT_PER_DAY;
         int jday = MJD_JDAY + GPS_EPOCH_MJD + ( 7 * week ) + dow;
         double sod = static_cast<double>( zcount % ZCOUNT_PER_DAY ) * 1.5;
         CommonTime ct;
         return ct.set( jday,
                        static_cast<long>( sod ),
                        sod - static_cast<long>( sod ),
                        timeSystem );
      }
      catch (InvalidParameter& ip)
      {
         InvalidRequest ir(ip);
         GPSTK_THROW(ir);
      }
   }

   void GPSWeekZcount::convertFromCommonTime( const CommonTime& ct )
   {
         /// This is the earliest CommonTime representable by GPSWeekZcount.
     static const CommonTime MIN_CT = GPSWeekZcount(0,0,TimeSystem::Any);

      if (ct < MIN_CT)
      {
         InvalidRequest ir("Unable to convert CommonTime to GPSWeekZcount.");
         GPSTK_THROW(ir);
      }

      long day, sod;
      double fsod;
      ct.get( day, sod, fsod, timeSystem );

         // find the number of days since the beginning of the GPS Epoch
      day -= MJD_JDAY + GPS_EPOCH_MJD;
         // find out how many weeks that is
      week = static_cast<int>( day / 7 );
         // find out what the day of week is
      day %= 7;

      zcount = static_cast<long>( day * ZCOUNT_PER_DAY )
         + static_cast<long>( static_cast<double>( sod + fsod ) / 1.5 );
   }

   std::string GPSWeekZcount::printf( const std::string& fmt ) const
   {
      try
      {
         using gpstk::StringUtils::formattedPrint;

         std::string rv = GPSWeek::printf( fmt );

         rv = formattedPrint( rv, getFormatPrefixInt() + "w",
                              "wu", getDayOfWeek() );
         rv = formattedPrint( rv, getFormatPrefixInt() + "z",
                              "zu", zcount );
         rv = formattedPrint( rv, getFormatPrefixInt() + "Z",
                              "Zu", zcount );
         rv = formattedPrint( rv, getFormatPrefixInt() + "c",
                              "cu", getZcount29() );
         rv = formattedPrint( rv, getFormatPrefixInt() + "C",
                              "Cu", getZcount32() );
         rv = formattedPrint( rv, getFormatPrefixInt() + "P",
                              "Ps", timeSystem.asString().c_str() );
         return rv;
      }
      catch( gpstk::StringUtils::StringException& exc )
      {
         GPSTK_RETHROW( exc );
      }
   }

   std::string GPSWeekZcount::printError( const std::string& fmt ) const
   {
      try
      {
         using gpstk::StringUtils::formattedPrint;

         std::string rv = GPSWeek::printError( fmt );

         rv = formattedPrint( rv, getFormatPrefixInt() + "w",
                              "ws", getError().c_str() );
         rv = formattedPrint( rv, getFormatPrefixInt() + "z",
                              "zs", getError().c_str() );
         rv = formattedPrint( rv, getFormatPrefixInt() + "Z",
                              "Zs", getError().c_str() );
         rv = formattedPrint( rv, getFormatPrefixInt() + "c",
                              "cs", getError().c_str() );
         rv = formattedPrint( rv, getFormatPrefixInt() + "C",
                              "Cs", getError().c_str() );
         rv = formattedPrint( rv, getFormatPrefixInt() + "P",
                              "Ps", getError().c_str() );
         return rv;
      }
      catch( gpstk::StringUtils::StringException& exc )
      {
         GPSTK_RETHROW( exc );
      }
   }

   bool GPSWeekZcount::setFromInfo( const IdToValue& info )
   {
      using namespace gpstk::StringUtils;

      for( IdToValue::const_iterator i = info.begin(); i != info.end(); i++ )
      {
            // based on the character, we know what to do...
         switch( i->first )
         {
            case 'w':
               zcount = asInt( i->second) * ZCOUNT_PER_DAY;
               break;

            case 'z':
            case 'Z':
               zcount = asInt( i->second );
               break;

            case 'c':
               setZcount29( asInt( i->second ) );
               break;

            case 'C':
               setZcount32( asInt( i->second ) );
               break;

            case 'P':
               timeSystem.fromString(i->second);
               break;

            default:
                  // do nothing
               break;
         };
      }

      return true;
   }

  //inline bool GPSWeekZcount::isValid() const
  //{
  //    return ( GPSWeek::isValid() &&
  //             zcount < ZCOUNT_PER_WEEK );
  //}

  //void GPSWeekZcount::reset()
  //{
  //    GPSWeek::reset();
  //    zcount = 0;
  //}

   GPSWeekZcount& GPSWeekZcount::setZcount29(unsigned int z)
   {
      setWeek10( (z >> 19) & bits10 );
      zcount = z & bits19;
      return *this;
   }

   GPSWeekZcount& GPSWeekZcount::setZcount32(unsigned int z)
   {
      week = z >> 19;
      zcount = z & bits19;
      return *this;
   }


} // namespace
