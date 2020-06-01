/// @file TimeString.cpp  print and scan using all TimeTag derived classes.

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

#include "TimeString.hpp"

#include "ANSITime.hpp"
#include "CivilTime.hpp"
#include "GPSWeekSecond.hpp"
#include "BDSWeekSecond.hpp"
#include "GALWeekSecond.hpp"
#include "QZSWeekSecond.hpp"
#include "GPSWeekZcount.hpp"
#include "JulianDate.hpp"
#include "MJD.hpp"
#include "UnixTime.hpp"
#include "YDSTime.hpp"

#include "TimeConverters.hpp"
#include "TimeConstants.hpp"

using namespace std;

namespace gpstk
{
   string printTime( const CommonTime& t,
                          const string& fmt )
   {
      try
      {
         string rv( fmt );
            // Convert to each TimeTag type and run its printf using rv, but
            // only if fmt contains the format characters of that type
         if(willPrintAs<ANSITime>(rv))
            rv = printAs<ANSITime>( t, rv );
         if(willPrintAs<CivilTime>(rv))
            rv = printAs<CivilTime>( t, rv );
         if(willPrintAs<GPSWeekSecond>(rv))
            rv = printAs<GPSWeekSecond>( t, rv );
         if(willPrintAs<GPSWeekZcount>(rv))
            rv = printAs<GPSWeekZcount>( t, rv );
         if(willPrintAs<JulianDate>(rv))
            rv = printAs<JulianDate>( t, rv );
         if(willPrintAs<MJD>(rv))
            rv = printAs<MJD>( t, rv );
         if(willPrintAs<UnixTime>(rv))
            rv = printAs<UnixTime>( t, rv );
         if(willPrintAs<YDSTime>(rv))
            rv = printAs<YDSTime>( t, rv );
         if(willPrintAs<GALWeekSecond>(rv))
            rv = printAs<GALWeekSecond>( t, rv );
         if(willPrintAs<BDSWeekSecond>(rv))
            rv = printAs<BDSWeekSecond>( t, rv );
         if(willPrintAs<QZSWeekSecond>(rv))
            rv = printAs<QZSWeekSecond>( t, rv );
      
         return rv;
      }
      catch( gpstk::StringUtils::StringException& se )
      {
         GPSTK_RETHROW( se );
      }
   }
   
      /// Fill the TimeTag object \a btime with time information found in
      /// string \a str formatted according to string \a fmt.
   void scanTime( TimeTag& btime,
                  const string& str,
                  const string& fmt )
   {
      try
      {
            // Get the mapping of character (from fmt) to value (from str).
         TimeTag::IdToValue info;
         TimeTag::getInfo( str, fmt, info );
         
         if( btime.setFromInfo( info ) )
         {
            return;
         }
         
            // Convert to CommonTime, and try to set using all formats.
         CommonTime ct( btime.convertToCommonTime() );
         scanTime( ct, str, fmt );

            // Convert the CommonTime into the requested format.
         btime.convertFromCommonTime( ct );
      }
      catch( gpstk::InvalidRequest& ir )
      {
         GPSTK_RETHROW( ir );
      }
      catch( gpstk::StringUtils::StringException& se )
      {
         GPSTK_RETHROW( se );
      }
   }
   
   void scanTime( CommonTime& t,
                  const string& str,
                  const string& fmt )
   {
      try
      {
         using namespace gpstk::StringUtils;

            // Get the mapping of character (from fmt) to value (from str).
         TimeTag::IdToValue info;
         TimeTag::getInfo( str, fmt, info );
         
            // These indicate which information has been found.
         bool hmjd( false ), hsow( false ), hweek( false ), hfullweek( false ),
            hdow( false ), hyear( false ), hmonth( false ), hday( false ),
            hzcount( false ), hdoy( false ), hzcount29( false ), 
            hzcount32( false ), hhour( false ), hmin( false ), hsec( false ),
            hsod( false ), hunixsec( false ), hunixusec( false ), 
            hepoch( false ), hansi( false ), hjulian( false ),
            hbdsw( false ), hqzsw( false ), hgalw( false ),
            hbdsfw( false ), hqzsfw( false ), hgalfw( false ),
            hbdse( false ), hqzse( false ), hgale( false);

            // These are to hold data that no one parses.
         int idow(0);
         TimeSystem ts;

         for( TimeTag::IdToValue::iterator itr = info.begin();
              itr != info.end(); itr++ )
         {
            switch( itr->first )
            {
               case 'P':
                  ts.fromString(itr->second);
                  t.setTimeSystem(ts);
                  break;

               case 'Q':
                  hmjd = true;
                  break;

               case 'Z':
                  hzcount = true;
                  break;

               case 's':
                  hsod = true;
                  break;

               case 'g':
                  hsow = true;
                  break;

               case 'w':
                  idow = asInt( itr->second );
                  hdow = true;
                  break;

               case 'G':
                  hweek = true;
                  break;

               case 'F':
                  hfullweek = true;
                  break;

               case 'j':
                  hdoy = true;
                  break;

               case 'b':
               case 'B':
                  hmonth = true;
                  break;

               case 'Y':
               case 'y':
                  hyear = true;
                  break;

               case 'a':
               case 'A':
                  {
                     hdow = true;
                     string thisDay = firstWord( itr->second );
                     lowerCase(thisDay);
                     if (isLike(thisDay, "sun.*")) idow = 0;
                     else if (isLike(thisDay, "mon.*")) idow = 1;
                     else if (isLike(thisDay, "tue.*")) idow = 2;
                     else if (isLike(thisDay, "wed.*")) idow = 3;
                     else if (isLike(thisDay, "thu.*")) idow = 4;
                     else if (isLike(thisDay, "fri.*")) idow = 5;
                     else if (isLike(thisDay, "sat.*")) idow = 6;
                     else
                     {
                        hdow = false;
                     }
                  }
                  break;
                  
               case 'm':
                  hmonth = true;
                  break;

               case 'd':
                  hday = true;
                  break;

               case 'H':
                  hhour = true;
                  break;

               case 'M':
                  hmin = true;
                  break;

               case 'S':
                  hsec = true;
                  break;

               case 'f':
                  hsec = true;
                  // a small hack to make fractional seconds work
                  info['S'] = info['f'];
                  break;

               case 'U':
                  hunixsec = true;
                  break;

               case 'u':
                  hunixusec = true;
                  break;
                  
               case 'c':
                  hzcount29 = true;
                  break;

               case 'C':
                  hzcount32 = true;
                  break;

               case 'J':
                  hjulian = true;
                  break;
                  
               case 'K':
                  hansi = true;
                  break;
                  
               case 'E':
                  hepoch = true;
                  break;

               case 'R': hepoch = hbdse = true; break;
               case 'T': hepoch = hgale = true; break;
               case 'V': hepoch = hqzse = true; break;

               case 'D': hfullweek = hbdsfw = true; break;
               case 'e': hweek = hbdsw = true; break;
               case 'L': hfullweek = hgalfw = true; break;
               case 'l': hweek = hgalw = true; break;
               case 'I': hfullweek = hqzsfw = true; break;
               case 'i': hweek = hqzsw = true; break;

               default:
                  {
                     // do nothing
                  }
                  break;

            };
         }     // end loop over Id/Value pairs

         if( hyear )
         {
            if( hmonth && hday )
            {
               CivilTime tt;
               tt.setFromInfo( info );
               if( hsod )
               {
                  convertSODtoTime( asDouble( info['s'] ), 
                                    tt.hour, tt.minute, tt.second );
               }
               t = tt.convertToCommonTime();
               return;
            }
            else  // use YDSTime as default
            {
               YDSTime tt;
               tt.setFromInfo( info );
               if( hhour && hmin && hsec )
               {
                  tt.sod = convertTimeToSOD( asInt( info['H'] ), 
                                             asInt( info['M'] ), 
                                             asDouble( info['S'] ) );
               }
               t = tt.convertToCommonTime();
               return;
            }

         } // end of if( hyear )

         if( hzcount32 ||
             (hfullweek && hzcount) ||
             (hepoch && (hzcount29 || 
                         (hweek && hzcount))) )
         {
            GPSWeekZcount tt;
            tt.setFromInfo( info );
            t = tt.convertToCommonTime();
            return;
         }

         if ( (hepoch && hweek) || hfullweek )
         {
            WeekSecond* ptt;
            if(hbdse || hbdsfw || hbdsw) ptt = new BDSWeekSecond();
            else if(hqzse || hqzsfw || hqzsw) ptt = new QZSWeekSecond();
            else if(hgale || hgalfw || hgalw) ptt = new GALWeekSecond();
            else ptt = new GPSWeekSecond();
            ptt->setFromInfo(info);
            if( hdow && !hsow )
            {
               ptt->sow = asInt( info['w'] ) * SEC_PER_DAY;
               if( hsod )
               {
                  ptt->sow += asDouble( info['s'] );
               }
               else if( hhour && hmin && hsec )
               {
                  ptt->sow += convertTimeToSOD( asInt( info['H'] ), 
                                              asInt( info['M'] ), 
                                              asDouble( info['S'] ) );
               }
            }
            t = ptt->convertToCommonTime();
            return;
         }

         if( hmjd )
         {
            MJD tt;
            tt.setFromInfo( info );
            t = tt.convertToCommonTime();
            return;
         }

         if( hjulian )
         {
            JulianDate tt;
            tt.setFromInfo( info );
            t = tt.convertToCommonTime();
            return;
         }

         if( hansi )
         {
            ANSITime tt;
            tt.setFromInfo( info );
            t = tt.convertToCommonTime();
            return;
         } 
         
         if( hunixsec || hunixusec )
         {
            UnixTime tt;
            tt.setFromInfo( info );
            t = tt.convertToCommonTime();
            return;
         }

         InvalidRequest ir("Incomplete time specification for readTime");
         GPSTK_THROW( ir );
      }
      catch( gpstk::StringUtils::StringException& se )
      {
         GPSTK_RETHROW( se );
      }
   }   

   void mixedScanTime( CommonTime& t,
                       const string& str,
                       const string& fmt )
   {
      try
      {
         using namespace gpstk::StringUtils;

            // Get the mapping of character (from fmt) to value (from str).
         TimeTag::IdToValue info;
         TimeTag::getInfo( str, fmt, info );
         
            // These indicate which information has been found.
         bool hsow( false ), hweek( false ), hfullweek( false ),
            hdow( false ), hyear( false ), hmonth( false ), hday( false ),
            hzcount( false ), hdoy( false ), hzcount29( false ), 
            hhour( false ), hmin( false ), hsec( false ),
            hsod( false ), hepoch( false ), hunixsec( false ),
            hunixusec( false ),
            hbdsw( false ), hqzsw( false ), hgalw( false ),
            hbdsfw( false ), hqzsfw( false ), hgalfw( false ),
            hbdse( false ), hqzse( false ), hgale( false);
#pragma unused(hunixsec,hunixusec)
            // MJD, Julian Date, ANSI time, Unix time, and 32-bit Zcounts
            // are treated as stand-alone types and are not mixed with others
            // if detected.
         
            // These variables will hold the values for use later.
         double isow, isod, isec;
         int iweek, ifullweek, idow, iyear, imonth, iday, izcount, idoy,
            izcount29, ihour, imin, iepoch;
         TimeSystem ts;

         for( TimeTag::IdToValue::iterator itr = info.begin();
              itr != info.end(); itr++ )
         {
            switch( itr->first )
            {
               case 'P':
                  ts.fromString(itr->second);
                  t.setTimeSystem(ts);
                  break;

               case 'Q':
                  t = MJD( asLongDouble(itr->second) );
                  return;

               case 'J':
                  t = JulianDate( asLongDouble(itr->second) );
                  return;
                  
               case 'C':
                  t = GPSWeekZcount().setZcount32( asInt(itr->second) );
                  return;

               case 'K':
                  t = ANSITime( asInt(itr->second) );
                  return;
                  
               case 'U':
               case 'u':
               {
                  UnixTime tt;
                  tt.setFromInfo( info );
                  t = tt.convertToCommonTime();
                  return;
               }
               break;

               case 'Z':
                  hzcount = true;
                  izcount = asInt(itr->second);
                  break;

               case 's':
                  hsod = true;
                  isod = asDouble(itr->second);
                  break;

               case 'g':
                  hsow = true;
                  isow = asDouble(itr->second);
                  break;

               case 'w':
                  idow = asInt(itr->second);
                  hdow = true;
                  break;

               case 'G':
                  hweek = true;
                  iweek = asInt(itr->second);
                  break;

               case 'F':
                  hfullweek = true;
                  ifullweek = asInt(itr->second);
                  break;

               case 'j':
                  hdoy = true;
                  idoy = asInt(itr->second);
                  break;

               case 'b':
               case 'B':
                  hmonth = true;
                  imonth = asInt(itr->second);
                  break;

               case 'Y':
               case 'y':
                  hyear = true;
                  iyear = asInt(itr->second);
                  break;

               case 'a':
               case 'A':
               {
                  hdow = true;
                  string thisDay = firstWord( itr->second );
                  lowerCase(thisDay);
                  if (isLike(thisDay, "sun.*")) idow = 0;
                  else if (isLike(thisDay, "mon.*")) idow = 1;
                  else if (isLike(thisDay, "tue.*")) idow = 2;
                  else if (isLike(thisDay, "wed.*")) idow = 3;
                  else if (isLike(thisDay, "thu.*")) idow = 4;
                  else if (isLike(thisDay, "fri.*")) idow = 5;
                  else if (isLike(thisDay, "sat.*")) idow = 6;
               }
               break;
                  
               case 'm':
                  hmonth = true;
                  imonth = asInt(itr->second);
                  break;

               case 'd':
                  hday = true;
                  iday = asInt(itr->second);
                  break;

               case 'H':
                  hhour = true;
                  ihour = asInt(itr->second);
                  break;

               case 'M':
                  hmin = true;
                  imin = asInt(itr->second);
                  break;

               case 'S':
                  hsec = true;
                  isec = asDouble(itr->second);
                  break;

               case 'f':
                  hsec = true;
                  isec = asDouble(itr->second);
                  break;

               case 'c':
                  hzcount29 = true;
                  izcount29 = asInt(itr->second);
                  break;

               case 'E':
                  hepoch = true;
                  iepoch = asInt(itr->second);
                  break;

               case 'R': hepoch = hbdse = true; iepoch = asInt(itr->second); break;
               case 'T': hepoch = hgale = true; iepoch = asInt(itr->second); break;
               case 'V': hepoch = hqzse = true; iepoch = asInt(itr->second); break;

               case 'D': hfullweek = hbdsfw = true; break;
               case 'e': hweek = hbdsw = true; break;
               case 'L': hfullweek = hgalfw = true; break;
               case 'l': hweek = hgalw = true; break;
               case 'I': hfullweek = hqzsfw = true; break;
               case 'i': hweek = hqzsw = true; break;

               default:
                     // do nothing
                  break;

            };
         }

         bool hbds(hbdse || hbdsfw || hbdsw);
         bool hgal(hgale || hgalfw || hgalw);
         bool hqzs(hqzse || hqzsfw || hqzsw);

            // We'll copy this time to 't' after all of the processing.
         CommonTime ct;
         ct.setTimeSystem(t.getTimeSystem());
         
            // Go through all of the types in order of least precise to most
            // precise.
         if( hepoch ) 
         {
            WeekSecond *ptt;
            if(hbds) ptt = new BDSWeekSecond(ct);
            else if(hqzs) ptt = new QZSWeekSecond(ct);
            else if(hgal) ptt = new GALWeekSecond(ct);
            else ptt = new GPSWeekSecond(ct);
            ptt->setEpoch( iepoch );
            ct = ptt->convertToCommonTime();
         }
         
         if( hyear )
         {
            YDSTime tt(ct);
            tt.year = iyear;
            ct = tt.convertToCommonTime();
         }
 
         if( hmonth )
         {
            CivilTime tt(ct);
            tt.month = imonth;
            ct = tt.convertToCommonTime();
         }

         if( hfullweek )
         {
            WeekSecond *ptt;
            if(hbds) ptt = new BDSWeekSecond(ct);
            else if(hqzs) ptt = new QZSWeekSecond(ct);
            else if(hgal) ptt = new GALWeekSecond(ct);
            else ptt = new GPSWeekSecond(ct);
            ptt->week = ifullweek;
            ct = ptt->convertToCommonTime();
         }
         
         if( hweek )
         {
            WeekSecond *ptt;
            if(hbds) ptt = new BDSWeekSecond(ct);
            else if(hqzs) ptt = new QZSWeekSecond(ct);
            else if(hgal) ptt = new GALWeekSecond(ct);
            else ptt = new GPSWeekSecond(ct);
            ptt->setModWeek(iweek);
            ct = ptt->convertToCommonTime();
         }
         
         if( hdow )
         {
            WeekSecond *ptt;
            if(hbds) ptt = new BDSWeekSecond(ct);
            else if(hqzs) ptt = new QZSWeekSecond(ct);
            else if(hgal) ptt = new GALWeekSecond(ct);
            else ptt = new GPSWeekSecond(ct);
            ptt->sow = static_cast<double>(idow) * SEC_PER_DAY;
            ct = ptt->convertToCommonTime();
         }
         
         if( hday )
         {
            CivilTime tt(ct);
            tt.day = iday;
            ct = tt.convertToCommonTime();
         }
         
         if( hdoy )
         {
            YDSTime tt(ct);
            tt.doy = idoy;
            ct = tt.convertToCommonTime();
         }
         
         if( hzcount29 )
         {
            GPSWeekZcount tt(ct);
            tt.setZcount29( izcount29 );
            ct = tt.convertToCommonTime(); 
         }

         if( hzcount )
         {
            GPSWeekZcount tt(ct);
            tt.zcount = izcount;
            ct = tt.convertToCommonTime();
         }

         if( hhour )
         {
            CivilTime tt(ct);
            tt.hour = ihour;
            ct = tt.convertToCommonTime();
         }

         if( hmin )
         {
            CivilTime tt(ct);
            tt.minute = imin;
            ct = tt.convertToCommonTime();
         }
         
         if( hsow )
         {
            WeekSecond *ptt;
            if(hbds) ptt = new BDSWeekSecond(ct);
            else if(hqzs) ptt = new QZSWeekSecond(ct);
            else if(hgal) ptt = new GALWeekSecond(ct);
            else ptt = new GPSWeekSecond(ct);
            ptt->sow = isow;
            ct = ptt->convertToCommonTime();
         }
         
         if( hsod )
         {
            YDSTime tt(ct);
            tt.sod = isod;
            ct = tt.convertToCommonTime();
         }

         if( hsec )
         {
            CivilTime tt(ct);
            tt.second = isec;
            ct = tt.convertToCommonTime();
         }
         
         t = ct;
      }
      catch( gpstk::StringUtils::StringException& se )
      {
         GPSTK_RETHROW( se );
      }
   }   

} // namespace gpstk
