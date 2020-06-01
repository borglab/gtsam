/// @file QZSWeekSecond.hpp  Define QZS week and seconds-of-week; inherits WeekSecond

#ifndef GPSTK_QZSWEEKSECOND_HPP
#define GPSTK_QZSWEEKSECOND_HPP

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

#include "WeekSecond.hpp"

namespace gpstk
{
   /// This class handles the week and seconds-of-week of the QZS TimeTag classes.
   /// The QZS week is specified by (GPS without the rollover)
   /// 16-bit ModWeek, rollover at 65535, bitmask 0xFFFF and epoch QZS_EPOCH_MJD
   class QZSWeekSecond : public WeekSecond
   {
   public:

      /// Constructor.
      QZSWeekSecond(unsigned int w = 0,
                       double s = 0.,
                       TimeSystem ts = TimeSystem::QZS)
         : WeekSecond(w,s)
      { timeSystem = ts; }

      /// Constructor from CommonTime
      QZSWeekSecond( const CommonTime& right )
      {
         convertFromCommonTime( right );
      }

      /// Destructor.
      ~QZSWeekSecond() {}

      
      /// Return the number of bits in the bitmask used to get the ModWeek from the
      /// full week.
      int Nbits(void) const
      {
         static const int n=16;
         return n;
      }

      /// Return the bitmask used to get the ModWeek from the full week.
      int bitmask(void) const
      {
         static const int bm=0xFFFF;
         return bm;
      }

      /// Return the Modified Julian Date (MJD) of epoch for this system.
      long MJDEpoch(void) const
      {
         static const long e=QZS_EPOCH_MJD;
         return e;
      }

      /// Return a string containing the characters that this class
      /// understands when printing times.
      virtual std::string getPrintChars() const
      {
         return "VIiwgP";
      }

      /// Return a string containing the default format to use in printing.
      virtual std::string getDefaultFormat() const
      {
         return "%I %g %P";
      }

      /// This function formats this time to a string.  The exceptions
      /// thrown would only be due to problems parsing the fmt string.
      virtual std::string printf(const std::string& fmt) const
      {
         try {
            using gpstk::StringUtils::formattedPrint;

            std::string rv = fmt;
            rv = formattedPrint( rv, getFormatPrefixInt() + "V",
                                 "Vu", getEpoch() );
            rv = formattedPrint( rv, getFormatPrefixInt() + "I",
                                 "Iu", week );
            rv = formattedPrint( rv, getFormatPrefixInt() + "i",
                                 "iu", getModWeek() );
            rv = formattedPrint( rv, getFormatPrefixInt() + "w",
                                 "wu", getDayOfWeek() );
            rv = formattedPrint( rv, getFormatPrefixFloat() + "g",
                                 "gf", sow );
            rv = formattedPrint( rv, getFormatPrefixInt() + "P",
                                 "Ps", timeSystem.asString().c_str() );
            return rv;
         }
         catch(gpstk::StringUtils::StringException& e)
         { GPSTK_RETHROW(e); }
      }

      /// This function works similarly to printf.  Instead of filling
      /// the format with data, it fills with error messages.
      virtual std::string printError(const std::string& fmt) const
      {
         try {
            using gpstk::StringUtils::formattedPrint;
            std::string rv = fmt;

            rv = formattedPrint( rv, getFormatPrefixInt() + "V",
                                 "Vs", "BadQZSepoch");
            rv = formattedPrint( rv, getFormatPrefixInt() + "I",
                                 "Is", "BadQZSfweek");
            rv = formattedPrint( rv, getFormatPrefixInt() + "i",
                                 "is", "BadQZSmweek");
            rv = formattedPrint( rv, getFormatPrefixInt() + "w",
                                 "wu", "BadQZSdow");
            rv = formattedPrint( rv, getFormatPrefixFloat() + "g",
                                 "gf", "BadQZSsow");
            rv = formattedPrint( rv, getFormatPrefixInt() + "P",
                                 "Ps", "BadQZSsys");
            return rv;
         }
         catch(gpstk::StringUtils::StringException& e)
         { GPSTK_RETHROW(e); }
      }

      /// Set this object using the information provided in \a info.
      /// @param info the IdToValue object to which this object shall be set.
      /// @return true if this object was successfully set using the
      ///  data in \a info, false if not.
      bool setFromInfo( const IdToValue& info )
      {
         using namespace gpstk::StringUtils;

         for( IdToValue::const_iterator i = info.begin(); i != info.end(); i++ )
         {
               // based on the character, we know what to do...
            switch ( i->first )
            {
               case 'V':
                  setEpoch( asInt( i->second ) );
                  break;
               case 'I':
                  week = asInt( i->second );
                  break;
               case 'i':
                  setModWeek( asInt( i->second ) );
                  break;
               case 'P':
                  timeSystem.fromString(i->second);
                  break;
               default:
                     // do nothing
                  break;
            };

         } // end of for loop

         return true;
      }

   }; // end class QZSWeekSecond

} // namespace

#endif // GPSTK_QZSWEEKSECOND_HPP
