/// @file GALWeekSecond.hpp  Define GAL week and seconds-of-week; inherits WeekSecond

#ifndef GPSTK_GALWEEKSECOND_HPP
#define GPSTK_GALWEEKSECOND_HPP

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
   /// This class handles the week and seconds-of-week of the GAL TimeTag classes.
   /// The GAL week is specified by
   /// 12-bit ModWeek, rollover at 4096, bitmask 0xFFF and epoch GAL_EPOCH_MJD
   class GALWeekSecond : public WeekSecond
   {
   public:

      /// Constructor.
      GALWeekSecond(unsigned int w = 0,
                       double s = 0.,
                       TimeSystem ts = TimeSystem::GAL) throw()
         : WeekSecond(w,s)
      { timeSystem = ts; }

      /// Constructor from CommonTime
      GALWeekSecond( const CommonTime& right )
      {
         convertFromCommonTime( right );
      }

      /// Destructor.
      ~GALWeekSecond() throw() {}
      
      /// Return the number of bits in the bitmask used to get the ModWeek from the
      /// full week.
      int Nbits(void) const
      {
         static const int n=12;
         return n;
      }

      /// Return the bitmask used to get the ModWeek from the full week.
      int bitmask(void) const
      {
         static const int bm=0xFFF;
         return bm;
      }

      /// Return the Modified Julian Date (MJD) of epoch for this system.
      long MJDEpoch(void) const
      {
         static const long e=GAL_EPOCH_MJD;
         return e;
      }

      /// Return a string containing the characters that this class
      /// understands when printing times.
      virtual std::string getPrintChars() const
      {
         return "TLlwgP";
      }

      /// Return a string containing the default format to use in printing.
      virtual std::string getDefaultFormat() const
      {
         return "%L %g %P";
      }

      /// This function formats this time to a string.  The exceptions
      /// thrown would only be due to problems parsing the fmt string.
      virtual std::string printf(const std::string& fmt) const
      {
         try {
            using gpstk::StringUtils::formattedPrint;

            std::string rv = fmt;
            rv = formattedPrint( rv, getFormatPrefixInt() + "T",
                                 "Tu", getEpoch() );
            rv = formattedPrint( rv, getFormatPrefixInt() + "L",
                                 "Lu", week );
            rv = formattedPrint( rv, getFormatPrefixInt() + "l",
                                 "lu", getModWeek() );
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

            rv = formattedPrint( rv, getFormatPrefixInt() + "T",
                                 "Ts", "BadGALepoch");
            rv = formattedPrint( rv, getFormatPrefixInt() + "L",
                                 "Ls", "BadGALfweek");
            rv = formattedPrint( rv, getFormatPrefixInt() + "l",
                                 "ls", "BadGALmweek");
            rv = formattedPrint( rv, getFormatPrefixInt() + "w",
                                 "wu", "BadGALdow");
            rv = formattedPrint( rv, getFormatPrefixFloat() + "g",
                                 "gf", "BadGALsow");
            rv = formattedPrint( rv, getFormatPrefixInt() + "P",
                                 "Ps", "BadGALsys");
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
               case 'T':
                  setEpoch( asInt( i->second ) );
                  break;
               case 'L':
                  week = asInt( i->second );
                  break;
               case 'l':
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

   }; // end class GALWeekSecond

} // namespace

#endif // GPSTK_GALWEEKSECOND_HPP
