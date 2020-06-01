/// @file ANSITime.hpp

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

#ifndef GPSTK_ANSITIME_HPP
#define GPSTK_ANSITIME_HPP

#include "TimeTag.hpp"
#include "TimeSystem.hpp"

namespace gpstk
{
      /**
       * This class encapsulates the "ANSITime (seconds since Unix epoch)" time
       * representation.
       */
   class ANSITime : public TimeTag
   {
   public:
         /**
          * @defgroup utbo ANSITime Basic Operations
          * Default and Copy Constructors, Assignment Operator and Destructor.
          */
         //@{

         /**
          * Default Constructor.
          * All elements are initialized to zero.
          */
     ANSITime( time_t t = 0,
               TimeSystem ts = TimeSystem::Unknown )
        : time(t)
      { timeSystem = ts; }

         /**
          * Copy Constructor.
          * @param right a reference to the ANSITime object to copy
          */
      ANSITime( const ANSITime& right )
	: time( right.time )
      { timeSystem = right.timeSystem; }

         /**
          * Alternate Copy Constructor.
          * Takes a const TimeTag reference and copies its contents via
          * conversion to CommonTime.
          * @param right a const reference to the BasicTime object to copy
          * @throw InvalidRequest on over-/under-flow
          */
      ANSITime( const TimeTag& right )
      {
         convertFromCommonTime( right.convertToCommonTime() );
      }

         /**
          * Alternate Copy Constructor.
          * Takes a const CommonTime reference and copies its contents via
          * the convertFromCommonTime method.
          * @param right a const reference to the CommonTime object to copy
          * @throw InvalidRequest on over-/under-flow
          */
      ANSITime( const CommonTime& right )
      {
         convertFromCommonTime( right );
      }

         /**
          * Assignment Operator.
          * @param right a const reference to the ANSITime to copy
          * @return a reference to this ANSITime
          */
      ANSITime& operator=( const ANSITime& right );

         /// Virtual Destructor.
      virtual ~ANSITime()
         throw()
      {}
         //@}

         // The following functions are required by TimeTag.
      virtual CommonTime convertToCommonTime() const;

      virtual void convertFromCommonTime( const CommonTime& ct );

         /// This function formats this time to a string.  The exceptions
         /// thrown would only be due to problems parsing the fmt string.
      virtual std::string printf( const std::string& fmt ) const;

         /// This function works similarly to printf. Instead of filling
         /// the format with data, it fills with error messages.
      virtual std::string printError( const std::string& fmt ) const;

         /**
          * Set this object using the information provided in \a info.
          * @param info the IdToValue object to which this object shall be set.
          * @return true if this object was successfully set using the
          *  data in \a info, false if not.
          */
      virtual bool setFromInfo( const IdToValue& info );

         /// Return a string containing the characters that this class
         /// understands when printing times.
      virtual std::string getPrintChars() const
      {
         return "KP";
      }

         /// Return a string containing the default format to use in printing.
      virtual std::string getDefaultFormat() const
      {
         return "%K %P";
      }

      virtual bool isValid() const;

      virtual void reset() ;

         /**
          * @defgroup utco ANSITime Comparison Operators
          * All comparison operators have a parameter "right" which corresponds
          *  to the ANSITime object to the right of the symbol.
          * All comparison operators are const and return true on success
          *  and false on failure.
          */
         //@{
      bool operator==( const ANSITime& right ) const;
      bool operator!=( const ANSITime& right ) const;
      bool operator<( const ANSITime& right ) const;
      bool operator>( const ANSITime& right ) const;
      bool operator<=( const ANSITime& right ) const;
      bool operator>=( const ANSITime& right ) const;
         //@}

      time_t time;
   };

} // namespace

#endif // GPSTK_ANSITIME_HPP
