/// @file WeekSecond.hpp  Implement full week, mod week and seconds-of-week time
/// representation.

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

#ifndef GPSTK_WEEKSECOND_HPP
#define GPSTK_WEEKSECOND_HPP

#include "Week.hpp"
#include "TimeConstants.hpp"
#include "TimeSystem.hpp"

namespace gpstk
{
   /// This class encapsulates the "Full Week and Seconds-of-week"
   /// time representation.
   class WeekSecond : public Week
   {
   public:

         /**
          * @defgroup wsbo WeekSecond Basic Operations
          * Default and Copy Constructors, Assignment Operator and Destructor.
          */
         //@{

         /**
          * Default Constructor.
          * All elements are initialized to zero.
          */
      WeekSecond(unsigned int w = 0,
                 double s = 0.,
                 TimeSystem ts = TimeSystem::Unknown)
            : Week(w), sow(s)
      { timeSystem = ts; }

         /**
          * Copy Constructor.
          * @param right a reference to the WeekSecond object to copy
          */
      WeekSecond( const WeekSecond& right )
            : Week( right ), sow( right.sow )
      { timeSystem = right.timeSystem; }

         /**
          * Alternate Copy Constructor.
          * Takes a const TimeTag reference and copies its contents via
          * conversion to CommonTime.
          * @param right a const reference to the BasicTime object to copy
          * @throw InvalidRequest on over-/under-flow
          */
      WeekSecond( const TimeTag& right )
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
      WeekSecond( const CommonTime& right )
      {
         convertFromCommonTime( right );
      }

         /**
          * Assignment Operator.
          * @param right a const reference to the WeekSecond to copy
          * @return a reference to this WeekSecond
          */
      WeekSecond& operator=( const WeekSecond& right );

         /// Virtual Destructor.
      virtual ~WeekSecond()
      {}
         //@}

         // The following functions are required by TimeTag.

      virtual CommonTime convertToCommonTime() const;

      virtual void convertFromCommonTime( const CommonTime& ct );

      virtual bool isValid() const;

      virtual void reset();

      inline virtual unsigned int getDayOfWeek() const
      {
         return static_cast<unsigned int>(sow) / SEC_PER_DAY;
      }

      inline double getSOW() const { return sow; }

      /// @defgroup wsco WeekSecond Comparison Operators
      /// All comparison operators have a parameter "right" which corresponds
      /// to the WeekSecond object to the right of the symbol.
      /// All comparison operators are const and return true on success
      /// and false on failure.
         //@{
      bool operator==( const WeekSecond& right ) const;
      bool operator!=( const WeekSecond& right ) const;
      bool operator<( const WeekSecond& right ) const;
      bool operator>( const WeekSecond& right ) const;
      bool operator<=( const WeekSecond& right ) const;
      bool operator>=( const WeekSecond& right ) const;
         //@}

      double sow;
   };

}

#endif // GPSTK_WEEKSECOND_HPP
