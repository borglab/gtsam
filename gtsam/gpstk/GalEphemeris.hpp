/// @file GalEphemeris.hpp Encapsulates the Galileo broadcast ephemeris and clock.
/// Inherits OrbitEph, which does most of the work; this class adds health and
/// accuracy information, fit interval, ionospheric correction terms and data
/// flags.

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

#ifndef GPSTK_GALORBITEPH_HPP
#define GPSTK_GALORBITEPH_HPP

#include <string>
#include "Exception.hpp"
#include "CommonTime.hpp"
#include "OrbitEph.hpp"

namespace gpstk
{
   class GalEphemeris : public OrbitEph
   {
   public:
	   /// Default constuctor
      GalEphemeris(void) : fitDuration(4)
      {
         beginValid.setTimeSystem(TimeSystem::GAL);
         endValid.setTimeSystem(TimeSystem::GAL);
         ctToe.setTimeSystem(TimeSystem::GAL);
         ctToc.setTimeSystem(TimeSystem::GAL);
         transmitTime.setTimeSystem(TimeSystem::GAL);
      }

      /// Destructor
      virtual ~GalEphemeris(void) {}

      /// Create a copy of this object and return a pointer to it. This function
      /// overrides that in the base class.
      virtual GalEphemeris* clone(void) const
         { return new GalEphemeris(*this); }

      /// Returns true if the time, ct, is within the period of validity of
      /// this OrbitEph object.
      /// @throw Invalid Request if the required data has not been stored.
      virtual bool isValid(const CommonTime& ct) const;

      /// Return a string that will identify the derived class
      virtual std::string getName(void) const
         { return std::string("GalEphemeris"); }

      /// This function returns the health status of the SV.
      virtual bool isHealthy(void) const;

      /// Compute the accuracy in meters from the accuracy flag (URA).
      double getAccuracy(void) const
         { return accuracy; }

      /// adjustBeginningValidity determines the beginValid and endValid times.
      /// @throw Invalid Request if the required data has not been stored.
      virtual void adjustValidity(void);
      
      /// Dump the overhead information as a string containing a single line.
      /// @throw Invalid Request if the required data has not been stored.
      virtual std::string asString(void) const;

      /// Dump the orbit, etc information to the given output stream.
      /// @throw Invalid Request if the required data has not been stored.
      virtual void dumpBody(std::ostream& os = std::cout) const;

   // member data
      CommonTime transmitTime;   ///< Time of transmission
      long HOWtime;              ///< Time (seconds-of-week) of handover word (txmit)
      short IODnav;              ///< Index of data - Nav
      short health;              ///< Satellite health - a bit map
      double accuracy;           ///< Accuracy in meters or -1: as broadcast
      double Tgda;               ///< Ionospheric E5a/E1 data correction (seconds)
      double Tgdb;               ///< Ionospheric E5b/E1 data correction (seconds)
      short datasources;         ///< bit map
      short fitDuration;         ///< not in the BCE - set to a default

   }; // end class GalEphemeris

   //@}
   
} // end namespace

#endif // GPSTK_GALORBITEPH_HPP
