/// @file QZSEphemeris.hpp Encapsulates the QZSS broadcast ephemeris and clock.
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

#ifndef GPSTK_QZSORBITEPH_HPP
#define GPSTK_QZSORBITEPH_HPP

#include <string>
#include "Exception.hpp"
#include "CommonTime.hpp"
#include "OrbitEph.hpp"

namespace gpstk
{
   class QZSEphemeris : public OrbitEph
   {
   public:
	   /// Default constuctor
      QZSEphemeris(void)
      {
         beginValid.setTimeSystem(TimeSystem::QZS);
         endValid.setTimeSystem(TimeSystem::QZS);
         ctToe.setTimeSystem(TimeSystem::QZS);
         ctToc.setTimeSystem(TimeSystem::QZS);
         transmitTime.setTimeSystem(TimeSystem::QZS);
      }

      /// Destructor
      virtual ~QZSEphemeris(void) {}

      /// Create a copy of this object and return a pointer to it. This function
      /// overrides that in the base class.
      virtual QZSEphemeris* clone(void) const
         { return new QZSEphemeris(*this); }

      /// Returns true if the time, ct, is within the period of validity of
      /// this OrbitEph object.
      /// @throw Invalid Request if the required data has not been stored.
      virtual bool isValid(const CommonTime& ct) const;

      /// Return a string that will identify the derived class
      virtual std::string getName(void) const
         { return std::string("QZSEphemeris"); }

      /// This function returns the health status of the SV.
      virtual bool isHealthy(void) const;

      /// Determine the health by signal, where
      ///    which = 5 4 3 2 1 as signal = L1C/A L2C L5 L1C LEX.
      /// Cf IS-QZSS 5.2.2.2.3 and Table 5.1.2-1 pg 50
      bool isHealthy(const int which) const;

      /// Compute the accuracy in meters from the accuracy flag (URA).
      double getAccuracy(void) const
         { return accuracy; }

      /// adjustBeginningValidity determines the beginValid and endValid times.
      /// @throw Invalid Request if the required data has not been stored.
      virtual void adjustValidity(void);
      
      /// Dump the orbit, etc information to the given output stream.
      /// @throw Invalid Request if the required data has not been stored.
      virtual void dumpBody(std::ostream& os = std::cout) const;

      /// Compute the fit duration in hours, and adjust the times of validity, given
      /// the fit interval flag.
      /// @param fitint fit interval flag
      void setFitIntervalFlag(const short fitint)
      {
         fitDuration = getFitInterval(fitint);
         adjustValidity();
      }

   // member data
      CommonTime transmitTime;   ///< Time of transmission
      long HOWtime;              ///< Time (seconds-of-week) of handover word (txmit)
      short IODE;                ///< Index of data - ephemeris
      short IODC;                ///< Index of data - clock
      short health;              ///< Satellite health - a flag
      double accuracy;           ///< Accuracy in meters
      double Tgd;                ///< Ionospheric L1/L2 data correction (seconds)

      short codeflags;           ///< L2 codes
      short L2Pdata;             ///< L2 P data flag
      short fitDuration;         ///< fit interval flag (0=2hrs, 1>4hrs) IS-QZSS 8.1.1
      short fitint;              ///< for RINEX

   private:
      /// Get the fit interval in hours from the fit interval flag
      static short getFitInterval(const short fitIntFlag)
         {
            if(fitIntFlag == 0) return 2;
            else return 4; // actually >4 hours
         }

   }; // end class QZSEphemeris

   //@}
   
} // end namespace

#endif // GPSTK_QZSORBITEPH_HPP
