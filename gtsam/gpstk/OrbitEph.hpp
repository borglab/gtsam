/// @file OrbitEph.hpp Encapsulates the "least common denominator" orbit parameters
/// that determine a satellite ephemeris, that is, clock model, Kepler orbit elements
/// plus harmonic perturbations with time of ephemeris, satellite ID, and begin and
/// end times of validity.
/// Although it can also be used alone, this class is most often to be used as a base
/// class for a fuller implementation of the ephemeris and clock, by adding health
/// and accuracy information, fit interval, ionospheric correction terms and data
/// flags. It serves as the base class for broadcast ephemerides for GPS, QZSS,
/// Galileo, and BeiDou, with RINEX Navigation input, among others.

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

#ifndef GPSTK_ORBITEPH_HPP
#define GPSTK_ORBITEPH_HPP

#include <string>
#include "Exception.hpp"
#include "CommonTime.hpp"
#include "ObsID.hpp"
#include "SatID.hpp"
#include "Xvt.hpp"
//#include "Rinex3NavData.hpp"

namespace gpstk
{
   class OrbitEph
   {
   public:
	   /// Default constuctor
      OrbitEph(void) : dataLoadedFlag(false), dndot(0.0), Adot(0.0),
                       beginValid(CommonTime::END_OF_TIME),
                       endValid(CommonTime::BEGINNING_OF_TIME)
         { }

      /// Destructor
      virtual ~OrbitEph(void) {}

      /// Create a copy of this object and return a pointer to it. This function
      /// MUST be overridden in any derived class using the derived class name.
      virtual OrbitEph* clone(void) const
         { return new OrbitEph(*this); }

      /// Returns true if the time, ct, is within the period of validity of
      /// this OrbitEph object.
      /// @throw Invalid Request if the required data has not been stored.
      virtual bool isValid(const CommonTime& ct) const;

      /// Return true if orbit data have been loaded.
      /// Returns false if no data have been loaded.
      virtual bool dataLoaded(void) const
         { return dataLoadedFlag; }

      /// Return a string that will identify the derived class
      virtual std::string getName(void) const
         { return std::string("OrbitEph"); }

      /// This function returns the health status of the SV. OrbitEph has no health
      /// information, so it returns true; however the derived class should override
      /// this function, computing a meaningful health.
      virtual bool isHealthy(void) const
      {
         if(!dataLoadedFlag) GPSTK_THROW(InvalidRequest("Data not loaded"));
         return true;
      }

      /// Compute the satellite clock bias (seconds) at the given time
      /// @throw Invalid Request if the required data has not been stored.
      double svClockBias(const CommonTime& t) const;

      /// Compute the satellite clock drift (sec/sec) at the given time
      /// @throw Invalid Request if the required data has not been stored.
      double svClockDrift(const CommonTime& t) const;
      
      /// Compute satellite position at the given time.
      /// This implements equations of motion as defined in IS-GPS-200.
      /// (This code has its origins in 1980's FORTRAN code that has
      /// been ported to C, then C++, then became part of the gpstk.
      /// The original code was based on IS-GPS-200 Table 20-IV.
      /// In July 2013, the code was modified to conform to Table 30-II
      /// which includes additional time-dependent terms (A(dot) 
      /// and delta n(dot)) that are in CNAV but not in LNAV.  These
      /// changes should be backward compatible with LNAV as long as the 
      /// Adot and dndot variables are appropriately set to 0.0 by the 
      /// LNAV loaders.) 
      /// @throw Invalid Request if the required data has not been stored.
      Xvt svXvt(const CommonTime& t) const;

      /// Compute satellite relativity correction (sec) at the given time
      /// @throw Invalid Request if the required data has not been stored.
      double svRelativity(const CommonTime& t) const;

      /// adjustBeginningValidity determines the beginValid and endValid times.
      /// In OrbitEph it simply assumes a 4-hour fit interval; however the derived
      /// class should override this function, using an appropriate fit interval.
      /// @throw Invalid Request if the required data has not been stored.
      virtual void adjustValidity(void)
      {
         if(!dataLoadedFlag) GPSTK_THROW(InvalidRequest("Data not loaded"));
         beginValid = ctToe - 7200.0;
         endValid = ctToe + 7200.0;
      }
      
      /// Dump the overhead information as a string containing a single line.
      /// @throw Invalid Request if the required data has not been stored.
      virtual std::string asString(void) const;

      /// Utility routine for dumpBody(); return the time in the appropriate time
      /// system as a string.  Override for other than GPS time systems
      /// @param CommonTime t time to display
      /// @param bool showHead if true, print only a header (default false)
      virtual std::string timeDisplay(const CommonTime& t, bool showHead=false) const;

      /// Dump the overhead information to the given output stream.
      /// @throw Invalid Request if the required data has not been stored.
      virtual void dumpHeader(std::ostream& os = std::cout) const;

      /// Dump the orbit, etc information to the given output stream.
      /// @throw Invalid Request if the required data has not been stored.
      virtual void dumpBody(std::ostream& os = std::cout) const;

      /// Dump the object to the given output stream.
      /// @throw Invalid Request if the required data has not been stored.
      virtual void dump(std::ostream& os = std::cout) const
      {
         dumpHeader(os);
         dumpBody(os);
      }

      /// Define this OrbitEph by converting the given RINEX navigation data.
      /// NB this will be both overridden and called by the derived classes
      /// NB currently has fixes for MGEX data.
      /// @param rnd Rinex3NavData
      /// @return true if OrbitEph was defined, false otherwise
      //virtual bool load(const Rinex3NavData& rnd);

   // member data
     
      // overhead
      bool dataLoadedFlag; ///< True if data is present
      SatID satID;         ///< Define satellite system and specific SV
      ObsID obsID;         ///< Defines carrier and tracking code
      CommonTime ctToe;    ///< Ephemeris epoch

      // Clock information
      CommonTime ctToc;    ///< Clock Epoch
      double af0;          ///< SV clock error (sec)
      double af1;          ///< SV clock drift (sec/sec)
      double af2;          ///< SV clock drift rate (sec/sec**2)

      // Major orbit parameters
      double M0;           ///< Mean anomaly (rad)
      double dn;           ///< Correction to mean motion (rad/sec)
      double ecc;          ///< Eccentricity
      double A;            ///< Semi-major axis (m)
      double OMEGA0;       ///< Rt ascension of ascending node (rad)
      double i0;           ///< Inclination (rad)
      double w;            ///< Argument of perigee (rad)
      double OMEGAdot;     ///< Rate of Rt ascension (rad/sec)
      double idot;         ///< Rate of inclination angle (rad/sec)
      // Orbit parameters for modernized message
      double dndot;        ///< Rate of correction to mean motion (rad/sec/sec)
      double Adot;         ///< Rate of semi-major axis (m/sec)

      // Harmonic perturbations
      double Cuc;          ///< Cosine latitude (rad)
      double Cus;          ///< Sine latitude (rad)
      double Crc;          ///< Cosine radius (m)
      double Crs;          ///< Sine radius (m)
      double Cic;          ///< Cosine inclination (rad)
      double Cis;          ///< Sine inclination (rad)

      CommonTime beginValid;  ///< Time at beginning of validity
      CommonTime endValid;    ///< Time at end of fit validity

   }; // end class OrbitEph

   //@}
   
   /// Write OrbitEph to output stream
   std::ostream& operator<<(std::ostream& os, const OrbitEph& eph);

} // end namespace

#endif // GPSTK_ORBITEPH_HPP

