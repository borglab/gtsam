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
 * @file ObsRngDev.hpp
 * Observed range deviation computation & storage.
 */


#ifndef OBSRNGDEV_HPP
#define OBSRNGDEV_HPP

#include <ostream>

#include "CommonTime.hpp"
#include "XvtStore.hpp"
#include "Exception.hpp"
#include "GPSEllipsoid.hpp"
#include "IonoModelStore.hpp"
#include "TropModel.hpp"
#include "ValidType.hpp"
#include "SatID.hpp"
#include "GNSSconstants.hpp"

namespace gpstk
{

/**
 * A single (one observation from one sv), Observed Range Deviation (ORD).
 * It contains all of the parameters that define an ORD and includes
 * metadata on ORD computation such as SV position and health.
 */
   class ObsRngDev
   {
   public:

      /**
       * default constructor.
       * Creates an empty, useless object to facilitate STL containers of this
       * object.
       */
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wreorder"
       ObsRngDev() throw() : wonky(false), obstime(CommonTime::END_OF_TIME) {};
#pragma clang diagnostic pop
      /**
       * constructor.
       * Creates an ORD, with no ionospheric correction and a default
       * trop correction.
       * \param prange the observed pseudorange
       * \param svid the SV being observed
       * \param time the time of the observation
       * \param rxpos the earth-centered, earth-fixed receiver position
       * \param eph a store of either broadcast or precise ephemerides
       * \param em an EllipsoidModel for performing range calculations
       * \param ion a store of nav based ionospheric models
       * \param fq the GPS frequency (L1 or L2) from which the obs was made
       * \param svTime true if prange is in SV time, false for RX time.
       */ 
      ObsRngDev(const double prange,
                const SatID& svid,
                const CommonTime& time,
                const Position& rxpos,
                const XvtStore<SatID>& eph,
                EllipsoidModel& em,
                bool svTime = false);
      /**
       * constructor.
       * Creates an ORD, applies a single-frequency nav-message based
       * ionospheric correction and a default trop correction.
       * \param prange the observed pseudorange
       * \param prn the PRN number of the observed SV
       * \param time the time of the observation
       * \param rxpos the earth-centered, earth-fixed receiver position
       * \param eph a store of either broadcast or precise ephemerides
       * \param em an EllipsoidModel for performing range calculations
       * \param ion a store of nav based ionospheric models
       * \param fq the GPS frequency (L1 or L2) from which the obs was made
       * \param svTime true if prange is in SV time, false for RX time.
       */ 
      ObsRngDev(const double prange,
                const SatID& svid,
                const CommonTime& time,
                const Position& rxpos,
                const XvtStore<SatID>& eph,
                EllipsoidModel& em,
                const IonoModelStore& ion,
                IonoModel::Frequency fq,
                bool svTime = false);
      /**
       * constructor.
       * Creates an ORD, applies no ionospheric correction and
       * a user-specified trop correction.
       * \param prange the observed pseudorange
       * \param prn the PRN number of the observed SV
       * \param time the time of the observation
       * \param rxpos the earth-centered, earth-fixed receiver position
       * \param eph a store of either broadcast or precise ephemerides
       * \param em an EllipsoidModel for performing range calculations
       * \param tm a TropModel for performing trop calculation
       * \param ion a store of nav based ionospheric models
       * \param fq the GPS frequency (L1 or L2) from which the obs was made
       * \param svTime true if prange is in SV time, false for RX time.
       */ 
      ObsRngDev(const double prange,
                const SatID& svid,
                const CommonTime& time,
                const Position& rxpos,
                const XvtStore<SatID>& eph,
                EllipsoidModel& em,
                const TropModel& tm,
                bool svTime = false);
   
      /**
       * constructor.
       * Creates an ORD, applies a single-frequency nav-message based
       * ionospheric correction and a user-specified trop correction.
       * \param prange the observed pseudorange
       * \param prn the PRN number of the observed SV
       * \param time the time of the observation
       * \param rxpos the earth-centered, earth-fixed receiver position
       * \param eph a store of either broadcast or precise ephemerides
       * \param em an EllipsoidModel for performing range calculations
       * \param tm a TropModel for performing trop calculation
       * \param ion a store of nav based ionospheric models
       * \param fq the GPS frequency (L1 or L2) from which the obs was made
       * \param svTime true if prange is in SV time, false for RX time.
       */ 
      ObsRngDev(const double prange,
                const SatID& svid,
                const CommonTime& time,
                const Position& rxpos,
                const XvtStore<SatID>& eph,
                EllipsoidModel& em,
                const TropModel& tm,
                const IonoModelStore& ion,
                IonoModel::Frequency fq,
                bool svTime = false);
   
      /**
       * constructor.
       * Creates an ORD, applies a dual-frequency ionospheric correction
       * and a default trop correction.
       * \param prange the observed pseudorange
       * \param prn the PRN number of the observed SV
       * \param time the time of the observation
       * \param rxpos the earth-centered, earth-fixed receiver position
       * \param eph a store of either broadcast or precise ephemerides
       * \param em an EllipsoidModel for performing range calculations
       * \param svTime true if prange is in SV time, false for RX time.
       */ 
      ObsRngDev(const double prange1,
                const double prange2,
                const SatID& svid,
                const CommonTime& time,
                const Position& rxpos,
                const XvtStore<SatID>& eph,
                EllipsoidModel& em,
                bool svTime = false,
                double gamma = GAMMA_GPS);
   
      /**
       * constructor.
       * Creates an ORD, applies a dual-frequency ionospheric correction
       * and a user-specified trop correction.
       * \param prange the observed pseudorange
       * \param prn the PRN number of the observed SV
       * \param time the time of the observation
       * \param rxpos the earth-centered, earth-fixed receiver position
       * \param eph a store of either broadcast or precise ephemerides
       * \param em an EllipsoidModel for performing range calculations
       * \param tm a TropModel for performing trop calculations
       * \param svTime true if prange is in SV time, false for RX time.
       */ 
      ObsRngDev(const double prange1,
                const double prange2,
                const SatID& svid,
                const CommonTime& time,
                const Position& rxpos,
                const XvtStore<SatID>& eph,
                const EllipsoidModel& em,
                const TropModel& tm,
                bool svTime = false,
                double gamma = GAMMA_GPS);
   
      /// destructor
      virtual ~ObsRngDev() throw() {}

      // get accessor methods ----------------------------------------------
      /**
       * returns the time of the SV observation
       * \return time of SV observation
       */
      const CommonTime& getTime() const throw() { return obstime; }

      /**
       * returns the observed SV's identifier
       * \return svid
       */
      SatID getSvID() const throw() { return svid; }

      /**
       * returns the SV azimuth angle (in degrees) in relation to the rx
       * \return SV azimuth angle
       */
      vfloat getAzimuth() const throw() { return azimuth; }

      /**
       * returns elevation (in degrees) of the SV in relation to the rx
       * \return SV elevation angle
       */
      vfloat getElevation() const throw() { return elevation; }

      /**
       * returns the 6-bit SV health bitfield from epehemeris, subframe 1
       * \return SV health bitfield
       */
      vshort getHealth() const throw() { return health; }

      /**
       * returns the Issue Of Data, Clock (IODC) from ephemeris, subframe 1
       * \return ephemeris IODC
       */
      vshort getIODC() const throw() { return iodc; }

      /**
       * returns the observed range deviation (ORD) (in meters)
       * \returns ORD
       */
      double getORD() const throw() { return ord; }

      /**
       * returns the ionospheric offset (in meters)
       * \returns ionospheric offset
       */
      vdouble getIono() const throw() { return iono; }

      /**
       * returns the tropospheric offset (in meters)
       * \returns tropospheric offset
       */
      vdouble getTrop() const throw() { return trop; }

      friend std::ostream& operator<<(std::ostream& s, 
                                      const ObsRngDev& r) throw();

      void applyClockOffset(double clockOffset)
      {ord -= clockOffset;}

   static bool debug;

   private:
      void computeOrd(double obs,
                      const Position& rxpos,
                      const XvtStore<SatID>& eph,
                      const EllipsoidModel& em,
                      bool svTime)
      {
         if (svTime) 
            computeOrdTx(obs, rxpos, eph, em);
         else 
            computeOrdRx(obs, rxpos, eph, em);
         return;
      }



      void computeOrdTx(double obs,
                        const Position& rxpos,
                        const XvtStore<SatID>& eph,
                        const EllipsoidModel& em);
   
      void computeOrdRx(double obs,
                        const Position& rxpos,
                        const XvtStore<SatID>& eph,
                        const EllipsoidModel& em);

      void computeTrop(const TropModel& tm);

   public:
      CommonTime obstime;           ///< time of SV observation
      SatID svid;                ///< PRN number of observed SV
      double ord;                ///< difference between expected and observed range
      unsigned wonky;            ///< A bitmask defined by the application to flag questionable data

      vfloat azimuth;            ///< SV azimuth
      vfloat elevation;          ///< SV elevation
      vshort health;             ///< SV health bitfield
      vshort iodc;               ///< ephemeris IODC
      vdouble rho;               ///< expected geometric range
      vdouble iono;              ///< iono correction (meters)
      vdouble trop;              ///< trop correction (meters)
   };
}
#endif
