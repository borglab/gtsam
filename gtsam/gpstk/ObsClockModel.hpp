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

#ifndef OBSCLOCKMODEL_HPP
#define OBSCLOCKMODEL_HPP

#include <map>
#include "Exception.hpp"
#include "gps_constants.hpp"

#include "Stats.hpp"
#include "ClockModel.hpp"
#include "ORDEpoch.hpp"

/**
 * @file ObsClockModel.hpp
 * Yet another abstract class used to define an interface to a model that
 * accepts GPS observation datat and determines a clock model from it. It
 * mainly adds the ability to specify the characteristcs of the observations
 * that are to be accpeted into the model. It also defines a function that
 * accepts Observed Range Deviations and computes the mean of these (that
 * meet the selection criteria) as an estimate of the receiver clock.
 */


namespace gpstk
{
   class ObsClockModel : public ClockModel
   {
   public:
      enum SvMode
      {
         IGNORE,   ///< do not include ORDs from this SV
         HEALTHY,  ///< include ORDs from this SV if it is healthy
         ALWAYS    ///< always include ORDs from this SV
      };
      
      enum SvStatus
      {
         USED,      ///< ORD used in the clock bias computation
         MANUAL,    ///< ORD removed from computation by user request 
         SVHEALTH,  ///< ORD removed from computation because SV unhealthy
         ELEVATION, ///< ORD removed from computation because SV elevation < mask 
         WONKY,     ///< ORD removed due to being flagged wonky
         SIGMA      ///< ORD removed from computation because it was outlier
      };

      /// defines a store for each SV's #SvMode
      typedef std::map<SatID, SvMode> SvModeMap;

      /// defines a store for each SV's #SvStatus
      typedef std::map<SatID, SvStatus> SvStatusMap;

      ObsClockModel(double sigma = 2, double elmask = 0, SvMode mode = ALWAYS)
         : sigmam(sigma), elvmask(elmask), useWonkyData(false)
      {
         status.clear();
         setSvMode(mode);
      }

      virtual void addEpoch(const ORDEpoch& re) throw(gpstk::InvalidValue) = 0;

      // set accessor methods ----------------------------------------------   

      /**
       * set a SvMode for all SVs.
       * \param right #SvModeMap
       * \return a reference to this object
       */
      ObsClockModel& setSvModeMap(const SvModeMap& right) throw();

      /** 
       * set the SvMode for a particular SV.
       * \param prn Sv number of the SV
       * \param mode #SvMode for the SV
       * \return a reference to this object
       */
      ObsClockModel& setSvMode(const SatID& svid, const SvMode& mode) throw()
      { modes[svid] = mode; return *this; }
      
      /** 
       * set the SvMode for all SVs
       * \param mode #SvMode for the SVs
       * \return a reference to this object
       */
      ObsClockModel& setSvMode(const SvMode& mode) throw()
      {
         for(int prn = 1; prn <= gpstk::MAX_PRN; prn++)
         {
            SatID svid(prn, SatID::systemGPS);
            modes[svid] = mode;
         }
         return *this;
      }

      /**
       * set the sigma multiple for ORD stripping.
       * \param right sigma multiple value
       * \return a reference to this object
       */
      ObsClockModel& setSigmaMultiplier(double right) throw()
      { sigmam = right; return *this; }
   
      /**
       * set the elevation mask angle for ORD stripping.
       * \param right elevation mask angle value
       * \return a reference to this object
       */
      ObsClockModel& setElevationMask(double right) throw()
      { elvmask = right; return *this; }

      /** Set useWonkyData true and ords that are flagged as wonky
          will be included in any clock estimation calculations.
      **/
      ObsClockModel& setUseWonkyData(bool right) throw()
      { useWonkyData = right; return *this; }

      // get accessor methods ----------------------------------------------   
   
      /**
       * get the map indicating how each ORD was used in the bias computation.
       * \return a const reference to the #SvStatusMap
       */
      const SvStatusMap& getSvStatusMap() const throw()
      { return status; };

      /**
       * get the status of a particular ORD in the bias computation.
       * \param prn the PRN number indicating the ORD of interest
       * \return #SvStatus
       * \exception ObjectNotFound an ORD for that SV is not in the map
       */
      SvStatus getSvStatus(const SatID& svid) const throw(ObjectNotFound);

      /**
       * get the map indicating how to use each ORD in the bias computation.
       * \return a const reference to the #SvModeMap
       */
      const SvModeMap& getSvModeMap() const throw() { return modes; }

      /**
       * get how a particular ORD is to be used in the bias computation.
       * \param prn the Sv number indicating the mode of interest
       * \return #SvMode
       * \exception ObjectNotFound a mode for that SV is not in the map
       */
      SvMode getSvMode(const SatID& svid) const throw(ObjectNotFound);

      /**
       * returns the sigma multiple value used for ORD stripping.
       * \return sigma multiple
       */
      double getSigmaMultiplier() const throw() { return sigmam; } 

      /**
       * returns the elevation mask angle used for ORD stripping.
       * \return elevation mask angle
       */
      double getElevationMask() const throw() { return elvmask; }


      /**
       * return the current value of the userWonkyData flag.
       */
      bool getUseWonkyData() const throw()
      { return useWonkyData; }

      /// Computes an average of all ORD in the epoch that pass the elevation
      /// mask, and SvModeMap tests, removes those ORDS that exceede the sigmam
      /// value and returns the resulting statistics. This is effectivly a simple
      /// single epoch clock model.
      Stats<double> simpleOrdClock(const ORDEpoch& oe)
         throw(InvalidValue);

      virtual void dump(std::ostream& s, short detail=1) const throw();

      friend std::ostream& operator<<(std::ostream& s, const ObsClockModel& r)
      { r.dump(s, 0); return s; };
      
   protected:

      double sigmam;        ///< sigma multiple value for ORD stripping
      double elvmask;       ///< elevation mask angle for ORD stripping
      SvStatusMap status;   ///< map of ORD usage in bias computation
      SvModeMap modes;      ///< map of modes to use ORDs in bias computation
      bool useWonkyData;    ///< set to use ords that are flagged wonky
   };
}
#endif
