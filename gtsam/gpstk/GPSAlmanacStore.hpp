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
 * @file GPSAlmanacStore.hpp
 * Store GPS almanac information (i.e. like the data in subframes 4&5) and 
 * compute satellite Xvt based upon this data and the algorithms defined
 * in the IS-GPS-200.
 */
 
#ifndef GPSTK_GPSALMANACSTORE_HPP
#define GPSTK_GPSALMANACSTORE_HPP

#include <iostream>
#include <string>
#include <map>

#include "XvtStore.hpp"
#include "SatID.hpp"
#include "AlmOrbit.hpp"
#include "EngAlmanac.hpp"
#include "OrbElemStore.hpp"

namespace gpstk
{
   /** @defgroup ephemstore */
   //@{

   /// Store GPS almanac information (i.e. like the data in subframes 4&5) and 
   /// compute satellite Xvt based upon this data and the algorithms defined
   /// in the IS-GPS-200.
   class GPSAlmanacStore : public OrbElemStore
   {
   public:

      GPSAlmanacStore()
         throw()
         : initialTime(CommonTime::END_OF_TIME), 
           finalTime(CommonTime::BEGINNING_OF_TIME)
      {}

      virtual ~GPSAlmanacStore()
      {}
      
      /// Returns the position, velocity, and clock offset of the indicated
      /// object in ECEF coordinates (meters) at the indicated time.
      /// @param[in] id the object's identifier
      /// @param[in] t the time to look up
      /// @return the Xvt of the object at the indicated time
      /// @throw InvalidRequest If the request can not be completed for any
      ///    reason, this is thrown. The text may have additional
      ///    information as to why the request failed.
      virtual Xvt getXvt(const SatID& id, const CommonTime& t) 
         const throw( gpstk::InvalidRequest );


      /// Returns the position, velocity, and clock offset of the indicated
      /// object in ECEF coordinates (meters) at the indicated time.
      /// It differs from getXvt in that it uses a different search
      /// algorithm - It will use the most recently received almanac
      /// information, thereby imitating what a receiver would be doing
      /// in real-time.  If there is no almanac data for that object prior
      /// to the requested time, InvalidRequest will be thrown.
      /// @param[in] id the object's identifier
      /// @param[in] t the time to look up
      /// @return the Xvt of the object at the indicated time
      /// @throw InvalidRequest If the request can not be completed for any
      ///    reason, this is thrown.  The text may have additional
      ///    information as to why the request failed.
      virtual Xvt getXvtMostRecentXmit(const SatID id, const CommonTime& t)
         const throw( gpstk::InvalidRequest );
      

      /// A debugging function that outputs in human readable form,
      /// all data stored in this object.
      /// @param[in] s the stream to receive the output; defaults to cout
      /// @param[in] detail the level of detail to provide
      virtual void dump(std::ostream& s = std::cout, short detail = 0) 
         const throw();


      /// Edit the dataset, removing data outside the indicated time interval
      /// @param[in] tmin defines the beginning of the time interval
      /// @param[in] tmax defines the end of the time interval
      virtual void edit(const CommonTime& tmin, 
                        const CommonTime& tmax = CommonTime::END_OF_TIME)
         throw();

      /// Clear the dataset, meaning remove all data
      virtual void clear(void) throw()
      { uba.clear(); }

      /// Return time system (NB assumed always to be GPS)
      virtual TimeSystem getTimeSystem(void) const throw()
         { return TimeSystem::GPS; }

      /// Determine the earliest time for which this object can successfully 
      /// determine the Xvt for any object.
      /// @return The initial time
      /// @throw InvalidRequest This is thrown if the object has no data.
      virtual CommonTime getInitialTime()
         const throw()
      {return initialTime;}

      
      /// Determine the latest time for which this object can successfully 
      /// determine the Xvt for any object.
      /// @return The final time
      /// @throw InvalidRequest This is thrown if the object has no data.
      virtual CommonTime getFinalTime()
         const throw()
      {return finalTime;}

      virtual bool velocityIsPresent()
         const throw()
      {return true;}

      /// Return true if velocity data is present in the store
      virtual bool hasVelocity() const throw()
      { return true; }

      /// Return true if the given SatID is present in the store
      virtual bool isPresent(const SatID& sat) const throw()
      {
         if(uba.find(sat) == uba.end()) return false;
         return true;
      }

      //---------------------------------------------------------------
      // Below are interfaces that are unique to this class (i.e. not 
      // in the parent class)
      //---------------------------------------------------------------

      /// Returns the health of an SV for a particular time
      /// @param sat the satellite's SatID
      /// @param t the time to look up
      /// @return the SV health bits
      /// @throw InvalidRequest no data found in store
      short getSatHealth(const SatID sat, const CommonTime& t) 
         const throw( gpstk::InvalidRequest);

      bool addAlmanac(const AlmOrbit& alm) throw();

      bool addAlmanac(const EngAlmanac& alm) throw();

      /// gets the closest almanac for the given time and satellite id,
      /// closest being in the past or future and "closest" being defined
      /// in terms of almanc time of epoch.
      /// @param sat the satellite's SatID
      /// @param t the time of interest
      AlmOrbit findAlmanac(const SatID sat, const CommonTime& t) 
         const throw( gpstk::InvalidRequest );

      /// gets the most recent almanac for the given time and satellite id,
      /// most recent meaning it must have a transmit time before
      /// the specified time.
      /// @param sat the satellite's SatID
      /// @param t the time of interest
      AlmOrbit findMostRecentAlmanac(const SatID sat, const CommonTime& t)
         const throw( gpstk::InvalidRequest );

      /// returns all almanacs closest to t for all satellites
      AlmOrbits findAlmanacs(const CommonTime& t) 
         const throw( gpstk::InvalidRequest );

   protected:
      /// This is intended to just store weekly sets of unique EngAlmanacs
      /// for a single SV.  The key is ToA
      typedef std::map<CommonTime, AlmOrbit> EngAlmMap;

      /// This is intended to hold all unique EngEphemerises for each SV
      /// The key is the SatID of the SV.
      typedef std::map<SatID, EngAlmMap> UBAMap;

      /// The map where all EngAlmanacs are stored.
      UBAMap uba;

      CommonTime initialTime; //< Earliest Toa minus a half week
      CommonTime finalTime;   //< Last Toa plus a half week
      
   };

   //@}
}
#endif
