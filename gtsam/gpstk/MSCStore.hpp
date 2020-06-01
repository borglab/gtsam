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
//  Copyright 2007, The University of Texas at Austin
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
 * @file MSCStore.hpp
 * Store Monitor Station coordinate information and return either the
 * raw information or the position at a given time based on integration 
 * of station velocity information over time since the epoch time. 
 */
 
#ifndef GPSTK_MSCSTORE_HPP
#define GPSTK_MSCSTORE_HPP

#include <iostream>
#include <string>
#include <list>
#include <map>

#include "FileStore.hpp"
#include "XvtStore.hpp"
#include "MSCData.hpp"
#include "MSCStream.hpp"
#include "MSCHeader.hpp"

namespace gpstk
{
   /** @addtogroup MSC */
   //@{
   
   /// Store Monitor Station coordinate information and return either the
   /// raw information or the position at a given time based on integration 
   /// of station velocity information over time since the epoch time. 
   class MSCStore : public XvtStore<std::string>,
                    public FileStore<gpstk::MSCHeader>
   {
   public:

      MSCStore()
         throw()
         : initialTime(CommonTime::END_OF_TIME), 
           finalTime(CommonTime::END_OF_TIME)
      {}

      virtual ~MSCStore()
      {}

      /// Returns the position, velocity, and clock offset of the indicated
      /// station in ECEF coordinates (meters) at the indicated time.
      /// @param[in] stationID id string
      /// @param[in] t the time to look up
      /// @return the Xvt of the station at time
      /// @throw InvalidRequest If the request can not be completed for any
      ///    reason, this is thrown. The teXvt may have additional
      ///    information as to why the request failed.
      Xvt getXvt(const std::string& stationID, const CommonTime& t)
         const throw( gpstk::InvalidRequest );

      /// Returns the position, velocity, and clock offset of the indicated
      /// station in ECEF coordinates (meters) at the indicated time.
      /// @param[in] stationIDno id number
      /// @param[in] t the time to look up
      /// @return the Xvt of the station at time
      /// @throw InvalidRequest If the request can not be completed for any
      ///    reason, this is thrown. The teXvt may have additional
      ///    information as to why the request failed.
      Xvt getXvt(unsigned long& stationIDno, const CommonTime& t)
         const throw( gpstk::InvalidRequest );


      /// A debugging function that outputs in human readable form,
      /// all data stored in this object.
      /// @param[in] s the stream to receive the output; defaults to cout
      /// @param[in] detail the level of detail to provide
      void dump(std::ostream& s = std::cout, short detail = 0)
         const throw();


      /// Edit the dataset, removing data outside the indicated time interval
      /// @param tmin defines the beginning of the time interval
      /// @param tmax defines the end of the time interval
      void edit(const CommonTime& tmin = CommonTime::BEGINNING_OF_TIME, 
                const CommonTime& tmax = CommonTime::END_OF_TIME )
         throw();

      /// Determine the earliest time for which this object can successfully 
      /// determine the Xvt for any station.
      /// @return The initial time
      /// @throw InvalidRequest This is thrown if the object has no data.
      CommonTime getInitialTime()
         const throw()
      {return initialTime;}

      
      /// Determine the latest time for which this object can successfully 
      /// determine the Xvt for any station.
      /// @return The final time
      /// @throw InvalidRequest This is thrown if the object has no data.
      CommonTime getFinalTime()
         const throw()
      {return finalTime;}

      bool velocityIsPresent()
         const throw()
      {return true;}

      /// Return time system
      TimeSystem getTimeSystem(void) const throw()
         { return TimeSystem::Any; }

      //---------------------------------------------------------------
      // FileStore interfaces
      //---------------------------------------------------------------
      void loadFile(const std::string& filename) 
         throw( FileMissingException );	 

      //---------------------------------------------------------------
      // Below are interfaces that are unique to this class (i.e. not 
      // in the parent class)
      //---------------------------------------------------------------
      /// Add an MSCData object to this collection.
      /// @param msc the MSCData to add
      /// @return true if monitor station coordinates were added, false otherwise
      bool addMSC(const MSCData& msc)
         throw();
      
      /// Remove all data from this collection.   
      void clear()
         throw()
      {edit(CommonTime::BEGINNING_OF_TIME, 
            CommonTime::BEGINNING_OF_TIME);}
      
      /// Get the number of MSCData objects in this collection.
      /// @return the number of MSCData records in the map
      unsigned size()
         const throw();
      
      /// Find an appropriate MSCData object for a given station and time.
      /// @param stationID ID of station of interest
      /// @param t time of interest
      /// @throw InvalidRequest object thrown when no ephemeris is found
      /// Note: There may be more than one MSCData object for a given
      /// station.  If so, findMSC( ) returns the MSCData object with the
      /// latest epoch time that is prior to t. 
      const MSCData& findMSC(const std::string& stationID, const CommonTime& t)
         const throw( gpstk::InvalidRequest );
      const MSCData& findMSC(const unsigned long stationID, const CommonTime& t) 
         const throw( gpstk::InvalidRequest );

      /// Add all MSCData to an existing list<MSCData>.
      /// @return the number of MSCData added.
      int addToList(std::list<MSCData>& v)
         const throw();

      /// Return a list of the station IDs for all MSCData objects
      /// currently stored. 
      std::list<std::string> getIDList();
         
      /// XvtStore interface
      bool hasVelocity() const throw() { return false; }

      /// XvtStore interface
      /// Return true if the given IndeXvtype is present in the store
      bool isPresent(const std::string& id) const throw()
         { return (mscMap.find(id) != mscMap.end()); }

   private:
      /// StaMSCMap is a list of MSCData objects for a particular station
      /// in order of their effective epoch
      typedef std::map<CommonTime, MSCData> StaMSCMap;
      typedef StaMSCMap::const_iterator SMMci;
      typedef StaMSCMap::iterator SMMi;
      
      /// MSCMap is a set of StaMSCMap objects for all stations
      typedef std::map<std::string, StaMSCMap> MSCMap;
      typedef MSCMap::const_iterator MMci;
      typedef MSCMap::iterator MMi;
      
      /// The map where all MSCData objects are stored.
      MSCMap mscMap;
      
      CommonTime initialTime; //< Time of the first MSCData object
      CommonTime finalTime;   //< Time of the last MSCData object
                           //< (N.B.: finalTime is irrelevant in the 
                           //<  current implementation as there is no
                           //<  "end of effectivity" for an MSCData object.
      
      static const double SEC_YEAR;
   }; // end class MSCStore
   //@}
   
} // namespace gpstk
#endif  // GPSTK_MSCSTORE_HPP
