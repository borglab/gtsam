/// @file OrbitEphStore.hpp
/// Class for storing and/or computing position, velocity, and clock data using
/// tables of <SatID, <time, OrbitEph> >, initial and final times and search methods.
/// Note this class may be used as is, using OrbitEph rather than its descendants,
/// without health or accuracy information; however most likely the user will
/// define a class that inherits this and uses one the classes derived from OrbitEph,
/// for example GPSEphemerisStore and GPSEphemeris.

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

#ifndef GPSTK_ORBITEPHSTORE_HPP
#define GPSTK_ORBITEPHSTORE_HPP

#include <iostream>
#include <list>

#include "OrbitEph.hpp"
#include "Exception.hpp"
#include "SatID.hpp"
#include "CommonTime.hpp"
#include "XvtStore.hpp"
//#include "Rinex3NavData.hpp"

namespace gpstk
{
   /** @addtogroup ephemstore */
   //@{

   /// Class for storing and accessing an objects position, 
   /// velocity, and clock data. Also defines a simple interface to remove
   /// data that has been added.
   class OrbitEphStore : public XvtStore<SatID>
   {
   public:
      
      /// Default empty constructor. Derived classes may want to override this
      /// constructor in order to set the store's time system. If the store will be
      /// used only for satellites of a single time system, member timeSystem could
      /// be defined to that system; TimeSystem::Any is used here to allow store
      /// to hold satellites with differing time systems.
      OrbitEphStore()
         : initialTime(CommonTime::END_OF_TIME), 
           finalTime(CommonTime::BEGINNING_OF_TIME),
           strictMethod(true), onlyHealthy(false)
      {
         timeSystem = TimeSystem::Any;
         initialTime.setTimeSystem(timeSystem);
         finalTime.setTimeSystem(timeSystem);
      }

      /// Destructor
      virtual ~OrbitEphStore() { clear(); }

      /// Return a string that will identify the derived class
      virtual std::string getName(void) const
         { return std::string("OrbitEphStore"); }

      /// Returns the position, velocity, and clock offset of the indicated
      /// object in ECEF coordinates (meters) at the indicated time.
      /// @param[in] id satellite SatID
      /// @param[in] t the time to look up
      /// @return the Xvt of the satellite at the indicated time
      /// @throw InvalidRequest if the satellite is not stored or there are no
      ///        orbit elements at time t.
      virtual Xvt getXvt(const SatID& id, const CommonTime& t) const;

      /// Output summary of store data in human readable form, with detail:
      ///  0: Time limits and number of entries for entire store
      ///  1: Level 0 plus for each satellite: one line giving number and time limits
      ///  2: For each satellite: number of entries and time limits, followed by one
      ///     line summary per ephemeris with Toe, Toc, Key, and hours of validity
      ///  3: For each satellite: number of entries and time limits, followed by one
      ///     line summary per ephemeris with begin, Toe, Toc, and end valid times
      ///  other: For each satellite: number of entries and time limits, followed by
      ///     full dump using the OrbitEph->dump() function.
      /// @param[in] s the stream to receive the output; defaults to cout
      /// @param[in] detail the level of detail to provide
      virtual void dump(std::ostream& s=std::cout, short detail=0) const;

      /// Edit the dataset, removing data outside the indicated time interval
      /// @param[in] tmin defines the beginning of the time interval
      /// @param[in] tmax defines the end of the time interval
      virtual void edit(const CommonTime& tmin, 
                        const CommonTime& tmax = CommonTime::END_OF_TIME);

      /// Clear the dataset, meaning remove all data
      virtual void clear(void)
      {
         for(SatTableMap::iterator ui=satTables.begin(); ui!=satTables.end(); ui++) {
            TimeOrbitEphTable& toet = ui->second;
            toet.clear();
         } 

         satTables.clear();

         initialTime = CommonTime::END_OF_TIME;
         initialTime.setTimeSystem(timeSystem);
         finalTime = CommonTime::BEGINNING_OF_TIME;
         finalTime.setTimeSystem(timeSystem);
      }

      /// Return the earliest time in the store.
      /// @return The store initial time
      virtual CommonTime getInitialTime() const
      { return initialTime; }

      /// Return the latest time in the store.
      /// @return The store final time
      virtual CommonTime getFinalTime() const
      { return finalTime; }

      /// Return the earliest time in the store for the given satellite.
//TD make work for sat = -1, system
      /// @param sat Satellite (or system if sat.id = -1) of interest
      /// @return The store initial time
      virtual CommonTime getInitialTime(const SatID& sat) const
      {
         if(satTables.find(sat) == satTables.end())
            return CommonTime::END_OF_TIME;

         CommonTime limit(CommonTime::END_OF_TIME);
         const TimeOrbitEphTable& table = getTimeOrbitEphMap(sat);
         TimeOrbitEphTable::const_iterator it;
         for (it = table.begin(); it != table.end(); ++it) {
            CommonTime ct(it->first);
            ct.setTimeSystem(timeSystem);
            if(ct < limit) limit = ct;
         }
         return limit;
      }

      /// Return the latest time in the store for the given satellite.
      /// @param sat Satellite (or system if sat.id = -1) of interest
      /// @return The store final time
      virtual CommonTime getFinalTime(const SatID& sat) const
      {
         if(satTables.find(sat) == satTables.end())
            return CommonTime::END_OF_TIME;

         CommonTime limit(CommonTime::BEGINNING_OF_TIME);
         const TimeOrbitEphTable& table = getTimeOrbitEphMap(sat);
         TimeOrbitEphTable::const_iterator it;
         for (it = table.begin(); it != table.end(); ++it) {
            CommonTime ct(it->first);
            ct.setTimeSystem(timeSystem);
            if(ct > limit) limit = ct;
         }
         return limit;
      }

      /// Return true if velocity data is present in the store
      virtual bool hasVelocity() const
      { return true; }

      // deprecated, same as hasVelocity()
      virtual bool velocityIsPresent() const
      { return hasVelocity(); }

      /// Return true if the given SatID is present in the store
      virtual bool isPresent(const SatID& sat) const
      {
         if(satTables.find(sat) != satTables.end()) return true;
         return false;
      }

      /// Return the time system of the store. This may be redefined in the 
      /// default constructor by the derived class.
      virtual TimeSystem getTimeSystem(void) const
      { return timeSystem; }

      //---------------------------------------------------------------
      // This ends the XvtStore<SatID> interface. Below are interfaces that are
      // unique to this class (i.e. not in the parent class XvtStore<SatID>)
      //---------------------------------------------------------------

      /// Get the number of OrbitEph objects in this collection for all satellites.
      /// @return the number of OrbitEph records in the map for all satellites.
      unsigned size(void) const;

      /// Get the number of OrbitEph objects in this store for the given satellite,
      /// or if sat.id == -1, the total for all satellites in this system.
      /// @return the number of OrbitEph records stored for the given satellite.
      unsigned size(const SatID& sat) const;

      /// Add an OrbitEph object to this collection.
      /// @param eph pointer to the OrbitEph to add
      /// @return pointer to new OrbitEph if it successful, NULL otherwise
      virtual OrbitEph* addEphemeris(const OrbitEph* eph);

      /// Add an OrbitEph object to this collection, converting the given RINEX
      /// navigation data.
      /// @param rnd Rinex3NavData
      /// @return pointer to the new object, NULL if data could not be added.
      //virtual OrbitEph* addEphemeris(const Rinex3NavData& rnd);

      /// Return true if OrbitEph with the same sat and time already exists in table.
      bool isPresent(const SatID& sat, const CommonTime& t) const
      {
         if(satTables.find(sat) == satTables.end())
            return false;
         const TimeOrbitEphTable& table = getTimeOrbitEphMap(sat);
         if(table.find(t) == table.end())
            return false;
         return true;
      }

      /// Explanation of find() function for OrbitEphStore
      /// The findUserOrbitEph() funtion does the best possible job of emulating the
      /// choice that would be made by a real-time user following the regardless
      /// of health status. It is suggested that the user call rationalize()
      /// to adjust the begin/end times of validity after loading all the elements
      /// and before using the store. There exists a second find fuction,
      /// findNearOrbitEph() that is provided for compatibility with past uses
      /// of the GPSEphemerisStore class. findNearOrbitEph() MAY return elements
      /// that are outside the range of validity and therefore need to be used
      /// with caution. Therefore, use findNearOrbitEph() only directly, carefully
      /// examining the resulting set of orbital elements and make an informed
      /// decision before using the OrbitEph.getXvt() functions.  
      /// @param sat SatID of satellite of interest
      /// @param t time with which to search for OrbitEph
      /// @return a pointer to the desired OrbitEph, or NULL if no OrbitEph found.
      virtual const OrbitEph* findUserOrbitEph(const SatID& sat, const CommonTime& t)
         const;

      /// Find an OrbitEph for the indicated satellite at time t. The OrbitEph
      /// chosen is the one with HOW time closest to the time t, (i.e. with
      /// smallest fabs(t-HOW), but still within the fit interval.
      /// @param sat the satellite of interest
      /// @param t the time of interest
      /// @return a pointer to the desired OrbitEph, or NULL if no OrbitEph found.
      virtual const OrbitEph* findNearOrbitEph(const SatID& sat, const CommonTime& t)
         const;

      /// Find an OrbitEph for the indicated satellite at time t, using the find...()
      /// routine appropriate for the current search method.
      /// @param sat the satellite of interest
      /// @param t the time of interest
      /// @return a pointer to the desired OrbitEph, or NULL if no OrbitEph found.
      const OrbitEph* findOrbitEph(const SatID& sat, const CommonTime& t) const
      {
         return (strictMethod ? findUserOrbitEph(sat,t) : findNearOrbitEph(sat, t));
      }

      /// Add all ephemerides to an existing list<OrbitEph>.
      /// If SatID sat is given, limit selections to sat's satellite system, plus if
      /// sat's id is not -1, limit to sat's id as well.
      /// @return the number of ephemerides added.
      virtual int addToList(std::list<OrbitEph*>& v,
                           SatID sat=SatID(-1,SatID::systemUnknown)) const;

      /// use findNearOrbitEph() in getXvt() and getSatHealth()
      void SearchNear(void)
      { strictMethod = false; }

      /// use findUserOrbitEph() in getXvt() and getSatHealth() (the default)
      void SearchUser(void)
      { strictMethod = true; }

      /// get the flag that limits getXvt() to healthy ephemerides
      bool getOnlyHealthyFlag(void) const
      { return onlyHealthy; }

      /// set the flag that limits getXvt() to healthy ephemerides
      void setOnlyHealthyFlag(bool flag)
      { onlyHealthy = flag; }

      /// Return the satellite health at the given time.
      /// @param SatID sat satellite of interest
      /// @param CommonTime t time of interest
      /// @return true if the satellite is healthy
      bool getSatHealth(const SatID& sat, const CommonTime& t) const
      {
         // get the appropriate OrbitEph
         const OrbitEph *eph = findOrbitEph(sat,t);
         return eph->isHealthy();
      }

      /// This map stores sets of unique orbital elements for a single satellite.
      /// The key is the beginning of the period of validity for each set of elements.
      typedef std::map<CommonTime, OrbitEph*> TimeOrbitEphTable;

      /// This map holds all unique OrbitEph for each satellite
      /// The key is the SatID of the satellite.
      typedef std::map<SatID, TimeOrbitEphTable> SatTableMap;

      /// Returns a map of the ephemerides available for the specified
      /// satellite.  Note that the return is specifically chosen as a
      /// const reference.  The intent is to provide "read only" access
      /// for analysis.  If the map needs to be modified, see other methods.
      /// @param sat SatID of satellite of interest
      /// @return a reference to the map
      const TimeOrbitEphTable& getTimeOrbitEphMap(const SatID& sat) const;

      /// string used to format times, used in dump() and exceptions.
      static const std::string fmt;

      std::string message;

   protected:

      /// The map <SatID, <CommonTime, OrbitEph> > where all OrbitEph are stored.
      SatTableMap satTables;

      CommonTime initialTime; ///< Time of the earliest OrbitEph in the map
      CommonTime finalTime;   ///< Time of the latest OrbitEph in the map

      TimeSystem timeSystem;  ///< Time system of store i.e. initial and final times

      /// flag indicating search method (find...Eph) to use in getSatXvt
      ///  and getSatHealth
      bool strictMethod;

      /// flag indicating unhealthy ephemerides should be excluded from getXvt,
      /// otherwise it will throw (default false)
      bool onlyHealthy;

      /// Convenience routines
      void updateTimeLimits(const OrbitEph* eph)
      {
         const CommonTime beg(eph->beginValid), end(eph->endValid);
         updateTimeLimits(beg,end);
      }
      void updateTimeLimits(const OrbitEph& eph)
      {
         const CommonTime beg(eph.beginValid), end(eph.endValid);
         updateTimeLimits(beg,end);
      }
      void updateTimeLimits(const CommonTime& bb, const CommonTime& ee)
      {
         CommonTime beg(bb), end(ee);
         beg.setTimeSystem(timeSystem);
         end.setTimeSystem(timeSystem);
         if(beg < initialTime) initialTime = beg;
         if(end > finalTime) finalTime = end;
      }
      
   }; // end class OrbitEphStore

   //@}

} // namespace

#endif // GPSTK_ORBITEPHSTORE_HPP
