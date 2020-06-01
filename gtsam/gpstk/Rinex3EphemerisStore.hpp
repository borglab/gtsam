/// @file Rinex3EphemerisStore.hpp
/// Read and store RINEX formated navigation message (Rinex3Nav) data, following
/// the RINEX 3.02 spec. Support for GNSS GPS, GAL, GLO, BDS, QZS (GEO TBD).

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

#ifndef GPSTK_RINEX3EPHEMERISSTORE_HPP
#define GPSTK_RINEX3EPHEMERISSTORE_HPP

#include <iostream>
#include <string>
#include <list>
#include <map>
#include <algorithm>

#include "Exception.hpp"
#include "CommonTime.hpp"
#include "BDSWeekSecond.hpp"
#include "GPSWeekSecond.hpp"
#include "SatID.hpp"
#include "Xvt.hpp"
#include "Rinex3NavHeader.hpp"
#include "Rinex3NavData.hpp"
#include "TimeSystemCorr.hpp"

#include "FileStore.hpp"

#include "OrbitEphStore.hpp"
#include "GloEphemerisStore.hpp"
//#include "GeoEphemerisStore.hpp"

namespace gpstk
{
   /** @addtogroup ephemstore */
   //@{

   class Rinex3EphemerisStore : public XvtStore<SatID>
   {
   // member data:
   private:

      /// FileStore for the Rinex3Nav input files
      FileStore<Rinex3NavHeader> NavFiles;

      /// Ephemeris store for orbit-based nav messages (stored as descendents of
      /// OrbitEph), including systems GPS, Galileo, BeiDou, and QZSS
      OrbitEphStore ORBstore;

      /// Ephemeris store for Glonass nav messages (stored as GloRecord)
      GloEphemerisStore GLOstore;

      /// Ephemeris store for Geosync nav messages (stored as GeoRecord)
      //GeoEphemerisStore GEOstore;

   public:

      /// Rinex file header last read by loadFile()
      Rinex3NavHeader Rhead;

      /// Rinex file data last read by loadFile()
      Rinex3NavData Rdata;

      /// Map of time system corrections, similar to mapTimeCorr in Rinex3NavHeader,
      /// and taken from either loadFile() (RinexNavHeader) or user input.
      /// key = TimeSystemCorrection::asString4().
      /// User may add to the list with addTimeCorr()
      std::map<std::string, TimeSystemCorrection> mapTimeCorr;

      /// string containing what() of exceptions caught by loadFile()
      std::string what;

   // member functions:

      Rinex3EphemerisStore()
      {
         bool flag = ORBstore.getOnlyHealthyFlag();    // default is ORB default
         GLOstore.setCheckHealthFlag(flag);
      }

      /// destructor
      virtual ~Rinex3EphemerisStore()
      { }
      
   // XvtStore interface:

      /// Returns the position, velocity, and clock offset of the indicated
      /// object in ECEF coordinates (meters) at the indicated time.
      /// @param[in] sat the satellite of interest
      /// @param[in] ttag the time to look up
      /// @return the Xvt of the object at the indicated time
      /// @throw InvalidRequest If the request can not be completed for any
      ///    reason, this is thrown. The text may have additional
      ///    information as to why the request failed.
      virtual Xvt getXvt(const SatID& sat, const CommonTime& ttag) const;

      /// Dump information about the store to an ostream.
      /// @param[in] os ostream to receive the output; defaults to std::cout
      /// @param[in] detail integer level of detail to provide; allowed values are
      ///    0: number of satellites, time step and time limits (default)
      ///    1: above plus flags, gap and interval values, and number of data/sat
      ///    2: above plus all the data tables
      virtual void dump(std::ostream& os=std::cout, short detail=0) const;

      /// Edit the dataset, removing data outside the indicated time interval
      /// @param[in] tmin defines the beginning of the time interval
      /// @param[in] tmax defines the end of the time interval
      virtual void edit(const CommonTime& tmin, 
                        const CommonTime& tmax = CommonTime::END_OF_TIME)
      {
         if(ORBstore.size()) ORBstore.edit(tmin, tmax);
         if(GLOstore.size()) GLOstore.edit(tmin, tmax);
         //if(GEOstore.size()) GEOstore.edit(tmin, tmax);
      }

      /// Clear the dataset, meaning remove all data
      virtual void clear(void)
      {
         NavFiles.clear();
         ORBstore.clear();
         GLOstore.clear();
         //GEOstore.clear();
      }

      /// Return time system of this store. NB this is needed only to satisfy the
      /// XvtStore virtual interface; the system stores (GPSstore, GLOstore, etc)
      /// will be used internally to determine time system.
      virtual TimeSystem getTimeSystem(void) const
         { return TimeSystem::Any; }

      /// Determine the earliest time for which this object can successfully 
      /// determine the Xvt for any object.
      /// @return the earliest time in the table
      /// @throw InvalidRequest if the object has no data.
      virtual CommonTime getInitialTime(void) const;

      /// Determine the latest time for which this object can successfully 
      /// determine the Xvt for any object.
      /// @return the latest time in the table
      /// @throw InvalidRequest if the object has no data.
      virtual CommonTime getFinalTime(void) const;

      /// Return true if IndexType=SatID is present in the data tables
      virtual bool isPresent(const SatID& sat) const
      {
         switch(sat.system) {
            case SatID::systemGPS:
            case SatID::systemGalileo:
            case SatID::systemBeiDou:
            case SatID::systemQZSS:
               return ORBstore.isPresent(sat);
            case SatID::systemGlonass:
               return GLOstore.isPresent(sat);
            //case SatID::systemGeosync:
               //return GEOstore.isPresent(sat);
            default:
               return false;
         }
         return false;
      }

      /// Return true if velocity is present in the data tables
      virtual bool hasVelocity(void) const
         { return true; }

   // end of XvtStore interface

      /// Determine the earliest time for which this object can successfully 
      /// determine the Xvt for this satellite or system (sat.id == -1).
      /// @param SatID sat satellite, or system if sat.id==-1
      /// @return the earliest time in the table
      /// @throw InvalidRequest if the object has no data.
      virtual CommonTime getInitialTime(const SatID& sat) const;

      /// Determine the latest time for which this object can successfully 
      /// determine the Xvt for this satellite or system (sat.id == -1).
      /// @param SatID sat satellite, or system if sat.id==-1
      /// @return the latest time in the table
      /// @throw InvalidRequest if the object has no data.
      virtual CommonTime getFinalTime(const SatID& sat) const;

      /// add a Rinex3NavData to the store
      /// @param Rinex3NavData Rdata data to be added
      /// @return true if data was added, false otherwise
      bool addEphemeris(const Rinex3NavData& Rdata);

      /// add filename and header to FileStore
      /// @param string filename file name to be added
      /// @param Rinex3NavHeader head header to be added
      void addFile(const std::string& filename, Rinex3NavHeader& head)
      { NavFiles.addFile(filename,head); }

      /// load a RINEX navigation file
      /// @param string filename name of the RINEX navigation file to read
      /// @param bool dump if true, dump header and nav data as read, default false
      /// @param ostream stream to which dump is written, default cout
      /// @return -1 failed to open file,
      ///         -2 failed to read header (this->Rhead),
      ///         -3 failed to read data (this->Rdata),
      ///        >=0 number of nav records read
      /// @throw some other problem
      int loadFile(const std::string& filename, bool dump=false,
                    std::ostream& s=std::cout);

      /// use to access the data records in the store in bulk
      /// Add all Rinex3NavData in this store to the given list. If sat is defined,
      /// (its default is (-1,mixed)), then add only objects of sat's system,
      /// further if sat.id is not -1, add only records for that sat.
      int addToList(std::list<Rinex3NavData>& theList,
                    SatID sat=SatID(-1,SatID::systemMixed)) const;

      /// get the number of records or ephemerides for the given satellite;
      /// an overload size(SatID::SatelliteSystem) gives the total per system.
      /// @param SatID sat satellite
      /// @return the number of records or ephemerides
      int size(const SatID sat) const
      {
         int n(0);
         SatID::SatelliteSystem sys = sat.system;

         if(sys == SatID::systemMixed || sys == SatID::systemGPS ||
            sys == SatID::systemGalileo || sys == SatID::systemBeiDou ||
            sys == SatID::systemQZSS)
            n += ORBstore.size(sat);

         if(sys == SatID::systemMixed || sys == SatID::systemGlonass)
            n += GLOstore.size();

         //if(sys == SatID::systemMixed || sys == SatID::systemGeosync)
         //   n += GEOstore.size();

         return n;
      }

      /// get the number of records by system; default is systemMixed,
      /// which returns the overall total
      /// @param SatID::SatelliteSystem GNSS of interest
      /// @return the number of records or ephemerides in this system
      int size(const SatID::SatelliteSystem sys = SatID::systemMixed) const
         { return size(SatID(-1,sys)); }

      /// Add to the map of time system corrections. Overwrite the existing
      /// correction of the same type, if it exists.
      /// @return true if an existing correction was overwritten.
      bool addTimeCorr(const TimeSystemCorrection& tsc)
      {
         // true if this type already exists
         bool overwrite(mapTimeCorr.find(tsc.asString4()) != mapTimeCorr.end());

         // add or overwrite it
         mapTimeCorr[tsc.asString4()] = tsc;

         return overwrite;
      }

      /// Delete from the map of time system corrections.
      /// @param type of TimeSystemCorrection, as a string,
      ///                   i.e. TimeSystemCorrection::asString4()
      /// @return true if an existing correction was deleted.
      bool delTimeCorr(const std::string& typestr)
      {
         std::map<std::string, TimeSystemCorrection>::iterator it;
         it = mapTimeCorr.find(typestr);
         if(it != mapTimeCorr.end()) {
            mapTimeCorr.erase(it);
            return true;
         }
         return false;
      }

      /// Fill out the time system corrections "network" by adding corrections that
      /// can be derived from existing corrections. For example,
      ///   given GPUT (GPS to UTC(USNO), from LEAP SECONDS)
      ///     and GLUT (GLO to UTC(SU), from CORR TO SYSTEM TIME)
      /// compute GLGP (GLO to GPS) by assuming all UTC's are equivalent.
      /// @return the number of new TimeSystemCorrection's
      int expandTimeCorrMap(void)
      {
         int n(0);
         std::map<std::string, TimeSystemCorrection>::iterator it,jt;

         // GLGP : GLO to GPS
         if(mapTimeCorr.find(std::string("GLGP")) == mapTimeCorr.end()) {
            it = mapTimeCorr.find(std::string("GPUT"));
            jt = mapTimeCorr.find(std::string("GLUT"));
            if(it != mapTimeCorr.end() && jt != mapTimeCorr.end()
               && mapTimeCorr.find(std::string("GLGP")) == mapTimeCorr.end())
            {
               TimeSystemCorrection tc("GLGP");
               tc.A0 = jt->second.A0 - it->second.A0;
               tc.A1 = jt->second.A1 - it->second.A1; // probably zeros
               tc.refYr = it->second.refYr;
               tc.refMon = it->second.refMon;
               tc.refDay = it->second.refDay;
               tc.refWeek = jt->second.refWeek;
               tc.refSOW = jt->second.refSOW;
               tc.geoProvider = jt->second.geoProvider;  // blank
               tc.geoUTCid = 0;                          // NA
               mapTimeCorr[tc.asString4()] = tc;
               n++;
            }
         }

         // GPGA : GPS to GAL
         if(mapTimeCorr.find(std::string("GPGA")) == mapTimeCorr.end()) {
            it = mapTimeCorr.find(std::string("GAUT"));
            jt = mapTimeCorr.find(std::string("GPUT"));
            if(it != mapTimeCorr.end() && jt != mapTimeCorr.end()
               && mapTimeCorr.find(std::string("GPGA")) == mapTimeCorr.end())
            {
               TimeSystemCorrection tc("GPGA");
               tc.A0 = jt->second.A0 - it->second.A0;
               tc.A1 = jt->second.A1 - it->second.A1;
               tc.refYr = it->second.refYr;           // take ref time from GAL
               tc.refMon = it->second.refMon;
               tc.refDay = it->second.refDay;
               tc.refWeek = it->second.refWeek;
               tc.refSOW = it->second.refSOW;
               tc.geoProvider = it->second.geoProvider;  // blank
               tc.geoUTCid = 0;                          // NA
               mapTimeCorr[tc.asString4()] = tc;
               n++;
            }
         }

         // BDGP : BDS to GPS
         if(mapTimeCorr.find(std::string("BDGP")) == mapTimeCorr.end()) {
            it = mapTimeCorr.find(std::string("GPUT"));
            jt = mapTimeCorr.find(std::string("BDUT"));
            if(it != mapTimeCorr.end() && jt != mapTimeCorr.end()
               && mapTimeCorr.find(std::string("BDGP")) == mapTimeCorr.end())
            {
               TimeSystemCorrection tc("BDGP");
               tc.A0 = jt->second.A0 - it->second.A0;
               tc.A1 = jt->second.A1 - it->second.A1;
               tc.refYr = it->second.refYr;
               tc.refMon = it->second.refMon;
               tc.refDay = it->second.refDay;
               BDSWeekSecond bws(jt->second.refWeek,jt->second.refSOW);
               GPSWeekSecond gws(bws);
               tc.refWeek = gws.week;
               tc.refSOW = gws.sow;
               tc.geoProvider = jt->second.geoProvider;  // blank
               tc.geoUTCid = 7;
               mapTimeCorr[tc.asString4()] = tc;
               n++;
            }
         }

         return n;
      }

      /// Utility routine for getXvt and addEphemeris to test time systems and
      /// convert if necessary. Convert ttag to the target time system, using the
      /// first appropriate correction in mapTimeCorr, and return it.
      /// If no correction is found, ttag is unchanged and an exception is thrown.
      CommonTime correctTimeSystem(const CommonTime ttag,
                                   const TimeSystem targetSys) const;

      /// Find the appropriate time system correction object in the collection for the
      /// given time systems, and dump it to a string and return that string.
      std::string dumpTimeSystemCorrection(const TimeSystem fromSys,
                                           const TimeSystem toSys) const;

      /// Get integration step for GLONASS Runge-Kutta algorithm (seconds)
      double getGLOStep(void) const
         { return GLOstore.getIntegrationStep(); }

      /// Set integration step for GLONASS Runge-Kutta algorithm (seconds)
      void setGLOStep(double step)
         { GLOstore.setIntegrationStep(step); }

      /// Get flag that causes unhealthy (Orbit-based) ephemerides to be excluded
      bool getOnlyHealthyFlag(void) const
      {
         return ORBstore.getOnlyHealthyFlag();
      }

      /// Set flag that causes unhealthy ephemerides to be excluded
      void setOnlyHealthyFlag(bool flag)
      {
         ORBstore.setOnlyHealthyFlag(flag);
         GLOstore.setCheckHealthFlag(flag);
      }

      /// use findNearEphemeris() in the getSat...() routines (Orbit-based systems)
      void SearchNear(void)
      {
         ORBstore.SearchNear();
      }

      /// use findEphemeris() in the getSat...() routines (the default) (Orbits)
      void SearchUser(void)
      {
         ORBstore.SearchUser();
      }

   }; // end class Rinex3EphemerisStore

}  // namespace

#endif // GPSTK_RINEX3EPHEMERISSTORE_HPP
