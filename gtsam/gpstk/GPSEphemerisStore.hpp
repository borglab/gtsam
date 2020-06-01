/// @file GPSEphemerisStore.hpp
/// Class for storing and/or computing position, velocity, and clock data using
/// tables of <SatID, <time, GPSEphemeris> >. Inherits OrbitEphStore, which includes
/// initial and final times and search methods. GPSEphemeris inherits OrbitEph and
/// adds health and accuracy information, which this class makes use of.

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

#ifndef GPSTK_GPSORBITEPHSTORE_HPP
#define GPSTK_GPSORBITEPHSTORE_HPP

#include "OrbitEphStore.hpp"
#include "GPSEphemeris.hpp"
#include "Exception.hpp"
#include "SatID.hpp"
#include "CommonTime.hpp"
#include "RinexNavData.hpp"

namespace gpstk
{
   /** @addtogroup ephemstore */
   //@{

   /// Class for storing and accessing an objects position,
   /// velocity, and clock data. Also defines a simple interface to remove
   /// data that has been added.
   class GPSEphemerisStore : public OrbitEphStore
   {
   public:

      /// Default empty constructor.
      GPSEphemerisStore()
      {
         timeSystem = TimeSystem::GPS;
         initialTime.setTimeSystem(timeSystem);
         finalTime.setTimeSystem(timeSystem);
      }

      /// Destructor
      virtual ~GPSEphemerisStore() { clear(); }

      /// Return a string that will identify the derived class
      virtual std::string getName(void) const
         { return std::string("GPSEphemerisStore"); }

      /// Notes regarding the rationalize() function.
      /// The timing relationships defined in IS-GPS-200 20.3.4.5 mean
      /// 1. The end of validity of a given set of orbital elements
      /// may be determined by the beginning of transmission of a new
      /// upload.
      /// 2. The beginning of validity of the SECOND set of elements
      /// following and upload should be Toe-(0.5 fit interval) but
      /// it is not practical to differentiate between the first and
      /// second set following an upload when only looking at a
      /// single set of elements.
      /// The rationalize() function is a means of addressing these
      /// shortcomings. The intention is to load all the navigation
      /// message data in the store, then call rationalize( ). The
      /// function will sweep through the ordered set of elements and
      /// make appropriate adjustments to beginning and end of
      /// validity values. In general, the only changes will
      /// occur in set of elements immediately before an upload,
      /// the first set following the upload, and (perhaps) the
      /// second set following the upload.
      void rationalize(void);

      /// Add a GPSEphemeris object to this collection
      /// @return pointer to the new object, or NULL if it could not be added
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Woverloaded-virtual"
       virtual GPSEphemeris* addEphemeris(const GPSEphemeris& gpseph)
      {
         try {
            GPSEphemeris neweph(gpseph);
            OrbitEph *oeptr = dynamic_cast<OrbitEph*>(&neweph);
            oeptr = OrbitEphStore::addEphemeris(oeptr);
            return dynamic_cast<GPSEphemeris*>(oeptr);
         }
         catch(Exception& e) { GPSTK_RETHROW(e); }
      }
#pragma clang diagnostic pop
      /// Find a GPSEphemeris for the indicated satellite at time t, using the
      /// OrbitEphStore::find() routine, which considers the current search method.
      /// @param sat the satellite of interest
      /// @param t the time of interest
      /// @return a pointer to the desired OrbitEph, or NULL if no OrbitEph found.
      /// @throw InvalidRequest if the satellite system is not BeiDou, or if
      /// ephemeris is not found.
      const GPSEphemeris& findEphemeris(const SatID& sat, const CommonTime& t) const
      {
         if(sat.system != SatID::systemBeiDou) {
            InvalidRequest e("Invalid satellite system");
            GPSTK_THROW(e);
         }

         const OrbitEph *eph = findOrbitEph(sat,t);
         if(!eph) {
            InvalidRequest e("Ephemeris not found");
            GPSTK_THROW(e);
         }

         // gynmastics...
         const GPSEphemeris* geph = dynamic_cast<const GPSEphemeris*>(eph);
         const GPSEphemeris& gpseph(*geph);

         return gpseph;
      }

      /// Add all ephemerides to an existing list<GPSEphemeris> for given satellite
      /// If sat.id is -1 (the default), all ephemerides are added.
      /// @return the number of ephemerides added.
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Woverloaded-virtual"
       int addToList(std::list<GPSEphemeris>& gpslist,
                        SatID sat=SatID(-1,SatID::systemGPS)) const;

   }; // end class GPSEphemerisStore
#pragma clang diagnostic pop
   //@}

} // namespace

#endif // GPSTK_GPSORBITEPHSTORE_HPP
