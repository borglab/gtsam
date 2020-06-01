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
 * @file GPSOrbElemStore.hpp
 * Store GPS broadcast OrbElem information (i.e. like the data in
 * subframes 1-3) and computes satellite Xvt based upon this data and the
 * algorithms defined for that data in the IS-GPS-200.
 */

#ifndef GPSTK_GPSORBELEMSTORE_HPP
#define GPSTK_GPSORBELEMSTORE_HPP

#include <iostream>
#include <list>
#include <map>

#include "OrbElem.hpp"
#include "OrbElemStore.hpp"
#include "Exception.hpp"
#include "SatID.hpp"
#include "CommonTime.hpp"
#include "XvtStore.hpp"
#include "TimeSystem.hpp"

namespace gpstk
{
   /** @addtogroup ephemstore */
   //@{

   /// Class for storing and accessing GPS SV's position, 
   /// velocity, and clock data. Also defines a simple interface to remove
   /// data that has been added.
   class GPSOrbElemStore : public OrbElemStore
   {
   public:
      
      GPSOrbElemStore()
         throw()
         : initialTime(CommonTime::END_OF_TIME), 
           finalTime(CommonTime::BEGINNING_OF_TIME),
           strictMethod(true)
      {
       initialTime.setTimeSystem(TimeSystem::GPS);
       finalTime.setTimeSystem(TimeSystem::GPS);
      }


      virtual ~GPSOrbElemStore()
      { clear();}

      /// Returns the position, velocity, and clock offset of the indicated
      /// satellite in ECEF coordinates (meters) at the indicated time.
      /// @param[in] sat the SV's SatID
      /// @param[in] t the time to look up
      /// @return the Xvt of the SV at time
      /// @throw InvalidRequest If the request can not be completed for any
      ///    reason, this is thrown. The text may have additional
      ///    information as to why the request failed.  Possible reasons
      ///    include
      ///       1. No orbital elements stored for the SV
      ///       2. No orbital elements with time of validity covering time t
      ///       3. Orbital elements appropriate for time t are unhealhty
      ///  The purpose of getX is to be as SAFE as possible.  
      ///  If you MUST obtain SV PVT in the failure conditions noted above,
      ///  please consider calling findOrbElem( ) or findNearOrbElem( ) 
      ///  directly to obtain elements, then use OrbElem.getXvt( ) 
      ///  to obtain positions.  
      virtual Xvt getXvt( const SatID& sat, const CommonTime& t ) const
         throw( InvalidRequest );

      /// A debugging function that outputs in human readable form,
      /// all data stored in this object.
      /// @param[in] s the stream to receive the output; defaults to cout
      /// @param[in] detail the level of detail to provide
      virtual void dump( std::ostream& s = std::cout, short detail = 0 ) const
         throw();

      /// Edit the dataset, removing data outside the indicated time interval
      /// @param tmin defines the beginning of the time interval, included
      /// @param tmax defines the end of the time interval. not included
      /// [tmin, tmax)
      virtual void edit( const CommonTime& tmin, 
                         const CommonTime& tmax = CommonTime::END_OF_TIME )
         throw();


      /// Return time system (NB assumed always to be GPS)
      virtual TimeSystem getTimeSystem(void) const throw()
         { return TimeSystem::GPS; }

      /// Determine the earliest time for which this object can successfully 
      /// determine the Xvt for any satellite.
      /// @return The initial time
      /// @throw InvalidRequest This is thrown if the object has no data.
      virtual CommonTime getInitialTime() const
         throw()
         { return initialTime; }


      /// Determine the latest time for which this object can successfully
      /// determine the Xvt for any satellite.
      /// @return The final time
      /// @throw InvalidRequest This is thrown if the object has no data.
      virtual CommonTime getFinalTime() const
         throw()
         { return finalTime; }

      virtual bool velocityIsPresent()
         const throw()
      { return true; }

      /// Return true if velocity data is present in the store
      virtual bool hasVelocity() const throw()
      { return true; }

      /// Return true if the given IndexType is present in the store
      virtual bool isPresent(const SatID& sat) const throw()
      {
         if(ube.find(sat) != ube.end()) return true;
         return false;
      }

      //---------------------------------------------------------------
      // Below are interfaces that are unique to this class (i.e. not
      // in the parent class)
      //---------------------------------------------------------------

      /// Returns the health of an SV for a particular time.
      /// @param sat the satellite's SatID
      /// @param t the time to look up
      /// @return the SV health bits
      /// @throw InvalidRequest no matching OrbElem found in the store
      bool isHealthy( const SatID& sat, const CommonTime& t ) const
         throw( InvalidRequest );


      /// Add an OrbElem object to this collection.
      /// @param eph the OrbElem to add
      /// @return true if OrbElem was added, false otherwise
      bool addOrbElem( const OrbElem& eph )
         throw(InvalidParameter,Exception);

      /// Remove all data from this collection.
      void clear()
         throw()
      {
         for( UBEMap::iterator ui = ube.begin(); ui != ube.end(); ui++)
         {
            OrbElemMap& oem = ui->second;
            for (OrbElemMap::iterator oi = oem.begin(); oi != oem.end(); oi++)
            {
               delete oi->second;
            }
         } 
        ube.clear();
        initialTime = gpstk::CommonTime::END_OF_TIME;
        finalTime = gpstk::CommonTime::BEGINNING_OF_TIME;
        initialTime.setTimeSystem(TimeSystem::GPS);
        finalTime.setTimeSystem(TimeSystem::GPS); 
       }

      /// Get the number of OrbElem objects in this collection.
      /// @return the number of OrbElem records in the map
      unsigned size() const
         throw();

      /*
       *  Explanation of find( ) function for GPSOrbElemStore
       *  
       *  The findOrbElem( ) funtion
       *  does the best possible job of emulating the choice
       *  that would be made by a real-time user following the
       *  selection crieteria and warnings defined in the
       *  IS-GPS-200.  
       *
       *  It is strongly suggested that the user load ALL 
       *  available set of orbital elements into the store, 
       *  regardless of health status.  It is furthermore
       *  suggested the user call rationalize( ) to adjust
       *  the begin/end times of validity after loading 
       *  all the elements and before using the store. 
       *
       *  There exists a second find fuction, findNearOrbElem( ).
       *  This is provided for compatibility with past uses
       *  of the GPSEphemerisStore class.  findNearOrbElem( ) MAY 
       *  return elements that are outside the range of 
       *  validity and therefore need to be used with caution.   
       *  Therefore,if you wish
       *  to use findNearOrbElem( ), you should do so directly, 
       *  carefully examine the resulting set of orbital elements
       *  and make an informed decision before using the 
       *  OrbElem.get????( ) functions.  
       */
      /// @param sat SatID of satellite of interest
      /// @param t time with which to search for OrbElem
      /// @return a reference to the desired OrbElem
      /// @throw InvalidRequest object thrown when no OrbElem is found
      const OrbElem* findOrbElem( const SatID& sat, const CommonTime& t )
         const throw( InvalidRequest );


      /// Find an OrbElem for the indicated satellite at time t. The OrbElem
      /// chosen is the one with HOW time closest to the time t, (i.e. with
      /// smallest fabs(t-HOW), but still within the fit interval.
      /// @param sat the SV of interest
      /// @param t the time of interest
      /// @return a reference to desired OrbElem
      /// @throw InvalidRequest object thrown when no OrbElem is found
      const OrbElem* findNearOrbElem( const SatID& sat, const CommonTime& t )
         const throw( InvalidRequest );

      /// Add all ephemerides to an existing list<OrbElem>.
      /// @return the number of ephemerides added.
      int addToList( std::list<OrbElem*>& v ) const
         throw();

      /// use findNearOrbElem() in the getSat...() routines
      void SearchNear(void)
         throw()
      { strictMethod = false; }

      /// use findUserOrbElem() in the getSat...() routines (the default)
      void SearchUser(void)
         throw()
      { strictMethod = true; }

      /// This is intended to store sets of unique orbital elements for a single SV.
      /// The key is the beginning of the period of validity for each set of elements. 
      typedef std::map<CommonTime, OrbElem*> OrbElemMap;

      /// Returns a map of the ephemerides available for the specified
      /// satellite.  Note that the return is specifically chosen as a
      /// const reference.  The intent is to provide "read only" access
      /// for analysis.  If the map needs to be modified, see other methods.
      const OrbElemMap& getOrbElemMap( const SatID& sat ) const
         throw( InvalidRequest );

      /*
       *  Notes regarding the rationalize( ) function.
       *  The timing relationships defined in IS-GPS-200 20.3.4.5 mean
       *  (1.) The end of validity of a given set of orbital elements
       *  may be determined by the beginning of transmission of a new
       *  upload.   
       *  (2.) The beginning of validity of the SECOND set of elements
       *  following and upload should be Toe-(0.5 fit interval) but
       *  it is not practical to differentiate between the first and

       *  second set following an upload when only looking at a 
       *  single set of elements.
       *
       *  The rationalize( ) function is a means of addressing these 
       *  shortcomings.   The intention is to load all the navigation
       *  message data in the store, then call rationalize( ).  The
       *  function will sweep through the ordered set of elements and
       *  make appropriate adjustments to beginning and end of 
       *  validity values.  In general, the only changes will
       *  occur in set of elements immediately before an upload,
       *  the first set following the upload, and (perhaps) the
       *  second set following the upload. 
       * 
       */ 
      void rationalize( );

      protected:
     
      /// This is intended to hold all unique EngEphemerides for each SV
      /// The key is the prn of the SV.
      typedef std::map<SatID, OrbElemMap> UBEMap;

      /// The map where all EngEphemerides are stored.
      UBEMap ube;

      CommonTime initialTime; //< Time of the first OrbElem
      CommonTime finalTime;   //< Time of the last OrbElem

      /// flag indicating search method (find...Eph) to use in getSatXvt
      ///  and getSatHealth
      bool strictMethod;

      // Here are a couple of methods to simplify the .cpp
      void updateInitialFinal(const OrbElem& eph)
      {
        if (eph.beginValid<initialTime)       
          initialTime = eph.beginValid;
         
        if (eph.endValid>finalTime)               
          finalTime = eph.endValid;
      }
      
     // virtual void dumpOnePRN( std::ostream& s = std::cout, OrbElemMap& em) const
     //    throw();

   }; // end class

   //@}

} // namespace

#endif
