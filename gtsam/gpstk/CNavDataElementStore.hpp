#pragma ident "$Id:$"

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
//  Copyright 2013, The University of Texas at Austin
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
 * @file CNavDataElementStore.hpp
 * A class for storing CNavDataElements. Note that this is NOT a descendent
 * of XVTStore and is NOT designed for convenient access of orbit/clock
 * data used in XVT calculations.  Instead, it is designed to support 
 * storing CNAV "overhead" message data such as UTC, IONO, ISC, GGTO, text.  
 * These are things that were associated with the "almanac" in the legacy data
 * but occur in a less structured manner in CNAV and CNAV-2. 
 */

#ifndef GPSTK_CNAVDATAELEMENTSTORE_INCLUDE
#define GPSTK_CNAVDATAELEMENTSTORE_INCLUDE

#include <iostream>

#include "CNavDataElement.hpp"
#include "Exception.hpp"
#include "CommonTime.hpp"

namespace gpstk
{
   class CNavDataElementStore
   {
   public:
      CNavDataElementStore(bool keepOnlyUnique=false);
      
      ~CNavDataElementStore() { clear(); };

      /// Clear the dataset, meaning remove all data
      virtual void clear();

      /// A debugging function that outputs in human readable form,
      /// all data stored in this object.
      /// @param[in] s the stream to receive the output; defaults to cout
      /// @param[in] detail the level of detail to provide
      void dump(std::ostream& s = std::cout, short detail = 0) const;

      /// Edit the dataset, removing data outside the indicated time interval
      /// @param[in] tmin defines the beginning of the time interval
      /// @param[in] tmax defines the end of the time interval
      void edit(const CommonTime& tmin, 
                        const CommonTime& tmax = CommonTime::END_OF_TIME);

      /// Return the time system of the store
      /// Only one option for CNAV/CNAV-2.
      TimeSystem getTimeSystem(void) const {return (TimeSystem::GPS);}

      /// Return the earliest transmission time of any object in this 
      /// store.
      /// @return The initial time
      CommonTime getInitialTime() const;

      /// Return the latest time of any object in this store.
      /// @return The final time
      CommonTime getFinalTime() const;

      /// Add a CNavDataElement to the store.
      /// @param cnde the CNavDataElement to be added.
      /// @return true if the object was added, false otherwise
      bool addDataElement(const CNavDataElement& cnde);

      /// Return the number of CNavDataElement objects in the store.
      /// @return the number of CNavDataElement objects in the store. 
      unsigned long size() const; 

      /// Need to add methods to FIND particular data elements.   Need
      /// to figure those out first. 

      /// Intended to store sets CNavDataElements for a specified SV.
      /// The index in the TRANSMIT TIME associated with the element.
      /// Given only a single message can be transmitted at a given time
      /// this guarantees uniqueness.
      /// NOTE: EVENTUALLY, I want to arrange matters so there are TWO
      /// OPTIONS: (1.) Store only unique information, (2.) Store ALL available
      /// data elements. 
      typedef std::map<CommonTime, CNavDataElement*> DataElementMap;

      /// Returns a map of the data elements for the specified satellite.  
      /// Optionally a time range of interest may be specified.  
      /// The return is specifically chosen to be a const reference.  The
      /// intent is to provide "read only" access for analysis.
      /// If there are no elements present for the requested satellite, 
      /// the method will throw InvalidRequest.
      const DataElementMap& 
      getDataElementMap(const SatID& satID,
                        const CommonTime& begin=CommonTime::BEGINNING_OF_TIME,
                        const CommonTime& end=CommonTime::END_OF_TIME) const
                   throw(InvalidRequest);

   protected:

      typedef std::map<SatID, DataElementMap> DEMap;

         // Map where all the CNavDataElements are stored. 
      DEMap deMap;  

      CommonTime initialTime;
      CommonTime finalTime; 

      bool keepingOnlyUnique;
      
   };    // End of class
}        // End of namespace
         
#endif 




