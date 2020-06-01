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
 * @file CNavDataElementStore.cpp
 *
 */

#include <iostream>

#include "CNavDataElementStore.hpp"
#include "StringUtils.hpp"
#include "TimeString.hpp"

using namespace std;
using namespace gpstk;

namespace gpstk
{
   CNavDataElementStore::CNavDataElementStore(bool keepOnlyUnique):
      initialTime(CommonTime::BEGINNING_OF_TIME),
      finalTime(CommonTime::END_OF_TIME),
      keepingOnlyUnique(false)
    {
       initialTime.setTimeSystem(TimeSystem::GPS); 
       finalTime.setTimeSystem(TimeSystem::GPS); 
       keepingOnlyUnique = keepOnlyUnique;
    }

   void CNavDataElementStore::clear()
   {
      for( DEMap::iterator ui = deMap.begin(); ui != deMap.end(); ui++)
      {
         DataElementMap& dem = ui->second;
         for (DataElementMap::iterator oi = dem.begin(); oi != dem.end(); oi++)
         {
            delete oi->second;
         }
      } 
     deMap.clear();
     initialTime = gpstk::CommonTime::END_OF_TIME;
     finalTime = gpstk::CommonTime::BEGINNING_OF_TIME;
     initialTime.setTimeSystem(TimeSystem::GPS);
     finalTime.setTimeSystem(TimeSystem::GPS); 
   }

   void CNavDataElementStore::dump(std::ostream& s, short detail) const
   {
      DEMap::const_iterator cit1;
      for (cit1=deMap.begin();cit1!=deMap.end();cit1++)
      {
         SatID satID = (SatID) cit1->first;
         s << "*******************************************************" << endl;
         s << "CNAV Data Elements for " << satID << endl;
         DataElementMap& dem = (DataElementMap&) cit1->second; 
         DataElementMap::const_iterator cit2;
         for (cit2=dem.begin();cit2!=dem.end();cit2++)
         {
            const CNavDataElement* p = (const CNavDataElement*) cit2->second;
            p->dump(s);
         }
      }   
   }

   void CNavDataElementStore::edit(const CommonTime& tmin, 
                     const CommonTime& tmax)
   {
      for (DEMap::iterator i=deMap.begin(); i!=deMap.end();i++)
      {
         DataElementMap& dem = i->second;
         DataElementMap::iterator lower = dem.lower_bound(tmin);
         if (lower!=dem.begin())
         {
            for (DataElementMap::iterator dmi = dem.begin();dmi!=lower; dmi++)
            { 
               delete dmi->second;
            }
            dem.erase(dem.begin(),lower);
         }

         DataElementMap::iterator upper = dem.upper_bound(tmax);
         if (upper!=dem.end())
         {
            for (DataElementMap::iterator dmi=upper;dmi!=dem.end();dmi++)
            {
               delete dmi->second;
            }
            dem.erase(upper,dem.end());
         }
      }
      initialTime = tmin;
      finalTime = tmax; 
   }

   CommonTime CNavDataElementStore::getInitialTime() const
   {
      return initialTime;
   }

   CommonTime CNavDataElementStore::getFinalTime() const
   {
      return finalTime; 
   }

   bool CNavDataElementStore::addDataElement(const CNavDataElement& cnde)
   {
      bool rc = false;
#pragma unused(rc)
       
       SatID sid = cnde.satID;
      DataElementMap& dem = deMap[sid];

         // If the map is empty, load object and return
      if (dem.size()==0)
      {
         dem[cnde.ctXmit] = cnde.clone();
         if (cnde.ctXmit<initialTime) initialTime = cnde.ctXmit;
         if (cnde.ctXmit>finalTime) finalTime = cnde.ctXmit;
         return true; 
      }

         // Check to see if there is already an entry in the map.
         // If not, add the element and return true.  
         // If an entry DOES exist, do NOT update and return false.
      DataElementMap::iterator it = dem.find(cnde.ctXmit);
      if (it==dem.end())
      {
         dem[cnde.ctXmit] = cnde.clone();
         if (cnde.ctXmit<initialTime) initialTime = cnde.ctXmit;
         if (cnde.ctXmit>finalTime) finalTime = cnde.ctXmit;
         return true;
      }
      return false;
   }

   unsigned long CNavDataElementStore::size() const
   {
      unsigned long counter = 0;
      for (DEMap::const_iterator i=deMap.begin(); i!=deMap.end(); i++)
      {
         counter += i->second.size();
      }
      return counter; 
   }

   /// Need to add methods to FIND particular data elements.   Need
   /// to figure those out first. 

      // NEED TO IMPROVE THIS TO FILTER TO ONLY THOSE ELEMENTS THAT
      // ARE WITHIN THE TIME BOUNDS.
   const CNavDataElementStore::DataElementMap& 
   CNavDataElementStore::getDataElementMap(const SatID& satID,
                                           const CommonTime& begin,
                                           const CommonTime& end) const
                throw(InvalidRequest)
   {
      DEMap::const_iterator cit = deMap.find(satID);
      if (cit==deMap.end())
      {
         InvalidRequest e("No CNAV data elements for satellite "+
                           gpstk::StringUtils::asString(satID));
         GPSTK_THROW(e);
      }
      return(cit->second);
   }
}  // End of namespace


