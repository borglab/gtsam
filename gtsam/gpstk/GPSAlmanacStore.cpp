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
 * @file AlmanacStore.cpp
 * Store GPS almanac information (i.e. like the data in subframes 4&5) and 
 * compute satellite Xvt based upon this data and the algorithms defined
 * in the IS-GPS-200.
 */

#include "GPSAlmanacStore.hpp"
#include "StringUtils.hpp"
#include "gps_constants.hpp"
#include "CommonTime.hpp"

namespace gpstk
{
   Xvt GPSAlmanacStore::getXvt(const SatID& sat, const CommonTime& t)
      const throw(InvalidRequest)
   {
      AlmOrbit a = findAlmanac(sat, t);
      return a.svXvt(t);
   }

   Xvt GPSAlmanacStore::getXvtMostRecentXmit(const SatID sat, const CommonTime& t) 
         const throw(InvalidRequest)
   {
      AlmOrbit a = findMostRecentAlmanac(sat, t);
      return a.svXvt(t);
   }
   
   short GPSAlmanacStore::getSatHealth(const SatID sat, const CommonTime& t)
      const throw(InvalidRequest)
   {
      AlmOrbit a = findAlmanac(sat, t);
      return a.getSVHealth();
   }

   bool GPSAlmanacStore::addAlmanac(const AlmOrbit& alm) throw()
   {
      if ((alm.getPRNID() >= 1) && (alm.getPRNID() <= MAX_PRN_GPS))
      {
         SatID sat(alm.getPRNID(),SatID::systemGPS);
         CommonTime toa = alm.getToaTime();
         uba[sat][toa] = alm;
         CommonTime tmin(toa - gpstk::HALFWEEK);
         CommonTime tmax(toa + gpstk::HALFWEEK);
         if (tmin < initialTime)
            initialTime = tmin;
         if (tmax > finalTime)
            finalTime = tmax;
         return true;
      }
      return false;
   }

   bool GPSAlmanacStore::addAlmanac(const EngAlmanac& alm) throw()
   {
      AlmOrbits ao = alm.getAlmOrbElems();
      AlmOrbits::const_iterator oci;
      for (oci = ao.begin(); oci != ao.end(); oci++)
         addAlmanac((*oci).second);

      return true;
   }

   /// gets the closest almanac for the given time and satellite,
   /// closest being in the past or future.
   AlmOrbit GPSAlmanacStore::findAlmanac(const SatID sat, const CommonTime& t) 
      const throw(InvalidRequest)
   {
      UBAMap::const_iterator satItr = uba.find(sat);
      if (satItr == uba.end())
      {
         InvalidRequest e("No almanacs for satellite " +
                        StringUtils::asString(sat));
         GPSTK_THROW(e);
      }
         
      const EngAlmMap& eam = satItr->second;

      // find the closest almanac BEFORE t, if any.
      EngAlmMap::const_iterator neXvtItr = eam.begin(), almItr = eam.end();
         
      while ( (neXvtItr != eam.end()) &&
              (neXvtItr->first < t) )
      {
         almItr = neXvtItr;
         neXvtItr++;
      }

      if (almItr == eam.end())
      {
         if (neXvtItr == eam.end()) 
         {
            InvalidRequest e("No almanacs for time " + t.asString());
            GPSTK_THROW(e);
         }
         else
         {
            almItr = neXvtItr;
         }
      }

      // check the neXvt almanac (the first one after t's time)
      // to see if it's closer than the one before t
      if (neXvtItr != eam.end())
      {
         if ( (neXvtItr->first - t) < (t - almItr->first))
            almItr = neXvtItr;
      }
      return (*almItr).second;
   }

   AlmOrbit GPSAlmanacStore::findMostRecentAlmanac(const SatID sat, const CommonTime& t) 
         const throw(InvalidRequest)
   {
      UBAMap::const_iterator satItr = uba.find(sat);
      if (satItr == uba.end())
      {
         InvalidRequest e("No almanacs for satellite " +
                        StringUtils::asString(sat));
         GPSTK_THROW(e);
      }
         
      const EngAlmMap& eam = satItr->second;

      // find the closest almanac BEFORE t, if any.
      EngAlmMap::const_iterator neXvtItr = eam.begin(), almItr = eam.end();
         
      while ( (neXvtItr != eam.end()) &&
              (neXvtItr->second.getTransmitTime() < t) )
      {
         almItr = neXvtItr;
         neXvtItr++;
      }

      if (almItr == eam.end())
      {
         if (neXvtItr == eam.end()) 
         {
            InvalidRequest e("No almanacs for time " + t.asString());
            GPSTK_THROW(e);
         }
         else
         {
            almItr = neXvtItr;
         }
      }
      return (*almItr).second;     
   }

   AlmOrbits GPSAlmanacStore::findAlmanacs(const CommonTime& t) 
      const throw(InvalidRequest)
   {
      AlmOrbits ao;
      UBAMap::const_iterator satItr = uba.begin();
      while (satItr != uba.end())
      {
         try
         {
            AlmOrbit a = findAlmanac(satItr->first, t);
            ao[satItr->first] = a;
         }
         /// who cares about exceptions - the map will
         /// be empty if there are no alms...
         catch(...)
         {}

         satItr++;
      }
      return ao;
   }


   void GPSAlmanacStore::edit(const CommonTime& tmin, const CommonTime& tmax)
      throw()
   {
      std::cout << "Not yet implimented" << std::endl;
   }


   void GPSAlmanacStore::dump(std::ostream& s, short detail)
      const throw()
   {
      UBAMap::const_iterator i;
      EngAlmMap::const_iterator j;
      for (i=uba.begin(); i!= uba.end(); i++)
      {
         const EngAlmMap& eam = i->second;
         for (j=eam.begin(); j!=eam.end(); j++)
         {
            j->second.dump(s, detail);
            s << std::endl;
         }
      }
   }
   
}
