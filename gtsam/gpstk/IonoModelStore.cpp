#pragma ident "$Id$"

/**
 * @file IonoModelStore.cpp
 * Store GPS Navigation Message based ionospheric models
 */


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


#include "IonoModelStore.hpp"

using namespace std;

namespace gpstk
{

      /* Get the ionospheric correction value.
       *
       * \param time the time of the observation
       * \param rxgeo the WGS84 geodetic position of the receiver
       * \param svel the elevation angle between the rx and SV (degrees)
       * \param svaz the azimuth angle between the rx and SV (degrees)
       * \param freq the GPS frequency the observation was made from
       * \return the ionospheric correction (meters)
       */
   double IonoModelStore::getCorrection(const CommonTime& time,
                                        const Position& rxgeo,
                                        double svel,
                                        double svaz,
                                        IonoModel::Frequency freq) const
      throw(IonoModelStore::NoIonoModelFound)
   {

      IonoModelMap::const_iterator i = ims.upper_bound(time);
      if (!ims.empty() && i != ims.begin())
      {
         i--;
         return i->second.getCorrection(time, rxgeo, svel, svaz, freq);
      }
      else
      {
         NoIonoModelFound e;
         GPSTK_THROW(e);
      }

   }  // End of method 'IonoModelStore::getCorrection()'


      /* Add an IonoModel to this collection
       *
       * \param mt the time the model is valid from
       * \param im the IonoModel to add
       * \return true if the model was added, false otherwise
       */
   bool IonoModelStore::addIonoModel(const CommonTime& mt, const IonoModel& im)
      throw()
   {

      if (!im.isValid())
         return false;

      IonoModelMap::const_iterator i = ims.upper_bound(mt);
      if (!ims.empty() && i != ims.begin())
      {
            // compare to previous stored model and if they have the
            // the same alpha and beta parameters don't store it
         i--;
         if (im == i->second)
            return false;
      }

      ims[mt] = im;

      return true;

   }  // End of method 'IonoModelStore::addIonoModel()'


}  // End of namespace gpstk
