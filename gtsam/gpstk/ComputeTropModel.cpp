//============================================================================
//
//  This file is part of GPSTk, the GPS Toolkit.
//
//  The GPSTk is free software; you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published
//  by the Free Software Foundation; either version 3.0 of the License, or
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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2007, 2008, 2011 
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
 * @file ComputeTropModel.cpp
 * This is a class to compute the main values related to a given
 * GNSS tropospheric model.
 */

#include <gtsam/gpstk/ComputeTropModel.hpp>


namespace gpstk
{


      // Returns a string identifying this object.
   std::string ComputeTropModel::getClassName() const
   { return "ComputeTropModel"; }



      /* Returns a satTypeValueMap object, adding the new data generated when
       * calling a modeling object.
       *
       * @param time      Epoch.
       * @param gData     Data object holding the data.
       */
   satTypeValueMap& ComputeTropModel::Process( const CommonTime& time,
                                         satTypeValueMap& gData )
      throw(ProcessingException)
   {

      try
      {

         SatIDSet satRejectedSet;

            // Loop through all the satellites
         satTypeValueMap::iterator stv;
         for(stv = gData.begin(); stv != gData.end(); ++stv) 
         {

            // First check if TropModel was set
            if(pTropModel==NULL)
            {
                  // If TropModel is missing, then remove all satellites
               satRejectedSet.insert( (*stv).first );
               continue;
            }

               // If satellite elevation is missing, remove satellite
            if( (*stv).second.find(TypeID::elevation) == (*stv).second.end() )
            {
               satRejectedSet.insert( (*stv).first );
               continue;
            }
            else
            {

                  // Scalar to hold satellite elevation
               double elevation( (*stv).second(TypeID::elevation) );
               double tropoCorr(0.0), dryZDelay(0.0), wetZDelay(0.0);
               double dryMap(0.0), wetMap(0.0);

               try
               {
                     // Compute tropospheric slant correction
                  tropoCorr = pTropModel->correction(elevation);
                  dryZDelay = pTropModel->dry_zenith_delay();
                  wetZDelay = pTropModel->wet_zenith_delay();
                  dryMap = pTropModel->dry_mapping_function(elevation);
                  wetMap = pTropModel->wet_mapping_function(elevation);

                     // Check validity
                  if( !(pTropModel->isValid()) )
                  {
                     tropoCorr = 0.0;
                     dryZDelay = 0.0;
                     wetZDelay = 0.0;
                     dryMap    = 0.0;
                     wetMap    = 0.0;
                  }

               }
               catch(TropModel::InvalidTropModel& e)
               {
                     // If some problem appears, then schedule this
                     // satellite for removal
                  satRejectedSet.insert( (*stv).first );
                  continue;    // Skip this SV if problems arise
               };

                  // Now we have to add the new values to the data structure
               (*stv).second[TypeID::tropoSlant] = tropoCorr;
               (*stv).second[TypeID::dryTropo] = dryZDelay;
               (*stv).second[TypeID::wetTropo] = wetZDelay;
               (*stv).second[TypeID::dryMap] = dryMap;
               (*stv).second[TypeID::wetMap] = wetMap;

            }

         }  // End of loop 'for(stv = gData.begin()...'

            // Remove satellites with missing data
         gData.removeSatID(satRejectedSet);

         return gData;

      }   // End of try...
      catch(Exception& u)
      {
            // Throw an exception if something unexpected happens
         ProcessingException e( getClassName() + ":"
                                + u.what() );

         GPSTK_THROW(e);

      }

   } // End ComputeTropModel::Process()


} // End of namespace gpstk
