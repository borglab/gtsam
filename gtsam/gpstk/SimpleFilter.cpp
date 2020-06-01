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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2008, 2009, 2011
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
 * @file SimpleFilter.cpp
 * This class filters out satellites with observations grossly out of bounds.
 */

#include <gtsam/gpstk/SimpleFilter.hpp>


namespace gpstk
{

      // Returns a string identifying this object.
   std::string SimpleFilter::getClassName() const
   { return "SimpleFilter"; }


      // Returns a satTypeValueMap object, filtering the target observables.
      //
      // @param gData     Data object holding the data.
      //
   satTypeValueMap& SimpleFilter::Process(satTypeValueMap& gData)
      throw(ProcessingException)
   {

      try
      {

         SatIDSet satRejectedSet;

            // Check all the indicated TypeID's
         TypeIDSet::const_iterator pos;
         for (pos = filterTypeSet.begin(); pos != filterTypeSet.end(); ++pos)
         {

            double value(0.0);

               // Loop through all the satellites
            satTypeValueMap::iterator it;
            for (it = gData.begin(); it != gData.end(); ++it) 
            {
               try
               {
                     // Try to extract the values
                  value = (*it).second(*pos);

                     // Now, check that the value is within bounds
                  if ( !( checkValue(value) ) )
                  {
                        // If value is out of bounds, then schedule this
                        // satellite for removal
                      satRejectedSet.insert( (*it).first );
                  }
               }
               catch(...)
               {
                     // If some value is missing, then schedule this satellite
                     // for removal
                  satRejectedSet.insert( (*it).first );
               }
            }

               // Before checking next TypeID, let's remove satellites with
               // data out of bounds
            gData.removeSatID(satRejectedSet);
         }

         return gData;

      }
      catch(Exception& u)
      {
            // Throw an exception if something unexpected happens
         ProcessingException e( getClassName() + ":"
                                + u.what() );

         GPSTK_THROW(e);

      }

   }  // End of 'SimpleFilter::Process()'


} // End of namespace gpstk
