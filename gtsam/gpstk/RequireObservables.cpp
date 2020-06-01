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
 * @file RequireObservables.cpp
 * This class filters out satellites with observations grossly out of bounds.
 */

#include <gtsam/gpstk/RequireObservables.hpp>


namespace gpstk
{

      // Returns a string identifying this object.
   std::string RequireObservables::getClassName() const
   { return "RequireObservables"; }



      /* Method to add a set of TypeID's to be required.
       *
       * @param typeSet    Set of TypeID's to be required.
       */
   RequireObservables& RequireObservables::addRequiredType(TypeIDSet& typeSet)
   {

      requiredTypeSet.insert( typeSet.begin(),
                              typeSet.end() );

      return (*this);

   }  // End of method 'RequireObservables::addRequiredType()'



      // Returns a satTypeValueMap object, filtering the target observables.
      //
      // @param gData     Data object holding the data.
      //
   satTypeValueMap& RequireObservables::Process(satTypeValueMap& gData)
      throw(ProcessingException)
   {

      try
      {

         SatIDSet satRejectedSet;

            // Loop through all the satellites
         for ( satTypeValueMap::iterator satIt = gData.begin();
               satIt != gData.end();
               ++satIt )
         {


               // Check all the indicated TypeID's
            for ( TypeIDSet::const_iterator typeIt = requiredTypeSet.begin();
                  typeIt != requiredTypeSet.end();
                  ++typeIt )
            {


                  // Try to find required type
               typeValueMap::iterator it( (*satIt).second.find(*typeIt) );

                  // Now, check if this TypeID exists in this data structure
               if ( it == (*satIt).second.end() )
               {
                     // If we couldn't find type, then schedule this
                     // satellite for removal
                  satRejectedSet.insert( (*satIt).first );

                     // It is not necessary to keep looking
                  typeIt = requiredTypeSet.end();
                  --typeIt;
               }

            }

         }

            // Let's remove satellites without all TypeID's
         gData.removeSatID(satRejectedSet);

         return gData;

      }
      catch(Exception& u)
      {
            // Throw an exception if something unexpected happens
         ProcessingException e( getClassName() + ":"
                                + u.what() );

         GPSTK_THROW(e);

      }

   }  // End of 'RequireObservables::Process()'


} // End of namespace gpstk
