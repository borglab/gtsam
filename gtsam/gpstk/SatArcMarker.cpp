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
 * @file SatArcMarker.cpp
 * This class keeps track of satellite arcs caused by cycle slips.
 */

#include <gtsam/gpstk/SatArcMarker.hpp>


namespace gpstk
{

      // Returns a string identifying this object.
   std::string SatArcMarker::getClassName() const
   { return "SatArcMarker"; }



      /* Common constructor
       *
       * @param watchFlag        Cycle slip flag to be watched.
       * @param delUnstableSats  Whether unstable satellites will be deleted.
       * @param unstableTime     Number of seconds since last arc change
       *                         that a satellite will be considered as
       *                         unstable.
       */
   SatArcMarker::SatArcMarker( const TypeID& watchFlag,
                               const bool delUnstableSats,
                               const double unstableTime )
      : watchCSFlag(watchFlag), deleteUnstableSats(delUnstableSats)
   {

         // Set unstableTime value
      setUnstablePeriod( unstableTime );

   }  // End of method 'SatArcMarker::SatArcMarker()'



      /* Method to set the number of seconds since last arc change that a
       *  satellite will be considered as unstable.
       *
       * @param unstableTime     Number of seconds since last arc change
       *                         that a satellite will be considered as
       *                         unstable.
       */
   SatArcMarker& SatArcMarker::setUnstablePeriod(const double unstableTime)
   {

         // Check unstableTime value
      if (unstableTime > 0.0)
      {
         unstablePeriod = unstableTime;
      }
      else
      {
         unstablePeriod = 0.0;
      }

      return (*this);

   }  // End of method 'SatArcMarker::setUnstablePeriod()'



      /* Method to get the arc changed epoch.
       * @param sat              Interested SatID.
       */
   CommonTime SatArcMarker::getArcChangedEpoch(const SatID& sat)
   {
      std::map<SatID, CommonTime>::iterator it = satArcChangeMap.find(sat);
      if(it!=satArcChangeMap.end())
      {
         return it->second;
      }
      else
      {
         return CommonTime::BEGINNING_OF_TIME;
      }

   }  // End of method 'SatArcMarker::getArcChangedEpoch()'


      /* Returns a satTypeValueMap object, adding the new data generated
       *  when calling this object.
       *
       * @param epoch     Time of observations.
       * @param gData     Data object holding the data.
       */
   satTypeValueMap& SatArcMarker::Process( const CommonTime& epoch,
                                           satTypeValueMap& gData )
      throw(ProcessingException)
   {

      try
      {

         double flag(0.0);

         SatIDSet satRejectedSet;

            // Loop through all the satellites
         for ( satTypeValueMap::iterator it = gData.begin();
               it != gData.end();
               ++it )
         {
            try
            {
                  // Try to extract the CS flag value
               flag = (*it).second(watchCSFlag);
            }
            catch(...)
            {
                  // If flag is missing, then schedule this satellite
                  // for removal
               satRejectedSet.insert( (*it).first );
               continue;
            }

               // Check if satellite currently has entries
            std::map<SatID, double>::const_iterator itArc(
                                                satArcMap.find( (*it).first ) );
            if( itArc == satArcMap.end() )
            {
                  // If it doesn't have an entry, insert one
               satArcMap[ (*it).first ] = 0.0;
               satArcChangeMap[ (*it).first ] = CommonTime::BEGINNING_OF_TIME;

                  // This is a new satellite
               satIsNewMap[ (*it).first ] = true;
            }

               // Check if we are inside unstable period
            bool insideUnstable(std::abs(epoch-satArcChangeMap[(*it).first]) <=
                                                               unstablePeriod );

               // Satellites can be new only once, and having at least once a
               // flag > 0.0 outside 'unstablePeriod' will make them old.
            if( satIsNewMap[ (*it).first ] &&
                !insideUnstable            &&
                flag <= 0.0 )
            {
               satIsNewMap[ (*it).first ] = false;
            }


               // Check if there was a cycle slip
            if ( flag > 0.0 )
            {
                  // Increment the value of "TypeID::satArc"
               satArcMap[ (*it).first ] = satArcMap[ (*it).first ] + 1.0;

                  // Update arc change epoch
               satArcChangeMap[ (*it).first ] = epoch;

                  // If we want to delete unstable satellites, we must do it
                  // also when arc changes, but only if this SV is not new
               if ( deleteUnstableSats  &&
                    (!satIsNewMap[ (*it).first ]) )
               {
                  satRejectedSet.insert( (*it).first );
               }

            }


               // Test if we want to delete unstable satellites. Only do it
               // if satellite is NOT new and we are inside unstable period
            if ( insideUnstable &&
                 deleteUnstableSats &&
                 ( !satIsNewMap[ (*it).first ] ) )
            {
               satRejectedSet.insert( (*it).first );
            }

               // We will insert satellite arc number
            (*it).second[TypeID::satArc] = satArcMap[ (*it).first ];

         }

            // Remove satellites with missing data
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

   }  // End of method 'SatArcMarker::Process()'



      /* Returns a gnnsSatTypeValue object, adding the new data generated
       *  when calling this object.
       *
       * @param gData    Data object holding the data.
       */
   gnssSatTypeValue& SatArcMarker::Process(gnssSatTypeValue& gData)
      throw(ProcessingException)
   {

      try
      {

         Process(gData.header.epoch, gData.body);

         return gData;

      }
      catch(Exception& u)
      {
            // Throw an exception if something unexpected happens
         ProcessingException e( getClassName() + ":"
                                + u.what() );

         GPSTK_THROW(e);

      }

   }  // End of method 'SatArcMarker::Process()'



      /* Returns a gnnsRinex object, adding the new data generated when
       *  calling this object.
       *
       * @param gData    Data object holding the data.
       */
   gnssRinex& SatArcMarker::Process(gnssRinex& gData)
      throw(ProcessingException)
   {

      try
      {

         Process(gData.header.epoch, gData.body);

         return gData;

      }
      catch(Exception& u)
      {
            // Throw an exception if something unexpected happens
         ProcessingException e( getClassName() + ":"
                                + u.what() );

         GPSTK_THROW(e);

      }

   }  // End of method 'SatArcMarker::Process()'


}  // End of namespace gpstk
