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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2008, 2011
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
 * @file EclipsedSatFilter.cpp
 * This class filters out satellites that are eclipsed by Earth shadow.
 */

#include <gtsam/gpstk/EclipsedSatFilter.hpp>


namespace gpstk
{

      // Returns a string identifying this object.
   std::string EclipsedSatFilter::getClassName() const
   { return "EclipsedSatFilter"; }



      /* Sets aperture of shadow cone, in degrees.
       *
       * @param angle   Aperture angle of shadow cone, in degrees.
       *
       * \warning Valid values are within 0 and 90 degrees.
       */
   EclipsedSatFilter& EclipsedSatFilter::setConeAngle(const double angle)
   {
         // Check that cone angle is within sane limits
      if( (angle >= 0.0) && (angle < 90.0) )
      {
            // If angle is right, change cone aperture angle
         coneAngle = angle;
      }

      return (*this);

   }  // End of method 'EclipsedSatFilter::setConeAngle()'



      /* Sets time after exiting shadow that satellite will still be 
       *  filtered out, in seconds.
       * @param pShTime    Time after exiting shadow that satellite will
       *                   still be filtered out, in seconds.
       */
   EclipsedSatFilter& EclipsedSatFilter::setPostShadowPeriod(
                                                         const double pShTime)
   {
         // Check that post shadow period is positive
      if( pShTime >= 0.0 )
      {
         postShadowPeriod = pShTime;
      }

      return (*this);

   }  // End of method 'EclipsedSatFilter::setPostShadowPeriod()'



      /* Returns a satTypeValueMap object, adding the new data generated
       *  when calling this object.
       *
       * @param epoch     Time of observations.
       * @param gData     Data object holding the data.
       */
   satTypeValueMap& EclipsedSatFilter::Process( const CommonTime& epoch,
                                                satTypeValueMap& gData )
      throw(ProcessingException)
   {

      try
      {

         SatIDSet satRejectedSet;

            // Set the threshold to declare that satellites are in eclipse
            // threshold = cos(180 - coneAngle/2)
         double threshold( std::cos(PI - coneAngle/2.0*DEG_TO_RAD) );

            // Compute Sun position at this epoch, and store it in a Triple
         SunPosition sunPosition;
         Triple sunPos(sunPosition.getPosition(epoch));

            // Define a Triple that will hold satellite position, in ECEF
         Triple svPos(0.0, 0.0, 0.0);

            // Loop through all the satellites
         satTypeValueMap::iterator it;
         for (it = gData.begin(); it != gData.end(); ++it) 
         {
               // Check if satellite position is not already computed
            if( ( (*it).second.find(TypeID::satX) == (*it).second.end() ) ||
                ( (*it).second.find(TypeID::satY) == (*it).second.end() ) ||
                ( (*it).second.find(TypeID::satZ) == (*it).second.end() ) )
            {

                  // If satellite position is missing, then schedule this 
                  // satellite for removal
               satRejectedSet.insert( (*it).first );
               continue;
            }
            else
            {
                  // Get satellite position out of GDS
               svPos[0] = (*it).second[TypeID::satX];
               svPos[1] = (*it).second[TypeID::satY];
               svPos[2] = (*it).second[TypeID::satZ];
            }

               // Unitary vector from Earth mass center to satellite
            Triple rk( svPos.unitVector() );

               // Unitary vector from Earth mass center to Sun
            Triple ri( sunPos.unitVector() );

               // Get dot product between unitary vectors = cosine(angle)
            double cosAngle(ri.dot(rk));

               // Check if satellite is within shadow
            if(cosAngle <= threshold)
            {
                  // If satellite is eclipsed, then schedule it for removal
               satRejectedSet.insert( (*it).first );

                  // Keep track of last known epoch the satellite was in eclipse
               shadowEpoch[(*it).first] = epoch;

               continue;
            }
            else
            {
                  // Maybe the satellite is out fo shadow, but it was recently
                  // in eclipse. Check also that.
               if( shadowEpoch.find( (*it).first ) != shadowEpoch.end() )
               {
                     // If satellite was recently in eclipse, check if elapsed
                     // time is less or equal than postShadowPeriod
                  if( std::abs( ( epoch - shadowEpoch[(*it).first] ) ) <=
                                postShadowPeriod )
                  {
                        // Satellite left shadow, but too recently. Delete it
                     satRejectedSet.insert( (*it).first );
                  }
                  else
                  {
                        // If satellite left shadow a long time ago, set it free
                     shadowEpoch.erase( (*it).first );
                  }
               }
            }

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

   }  // End of 'EclipsedSatFilter::Process()'


      /* Returns a gnnsRinex object, adding the new data generated when
       *  calling this object.
       *
       * @param gData    Data object holding the data.
       */
   gnssRinex& EclipsedSatFilter::Process(gnssRinex& gData)
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

   }  // End of 'EclipsedSatFilter::Process()'


} // End of namespace gpstk
