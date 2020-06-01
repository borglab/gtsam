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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2007, 2008, 2009, 2011 
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
 * @file ComputeWindUp.cpp
 * This class computes the wind-up effect on the phase observables, in radians.
 */

#include "ComputeWindUp.hpp"

using namespace std;

namespace gpstk
{

      // Returns a string identifying this object.
   std::string ComputeWindUp::getClassName() const
   { return "ComputeWindUp"; }



      /* Returns a satTypeValueMap object, adding the new data generated when
       * calling this object.
       *
       * @param time      Epoch corresponding to the data.
       * @param gData     Data object holding the data.
       */
   satTypeValueMap& ComputeWindUp::Process( const CommonTime& time,
                                            satTypeValueMap& gData )
      throw(ProcessingException)
   {

      try
      {

            // Compute Sun position at this epoch
         SunPosition sunPosition;
         Triple sunPos(sunPosition.getPosition(time));

            // Define a Triple that will hold satellite position, in ECEF
         Triple svPos(0.0, 0.0, 0.0);

         SatIDSet satRejectedSet;

            // Loop through all the satellites
         for ( satTypeValueMap::iterator it = gData.begin();
               it != gData.end();
               ++it )
         {

               // First check if this satellite has previous arc information
            if( satArcMap.find( (*it).first ) == satArcMap.end() )
            {
                  // If it doesn't have an entry, insert one
               satArcMap[ (*it).first ] = 0.0;
            };

               // Then, check both if there is arc information, and if current
               // arc number is different from arc number in storage (which
               // means a cycle slip happened)
            if ( (*it).second.find(TypeID::satArc) != (*it).second.end() &&
                 (*it).second(TypeID::satArc) != satArcMap[ (*it).first ] )
            {
                  // If different, update satellite arc in storage
               satArcMap[ (*it).first ] = (*it).second(TypeID::satArc);

                  // Reset phase information
               phase_satellite[ (*it).first ].previousPhase = 0.0;
               phase_station[ (*it).first ].previousPhase = 0.0;

            }


               // Use ephemeris if satellite position is not already computed
            if( ( (*it).second.find(TypeID::satX) == (*it).second.end() ) ||
                ( (*it).second.find(TypeID::satY) == (*it).second.end() ) ||
                ( (*it).second.find(TypeID::satZ) == (*it).second.end() ) )
            {

               if(pEphemeris==NULL)
               {

                     // If ephemeris is missing, then remove all satellites
                  satRejectedSet.insert( (*it).first );

                  continue;

               }
               else
               {

                     // Try to get satellite position
                     // if it is not already computed
                  try
                  {
                        // For our purposes, position at receive time
                        // is fine enough
                     Xvt svPosVel(pEphemeris->getXvt( (*it).first, time ));

                        // If everything is OK, then continue processing.
                     svPos[0] = svPosVel.x.theArray[0];
                     svPos[1] = svPosVel.x.theArray[1];
                     svPos[2] = svPosVel.x.theArray[2];

                  }
                  catch(...)
                  {

                        // If satellite is missing, then schedule it
                        // for removal
                     satRejectedSet.insert( (*it).first );

                     continue;

                  }

               }

            }
            else
            {

                  // Get satellite position out of GDS
               svPos[0] = (*it).second[TypeID::satX];
               svPos[1] = (*it).second[TypeID::satY];
               svPos[2] = (*it).second[TypeID::satZ];

            }  // End of 'if( ( (*it).second.find(TypeID::satX) == ...'


               // Let's get wind-up value in radians, and insert it
               // into GNSS data structure.
            (*it).second[TypeID::windUp] =
               getWindUp((*it).first, time, svPos, sunPos);

         }  // End of 'for (it = gData.begin(); it != gData.end(); ++it)'

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

   }  // End of method 'ComputeWindUp::Process()'



      /* Sets name of "PRN_GPS"-like file containing satellite data.
       * @param name      Name of satellite data file.
       */
   ComputeWindUp& ComputeWindUp::setFilename(const string& name)
   {

      fileData = name;
      satData.open(fileData);

      return (*this);

   }  // End of method 'ComputeWindUp::setFilename()'



      /* Compute the value of the wind-up, in radians.
       * @param sat       Satellite IDmake
       * @param time      Epoch of interest
       * @param satpos    Satellite position, as a Triple
       * @param sunpos    Sun position, as a Triple
       *
       * @return Wind-up computation, in radians
       */
   double ComputeWindUp::getWindUp( const SatID& satid,
                                    const CommonTime& time,
                                    const Triple& sat,
                                    const Triple& sunPosition )
   {

         // Get satellite rotation angle

         // Get vector from Earth mass center to receiver
      Triple rxPos(nominalPos.X(), nominalPos.Y(), nominalPos.Z());

         // Vector from SV to Sun center of mass
      Triple gps_sun( sunPosition-sat );

         // Define rk: Unitary vector from satellite to Earth mass center
      Triple rk( ( (-1.0)*(sat.unitVector()) ) );

         // Define rj: rj = rk x gps_sun, then make sure it is unitary
      Triple rj( (rk.cross(gps_sun)).unitVector() );

         // Define ri: ri = rj x rk, then make sure it is unitary
         // Now, ri, rj, rk form a base in the satellite body reference
         // frame, expressed in the ECEF reference frame
      Triple ri( (rj.cross(rk)).unitVector() );


         // Compute unitary vector vector from satellite to RECEIVER
      Triple rrho( (rxPos-sat).unitVector() );

         // Projection of "rk" vector to line of sight vector (rrho)
      double zk(rrho.dot(rk));

         // Get a vector without components on rk (i.e., belonging
         // to ri, rj plane)
      Triple dpp(rrho-zk*rk);

         // Compute dpp components in ri, rj plane
      double xk(dpp.dot(ri));
      double yk(dpp.dot(rj));

         // Compute satellite rotation angle, in radians
      double alpha1(std::atan2(yk,xk));


         // Get receiver rotation angle

         // Redefine rk: Unitary vector from Receiver to Earth mass center
      rk = (-1.0)*(rxPos.unitVector());

         // Let's define a NORTH unitary vector in the Up, East, North
         // (UEN) topocentric reference frame
      Triple delta(0.0, 0.0, 1.0);

         // Rotate delta to XYZ reference frame
      delta =
         (delta.R2(nominalPos.geodeticLatitude())).R3(-nominalPos.longitude());


         // Computation of reference trame unitary vectors for receiver
         // rj = rk x delta, and make it unitary
      rj = (rk.cross(delta)).unitVector();

         // ri = rj x rk, and make it unitary
      ri = (rj.cross(rk)).unitVector();

         // Projection of "rk" vector to line of sight vector (rrho)
      zk = rrho.dot(rk);

         // Get a vector without components on rk (i.e., belonging
         // to ri, rj plane)
      dpp = rrho-zk*rk;

         // Compute dpp components in ri, rj plane
      xk = dpp.dot(ri);
      yk = dpp.dot(rj);

         // Compute receiver rotation angle, in radians
      double alpha2(std::atan2(yk,xk));

      double wind_up(0.0);

         // Find out if satellite belongs to block "IIR", because
         // satellites of block IIR have a 180 phase shift
      CommonTime time2(time);
      time2.setTimeSystem(TimeSystem::Any);
      	
      if(satData.getBlock( satid, time2 ) == "IIR")
      {
         wind_up = PI;
      }

      alpha1 = alpha1 + wind_up;

      double da1(alpha1-phase_satellite[satid].previousPhase);

      double da2(alpha2-phase_station[satid].previousPhase);

         // Let's avoid problems when passing from 359 to 0 degrees.
      phase_satellite[satid].previousPhase += std::atan2( std::sin(da1),
                                                          std::cos(da1) );

      phase_station[satid].previousPhase += std::atan2( std::sin(da2),
                                                        std::cos(da2) );

         // Compute wind up effect in radians
      wind_up = phase_satellite[satid].previousPhase -
                phase_station[satid].previousPhase;

      return wind_up;

   }  // End of method 'ComputeWindUp::getWindUp()'



}  // End of namespace gpstk
