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
 * @file ComputeSatPCenter.cpp
 * This class computes the satellite antenna phase correction, in meters.
 */

#include <gtsam/gpstk/ComputeSatPCenter.hpp>

using namespace std;

namespace gpstk
{

      // Returns a string identifying this object.
   std::string ComputeSatPCenter::getClassName() const
   { return "ComputeSatPCenter"; }


      /* Returns a satTypeValueMap object, adding the new data generated when
       * calling this object.
       *
       * @param time      Epoch corresponding to the data.
       * @param gData     Data object holding the data.
       */
   satTypeValueMap& ComputeSatPCenter::Process(const CommonTime& time,
                                           satTypeValueMap& gData)
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
         satTypeValueMap::iterator it;
         for (it = gData.begin(); it != gData.end(); ++it)
         {

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


               // Let's get the satellite antenna phase correction value in
               // meters, and insert it in the GNSS data structure.
            (*it).second[TypeID::satPCenter] =
               getSatPCenter((*it).first, time, svPos, sunPos);

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

   }  // End of method 'ComputeSatPCenter::Process()'



      /* Sets name of "PRN_GPS"-like file containing satellite data.
       * @param name      Name of satellite data file.
       */
   ComputeSatPCenter& ComputeSatPCenter::setFilename(const string& name)
   {

      fileData = name;
      satData.open(fileData);

      return (*this);

   }  // End of method 'ComputeSatPCenter::setFilename()'



      /* Compute the value of satellite antenna phase correction, in meters.
       * @param satid     Satellite ID
       * @param time      Epoch of interest
       * @param satpos    Satellite position, as a Triple
       * @param sunpos    Sun position, as a Triple
       *
       * @return Satellite antenna phase correction, in meters.
       */
   double ComputeSatPCenter::getSatPCenter( const SatID& satid,
                                            const CommonTime& time,
                                            const Triple& satpos,
                                            const Triple& sunPosition )
   {

         // Unitary vector from satellite to Earth mass center (ECEF)
      Triple rk( ( (-1.0)*(satpos.unitVector()) ) );

         // Unitary vector from Earth mass center to Sun (ECEF)
      Triple ri( sunPosition.unitVector() );

         // rj = rk x ri: Rotation axis of solar panels (ECEF)
      Triple rj(rk.cross(ri));

         // Redefine ri: ri = rj x rk (ECEF)
      ri = rj.cross(rk);

         // Let's convert ri to an unitary vector. (ECEF)
      ri = ri.unitVector();

         // Get vector from Earth mass center to receiver
      Triple rxPos(nominalPos.X(), nominalPos.Y(), nominalPos.Z());

         // Compute unitary vector vector from satellite to RECEIVER
      Triple rrho( (rxPos-satpos).unitVector() );

         // When not using Antex information, if satellite belongs to block
         // "IIR" its correction is 0.0, else it will depend on satellite model.

         // This variable that will hold the correction, 0.0 by default
      double svPCcorr(0.0);

         // Check is Antex antenna information is available or not, and if
         // available, whether satellite phase center information is absolute
         // or relative
      bool absoluteModel( false );
      if( pAntexReader != NULL )
      {
         absoluteModel = pAntexReader->isAbsolute();
      }

      if( absoluteModel )
      {

            // We will need the elevation, in degrees. It is found using
            // dot product and the corresponding unitary angles

         double nadir = std::acos( rrho.dot(rk) ) * RAD_TO_DEG;

            // The nadir angle should always smaller than 14.0 deg, 
            // but some times it's a bit bigger than 14.0 deg, we 
            // force it to 14.0 deg to stop throwing an exception.
            // The Reference is available at:
            // http://igscb.jpl.nasa.gov/igscb/resource/pubs/02_ott/session_8.pdf
         nadir = (nadir>14) ? 14.0 : nadir;

         double elev( 90.0 - nadir );

            // Get satellite information in Antex format. Currently this
            // only works for GPS and Glonass.
         if( satid.system == SatID::systemGPS )
         {
            std::stringstream sat;
            sat << "G";
            if( satid.id < 10 )
            {
               sat << "0";
            }
            sat << satid.id;

               // Get satellite antenna information out of AntexReader object
            Antenna antenna( pAntexReader->getAntenna( sat.str(), time ) );

               // Get antenna eccentricity for frequency "G01" (L1), in
               // satellite reference system.
               // NOTE: It is NOT in ECEF, it is in UEN!!!
            Triple satAnt( antenna.getAntennaEccentricity( Antenna::G01) );

               // Now, get the phase center variation.
            Triple var( antenna.getAntennaPCVariation( Antenna::G01, elev) );

               // We must substract them
            satAnt = satAnt - var;

                  // Change to ECEF
            Triple svAntenna( satAnt[2]*ri + satAnt[1]*rj + satAnt[0]*rk );

               // Projection of "svAntenna" vector to line of sight vector rrho
            svPCcorr =  (rrho.dot(svAntenna));

         }
         else
         {
               // Check if this satellite belongs to Glonass system
            if( satid.system == SatID::systemGlonass )
            {
               std::stringstream sat;
               sat << "R";
               if( satid.id < 10 )
               {
                  sat << "0";
               }
               sat << satid.id;

                  // Get satellite antenna information out of AntexReader object
               Antenna antenna( pAntexReader->getAntenna( sat.str(), time ) );

                  // Get antenna offset for frequency "R01" (Glonass), in
                  // satellite reference system.
                  // NOTE: It is NOT in ECEF, it is in UEN!!!
               Triple satAnt( antenna.getAntennaEccentricity( Antenna::R01) );

                  // Now, get the phase center variation.
               Triple var( antenna.getAntennaPCVariation( Antenna::R01, elev) );

                  // We must substract them
               satAnt = satAnt - var;

                     // Change to ECEF
               Triple svAntenna( satAnt[2]*ri + satAnt[1]*rj + satAnt[0]*rk );

                  // Project "svAntenna" vector to line of sight vector rrho
               svPCcorr = (rrho.dot(svAntenna));

            }
            else
            {
                  // In this case no correction will be computed
               svPCcorr = 0.0;
            }

         }  // End of 'if( satid.system == SatID::systemGPS )...'

      }
      else
      {
            // If no Antex information is given, or if phase center information
            // uses a relative model, then use a simpler, older approach

            // Please note that in this case all GLONASS satellite are
            // considered as having phase center at (0.0, 0.0, 0.0). The former
            // is not true for 'GLONASS-M' satellites (-0.545, 0.0, 0.0 ), but
            // currently there is no simple way to take this into account.

            // For satellites II and IIA:
         if( (satData.getBlock( satid, time ) == "II") ||
             (satData.getBlock( satid, time ) == "IIA") )
         {

               // First, build satellite antenna vector for models II/IIA
            Triple svAntenna(0.279*ri + 1.023*rk);

               // Projection of "svAntenna" vector to line of sight vector rrho
            svPCcorr =  (rrho.dot(svAntenna));

         }
         else
         {
               // For satellites belonging to block "I"
            if( (satData.getBlock( satid, time ) == "I") )
            {

                  // First, build satellite antenna vector for model I
               Triple svAntenna(0.210*ri + 0.854*rk);

                  // Projection of "svAntenna" to line of sight vector (rrho)
               svPCcorr =  (rrho.dot(svAntenna));
            }

         }  // End of 'if( (satData.getBlock( satid, time ) == "II") ||...'

      }  // End of 'if( absoluteModel )...'


         // This correction is interpreted as an "advance" in the signal,
         // instead of a delay. Therefore, it has negative sign
      return (-svPCcorr);

   }  // End of method 'ComputeSatPCenter::getSatPCenter()'



}  // End of namespace gpstk
