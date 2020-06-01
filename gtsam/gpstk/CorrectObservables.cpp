#pragma ident "$Id$"

/**
 * @file CorrectObservables.cpp
 * This class corrects observables from effects such as antenna excentricity,
 * difference in phase centers, offsets due to tide effects, etc.
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
//  Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2007, 2008, 2009
//
//============================================================================


#include <gtsam/gpstk/CorrectObservables.hpp>


namespace gpstk
{

// Index initially assigned to this class
int CorrectObservables::classIndex = 4300000;


// Returns an index identifying this object.
int CorrectObservables::getIndex() const
{
        return index;
}


// Returns a string identifying this object.
std::string CorrectObservables::getClassName() const
{
        return "CorrectObservables";
}



/* Returns a satTypeValueMap object, adding the new data generated
 * when calling this object.
 *
 * @param time      Epoch corresponding to the data.
 * @param gData     Data object holding the data.
 */
satTypeValueMap& CorrectObservables::Process( const CommonTime& time,
                                              satTypeValueMap& gData )
throw(ProcessingException)
{

        try
        {

                // Compute station latitude and longitude
                double lat(nominalPos.geodeticLatitude());
                double lon(nominalPos.longitude());

                // Define station position as a Triple, in ECEF
                Triple staPos( nominalPos.getX(),
                               nominalPos.getY(),
                               nominalPos.getZ() );


                // Compute initial displacement vectors, in meters [UEN]
                Triple initialBias( extraBiases + monumentVector );
                Triple dispL1( initialBias );
                Triple dispL2( initialBias );
                Triple dispL5( initialBias );
                Triple dispL6( initialBias );
                Triple dispL7( initialBias );
                Triple dispL8( initialBias );


                // Check if we have a valid Antenna object
                if( antenna.isValid() )
                {
                        // Compute phase center offsets
                        L1PhaseCenter = antenna.getAntennaEccentricity( Antenna::G01 );
                        L2PhaseCenter = antenna.getAntennaEccentricity( Antenna::G02 );
                }



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

                        } // End of 'if( ( (*it).second.find(TypeID::satX) == ...'
                        else
                        {

                                // Get satellite position out of GDS
                                svPos[0] = (*it).second[TypeID::satX];
                                svPos[1] = (*it).second[TypeID::satY];
                                svPos[2] = (*it).second[TypeID::satZ];

                        }


                        // Declare the variables where antenna PC variations
                        // will be stored. Only values for L1 and L2 will be
                        // computed, in UEN system
                        Triple L1Var( 0.0, 0.0, 0.0 );
                        Triple L2Var( 0.0, 0.0, 0.0 );

                        // Check if we have a valid Antenna object
                        if( antenna.isValid() )
                        {

                                // Check if we have elevation information
                                if( (*it).second.find(TypeID::elevation) != (*it).second.end() )
                                {

                                        // Get elevation value
                                        double elev( (*it).second[TypeID::elevation] );

                                        // Check if azimuth is also required
                                        if( !useAzimuth )
                                        {

                                                // In this case, use methods that only need elevation
                                                try
                                                {

                                                        // Compute phase center variation values
                                                        L1Var = antenna.getAntennaPCVariation( Antenna::G01,
                                                                                               elev );
                                                        L2Var = antenna.getAntennaPCVariation( Antenna::G02,
                                                                                               elev );

                                                }
                                                catch(InvalidRequest& ir)
                                                {
                                                        // Throw an exception if something unexpected
                                                        // happens
                                                        ProcessingException e( getClassName() + ":"
                                                                               + StringUtils::asString( getIndex() ) + ":"
                                                                               + "Unexpected problem found when trying to "
                                                                               + "compute antenna offsets" );

                                                        GPSTK_THROW(e);
                                                } // End fo 'try'

                                        }
                                        else
                                        {

                                                // Check if we have azimuth information
                                                if( (*it).second.find(TypeID::azimuth) !=
                                                    (*it).second.end() )
                                                {

                                                        // Get azimuth value
                                                        double azim( (*it).second[TypeID::azimuth] );

                                                        // Use a gentle fallback mechanism to get antenna
                                                        // phase center variations
                                                        try
                                                        {
                                                                // Compute phase center variation values
                                                                L1Var = antenna.getAntennaPCVariation( Antenna::G01,
                                                                                                       elev,
                                                                                                       azim );

                                                                L2Var = antenna.getAntennaPCVariation( Antenna::G02,
                                                                                                       elev,
                                                                                                       azim );

                                                        }
                                                        catch(InvalidRequest& ir)
                                                        {
                                                                // We  "graceful degrade" to a simpler mechanism
                                                                try
                                                                {

                                                                        // Compute phase center variation values
                                                                        L1Var =
                                                                                antenna.getAntennaPCVariation( Antenna::G01,
                                                                                                               elev );

                                                                        L2Var =
                                                                                antenna.getAntennaPCVariation( Antenna::G02,
                                                                                                               elev );

                                                                }
                                                                catch(InvalidRequest& ir)
                                                                {
                                                                        // Throw an exception if something unexpected
                                                                        // happens
                                                                        ProcessingException e( getClassName() + ":"
                                                                                               + StringUtils::asString( getIndex() ) + ":"
                                                                                               + "Unexpected problem found when trying to "
                                                                                               + "compute antenna offsets" );

                                                                        GPSTK_THROW(e);
                                                                } // End fo 'try'

                                                        } // End fo 'try'

                                                }
                                                else
                                                {

                                                        // Throw an exception if something unexpected happens
                                                        ProcessingException e( getClassName() + ":"
                                                                               + StringUtils::asString( getIndex() ) + ":"
                                                                               + "Azimuth information could not be found, "
                                                                               + "so antenna PC offsets can not be computed");

                                                        GPSTK_THROW(e);

                                                } // End of 'if( (*it).second.find(TypeID::azimuth) !=...'

                                        } // End of 'if( !useAzimuth )'

                                }
                                else
                                {

                                        // Throw an exception if there is no elevation data
                                        ProcessingException e( getClassName() + ":"
                                                               + StringUtils::asString( getIndex() ) + ":"
                                                               + "Elevation information could not be found, "
                                                               + "so antenna PC offsets can not be computed" );

                                        GPSTK_THROW(e);

                                } // End of 'if( (*it).second.find(TypeID::elevation) != ...'


                        } // End of 'if( antenna.isValid() )...'


                        // Update displacement vectors with current phase centers
                        Triple dL1( dispL1 + L1PhaseCenter - L1Var );
                        Triple dL2( dispL2 + L2PhaseCenter - L2Var );
                        Triple dL5( dispL5 + L5PhaseCenter );
                        Triple dL6( dispL6 + L6PhaseCenter );
                        Triple dL7( dispL7 + L7PhaseCenter );
                        Triple dL8( dispL8 + L8PhaseCenter );

                        // Compute vector station-satellite, in ECEF
                        Triple ray(svPos - staPos);

                        // Rotate vector ray to UEN reference frame
                        ray = (ray.R3(lon)).R2(-lat);

                        // Convert ray to an unitary vector
                        ray = ray.unitVector();

                        // Compute corrections = displacement vectors components
                        // along ray direction.
                        double corrL1(dL1.dot(ray));
                        double corrL2(dL2.dot(ray));
                        double corrL5(dL5.dot(ray));
                        double corrL6(dL6.dot(ray));
                        double corrL7(dL7.dot(ray));
                        double corrL8(dL8.dot(ray));


                        // Find which observables are present, and then
                        // apply corrections

                        // Look for C1
                        if( (*it).second.find(TypeID::C1) != (*it).second.end() )
                        {
                                (*it).second[TypeID::C1] = (*it).second[TypeID::C1] + corrL1;
                        };

                        // Look for P1
                        if( (*it).second.find(TypeID::P1) != (*it).second.end() )
                        {
                                (*it).second[TypeID::P1] = (*it).second[TypeID::P1] + corrL1;
                        };

                        // Look for L1
                        if( (*it).second.find(TypeID::L1) != (*it).second.end() )
                        {
                                (*it).second[TypeID::L1] = (*it).second[TypeID::L1] + corrL1;
                        };

                        // Look for C2
                        if( (*it).second.find(TypeID::C2) != (*it).second.end() )
                        {
                                (*it).second[TypeID::C2] = (*it).second[TypeID::C2] + corrL2;
                        };

                        // Look for P2
                        if( (*it).second.find(TypeID::P2) != (*it).second.end() )
                        {
                                (*it).second[TypeID::P2] = (*it).second[TypeID::P2] + corrL2;
                        };

                        // Look for L2
                        if( (*it).second.find(TypeID::L2) != (*it).second.end() )
                        {
                                (*it).second[TypeID::L2] = (*it).second[TypeID::L2] + corrL2;
                        };

                        // Look for C5
                        if( (*it).second.find(TypeID::C5) != (*it).second.end() )
                        {
                                (*it).second[TypeID::C5] = (*it).second[TypeID::C5] + corrL5;
                        };

                        // Look for L5
                        if( (*it).second.find(TypeID::L5) != (*it).second.end() )
                        {
                                (*it).second[TypeID::L5] = (*it).second[TypeID::L5] + corrL5;
                        };

                        // Look for C6
                        if( (*it).second.find(TypeID::C6) != (*it).second.end() )
                        {
                                (*it).second[TypeID::C6] = (*it).second[TypeID::C6] + corrL6;
                        };

                        // Look for L6
                        if( (*it).second.find(TypeID::L6) != (*it).second.end() )
                        {
                                (*it).second[TypeID::L6] = (*it).second[TypeID::L6] + corrL6;
                        };

                        // Look for C7
                        if( (*it).second.find(TypeID::C7) != (*it).second.end() )
                        {
                                (*it).second[TypeID::C7] = (*it).second[TypeID::C7] + corrL7;
                        };

                        // Look for L7
                        if( (*it).second.find(TypeID::L7) != (*it).second.end() )
                        {
                                (*it).second[TypeID::L7] = (*it).second[TypeID::L7] + corrL7;
                        };

                        // Look for C8
                        if( (*it).second.find(TypeID::C8) != (*it).second.end() )
                        {
                                (*it).second[TypeID::C8] = (*it).second[TypeID::C8] + corrL8;
                        };

                        // Look for L8
                        if( (*it).second.find(TypeID::L8) != (*it).second.end() )
                        {
                                (*it).second[TypeID::L8] = (*it).second[TypeID::L8] + corrL8;
                        };

                }

                // Remove satellites with missing data
                gData.removeSatID(satRejectedSet);

                return gData;

        }
        catch(Exception& u)
        {
                // Throw an exception if something unexpected happens
                ProcessingException e( getClassName() + ":"
                                       + StringUtils::asString( getIndex() ) + ":"
                                       + u.what() );

                GPSTK_THROW(e);

        }

}     // End of method 'CorrectObservables::Process()'


}  // End of namespace gpstk
