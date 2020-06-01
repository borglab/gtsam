#pragma ident "$Id$"

/**
 * @file Antenna.cpp
 * Encapsulates the data related to GNSS antennas.
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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2009
//
//============================================================================


#include "Antenna.hpp"



namespace gpstk
{


      /* Common constructor.
       *
       * @param[in] eccL1     Eccentricity Triple (meters) for GPS L1 freq.
       * @param[in] eccL2     Eccentricity Triple (meters) for GPS L2 freq.
       */
   Antenna::Antenna( const Triple& eccL1,
                     const Triple& eccL2 )
   {
         // Add eccentricities
      addAntennaEcc(G01, eccL1);
      addAntennaEcc(G02, eccL2);

   }  // End of constructor 'Antenna::Antenna()'



      /* Common constructor.
       *
       * @param[in] NorthEccL1   North eccentricity (meters) for GPS L1 freq
       * @param[in] EastEccL1    East eccentricity (meters) for GPS L1 freq
       * @param[in] UpEccL1      Up eccentricity (meters) for GPS L1 freq
       * @param[in] NorthEccL2   North eccentricity (meters) for GPS L2 freq
       * @param[in] EastEccL2    East eccentricity (meters) for GPS L2 freq
       * @param[in] UpEccL2      Up eccentricity (meters) for GPS L2 freq
       */
   Antenna::Antenna( double NorthEccL1,
                     double EastEccL1,
                     double UpEccL1,
                     double NorthEccL2,
                     double EastEccL2,
                     double UpEccL2 )
   {
         // Add eccentricities
      addAntennaEcc(G01, NorthEccL1, EastEccL1, UpEccL1);
      addAntennaEcc(G02, NorthEccL2, EastEccL2, UpEccL2);

   }  // End of constructor 'Antenna::Antenna()'



      /* Get antenna data.
       *
       * @param[in] dataType     Antenna data type to be fetched
       */
   std::string Antenna::getAntennaData( AntennaDataType dataType ) const
      throw(InvalidRequest)
   {

         // Look for this frequency in the antenna data map
         // Define iterator
      AntennaDataMap::const_iterator it( antennaData.find(dataType) );
      if( it != antennaData.end() )
      {
            // If we found data for this data type, let's return it
         return (*it).second;
      }
      else
      {
         InvalidRequest e("No data was found for provided data type.");
         GPSTK_THROW(e);
      }
   }  // End of method 'Antenna::getAntennaData()'



      /* Get antenna eccentricity (or 'phase center offset' in Antex
       * parlance) as a Triple in UEN system.
       *
       * @param[in] freq      Frequency
       *
       * @warning The phase center offset Triple is in UEN system.
       */
   Triple Antenna::getAntennaEccentricity( frequencyType freq ) const
      throw(InvalidRequest)
   {

         // Look for this frequency in the antenna eccentricity map
         // Define iterator
      AntennaEccDataMap::const_iterator it( antennaEccMap.find(freq) );
      if( it != antennaEccMap.end() )
      {
            // If we found a Triple for this frequency, let's
            // return it in UEN system
         Triple toReturn( (*it).second[2], (*it).second[1], (*it).second[0] );
         return toReturn;
      }
      else
      {
         InvalidRequest e("No eccentricities were found for this frequency.");
         GPSTK_THROW(e);
      }

   }  // End of method 'Antenna::getAntennaEccentricity()'



      /* Get antenna phase center variation. Use this method when you
       * don't have azimuth dependent phase center patterns.
       *
       * This method returns a Triple, in UEN system, with the
       * elevation-dependent phase center variation.
       *
       * @param[in] freq      Frequency
       * @param[in] elevation Elevation (degrees)
       *
       * @warning The phase center variation Triple is in UEN system.
       */
   Triple Antenna::getAntennaPCVariation( frequencyType freq,
                                          double elevation ) const
      throw(InvalidRequest)
   {

         // The angle should be measured respect to zenith
      double angle( 90.0 - elevation );

         // Check that angle is within limits
      if( ( angle < zen1 ) ||
          ( angle > zen2 ) )
      {
         InvalidRequest e("Elevation is out of allowed range.");
         GPSTK_THROW(e);
      }

         // Look for this frequency in noAziMap
         // Define iterator
      NoAziDataMap::const_iterator it( noAziMap.find(freq) );
      if( it != noAziMap.end() )
      {
            // Get the normalized angle
         double normalizedAngle( (angle-zen1)/dzen );

            // Return result. Only the "Up" component is important.
         Triple result( linearInterpol( (*it).second, normalizedAngle ),
                        0.0,
                        0.0 );

         return result;
      }
      else
      {
         InvalidRequest e("No data was found for this frequency.");
         GPSTK_THROW(e);
      }

   }  // End of method 'Antenna::getAntennaPCVariation()'



      /* Get antenna phase center variation.
       *
       * This method returns a Triple, in UEN system, with the
       * elevation and azimuth-dependent phase center variation.
       *
       * @param[in] freq      Frequency
       * @param[in] elevation Elevation (degrees)
       * @param[in] azimuth   Azimuth (degrees)
       *
       * @warning The phase center variation Triple is in UEN system.
       */
   Triple Antenna::getAntennaPCVariation( frequencyType freq,
                                          double elevation,
                                          double azimuth ) const
      throw(InvalidRequest)
   {

         // The angle should be measured respect to zenith
      double angle( 90.0 - elevation );

         // Check that angle is within limits
      if( ( angle < zen1 ) ||
          ( angle > zen2 ) )
      {
         InvalidRequest e("Elevation is out of allowed range.");
         GPSTK_THROW(e);
      }

         // Reduce azimuth to 0 <= azimuth < 360 interval
      while( azimuth < 0.0 )
      {
         azimuth += 360.0;
      }
      while( azimuth >= 360.0 )
      {
         azimuth -= 360.0;
      }

         // Look for this frequency in pcMap
         // Define iterator
      PCDataMap::const_iterator it( pcMap.find(freq) );
      if( it != pcMap.end() )
      {

            // Get the right azimuth interval
         const double lowerAzimuth( std::floor(azimuth/dazi) * dazi );
         const double upperAzimuth( lowerAzimuth + dazi );

               // Look for data vectors
         AzimuthDataMap::const_iterator it2( (*it).second.find(lowerAzimuth) );
         AzimuthDataMap::const_iterator it3( (*it).second.find(upperAzimuth) );

            // Find the fraction from 'lowerAzimuth'
         const double fractionalAzimuth( ( azimuth - lowerAzimuth ) /
                                         ( upperAzimuth - lowerAzimuth ) );

            // Check if 'azimuth' exactly corresponds to a value in the map
         if( fractionalAzimuth == 0.0 )
         {
               // Check if there is data for 'lowerAzimuth'
            if( it2 != (*it).second.end() )
            {
                  // Get the normalized angle
               const double normalizedAngle( (angle-zen1)/dzen );

                  // Return result. Only the "Up" component is important.
               Triple result( linearInterpol( (*it2).second, normalizedAngle ),
                              0.0,
                              0.0 );

               return result;

            }
            else
            {
               InvalidRequest e("No data was found for this azimuth.");
               GPSTK_THROW(e);
            }
         }
         else
         {
               // We have to interpolate
               // Check if there is data for 'lowerAzimuth' and 'upperAzimuth'
            if( it2 != (*it).second.end() &&
                it3 != (*it).second.end() )
            {
                  // Get the normalized angle
               const double normalizedAngle( (angle-zen1)/dzen );

                  // Find values corresponding to both azimuths
               double val1( linearInterpol( (*it2).second, normalizedAngle ) );
               double val2( linearInterpol( (*it3).second, normalizedAngle ) );

                  // Return result. Only the "Up" component is important.
               Triple result( ( val1 + (val2-val1) * fractionalAzimuth ),
                              0.0,
                              0.0 );

               return result;

            }
            else
            {
               InvalidRequest e("Not enough data was found for this azimuth.");
               GPSTK_THROW(e);
            }
         }

      }
      else
      {

         InvalidRequest e("No data was found for this frequency.");
         GPSTK_THROW(e);

      }  // End of 'if( it != pcMap.end() )...'

   }  // End of method 'Antenna::getAntennaPCVariation()'



      /* Add antenna phase center eccentricities, in METERS.
       *
       * @param[in] freq        Frequency.
       * @param[in] northEcc    North eccentricity component, in METERS.
       * @param[in] eastEcc     East eccentricity component, in METERS.
       * @param[in] upEcc       Up eccentricity component, in METERS.
       */
   Antenna Antenna::addAntennaEcc( frequencyType freq,
                                   double northEcc,
                                   double eastEcc,
                                   double upEcc )
   {

         // Build a Triple with the eccentricities
      Triple ecc(northEcc, eastEcc, upEcc);

         // Get Triple into eccentricities map
      antennaEccMap[freq] = ecc;

         // Return this object
      return (*this);

   }  // End of method 'Antenna::addAntennaEcc()'



      /* Add antenna phase center RMS eccentricities, in METERS.
       *
       * @param[in] freq        Frequency.
       * @param[in] northRMS    North eccentricity RMS component, in METERS.
       * @param[in] eastRMS     East eccentricity RMS component, in METERS.
       * @param[in] upRMS       Up eccentricity RMS component, in METERS.
       */
   Antenna Antenna::addAntennaRMSEcc( frequencyType freq,
                                      double northRMS,
                                      double eastRMS,
                                      double upRMS )
   {

         // Build a Triple with eccentricities RMS
      Triple ecc(northRMS, eastRMS, upRMS);

         // Get Triple into eccentricities map
      antennaRMSEccMap[freq] = ecc;

         // Return this object
      return (*this);

   }  // End of method 'Antenna::addAntennaRMSEcc()'



      // Returns if this object is valid. The validity criteria is to
      // have a non-empty 'antennaData' map AND a non-empty 'antennaEccMap'.
   bool Antenna::isValid() const
   {

      if( getAntennaDataSize()   > 0 &&
          getAntennaEccMapSize() > 0 )
      {
         return true;
      }
      else
      {
         return false;
      }

   }  // End of method 'Antenna::isValid()'



      /* Linear interpolation as function of normalized angle
       *
       * @param[in] dataVector         std::vector holding data.
       * @param[in] normalizedAngle    Normalized angle.
       *
       * 'normalizedAngle' is a value corresponding to the original angle
       * divided by the angle interval.
       */
   double Antenna::linearInterpol( const std::vector<double>& dataVector,
                                   double normalizedAngle ) const
   {

         // Get the index value 'normalizedAngle' is equivalent to
      int index( static_cast<int>( std::floor(normalizedAngle) ) );

         // Find the fraction from 'index'
      double fraction( normalizedAngle - std::floor(normalizedAngle) );

         // Check if 'normalizedAngle' is exactly a value in the map
      if( fraction == 0.0 )
      {
               // Return result
            return dataVector[index];
      }
      else
      {
            // In this case, we have to interpolate
         double val1( dataVector[index] );
         double val2( dataVector[index+1] );

             // Return result
         return ( val1 + (val2-val1) * fraction );
      }

   }  // End of method 'Antenna::elevationInterpol()'



}  // End of namespace gpstk
