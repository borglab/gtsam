#pragma ident "$Id$"

/**
 * @file ExtractData.cpp
 * This is the base class to ease data extraction from a Rinex3ObsData object.
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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2012
//
//============================================================================


#include "ExtractData.hpp"


namespace gpstk
{


      /* Pull out the selected observation type from a Rinex3ObsData object
       *
       * @param rinexData     The Rinex data set holding the observations
       * @param index         Index representing the observation type. It is
       *                      obtained from corresponding RINEX Obs Header
       *                      using method 'Rinex3ObsHeader::getObsIndex()'.
       *
       * @return
       *  Number of satellites with this kind of data available
       */
   int ExtractData::getData( const Rinex3ObsData& rinexData, int index )
      throw(InvalidRequest)
   {

      try
      {

            // Let's make sure each time we start with clean Vectors
         availableSV.resize(0);
         obsData.resize(0);

            // Create a CheckPRData object with the given limits
         CheckPRData checker(minPRange, maxPRange);

            // Let's define the "it" iterator to visit the observations PRN map
            // RinexSatMap is a map from SatID to RinexObsTypeMap:
            //      std::map<SatID, RinexObsTypeMap>
         Rinex3ObsData::DataMap::const_iterator it;
         for ( it  = rinexData.obs.begin();
               it != rinexData.obs.end();
               it++ )
         {

               // The satellites are stored in the first elements of the map...
            SatID sat(it->first);
               // .. and vectors of available obs are in the second elements
            std::vector<RinexDatum> vecData(it->second);

               // Extract observation value
            double obsValue( (vecData[index]).data );

               // Let's check if we found this type of observation and if it
               // is between the limits
            if ( !(checkData) || (checker.check(obsValue)) )
            {

                  // Store all relevant data of this epoch
               availableSV = availableSV && sat;
               obsData = obsData && obsValue;

            }

         } // End of data extraction from this epoch

      }
      catch(...)
      {
         InvalidRequest e("Unable to get data from Rinex3ObsData object");
         GPSTK_THROW(e);
      }

         // Let's record the number of SV with this type of data available
      numSV = (int)obsData.size();

         // If everything is fine so far, then the results should be valid
      valid = true;

      return numSV;

   }  // End of method 'ExtractData::getData()'


      /* Pull out the selected observation type from a Rinex3ObsData object
       *
       * @param rinexData  The Rinex data set holding the observations
       * @param type       String representing the observation type.
       * @param hdr        RINEX Observation Header for current RINEX file
       */
   int ExtractData::getData( const Rinex3ObsData& rinexData,
                             std::string type,
                             const Rinex3ObsHeader& hdr )
      throw(InvalidRequest)
   {
         // Get the index corresponding to this observation type
      int index( hdr.getObsIndex(type) );

         // Call the appropriate method
      return ( getData(rinexData, index) );

   }  // End of method 'ExtractData::getData()'


}  // End of namespace gpstk

