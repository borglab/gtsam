#pragma ident "$Id$"

/**
 * @file ExtractCombinationData.cpp
 * This is the base class to ease extraction of a combination of data from
 * a Rinex3ObsData object.
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


#include "ExtractCombinationData.hpp"

using namespace std;

namespace gpstk
{

      /* Get a combination of observations from a Rinex3ObsData object
       *
       * @param rinexData  The Rinex data set holding the observations
       * @param indexObs1  Index representing the observation type #1.
       * @param indexObs2  Index representing the observation type #2.
       *
       * @return
       *    Number of SVs with this combination of observables available
       *
       * @note
       *    The indexes are obtained from the RINEX Observation Header
       *    using method 'Rinex3ObsHeader::getObsIndex()'.
       */
   int ExtractCombinationData::getData( const Rinex3ObsData& rinexData,
                                        int indexObs1,
                                        int indexObs2 )
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
            vector<RinexDatum> vecData(it->second);

               // Extract observation values
            double obsValue1( (vecData[indexObs1]).data );
            double obsValue2( (vecData[indexObs2]).data );

               // Compute the combination
            double combinationValue( getCombination( obsValue1, obsValue2 ) );

               // Let's check if the combination is between the limits
            if ( !(checkData) || (checker.check(combinationValue)) )
            {

                  // Store all relevant data of this epoch
               availableSV = availableSV && sat;
               obsData = obsData && combinationValue;

            }

         }  // End of data combination extraction from this epoch

      }
      catch(...)
      {
         InvalidRequest e("Unable to compute combination from Rinex3ObsData object");
         GPSTK_THROW(e);
      }

         // Let's record the number of SV with this type of data available
      numSV = (int)obsData.size();

         // If everything is fine so far, then the results should be valid
      valid = true;

      return numSV;

   };  // End of method 'ExtractCombinationData::getData()'


      /** Get a combination of observations from a Rinex3ObsData object
          *
          * @param rinexData  The Rinex data set holding the observations.
          * @param type1      String representing observation type #1.
          * @param type2      String representing observation type #2.
          * @param hdr        RINEX Observation Header for current RINEX file.
          *
          * @return
          *    Number of SVs with this combination of observables available
          */
   int ExtractCombinationData::getData( const Rinex3ObsData& rinexData,
                                        std::string type1,
                                        std::string type2,
                                        const Rinex3ObsHeader& hdr )
   throw(InvalidRequest)
   {
         // Get the indexes corresponding to these observation types
      int index1( hdr.getObsIndex(type1) );
      int index2( hdr.getObsIndex(type2) );

         // Call the appropriate method
      return ( getData(rinexData, index1, index2) );

   }  // End of method 'ExtractData::getData()'


}  // End of namespace gpstk

