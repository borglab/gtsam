#pragma ident "$Id$"

/**
 * @file ExtractCombinationData.hpp
 * This is the base class to ease extraction of a combination of data from
 * a Rinex3ObsData object.
 */


#ifndef GPSTK_EXTRACTCOMBINATIONDATA_HPP
#define GPSTK_EXTRACTCOMBINATIONDATA_HPP


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
//  Dagoberto Salazar - gAGE. 2006, 2012
//
//============================================================================


#include "ExtractData.hpp"


namespace gpstk
{

      /** @addtogroup RinexObs */
      //@{


      /// This class eases the extraction of a combination of data from
      /// a Rinex3ObsData object.
   class ExtractCombinationData : public ExtractData
   {
   public:

         /// Default constructor
      ExtractCombinationData()
      { valid = false; checkData = true; };


         /** Get a combination of observations from a Rinex3ObsData object
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
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Woverloaded-virtual"
       virtual int getData( const Rinex3ObsData& rinexData,
                           int indexObs1,
                           int indexObs2 )
         throw(InvalidRequest);

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
      virtual int getData( const Rinex3ObsData& rinexData,
                           std::string type1,
                           std::string type2,
                           const Rinex3ObsHeader& hdr )
         throw(InvalidRequest);
#pragma clang diagnostic pop


         /// Destructor
      virtual ~ExtractCombinationData() {};


   protected:


         /// Compute the combination of observables. You must define this
         /// method according to your specific combination.
      virtual double getCombination( double obs1, double obs2 )
         throw(InvalidRequest) = 0;


   }; // End of class 'ExtractCombinationData'


      //@}
   
}  // End of namespace gpstk

#endif   // GPSTK_EXTRACTCOMBINATIONDATA_HPP
