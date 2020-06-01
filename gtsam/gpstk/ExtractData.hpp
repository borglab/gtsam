#pragma ident "$Id$"

/**
 * @file ExtractData.hpp
 * This is the base class to ease data extraction from a RinexObsData object.
 */

#ifndef GPSTK_EXTRACTDATA_HPP
#define GPSTK_EXTRACTDATA_HPP

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


#include "Exception.hpp"
#include "Rinex3ObsHeader.hpp"
#include "Rinex3ObsData.hpp"
#include "CheckPRData.hpp"
#include "Vector.hpp"


namespace gpstk
{

      /** @addtogroup RinexObs */
      //@{


      /// This is the base class to ease data extraction from a RinexObsData object.
   class ExtractData
   {
   public:

         /// Return validity of data
      inline bool isValid(void) const
      { return valid; }


         /// Number of satellites with available data
      int numSV;


         /// Vector with the PRN of satellites with available data.
      Vector<SatID> availableSV;


         /// Vector holding the available data
      Vector<double> obsData;


         /// Default constructor
      ExtractData()
         : checkData(true), valid(false), minPRange(15000000.0),
           maxPRange(30000000.0)
      {};


         /** Pull out the selected observation type from a Rinex3ObsData object
          *
          * @param rinexData  The Rinex data set holding the observations
          * @param index      Index representing the observation type. It is
          *                   obtained from corresponding RINEX Obs Header
          *                   using method 'Rinex3ObsHeader::getObsIndex()'.
          *
          * @return
          *  Number of satellites with this kind of data available
          */
      virtual int getData( const Rinex3ObsData& rinexData, int index )
         throw(InvalidRequest);


         /** Pull out the selected observation type from a Rinex3ObsData object
          *
          * @param rinexData  The Rinex data set holding the observations
          * @param type       String representing the observation type.
          * @param hdr        RINEX Observation Header for current RINEX file.
          */
      virtual int getData( const Rinex3ObsData& rinexData,
                           std::string type,
                           const Rinex3ObsHeader& hdr )
         throw(InvalidRequest);


         /// Set this to true if you want to enable data checking within given
         /// boundaries (default for code measurements)
      bool checkData;


         /// Set the minimum pseudorange value allowed for data (in meters).
      virtual ExtractData& setMinPRange( double minPR )
      { minPRange = minPR; return (*this); };


         /// Get the minimum pseudorange value allowed for data (in meters).
      virtual double getMinPRange(void) const
      { return minPRange; };


         /// Set the maximum pseudorange value allowed for data (in meters).
      virtual ExtractData& setMaxPRange( double maxPR )
      { maxPRange = maxPR; return (*this); };


         /// Get the minimum pseudorange value allowed for data (in meters).
      virtual double getMaxPRange(void) const
      { return maxPRange; };


         /// Destructor
      virtual ~ExtractData() {};


   protected:


         /// True only if results are valid
      bool valid;


         /// Minimum pseudorange value allowed for input data (in meters).
      double minPRange;


         /// Maximum pseudorange value allowed for input data (in meters).
      double maxPRange;


   }; // End of class 'ExtractData'
   

      //@}
   
}  // End of namespace gpstk

#endif   // GPSTK_EXTRACTDATA_HPP
