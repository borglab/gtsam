#pragma ident "$Id$"

/**
 * @file SimpleIURAWeight.hpp
 * Class to assign weights to satellites based on their URA Index (IURA).
 */

#ifndef GPSTK_SIMPLEIURAWEIGHT_HPP
#define GPSTK_SIMPLEIURAWEIGHT_HPP

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
//  Dagoberto Salazar - gAGE. 2006, 2011
//
//============================================================================



#include "WeightBase.hpp"
#include "GPSEphemerisStore.hpp"
#include "TabularSatStore.hpp"
#include "EngEphemeris.hpp"
#include "RinexObsHeader.hpp"
#include "GNSSconstants.hpp"
#include "GPS_URA.hpp"
#include "Xvt.hpp"
#include "XvtStore.hpp"
#include "Vector.hpp"
#include <vector>

#include "Position.hpp"
#include "TropModel.hpp"
#include "geometry.hpp"                   // DEG_TO_RAD
#include <cmath>



namespace gpstk
{

      /** @addtogroup GPSsolutions */
      //@{

      /**
       * Class to assign weights to satellites based on their URA Index (IURA).
       */
   class SimpleIURAWeight : public WeightBase
   {
   public:

         /// Empty constructor
      SimpleIURAWeight(void) { valid = false; };


         /** Compute and return a vector with the weights for the given SVs.
          *
          * @param time           Epoch weights will be computed for
          * @param Satellites     Vector of satellites
          * @param bcEph          Satellite broadcast ephemeris
          * 
          * @return
          *  Number of satellites with valid weights
          *
          * \note
          * Method isValid() will return false if some satellite does not have
          * a valid weight. Also, its PRN will be set to a negative value.
          *
          */
      virtual int getWeights( CommonTime& time,
                              Vector<SatID>& Satellites,
                              GPSEphemerisStore& bcEph )
         throw(InvalidWeights);


         /** Compute and return a vector with the weights for the given SVs.
          *
          * @param time           Epoch weights will be computed for
          * @param Satellites     Vector of satellites
          * @param preciseEph     Satellite precise ephemeris
          * 
          * @return
          *  Number of satellites with valid weights
          *
          * \note
          * Method isValid() will return false if some satellite does not have a
          * valid weight. Also, its PRN will be set to a negative value.
          *
          * \note
          * This method assigns an URA of 0.1 m to all satellites.
          *
          */
      virtual int getWeights( CommonTime& time,
                              Vector<SatID>& Satellites,
                              TabularSatStore<Xvt>& preciseEph )
         throw(InvalidWeights);

         /// Vector of weights for these satellites
      Vector<double> weightsVector;

         /// Vector with the PRN of satellites with weights available for computing.
      Vector<SatID> availableSV;

         /// Vector with the PRN of satellites rejected or with no proper weights.
      Vector<SatID> rejectedSV;

         /// Return validity of weights
      virtual bool isValid(void)
      { return valid; }


   }; // End of class 'SimpleIURAWeight'


      //@}


}  // End of namespace gpstk

#endif   // GPSTK_SIMPLEIURAWEIGHT_HPP
