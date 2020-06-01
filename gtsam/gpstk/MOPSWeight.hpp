#pragma ident "$Id$"

/**
 * @file MOPSWeight.hpp
 * Class to assign weights to satellites based on the Appendix J of MOPS
 * document RTCA DO-229D.
 */

#ifndef GPSTK_MOPSWEIGHT_HPP
#define GPSTK_MOPSWEIGHT_HPP

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
//  Dagoberto Salazar - gAGE. 2006, 2008, 2010, 2011
//
//============================================================================



#include "WeightBase.hpp"
#include "XvtStore.hpp"
#include "GPSEphemerisStore.hpp"
#include "TabularSatStore.hpp"
#include "EngEphemeris.hpp"
#include "RinexObsHeader.hpp"
#include "Position.hpp"
#include "SimpleIURAWeight.hpp"
#include "TropModel.hpp"
#include "GNSSconstants.hpp"
#include "geometry.hpp"                   // DEG_TO_RAD
#include <cmath>
#include <vector>


namespace gpstk
{

      /** @addtogroup GPSsolutions */
      //@{

      /** Class to assign weights to satellites based on the Appendix J
       *  of MOPS document RTCA DO-229D.
       *
       * This class implements an algorithm to assign weights to satellites
       * based on the RTCA "Minimum Operational Performance Standards" (MOPS),
       * version C (RTCA/DO-229D), sections J.2.3. "Variance of Ionospheric
       * Delay", J.2.4. "Variance of Airborne Receiver Errors" and J.2.5.
       * "Variance of Tropospheric errors".
       *
       * It is meant to be used with class "MOPSTropModel".
       *
       */
   class MOPSWeight: WeightBase
   {
   public:


         /// Empty constructor
      MOPSWeight(void)
      { valid = false; };


         /** Computes a vector with the weights for the given satellites.
          *
          * @param time               Epoch weights will be computed for.
          * @param Satellites         Vector of satellites.
          * @param bcEph              Satellite broadcast ephemeris.
          * @param ionoCorrections    Ionospheric corrections computed using
          *                           Klobuchar model.
          * @param elevationVector    Vector of elevations, in degrees.
          * @param azimuthVector      Vector of azimuths, in degrees.
          * @param rxPosition         Position of the receiver.
          * @param rxClass            Integer indicating receiver class
          *                           according MOPS-C. It is 2 by default
          *                           (conservative setting).
          *
          * @return Number of satellites with valid weights.
          *
          * \note
          * Method isValid() will return 'false' if some satellite does not have
          * a valid weight. Also, its PRN will be set to a negative value.
          *
          */
      virtual int getWeights( CommonTime& time,
                              Vector<SatID>& Satellites,
                              GPSEphemerisStore& bcEph,
                              Vector<double>& ionoCorrections,
                              Vector<double>& elevationVector,
                              Vector<double>& azimuthVector,
                              Position rxPosition,
                              int rxClass = 2 )
         throw(InvalidWeights);


         /** Computes a vector with the weights for the given satellites.
          *
          * @param time               Epoch weights will be computed for.
          * @param Satellites         Vector of satellites.
          * @param preciseEph         Satellite precise ephemeris.
          * @param ionoCorrections    Ionospheric corrections computed using
          *                           Klobuchar model.
          * @param elevationVector    Vector of elevations, in degrees.
          * @param azimuthVector      Vector of azimuths, in degrees.
          * @param rxPosition         Position of the receiver.
          * @param rxClass            Integer indicating receiver class.
          *                           according MOPS-C. It is 2 by default
          *                           (conservative setting).
          *
          * @return Number of satellites with valid weights.
          *
          * \note
          * Method isValid() will return 'false' if some satellite does not have
          * a valid weight. Also, its PRN will be set to a negative value.
          *
          */
      virtual int getWeights( CommonTime& time,
                              Vector<SatID>& Satellites,
                              TabularSatStore<Xvt>& preciseEph,
                              Vector<double>& ionoCorrections,
                              Vector<double>& elevationVector,
                              Vector<double>& azimuthVector,
                              Position rxPosition,
                              int rxClass = 2 )
         throw(InvalidWeights);


         /// Vector of weights for these satellites
      Vector<double> weightsVector;


         /// Vector with the PRN of satellites with weights available
         /// for computing.
      Vector<SatID> availableSV;


         /// Vector with the PRN of satellites rejected or with
         /// no proper weights.
      Vector<SatID> rejectedSV;


         /// Return validity of weights
      virtual bool isValid(void)
      { return valid; }


   private:


         /// Compute satellites' weights.
      void Compute( int goodSV,
                    SimpleIURAWeight& sIura,
                    Vector<SatID>& Satellites,
                    Vector<double>& ionoCorrections,
                    Vector<double>& elevationVector,
                    Vector<double>& azimuthVector,
                    Position rxPosition,
                    int rxClass )
         throw(InvalidWeights);


         /// Compute ionospheric sigma^2 according to Appendix J.2.3 and
         /// Appendix A.4.4.10.4 in MOPS-C.
      double sigma2iono( double& ionoCorrection,
                         double& elevation,
                         double& azimuth,
                         Position rxPosition )
         throw(InvalidWeights);


   }; // End of class 'MOPSWeight'


      //@}


}  // End of namespace gpstk

#endif   // GPSTK_MOPSWEIGHT_HPP
