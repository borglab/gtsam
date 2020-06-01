#pragma ident "$Id$"

/**
 * @file ModeledPseudorangeBase.hpp
 * Abstract base class for modeled pseudoranges
 */

#ifndef GPSTK_MODELEDPSEUDORANGEBASE_HPP
#define GPSTK_MODELEDPSEUDORANGEBASE_HPP

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
//  Dagoberto Salazar - gAGE. 2006, 2008, 2011
//
//============================================================================



#include "Xvt.hpp"
#include "Position.hpp"
#include "Matrix.hpp"
#include "Vector.hpp"
#include "RinexObsHeader.hpp"
#include <vector>


namespace gpstk
{
      /** @addtogroup GPSsolutions */
      //@{

      /**
       * This abstract class deals with modeled pseudoranges.
       */
   class ModeledPseudorangeBase
   {
   public:

         /// Implicit constructor
      ModeledPseudorangeBase() : minElev(10.0) {};

         /// Either estimated or "a priori" position of receiver
      Position rxPos;

         /// Estimated geometric ranges from satellites to receiver
      Vector<double> geometricRho;

         /// Satellite clock biases, in meters
      Vector<double> svClockBiases;

         /// Satellite ECEF positions (m) and velocities (m/s) at
         /// transmission time
      Vector<Xvt> svXvt;

         /// Epoch when the signal left the satellite (Transmission time)
      Vector<CommonTime> svTxTime;

         /// Total Group Delay (TGD) of satellites, in meters
      Vector<double> svTGD;

         /// Relativity corrections of satellites, in meters
      Vector<double> svRelativity;

         /// Ionospheric corrections in each receiver-satellite ray, in meters
      Vector<double> ionoCorrections;

         /// Tropospheric corrections in each receiver-satellite ray, in meters
      Vector<double> tropoCorrections;

         /// Observed (measured) pseudoranges from satellites to receiver
      Vector<double> observedPseudoranges;

         /// Modeled pseudoranges from satellites to receiver
      Vector<double> modeledPseudoranges;

         /// Prefit-residuals: Difference between Pseudoranges and
         /// ModeledPseudoranges
      Vector<double> prefitResiduals;

         /// Matrix of Geometry (director cosines from receiver to satellites)
      Matrix<double> geoMatrix;

         /// The elevation cut-off angle for accepted satellites. By default
         /// it is set to 10 degrees
      double minElev;

         /// Any other biases (in meters) that the user wants to include. It
         /// will be substracted from modeled pseudoranges.
      Vector<double> extraBiases;

         /// Boolean telling if there are available computed data for at
         /// least 4 satellites
      bool validData;

         /// Vector with the PRN of satellites available for computing.
      Vector<SatID> availableSV;

         /// Vector with the PRN of satellites rejected or not used
         /// in computing.
      Vector<SatID> rejectedSV;

         /// Vector with the geodetic elevation of satellites from the
         /// receiver point of view.
      Vector<double> elevationSV;

         /// Vector with the geodetic azimuth of satellites from the
         /// receiver point of view.
      Vector<double> azimuthSV;

         /// Destructor.
      virtual ~ModeledPseudorangeBase() {};


   protected:

         /// Method to set the initial (a priori) position of receiver.
      virtual int setInitialRxPosition() = 0;


   }; // End of class 'ModeledPseudorangeBase'

      //@}

}  // End of namespace gpstk

#endif   // GPSTK_MODELEDPSEUDORANGEBASE_HPP
