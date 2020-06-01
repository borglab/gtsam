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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2007, 2008, 2011 
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
 * @file ComputeTropModel.hpp
 * This is a class to compute the main values related to a given
 * GNSS tropospheric model.
 */

#ifndef GPSTK_COMPUTETROPMODEL_HPP
#define GPSTK_COMPUTETROPMODEL_HPP

#include <gtsam/gpstk/ProcessingClass.hpp>
#include <gtsam/gpstk/TropModel.hpp>


namespace gpstk
{

      /** @addtogroup GPSsolutions */
      //@{

      /** This is a class to compute the main values related to a given
       *  GNSS tropospheric model.
       *
       * This class is intended to be used with GNSS Data Structures (GDS).
       * It is a more modular alternative to classes such as ModelObs
       * and ModelObsFixedStation.
       *
       * A typical way to use this class follows:
       *
       * @code
       *      // Input observation file stream
       *   RinexObsStream rin("ebre0300.02o");
       *
       *      // Define the tropospheric model to be used
       *   NeillTropModel neillTM;
       *   neillTM.setReceiverLatitude(lat);
       *   neillTM.setReceiverHeight(height);
       *   neillTM.setDayOfYear(doy);
       *
       *      // Now, create the ComputeTropModel object
       *   ComputeTropModel computeTropo(neillTM);
       *
       *   gnssRinex gRin;  // GNSS data structure for fixed station data
       *
       *   while(rin >> gRin)
       *   {
       *         // Apply the tropospheric model on the GDS
       *      gRin >> computeTropo;
       *   }
       *
       * @endcode
       *
       * The "ComputeTropModel" object will visit every satellite in
       * the GNSS data structure that is "gRin" and will try to compute
       * the main values of the corresponding tropospheric model: Total
       * tropospheric slant correction, dry vertical delay, wet vertical delay,
       * dry mapping function value and wet mapping function value.
       *
       * When used with the ">>" operator, this class returns the same
       * incoming data structure with the extra data inserted along their
       * corresponding satellites.
       *
       * Be warned that if a given satellite does not have the information
       * needed (mainly elevation), it will be summarily deleted from the data
       * structure. This also implies that if you try to use a
       * "ComputeTropModel" object without first defining the tropospheric
       * model, then ALL satellites will be deleted.
       *
       * @sa TropModel.hpp
       *
       */
   class ComputeTropModel : public ProcessingClass
   {
   public:

         /// Default constructor.
      ComputeTropModel()
         : pTropModel(NULL)
      { };


         /** Explicit constructor.
          *
          * @param RxCoordinates Reference station coordinates.
          * @param dEphemeris    EphemerisStore object to be used by default.
          * @param dObservable   Observable type to be used by default.
          * @param applyTGD      Whether or not C1 observable will be
          *                      corrected from TGD effect.
          *
          */
      ComputeTropModel(TropModel& tropoModel)
      { pTropModel = &tropoModel; };


         /** Returns a satTypeValueMap object, adding the new data generated
          *  when calling a modeling object.
          *
          * @param time      Epoch.
          * @param gData     Data object holding the data.
          */
      virtual satTypeValueMap& Process( const CommonTime& time,
                                        satTypeValueMap& gData )
         throw(ProcessingException);


         /** Returns a gnnsSatTypeValue object, adding the new data generated
          *  when calling a modeling object.
          *
          * @param gData    Data object holding the data.
          */
      virtual gnssSatTypeValue& Process(gnssSatTypeValue& gData)
         throw(ProcessingException)
      { Process(gData.header.epoch, gData.body); return gData; };


         /** Returns a gnnsRinex object, adding the new data generated when
          *  calling a modeling object.
          *
          * @param gData    Data object holding the data.
          */
      virtual gnssRinex& Process(gnssRinex& gData)
         throw(ProcessingException)
      { Process(gData.header.epoch, gData.body); return gData; };


         /// Method to get a pointer to the default TropModel to be used
         /// with GNSS data structures.
      virtual TropModel *getTropModel() const
      { return pTropModel; };


         /** Method to set the default TropModel to be used with GNSS
          *  data structures.
          *
          * @param tropoModel   TropModel object to be used by default
          */
      virtual ComputeTropModel& setTropModel(TropModel& tropoModel)
      { pTropModel = &tropoModel; return (*this); };


         /// Returns a string identifying this object.
      virtual std::string getClassName(void) const;


         /// Destructor.
      virtual ~ComputeTropModel() {};


   private:


         /// Pointer to default TropModel object when working with GNSS
         /// data structures.
      TropModel *pTropModel;


   }; // End of class 'ComputeTropModel'

      //@}

}  // End of namespace gpstk

#endif   // GPSTK_COMPUTETROPMODEL_HPP
