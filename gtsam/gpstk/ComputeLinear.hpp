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
 * @file ComputeLinear.hpp
 * This class computes linear combinations of GDS data.
 */

#ifndef GPSTK_COMPUTELINEAR_HPP
#define GPSTK_COMPUTELINEAR_HPP

#include <gtsam/gpstk/ProcessingClass.hpp>



namespace gpstk
{

      /** @addtogroup DataStructures */
      //@{


      /** This class computes linear combinations of GDS data.
       *
       * This class is meant to be used with the GNSS data structures (GDS)
       * objects found in "DataStructures" class, and it is intended to be
       * used as a more flexible replacement for classes such as ComputePC.hpp.
       *
       * A typical way to use this class follows:
       *
       * @code
       *
       *      // Define a linear combination to compute PC using C1
       *   gnssLinearCombination pcComb;
       *   pcComb.header = TypeID::PC;
       *   pcComb.body[TypeID::C1] = +GAMMA_GPS/0.646944444;
       *   pcComb.body[TypeID::P2] = -1.0/0.646944444;
       *
       *
       *      // Define another linear combination to compute PC prefit
       *      // residual
       *   gnssLinearCombination prefitComb;
       *   prefitComb.header = TypeID::prefitC;
       *   prefitComb.body[TypeID::PC] = +1.0;
       *   prefitComb.body[TypeID::rho] = -1.0;
       *   prefitComb.body[TypeID::dtSat] = +1.0;
       *   prefitComb.body[TypeID::rel] = -1.0;
       *   prefitComb.body[TypeID::tropoSlant] = -1.0;
       *
       *
       *      // Object to compute linear combinations of data
       *      // Linear combinations will be computed in a FIFO basis
       *   ComputeLinear linear;
       *   linear.addLinear(pcComb);
       *   linear.addLinear(prefitComb);
       *
       *      // Load observation data
       *   RinexObsStream rin("ebre0300.02o");
       *
       *      // Loads precise ephemeris object with file data
       *   SP3EphemerisStore SP3EphList;
       *   SP3EphList.loadFile("igs11513.sp3");
       *
       *      // Sets nominal position of receiver
       *   Position nominalPos(4833520.3800, 41536.8300, 4147461.2800);
       *
       *      // Declare a MOPSTropModel object, setting the defaults
       *   MOPSTropModel mopsTM( nominalPos.getAltitude(),
       *                         nominalPos.getGeodeticLatitude(), 30);
       *
       *      // Object to compute the tropospheric data
       *   ComputeTropModel computeTropo(mopsTM);
       *
       *      // Declare a basic modeler
       *   BasicModel basic(nominalPos, SP3EphList);
       *
       *   gnssRinex gRin;
       *
       *   while(rin >> gRin)
       *   {
       *
       *      gRin >> basic >> computeTropo >> linear;
       *
       *         // Dump results
       *      gRin.body.dump(cout,1);
       *   }
       *
       * @endcode
       *
       * The "ComputeLinear" object will visit every satellite in the GNSS
       * data structure that is "gRin" and will compute the previously defined
       * linear combination (or combinations).
       *
       * When used with the ">>" operator, this class returns the same
       * incoming data structure with the results inserted in it. Be warned
       * that if a given satellite does not have the observations or data
       * required by the linear combination definition, such data will be
       * taken as zero.
       *
       * \warning If the "ComputeLinear" object has more than one linear
       * combination definition, they will be applied in the same order they
       * were added to the object, i.e. in a FIFO (First Input - First Output)
       * basis. Therefore, you must be mindful of combination order.
       *
       * @sa ComputeCombination.hpp, ComputePC.hpp, ModelObsFixedStation.hpp
       * and ModelObs.hpp, among others, for related classes.
       */
   class ComputeLinear : public ProcessingClass
   {
   public:

         /// Default constructor
      ComputeLinear()
      { clearAll(); };


         /** Common constructor
          *
          * @param linearComb   Linear combination to be computed.
          */
      ComputeLinear( const gnssLinearCombination& linearComb )
      { linearList.push_back(linearComb); };


         /** Common constructor
          *
          * @param list    List of linear combination definitions to compute.
          */
      ComputeLinear(const LinearCombList& list)
         : linearList(list)
      { };


         /** Returns a satTypeValueMap object, adding the new data generated
          *  when calling this object.
          *
          * @param time      Epoch corresponding to the data.
          * @param gData     Data object holding the data.
          */
      virtual satTypeValueMap& Process( const CommonTime& time,
                                        satTypeValueMap& gData )
         throw(ProcessingException);


         /** Returns a gnnsSatTypeValue object, adding the new data 
          *  generated when calling this object.
          *
          * @param gData    Data object holding the data.
          */
      virtual gnssSatTypeValue& Process(gnssSatTypeValue& gData)
         throw(ProcessingException)
      { Process(gData.header.epoch, gData.body); return gData; };


         /** Returns a gnnsRinex object, adding the new data generated 
          *  when calling this object.
          *
          * @param gData    Data object holding the data.
          */
      virtual gnssRinex& Process(gnssRinex& gData)
         throw(ProcessingException)
      { Process(gData.header.epoch, gData.body); return gData; };


         /// Returns the list of linear combinations to be computed.
      virtual LinearCombList getLinearCombinations(void) const
      { return linearList; };


         /// Clear all linear combinations.
      virtual ComputeLinear& clearAll(void)
      { linearList.clear(); return (*this); };


         /** Sets a linear combinations to be computed.
          *
          * @param linear    Linear combination definition to compute.
          *
          * @warning All previous linear combinations will be deleted.
          */
      virtual ComputeLinear& setLinearCombination(
                                       const gnssLinearCombination& linear )
      { clearAll(); linearList.push_back(linear); return (*this); };


         /** Sets the list of linear combinations to be computed.
          *
          * @param list    List of linear combination definitions to compute.
          *
          * @warning All previous linear combinations will be deleted.
          */
      virtual ComputeLinear& setLinearCombination(const LinearCombList& list)
      { clearAll(); linearList = list; return (*this); };


         /** Add a linear combination to be computed.
          *
          * @param linear    Linear combination definitions to be added.
          */
      virtual ComputeLinear& addLinear(const gnssLinearCombination& linear)
      { linearList.push_back(linear); return (*this); };


         /// Returns a string identifying this object.
      virtual std::string getClassName(void) const;


         /// Destructor
      virtual ~ComputeLinear() {};


   private:


         /// List of linear combinations to compute
      LinearCombList linearList;


   }; // End class ComputeLinear

      //@}

}  // End of namespace gpstk

#endif   // GPSTK_COMPUTELINEAR_HPP
