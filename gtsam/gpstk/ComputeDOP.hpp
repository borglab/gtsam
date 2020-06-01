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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2008, 2011
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
 * @file ComputeDOP.hpp
 * This class computes the usual DOP values: GDOP, PDOP, TDOP, HDOP and VDOP.
 */

#ifndef GPSTK_COMPUTEDOP_HPP
#define GPSTK_COMPUTEDOP_HPP

#include "ProcessingClass.hpp"



namespace gpstk
{

      /** @addtogroup DataStructures */
      //@{


      /** This class computes the usual DOP values: GDOP, PDOP, TDOP, HDOP
       *  and VDOP.
       *
       * This class is meant to be used with the GNSS data structures objects
       * found in "DataStructures" class.
       *
       * A typical way to use this class follows:
       *
       * @code
       *      // Create the input obs file stream
       *   RinexObsStream rin("ebre0300.02o");
       *
       *      // Loads precise ephemeris object with file data
       *   SP3EphemerisStore SP3EphList;
       *   SP3EphList.loadFile("igs11513.sp3");
       *
       *      // Sets nominal position of receiver
       *   Position nominalPos(4833520.3800, 41536.8300, 4147461.2800);
       *
       *      // Object to compute basic model data
       *   BasicModel basicM(nominalPos, SP3EphList);
       *
       *      // Declare a base-changing object: ECEF to North-East-Up (NEU)
       *   XYZ2NEU baseChange(nominalPos);
       *
       *      // Object to compute DOP
       *   ComputeDOP cDOP;
       *
       *   gnssRinex gRin;
       *   while(rin >> gRin)
       *   {
       *      gRin >> basicM >> baseChange >> cDOP;
       *   }
       * @endcode
       *
       * Please note that, in order to work appropriately, class ComputeDOP
       * needs that the GNSS Data Structure contains the values of the full
       * geometry matrix both for XYZ and ENU. This may be achieved if before
       * calling the ComputeDOP objects, we call objects from classes like
       * BasicModel and XYZ2NEU, among others.
       *
       * \warning If the GNSS Data Structure does not contain the necessary
       * geometric coefficients, ComputeDOP will return the corresponding
       * DOP values as -1.0.
       *
       * \sa BasicModel.hpp, ModelObsFixedStation.hpp, ModelObs.hpp,
       * ModeledReferencePR.hpp, ModeledPR.hpp, XYZ2NEU.hpp and XYZ2NED.hpp.
       *
       */
   class ComputeDOP : public ProcessingClass
   {
   public:

         /// Default constructor
      ComputeDOP()
         : gdop(-1.0), pdop(-1.0), tdop(-1.0), hdop(-1.0), vdop(-1.0)
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


         /// Returns GDOP.
      virtual double getGDOP(void) const
      { return gdop; };


         /// Returns PDOP.
      virtual double getPDOP(void) const
      { return pdop; };


         /// Returns TDOP.
      virtual double getTDOP(void) const
      { return tdop; };


         /// Returns HDOP.
      virtual double getHDOP(void) const
      { return hdop; };


         /// Returns VDOP.
      virtual double getVDOP(void) const
      { return vdop; };


         /// Returns a string identifying this object.
      virtual std::string getClassName(void) const;


         /// Destructor
      virtual ~ComputeDOP() {};


   private:


         /// Geometric Dilution of Precision
      double gdop;

         /// Position Dilution of Precision
      double pdop;

         /// Time Dilution of Precision
      double tdop;

         /// Horizontal Dilution of Precision
      double hdop;

         /// Vertical Dilution of Precision
      double vdop;

   }; // End of class 'ComputeDOP'

      //@}

}  // End of namespace gpstk

#endif   // GPSTK_COMPUTEDOP_HPP
