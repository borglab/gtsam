#pragma ident "$Id$"



#ifndef GPSTK_IONOMODEL_HPP
#define GPSTK_IONOMODEL_HPP

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
//  Copyright 2004, The University of Texas at Austin
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
 * @file IonoModel.cpp
 * Implementation of the ICD-GPS-200 Ionosphere model.
 */

#include "CommonTime.hpp"
#include "EngAlmanac.hpp"
#include "Position.hpp"

namespace gpstk
{
      /** @addtogroup GPSsolutions */
      //@{

      /**
       * Model of the ionosphere.
       * It is used to compute the delay of the satellite signal
       * as seen at the receiver caused by the ionosphere for a
       * "one frequency" user.
       *
       * See ICD-GPS-200, section 20.3.3.5.2.5 and  Figure 20-4.
       *
       * Verbatim from the above section:
       * "It is estimated that the use of this model will provide at least
       * a 50% reduction in the single-frequency user's RMS error due to
       * ionospheric propagation effects. During extended operations, or for
       * the Block IIR SVs in the Autonav mode if the CS is unable to upload
       * the SVs, the use of this model will yield unpredictable results."
       */
   class IonoModel
   {
   public:
        
         /// Thrown when attempting to use a model for which all necessary
         /// parameters have not been specified.
         /// @ingroup exceptiongroup
      NEW_EXCEPTION_CLASS(InvalidIonoModel, gpstk::Exception);
 

      enum Frequency
      {
         L1,  ///< L1 frequency (1575.42 MHz)
         L2   ///< L2 frequency (1227.60 MHz)
      };
      
         /// default constructor, creates an invalid model
      IonoModel() throw() : valid(false) {}
      
         /// destructor
      virtual ~IonoModel() throw() {}
      
         /**
          * constructor.
          * Creates a valid model with satellite transmitted alpha
          * and beta parameters provided from almanac.
          * \param a an array containing the four alpha terms
          * \param b an array containing the four beta terms
          */
      IonoModel(const double a[4], const double b[4]) throw();
      
         /**
          * EngAlmanac constructor.
          * Creates a valid model from and EngAlmanac object
          * \param engalm an EngAlmanac object
          */
      IonoModel(const EngAlmanac& engalm) throw();
      
         /** Method to feed the model with satellite transmitted alpha
          * and beta parameters provided from almanac.
          * \param a an array containing the four alpha terms
          * \param b an array containing the four beta terms
          */
      void setModel(const double a[4], const double b[4]) throw();
      
         /**
          * returns the validity of the model.
          * \return model validity
          */
      bool isValid() const throw() { return valid; }
      
         /**
          * get the ionospheric correction value.
          * \param time the time of the observation
          * \param rxgeo the WGS84 geodetic position of the receiver
          * \param svel the elevation angle between the rx and SV (degrees)
          * \param svaz the azimuth angle between the rx and SV (degrees)
          * \param freq the GPS frequency the observation was made from
          * \return the ionospheric correction (meters)
          */
      double getCorrection(const CommonTime& time,
                           const Position& rxgeo,
                           double svel,
                           double svaz,
                           Frequency freq = L1) const
         throw(InvalidIonoModel);

         /// equality operator
      bool operator==(const IonoModel& right) const throw();

         /// inequality operator
      bool operator!=(const IonoModel& right) const throw();     

 private:

      double alpha[4];
      double beta[4];

      bool valid;
   };
      //@}
}

#endif
