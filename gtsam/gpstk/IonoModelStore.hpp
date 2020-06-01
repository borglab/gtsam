#pragma ident "$Id$"

/**
 * @file IonoModelStore.hpp
 * Store GPS Navigation Message based ionospheric models
 */


#ifndef GPSTK_IONOMODELSTORE_HPP
#define GPSTK_IONOMODELSTORE_HPP

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


#include <map>
#include "CommonTime.hpp"
#include "IonoModel.hpp"

namespace gpstk
{
      /** @addtogroup GPSsolutions */
      //@{

      /**
       * This class defines an interface to hide how we determine
       * the ionospheric delay as determined from GPS navigation message
       * based models at some point in time
       */
   class IonoModelStore
   {
   public:


         /**
          * Thrown when attempting to get a model that isn't stored.
          * @ingroup exceptiongroup
          */
      NEW_EXCEPTION_CLASS(NoIonoModelFound, gpstk::Exception);


         /// constructor
      IonoModelStore() throw() {}


         /// destructor
      virtual ~IonoModelStore() throw() {}


         /** Get the ionospheric correction value.
          *
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
                           IonoModel::Frequency freq = IonoModel::L1) const
         throw(NoIonoModelFound);


         /** Add an IonoModel to this collection
          *
          * \param mt the time the model is valid from
          * \param im the IonoModel to add
          * \return true if the model was added, false otherwise
          */
      bool addIonoModel( const CommonTime& mt,
                         const IonoModel& im )
         throw();


   private:


      typedef std::map<CommonTime, IonoModel> IonoModelMap;

      IonoModelMap ims;


   }; // End of class 'IonoModelStore'
   
      //@}

}  // End of namespace gpstk

#endif   // GPSTK_IONOMODELSTORE_HPP
