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
 * @file Decimate.cpp
 * This class decimates GNSS Data Structures data given a sampling interval,
 * a tolerance, and a starting epoch.
 */

#include <gtsam/gpstk/Decimate.hpp>


namespace gpstk
{

      // Returns a string identifying this object.
   std::string Decimate::getClassName() const
   { return "Decimate"; }



      /* Sets sampling interval.
       *
       * @param sampleInterval      Sampling interval, in seconds.
       */
   Decimate& Decimate::setSampleInterval(const double sampleInterval)
   {

         // Make sure that sample interval is positive
      if( sampleInterval >= 0.0 )
      {
         sampling = sampleInterval;
      }

      return (*this);

   }  // End of method 'Decimate::setSampleInterval()'



      /* Sets tolerance, in seconds.
       *
       * @param tol                 Tolerance, in seconds.
       */
   Decimate& Decimate::setTolerance(const double tol)
   {

         // Make sure that tolerance is positive
      if( tol >= 0.0 )
      {
         tolerance = tol;
      }

      return (*this);

   }  // End of method 'Decimate::setTolerance()'



      /* Returns a satTypeValueMap object, adding the new data generated when
       * calling this object.
       *
       * @param time      Epoch corresponding to the data.
       * @param gData     Data object holding the data.
       */
   satTypeValueMap& Decimate::Process( const CommonTime& time,
                                       satTypeValueMap& gData )
      throw(DecimateEpoch)
   {

         // Set a threshold
      double threshold( std::abs(sampling - tolerance) );

         // Check if current epoch - lastEpoch is NOT within threshold,
         // implying that it must be decimated
      if ( !(std::abs(time - lastEpoch) > threshold) )
      {

            // If epoch must be decimated, we issue an Exception
         DecimateEpoch e("This epoch must be decimated.");

         GPSTK_THROW(e);

      }

         // Update reference epoch
      lastEpoch = time;

      return gData;

   }  // End of method 'Decimate::Process()'


}  // End of namespace gpstk
