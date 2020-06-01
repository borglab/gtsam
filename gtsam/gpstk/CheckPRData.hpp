#pragma ident "$Id$"

/**
 * @file CheckPRData.hpp
 * This class checks whether pseudorange data is between reasonable values.
 */

#ifndef GPSTK_CHECKPRDATA_HPP
#define GPSTK_CHECKPRDATA_HPP

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



namespace gpstk
{

      /** @addtogroup RinexObs */
      //@{


      /// This class checks that pseudorange data is between reasonable values.
   class CheckPRData
   {
   public:

         /// Default constructor
      CheckPRData()
         : minPRange(15000000.0), maxPRange(30000000.0)
      {};


         /// Constructor that allows to set the data span values
      CheckPRData( const double& min, const double& max )
         : minPRange(min), maxPRange(max)
      {};


         /** Checks that the given pseudorange data is between the limits.
          *
          * @param prange    The pseudorange data to be tested
          *
          * @return
          *    True if check was OK.
          */
      virtual bool check( double prange ) const
      { return ( (prange >= minPRange) && (prange <= maxPRange) ); };


         /// Set the minimum pseudorange value allowed for data (in meters).
      virtual CheckPRData& setMinPRange( double minPR )
      { minPRange = minPR; return (*this); };


         /// Get the minimum pseudorange value allowed for data (in meters).
      virtual double getMinPRange(void) const
      { return minPRange; };


         /// Set the maximum pseudorange value allowed for data (in meters).
      virtual CheckPRData& setMaxPRange( double maxPR )
      { maxPRange = maxPR; return (*this); };


         /// Get the maximum pseudorange value allowed for data (in meters).
      virtual double getMaxPRange(void) const
      { return maxPRange; };


         /// Destructor
      virtual ~CheckPRData() {};


   protected:


         /// Minimum pseudorange value allowed for input data (in meters).
      double minPRange;

         /// Maximum pseudorange value allowed for input data (in meters).
      double maxPRange;


   }; // End of class 'CheckPRData'


      //@}
   
}  // End of namespace gpstk

#endif   // GPSTK_CHECKPRDATA_HPP
