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
 * @file PCSmoother.cpp
 * This class smoothes PC code observables using the corresponding LC
 * phase observable.
 */

#include <gtsam/gpstk/PCSmoother.hpp>


namespace gpstk
{

      // Returns a string identifying this object.
   std::string PCSmoother::getClassName() const
   { return "PCSmoother"; }



      /* Returns a satTypeValueMap object, adding the new data generated
       * when calling this object.
       *
       * @param gData     Data object holding the data.
       */
   satTypeValueMap& PCSmoother::Process(satTypeValueMap& gData)
      throw(ProcessingException)
   {

      try
      {

         double codeObs(0.0);
         double phaseObs(0.0);
         double flagObs1(0.0);
         double flagObs2(0.0);

         SatIDSet satRejectedSet;

            // Loop through all satellites
         satTypeValueMap::iterator it;
         for (it = gData.begin(); it != gData.end(); ++it)
         {

            try
            {

                  // Try to extract the values
               codeObs  = (*it).second(codeType);
               phaseObs = (*it).second(phaseType);

            }
            catch(...)
            {

                  // If some value is missing, then schedule this satellite
                  // for removal
               satRejectedSet.insert( (*it).first );

               continue;

            }

            try
            {

                  // Try to get the first cycle slip flag
               flagObs1  = (*it).second(csFlag1);

            }
            catch(...)
            {

                  // If flag #1 is not found, no cycle slip is assumed
                  // You REALLY want to have BOTH CS flags properly set
               flagObs1 = 0.0;

            }

            try
            {

                  // Try to get the second cycle slip flag
               flagObs2  = (*it).second(csFlag2);

            }
            catch(...)
            {

                  // If flag #2 is not found, no cycle slip is assumed
                  // You REALLY want to have BOTH CS flags properly set
               flagObs2 = 0.0;

            }

               // Get the smoothed PC.
            (*it).second[resultType] = getSmoothing( (*it).first,
                                                     codeObs,
                                                     phaseObs,
                                                     flagObs1,
                                                     flagObs2 );

         }  // End of 'for (it = gData.begin(); it != gData.end(); ++it)'

            // Remove satellites with missing data
         gData.removeSatID(satRejectedSet);

         return gData;

      }
      catch(Exception& u)
      {
            // Throw an exception if something unexpected happens
         ProcessingException e( getClassName() + ":"
                                + u.what() );

         GPSTK_THROW(e);

      }

   }  // End of method 'PCSmoother::Process()'



      /* Method to set the maximum size of filter window, in samples.
       *
       * @param maxSize       Maximum size of filter window, in samples.
       */
   PCSmoother& PCSmoother::setMaxWindowSize(const int& maxSize)
   {

         // Don't allow window sizes less than 1
      if (maxSize > 1)
      {
         maxWindowSize = maxSize;
      }
      else
      {
         maxWindowSize = 1;
      }

      return (*this);

   }  // End of method 'PCSmoother::setMaxWindowSize()'



      /* Compute the smoothed code observable.
       *
       * @param sat        Satellite object.
       * @param code       Code measurement.
       * @param phase      Phase measurement.
       * @param flag1      Cycle slip flag in L1.
       * @param flag2      Cycle slip flag in L2.
       */
   double PCSmoother::getSmoothing( const SatID& sat,
                                    const double& code,
                                    const double& phase,
                                    const double& flag1,
                                    const double& flag2 )
   {

         // In case we have a cycle slip either in L1 or L2
      if ( (flag1!=0.0) || (flag2!=0.0) )
      {
            // Prepare the structure for the next iteration
         SmoothingData[sat].previousCode = code;
         SmoothingData[sat].previousPhase = phase;
         SmoothingData[sat].windowSize = 1;

            // We don't need any further processing
         return code;

      }

         // In case we didn't have cycle slip
      double smoothedCode(0.0);

         // Increment size of window and check limit
      ++SmoothingData[sat].windowSize;
      if (SmoothingData[sat].windowSize > maxWindowSize)
      {
         SmoothingData[sat].windowSize = maxWindowSize;
      }

         // The formula used is the following:
         //
         // CSn = (1/n)*Cn + ((n-1)/n)*(CSn-1 + Ln - Ln-1)
         //
         // As window size "n" increases, the former formula gives more
         // weight to the previous smoothed code CSn-1 plus the phase bias
         // (Ln - Ln-1), and less weight to the current code observation Cn
      smoothedCode = ( code +
               ((static_cast<double>(SmoothingData[sat].windowSize)) - 1.0) *
               (SmoothingData[sat].previousCode +
               (phase - SmoothingData[sat].previousPhase) ) ) /
               (static_cast<double>(SmoothingData[sat].windowSize));

         // Store results for next iteration
      SmoothingData[sat].previousCode = smoothedCode;
      SmoothingData[sat].previousPhase = phase;

      return smoothedCode;

   }  // End of method 'PCSmoother::getSmoothing()'


}  // End of namespace gpstk
