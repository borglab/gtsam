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
 * @file LICSDetector.cpp
 * This is a class to detect cycle slips using LI observables.
 */

#include "LICSDetector.hpp"


namespace gpstk
{

      // Returns a string identifying this object.
   std::string LICSDetector::getClassName() const
   { return "LICSDetector"; }


      /* Common constructor
       *
       * @param mThr    Minimum threshold to declare cycle slip, in meters.
       * @param drift   LI combination limit drift, in meters/second.
       * @param dtMax   Maximum interval of time allowed between two
       *                successive epochs, in seconds.
       */
   LICSDetector::LICSDetector( const double& mThr,
                               const double& drift,
                               const double& dtMax,
                               const bool& use )
      : obsType(TypeID::LI), lliType1(TypeID::LLI1), lliType2(TypeID::LLI2),
        resultType1(TypeID::CSL1), resultType2(TypeID::CSL2), useLLI(use)
   {
      setDeltaTMax(dtMax);
      setMinThreshold(mThr);
      setLIDrift(drift);
   }


      /* Returns a satTypeValueMap object, adding the new data generated
       *  when calling this object.
       *
       * @param epoch     Time of observations.
       * @param gData     Data object holding the data.
       * @param epochflag Epoch flag.
       */
   satTypeValueMap& LICSDetector::Process( const CommonTime& epoch,
                                           satTypeValueMap& gData,
                                           const short& epochflag )
      throw(ProcessingException)
   {

      try
      {

         double value1(0.0);
         double lli1(0.0);
         double lli2(0.0);

         SatIDSet satRejectedSet;

            // Loop through all the satellites
         satTypeValueMap::iterator it;
         for (it = gData.begin(); it != gData.end(); ++it)
         {
            try
            {
                  // Try to extract the values
               value1 = (*it).second(obsType);
            }
            catch(...)
            {
                  // If some value is missing, then schedule this satellite
                  // for removal
               satRejectedSet.insert( (*it).first );
               continue;
            }

            if (useLLI)
            {
               try
               {
                     // Try to get the LLI1 index
                  lli1  = (*it).second(lliType1);
               }
               catch(...)
               {
                     // If LLI #1 is not found, set it to zero
                     // You REALLY want to have BOTH LLI indexes properly set
                  lli1 = 0.0;
               }

               try
               {
                     // Try to get the LLI2 index
                  lli2  = (*it).second(lliType2);
               }
               catch(...)
               {
                     // If LLI #2 is not found, set it to zero
                     // You REALLY want to have BOTH LLI indexes properly set
                  lli2 = 0.0;
               }
            }

               // If everything is OK, then get the new values inside the
               // structure. This way of computing it allows concatenation of
               // several different cycle slip detectors
            (*it).second[resultType1] += getDetection( epoch,
                                                       (*it).first,
                                                       (*it).second,
                                                       epochflag,
                                                       value1,
                                                       lli1,
                                                       lli2 );

            if ( (*it).second[resultType1] > 1.0 )
            {
               (*it).second[resultType1] = 1.0;
            }

               // We will mark both cycle slip flags
            (*it).second[resultType2] = (*it).second[resultType1];

         }

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

   }  // End of method 'LICSDetector::Process()'



      /* Method to set the maximum interval of time allowed between two
       * successive epochs.
       *
       * @param maxDelta      Maximum interval of time, in seconds
       */
   LICSDetector& LICSDetector::setDeltaTMax(const double& maxDelta)
   {
         // Don't allow delta times less than or equal to 0
      if (maxDelta > 0.0)
      {
         deltaTMax = maxDelta;
      }
      else
      {
         deltaTMax = 61.0;
      }

      return (*this);

   }  // End of method 'LICSDetector::setDeltaTMax()'



      /* Method to set the minimum threshold for cycle slip detection, in
       * meters.
       *
       * @param mThr    Minimum threshold for cycle slip detection, in
       *                meters.
       */
   LICSDetector& LICSDetector::setMinThreshold(const double& mThr)
   {
         // Don't allow thresholds less than 0
      if (mThr < 0.0)
      {
         minThreshold = 0.04;
      }
      else
      {
         minThreshold = mThr;
      }

      return (*this);

   }  // End of method 'LICSDetector::setMinThreshold()'



      /* Method to set the LI combination limit drift, in meters/second
       *
       * @param drift     LI combination limit drift, in meters/second.
       */
   LICSDetector& LICSDetector::setLIDrift(const double& drift)
   {
         // Don't allow drift less than or equal to 0
      if (drift > 0.0)
      {
         LIDrift = drift;
      }
      else
      {
         LIDrift = 0.002;
      }

      return (*this);

   }  // End of method 'LICSDetector::setLIDrift()'



      /* Returns a gnnsRinex object, adding the new data generated when
       * calling this object.
       *
       * @param gData    Data object holding the data.
       */
   gnssRinex& LICSDetector::Process(gnssRinex& gData)
      throw(ProcessingException)
   {

      try
      {

         Process(gData.header.epoch, gData.body, gData.header.epochFlag);

         return gData;

      }
      catch(Exception& u)
      {
            // Throw an exception if something unexpected happens
         ProcessingException e( getClassName() + ":"
                                + u.what() );

         GPSTK_THROW(e);

      }

   }  // End of method 'LICSDetector::Process()'


      /* Method that implements the LI cycle slip detection algorithm
       *
       * @param epoch     Time of observations.
       * @param sat       SatID.
       * @param tvMap     Data structure of TypeID and values.
       * @param epochflag Epoch flag.
       * @param li        Current LI observation value.
       * @param lli1      LLI1 index.
       * @param lli2      LLI2 index.
       */
   double LICSDetector::getDetection( const CommonTime& epoch,
                                      const SatID& sat,
                                      typeValueMap& tvMap,
                                      const short& epochflag,
                                      const double& li,
                                      const double& lli1,
                                      const double& lli2 )
   {

      bool reportCS(false);

         // Difference between current and former epochs, in sec
      double currentDeltaT(0.0);

         // Difference between current and former LI values
      double currentBias(0.0);

         // Limit to declare cycle slip
      double deltaLimit(0.0);

      double delta(0.0);
      double tempLLI1(0.0);
      double tempLLI2(0.0);


         // Get the difference between current epoch and former epoch,
         // in seconds
      currentDeltaT = ( epoch - LIData[sat].formerEpoch );

         // Store current epoch as former epoch
      LIData[sat].formerEpoch = epoch;

         // Current value of LI difference
      currentBias = li - LIData[sat].formerLI;

         // Increment window size
      ++LIData[sat].windowSize;

         // Check if receiver already declared cycle slip or too much time
         // has elapsed
         // Note: If tvMap(lliType1) or tvMap(lliType2) don't exist, then 0
         // will be returned and those tests will pass
      if ( (tvMap(lliType1)==1.0) ||
           (tvMap(lliType1)==3.0) ||
           (tvMap(lliType1)==5.0) ||
           (tvMap(lliType1)==7.0) )
      {
         tempLLI1 = 1.0;
      }

      if ( (tvMap(lliType2)==1.0) ||
           (tvMap(lliType2)==3.0) ||
           (tvMap(lliType2)==5.0) ||
           (tvMap(lliType2)==7.0) )
      {
         tempLLI2 = 1.0;
      }

      if ( (epochflag==1)  ||
           (epochflag==6)  ||
           (tempLLI1==1.0) ||
           (tempLLI2==1.0) ||
           (currentDeltaT > deltaTMax) )
      {

            // We reset the filter with this
         LIData[sat].windowSize = 0;

         reportCS = true;
      }

      if (LIData[sat].windowSize > 1)
      {
         deltaLimit = minThreshold + std::abs(LIDrift*currentDeltaT);

            // Compute a linear interpolation and compute
            // LI_predicted - LI_current
         delta = std::abs( currentBias - (LIData[sat].formerBias *
                           currentDeltaT / LIData[sat].formerDeltaT) );

         if (delta > deltaLimit)
         {
               // We reset the filter with this
            LIData[sat].windowSize = 0;

            reportCS = true;
         }

      }

         // Let's prepare for the next time
      LIData[sat].formerLI = li;
      LIData[sat].formerBias = currentBias;
      LIData[sat].formerDeltaT = currentDeltaT;

      if (reportCS)
      {
         return 1.0;
      }
      else
      {
         return 0.0;
      }

   }  // End of method 'LICSDetector::getDetection()'


}  // End of namespace gpstk
