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
 * @file LICSDetector2.cpp
 * This is a class to detect cycle slips using LI observables and a
 * 2nd order fitting curve.
 */

#include "LICSDetector2.hpp"


namespace gpstk
{

      // Returns a string identifying this object.
   std::string LICSDetector2::getClassName() const
   { return "LICSDetector2"; }


      // Minimum buffer size . It is always set to 5
   const int LICSDetector2::minBufferSize = 5;


      /* Common constructor
       *
       * @param satThr  Saturation threshold to declare cycle slip, in meters.
       * @param tc      Threshold time constant, in seconds.
       * @param dtMax   Maximum interval of time allowed between two
       *                successive epochs, in seconds.
       */
   LICSDetector2::LICSDetector2( const double& satThr,
                                 const double& tc,
                                 const double& dtMax,
                                 const bool& use )
      : obsType(TypeID::LI), lliType1(TypeID::LLI1), lliType2(TypeID::LLI2),
        resultType1(TypeID::CSL1), resultType2(TypeID::CSL2), useLLI(use)
   {
      setDeltaTMax(dtMax);
      setSatThreshold(satThr);
      setTimeConst(tc);
   }



      /* Returns a satTypeValueMap object, adding the new data generated
       *  when calling this object.
       *
       * @param epoch     Time of observations.
       * @param gData     Data object holding the data.
       * @param epochflag Epoch flag.
       */
   satTypeValueMap& LICSDetector2::Process( const CommonTime& epoch,
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

   }  // End of method 'LICSDetector2::Process()'



      /* Method to set the maximum interval of time allowed between two
       *  successive epochs.
       *
       * @param maxDelta      Maximum interval of time, in seconds
       */
   LICSDetector2& LICSDetector2::setDeltaTMax(const double& maxDelta)
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

   }  // End of method 'LICSDetector2::setDeltaTMax()'



      /* Method to set the saturation threshold for cycle slip detection, in
       * meters.
       *
       * @param satThr  Saturation threshold for cycle slip detection, in
       *                meters.
       */
   LICSDetector2& LICSDetector2::setSatThreshold(const double& satThr)
   {
         // Don't allow saturation thresholds less than or equal to 0
      if (satThr > 0.0)
      {
         satThreshold = satThr;
      }
      else
      {
         satThreshold = 0.08;
      }

      return (*this);

   }  // End of method 'LICSDetector2::setSatThreshold()'



      /* Method to set threshold time constant, in seconds
       *
       * @param tc      Threshold time constant, in seconds.
       *
       * \warning Be sure you have a very good reason to change this value.
       */
   LICSDetector2& LICSDetector2::setTimeConst(const double& tc)
   {
         // Don't allow a time constant less than or equal to 0
      if (tc > 0.0)
      {
         timeConst = tc;
      }
      else
      {
         timeConst = 60.0;
      }

      return (*this);

   }  // End of method 'LICSDetector2::setTimeConst()'



      /* Method to set the maximum buffer size for data, in samples.
       *
       * @param maxBufSize      Maximum buffer size for data, in samples.
       *
       * \warning You must not set a value under minBufferSize, which
       * usually is 5.
       */
   LICSDetector2& LICSDetector2::setMaxBufferSize(const int& maxBufSize)
   {
         // Don't allow buffer sizes less than minBufferSize
      if (maxBufSize >= minBufferSize)
      {
         maxBufferSize = maxBufSize;
      }
      else
      {
         maxBufferSize = minBufferSize;
      }

      return (*this);

   }  // End of method 'LICSDetector2::setMaxBufferSize()'



      /* Returns a gnnsRinex object, adding the new data generated when
       * calling this object.
       *
       * @param gData    Data object holding the data.
       */
   gnssRinex& LICSDetector2::Process(gnssRinex& gData)
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

   }  // End of method 'LICSDetector2::Process()'


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
   double LICSDetector2::getDetection( const CommonTime& epoch,
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

      double tempLLI1(0.0);
      double tempLLI2(0.0);


         // Get current buffer size
      size_t s( LIData[sat].LIEpoch.size() );

         // Get the difference between current epoch and LAST epoch,
         // in seconds, but first test if we have epoch data inside LIData
      if(s > 0)
      {
         currentDeltaT = ( epoch - LIData[sat].LIEpoch.back() );
      }
      else
      {
            // This will yield a very big value
         currentDeltaT = ( epoch - CommonTime::BEGINNING_OF_TIME );
      }


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

            // We reset buffer with the following lines
         LIData[sat].LIEpoch.clear();
         LIData[sat].LIBuffer.clear();

            // current buffer size should be updated
         s = LIData[sat].LIEpoch.size();

            // Report cycle slip
         reportCS = true;
      }

         // Check if we have enough data to start processing.
      if (s >= (size_t)minBufferSize)
      {
            // Declare a Vector for LI measurements
         Vector<double> y(s, 0.0);

            // Declare a Matrix for epoch information
         Matrix<double> M(s, 3, 0.0);

            // We store here the OLDEST (or FIRST) epoch in buffer for future
            // reference. This is important because adjustment will be made
            // with respect to that first epoch
         CommonTime firstEpoch(LIData[sat].LIEpoch.front());

            // Feed 'y' with data
         for(size_t i=0; i<s; i++)
         {
               // The newest goes first in 'y' vector
            y(i) = LIData[sat].LIBuffer[s-1-i];
         }

            // Feed 'M' with data
         for(size_t i=0; i<s; i++)
         {
               // Compute epoch difference with respect to FIRST epoch
            double dT( LIData[sat].LIEpoch[s-1-i] - firstEpoch );

            M(i,0) = 1.0;
            M(i,1) = dT;
            M(i,2) = dT*dT;
         }

            // Now, proceed to find a 2nd order fiting curve using a least
            // mean squares (LMS) adjustment
         Matrix<double> MT(transpose(M));
         Matrix<double> covMatrix( MT * M );

         // Let's try to invert MT*M   matrix
         try
         {
            covMatrix = inverseChol( covMatrix );
         }
         catch(...)
         {
               // If covMatrix can't be inverted we have a serious problem
               // with data, so reset buffer and declare cycle slip
            LIData[sat].LIEpoch.clear();
            LIData[sat].LIBuffer.clear();

            reportCS = true;
         }


            // Now, compute the Vector holding the results of adjustment to
            // second order curve
         Vector<double> a(covMatrix * MT * y);

            // The next step is to compute the maximum deviation from
            // adjustment, in order to assess if our adjustment is too noisy
         double maxDeltaLI(0.0);

         for(size_t i=0; i<s; i++)
         {
               // Compute epoch difference with respect to FIRST epoch
            double dT( LIData[sat].LIEpoch[s-1-i] - firstEpoch );

               // Compute adjusted LI value
            double LIa( a(0) + a(1)*dT + a(2)*dT*dT );

               // Find maximum deviation in current data buffer
            double deltaLI( std::abs(LIa - LIData[sat].LIBuffer[s-1-i]) );
            if( deltaLI > maxDeltaLI )
            {
                maxDeltaLI = deltaLI;
            }
         }

            // Compute epoch difference with respect to FIRST epoch
         double deltaT( epoch - firstEpoch );

            // Compute current adjusted LI value
         double currentLIa( a(0) + a(1)*deltaT + a(2)*deltaT*deltaT );

            // Difference between current and adjusted LI values
         double currentBias( std::abs( currentLIa - li ) );

            // We will continue processing only if we trust our current
            // adjustment, i.e: it is NOT too noisy
         if( (2.0*maxDeltaLI) < currentBias )
         {
               // Compute limit to declare cycle slip
            double deltaLimit( satThreshold /
                               ( 1.0 + ( 1.0 /
                                         std::exp(currentDeltaT/timeConst) )));

               // Check if current LI deviation is above deltaLimit threshold
            if( currentBias > deltaLimit )
            {
                  // Reset buffer and declare cycle slip
               LIData[sat].LIEpoch.clear();
               LIData[sat].LIBuffer.clear();

               reportCS = true;

            }

         }

      }
      else
      {
            // If we don't have enough data, we report cycle slips
         reportCS = true;
      }

         // Let's prepare for the next epoch

         // Store current epoch at the end of deque
      LIData[sat].LIEpoch.push_back(epoch);

         // Store current value of LI at the end of deque
      LIData[sat].LIBuffer.push_back(li);

         // Update current buffer size
      s = LIData[sat].LIEpoch.size();

         // Check if we have exceeded maximum window size
      if(s > size_t(maxBufferSize))
      {
            // Get rid of oldest data, which is at the beginning of deque
         LIData[sat].LIEpoch.pop_front();
         LIData[sat].LIBuffer.pop_front();

      }


      if (reportCS)
      {
         return 1.0;
      }
      else
      {
         return 0.0;
      }

   }  // End of method 'LICSDetector2::getDetection()'


}  // End of namespace gpstk
