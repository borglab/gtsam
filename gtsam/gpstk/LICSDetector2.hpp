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
 * @file LICSDetector2.hpp
 * This is a class to detect cycle slips using LI observables and a
 * 2nd order fitting curve.
 */

#ifndef GPSTK_LICSDETECTOR2_HPP
#define GPSTK_LICSDETECTOR2_HPP

#include <deque>
#include "ProcessingClass.hpp"



namespace gpstk
{

      /** @addtogroup GPSsolutions */
      //@{


      /** This is a class to detect cycle slips using LI observables and a
       *  2nd order fitting curve.
       *
       * This class is meant to be used with the GNSS data structures objects
       * found in "DataStructures" class.
       *
       * A typical way to use this class follows:
       *
       * @code
       *   RinexObsStream rin("ebre0300.02o");
       *
       *   gnssRinex gRin;
       *   ComputeLI getLI;
       *   LICSDetector2 markCSLI;
       *
       *   while(rin >> gRin)
       *   {
       *      gRin >> getLI >> markCSLI;
       *   }
       * @endcode
       *
       * The "LICSDetector2" object will visit every satellite in the GNSS data
       * structure that is "gRin" and will decide if a cycle slip has happened
       * in the given observable.
       *
       * The algorithm will use LI observables, and the LLI1 and LLI2 indexes.
       * The result (a 1 if a cycle slip is found, 0 otherwise) will be stored
       * in the data structure both as the CSL1 and CSL2 indexes.
       *
       * This algorithm will take a set of samples (between 5 and 12 by
       * default) and will build a 2nd order fitting curve using a least mean
       * squares adjustment method. Then, the current LI bias with respect to
       * this fitting curve is computed, and the result compared with a
       * time-varying threshold value.
       *
       * Such threshold value is computed using an exponential function with a
       * given saturation threshold and time constant. The maximum interval of
       * time allowed between two successive epochs is also used as criterion
       * for cycle slip detection.
       *
       * The default values for threshold computing are carefully tuned, so you
       * must not change them without a very good reason.
       *
       * On the other hand, it is very important that you adjust the maximum
       * time interval to your sampling rate. This time interval is 61 seconds
       * by default, which is appropriate for 30 seconds per sample RINEX
       * observation files.
       *
       * When used with the ">>" operator, this class returns the same incoming
       * data structure with the cycle slip indexes inserted along their
       * corresponding satellites. Be warned that if a given satellite does not
       * have the observations required, it will be summarily deleted from the
       * data structure.
       *
       * Be aware that some combinations of cycle slips in L1 and L2 may result
       * in a false negative when using a cycle slip detector based on LI.
       * Therefore, to be on the safe side you should complement this with
       * another kind of detector, such as one based on the Melbourne-Wubbena
       * combination.
       *
       * Given that this cycle slip detector needs a given ammount of samples
       * in order to start working, it will declare cycle slips when its
       * buffers don't have enough data. The minimum buffer size is 5 samples
       * by default, so every time this detector starts, or when it finds a
       * cycle slip, there will tipically be at least four successive epochs
       * when cycle slips are declared.
       *
       * Therefore, and taking into account that this detector is often used in
       * combination with SatArcMarker objects, you must adjust the unstable
       * period in those objects in order to match the feeding buffer time of
       * this type of detectors.
       *
       * @sa MWCSDetector.hpp, LICSDetector.hpp and SatArcMarker.hpp.
       *
       * \warning Cycle slip detectors are objets that store their internal
       * state, so you MUST NOT use the SAME object to process DIFFERENT data
       * streams.
       *
       */
   class LICSDetector2 : public ProcessingClass
   {
   public:

         /// Default constructor, setting default parameters.
      LICSDetector2() : obsType(TypeID::LI), lliType1(TypeID::LLI1),
                        lliType2(TypeID::LLI2), resultType1(TypeID::CSL1),
                        resultType2(TypeID::CSL2), deltaTMax(61.0),
                        satThreshold(0.08), timeConst(60.0), useLLI(true),
                        maxBufferSize(12)
      { };


         /** Common constructor
          *
          * @param satThr  Saturation threshold to declare cycle slip, in
          *                meters.
          * @param tc      Threshold time constant, in seconds.
          * @param dtMax   Maximum interval of time allowed between two
          *                successive epochs, in seconds.
          */
      LICSDetector2( const double& satThr,
                     const double& tc,
                     const double& dtMax = 61.0,
                     const bool& use = true );


         /** Returns a satTypeValueMap object, adding the new data generated
          *  when calling this object.
          *
          * @param epoch     Time of observations.
          * @param gData     Data object holding the data.
          * @param epochflag Epoch flag.
          */
      virtual satTypeValueMap& Process( const CommonTime& epoch,
                                        satTypeValueMap& gData,
                                        const short& epochflag = 0 )
         throw(ProcessingException);



         /** Method to get the maximum interval of time allowed between two
          *  successive epochs, in seconds.
          */
      virtual double getDeltaTMax() const
      { return deltaTMax; };


         /** Method to set the maximum interval of time allowed between two
          *  successive epochs.
          *
          * @param maxDelta      Maximum interval of time, in seconds
          */
      virtual LICSDetector2& setDeltaTMax(const double& maxDelta);


         /** Method to get the saturation threshold for cycle slip detection,
          *  in meters.
          */
      virtual double getSatThreshold() const
      { return satThreshold; };


         /** Method to set the saturation threshold for cycle slip detection,
          *  in meters.
          *
          * @param satThr  Saturation threshold for cycle slip detection, in
          *                meters.
          *
          * \warning Be sure you have a very good reason to change this value.
          */
      virtual LICSDetector2& setSatThreshold(const double& satThr);


         /// Method to get threshold time constant, in seconds
      virtual double getTimeConst() const
      { return timeConst; };


         /** Method to set threshold time constant, in seconds
          *
          * @param tc      Threshold time constant, in seconds.
          *
          * \warning Be sure you have a very good reason to change this value.
          */
      virtual LICSDetector2& setTimeConst(const double& tc);


         /// Method to know if the LLI check is enabled or disabled.
      virtual bool getUseLLI() const
      { return useLLI; };


         /** Method to set whether the LLI indexes will be used as an aid
          *  or not.
          *
          * @param use   Boolean value enabling/disabling LLI check
          */
      virtual LICSDetector2& setUseLLI(const bool& use)
      { useLLI = use; return (*this); };


         /** Method to get the maximum buffer size for data, in samples.
          */
      virtual double getMaxBufferSize() const
      { return maxBufferSize; };


         /** Method to set the maximum buffer size for data, in samples.
          *
          * @param maxBufSize      Maximum buffer size for data, in samples.
          *
          * \warning You must not set a value under minBufferSize, which
          * usually is 5.
          */
      virtual LICSDetector2& setMaxBufferSize(const int& maxBufSize);


         /** Returns a gnnsSatTypeValue object, adding the new data generated
          *  when calling this object.
          *
          * @param gData    Data object holding the data.
          */
      virtual gnssSatTypeValue& Process(gnssSatTypeValue& gData)
         throw(ProcessingException)
      { Process(gData.header.epoch, gData.body); return gData; };


         /** Returns a gnnsRinex object, adding the new data generated when
          *  calling this object.
          *
          * @param gData    Data object holding the data.
          */
      virtual gnssRinex& Process(gnssRinex& gData)
         throw(ProcessingException);


         /// Returns a string identifying this object.
      virtual std::string getClassName(void) const;


         /// Destructor
      virtual ~LICSDetector2() {};


   private:


         /// Type of observable.
      TypeID obsType;


         /// Type of LLI1 record.
      TypeID lliType1;


         /// Type of LLI2 record.
      TypeID lliType2;


         /// Type of result #1.
      TypeID resultType1;


         /// Type of result #2.
      TypeID resultType2;


         /// Maximum interval of time allowed between two successive epochs,
         /// in seconds.
      double deltaTMax;


         /// Saturation threshold to declare cycle slip, in meters.
      double satThreshold;


         /// Threshold time constant, in seconds.
      double timeConst;


         /// This field tells whether to use or ignore the LLI indexes as
         /// an aid.
      bool useLLI;


         /// Maximum buffer size.
      int maxBufferSize;


         /// Minimum size of buffer. It is always set to 5
      static const int minBufferSize;


         /// A structure used to store filter data for a SV.
      struct filterData
      {
            // Default constructor initializing the data in the structure
         filterData()
         {};

         std::deque<CommonTime> LIEpoch; ///< Epochs of previous LI observables.
         std::deque<double> LIBuffer;  ///< Values of previous LI observables.
      };


         /// Map holding the information regarding every satellite
      std::map<SatID, filterData> LIData;


         /** Method that implements the LI cycle slip detection algorithm
          *
          * @param epoch     Time of observations.
          * @param sat       SatID.
          * @param tvMap     Data structure of TypeID and values.
          * @param epochflag Epoch flag.
          * @param li        Current LI observation value.
          * @param lli1      LLI1 index.
          * @param lli2      LLI2 index.
          */
      virtual double getDetection( const CommonTime& epoch,
                                   const SatID& sat,
                                   typeValueMap& tvMap,
                                   const short& epochflag,
                                   const double& li,
                                   const double& lli1,
                                   const double& lli2 );


  }; // End of class 'LICSDetector2'

      //@}

}  // End of namespace gpstk

#endif   // GPSTK_LICSDETECTOR2_HPP
