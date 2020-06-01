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
 * @file CodeSmoother.hpp
 * This class smoothes a given code observable using the corresponding
 * phase observable.
 */

#ifndef GPSTK_CODESMOOTHER_HPP
#define GPSTK_CODESMOOTHER_HPP

#include <gtsam/gpstk/ProcessingClass.hpp>


namespace gpstk
{

      /** @addtogroup DataStructures */
      //@{


      /** This class smoothes a given code observable using the corresponding
       *  phase observable.
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
       *   OneFreqCSDetector markCSC1;    // We MUST mark cycle slips
       *   CodeSmoother smoothC1;
       *
       *   while(rin >> gRin)
       *   {
       *      gRin >> markCSC1 >> smoothC1;
       *   }
       * @endcode
       *
       * The "CodeSmoother" object will visit every satellite in the GNSS data
       * structure that is "gRin" and will smooth the given code observation
       *  using the corresponding phase observation.
       *
       * By default, the algorithm will use C1 and L1 observables, and the
       * CSL1 index will be consulted for cycle slip information. You can
       * change these settings with the appropriate methods.
       *
       * When used with the ">>" operator, this class returns the same incoming
       * data structure with the code observation smoothed (unless the
       * 'resultType' field is changed). Be warned that if a given satellite
       * does not have the  observations required, it will be summarily deleted
       * from the data structure.
       *
       * Another important parameter is the maxWindowSize field. By default, it
       * is set to 100 samples (you may adjust that with the setMaxWindowSize()
       * method).
       *
       * A window of 100 samples is typical and appropriate when working with
       * data sampled at 1 Hz, because then the full window will last at most
       * 100 seconds.
       *
       * However, if for instance your samples are taken at 30 seconds (and you
       * are working with C1/L1 or other ionosphere-affected observation pair),
       * then the former value of number of samples will yield a window of 50
       * minutes will be used and you will get badly distorted data because of
       * ionosphere drift, among other effects.
       *
       * A good rule here is to make sure that the filter window lasts at most
       * 5 minutes. Therefore, for a 30 s sampling data set you should set your
       * smoother object like this:
       *
       * @code
       *   CodeSmoother smoothC1;
       *   smoothC1.setMaxWindowSize(8);
       * @endcode
       *
       * Resulting in a 4 minutes filter window.
       *
       * \warning Code smoothers are objets that store their internal state,
       * so you MUST NOT use the SAME object to process DIFFERENT data streams.
       *
       */
   class CodeSmoother : public ProcessingClass
   {
   public:

         /// Default constructor, setting default parameters and C1 and L1
         /// as observables.
      CodeSmoother() : codeType(TypeID::C1), phaseType(TypeID::L1),
         resultType(TypeID::C1), maxWindowSize(100), csFlag(TypeID::CSL1) 
      { };


         /** Common constructor
          *
          * @param codeT         Type of code to be smoothed.
          * @param mwSize        Maximum  size of filter window, in samples.
          */
      CodeSmoother( const TypeID& codeT,
                    const int& mwSize = 100 );


         /** Returns a satTypeValueMap object, adding the new data generated
          *  when calling this object.
          *
          * @param gData     Data object holding the data.
          */
      virtual satTypeValueMap& Process(satTypeValueMap& gData)
         throw(ProcessingException);


         /** Returns a gnnsSatTypeValue object, adding the new data generated
          *  when calling this object.
          *
          * @param gData    Data object holding the data.
          */
      virtual gnssSatTypeValue& Process(gnssSatTypeValue& gData)
         throw(ProcessingException)
      { Process(gData.body); return gData; };


         /** Returns a gnnsRinex object, adding the new data generated when
          *  calling this object.
          *
          * @param gData    Data object holding the data.
          */
      virtual gnssRinex& Process(gnssRinex& gData)
         throw(ProcessingException)
      { Process(gData.body); return gData; };


         /// Method to get the default code type being used.
      virtual TypeID getCodeType() const
      { return codeType; };


         /** Method to set the default code type to be used.
          *
          * @param codeT     TypeID of code to be used
          */
      virtual CodeSmoother& setCodeType(const TypeID& codeT)
      { codeType = codeT; return (*this); };


         /// Method to get the default phase type being used.
      virtual TypeID getPhaseType() const
      { return phaseType; };


         /** Method to set the default phase type to be used.
          *
          * @param phaseT    TypeID of phase to be used
          */
      virtual CodeSmoother& setPhaseType(const TypeID& phaseT)
      { phaseType = phaseT; return (*this); };


         /// Method to get the default cycle slip type being used.
      virtual TypeID getCSFlag() const
      { return csFlag; };


         /** Method to set the default cycle slip type to be used.
          *
          * @param csT   Cycle slip type to be used
          */
      virtual CodeSmoother& setCSFlag(const TypeID& csT)
      { csFlag = csT; return (*this); };


         /// Method to get the default return type being used.
      virtual TypeID getResultType() const
      { return resultType; };


         /** Method to set the default return type to be used.
          *
          * @param returnT    TypeID to be returned
          */
      virtual CodeSmoother& setResultType(const TypeID& resultT)
      { resultType = resultT; return (*this); };


         /// Method to get the maximum size of filter window, in samples.
      virtual int getMaxWindowSize() const
      { return maxWindowSize; };


         /** Method to set the maximum size of filter window, in samples.
          *
          * @param maxSize       Maximum size of filter window, in samples.
          */
      virtual CodeSmoother& setMaxWindowSize(const int& maxSize);


         /// Returns a string identifying this object.
      virtual std::string getClassName(void) const;


         /// Destructor
      virtual ~CodeSmoother() {};


   private:

         /// Type of code observation to be used.
      TypeID codeType;


         /// Type of phase observation to be used.
      TypeID phaseType;


         /// Type assigned to the resulting smoothed code.
      TypeID resultType;


         /// Maximum size of filter window, in samples.
      int maxWindowSize;


         /// Cycle slip flag. It MUST be present.
         /// @sa OneFreqCSDetector.hpp class.
      TypeID csFlag;


         /// A structure used to store filter data for a SV.
      struct filterData
      {
            // Default constructor initializing the data in the structure
         filterData() : windowSize(1), previousCode(0.0), previousPhase(0.0) {};

         int windowSize;       ///< The filter window size.
         double previousCode;  ///< Accumulated mean bias (pseudorange - phase).
         double previousPhase; ///< Accumulated mean bias sigma squared.
      };


         /// Map holding the information regarding every satellite
      std::map<SatID, filterData> SmoothingData;


         /** Compute the smoothed code observable.
          *
          * @param sat        Satellite object.
          * @param code       Code measurement.
          * @param phase      Phase measurement.
          * @param flag       Cycle slip flag.
          */
      virtual double getSmoothing( const SatID& sat,
                                   const double& code,
                                   const double& phase,
                                   const double& flag );


   }; // End of class 'CodeSmoother'

      //@}

}  // End of namespace gpstk

#endif   // GPSTK_CODESMOOTHER_HPP
