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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2008, 2009, 2011
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
 * @file SatArcMarker.hpp
 * This class keeps track of satellite arcs caused by cycle slips.
 */

#ifndef GPSTK_SATARCMARKER_HPP
#define GPSTK_SATARCMARKER_HPP

#include <gtsam/gpstk/ProcessingClass.hpp>



namespace gpstk
{

      /** @addtogroup GPSsolutions */
      //@{


      /** This class keeps track of satellite arcs caused by cycle slips.
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
       *   LICSDetector markCSLI;
       *   SatArcMarker markArc;
       *
       *   while(rin >> gRin)
       *   {
       *      gRin >> markCSLI >> markArc;
       *   }
       * @endcode
       *
       * The "SatArcMarker" object will visit every satellite in the GNSS
       * data structure that is "gRin" and, if a cycle slip has happened
       * (indicated by the corresponding CS flag), it will increase the
       * value of the corresponding "TypeID::satArc" type.
       *
       * By default, the "SatArcMarker" objects will only watch the
       * "TypeID::CSL1" cycle slip flag. This may be changed, although it is
       * rarely necessary because CS detectors raise all flags when a cycle
       * slip happens.
       *
       * An important feature of "SatArcMarker" objects is that you can
       * set a period after arc change when the affected satellite will be
       * considered "unstable". By default, unstable period is set to 31 s,
       * but it may be changed using the appropriate constructor or using
       * the "setUnstablePeriod()" method.
       *
       * Likewise, SatArcMarker" objects are also able to delete unstable
       * satellites from GDS. This feature is disabled by default, but may
       * be activated using the appropriate constructor or using the
       * "setDeleteUnstableSats()".
       *
       * \warning Be aware that this class DOES NOT apply cycle slip detection
       * algorithms, so you MUST invoke CS detection objects BEFORE calling
       * SatArcMarker objects.
       *
       * \warning Objects in this class store their internal state, so you
       * MUST NOT use the SAME object to process DIFFERENT data streams.
       *
       * @sa LICSDetector.hpp and MWCSDetector.hpp for CS detection classes.
       *
       */
   class SatArcMarker : public ProcessingClass
   {
      public:

         /// Default constructor. It will only watch "TypeID::CSL1" flag.
      SatArcMarker()
         : watchCSFlag(TypeID::CSL1), deleteUnstableSats(false),
           unstablePeriod(31.0)
      { };


         /** Common constructor
          *
          * @param watchFlag        Cycle slip flag to be watched.
          * @param delUnstableSats  Whether unstable satellites will be deleted.
          * @param unstableTime     Number of seconds since last arc change
          *                         that a satellite will be considered as
          *                         unstable.
          */
      SatArcMarker( const TypeID& watchFlag,
                    const bool delUnstableSats,
                    const double unstableTime );


         /// Method to get the default CS flag type being used.
      virtual TypeID getCSFlag() const
      { return watchCSFlag; };


         /** Method to set the default CS flag type to be used.
          *
          * @param watchFlag     Cycle slip flag to be watched.
          */
      virtual SatArcMarker& setCSFlag(const TypeID& watchFlag)
      { watchCSFlag = watchFlag; return (*this); };


         /// Method to known if unstable satellites will be deleted.
      virtual bool getDeleteUnstableSat() const
      { return deleteUnstableSats; };


         /** Method to set if unstable satellites will be deleted.
          *
          * @param delUnstableSats  Whether unstable satellites will be deleted.
          */
      virtual SatArcMarker& setDeleteUnstableSats(const bool delUnstableSats)
      { deleteUnstableSats = delUnstableSats; return (*this); };


         /// Method to get the number of seconds since last arc change that a
         /// satellite will be considered as unstable.
      virtual double getUnstablePeriod() const
      { return unstablePeriod; };


         /** Method to set the number of seconds since last arc change that a
          *  satellite will be considered as unstable.
          *
          * @param unstableTime     Number of seconds since last arc change
          *                         that a satellite will be considered as
          *                         unstable.
          */
      virtual SatArcMarker& setUnstablePeriod(const double unstableTime);


         /** Method to get the arc changed epoch.
          * @param sat              Interested SatID.
          */
      virtual CommonTime getArcChangedEpoch(const SatID& sat);

         /** Returns a satTypeValueMap object, adding the new data generated
          *  when calling this object.
          *
          * @param epoch     Time of observations.
          * @param gData     Data object holding the data.
          */
      virtual satTypeValueMap& Process( const CommonTime& epoch,
                                        satTypeValueMap& gData )
         throw(ProcessingException);


         /** Returns a gnnsSatTypeValue object, adding the new data generated
          *  when calling this object.
          *
          * @param gData    Data object holding the data.
          */
      virtual gnssSatTypeValue& Process(gnssSatTypeValue& gData)
         throw(ProcessingException);


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
      virtual ~SatArcMarker() {};


   private:


         /// Cycle slip flag to be watched.
      TypeID watchCSFlag;


         /// Flag indicating if unstable satellites will be deleted.
      bool deleteUnstableSats;


         /// Number of seconds since arc change that a satellite will be
         /// considered as unstable.
      double unstablePeriod;


         /// Map holding information regarding every satellite
      std::map<SatID, double> satArcMap;


         /// Map holding information about epoch of last arc change
      std::map<SatID, CommonTime> satArcChangeMap;


         /// Map keeping track if this satellite is new or not
      std::map<SatID, bool> satIsNewMap;


   }; // End of class 'SatArcMarker'

   //@}

}  // End of namespace gpstk

#endif   // GPSTK_SATARCMARKER_HPP
