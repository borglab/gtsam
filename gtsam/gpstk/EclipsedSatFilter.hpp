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
 * @file EclipsedSatFilter.hpp
 * This class filters out satellites that are eclipsed by Earth shadow.
 */

#ifndef GPSTK_ECLIPSEDSATFILTER_HPP
#define GPSTK_ECLIPSEDSATFILTER_HPP

#include <math.h>
#include <gtsam/gpstk/Triple.hpp>
#include <gtsam/gpstk/SunPosition.hpp>
#include <gtsam/gpstk/Position.hpp>
#include <gtsam/gpstk/ProcessingClass.hpp>
#include <gtsam/gpstk/geometry.hpp>            // DEG_TO_RAD




namespace gpstk
{

      /** @addtogroup GPSsolutions */
      //@{


      /** This class filters out satellites that are eclipsed by Earth shadow.
       *
       * This class is meant to be used with the GNSS data structures objects
       * found in "DataStructures" class.
       *
       * A typical way to use this class follows:
       *
       * @code
       *      // Input observation file stream
       *   RinexObsStream rin("ebre0300.02o");
       *
       *      // Load the precise ephemeris file
       *   SP3EphemerisStore sp3Eph;
       *   sp3Eph.loadFile("igs11513.sp3");

       *      // Reference position of receiver station
       *   Position nominalPos(4833520.2269, 41537.00768, 4147461.489);
       *
       *      // Object to compute basic model data
       *   BasicModel basicM(nominalPos, sp3Eph);
       *
       *      // Object to detect and delete satellites in eclipse
       *   EclipsedSatFilter eclipsedSV;
       *
       *      // Some more code and definitions here...
       *
       *   gnssRinex gRin;  // GNSS data structure for fixed station data
       *
       *   while(rin >> gRin)
       *   {
       *
       *         // Apply the model on the GDS and delete satellites in eclipse
       *      gRin >> basicM >> eclipsedSV;
       *   }
       * @endcode
       *
       * The "EclipsedSatFilter" object will visit every satellite in the GNSS
       * data structure that is "gRin" and will determine if such satellite is
       * in eclipse, or whether it recently was.
       *
       * This effect may be important when using precise positioning, because
       * satellite orbits tend to degrade when satellites are in eclipse, or
       * when they have been in eclipse recently.
       *
       * There are two adjustable parameters in this class: Shadow cone angle
       * (30 degrees by default), and the period after eclipse that the
       * satellite will still be deemed unreliable (1800 seconds by default).
       *
       */
   class EclipsedSatFilter : public ProcessingClass
   {
      public:

         /// Default constructor.
      EclipsedSatFilter() : coneAngle(30.0), postShadowPeriod(1800.0)
      { };


         /** Common constructor
          *
          * @param angle      Aperture angle of shadow cone, in degrees.
          * @param pShTime    Time after exiting shadow that satellite will
          *                   still be filtered out, in seconds.
          */
      EclipsedSatFilter( const double angle,
                         const double pShTime )
         : coneAngle(angle), postShadowPeriod(pShTime)
      { };


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
         throw(ProcessingException)
      { Process(gData.header.epoch, gData.body); return gData; };


         /** Returns a gnnsRinex object, adding the new data generated when
          *  calling this object.
          *
          * @param gData    Data object holding the data.
          */
      virtual gnssRinex& Process(gnssRinex& gData)
         throw(ProcessingException);


         /// Returns aperture of shadow cone, in degrees.
      virtual double getConeAngle(void) const
      { return coneAngle; };


         /** Sets aperture of shadow cone, in degrees.
          *
          * @param angle   Aperture angle of shadow cone, in degrees.
          *
          * \warning Valid values are within 0 and 90 degrees.
          */
      virtual EclipsedSatFilter& setConeAngle(const double angle);


         /// Returns time after exiting shadow that satellite will still be 
         /// filtered out, in seconds.
      virtual double getPostShadowPeriod(void) const
      { return postShadowPeriod; };


         /** Sets time after exiting shadow that satellite will still be 
          *  filtered out, in seconds.
          * @param pShTime    Time after exiting shadow that satellite will
          *                   still be filtered out, in seconds.
          */
      virtual EclipsedSatFilter& setPostShadowPeriod(const double pShTime);


         /// Returns a string identifying this object.
      virtual std::string getClassName(void) const;


         /// Destructor
      virtual ~EclipsedSatFilter() {};


   private:


         /// Aperture angle of shadow cone, in degrees.
      double coneAngle;

         /// Time after exiting shadow that satellite will still be 
         /// filtered out, in seconds.
      double postShadowPeriod;

         /// Map holding the time information about every satellite in eclipse
      std::map<SatID, CommonTime> shadowEpoch;

   }; // End of class 'EclipsedSatFilter'

      //@}

}  // End of namespace gpstk

#endif   // GPSTK_ECLIPSEDSATFILTER_HPP
