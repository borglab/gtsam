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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2007, 2008, 2009, 2011 
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
 * @file ComputeWindUp.hpp
 * This class computes the wind-up effect on the phase observables, in radians.
 */

#ifndef GPSTK_COMPUTEWINDUP_HPP
#define GPSTK_COMPUTEWINDUP_HPP

#include <string>
#include "ProcessingClass.hpp"
#include "Triple.hpp"
#include "Position.hpp"
#include "SunPosition.hpp"
#include "XvtStore.hpp"
#include "SatDataReader.hpp"
#include "geometry.hpp"



namespace gpstk
{

      /** @addtogroup DataStructures */
      //@{


      /** This class computes the wind-up effect on the phase observables,
       *  in radians.
       *
       * This class is meant to be used with the GNSS data structures objects
       * found in "DataStructures" class.
       *
       * A typical way to use this class follows:
       *
       * @code
       *      // Create the input obs file stream
       *   RinexObsStream rin("ebre0300.02o");
       *
       *      // Loads precise ephemeris object with file data
       *   SP3EphemerisStore SP3EphList;
       *   SP3EphList.loadFile("igs11513.sp3");
       *
       *      // Sets nominal position of receiver
       *   Position nominalPos(4833520.3800, 41536.8300, 4147461.2800);
       *
       *   gnssRinex gRin;
       *
       *   ComputeWindUp windup( SP3EphList,
       *                         nominalPos );
       *
       *   while(rin >> gRin)
       *   {
       *      gRin >> windup;
       *   }
       * @endcode
       *
       * The "ComputeWindUp" object will visit every satellite in the GNSS
       * data structure that is "gRin" and will compute the corresponding
       * receiver-satellite wind-up effect, in radians.
       *
       * When used with the ">>" operator, this class returns the same
       * incoming data structure with the wind-up inserted in it. Be warned
       * that if a given satellite does not have the observations required,
       * it will be summarily deleted from the data structure.
       *
       * \warning ComputeWindUp objects store their internal state, so
       * you MUST NOT use the SAME object to process DIFFERENT data streams.
       *
       * \warning It is recommended that ComputeWindUp objects are used after
       * calling a SatArcMarker object, because they work better when cycle
       * slips are properly tracked.
       *
       */
   class ComputeWindUp : public ProcessingClass
   {
   public:

         /// Default constructor
      ComputeWindUp()
         : pEphemeris(NULL), nominalPos(0.0, 0.0, 0.0),
           satData("PRN_GPS"), fileData("PRN_GPS")
      { };


         /** Common constructor
          *
          * @param ephem     Satellite ephemeris.
          * @param stapos    Nominal position of receiver station.
          * @param filename  Name of "PRN_GPS"-like file containing
          *                  satellite data.
          *
          * @warning If filename is not given, this class will look for a
          * file named "PRN_GPS" in the current directory.
          */
      ComputeWindUp( XvtStore<SatID>& ephem,
                     const Position& stapos,
                     std::string filename="PRN_GPS" )
         : pEphemeris(&ephem), nominalPos(stapos), satData(filename),
           fileData(filename)
      { };


         /** Returns a satTypeValueMap object, adding the new data generated
          *  when calling this object.
          *
          * @param time      Epoch corresponding to the data.
          * @param gData     Data object holding the data.
          */
      virtual satTypeValueMap& Process( const CommonTime& time,
                                        satTypeValueMap& gData )
         throw(ProcessingException);


         /** Returns a gnnsSatTypeValue object, adding the new data
          *  generated when calling this object.
          *
          * @param gData    Data object holding the data.
          */
      virtual gnssSatTypeValue& Process(gnssSatTypeValue& gData)
         throw(ProcessingException)
      { Process(gData.header.epoch, gData.body); return gData; };


         /** Returns a gnnsRinex object, adding the new data generated
          *  when calling this object.
          *
          * @param gData    Data object holding the data.
          */
      virtual gnssRinex& Process(gnssRinex& gData)
         throw(ProcessingException)
      { Process(gData.header.epoch, gData.body); return gData; };


         /// Returns name of "PRN_GPS"-like file containing satellite data.
      virtual std::string getFilename(void) const
      { return fileData; };


         /** Sets name of "PRN_GPS"-like file containing satellite data.
          * @param name      Name of satellite data file.
          */
      virtual ComputeWindUp& setFilename(const std::string& name);


         /// Returns nominal position of receiver station.
      virtual Position getNominalPosition(void) const
      { return nominalPos; };


         /** Sets  nominal position of receiver station.
          * @param stapos    Nominal position of receiver station.
          */
      virtual ComputeWindUp& setNominalPosition(const Position& stapos)
        { nominalPos = stapos; return (*this); };


         /// Returns a pointer to the satellite ephemeris object
         /// currently in use.
      virtual XvtStore<SatID> *getEphemeris(void) const
      { return pEphemeris; };


         /** Sets satellite ephemeris object to be used.
          * @param ephem     Satellite ephemeris object.
          */
      virtual ComputeWindUp& setEphemeris(XvtStore<SatID>& ephem)
      { pEphemeris = &ephem; return (*this); };


         /// Returns a string identifying this object.
      virtual std::string getClassName(void) const;


         /// Destructor
      virtual ~ComputeWindUp() {};


   private:


         /// Satellite ephemeris to be used
      XvtStore<SatID> *pEphemeris;


         /// Receiver position
      Position nominalPos;


         /// Object to read satellite data file (PRN_GPS)
      SatDataReader satData;


         /// Name of "PRN_GPS"-like file containing satellite data.
      std::string fileData;


         /// A structure used to store phase data.
      struct phaseData
      {
            // Default constructor initializing the data in the structure
         phaseData() : previousPhase(0.0) {};

         double previousPhase;      ///< Previous phase.
      };


         /// Map to store station phase data
      std::map<SatID, phaseData> phase_station;


         /// Map to store satellite phase data
      std::map<SatID, phaseData> phase_satellite;


         /// Map to store satellite arc data
      std::map<SatID, double> satArcMap;


         /** Compute the value of the wind-up, in radians.
          * @param sat       Satellite ID
          * @param time      Epoch of interest
          * @param satpos    Satellite position, as a Triple
          * @param sunpos    Sun position, as a Triple
          *
          * @return Wind-up computation, in radians
          */
      virtual double getWindUp( const SatID& sat,
                                const CommonTime& time,
                                const Triple& satpos,
                                const Triple& sunpos );


   }; // End of class 'ComputeWindUp'

      //@}

}  // End of namespace gpstk

#endif // GPSTK_COMPUTEWINDUP_HPP
