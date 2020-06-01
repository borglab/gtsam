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
 * @file ComputeSatPCenter.hpp
 * This class computes the satellite antenna phase correction, in meters.
 */

#ifndef GPSTK_COMPUTESATPCENTER_HPP
#define GPSTK_COMPUTESATPCENTER_HPP

#include <cmath>
#include <string>
#include <sstream>
#include <gtsam/gpstk/ProcessingClass.hpp>
#include <gtsam/gpstk/Triple.hpp>
#include <gtsam/gpstk/Position.hpp>
#include <gtsam/gpstk/SunPosition.hpp>
#include <gtsam/gpstk/XvtStore.hpp>
#include <gtsam/gpstk/SatDataReader.hpp>
#include <gtsam/gpstk/AntexReader.hpp>
#include <gtsam/gpstk/geometry.hpp>
#include <gtsam/gpstk/StringUtils.hpp>



namespace gpstk
{

      /** @addtogroup DataStructures */
      //@{


      /** This class computes the satellite antenna phase correction, in meters.
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
       *   ComputeSatPCenter svPcenter( SP3EphList,
       *                                nominalPos );
       *
       *   while(rin >> gRin)
       *   {
       *      gRin >> svPcenter;
       *   }
       * @endcode
       *
       * The "ComputeSatPCenter" object will visit every satellite in the GNSS
       * data structure that is "gRin" and will compute the corresponding
       * satellite antenna phase correction, in meters.
       *
       * When used with the ">>" operator, this class returns the same
       * incoming data structure with the "satPCenter" TypeID inserted in it.
       * Be warned that if a given satellite does not have the required data,
       * it will be summarily deleted from the data structure.
       *
       * \warning The ComputeSatPCenter objects generate corrections that are
       * interpreted as an "advance" in the signal, instead of a delay.
       * Therefore, those corrections always hava a negative sign.
       *
       */
   class ComputeSatPCenter : public ProcessingClass
   {
   public:

         /// Default constructor
      ComputeSatPCenter()
         : pEphemeris(NULL), nominalPos(0.0, 0.0, 0.0),
           satData("PRN_GPS"), fileData("PRN_GPS"), pAntexReader(NULL)
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
      ComputeSatPCenter( XvtStore<SatID>& ephem,
                         const Position& stapos,
                         std::string filename="PRN_GPS" )
         : pEphemeris(&ephem), nominalPos(stapos), satData(filename),
           fileData(filename), pAntexReader(NULL)
      { };


         /** Common constructor
          *
          * @param stapos    Nominal position of receiver station.
          * @param filename  Name of "PRN_GPS"-like file containing
          *                  satellite data.
          *
          * @warning If filename is not given, this class will look for a
          * file named "PRN_GPS" in the current directory.
          */
      ComputeSatPCenter( const Position& stapos,
                         std::string filename="PRN_GPS" )
         : pEphemeris(NULL), nominalPos(stapos), satData(filename),
           fileData(filename), pAntexReader(NULL)
      { };


         /** Common constructor. Uses satellite antenna data from an Antex file.
          *
          * @param ephem     Satellite ephemeris.
          * @param stapos    Nominal position of receiver station.
          * @param antexObj  AntexReader object containing satellite
          *                  antenna data.
          *
          * @warning If 'AntexReader' object holds an Antex file with relative
          * antenna data, a simple satellite phase center model will be used.
          */
      ComputeSatPCenter( XvtStore<SatID>& ephem,
                         const Position& stapos,
                         AntexReader& antexObj )
         : pEphemeris(&ephem), nominalPos(stapos), pAntexReader(&antexObj)
      { };


         /** Common constructor. Uses satellite antenna data from an Antex file.
          *
          * @param stapos    Nominal position of receiver station.
          * @param antexObj  AntexReader object containing satellite
          *                  antenna data.
          *
          * @warning If 'AntexReader' object holds an Antex file with relative
          * antenna data, a simple satellite phase center model will be used.
          */
      ComputeSatPCenter( const Position& stapos,
                         AntexReader& antexObj )
         : pEphemeris(NULL), nominalPos(stapos), pAntexReader(&antexObj)
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
      virtual ComputeSatPCenter& setFilename(const std::string& name);


         /// Returns nominal position of receiver station.
      virtual Position getNominalPosition(void) const
      { return nominalPos; };


         /** Sets  nominal position of receiver station.
          * @param stapos    Nominal position of receiver station.
          */
      virtual ComputeSatPCenter& setNominalPosition(const Position& stapos)
        { nominalPos = stapos; return (*this); };


         /// Returns a pointer to the satellite ephemeris object
         /// currently in use.
      virtual XvtStore<SatID> *getEphemeris(void) const
      { return pEphemeris; };


         /** Sets satellite ephemeris object to be used.
          *
          * @param ephem     Satellite ephemeris object.
          */
      virtual ComputeSatPCenter& setEphemeris(XvtStore<SatID>& ephem)
      { pEphemeris = &ephem; return (*this); };


         /// Returns a pointer to the AntexReader object currently in use.
      virtual AntexReader *getAntexReader(void) const
      { return pAntexReader; };


         /** Sets AntexReader object to be used.
          *
          * @param antexObj  AntexReader object containing satellite
          *                  antenna data.
          */
      virtual ComputeSatPCenter& setAntexReader(AntexReader& antexObj)
      { pAntexReader = &antexObj; return (*this); };


         /// Returns a string identifying this object.
      virtual std::string getClassName(void) const;


         /// Destructor
      virtual ~ComputeSatPCenter() {};


   private:


         /// Satellite ephemeris to be used
      XvtStore<SatID> *pEphemeris;


         /// Receiver position
      Position nominalPos;


         /// Object to read satellite data file (PRN_GPS)
      SatDataReader satData;


         /// Name of "PRN_GPS"-like file containing satellite data.
      std::string fileData;


         /// Pointer to object containing satellite antenna data, if available.
      AntexReader* pAntexReader;


         /** Compute the value of satellite antenna phase correction, in meters
          * @param satid     Satellite ID
          * @param time      Epoch of interest
          * @param satpos    Satellite position, as a Triple
          * @param sunpos    Sun position, as a Triple
          *
          * @return Satellite antenna phase correction, in meters.
          */
      virtual double getSatPCenter( const SatID& satid,
                                    const CommonTime& time,
                                    const Triple& satpos,
                                    const Triple& sunPosition );


   }; // End of class 'ComputeSatPCenter'

      //@}

}  // End of namespace gpstk

#endif // GPSTK_COMPUTESATPCENTER_HPP
