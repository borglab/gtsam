#pragma ident "$Id$"

/**
 * @file IonexStore.hpp
 * Read and store Ionosphere maps. It computes TEC and RMS values with respect
 * to time and receiver position. Based on extracted TEC values, it calculates
 * the ionospheric delay.
 */

#ifndef GPSTK_IONEXSTORE_HPP
#define GPSTK_IONEXSTORE_HPP

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
//  Octavian Andrei - FGI ( http://www.fgi.fi ). 2008, 2009
//
//============================================================================


#include <map>

#include "FileStore.hpp"
#include "IonexData.hpp"

#include "geometry.hpp"                   // DEG_TO_RAD
#include "GNSSconstants.hpp"          // LX_FREQ, with X = 1,2,5,6,7,8
#include "Triple.hpp"

namespace gpstk
{

      /** @addtogroup IonosphereMaps */
      //@{

      /** This class reads and stores Ionosphere maps.
       *
       * It computes TEC and RMS values with respect to time and receiver
       * position. Based on extracted TEC values, it calculates the ionospheric
       * delay.
       *
       * @sa test ionex store.cpp for an example
       *
       *
       * @warning The first IONEX map refers to 00:00 UT, the last map
       *          to 24:00 UT. The time spacing of the maps (snapshots) is 2
       *          hours. When two consecutive files are loaded the previuous
       *          map for 24:00 UT is overwritten by the new 00:00 UT. This
       *          might affect the interpolation strategy.
       */
   class IonexStore : public FileStore<IonexHeader>
   {
   public:


         /// Default constructor.
      IonexStore()
         throw()
         : initialTime(CommonTime::END_OF_TIME),
           finalTime(CommonTime::BEGINNING_OF_TIME)
      {};


         /// destructor
      virtual ~IonexStore() {};


         /// Load the given IONEX file
      virtual void loadFile(const std::string& filename)
         throw(FileMissingException);


         /// Insert a new IonexData object into the store
      void addMap(const IonexData& iod)
         throw();


         /** Dump the store to the provided std::ostream (std::cout by default).
          *
          * @param s       std::ostream object to dump the data to.
          * @param detail  Determines how much detail to include in the output:
          *                0 list of filenames with their start and stop times.
          *                1 list of filenames with their start, stop times,
          *                  type of data and for how many epochs.
          */
      void dump( std::ostream& s = std::cout,
                 short detail = 0 ) const
         throw();


         /// Remove all data
      void clear() throw();


         /** Get IONEX TEC, RMS and ionosphere height values as a function of
          *  epoch and receiver's position.
          *
          * Four interpolation strategies are suported  (see also Ionex manual:
          * http://igscb.jpl.nasa.gov/igscb/data/format/ionex1.pdf )
          *
          * A simple 4-point formula is applied to interpolate between the grid
          * points. See more at IonexData::getValue()
          *
          * @param t          Time tag of signal (CommonTime object)
          * @param RX         Receiver position in ECEF cartesian coordinates
          *                   (meters).
          * @param strategy   Interpolation strategy
          *                   (1) take neareast map,
          *                   (2) interpolate between two consecutive maps,
          *                   (3) interpolate between two consecutive rotated
          *                       maps or,
          *                   (4) take neareast rotated map.
          *
          * @return values    TEC, RMS and ionosphere height values
          *                   (Triple object with: TEC and RMS in TECU and
          *                   the ionosphere height in meters)
          */
      Triple getIonexValue( const CommonTime& t,
                            const Position& RX,
                            int strategy = 3 ) const
         throw(InvalidRequest);



      /** Get slant total electron content (STEC) in TECU
       *
       * @param elevation     Time tag of signal (CommonTime object)
       * @param tecval        TEC value as derived from IONEX file (TECU)
       * @param ionoMapType   Type of ionosphere mapping function (string)
       *                      @sa IonexStore::iono_mapping_function
       *
       * @return              slant total electron content (TECU)
       */
   double getSTEC( const double& elevation,
                   const double& tecval,
                   const std::string& ionoMapType ) const
      throw (InvalidParameter);



         /** Get ionospheric slant delay for a given frequency
          *
          * @param elevation     Time tag of signal (CommonTime object)
          * @param tecval        TEC value as derived from IONEX file (TECU)
          * @param freq          Frequency value, in Hz
          * @param ionoMapType   Type of ionosphere mapping function (string)
          *                      @sa IonexStore::iono_mapping_function
          *
          * @return              Ionosphere slant delay (meters)
          */
      double getIono( const double& elevation,
                      const double& tecval,
                      const double& freq,
                      const std::string& ionoMapType ) const
         throw (InvalidParameter);


         // The next 6 functions define the interface for calculating
         // the ionospheric slant delay for a specific frequency

         /** Get ionospheric slant delay for L1 frequency
          *
          * @param elevation     Time tag of signal (CommonTime object)
          * @param tecval        TEC value as derived from IONEX file (TECU)
          * @param ionoMapType   Type of ionosphere mapping function (string)
          *                      @sa IonexStore::iono_mapping_function
          *
          * @return              Ionosphere slant delay (meters)
          */
      double getIonoL1( const double& elevation,
                        const double& tecval,
                        const std::string& ionoMapType ) const
         throw (InvalidParameter)
      { return getIono(elevation, tecval, L1_FREQ_GPS, ionoMapType); };


         /** Get ionospheric slant delay for L2 frequency
          *
          * @param elevation     Time tag of signal (CommonTime object)
          * @param tecval        TEC value as derived from IONEX file (TECU)
          * @param ionoMapType   Type of ionosphere mapping function (string)
          *                      @sa IonexStore::iono_mapping_function
          *
          * @return              Ionosphere slant delay (meters)
          */
      double getIonoL2( const double& elevation,
                        const double& tecval,
                        const std::string& ionoMapType ) const
         throw (InvalidParameter)
      { return getIono(elevation, tecval, L2_FREQ_GPS, ionoMapType); };


         /** Get ionospheric slant delay for L5 frequency
          *
          * @param elevation     Time tag of signal (CommonTime object)
          * @param tecval        TEC value as derived from IONEX file (TECU)
          * @param ionoMapType   Type of ionosphere mapping function (string)
          *                      @sa IonexStore::iono_mapping_function
          *
          * @return              Ionosphere slant delay (meters)
          */
      double getIonoL5( const double& elevation,
                        const double& tecval,
                        const std::string& ionoMapType ) const
         throw (InvalidParameter)
      { return getIono(elevation, tecval, L5_FREQ_GPS, ionoMapType); };


         /** Get ionospheric slant delay for L6 frequency
          *
          * @param elevation     Time tag of signal (CommonTime object)
          * @param tecval        TEC value as derived from IONEX file (TECU)
          * @param ionoMapType   Type of ionosphere mapping function (string)
          *                      @sa IonexStore::iono_mapping_function
          *
          * @return              Ionosphere slant delay (meters)
          */
      double getIonoL6( const double& elevation,
                        const double& tecval,
                        const std::string& ionoMapType ) const
         throw (InvalidParameter)
      { return getIono(elevation, tecval, L6_FREQ_GAL, ionoMapType); };


         /** Get ionospheric slant delay for L7 frequency
          *
          * @param elevation     Time tag of signal (CommonTime object)
          * @param tecval        TEC value as derived from IONEX file (TECU)
          * @param ionoMapType   Type of ionosphere mapping function (string)
          *                      @sa IonexStore::iono_mapping_function
          *
          * @return              Ionosphere slant delay (meters)
          */
      double getIonoL7( const double& elevation,
                        const double& tecval,
                        const std::string& ionoMapType ) const
         throw (InvalidParameter)
      { return getIono(elevation, tecval, L7_FREQ_GAL, ionoMapType); };


         /** Get ionospheric slant delay for L8 frequency
          *
          * @param elevation     Time tag of signal (CommonTime object)
          * @param tecval        TEC value as derived from IONEX file (TECU)
          * @param ionoMapType   Type of ionosphere mapping function (string)
          *                      @sa IonexStore::iono_mapping_function
          *
          * @return              Ionosphere slant delay (meters)
          */
      double getIonoL8( const double& elevation,
                        const double& tecval,
                        const std::string& ionoMapType ) const
         throw (InvalidParameter)
      { return getIono(elevation, tecval, L8_FREQ_GAL, ionoMapType); };


         /** Ionosphere mapping function
          *
          * @param elevation     Elevation of satellite as seen at receiver
          *                      (degrees).
          * @param ionoMapType   Type of ionosphere mapping function (string)
          *                      (0) NONE no mapping function is applied
          *                      (1) SLM  Single Layer Model (IGS)
          *                      (2) MSLM Modified Single Layer Model (CODE)
          *                      (3) ESM  Extended Slab Model (JLP)
          *
          * Details at: http://aiuws.unibe.ch/ionosphere/mslm.pdf
          *
          * @warning No implementation for JPL's mapping function.
          */
      double iono_mapping_function( const double& elevation,
                                    const std::string& ionoMapType ) const;


         /** Determine the earliest time for which this object can
          *  successfully determine the TEC values, and implicitly, the
          *  ionospheric delay for any object.
          *
          * @return     Initial time.
          *
          * @throw      InvalidRequest This is thrown if the object has no data.
          */
      CommonTime getInitialTime() const
         throw(InvalidRequest)
      { return initialTime; }


         /** Determine the latest time for which this object can successfully
          *  determine the TEC values, and implicitly, the ionospheric delay
          *  for any object.
          *
          * @return     Final time.
          *
          * @throw      InvalidRequest This is thrown if the object has no data.
          */
      CommonTime getFinalTime() const
         throw(InvalidRequest)
      { return finalTime; }


         /** Find a DCB value
          *
          * @param sat     SatID of satellite of interest
          * @param t       Time to search for DCB
          *
          * @return        DCB value found (nanoseconds).
          *
          * @throw InvalidRequest object thrown when no DCB value is found
          */
      double findDCB( const SatID sat,
                      const CommonTime& time ) const
         throw(InvalidRequest);


   private:


         /** These fields set the overall span of time for which this object
          *  contains data.
          *
          * @warning There may be gaps in the data, i.e. the data may not be
          *          continuous.
          */
      CommonTime initialTime, finalTime;


         /// The key to this map is IonexValType
      typedef std::map<IonexData::IonexValType, IonexData> IonexValTypeMap;


         /// The key to this map is the time
      typedef std::map<CommonTime, IonexValTypeMap> IonexMap;


         /// Map of IONEX maps
      IonexMap inxMaps;


         /// The key of this map is the time (first epoch as in IonexHeader)
      typedef std::map<CommonTime, IonexHeader::SatDCBMap> IonexDCBMap;


         /// Map of DCB values (IonexHeader.firstEpoch, IonexHeader.svsmap)
      IonexDCBMap inxDCBMap;


   }; // End of class 'IonexStore'

      //@}

}  // End of namespace gpstk
#endif   // GPSTK_IONEXSTORE_HPP
