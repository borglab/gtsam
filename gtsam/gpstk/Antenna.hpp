#pragma ident "$Id$"

/**
 * @file Antenna.hpp
 * Encapsulates the data related to GNSS antennas.
 */

#ifndef GPSTK_ANTENNA_HPP
#define GPSTK_ANTENNA_HPP

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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2009
//
//============================================================================


#include <cmath>
#include <vector>
#include <map>
#include <string>

#include "Exception.hpp"
#include "CommonTime.hpp"
#include "Triple.hpp"



namespace gpstk
{

      /** @addtogroup DataStructures */
      //@{

      /** This class encapsulates the data related to GNSS antennas according
       *  to IGS standards.
       *
       * Further information about antennas may be found in IGS ftp site:
       *
       *    ftp://igscb.jpl.nasa.gov/pub/station/general/
       *
       * At that site you'll find some very important files:
       *
       * \li rcvr_ant.tab: Official IGS naming conventions for GNSS equipment.
       * \li antex13.txt: ANTEX format definition.
       * \li igs05_wwww.atx: Absolute IGS phase center corrections for
       * satellite and receiver antennas. Field 'wwww' represents GPS week of
       * last file change.
       * \li igs05.atx: Link to latest igs05_wwww.atx file.
       * \li igs_01.atx: Relative IGS phase center corrections for
       * satellite and receiver antennas.
       *
       * This class is normally used in combination with 'AntexReader' class. A
       * typical way to use these classes follows:
       *
       * @code
       *      // Declare some Antenna objects
       *   Antenna antenna1;
       *   Antenna antenna2;
       *   Antenna satGPS02;
       *
       *      // Create AntexReader object
       *   AntexReader antexread;
       *
       *      // Open Antex file. 'igs05.atx' is for absolute phase centers,
       *      // while 'igs_01.atx' is for relative phase centers.
       *   antexread.open("igs05.atx");
       *
       *   double elevation( 34.5 );
       *   double azimuth( 163.2 );
       *
       *      // Get antenna data and accentricity for L1 for satellite GPS-07
       *      // at a specific epoch
       *   CivilTime epoch(2008, 6, 15, 10, 21, 12654.0);
       *   satGPS07 = antexread.getAntenna( "G07", epoch );
       *   cout << satGPS07.getAntennaPCOffset( Antenna::G01 ) << endl;
       *
       *      // Get antenna data and non-azimuth dependent phase center
       *      // offset value + eccentricity for L2.
       *      // Radome type is NOT taken into account
       *   antenna1  = antexread.getAntennaNoRadome("AOAD/M_B");
       *   cout << antenna1.getAntennaPCOffset( Antenna::G02, elevation );
       *
       *      // Get antenna data and azimuth-dependent phase center offset
       *      // value + eccentricity for L1.
       *      // Ask for a specific "strict IGS" model+radome combination
       *   std::string strictIGSModel( "ASH700936B_M    SNOW" );
       *   antenna2  = antexread.getAntenna( strictIGSModel );
       *   cout << antenna2.getAntennaPCOffset( Antenna::G01,
       *                                        elevation,
       *                                        azimuth );
       *
       * @endcode
       *
       * @sa AntexReader.hpp
       */
   class Antenna
   {
   public:

         /// Frequency type
      enum frequencyType
      {
         G01,                    ///< L1 - GPS
         G02,                    ///< L2 - GPS
         G05,                    ///< L5 - GPS
         R01,                    ///< G1 - GLONASS
         R02,                    ///< G2 - GLONASS
         E01,                    ///< E1 - Galileo
         E05,                    ///< E5a - Galileo
         E07,                    ///< E5b - Galileo
         E08,                    ///< E5 (E5a+E5b) - Galileo
         E06                     ///< E6 - Galileo
      };


         /// Antenna data type
      enum AntennaDataType
      {
         antennaType,         ///< Antenna type. IGS standard 'rcvr_ant.tab'
         antennaRadome,       ///< Antenna radome. IGS standard 'rcvr_ant.tab'
         serial,              ///< Serial number/satellite code "CNN"
         cnnn,                ///< Satellite code "CNNN"
         cosparID,            ///< COSPAR ID "YYYY-XXXA"
         calMethod,           ///< Calibration method
         agency,              ///< Name of agency
         numAntennas,         ///< Number of calibrated antennas
         date,                ///< Date
         sinexCode            ///< SINEX code
      };


         // Handy type definitions

         /// Map containing antenna data.
      typedef std::map< AntennaDataType, std::string > AntennaDataMap;

         /// Map for antenna phase center eccentricities.
      typedef std::map< frequencyType, Triple > AntennaEccDataMap;

         /// Map for non-azimuth dependent phase center patterns.
      typedef std::map< frequencyType, std::vector<double> > NoAziDataMap;

         /// Map for azimuth dependent phase center patterns.
      typedef std::map< double, std::vector<double> > AzimuthDataMap;

         /// Map for azimuth dependent phase center patterns and frequencies.
      typedef std::map< frequencyType, AzimuthDataMap > PCDataMap;



         /// Default constructor.
      Antenna() {};


         /** Common constructor.
          *
          * @param[in] eccL1      Eccentricity Triple (meters) for GPS L1 freq.
          * @param[in] eccL2      Eccentricity Triple (meters) for GPS L2 freq.
          */
      Antenna( const Triple& eccL1,
               const Triple& eccL2 );


         /** Common constructor.
          *
          * @param[in] NorthEccL1   North eccentricity (meters) for GPS L1 freq
          * @param[in] EastEccL1    East eccentricity (meters) for GPS L1 freq
          * @param[in] UpEccL1      Up eccentricity (meters) for GPS L1 freq
          * @param[in] NorthEccL2   North eccentricity (meters) for GPS L2 freq
          * @param[in] EastEccL2    East eccentricity (meters) for GPS L2 freq
          * @param[in] UpEccL2      Up eccentricity (meters) for GPS L2 freq
          */
      Antenna( double NorthEccL1,
               double EastEccL1,
               double UpEccL1,
               double NorthEccL2,
               double EastEccL2,
               double UpEccL2 );


         /** Get antenna eccentricity (or 'phase center offset' in Antex
          *  parlance) as a Triple in UEN system.
          *
          * @param[in] freq      Frequency
          *
          * @warning The phase center offset Triple is in UEN system.
          */
      Triple getAntennaEccentricity( frequencyType freq ) const
         throw(InvalidRequest);


         /** Get antenna phase center variation. Use this method when you
          *  don't have azimuth dependent phase center patterns.
          *
          * This method returns a Triple, in UEN system, with the
          * elevation-dependent phase center variation.
          *
          * @param[in] freq      Frequency
          * @param[in] elevation Elevation (degrees)
          *
          * @warning The phase center variation Triple is in UEN system.
          */
      Triple getAntennaPCVariation( frequencyType freq,
                                    double elevation ) const
         throw(InvalidRequest);


         /** Get antenna phase center variation.
          *
          * This method returns a Triple, in UEN system, with the
          * elevation and azimuth-dependent phase center variation.
          *
          * @param[in] freq      Frequency
          * @param[in] elevation Elevation (degrees)
          * @param[in] azimuth   Azimuth (degrees)
          *
          * @warning The phase center variation Triple is in UEN system.
          */
      Triple getAntennaPCVariation( frequencyType freq,
                                    double elevation,
                                    double azimuth ) const
         throw(InvalidRequest);


         /** Get antenna data.
          *
          * @param[in] dataType     Antenna data type to be fetched
          */
      std::string getAntennaData( AntennaDataType dataType ) const
         throw(InvalidRequest);


         /** Set antenna data.
          *
          * @param[in] dataType     Antenna data type to be set
          * @param[in] data         String of data to be stored
          */
      Antenna setAntennaData( AntennaDataType dataType,
                              const std::string& data )
      { antennaData[dataType] = data; return (*this); };


         /// Get antenna type.
      std::string getAntennaType() const
         throw(InvalidRequest)
      { return getAntennaData( antennaType ); };


         /** Set antenna type.
          *
          * @param[in] type      Type of antenna. IGS standard 'rcvr_ant.tab'
          */
      Antenna setAntennaType( const std::string& type )
      { return setAntennaData( antennaType, type ); };


         /// Get antenna radome.
      std::string getAntennaRadome() const
         throw(InvalidRequest)
      { return getAntennaData( antennaRadome ); };


         /** Set antenna radome.
          *
          * @param[in] radome    Type of radome. IGS standard 'rcvr_ant.tab'
          */
      Antenna setAntennaRadome( const std::string& radome )
      { return setAntennaData( antennaRadome, radome ); };


         /// Get antenna serial number or satellite code "CNN".
      std::string getAntennaSerial() const
         throw(InvalidRequest)
      { return getAntennaData(serial); };


         /** Set antenna serial number or satellite code "CNN".
          *
          * @param[in] sn      Serial number/satellite code "CNN"
          */
      Antenna setAntennaSerial( const std::string& sn )
      { return setAntennaData( serial, sn ); };


         /// Get antenna calibration method.
      std::string getAntennaCalMethod() const
         throw(InvalidRequest)
      { return getAntennaData(calMethod); };


         /** Set antenna calibration method.
          *
          * @param[in] method       Antenna calibration method
          */
      Antenna setAntennaCalMethod( const std::string& method )
      { return setAntennaData( calMethod, method ); };


         /// Get start of antenna validity period.
      CommonTime getAntennaValidFrom() const
      { return validFrom; };


         /** Set start of antenna validity period.
          *
          * @param[in] valFrom       Start of validity period
          */
      Antenna setAntennaValidFrom( const CommonTime& valFrom )
      { validFrom = valFrom; return (*this); };


         /// Get end of antenna validity period.
      CommonTime getAntennaValidUntil() const
      { return validUntil; };


         /** Set end of antenna validity period.
          *
          * @param[in] valUntil      End of validity period
          */
      Antenna setAntennaValidUntil( const CommonTime& valUntil )
      { validUntil = valUntil; return (*this); };


         /// Get increment of the azimuth.
      double getDazi() const
      { return dazi; };


         /** Set increment of the azimuth.
          *
          * @param[in] daz      Increment of the azimuth
          */
      Antenna setDazi( double daz )
      { dazi = daz; return (*this); };


         /// Get initial zenith grid value.
      double getZen1() const
      { return zen1; };


         /** Set initial zenith grid value.
          *
          * @param[in] z1      Initial zenith grid value
          */
      Antenna setZen1( double z1 )
      { zen1 = z1; return (*this); };


         /// Get final zenith grid value.
      double getZen2() const
      { return zen2; };


         /** Set final zenith grid value.
          *
          * @param[in] z2      Final zenith grid value
          */
      Antenna setZen2( double z2 )
      { zen2 = z2; return (*this); };


         /// Get increment of the zenith.
      double getDzen() const
      { return dzen; };


         /** Set increment of the zenith.
          *
          * @param[in] dz      Increment of the zenith
          */
      Antenna setDzen( double dz )
      { dzen = dz; return (*this); };


         /// Get number of frequencies.
      int getNumFreq() const
      { return numFreq; };


         /** Set number of frequencies.
          *
          * @param[in] nFreq      Number of frequencies
          */
      Antenna setNumFreq( int nFreq )
      { numFreq = nFreq; return (*this); };


         /// Get antenna data map.
      AntennaDataMap getAntennaDataMap() const
      { return antennaData; };


         /** Set antenna data map.
          *
          * @param[in] dataMap       Antenna data map
          */
      Antenna setAntennaDataMap( const AntennaDataMap& dataMap )
      { antennaData = dataMap; return (*this); };


         /// Get antenna comments.
      std::vector<std::string> getAntennaComments() const
      { return commentList; };


         /** Set antenna comments.
          *
          * @param[in] comments       Antenna comments vector
          */
      Antenna setAntennaComments( const std::vector<std::string>& comments )
      { commentList = comments; return (*this); };


         /** Add antenna comments.
          *
          * @param[in] comments       Antenna comments line
          */
      Antenna addAntennaComments( std::string comments )
      { commentList.push_back(comments); return (*this); };


         /// Get antenna phase center eccentricities map, in METERS.
      AntennaEccDataMap getAntennaEccMap() const
      { return antennaEccMap; };


         /** Set antenna phase center eccentricities map, in METERS.
          *
          * @param[in] eccMap  Antenna phase center eccentricities map, METERS.
          */
      Antenna setAntennaEccMap( const AntennaEccDataMap& eccMap )
      { antennaEccMap = eccMap; return (*this); };


         /** Add antenna phase center ecccentricities, in METERS.
          *
          * @param[in] freq        Frequency.
          * @param[in] trEcc       Eccentricity Triple, in METERS.
          */
      Antenna addAntennaEcc( frequencyType freq,
                             const Triple& trEcc )
      { antennaEccMap[freq] = trEcc; return (*this); };


         /** Add antenna phase center eccentricities, in METERS.
          *
          * @param[in] freq        Frequency.
          * @param[in] northEcc    North eccentricity component, in METERS.
          * @param[in] eastEcc     East eccentricity component, in METERS.
          * @param[in] upEcc       Up eccentricity component, in METERS.
          */
      Antenna addAntennaEcc( frequencyType freq,
                              double northEcc,
                              double eastEcc,
                              double upEcc );


         /// Get antenna phase center RMS eccentricities map, in METERS.
      AntennaEccDataMap getAntennaRMSEccMap() const
      { return antennaRMSEccMap; };


         /** Set antenna phase center RMS eccentricities map, in METERS.
          *
          * @param[in] eccRMSMap    Antenna phase center RMS eccentricities
          *                         map, in METERS
          */
      Antenna setAntennaRMSEccMap(const AntennaEccDataMap& eccRMSMap)
      { antennaRMSEccMap = eccRMSMap; return (*this); };


         /** Add antenna phase center RMS eccentricities, in METERS.
          *
          * @param[in] freq        Frequency.
          * @param[in] northRMS    North eccentricity RMS component, in METERS.
          * @param[in] eastRMS     East eccentricity RMS component, in METERS.
          * @param[in] upRMS       Up eccentricity RMS component, in METERS.
          */
      Antenna addAntennaRMSEcc( frequencyType freq,
                                double northRMS,
                                double eastRMS,
                                double upRMS );


         /// Get antenna non-azimuth dependent patterns map, in METERS.
      NoAziDataMap getAntennaNoAziMap() const
      { return noAziMap; };


         /** Set antenna non-azimuth dependent patterns map, in METERS.
          *
          * @param[in] naMap Antenna non-azimuth dependent patterns map, METERS.
          */
      Antenna setAntennaNoAziMap( const NoAziDataMap& naMap )
      { noAziMap = naMap; return (*this); };


         /** Add antenna non-azimuth dependent pattern, in METERS.
          *
          * @param[in] freq        Frequency.
          * @param[in] pcVec       Vector of phase centers, in METERS.
          */
      Antenna addAntennaNoAziPattern( frequencyType freq,
                                      const std::vector<double>& pcVec )
      { noAziMap[freq] = pcVec; return (*this); };


         /// Get antenna azimuth dependent patterns map, in METERS.
      PCDataMap getAntennaPCMap() const
      { return pcMap; };


         /** Set antenna azimuth dependent patterns map, in METERS.
          *
          * @param[in] pMap Antenna azimuth dependent patterns map, METERS.
          */
      Antenna setAntennaPCMap( const PCDataMap& pMap )
      { pcMap = pMap; return (*this); };


         /** Add antenna azimuth dependent pattern, in METERS.
          *
          * @param[in] freq        Frequency.
          * @param[in] azi         Azimuth.
          * @param[in] pcVec       Vector of phase centers, in METERS.
          */
      Antenna addAntennaPattern( frequencyType freq,
                                 double azi,
                                 const std::vector<double>& pcVec )
      { pcMap[freq][azi] = pcVec; return (*this); };


         /// Get antenna non-azimuth dependent RMS map, in METERS.
      NoAziDataMap getAntennaNoAziRMSMap() const
      { return noAziRMSMap; };


         /** Set antenna non-azimuth dependent RMS map, in METERS.
          *
          * @param[in] naRMSMap Antenna non-azimuth dependent RMS map, METERS.
          */
      Antenna setAntennaNoAziRMSMap( const NoAziDataMap& naRMSMap )
      { noAziRMSMap = naRMSMap; return (*this); };


         /** Add antenna non-azimuth dependent RMS, in METERS.
          *
          * @param[in] freq        Frequency.
          * @param[in] pcRMS       Vector of phase centers RMS, in METERS.
          */
      Antenna addAntennaNoAziRMS( frequencyType freq,
                                  const std::vector<double>& pcRMS )
      { noAziRMSMap[freq] = pcRMS; return (*this); };


         /// Get antenna azimuth dependent RMS map, in METERS.
      PCDataMap getAntennaPCRMSMap() const
      { return pcRMSMap; };


         /** Set antenna azimuth dependent RMS map, in METERS.
          *
          * @param[in] pRMSMap Antenna azimuth dependent patterns map, METERS.
          */
      Antenna setAntennaPCRMSMap( const PCDataMap& pRMSMap )
      { pcRMSMap = pRMSMap; return (*this); };


         /** Add antenna azimuth dependent RMS, in METERS.
          *
          * @param[in] freq        Frequency.
          * @param[in] azi         Azimuth.
          * @param[in] pcRMSVec    Vector of phase centers RMS, in METERS.
          */
      Antenna addAntennaPatternRMS( frequencyType freq,
                                    double azi,
                                    const std::vector<double>& pcRMSVec )
      { pcRMSMap[freq][azi] = pcRMSVec; return (*this); };


         /// Get size of antenna data map.
      int getAntennaDataSize() const
      { return antennaData.size(); };


         /// Get size of antenna phase center eccentricities map.
      int getAntennaEccMapSize() const
      { return antennaEccMap.size(); };


         /// Get size of non-azimuth dependent phase center patterns map.
      int getNoAziMapSize() const
      { return noAziMap.size(); };


         /// Get size of azimuth dependent phase center patterns map.
      int getPCMapSize() const
      { return pcMap.size(); };


         /// Returns if this object is valid. The validity criteria is to
         /// have a non-empty 'antennaData' map AND a non-empty 'antennaEccMap'.
      bool isValid() const;


         /// Destructor
      virtual ~Antenna() {};


   private:


         /// Map with antenna data
      AntennaDataMap antennaData;


      double dazi;                     ///< Increment of the azimuth

      double zen1;                     ///< Initial zenith grid value
      double zen2;                     ///< Final zenith grid value
      double dzen;                     ///< Increment of the zenith

      int numFreq;                     ///< Number of frequencies

      CommonTime validFrom;               ///< Start of validity period
      CommonTime validUntil;              ///< End of validity period

      std::vector<std::string> commentList;      ///< Comments


         /// Map holding antenna phase center eccentricities, in METERS
      AntennaEccDataMap antennaEccMap;


         /// Non-azimuth dependent phase center patterns map
      NoAziDataMap noAziMap;


         /// Azimuth dependent phase center patterns map
      PCDataMap pcMap;


         /// Map holding antenna phase center RMS eccentricities, in METERS
      AntennaEccDataMap antennaRMSEccMap;


         /// Non-azimuth dependent RMS
      NoAziDataMap noAziRMSMap;


         /// Azimuth dependent phase center RMS
      PCDataMap pcRMSMap;


         /** Linear interpolation as function of normalized angle
          *
          * @param[in] dataVector         std::vector holding data.
          * @param[in] normalizedAngle    Normalized angle.
          *
          * 'normalizedAngle' is a value corresponding to the original angle
          * divided by the angle interval.
          */
      double linearInterpol( const std::vector<double>& dataVector,
                             double normalizedAngle ) const;


   }; // End of class 'Antenna'


      //@}

}  // End of namespace gpstk

#endif   // GPSTK_ANTENNA_HPP
