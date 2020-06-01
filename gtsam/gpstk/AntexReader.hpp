#pragma ident "$Id$"

/**
 * @file AntexReader.hpp
 * Class to read antenna data from files in Antex format.
 */

#ifndef GPSTK_ANTEXREADER_HPP
#define GPSTK_ANTEXREADER_HPP

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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2009, 2011
//
//============================================================================


#include <string>
#include <map>

#include "Exception.hpp"
#include "FFTextStream.hpp"
#include "StringUtils.hpp"
#include "CommonTime.hpp"
#include "SatID.hpp"
#include "Antenna.hpp"


namespace gpstk
{

      /// Thrown when some problem appeared when reading Antex data
      /// @ingroup exceptiongroup
   NEW_EXCEPTION_CLASS(InvalidAntex, gpstk::Exception);


      /** @addtogroup formattedfile */
      //@{

      /** This is a class to read and parse antenna data in Antex file format.
       *
       * You may find antenna-related data at:
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
       * This class is normally used in combination with 'Antenna' class. A
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
       *      // Get antenna data and eccentricity for L1 for satellite GPS-07
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
       * @sa Antenna.hpp
       */
   class AntexReader : public FFTextStream
   {
   public:


         /**
          * @name AntexReaderFormatStrings
          * Antex Formatting Strings
          */
         //@{
      static const std::string versionString;      ///< "ANTEX VERSION / SYST"
      static const std::string pcvTypeString;      ///< "PCV TYPE / REFANT"
      static const std::string commentString;      ///< "COMMENT"
      static const std::string endOfHeader;        ///< "END OF HEADER"

      static const std::string startOfAntenna;     ///< "START OF ANTENNA"
      static const std::string typeSerial;         ///< "TYPE / SERIAL NO"
      static const std::string calibrationMethod;  ///< "METH / BY / # / DATE"
      static const std::string incrementAzimuth;   ///< "DAZI"
      static const std::string zenithGrid;         ///< "ZEN1 / ZEN2 / DZEN"
      static const std::string numberFreq;         ///< "# OF FREQUENCIES"
      static const std::string validFrom;          ///< "VALID FROM"
      static const std::string validUntil;         ///< "VALID UNTIL"
      static const std::string sinexCode;          ///< "SINEX CODE"
      static const std::string startOfFreq;        ///< "START OF FREQUENCY"
      static const std::string antennaEcc;         ///< "NORTH / EAST / UP"
      static const std::string endOfFreq;          ///< "END OF FREQUENCY"
      static const std::string startOfFreqRMS;     ///< "START OF FREQ RMS"
      static const std::string antennaEccRMS;      ///< "NORTH / EAST / UP"
      static const std::string endOfFreqRMS;       ///< "END OF FREQ RMS"
      static const std::string endOfAntenna;       ///< "END OF ANTENNA"
         //@}


         /// Phase center variation type
      enum pcvType
      {
         absolute = 1,           ///< Absolute PCV
         relative,               ///< Relative PCV
         Unknown                 ///< Unknown PCV
      };


         /// Default constructor
      AntexReader()
         : fileName(""), version(1.3), valid(false)
      {};


         /** Common constructor. It will always open Antex file for read and
          *  will load Antex file header data in one pass.
          *
          * @param fn   Antex data file to read
          *
          */
      AntexReader(const char* fn)
         : FFTextStream( fn, std::ios::in )
      { fileName = fn; loadHeader(); };


         /** Common constructor. It will always open Antex file for read and
          *  will load Antex file header data in one pass.
          *
          * @param fn   Antex data file to read
          *
          */
      AntexReader(const std::string& fn)
         : FFTextStream( fn.c_str(), std::ios::in )
      { fileName = fn; loadHeader(); };
       
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Woverloaded-virtual"
       

         /// Method to open and load Antex file header data.
      virtual void open(const char* fn);


         /// Method to open and load Antex file header data.
      virtual void open(const std::string& fn);
#pragma clag diagnostic pop

         /** Method to get antenna data from a given model. Just the model,
          *  without including the radome
          *
          * @param model      Antenna model, without including radome.
          *
          * @note Antenna model case is NOT relevant.
          *
          * @warning The antenna returned will be the first one in the Antex
          * file that matches the condition.
          */
      virtual Antenna getAntennaNoRadome(const std::string& model)
         throw(ObjectNotFound);


         /** Method to get antenna data from a given IGS model.
          *
          * @param model      IGS antenna model
          *
          * @note Antenna model case is NOT relevant.
          *
          * @note IGS antenna model combines antenna type and radome.
          *
          * @warning The antenna returned will be the first one in the Antex
          * file that matches the condition.
          *
          * @warning If IGS model doesn't include radome, method
          * 'getAntennaNoRadome()' will be automatically called.
          */
      virtual Antenna getAntenna(const std::string& model)
         throw(ObjectNotFound);


         /** Method to get antenna data from a given IGS model and serial.
          *
          * @param model      IGS antenna model
          * @param serial     Antenna serial number.
          *
          * @note Antenna model and serial number case is NOT relevant.
          *
          * @note IGS antenna model combines antenna type and radome.
          *
          * @warning The antenna returned will be the first one in the Antex
          * file that matches the conditions.
          */
      virtual Antenna getAntenna( const std::string& model,
                                  const std::string& serial )
         throw(ObjectNotFound);


         /** Method to get antenna data from a given IGS model and serial, and
          *  for a specific epoch.
          *
          * @param model      IGS antenna model
          * @param serial     Antenna serial number.
          * @param epoch      Validity epoch.
          *
          * @note Antenna model and serial number case is NOT relevant.
          *
          * @note IGS antenna model combines antenna type and radome.
          *
          * @warning The antenna returned will be the first one in the Antex
          * file that matches the conditions.
          */
      virtual Antenna getAntenna( const std::string& model,
                                  const std::string& serial,
                                  const CommonTime& epoch )
         throw(ObjectNotFound);


         /** Method to get antenna data from a given serial and a specific
          *  epoch.
          *
          * This method is particularly useful to look for satellite antennas.
          *
          * @param serial     Antenna serial number.
          * @param epoch      Validity epoch.
          *
          * @note Antenna serial number case is NOT relevant.
          *
          * @warning The antenna returned will be the first one in the Antex
          * file that matches the conditions.
          */
      virtual Antenna getAntenna( const std::string& serial,
                                  const CommonTime& epoch )
         throw(ObjectNotFound);


         /// Returns if this object is valid.
      bool isValid() const
      { return valid; };


         /// Returns version of Antex file.
      double getVersion() const
      { return version; };


         /// Returns if loaded antenna data file is absolute or relative.
      bool isAbsolute() const;


         /// This methods dumps all data in Antex file header.
      virtual void dump(std::ostream& s) const;


         /// Destructor
      virtual ~AntexReader() {};


   private:


         // Handy antenna data types

         // Validity:Antennas
      typedef std::map< CommonTime, Antenna > ValAntMap;

         // Calibration:Validity:Antennas
      typedef std::map< std::string, ValAntMap > CalValAntMap;

         // Serial:Calibration:Validity:Antennas
      typedef std::map< std::string, CalValAntMap > SerCalValAntMap;

         // Radome:Serial:Calibration:Validity:Antennas
      typedef std::map< std::string, SerCalValAntMap > RSCalValAntMap;

         // Model:Radome:Serial:Calibration:Validity:Antennas
      typedef std::map< std::string, RSCalValAntMap > AntennaDataMap;


         /// Map holding antennas already serched for (Antenna buffer)
      AntennaDataMap antennaMap;


         /// Antex file name
      std::string fileName;

         /// Antex file version
      double version;

         /// Satellite system
      SatID::SatelliteSystem system;

         /// PCV type
      pcvType type;

         /// Reference antenna type (relative)
      std::string refAntena;

         /// Reference antenna serial (relative)
      std::string refAntenaSerial;

         /// Comments in header (optional)
      std::vector<std::string> commentList;

         /// Whether this Antex header is valid
      bool valid;


         /// Parse a single header line. Returns label.
      std::string parseHeaderLine( const std::string& line )
         throw(InvalidAntex);


         /// Fill most Antenna data
      Antenna fillAntennaData( const std::string& firstLine );


         /// Method to load Antex file header data.
      virtual void loadHeader(void)
         throw( InvalidAntex,
                FFStreamError,
                gpstk::StringUtils::StringException );


   }; // End of class 'AntexReader'


      /// Operator << for AntexReader
   inline std::ostream& operator<<( std::ostream& s,
                                    const AntexReader& antread )
   {

         // Call dump() method
      antread.dump(s);

      return s;

   }  // End of operator '<<'

      //@}

}  // End of namespace gpstk

#endif  // GPSTK_ANTEXREADER_HPP
