#pragma ident "$Id$"

/**
 * @file AntexReader.cpp
 * Class to read antenna data from files in Antex format.
 */

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


#include "AntexReader.hpp"
#include "CivilTime.hpp"


using namespace std;
using namespace gpstk::StringUtils;

namespace gpstk
{


      // Let's define Antex formatting strings.
   const string AntexReader::versionString      = "ANTEX VERSION / SYST";
   const string AntexReader::pcvTypeString      = "PCV TYPE / REFANT";
   const string AntexReader::commentString      = "COMMENT";
   const string AntexReader::endOfHeader        = "END OF HEADER";

   const string AntexReader::startOfAntenna     = "START OF ANTENNA";
   const string AntexReader::typeSerial         = "TYPE / SERIAL NO";
   const string AntexReader::calibrationMethod  = "METH / BY / # / DATE";
   const string AntexReader::incrementAzimuth   = "DAZI";
   const string AntexReader::zenithGrid         = "ZEN1 / ZEN2 / DZEN";
   const string AntexReader::numberFreq         = "# OF FREQUENCIES";
   const string AntexReader::validFrom          = "VALID FROM";
   const string AntexReader::validUntil         = "VALID UNTIL";
   const string AntexReader::sinexCode          = "SINEX CODE";
   const string AntexReader::startOfFreq        = "START OF FREQUENCY";
   const string AntexReader::antennaEcc         = "NORTH / EAST / UP";
   const string AntexReader::endOfFreq          = "END OF FREQUENCY";
   const string AntexReader::startOfFreqRMS     = "START OF FREQ RMS";
   const string AntexReader::antennaEccRMS      = "NORTH / EAST / UP";
   const string AntexReader::endOfFreqRMS       = "END OF FREQ RMS";
   const string AntexReader::endOfAntenna       = "END OF ANTENNA";



      // Parse a single header line. Returns label.
   string AntexReader::parseHeaderLine( const std::string& line )
      throw(InvalidAntex)
   {

         // Get label. Remove trailing and leading blanks
      string label( strip( line.substr(60,20) ) );


         // Process version line
      if( label == versionString )
      {
         version = asDouble( line.substr(0,8) );
         char sys( line[20] );

         switch (sys)
         {
            case ' ':
            case 'G':
               system = SatID::systemGPS;
               break;
            case 'R':
               system = SatID::systemGlonass;
               break;
            case 'E':
               system = SatID::systemGalileo;
               break;
            case 'M':
               system = SatID::systemMixed;
               break;
            default:
               InvalidAntex e("Invalid GNSS system in Antex header.");
               GPSTK_THROW(e);
         }

         return label;

      }  // End of 'if( label == versionString )...'


         // Process PCV type line
      if( label == pcvTypeString )
      {
         char pcvt( line[0] );

         switch (pcvt)
         {
            case 'A':
               type = absolute;
               break;
            case 'R':
               type = relative;
               refAntena = strip( line.substr(20,20) );
               if( refAntena == "" )
               {
                  refAntena = "AOAD/M_T";
               }
               refAntenaSerial = strip( line.substr(40,20) );
               break;
            default:
               InvalidAntex e("Invalid PCV type in Antex header.");
               GPSTK_THROW(e);
         }

         return label;

      }  // End of 'if( label == pcvTypeString )...'


         // Process comments lines
      if( label == commentString )
      {

            // Get comment and put into 'commentList'
         commentList.push_back( strip( line.substr(0,60) ) );

         return label;

      }  // End of 'if( label == commentString )...'


         // Just in case none of the previous cases applied
      return label;


   }  // End of method 'AntexReader::parseHeaderLine()'



      // Fill most Antenna data
   Antenna AntexReader::fillAntennaData( const std::string& firstLine )
   {

         // These flags take care of "Valid From" and "Valid Until"
      bool validFromPresent(false);
      bool validUntilPresent(false);

         // Create 'Antenna' object to be returned
      Antenna antenna;

         // Fill antenna with information in first line
      antenna.setAntennaType( strip( firstLine.substr(0,15) ) );
      antenna.setAntennaRadome( strip( firstLine.substr(16,4) ) );
      antenna.setAntennaSerial( strip( firstLine.substr(20,20) ) );

      antenna.setAntennaData( Antenna::cnnn,
                              strip( firstLine.substr(40,10) ) );
      antenna.setAntennaData( Antenna::cosparID,
                              strip( firstLine.substr(50,10) ) );

         // Read the rest of data
      std::string line;

         // Read one line from file
      formattedGetLine(line, true);

      std::string label( strip( line.substr(60,20) ) );

         // Repeat until 'endOfAntenna' line
      while( label != endOfAntenna )
      {

            // Process 'calibrationMethod' line
         if( label == calibrationMethod )
         {
               // Get antenna calibration method
            antenna.setAntennaCalMethod( strip( line.substr(0,20) ) );
               // Get antenna calibration agency
            antenna.setAntennaData( Antenna::agency,
                                    strip( line.substr(20,20) ) );
               // Get number of individual antennas calibrated
            antenna.setAntennaData( Antenna::numAntennas,
                                    strip( line.substr(40,6) ) );
               // Get calibration date
            antenna.setAntennaData( Antenna::date,
                                    strip( line.substr(50,10) ) );

         }  // End of 'if( label == calibrationMethod )...'

            // Process 'incrementAzimuth' line
         if( label == incrementAzimuth )
         {
            antenna.setDazi( asDouble( strip( line.substr(2,6) ) ) );
         }

            // Process 'zenithGrid' line
         if( label == zenithGrid )
         {
            antenna.setZen1( asDouble( strip( line.substr(2,6) ) ) );
            antenna.setZen2( asDouble( strip( line.substr(8,6) ) ) );
            antenna.setDzen( asDouble( strip( line.substr(14,6) ) ) );
         }

            // Process 'numberFreq' line
         if( label == numberFreq )
         {
            antenna.setNumFreq( asInt( strip( line.substr(0,6) ) ) );
         }

            // Process 'validFrom' line
         if( label == validFrom )
         {
               // Get validity as Year, Month, Day, Hour, Min, Sec
            CivilTime valFrom( asInt( strip( line.substr(0,6) ) ),
                            asInt( strip( line.substr(6,6) ) ),
                            asInt( strip( line.substr(12,6) ) ),
                            asInt( strip( line.substr(18,6) ) ),
                            asInt( strip( line.substr(24,6) ) ),
                            asDouble( strip( line.substr(30,13) ) ),TimeSystem::Any );

            antenna.setAntennaValidFrom( valFrom );

               // Mark that we found "Valid From"
            validFromPresent = true;
         }

            // Process 'validUntil' line
         if( label == validUntil )
         {
               // Get validity as Year, Month, Day, Hour, Min, Sec
            CivilTime valUntil( asInt( strip( line.substr(0,6) ) ),
                             asInt( strip( line.substr(6,6) ) ),
                             asInt( strip( line.substr(12,6) ) ),
                             asInt( strip( line.substr(18,6) ) ),
                             asInt( strip( line.substr(24,6) ) ),
                             asDouble( strip( line.substr(30,13) ) ),TimeSystem::Any );

            antenna.setAntennaValidUntil( valUntil );

               // Mark that we found "Valid Until"
            validUntilPresent = true;
         }

            // Process 'sinexCode' line
         if( label == sinexCode )
         {
               // Get antenna Sinex Code
            antenna.setAntennaData( Antenna::sinexCode,
                                    strip( line.substr(0,10) ) );
         }

            // Process 'commentString' line
         if( label == commentString )
         {
               // Add antenna comment
            antenna.addAntennaComments( strip( line.substr(0,60) ) );
         }


            // Process frequency info
         if( label == startOfFreq )
         {

               // Get frequency indicator
            std::string freqString( strip( line.substr(3,3) ) );

               // Set frequency type
            Antenna::frequencyType freq;

            if( freqString == "G01" ) freq = Antenna::G01;
            else if( freqString == "G02" ) freq = Antenna::G02;
            else if( freqString == "G05" ) freq = Antenna::G05;
            else if( freqString == "R01" ) freq = Antenna::R01;
            else if( freqString == "R02" ) freq = Antenna::R02;
            else if( freqString == "E01" ) freq = Antenna::E01;
            else if( freqString == "E05" ) freq = Antenna::E05;
            else if( freqString == "E07" ) freq = Antenna::E07;
            else if( freqString == "E08" ) freq = Antenna::E08;
            else if( freqString == "E06" ) freq = Antenna::E06;

               // Read new line and extract label
            formattedGetLine(line, true);
            label = strip( line.substr(60,20) );

               // Repeat until 'endOfFreq' line
            while( label != endOfFreq )
            {

                  // Process 'antennaEcc' line
               if( label == antennaEcc )
               {
                     // Add antenna eccentricities, as METERS
                  antenna.addAntennaEcc( freq,
                              asDouble( strip( line.substr(0,10)  ) ) / 1000.0,
                              asDouble( strip( line.substr(10,10) ) ) / 1000.0,
                              asDouble( strip( line.substr(20,10) ) ) / 1000.0);
               }
               else
               {
                     // Check if this line contains "NOAZI" pattern
                  if( strip( line.substr(3,5) ) == "NOAZI" )
                  {
                        // We need a vector to store values
                     std::vector<double> pcVec;

                        // Get 'NOAZI' out of our way
                     stripFirstWord(line);

                        // Extract values (they are in milimeters)
                     for(double zen =  antenna.getZen1();
                                zen <= antenna.getZen2();
                                zen += antenna.getDzen() )
                     {
                        double value( asDouble( stripFirstWord(line) ) );
                           // Store values as meters
                        pcVec.push_back( (value / 1000.0 ) );
                     }

                        // Add pattern to antenna
                     antenna.addAntennaNoAziPattern( freq,
                                                     pcVec );
                  }
                  else
                  {
                        // This part processes azimuth-dependent patterns

                        // We need a vector to store values
                     std::vector<double> pcVec;

                        // Get 'azimuth
                     double azi( asDouble( stripFirstWord(line) ) );

                        // Extract values (they are in milimeters)
                     for(double zen =  antenna.getZen1();
                                zen <= antenna.getZen2();
                                zen += antenna.getDzen() )
                     {
                        double value( asDouble( stripFirstWord(line) ) );
                           // Store values as meters
                        pcVec.push_back( (value / 1000.0 ) );
                     }

                        // Add pattern to antenna
                     antenna.addAntennaPattern( freq,
                                                azi,
                                                pcVec );

                  }  // End of 'if( strip( line.substr(3,5) ) == "NOAZI" )...'

               }  // End of 'if( label == antennaEcc )...'

                  // Read new line and extract label
               formattedGetLine(line, true);

               label = strip( line.substr(60,20) );

            }  // End of 'while( label != endOfFreq )...'

         }  // End of 'if( label == startOfFreq )...'



            // Process frequency RMS info
         if( label == startOfFreqRMS )
         {

               // Get frequency indicator
            std::string freqString( strip( line.substr(3,3) ) );

               // Set frequency type
            Antenna::frequencyType freq;

            if( freqString == "G01" ) freq = Antenna::G01;
            else if( freqString == "G02" ) freq = Antenna::G02;
            else if( freqString == "G05" ) freq = Antenna::G05;
            else if( freqString == "R01" ) freq = Antenna::R01;
            else if( freqString == "R02" ) freq = Antenna::R02;
            else if( freqString == "E01" ) freq = Antenna::E01;
            else if( freqString == "E05" ) freq = Antenna::E05;
            else if( freqString == "E07" ) freq = Antenna::E07;
            else if( freqString == "E08" ) freq = Antenna::E08;
            else if( freqString == "E06" ) freq = Antenna::E06;

               // Read new line and extract label
            formattedGetLine(line, true);
            label = strip( line.substr(60,20) );

               // Repeat until 'endOfFreqRMS' line
            while( label != endOfFreqRMS )
            {

                  // Process 'antennaEccRMS' line
               if( label == antennaEccRMS )
               {
                     // Add antenna eccentricities RMS, as METERS
                  antenna.addAntennaRMSEcc( freq,
                              asDouble( strip( line.substr(0,10)  ) ) / 1000.0,
                              asDouble( strip( line.substr(10,10) ) ) / 1000.0,
                              asDouble( strip( line.substr(20,10) ) ) / 1000.0);
               }
               else
               {
                     // Check if this line contains "NOAZI" pattern RMS
                  if( strip( line.substr(3,5) ) == "NOAZI" )
                  {
                        // We need a vector to store RMS values
                     std::vector<double> pcRMS;

                        // Get 'NOAZI' out of our way
                     stripFirstWord(line);

                        // Extract values (they are in milimeters)
                     for(double zen =  antenna.getZen1();
                                zen <= antenna.getZen2();
                                zen += antenna.getDzen() )
                     {
                        double value( asDouble( stripFirstWord(line) ) );
                           // Store RMS values values as meters
                        pcRMS.push_back( (value / 1000.0 ) );
                     }

                        // Add pattern RMS to antenna
                     antenna.addAntennaNoAziRMS( freq,
                                                 pcRMS );
                  }
                  else
                  {
                        // This part processes azimuth-dependent patterns RMS

                        // We need a vector to store RMS values
                     std::vector<double> pcRMS;

                        // Get 'azimuth
                     double azi( asDouble( stripFirstWord(line) ) );

                        // Extract values (they are in milimeters)
                     for(double zen =  antenna.getZen1();
                                zen <= antenna.getZen2();
                                zen += antenna.getDzen() )
                     {
                        double value( asDouble( stripFirstWord(line) ) );
                           // Store RMS values as meters
                        pcRMS.push_back( (value / 1000.0 ) );
                     }

                        // Add pattern RMS to antenna
                     antenna.addAntennaPatternRMS( freq,
                                                   azi,
                                                   pcRMS );

                  }  // End of 'if( strip( line.substr(3,5) ) == "NOAZI" )...'

               }  // End of 'if( label == antennaEccRMS )...'

                  // Read new line and extract label
               formattedGetLine(line, true);

               label = strip( line.substr(60,20) );

            }  // End of 'while( label != endOfFreqRMS )...'

         }  // End of 'if( label == startOfFreqRMS )...'


            // Read another line from file
         formattedGetLine(line, true);

            // Get current label
         label = strip( line.substr(60,20) );
      }

         // Take care of "Valid From" field if it wasn't present
      if( !validFromPresent )
      {
            // Set as "CommonTime::BEGINNING_OF_TIME"
         antenna.setAntennaValidFrom( CommonTime::BEGINNING_OF_TIME );
      }

         // Take care of "Valid Until" field if it wasn't present
      if( !validUntilPresent )
      {
            // Set as "CommonTime::END_OF_TIME"
         antenna.setAntennaValidUntil( CommonTime::END_OF_TIME );
      }

      return antenna;

   }  // End of method 'Antenna::fillAntennaData()'



      // Method to load Antex file header data.
   void AntexReader::loadHeader(void)
      throw( InvalidAntex,
             FFStreamError,
             gpstk::StringUtils::StringException )
   {

      try
      {

         std::string label;

         while( label != endOfHeader )
         {
            std::string line;

               // Read one line from the file
            formattedGetLine(line, true);

               // Process line
            label = parseHeaderLine(line);
         }

         valid = true;

      }  // End of try block
      catch (InvalidAntex& ia)
      {
         GPSTK_RETHROW(ia);
      }
      catch (EndOfFile& e)
      {
         return;
      }
      catch (...)
      {
         InvalidAntex ia("Unknown error when reading Antex header.");
         GPSTK_THROW(ia);
      }

   } // End of method 'AntexReader::loadHeader()'



      /* Method to get antenna data from a given model. Just the model,
       * without including the radome
       *
       * @param model      Antenna model, without including radome.
       *
       * @note Antenna model case is NOT relevant.
       *
       * @warning The antenna returned will be the first one in the Antex
       * file that matches the condition.
       */
   Antenna AntexReader::getAntennaNoRadome(const string& model)
      throw(ObjectNotFound)
   {

         // Flag that signals if we found the antenna
      bool antennaFound(false);

         // Create 'Antenna' object to be returned
      Antenna antenna;

         // We need to read the data stream (file) from the beginning
      FFTextStream::open( fileName.c_str(), std::ios::in );

         // Strip radome, change to upper case and strip leading and
         // trailing spaces
      string uModel( strip( upperCase( model.substr(0,15) ) ) );

         // Getting antennas out of Antex file is a costly process, so this
         // object will keep a "buffer" called 'antennaMap' where all antennas
         // previously looked up are stored.
         // Then, let's look first into this "buffer"
      AntennaDataMap::const_iterator it1( antennaMap.find(uModel) );
      if( it1 != antennaMap.end() )
      {

            // Return the first antenna found. Note that there several
            // different maps in cascade.
         RSCalValAntMap::const_iterator it2( (*it1).second.begin() );
         SerCalValAntMap::const_iterator it3( (*it2).second.begin() );
         CalValAntMap::const_iterator it4( (*it3).second.begin() );
         ValAntMap::const_iterator it5( (*it4).second.begin() );

            // we found the antenna
         antenna = (*it5).second;
         antennaFound = true;

      }  // End of 'if( it1 != antennaMap.end() )...'


         // Antenna is not in 'antennaMap': Let's look for it in file
      if( !antennaFound )
      {

         try
         {

               // Repeat until antenna is found or End Of File
            while( !antennaFound )
            {

               std::string label;
               std::string line;

                  // Look for 'typeSerial' line
               while( label != typeSerial )
               {
                     // Read one line from file
                  formattedGetLine(line, true);

                     // Get label
                  label = strip( line.substr(60,20) );
               }

                  // Check if model matches. Read only model, not radome
               if( uModel == strip( line.substr(0,15) ) )
               {

                     // We found the antenna. Fill it with data
                  antenna = fillAntennaData( line );

                     // Insert antenna into buffer 'antennaMap'
                  antennaMap[ antenna.getAntennaType()      ]
                            [ antenna.getAntennaRadome()    ]
                            [ antenna.getAntennaSerial()    ]
                            [ antenna.getAntennaCalMethod() ]
                            [ antenna.getAntennaValidFrom() ] = antenna;

                  antennaFound = true;

               }  // End of 'if( uModel == strip( line.substr(0,15) ) )...'

            }  // End of 'while( !antennaFound )...'

         }  // End of try block
         catch( InvalidAntex& ia )
         {

               // We need to close this data stream
            (*this).close();

            GPSTK_RETHROW(ia);
         }
         catch( EndOfFile& e )
         {
               // We need to close this data stream
            (*this).close();

            ObjectNotFound notFound("Antenna not found in Antex file.");
            GPSTK_THROW(notFound);
         }
         catch(...)
         {
               // We need to close this data stream
            (*this).close();

            InvalidAntex ia("Unknown error when reading Antex header.");
            GPSTK_THROW(ia);
         }

      }  // End of 'if( !antennaFound )...'


         // We need to close this data stream
      (*this).close();

         // Return antenna
      return antenna;

   }  // End of method 'AntexReader::getAntennaNoRadome()'



      /* Method to get antenna data from a given IGS model.
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
   Antenna AntexReader::getAntenna(const string& model)
      throw(ObjectNotFound)
   {

         // Flag that signals if we found the antenna
      bool antennaFound(false);

         // Create 'Antenna' object to be returned
      Antenna antenna;


         // We need to read the data stream (file) from the beginning
      FFTextStream::open( fileName.c_str(), std::ios::in );

         // Change input to upper case and strip leading and trailing spaces
      string uModel( strip( upperCase( model.substr(0,15) ) ) );

         // Check if we have radome data here. If not, call alternative method
      if( model.size() < 17 )
      {
            // We need to close this data stream
         (*this).close();

         return getAntennaNoRadome(uModel);
      }

         // Get radome
      string uRadome( strip( upperCase( model.substr(16,4) ) ) );

         // Getting antennas out of Antex file is a costly process, so this
         // object will keep a "buffer" called 'antennaMap' where all antennas
         // previously looked up are stored.
         // Then, let's look first into this "buffer"
      AntennaDataMap::const_iterator it1( antennaMap.find(uModel) );
      if( it1 != antennaMap.end() )
      {

         RSCalValAntMap::const_iterator it2( (*it1).second.find(uRadome) );

         if( it2 != (*it1).second.end() )
         {

               // Return the first antenna found. Note that there several
               // different maps in cascade.
            SerCalValAntMap::const_iterator it3( (*it2).second.begin() );
            CalValAntMap::const_iterator it4( (*it3).second.begin() );
            ValAntMap::const_iterator it5( (*it4).second.begin() );

               // we found the antenna
            antenna = (*it5).second;
            antennaFound = true;

         }  // End of 'if( it2 != (*it1).second.end() )...'

      }  // End of 'if( it1 != antennaMap.end() )'


         // Antenna is not in 'antennaMap': Let's look for it in file
      if( !antennaFound )
      {

         try
         {

               // Repeat until antenna is found or End Of File
            while( !antennaFound )
            {

               std::string label;
               std::string line;

                  // Look for 'typeSerial' line
               while( label != typeSerial )
               {
                     // Read one line from file
                  formattedGetLine(line, true);

                     // Get label
                  label = strip( line.substr(60,20) );
               }

                  // Check if model matches. Read only model, not radome
               if( uModel == strip( line.substr(0,15) ) )
               {

                     // Check if radome matches
                  if( uRadome == strip( line.substr(16,4) ) )
                  {

                        // We found the antenna. Fill it with data
                     antenna = fillAntennaData( line );

                        // Insert antenna into buffer 'antennaMap'
                     antennaMap[ antenna.getAntennaType()      ]
                               [ antenna.getAntennaRadome()    ]
                               [ antenna.getAntennaSerial()    ]
                               [ antenna.getAntennaCalMethod() ]
                               [ antenna.getAntennaValidFrom() ] = antenna;

                     antennaFound = true;

                  }  // End of 'if( uModel == strip( line.substr(0,15) ) )...'

               } // End of 'if( uModel == strip( line.substr(0,15) ) )...'

            }  // End of 'while( !antennaFound )...'

         }  // End of try block
         catch( InvalidAntex& ia )
         {

               // We need to close this data stream
            (*this).close();

            GPSTK_RETHROW(ia);
         }
         catch( EndOfFile& e )
         {
               // We need to close this data stream
            (*this).close();

            ObjectNotFound notFound("Antenna not found in Antex file.");
            GPSTK_THROW(notFound);
         }
         catch(...)
         {
               // We need to close this data stream
            (*this).close();

            InvalidAntex ia("Unknown error when reading Antex header.");
            GPSTK_THROW(ia);
         }

      }  // End of 'if( !antennaFound )...'


         // We need to close this data stream
      (*this).close();

         // Return antenna
      return antenna;

   }  // End of method 'AntexReader::getAntenna()'



      /* Method to get antenna data from a given IGS model and serial.
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
   Antenna AntexReader::getAntenna( const string& model,
                                    const string& serial )
      throw(ObjectNotFound)
   {

         // Flag that signals if we found the antenna
      bool antennaFound(false);

         // Create 'Antenna' object to be returned
      Antenna antenna;


         // We need to read the data stream (file) from the beginning
      FFTextStream::open( fileName.c_str(), std::ios::in );

         // Change input to upper case and strip leading and trailing spaces
      string uModel( strip( upperCase( model.substr(0,15) ) ) );

         // Check if we have radome information here
      string uRadome;
      if( model.size() >= 17 )
      {
         uRadome = strip( upperCase( model.substr(16,4) ) );
      }

         // Get serial
      string uSerial( strip( upperCase( serial ) ) );

         // Getting antennas out of Antex file is a costly process, so this
         // object will keep a "buffer" called 'antennaMap' where all antennas
         // previously looked up are stored.
         // Then, let's look first into this "buffer"
      AntennaDataMap::const_iterator it1( antennaMap.find(uModel) );
      if( it1 != antennaMap.end() )
      {

         RSCalValAntMap::const_iterator it2( (*it1).second.find(uRadome) );

         if( it2 != (*it1).second.end() )
         {

            SerCalValAntMap::const_iterator it3( (*it2).second.find(uSerial) );

            if( it3 != (*it2).second.end() )
            {

                  // Return the first antenna found. Note that there several
                  // different maps in cascade.
               CalValAntMap::const_iterator it4( (*it3).second.begin() );
               ValAntMap::const_iterator it5( (*it4).second.begin() );

                  // we found the antenna
               antenna = (*it5).second;
               antennaFound = true;

            }  // End of 'if( it3 != (*it2).second.end() )...'

         }  // End of 'if( it2 != (*it1).second.end() )...'

      }  // End of 'if( it1 != antennaMap.end() )'


         // Antenna is not in 'antennaMap': Let's look for it in file
      if( !antennaFound )
      {

            // Antenna is not in 'antennaMap': Let's look for it in file
         try
         {

               // Repeat until antenna is found or End Of File
            while( !antennaFound )
            {

               std::string label;
               std::string line;

                  // Look for 'typeSerial' line
               while( label != typeSerial )
               {
                     // Read one line from file
                  formattedGetLine(line, true);

                     // Get label
                  label = strip( line.substr(60,20) );
               }

                  // Check if model matches. Read only model, not radome
               if( uModel == strip( line.substr(0,15) ) )
               {

                     // Check if radome matches
                  if( uRadome == strip( line.substr(16,4) ) )
                  {

                        // Check if serial matches
                     if( uSerial == strip( line.substr(20,20) ) )
                     {

                           // We found the antenna. Fill it with data
                        antenna = fillAntennaData( line );

                           // Insert antenna into buffer 'antennaMap'
                        antennaMap[ antenna.getAntennaType()      ]
                                  [ antenna.getAntennaRadome()    ]
                                  [ antenna.getAntennaSerial()    ]
                                  [ antenna.getAntennaCalMethod() ]
                                  [ antenna.getAntennaValidFrom() ] = antenna;

                        antennaFound = true;

                     }  // End of 'if( uSerial == strip( line.substr(20,20) ) )'

                  }  // End of 'if( uModel == strip( line.substr(0,15) ) )...'

               } // End of 'if( uModel == strip( line.substr(0,15) ) )...'

            }  // End of 'while( !antennaFound )...'

         }  // End of try block
         catch( InvalidAntex& ia )
         {

               // We need to close this data stream
            (*this).close();

            GPSTK_RETHROW(ia);
         }
         catch( EndOfFile& e )
         {
               // We need to close this data stream
            (*this).close();

            ObjectNotFound notFound("Antenna not found in Antex file.");
            GPSTK_THROW(notFound);
         }
         catch(...)
         {
               // We need to close this data stream
            (*this).close();

            InvalidAntex ia("Unknown error when reading Antex header.");
            GPSTK_THROW(ia);
         }

      }  // End of 'if( !antennaFound )...'


         // We need to close this data stream
      (*this).close();

         // Return antenna
      return antenna;

   }  // End of method 'AntexReader::getAntenna()'



      /* Method to get antenna data from a given IGS model and serial, and
       * for a specific epoch.
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
   Antenna AntexReader::getAntenna( const string& model,
                                    const string& serial,
                                    const CommonTime& epoch )
      throw(ObjectNotFound)
   {

         // Flag that signals if we found the antenna
      bool antennaFound(false);

         // Create 'Antenna' object to be returned
      Antenna antenna;


         // We need to read the data stream (file) from the beginning
      FFTextStream::open( fileName.c_str(), std::ios::in );

         // Change input to upper case and strip leading and trailing spaces
      const string uModel( strip( upperCase( model.substr(0,15) ) ) );

         // Check if we have radome information here
      string uRadome;
      if( model.size() >= 17 )
      {
         uRadome = strip( upperCase( model.substr(16,4) ) );
      }

      const string uSerial( strip( upperCase( serial ) ) );

         // Getting antennas out of Antex file is a costly process, so this
         // object will keep a "buffer" called 'antennaMap' where all antennas
         // previously looked up are stored.
         // Then, let's look first into this "buffer"
      AntennaDataMap::const_iterator it1( antennaMap.find(uModel) );
      if( it1 != antennaMap.end() )
      {

         RSCalValAntMap::const_iterator it2( (*it1).second.find(uRadome) );

         if( it2 != (*it1).second.end() )
         {

            SerCalValAntMap::const_iterator it3( (*it2).second.find(uSerial) );

            if( it3 != (*it2).second.end() )
            {

                  // Return the first antenna found with this model, radome and
                  // serial, and start looking from there. Note that
                  // calibration value is not taken into account here.
               CalValAntMap::const_iterator it4( (*it3).second.begin() );

               while( !antennaFound &&
                      it4 != (*it3).second.end() )
               {

                     // Let's define a reverse iterator
                  ValAntMap::const_reverse_iterator it5(
                                                      (*it4).second.rbegin() );

                  while( !antennaFound &&
                         it5 != (*it4).second.rend() )
                  {

                        // We found the antenna if 'epoch' is between
                        // "Valid From" and "Valid Until" fields
                     if( epoch >= (*it5).first &&
                         epoch <= (*it5).second.getAntennaValidUntil() )
                     {
                           // We found the antenna
                        antenna = (*it5).second;
                        antennaFound = true;
                     }

                     ++it5;

                  }  // End of 'while( !antennaFound && it5 != ...'

                  ++it4;

               }  // End of 'while( !antennaFound && it4 != ...'


            }  // End of 'if( it3 != (*it2).second.end() )...'

         }  // End of 'if( it2 != (*it1).second.end() )...'

      }  // End of 'if( it1 != antennaMap.end() )...'


         // Antenna is not in 'antennaMap': Let's look for it in file
      if( !antennaFound )
      {

            // Antenna is not in 'antennaMap': Let's look for it in file
         try
         {

               // Repeat until antenna is found or End Of File
            while( !antennaFound )
            {

               std::string label;
               std::string line;

                  // Look for 'typeSerial' line
               while( label != typeSerial )
               {
                     // Read one line from file
                  formattedGetLine(line, true);

                     // Get label
                  label = strip( line.substr(60,20) );
               }

                  // Check if model matches. Read only model, not radome
               if( uModel == strip( line.substr(0,15) ) )
               {

                     // Check if radome matches
                  if( uRadome == strip( line.substr(16,4) ) )
                  {

                        // Check if serial matches
                     if( uSerial == strip( line.substr(20,20) ) )
                     {

                           // Read the antenna.
                        antenna = fillAntennaData( line );

                           // Check if this antenna is valid at 'epoch'
                        if( epoch >= antenna.getAntennaValidFrom() &&
                            epoch <= antenna.getAntennaValidUntil() )
                        {

                              // We found it. Insert antenna into buffer
                              // 'antennaMap'
                           antennaMap[ antenna.getAntennaType()   ]
                                  [ antenna.getAntennaRadome()    ]
                                  [ antenna.getAntennaSerial()    ]
                                  [ antenna.getAntennaCalMethod() ]
                                  [ antenna.getAntennaValidFrom() ] = antenna;

                           antennaFound = true;
                        }

                     }  // End of 'if( uSerial == strip( line.substr(20,20) ) )'

                  }  // End of 'if( uModel == strip( line.substr(0,15) ) )...'

               } // End of 'if( uModel == strip( line.substr(0,15) ) )...'

            }  // End of 'while( !antennaFound )...'

         }  // End of try block
         catch( InvalidAntex& ia )
         {

               // We need to close this data stream
            (*this).close();

            GPSTK_RETHROW(ia);
         }
         catch( EndOfFile& e )
         {
               // We need to close this data stream
            (*this).close();

            ObjectNotFound notFound("Antenna not found in Antex file.");
            GPSTK_THROW(notFound);
         }
         catch(...)
         {
               // We need to close this data stream
            (*this).close();

            InvalidAntex ia("Unknown error when reading Antex header.");
            GPSTK_THROW(ia);
         }

      }  // End of 'if( !antennaFound )...'


         // We need to close this data stream
      (*this).close();

         // Return antenna
      return antenna;

   }  // End of method 'AntexReader::getAntenna()'



      /* Method to get antenna data from a given serial and a specific
       * epoch.
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
   Antenna AntexReader::getAntenna( const string& serial,
                                    const CommonTime& epoch )
      throw(ObjectNotFound)
   {

         // Flag that signals if we found the antenna
      bool antennaFound(false);

         // Create 'Antenna' object to be returned
      Antenna antenna;


         // We need to read the data stream (file) from the beginning
      FFTextStream::open( fileName.c_str(), std::ios::in );

      const string uSerial( strip( upperCase( serial ) ) );

         // Getting antennas out of Antex file is a costly process, so this
         // object will keep a "buffer" called 'antennaMap' where all antennas
         // previously looked up are stored.
         // Then, let's look first into this "buffer"
      AntennaDataMap::const_iterator it1( antennaMap.begin() );
      while( !antennaFound &&
             it1 != antennaMap.end() )
      {

         RSCalValAntMap::const_iterator it2( (*it1).second.begin() );

         while( !antennaFound &&
                it2 != (*it1).second.end() )
         {

               // Look for the serial
            SerCalValAntMap::const_iterator it3( (*it2).second.find(uSerial) );

            if( it3 != (*it2).second.end() )
            {

                  // Return the first antenna found with this model, radome and
                  // serial, and start looking from there. Note that
                  // calibration value is not taken into account here.
               CalValAntMap::const_iterator it4( (*it3).second.begin() );

               while( !antennaFound &&
                      it4 != (*it3).second.end() )
               {

                     // Let's define a reverse iterator
                  ValAntMap::const_reverse_iterator it5(
                                                      (*it4).second.rbegin() );

                  while( !antennaFound &&
                         it5 != (*it4).second.rend() )
                  {

                        // We found the antenna if 'epoch' is between
                        // "Valid From" and "Valid Until" fields
                     if( epoch >= (*it5).first &&
                         epoch <= (*it5).second.getAntennaValidUntil() )
                     {
                           // We found the antenna
                        antenna = (*it5).second;
                        antennaFound = true;
                     }

                     ++it5;

                  }  // End of 'while( !antennaFound && it5 != ...'

                  ++it4;

               }  // End of 'while( !antennaFound && it4 != ...'

            }  // End of 'if( it3 != (*it2).second.end() )...'

            ++it2;

         }  // End of 'while( !antennaFound && it2 != ...'

         ++it1;

      }  // End of 'while( !antennaFound && it1 != antennaMap.end() )...'


         // Antenna is not in 'antennaMap': Let's look for it in file
      if( !antennaFound )
      {

            // Antenna is not in 'antennaMap': Let's look for it in file
         try
         {

               // Repeat until antenna is found or End Of File
            while( !antennaFound )
            {

               std::string label;
               std::string line;

                  // Look for 'typeSerial' line
               while( label != typeSerial )
               {
                     // Read one line from file
                  formattedGetLine(line, true);

                     // Get label
                  label = strip( line.substr(60,20) );
               }

                  // Check if serial matches
               if( uSerial == strip( line.substr(20,20) ) )
               {

                     // Read the antenna.
                  antenna = fillAntennaData( line );

                     // Check if this antenna is valid at 'epoch'
                  if( epoch >= antenna.getAntennaValidFrom() &&
                      epoch <= antenna.getAntennaValidUntil() )
                  {

                        // We found it. Insert antenna into buffer 'antennaMap'
                     antennaMap[ antenna.getAntennaType()      ]
                               [ antenna.getAntennaRadome()    ]
                               [ antenna.getAntennaSerial()    ]
                               [ antenna.getAntennaCalMethod() ]
                               [ antenna.getAntennaValidFrom() ] = antenna;

                     antennaFound = true;

                  }

               }  // End of 'if( uSerial == strip( line.substr(20,20) ) )'

            }  // End of 'while( !antennaFound )...'

         }  // End of try block
         catch( InvalidAntex& ia )
         {

               // We need to close this data stream
            (*this).close();

            GPSTK_RETHROW(ia);
         }
         catch( EndOfFile& e )
         {
               // We need to close this data stream
            (*this).close();

            ObjectNotFound notFound("Antenna not found in Antex file.");
            GPSTK_THROW(notFound);
         }
         catch(...)
         {
               // We need to close this data stream
            (*this).close();

            InvalidAntex ia("Unknown error when reading Antex header.");
            GPSTK_THROW(ia);
         }

      }  // End of 'if( !antennaFound )...'


         // We need to close this data stream
      (*this).close();

         // Return antenna
      return antenna;

   }  // End of method 'AntexReader::getAntenna()'



      // Method to open and load Antex file header data.
   void AntexReader::open(const char* fn)
   {

      fileName = fn;

         // We need to be sure current data stream is closed
      (*this).close();

         // Open data stream
      FFTextStream::open(fn, std::ios::in);

         // We must be sure that previous antenna data is cleared.
      antennaMap.clear();
      version = 0.0;
      refAntena = "";
      refAntenaSerial = "";
      commentList.clear();
      valid = false;

         // Load header of Antex File
      loadHeader();

      return;

   }  // End of method 'AntexReader::open()'



      // Method to open and load Antex file header data.
   void AntexReader::open(const string& fn)
   {

      fileName = fn;

         // We need to be sure current data stream is closed
      (*this).close();

         // Open data stream
      FFTextStream::open(fn.c_str(), std::ios::in);

         // We must be sure that previous antenna data is cleared.
      antennaMap.clear();
      version = 0.0;
      refAntena = "";
      refAntenaSerial = "";
      commentList.clear();
      valid = false;

         // Load header of Antex File
      loadHeader();

      return;

   }  // End of method 'AntexReader::open()'



      // Returns if loaded antenna data file is absolute or relative.
   bool AntexReader::isAbsolute() const
   {
      if( type == absolute )
      {
         return true;
      }
      else
      {
         return false;
      }

   }  // End of method 'AntexReader::isAbsolute()'



      // This methods dumps all data in Antex header.
   void AntexReader::dump(ostream& s) const
   {

         // Print version
      s << "Antex Version " << version << endl;

         // Print satellite system
      s << "Satellite system: ";
      switch (system)
      {
         case SatID::systemGPS:
            s << "GPS";
            break;
         case SatID::systemGlonass:
            s << "GLONASS";
            break;
         case SatID::systemGalileo:
            s << "Galileo";
            break;
         case SatID::systemMixed:
            s << "Mixed";
            break;
		 default: break; //NB Determine if additional enumeration values need to be handled
      }
      s << endl;

         // Print PCV type and, if relative, reference antenna type and serial
      s << "PCV type: ";
      if( type == absolute )
      {
         s << "Absolute" << endl;
      }
      else
      {
         if( type == relative )
         {
            s << "Relative" << endl;
            s << "    Reference antenna type: " << refAntena << ", "
              << "Serial number: " << refAntenaSerial << endl;
         }
         else
         {
            s << "Unknown" << endl;
         }
      }
      s << endl;

         // Print comments
      s << "*** START OF COMMENTS ***" << endl;
      for(size_t i = 0; i < commentList.size(); ++i)
      {
         s << commentList[i] << endl;
      }
      s << "*** END OF COMMENTS ***" << endl << endl;

      s << "This data is ";
      if( isValid() )
      {
         s << "VALID";
      }
      else
      {
         s << "NOT VALID";
      }
      s << endl;

   }  // End of method 'AntexReader::dump()'


}  // End of namespace gpstk
