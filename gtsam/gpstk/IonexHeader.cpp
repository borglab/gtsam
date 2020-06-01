#pragma ident "$Id$"

/**
 * @file IonexHeader.cpp
 * This class encapsulates the header of Ionex file, including I/O
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
//  Octavian Andrei - FGI ( http://www.fgi.fi ). 2008
//
//============================================================================


#include <cctype>

#include "StringUtils.hpp"
#include "MathBase.hpp"
#include "IonexHeader.hpp"
#include "IonexStream.hpp"
#include "CivilTime.hpp"

using namespace std;
using namespace gpstk::StringUtils;

namespace gpstk
{
   const string IonexHeader::versionString         =  "IONEX VERSION / TYPE";
   const string IonexHeader::runByString           =  "PGM / RUN BY / DATE";
   const string IonexHeader::descriptionString     =  "DESCRIPTION";
   const string IonexHeader::commentString         =  "COMMENT";
   const string IonexHeader::firstTimeString       =  "EPOCH OF FIRST MAP";
   const string IonexHeader::lastTimeString        =  "EPOCH OF LAST MAP";
   const string IonexHeader::intervalString        =  "INTERVAL";
   const string IonexHeader::numMapsString         =  "# OF MAPS IN FILE";
   const string IonexHeader::mappingFunctionString =  "MAPPING FUNCTION";
   const string IonexHeader::elevationString       =  "ELEVATION CUTOFF";
   const string IonexHeader::observablesUsedString =  "OBSERVABLES USED";
   const string IonexHeader::numStationsString     =  "# OF STATIONS";
   const string IonexHeader::numSatsString         =  "# OF SATELLITES";
   const string IonexHeader::baseRadiusString      =  "BASE RADIUS";
   const string IonexHeader::mapDimensionString    =  "MAP DIMENSION";
   const string IonexHeader::hgtGridString         =  "HGT1 / HGT2 / DHGT";
   const string IonexHeader::latGridString         =  "LAT1 / LAT2 / DLAT";
   const string IonexHeader::lonGridString         =  "LON1 / LON2 / DLON";
   const string IonexHeader::exponentString        =  "EXPONENT";
   const string IonexHeader::startAuxDataString    =  "START OF AUX DATA";
   const string IonexHeader::endAuxDataString      =  "END OF AUX DATA";
   const string IonexHeader::endOfHeader           =  "END OF HEADER";

   const string IonexHeader::DCB::svsAuxDataString       =  "PRN / BIAS / RMS";
   const string IonexHeader::DCB::stationsAuxDataString  =
                                                         "STATION / BIAS / RMS";


      // Clear (empty out) header
   void IonexHeader::clear(void)
   {

      version = 1.0;
      descriptionList.clear();
      commentList.clear();
      interval = 0;
      numMaps = numStations = numSVs = mapDims = 0;
      elevation = baseRadius = 0;

      hgt[0] = hgt[1] = hgt[2] = 0.0;
      lat[0] = lat[1] = lat[2] = 0.0;
      lon[0] = lon[1] = lon[2] = 0.0;

      exponent = -1;    // that's the default value
      svsmap.clear();
      valid = auxDataFlag = false;

      return;

   }  // End of method 'IonexHeader::clear()'



      /* Simple debug output function.
       *
       * It simply outputs the version, name and number of maps contained
       * in this Ionex header.
       */
   void IonexHeader::dump(std::ostream& os) const
   {

      os << "-------------------------------- IONEX HEADER"
         << "--------------------------------" << endl;

      os << "First epoch            : " << firstEpoch << endl;
      os << "Last epoch             : " << lastEpoch << endl; 
      os << "Interval               : " << interval << endl;
      os << "Number of ionex maps   : " << numMaps << endl;
      os << "Mapping function       : " << mappingFunction << endl;
      os << "Elevation cut off      : " << elevation << endl;
      os << "Number of stations     : " << numStations << endl;
      os << "Number of satellites   : " << numSVs << endl;
      os << "Map dimensions         : " << mapDims << endl;

      os << "HGT1 / HGT2 / DHGT     : " << hgt[0] << " / "
                                        << hgt[1] << " / "
                                        << hgt[2] << endl;
      os << "LAT1 / LAT2 / DLAT     : " << lat[0] << " / "
                                        << lat[1] << " / "
                                        << lat[2] << endl;
      os << "LON1 / LON2 / DLON     : " << lon[0] << " / "
                                        << lon[1] << " / "
                                        << lon[2] << endl;
      os << "Valid object?          : " << valid  << endl;

      os << "-------------------------------- END OF HEADER"
         << "-------------------------------" << endl;

      os << endl;

   }  //End of method 'IonexHeader::dump()'



      /*
       * Parse a single auxiliary header record that contains "Differential
       * code biases".
       */
   void IonexHeader::ParseDcbRecord(std::string &line)
      throw (FFStreamError)
   {

      string label(line, 60, 20);

      if (label == DCB::svsAuxDataString)
      {
            // prepare the DCB structure
         char c = isspace(line[3]) ? 'G' : line[3];
         int prn     = asInt(line.substr(4,2));
         double bias = asDouble(line.substr(6,16));// * 1e-9; // change to seconds
         double rms  = asDouble(line.substr(16,26));

            // prepare SatID object that is the key of the map
         SatID::SatelliteSystem system;
         switch(line[3])
         {

            case ' ': case 'G': case 'g':
               system = SatID::systemGPS;
               break;

            case 'R': case 'r':
               system = SatID::systemGlonass;
               break;

            default:                   // non-IONEX system character
               FFStreamError e(std::string("Invalid system character \"")
                               + c + std::string("\""));
               GPSTK_THROW(e);

         }  // End of 'switch(line[3])'

         SatID svid = SatID(prn,system);

            // add to map
         svsmap[svid] = DCB(c,prn,bias,rms);

      }  // End of 'if (label == DCB::svsAuxDataString)'...
      else if (label == DCB::stationsAuxDataString)
      {

         // WARNING: at this stage the DCB values for the contributing
         // stations are not mapped.

      }
      else if (label == commentString)
      {

            // CODE's product has a comment line before aux data end
         string s = strip(line.substr(0,60));
         commentList.push_back(s);

      }
      else if (label == endAuxDataString)
      {

         auxDataFlag = false;          // End of aux data

      }
      else
      {

         FFStreamError e(std::string( "Unidentified IONEX::DCB label: "
                                      + label) );

         GPSTK_THROW(e);

      }  // End of 'if (label == endAuxDataString)'...

      return;

   }  // End of method 'IonexHeader::ParseDcbRecord()'



      /* Parse a single header record, and modify 'valid' accordingly.
       *
       * Used by reallyGetRecord for both IonexHeader and IonexData.
       */
   void IonexHeader::ParseHeaderRecord(std::string &line)
      throw (FFStreamError)
   {

      string label(line, 60, 20);

      if (label == versionString)
      {

         version  = asDouble(line.substr(0,20));
         fileType = strip(line.substr(20,20));
         system   = strip(line.substr(40,20));

      }
      else if (label == runByString)
      {

         fileProgram = strip(line.substr( 0,20));
         fileAgency  = strip(line.substr(20,20));
         date        = strip(line.substr(40,20));

      }
      else if (label == descriptionString)
      {

         string s = line.substr(0,60);
         descriptionList.push_back(s);

      }
      else if (label == commentString)
      {

         string s = line.substr(0,60);
         commentList.push_back(s);

      }
      else if (label == firstTimeString)
      {

         firstEpoch = parseTime(line);

      }
      else if (label == lastTimeString)
      {

         lastEpoch = parseTime(line);

      }
      else if (label == intervalString)
      {

         interval = asInt(line.substr(0,6));

      }
      else if (label == numMapsString)
      {

         numMaps = asInt(line.substr(0,6));

      }
      else if (label == mappingFunctionString)
      {

         mappingFunction = strip(line.substr(0, 6));

      }
      else if (label == elevationString)
      {

         elevation = asDouble(line.substr(0, 8));

      }
      else if (label == observablesUsedString)
      {

         observablesUsed = strip(line.substr(0,60));

      }
      else if (label == numStationsString)
      {

         numStations = asInt(line.substr(0,6));

      }
      else if (label == numSatsString)
      {

         numSVs = asInt(line.substr(0,6));

      }
      else if (label == baseRadiusString)
      {

         baseRadius = asDouble(line.substr(0, 8));

      }
      else if (label == mapDimensionString)
      {

         mapDims = asInt(line.substr(0,6));

      }
      else if (label == hgtGridString)
      {

         hgt[0] = asDouble(line.substr( 2, 6));
         hgt[1] = asDouble(line.substr( 8, 6));
         hgt[2] = asDouble(line.substr(14, 6));

      }
      else if (label == latGridString)
      {

         lat[0] = asDouble(line.substr( 2, 6));
         lat[1] = asDouble(line.substr( 8, 6));
         lat[2] = asDouble(line.substr(14, 6));

      }
      else if (label == lonGridString)
      {

         lon[0] = asDouble(line.substr( 2, 6));
         lon[1] = asDouble(line.substr( 8, 6));
         lon[2] = asDouble(line.substr(14, 6));

      }
      else if (label == exponentString)
      {

         exponent = asInt(line.substr(0,6));

      }
      else if (label == startAuxDataString)
      {

         auxData = strip(line.substr(0,60));
         auxDataFlag = true;

      }
      else if (label == endOfHeader)
      {

         auxDataFlag = true;
         valid = true;

      }
      else
      {

         FFStreamError e("Unidentified IONEX header record: " + label);

         GPSTK_THROW(e);

      }

      return;

   }  // End of method 'IonexHeader::ParseHeaderRecord()'



      // This function parses the entire header from the given stream
   void IonexHeader::reallyGetRecord(FFStream& ffs)
      throw(exception, FFStreamError, StringException)
   {

      IonexStream& strm = dynamic_cast<IonexStream&> (ffs);

         // if already read, just return
      if (strm.headerRead == true)
      {
         return;
      }

         // since we read a new header, we need to reinitialize
         // all our list structures. All the other objects should be ok.
         // This also applies if we threw an exception the first time we read
         // the header and are now re-reading it. Some of these data
         // structures could be full and we need to empty them.
      clear();

      string line;

      while (!valid)
      {

         strm.formattedGetLine(line);
         StringUtils::stripTrailing(line);

            // skip empty lines
         if (line.length() == 0)
         {
            continue;
         }
         else
         {

            if (line.length() < 60 || line.length() > 80)
            {

               FFStreamError e("Invalid line length");
               GPSTK_THROW(e);

            }

         }  // End of 'if (line.length() == 0)...'


         if (auxDataFlag)     // when it is set true, then parse auxiliar data
         {

            try
            {
               ParseDcbRecord(line);
            }
            catch (FFStreamError& e)
            {
               GPSTK_RETHROW(e);
            }

         }
         else
         {

            try
            {
               ParseHeaderRecord(line);
            }
            catch (FFStreamError& e)
            {
               GPSTK_RETHROW(e);
            }

         }  // End of 'if (auxDataFlag)...'

      }  // End of 'while (!valid)...' (not for the header)


         // Here come some validity checkings
         // Checking ionex version
      if (version != 1.0)
      {
         FFStreamError e( "Invalid IONEX version number " +
                          asString(version));
         GPSTK_THROW(e);
      }

         // time arguments consistency
      double interval0( (lastEpoch - firstEpoch) / (numMaps -1.0) );
      if (interval != static_cast<int>(interval0))
      {
         FFStreamError e("Inconsistent time arguments.");
         GPSTK_THROW(e);
      }

         // map dimension consistency
      if (mapDims == 2)
      {

         if ( (hgt[0] != hgt[1]) || (hgt[2] != 0.0) )
         {
            FFStreamError e("Error concerning map dimension.");
            GPSTK_THROW(e);
         }

      }
      else
      {

         if ( (hgt[0] == hgt[1]) || (hgt[2] == 0.0) )
         {
            FFStreamError e("Error concerning map dimension.");
            GPSTK_THROW(e);
         }

      }  // End of 'if (mapDims == 2)...'

         // grid checkings
      double grdfac[4];
      try
      {
         grdfac[0] = lat[0]/lat[2];
         grdfac[1] = lat[1]/lat[2];
         grdfac[2] = lon[0]/lon[2];
         grdfac[3] = lon[1]/lon[2];
      }
      catch(exception& e)
      {
         cerr << "Problems computing grdfac: " << e.what() << endl;
         throw;
      }

      for (int i = 0; i < 4; i++)
      {
         double xdif1( grdfac[i] - static_cast<int>(grdfac[i]) );
         double xdif( ABS(grdfac[i] - static_cast<int>(grdfac[i])) );
#pragma unused(xdif1)
          
         if (xdif > 1e-4)
         {
            FFStreamError e("Irregular Ionex data grid.");
            GPSTK_THROW(e);
         }

      }  // End of 'for (int i = 0; i < 4; i++)...'

         // reach end of header line
      strm.header = *this;
      strm.headerRead = true;

      return;

   }  // End of method 'IonexHeader::reallyGetRecord()'



   void IonexHeader::reallyPutRecord(FFStream& ffs) const
      throw(exception, FFStreamError, StringException)
   {

      IonexStream& strm = dynamic_cast<IonexStream&>(ffs);

      if (version != 1.0)
      {
         FFStreamError err( "Unknown IONEX version: " + asString(version,2) );
         err.addText("Make sure to set the version correctly.");
         GPSTK_THROW(err);
      }

      try
      {
         WriteHeaderRecords(strm);
      }
      catch(FFStreamError& e)
      {
         GPSTK_RETHROW(e);
      }
      catch(StringException& e)
      {
         GPSTK_RETHROW(e);
      }

   }  // End of method 'IonexHeader::reallyPutRecord()'


      // this function writes all valid header records
   void IonexHeader::WriteHeaderRecords(FFStream& ffs) const
      throw(FFStreamError, StringException)
   {
      IonexStream& strm = dynamic_cast<IonexStream&>(ffs);
      string line;

      if (valid)
      {

            // write first IONEX record
         line.clear();
         line  = rightJustify(asString(version,1), 8);
         line += string(12, ' ');
         if ((fileType[0] != 'I') && (fileType[0] != 'i'))
         {
            FFStreamError err("This isn't a Ionex file: " + 
                              fileType.substr(0,1));
            GPSTK_THROW(err);
         }

         line += leftJustify(fileType, 20);
         line += leftJustify(system, 20);
         line += leftJustify(versionString,20);
         strm << line << endl;
         strm.lineNumber++;


            // write second IONEX record
         line.clear();
         line += leftJustify(fileProgram,20);
         line += leftJustify(fileAgency,20);
         line += leftJustify(date,20);
         line += leftJustify(runByString,20);
         strm << line << endl;
         strm.lineNumber++;


            // write title (optional)
         if( commentList.size() > 0 )
         {
            line.clear();
            line += leftJustify(commentList[0],60);
            line += leftJustify(commentString,20);
            strm << line << endl;
            strm.lineNumber++;
         }


            // write multi-line description (optional)
         if (descriptionList.size() > 0)
         {

            vector<std::string>::size_type i = 0;

            for( ; i < descriptionList.size(); i++)
            {

               line.clear();
               line += leftJustify(descriptionList[i],60);
               line += leftJustify(descriptionString,20);
               strm << line << endl;
               strm.lineNumber++;

            }

         }  // End of 'if (descriptionList.size() > 0) ...'


            // write epoch of first epoch
         line.clear();
         line += writeTime(firstEpoch);
         line += string(24, ' ');
         line += leftJustify(firstTimeString,20);
         strm << line << endl;
         strm.lineNumber++;


            // write epoch of last epoch
         line.clear();
         line += writeTime(lastEpoch);
         line += string(24, ' ');
         line += leftJustify(lastTimeString,20);
         strm << line << endl;
         strm.lineNumber++;


            // write interval
         line.clear();
         line += rightJustify( asString(interval), 6 );
         line += string(54, ' ');
         line += leftJustify(intervalString,20);
         strm << line << endl;
         strm.lineNumber++;


            // write # of maps
         line.clear();
         line += rightJustify( asString<short>(numMaps), 6 );
         line += string(54, ' ');
         line += leftJustify(numMapsString,20);
         strm << line << endl;
         strm.lineNumber++;


            // write mapping function
         line.clear();
         line += string(2, ' ');
         line += rightJustify(mappingFunction, 4);
         line += string(54, ' ');
         line += leftJustify(mappingFunctionString,20);
         strm << line << endl;
         strm.lineNumber++;


            // write elevation cutoff
         line.clear();
         line += rightJustify( asString(elevation,1), 8 );
         line += string(52, ' ');
         line += leftJustify(elevationString,20);
         strm << line << endl;
         strm.lineNumber++;


            // write observables used
         line.clear();
         line += leftJustify(observablesUsed,60);
         line += leftJustify(observablesUsedString,20);
         strm << line << endl;
         strm.lineNumber++;


            // write # of stations (optional)
         if (numStations > 0)
         {
            line.clear();
            line += rightJustify( asString<short>(numStations), 6 );
            line += string(54, ' ');
            line += leftJustify(numStationsString,20);
            strm << line << endl;
            strm.lineNumber++;
         }


            // write # of satellites (optional)
         if (numSVs > 0)
         {
            line.clear();
            line += rightJustify( asString<short>(numSVs), 6 );
            line += string(54, ' ');
            line += leftJustify(numSatsString,20);
            strm << line << endl;
            strm.lineNumber++;
         }


            // write base radius
         line.clear();
         line += rightJustify( asString(baseRadius,1), 8 );
         line += string(52, ' ');
         line += leftJustify(baseRadiusString,20);
         strm << line << endl;
         strm.lineNumber++;


            // write map dimension
         line.clear();
         line += rightJustify( asString(mapDims), 6 );
         line += string(54, ' ');
         line += leftJustify(mapDimensionString,20);
         strm << line << endl;
         strm.lineNumber++;


            // write grid specifications
         line.clear();
         line += string(2, ' ');
         line += rightJustify( asString(hgt[0],1), 6 );
         line += rightJustify( asString(hgt[1],1), 6 );
         line += rightJustify( asString(hgt[2],1), 6 );
         line += string(40, ' ');
         line += leftJustify(hgtGridString,20);
         strm << line << endl;
         strm.lineNumber++;

         line.clear();
         line += string(2, ' ');
         line += rightJustify( asString(lat[0],1), 6 );
         line += rightJustify( asString(lat[1],1), 6 );
         line += rightJustify( asString(lat[2],1), 6 );
         line += string(40, ' ');
         line += leftJustify(latGridString,20);
         strm << line << endl;
         strm.lineNumber++;

         line.clear();
         line += string(2, ' ');
         line += rightJustify( asString(lon[0],1), 6 );
         line += rightJustify( asString(lon[1],1), 6 );
         line += rightJustify( asString(lon[2],1), 6 );
         line += string(40, ' ');
         line += leftJustify(lonGridString,20);
         strm << line << endl;
         strm.lineNumber++;


            // write default exponent (optional)
         line.clear();
         line += rightJustify( asString(exponent), 6 );
         line += string(54, ' ');
         line += leftJustify(exponentString,20);
         strm << line << endl;
         strm.lineNumber++;


            // write multi-line comment
         for( vector<std::string>::size_type i = 1; 
              i < commentList.size(); i++)
         {
            line.clear();
            line += leftJustify(commentList[i],60);
            line += leftJustify(commentString,20);
            strm << line << endl;
            strm.lineNumber++;
         }


            // write auxiliary data (optional)
         if (auxDataFlag)
         {

               // start of aux data
            line.clear();
            line += leftJustify(auxData,60);
            line += leftJustify(startAuxDataString,20);
            strm << line << endl;
            strm.lineNumber++;

            IonexHeader::SatDCBMap::const_iterator isv = svsmap.begin();

            for(; isv != svsmap.end(); isv++)
            {
               line.clear();
               line += isv->second.toString();
               line += string(34, ' ');
               line += leftJustify(DCB::svsAuxDataString,20);
               strm << line << endl;
               strm.lineNumber++;
            }

               // end of aux data
            line.clear();
            line += leftJustify(auxData,60);
            line += leftJustify(endAuxDataString,20);
            strm << line << endl;
            strm.lineNumber++;

         }  // End of 'if (auxDataFlag)...'


            // write record closing Ionex header
         line.clear();
         line += string(60, ' ');
         line += leftJustify(endOfHeader,20);
         strm << line << endl;
         strm.lineNumber++;

      }  // End of 'if (valid)...'
   }


      /* This function sets the time for this header.
       *
       * It looks at \a line to obtain the needed information.
       */
   CommonTime IonexHeader::parseTime(const string& line) const
   {

      int year, month, day, hour, min, sec;

      year  = asInt(line.substr( 0,6));
      month = asInt(line.substr( 6,6));
      day   = asInt(line.substr(12,6));
      hour  = asInt(line.substr(18,6));
      min   = asInt(line.substr(24,6));
      sec   = asInt(line.substr(30,6));

      return CivilTime(year, month, day, hour, min, (double)sec);

   }  // End of method 'IonexHeader::parseTime()'


         /** Converts the CommonTime \a dt into a Ionex Obs time
          * string for the header
          */
   string IonexHeader::writeTime(const CommonTime& dt) const
   {

      string line;

      line  = rightJustify(asString<short>(static_cast<CivilTime>(dt).year), 6);
      line += rightJustify(asString<short>(static_cast<CivilTime>(dt).month), 6);
      line += rightJustify(asString<short>(static_cast<CivilTime>(dt).day), 6);
      line += rightJustify(asString<short>(static_cast<CivilTime>(dt).hour), 6);
      line += rightJustify(asString<short>(static_cast<CivilTime>(dt).minute), 6);
      line += rightJustify(asString (static_cast<int>(static_cast<CivilTime>(dt).second)), 6);

      return line;

   }  // End of method 'IonexHeader::writeTime()'



}  // End of namespace gpstk
