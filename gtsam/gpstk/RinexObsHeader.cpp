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
 * @file RinexObsHeader.cpp
 * Encapsulate header of Rinex observation file, including I/O
 */

#include "StringUtils.hpp"
#include "RinexObsHeader.hpp"
#include "RinexObsStream.hpp"
#include "CivilTime.hpp"
#include "SystemTime.hpp"

using namespace std;
using namespace gpstk::StringUtils;

namespace gpstk
{
   const string RinexObsHeader::versionString =         "RINEX VERSION / TYPE";
   const string RinexObsHeader::runByString =           "PGM / RUN BY / DATE";
   const string RinexObsHeader::commentString =         "COMMENT";
   const string RinexObsHeader::markerNameString =      "MARKER NAME";
   const string RinexObsHeader::markerNumberString =    "MARKER NUMBER";
   const string RinexObsHeader::observerString =        "OBSERVER / AGENCY";
   const string RinexObsHeader::receiverString =        "REC # / TYPE / VERS";
   const string RinexObsHeader::antennaTypeString =     "ANT # / TYPE";
   const string RinexObsHeader::antennaPositionString = "APPROX POSITION XYZ";
   const string RinexObsHeader::antennaOffsetString =   "ANTENNA: DELTA H/E/N";
   const string RinexObsHeader::waveFactString =        "WAVELENGTH FACT L1/2";
   const string RinexObsHeader::numObsString =          "# / TYPES OF OBSERV";
   const string RinexObsHeader::intervalString =        "INTERVAL";
   const string RinexObsHeader::firstTimeString =       "TIME OF FIRST OBS";
   const string RinexObsHeader::lastTimeString =        "TIME OF LAST OBS";
   const string RinexObsHeader::receiverOffsetString =  "RCV CLOCK OFFS APPL";
   const string RinexObsHeader::leapSecondsString =     "LEAP SECONDS";
   const string RinexObsHeader::numSatsString =         "# OF SATELLITES";
   const string RinexObsHeader::prnObsString =          "PRN / # OF OBS";
   const string RinexObsHeader::endOfHeader =           "END OF HEADER";

   const unsigned int RinexObsType::C1depend=0x01;
   const unsigned int RinexObsType::L1depend=0x02;
   const unsigned int RinexObsType::L2depend=0x04;
   const unsigned int RinexObsType::P1depend=0x08;
   const unsigned int RinexObsType::P2depend=0x10;
   const unsigned int RinexObsType::EPdepend=0x20;
   const unsigned int RinexObsType::PSdepend=0x40;

   const RinexObsType RinexObsHeader::UN("UN", "Unknown or Invalid",   "unknown", 0);
   const RinexObsType RinexObsHeader::L1("L1", "L1 Carrier Phase",     "L1 cycles",
      RinexObsType::L1depend);
   const RinexObsType RinexObsHeader::L2("L2", "L2 Carrier Phase",     "L2 cycles",
      RinexObsType::L2depend);
   const RinexObsType RinexObsHeader::C1("C1", "C/A-code pseudorange", "meters",
      RinexObsType::C1depend);
   const RinexObsType RinexObsHeader::C2("C2", "L2C-code pseudorange", "meters", 0);
   const RinexObsType RinexObsHeader::P1("P1", "Pcode L1 pseudorange", "meters",
      RinexObsType::P1depend);
   const RinexObsType RinexObsHeader::P2("P2", "Pcode L2 pseudorange", "meters",
      RinexObsType::P2depend);
   const RinexObsType RinexObsHeader::D1("D1", "Doppler Frequency L1", "Hz", 0);
   const RinexObsType RinexObsHeader::D2("D2", "Doppler Frequency L2", "Hz", 0);
   const RinexObsType RinexObsHeader::S1("S1", "Signal-to-Noise L1",   "dB-Hz", 0);
   const RinexObsType RinexObsHeader::S2("S2", "Signal-to-Noise L2",   "dB-Hz", 0);
   const RinexObsType RinexObsHeader::T1("T1", "Transit 150 MHz",      "meters", 0);
   const RinexObsType RinexObsHeader::T2("T2", "Transit 400 MHz",      "meters", 0);
   // v 2.11
   const RinexObsType RinexObsHeader::C5("C5", "L5C-code pseudorange", "meters", 0);
   const RinexObsType RinexObsHeader::L5("L5", "L5 Carrier Phase",     "L5 cycles", 0);
   const RinexObsType RinexObsHeader::D5("D5", "Doppler Frequency L5", "Hz", 0);
   const RinexObsType RinexObsHeader::S5("S5", "Signal-to-Noise L5",   "dB-Hz", 0);
   // Galileo only
   const RinexObsType RinexObsHeader::C6("C6", "E6-code pseudorange",  "meters", 0);
   const RinexObsType RinexObsHeader::L6("L6", "E6 Carrier Phase",     "L6 cycles", 0);
   const RinexObsType RinexObsHeader::D6("D6", "Doppler Frequency E6", "Hz", 0);
   const RinexObsType RinexObsHeader::S6("S6", "Signal-to-Noise E6",   "dB-Hz", 0);

   const RinexObsType RinexObsHeader::C7("C7", "E5b-code pseudorange",  "meters", 0);
   const RinexObsType RinexObsHeader::L7("L7", "E5b Carrier Phase",     "L7 cycles", 0);
   const RinexObsType RinexObsHeader::D7("D7", "Doppler Frequency E5b", "Hz", 0);
   const RinexObsType RinexObsHeader::S7("S7", "Signal-to-Noise E5b",   "dB-Hz", 0);

   const RinexObsType RinexObsHeader::C8("C8", "E5a+b-code pseudorange", "meters", 0);
   const RinexObsType RinexObsHeader::L8("L8", "E5a+b Carrier Phase",    "L8 cycles", 0);
   const RinexObsType RinexObsHeader::D8("D8", "Doppler Frequency E5a+b","Hz", 0);
   const RinexObsType RinexObsHeader::S8("S8", "Signal-to-Noise E5a+b",  "dB-Hz", 0);

   RinexObsType sot[29] =
   {
      RinexObsHeader::UN,
      RinexObsHeader::L1, RinexObsHeader::L2,
      RinexObsHeader::C1, RinexObsHeader::C2,
      RinexObsHeader::P1, RinexObsHeader::P2,
      RinexObsHeader::D1, RinexObsHeader::D2,
      RinexObsHeader::S1, RinexObsHeader::S2,
      RinexObsHeader::T1, RinexObsHeader::T2,
      RinexObsHeader::C5, RinexObsHeader::L5, RinexObsHeader::D5, RinexObsHeader::S5,
      RinexObsHeader::C6, RinexObsHeader::L6, RinexObsHeader::D6, RinexObsHeader::S6,
      RinexObsHeader::C7, RinexObsHeader::L7, RinexObsHeader::D7, RinexObsHeader::S7,
      RinexObsHeader::C8, RinexObsHeader::L8, RinexObsHeader::D8, RinexObsHeader::S8
   };

   // Warning: the size of the above sot array needs to be put
   // in this initializer.
   const std::vector<RinexObsType> RinexObsHeader::StandardRinexObsTypes(sot,sot+29);

   std::vector<RinexObsType> RinexObsHeader::RegisteredRinexObsTypes
      = RinexObsHeader::StandardRinexObsTypes;

   void RinexObsHeader::reallyPutRecord(FFStream& ffs) const
      throw(std::exception, FFStreamError, StringException)
   {
      RinexObsStream& strm = dynamic_cast<RinexObsStream&>(ffs);

      strm.header = *this;

      unsigned long allValid;
      if (version == 2.0)        allValid = allValid20;
      else if (version == 2.1)   allValid = allValid21;
      else if (version == 2.11)  allValid = allValid211;
      else
      {
         FFStreamError err("Unknown RINEX version: " + asString(version,2));
         err.addText("Make sure to set the version correctly.");
         GPSTK_THROW(err);
      }

      if ((valid & allValid) != allValid)
      {
         FFStreamError err("Incomplete or invalid header.");
         err.addText("Make sure you set all header valid bits for all of the available data.");
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

   }  // end RinexObsHeader::reallyPutRecord


      // this function computes the number of valid header records which WriteHeaderRecords will write
   int RinexObsHeader::NumberHeaderRecordsToBeWritten(void) const throw()
   {
      int n=0;
      if(valid & RinexObsHeader::versionValid) n++;
      if(valid & RinexObsHeader::runByValid) n++;
      if(valid & RinexObsHeader::markerNameValid) n++;
      if(valid & RinexObsHeader::observerValid) n++;
      if(valid & RinexObsHeader::receiverValid) n++;
      if(valid & RinexObsHeader::antennaTypeValid) n++;
      if(valid & RinexObsHeader::antennaPositionValid) n++;
      if(valid & RinexObsHeader::antennaOffsetValid) n++;
      if(valid & RinexObsHeader::waveFactValid) {
         n++;
         if(extraWaveFactList.size()) n += 1 + (extraWaveFactList.size()-1)/7;
      }
      if(valid & RinexObsHeader::obsTypeValid) n += 1 + (obsTypeList.size()-1)/9;
      if(valid & RinexObsHeader::intervalValid) n++;
      if(valid & RinexObsHeader::firstTimeValid) n++;
      if(valid & RinexObsHeader::lastTimeValid) n++;
      if(valid & RinexObsHeader::markerNumberValid) n++;
      if(valid & RinexObsHeader::receiverOffsetValid) n++;
      if(valid & RinexObsHeader::leapSecondsValid) n++;
      if(valid & RinexObsHeader::commentValid) n += commentList.size();
      if(valid & RinexObsHeader::numSatsValid) n++;
      if(valid & RinexObsHeader::prnObsValid)
         n += numObsForSat.size() * (1+numObsForSat.begin()->second.size()/9);
      if(valid & RinexObsHeader::endValid) n++;
      return n;
   }

      // this function writes all valid header records
   void RinexObsHeader::WriteHeaderRecords(FFStream& ffs) const
      throw(FFStreamError, StringException)
   {
      RinexObsStream& strm = dynamic_cast<RinexObsStream&>(ffs);
      string line;
      if (valid & versionValid)
      {
         line  = rightJustify(asString(version,2), 9);
         line += string(11, ' ');
         if ((fileType[0] != 'O') && (fileType[0] != 'o'))
         {
            FFStreamError err("This isn't a Rinex Observation file: " +
                              fileType.substr(0,1));
            GPSTK_THROW(err);
         }

         if (system.system == RinexSatID::systemUnknown)
         {
            FFStreamError err("Invalid satellite system");
            GPSTK_THROW(err);
         }

         line += leftJustify(string("Observation"), 20);
         std::string str;
         str = system.systemChar();
         str = str + " (" + system.systemString() + ")";
         line += leftJustify(str, 20);
         line += versionString;
         strm << line << endl;
         strm.lineNumber++;
      }
      if (valid & runByValid)
      {
         line  = leftJustify(fileProgram,20);
         line += leftJustify(fileAgency,20);
         CommonTime dt;
         SystemTime sysTime;
         string dat = (static_cast<CivilTime>(sysTime)).printf("%04Y%02m%02d %02H%02M%02S %P");
         line += leftJustify(dat, 20);
         line += runByString;
         strm << line << endl;
         strm.lineNumber++;
      }
      if (valid & markerNameValid)
      {
         line  = leftJustify(markerName, 60);
         line += markerNameString;
         strm << line << endl;
         strm.lineNumber++;
      }
      if (valid & observerValid)
      {
         line  = leftJustify(observer, 20);
         line += leftJustify(agency, 40);
         line += observerString;
         strm << line << endl;
         strm.lineNumber++;
      }
      if (valid & receiverValid)
      {
         line  = leftJustify(recNo, 20);
         line += leftJustify(recType, 20);
         line += leftJustify(recVers, 20);
         line += receiverString;
         strm << line << endl;
         strm.lineNumber++;
      }
      if (valid & antennaTypeValid)
      {
         line  = leftJustify(antNo, 20);
         line += leftJustify(antType, 20);
         line += string(20, ' ');
         line += antennaTypeString;
         strm << line << endl;
         strm.lineNumber++;
      }
      if (valid & antennaPositionValid)
      {
         line  = rightJustify(asString(antennaPosition[0], 4), 14);
         line += rightJustify(asString(antennaPosition[1], 4), 14);
         line += rightJustify(asString(antennaPosition[2], 4), 14);
         line += string(18, ' ');
         line += antennaPositionString;
         strm << line << endl;
         strm.lineNumber++;
      }
      if (valid & antennaOffsetValid)
      {
         line  = rightJustify(asString(antennaOffset[0], 4), 14);
         line += rightJustify(asString(antennaOffset[1], 4), 14);
         line += rightJustify(asString(antennaOffset[2], 4), 14);
         line += string(18, ' ');
         line += antennaOffsetString;
         strm << line << endl;
         strm.lineNumber++;
      }
      if (valid & waveFactValid)
      {
         line  = rightJustify(asString<short>(wavelengthFactor[0]),6);
         line += rightJustify(asString<short>(wavelengthFactor[1]),6);
         line += string(48, ' ');
         line += waveFactString;
         strm << line << endl;
         strm.lineNumber++;

            // handle continuation lines
         if (!extraWaveFactList.empty())
         {
            vector<ExtraWaveFact>::const_iterator itr = extraWaveFactList.begin();

            while (itr != extraWaveFactList.end())
            {
               const int maxSatsPerLine = 7;
               short satsWritten = 0, satsLeft = (*itr).satList.size(), satsThisLine;
               vector<SatID>::const_iterator vecItr = (*itr).satList.begin();

               while ((vecItr != (*itr).satList.end())) {
                  if(satsWritten == 0) {
                     line  = rightJustify(asString<short>((*itr).wavelengthFactor[0]),6);
                     line += rightJustify(asString<short>((*itr).wavelengthFactor[1]),6);
                     satsThisLine = (satsLeft > maxSatsPerLine ? maxSatsPerLine : satsLeft);
                     line += rightJustify(asString<short>(satsThisLine),6);
                  }
                  try {
                     line += string(3, ' ') + RinexSatID(*vecItr).toString();
                  }
                  catch (Exception& e) {
                     FFStreamError ffse(e);
                     GPSTK_THROW(ffse);
                  }
                  satsWritten++;
                  satsLeft--;
                  if(satsWritten==maxSatsPerLine || satsLeft==0) {      // output a complete line
                     line += string(60 - line.size(), ' ');
                     line += waveFactString;
                     strm << line << endl;
                     strm.lineNumber++;
                     satsWritten = 0;
                  }
                  vecItr++;
               }
               itr++;
            }
         }
      }
      if (valid & obsTypeValid)
      {
         const int maxObsPerLine = 9;
         int obsWritten = 0;
         line = ""; // make sure the line contents are reset.

         vector<RinexObsType>::const_iterator itr = obsTypeList.begin();

         while (itr != obsTypeList.end())
         {
               // the first line needs to have the # of obs
            if (obsWritten == 0)
               line  = rightJustify(asString(obsTypeList.size()), 6);
               // if you hit 9, write out the line and start a new one
            else if ((obsWritten % maxObsPerLine) == 0)
            {
               line += numObsString;
               strm << line << endl;
               strm.lineNumber++;
               line  = string(6, ' ');
            }
            line += rightJustify(convertObsType(*itr), 6);
            obsWritten++;
            itr++;
         }
         line += string(60 - line.size(), ' ');
         line += numObsString;
         strm << line << endl;
         strm.lineNumber++;
      }
      if (valid & intervalValid)
      {
         line  = rightJustify(asString(interval, 3), 10);
         line += string(50, ' ');
         line += intervalString;
         strm << line << endl;
         strm.lineNumber++;
      }
      if (valid & firstTimeValid)
      {
         line  = writeTime(firstObs);
         line += string(48-line.size(),' ');
         if(firstSystem.system == RinexSatID::systemGPS) line += "GPS";
         if(firstSystem.system == RinexSatID::systemGlonass) line += "GLO";
         if(firstSystem.system == RinexSatID::systemGalileo) line += "GAL";
         line += string(60 - line.size(), ' ');
         line += firstTimeString;
         strm << line << endl;
         strm.lineNumber++;
      }
      if (valid & lastTimeValid)
      {
         line  = writeTime(lastObs);
         line += string(48-line.size(),' ');
         if(lastSystem.system == RinexSatID::systemGPS) line += "GPS";
         if(lastSystem.system == RinexSatID::systemGlonass) line += "GLO";
         if(lastSystem.system == RinexSatID::systemGalileo) line += "GAL";
         line += string(60 - line.size(), ' ');
         line += lastTimeString;
         strm << line << endl;
         strm.lineNumber++;
      }
      if (valid & markerNumberValid)
      {
         line  = leftJustify(markerNumber, 20);
         line += string(40, ' ');
         line += markerNumberString;
         strm << line << endl;
         strm.lineNumber++;
      }
      if (valid & receiverOffsetValid)
      {
         line  = rightJustify(asString(receiverOffset),6);
         line += string(54, ' ');
         line += receiverOffsetString;
         strm << line << endl;
         strm.lineNumber++;
      }
      if (valid & leapSecondsValid)
      {
         line  = rightJustify(asString(leapSeconds),6);
         line += string(54, ' ');
         line += leapSecondsString;
         strm << line << endl;
         strm.lineNumber++;
      }
      if (valid & commentValid)
      {
         vector<string>::const_iterator itr = commentList.begin();
         while (itr != commentList.end())
         {
            line  = leftJustify((*itr), 60);
            line += commentString;
            strm << line << endl;
            strm.lineNumber++;
            itr++;
         }
      }
      if (valid & numSatsValid)
      {
         line  = rightJustify(asString(numSVs), 6);
         line += string(54, ' ');
         line += numSatsString;
         strm << line << endl;
         strm.lineNumber++;
      }
      if (valid & prnObsValid)
      {
         const int maxObsPerLine = 9;
         map<SatID, vector<int> >::const_iterator itr = numObsForSat.begin();
         while (itr != numObsForSat.end())
         {
            int numObsWritten = 0;

            vector<int>::const_iterator vecItr = (*itr).second.begin();
            while (vecItr != (*itr).second.end())
            {
               if (numObsWritten == 0)
               {
                  try {
                     RinexSatID prn((*itr).first);
                     line  = string(3, ' ') + prn.toString();
                  }
                  catch (Exception& e) {
                     FFStreamError ffse(e);
                     GPSTK_RETHROW(ffse);
                  }
               }
               else if ((numObsWritten % maxObsPerLine)  == 0)
               {
                  line += prnObsString;
                  strm << line << endl;
                  strm.lineNumber++;
                  line  = string(6, ' ');
               }
               line += rightJustify(asString(*vecItr), 6);
               ++vecItr;
               ++numObsWritten;
            }
            line += string(60 - line.size(), ' ');
            line += prnObsString;
            strm << line << endl;
            strm.lineNumber++;
            itr++;
         }
      }
      if (valid & endValid)
      {
         line  = string(60, ' ');
         line += endOfHeader;
         strm << line << endl;
         strm.lineNumber++;
      }
   }   // end RinexObsHeader::WriteHeaderRecords()


      // this function parses a single header record
   void RinexObsHeader::ParseHeaderRecord(string& line)
      throw(FFStreamError)
   {
      string label(line, 60, 20);

      if (label == versionString)
      {
         version = asDouble(line.substr(0,20));
//         cout << "R2ObsHeader:ParseHeaderRecord:version = " << version << endl;
         fileType = strip(line.substr(20, 20));
         if ( (fileType[0] != 'O') &&
              (fileType[0] != 'o'))
         {
            FFStreamError e("This isn't a Rinex Obs file");
            GPSTK_THROW(e);
         }
         string system_str = strip(line.substr(40, 20));
         try {
            system.fromString(system_str);
         }
         catch (Exception& e)
         {
            FFStreamError ffse("Input satellite system is unsupported: " + system_str);
            GPSTK_THROW(ffse);
         }
         valid |= versionValid;
      }
      else if (label == runByString )
      {
         fileProgram =    strip(line.substr(0, 20));
         fileAgency =  strip(line.substr(20, 20));
         date =   strip(line.substr(40, 20));
         valid |= runByValid;
      }
      else if (label == commentString)
      {
         string s = strip(line.substr(0, 60));
         commentList.push_back(s);
         valid |= commentValid;
      }
      else if (label == markerNameString)
      {
         markerName = strip(line.substr(0,60));
         valid |= markerNameValid;
      }
      else if (label == markerNumberString)
      {
         markerNumber = strip(line.substr(0,20));
         valid |= markerNumberValid;
      }
      else if (label == observerString)
      {
         observer = strip(line.substr(0,20));
         agency = strip(line.substr(20,40));
         valid |= observerValid;
      }
      else if (label == receiverString)
      {
         recNo   = strip(line.substr(0, 20));
         recType = strip(line.substr(20,20));
         recVers = strip(line.substr(40,20));
         valid |= receiverValid;
      }
      else if (label ==antennaTypeString)
      {
         antNo =   strip(line.substr(0, 20));
         antType = strip(line.substr(20, 20));
         valid |= antennaTypeValid;
      }
      else if (label == antennaPositionString)
      {
         antennaPosition[0] = asDouble(line.substr(0,  14));
         antennaPosition[1] = asDouble(line.substr(14, 14));
         antennaPosition[2] = asDouble(line.substr(28, 14));
         valid |= antennaPositionValid;
      }
      else if (label == antennaOffsetString)
      {
         antennaOffset[0] = asDouble(line.substr(0,  14));
         antennaOffset[1] = asDouble(line.substr(14, 14));
         antennaOffset[2] = asDouble(line.substr(28, 14));
         valid |= antennaOffsetValid;
      }
      else if (label == waveFactString)
      {
            // first time reading this
         if (! (valid & waveFactValid))
         {
            wavelengthFactor[0] = asInt(line.substr(0,6));
            wavelengthFactor[1] = asInt(line.substr(6,6));
            valid |= waveFactValid;
         }
            // additional wave fact lines
         else
         {
            const int maxSatsPerLine = 7;
            int Nsats;
            ExtraWaveFact ewf;
            ewf.wavelengthFactor[0] = asInt(line.substr(0,6));
            ewf.wavelengthFactor[1] = asInt(line.substr(6,6));
            Nsats = asInt(line.substr(12,6));

            if (Nsats > maxSatsPerLine)   // > not >=
            {
               FFStreamError e("Invalid number of Sats for " + waveFactString);
               GPSTK_THROW(e);
            }

            for (int i = 0; i < Nsats; i++)
            {
               try {
                  RinexSatID prn(line.substr(21+i*6,3));
                  ewf.satList.push_back(prn);
               }
               catch (Exception& e){
                  FFStreamError ffse(e);
                  GPSTK_RETHROW(ffse);
               }
            }

            extraWaveFactList.push_back(ewf);
         }
      }
      else if (label == numObsString)
      {
         const int maxObsPerLine = 9;
            // process the first line
         if (! (valid & obsTypeValid))
         {
            numObs = asInt(line.substr(0,6));

            for (int i = 0; (i < numObs) && (i < maxObsPerLine); i++)
            {
               int position = i * 6 + 6 + 4;
               RinexObsType rt = convertObsType(line.substr(position,2));
               obsTypeList.push_back(rt);
            }
            valid |= obsTypeValid;
         }
            // process continuation lines
         else
         {
            for (int i = obsTypeList.size();
                 (i < numObs) && ( (i % maxObsPerLine) < maxObsPerLine); i++)
            {
               int position = (i % maxObsPerLine) * 6 + 6 + 4;
               RinexObsType rt = convertObsType(line.substr(position,2));
               obsTypeList.push_back(rt);
            }
         }
      }
      else if (label == intervalString)
      {
         interval = asDouble(line.substr(0, 10));
         valid |= intervalValid;
      }
      else if (label == firstTimeString)
      {
         firstObs = parseTime(line);
         firstSystem.system = RinexSatID::systemGPS;
         if(line.substr(48,3)=="GLO") firstSystem.system=RinexSatID::systemGlonass;
         if(line.substr(48,3)=="GAL") firstSystem.system=RinexSatID::systemGalileo;
         valid |= firstTimeValid;
      }
      else if (label == lastTimeString)
      {
         lastObs = parseTime(line);
         lastSystem.system = RinexSatID::systemGPS;
         if(line.substr(48,3)=="GLO") lastSystem.system=RinexSatID::systemGlonass;
         if(line.substr(48,3)=="GAL") lastSystem.system=RinexSatID::systemGalileo;
         valid |= lastTimeValid;
      }
      else if (label == receiverOffsetString)
      {
         receiverOffset = asInt(line.substr(0,6));
         valid |= receiverOffsetValid;
      }
      else if (label == leapSecondsString)
      {
         leapSeconds = asInt(line.substr(0,6));
         valid |= leapSecondsValid;
      }
      else if (label == numSatsString)
      {
         numSVs = asInt(line.substr(0,6)) ;
         valid |= numSatsValid;
      }
      else if (label == prnObsString)
      {
         const int maxObsPerLine = 9;
            // continuation lines... you have to know what PRN
            // this is continuing for, hence lastPRN
         if ((lastPRN.id != -1) &&
             (numObsForSat[lastPRN].size() != obsTypeList.size()))
         {
            for(int i = numObsForSat[lastPRN].size();
                (i < int(obsTypeList.size())) &&
                   ( (i % maxObsPerLine) < maxObsPerLine); i++)
            {
               numObsForSat[lastPRN].push_back(asInt(line.substr((i%maxObsPerLine)*6+6,6)));
            }
         }
         else
         {
            try {
               lastPRN.fromString(line.substr(3,3));
            }
            catch (Exception& e) {
               FFStreamError ffse(e);
               GPSTK_RETHROW(ffse);
            }
            vector<int> numObsList;
            for(int i = 0;
                   (i < int(obsTypeList.size())) && (i < maxObsPerLine); i++)
            {
               numObsList.push_back(asInt(line.substr(i*6+6,6)));
            }

            numObsForSat[lastPRN] = numObsList;
         }
         valid |= prnObsValid;
      }
      else if (label == endOfHeader)
      {
         valid |= endValid;
      }
      else
      {
         FFStreamError e("Unidentified label: " + label);
         GPSTK_THROW(e);
      }
   }   // end of RinexObsHeader::ParseHeaderRecord(string& line)


      // This function parses the entire header from the given stream
   void RinexObsHeader::reallyGetRecord(FFStream& ffs)
      throw(std::exception, FFStreamError,
            gpstk::StringUtils::StringException)
   {
      RinexObsStream& strm = dynamic_cast<RinexObsStream&>(ffs);

         // if already read, just return
      if (strm.headerRead == true)
         return;

         // since we're reading a new header, we need to reinitialize
         // all our list structures.  all the other objects should be ok.
         // this also applies if we threw an exception the first time we read
         // the header and are now re-reading it. some of these could be full
         // and we need to empty them.
      commentList.clear();
      wavelengthFactor[0] = wavelengthFactor[1] = 1;
      extraWaveFactList.clear();
      obsTypeList.clear();
      numObsForSat.clear();
      valid = 0;
      numObs = 0;
      lastPRN.id = -1;

      string line;

      while (!(valid & endValid))
      {
         strm.formattedGetLine(line);
         StringUtils::stripTrailing(line);

         if (line.length()==0)
         {
            FFStreamError e("No data read");
            GPSTK_THROW(e);
         }
         else if (line.length()<60 || line.length()>80)
         {
            FFStreamError e("Invalid line length");
            GPSTK_THROW(e);
         }

         try
         {
            ParseHeaderRecord(line);
         }
         catch(FFStreamError& e)
         {
            GPSTK_RETHROW(e);
         }

      }   // end while(not end of header)

      unsigned long allValid;
      if      (version == 2.0)      allValid = allValid20;
      else if (version == 2.1)      allValid = allValid21;
      else if (version == 2.11)     allValid = allValid211;
      else
      {
         FFStreamError e("Unknown or unsupported RINEX version " +
                         asString(version));
         GPSTK_THROW(e);
      }

      if ( (allValid & valid) != allValid)
      {
         FFStreamError e("Incomplete or invalid header");
         GPSTK_THROW(e);
      }

         // If we get here, we should have reached the end of header line
      strm.header = *this;
      strm.headerRead = true;

   }  // end of reallyGetRecord()



   RinexObsType
   RinexObsHeader::convertObsType(const string& oneObs)
      throw(FFStreamError)
   {
      RinexObsType ot(RegisteredRinexObsTypes[0]);   // Unknown type
      for(size_t i=0; i<RegisteredRinexObsTypes.size(); i++) {
         if(RegisteredRinexObsTypes[i].type == oneObs) {
            ot = RegisteredRinexObsTypes[i];
            break;
         }
         //FFStreamError e("Bad obs type: " + oneObs);
         //GPSTK_THROW(e);
      }
      return ot;
   }
   string
   RinexObsHeader::convertObsType(const RinexObsType& oneObs)
      throw(FFStreamError)
   {
      return oneObs.type;
   }


   CommonTime RinexObsHeader::parseTime(const string& line) const
   {
      int year, month, day, hour, min;
      double sec;

      year  = asInt(   line.substr(0,  6 ));
      month = asInt(   line.substr(6,  6 ));
      day   = asInt(   line.substr(12, 6 ));
      hour  = asInt(   line.substr(18, 6 ));
      min   = asInt(   line.substr(24, 6 ));
      sec   = asDouble(line.substr(30, 13));
      return CivilTime(year, month, day, hour, min, sec).convertToCommonTime();
   }

   string RinexObsHeader::writeTime(const CommonTime& dt) const
   {
      string line;
      CivilTime civTime(dt);
      line  = rightJustify(asString<short>(civTime.year), 6);
      line += rightJustify(asString<short>(civTime.month), 6);
      line += rightJustify(asString<short>(civTime.day), 6);
      line += rightJustify(asString<short>(civTime.hour), 6);
      line += rightJustify(asString<short>(civTime.minute), 6);
      line += rightJustify(asString(civTime.second, 7), 13);
      return line;
   }

   void RinexObsHeader::dump(ostream& s) const
   {
      size_t i,j;
      s << "---------------------------------- REQUIRED ----------------------------------\n";
      string str;
      str = system.systemChar();
      str = str + " (" + system.systemString() + ")";
      s << "Rinex Version " << fixed << setw(5) << setprecision(2) << version
         << ",  File type " << fileType << ",  System " << str << ".\n";
      s << "Prgm: " << fileProgram << ",  Run: " << date << ",  By: " << fileAgency << endl;
      s << "Marker name: " << markerName << ".\n";
      s << "Obs'r : " << observer << ",  Agency: " << agency << endl;
      s << "Rec#: " << recNo << ",  Type: " << recType << ",  Vers: " << recVers << endl;
      s << "Antenna # : " << antNo << ",  Type : " << antType << endl;
      s << "Position (XYZ,m) : " << setprecision(4) << antennaPosition << ".\n";
      s << "Antenna offset (ENU,m) : " << setprecision(4) << antennaOffset << ".\n";
      s << "Wavelength factors (default) L1:" << wavelengthFactor[0]
         << ", L2: " << wavelengthFactor[1] << ".\n";
      for(i=0; i<extraWaveFactList.size(); i++) {
         s << "Wavelength factors (extra)   L1:"
            << extraWaveFactList[i].wavelengthFactor[0]
            << ", L2: " << extraWaveFactList[i].wavelengthFactor[1]
            << ", for Sats";
         for(j=0; j<extraWaveFactList[i].satList.size(); j++)
            s << " " << extraWaveFactList[i].satList[j];
         s << endl;
      }
      s << "Observation types (" << obsTypeList.size() << ") :\n";
      for(i=0; i<obsTypeList.size(); i++)
         s << " Type #" << i << " = "
            << gpstk::RinexObsHeader::convertObsType(obsTypeList[i])
            << " " << obsTypeList[i].description
            << " (" << obsTypeList[i].units << ")." << endl;
      s << "Time of first obs " << (static_cast<CivilTime>(firstObs)).printf("%04Y/%02m/%02d %02H:%02M:%010.7f")
         << " " << (firstSystem.system==RinexSatID::systemGlonass ? "GLO" :
                   (firstSystem.system==RinexSatID::systemGalileo ? "GAL" : "GPS")) << endl;
      s << "(This header is ";
      if((valid & allValid211) == allValid211) s << "VALID 2.11";
      else if((valid & allValid21) == allValid21) s << "VALID 2.1";
      else if((valid & allValid20) == allValid20) s << "VALID 2.0";
      else s << "NOT VALID";
      s << " Rinex.)\n";

      if(!(valid & versionValid)) s << " Version is NOT valid\n";
      if(!(valid & runByValid)) s << " Run by is NOT valid\n";
      if(!(valid & markerNameValid)) s << " Marker Name is NOT valid\n";
      if(!(valid & observerValid)) s << " Observer is NOT valid\n";
      if(!(valid & receiverValid)) s << " Receiver is NOT valid\n";
      if(!(valid & antennaTypeValid)) s << " Antenna Type is NOT valid\n";
      if(!(valid & antennaPositionValid)) s << " Ant Position is NOT valid\n";
      if(!(valid & antennaOffsetValid)) s << " Antenna Offset is NOT valid\n";
      if(!(valid & waveFactValid)) s << " Wavelength factor is NOT valid\n";
      if(!(valid & obsTypeValid)) s << " Obs Type is NOT valid\n";
      if(!(valid & firstTimeValid)) s << " First time is NOT valid\n";
      if(!(valid & endValid)) s << " End is NOT valid\n";

      s << "---------------------------------- OPTIONAL ----------------------------------\n";
      if(valid & markerNumberValid) s << "Marker number : " << markerNumber << endl;
      if(valid & intervalValid) s << "Interval = "
         << fixed << setw(7) << setprecision(3) << interval << endl;
      if(valid & lastTimeValid) s << "Time of last obs "
         << (static_cast<CivilTime>(lastObs)).printf("%04Y/%02m/%02d %02H:%02M:%010.7f")
         << " " << (lastSystem.system==RinexSatID::systemGlonass ? "GLO":
                   (lastSystem.system==RinexSatID::systemGalileo ? "GAL" : "GPS")) << endl;
      if(valid & leapSecondsValid) s << "Leap seconds: " << leapSeconds << endl;
      if(valid & receiverOffsetValid) s << "Clock offset record is present and offsets "
         << (receiverOffset?"ARE":"are NOT") << " applied." << endl;
      if(valid & numSatsValid) s << "Number of Satellites with data : " << numSVs << endl;
      if(valid & prnObsValid) {
         s << "SAT  ";
         for(i=0; i<obsTypeList.size(); i++)
            s << setw(7) << convertObsType(obsTypeList[i]);
         s << endl;
         map<SatID, vector<int> >::const_iterator sat_itr = numObsForSat.begin();
         while (sat_itr != numObsForSat.end()) {
            vector<int> obsvec=sat_itr->second;
            s << " " << RinexSatID(sat_itr->first) << " ";
            for(i=0; i<obsvec.size(); i++) s << " " << setw(6) << obsvec[i];
            s << endl;
            sat_itr++;
         }
      }
      if(commentList.size() && !(valid & commentValid)) s << " Comment is NOT valid\n";
      s << "Comments (" << commentList.size() << ") :\n";
      for(i=0; i<commentList.size(); i++)
         s << commentList[i] << endl;
      s << "-------------------------------- END OF HEADER -------------------------------\n";
   }

   // return 1 if type already defined,
   //        0 if successful
   //       -1 if not successful - invalid input
   int RegisterExtendedRinexObsType(string t, string d, string u, unsigned int dep)
   {
      if(t.empty()) return -1;
      // throw if t="UN" ?
      // check that it is not duplicated
      for(size_t i=0; i<RinexObsHeader::RegisteredRinexObsTypes.size(); i++) {
         if(RinexObsHeader::RegisteredRinexObsTypes[i].type == t) { return 1; }
      }
      RinexObsType ot;
      if(t.size()>2) t.resize(2,' '); ot.type = stripTrailing(t);
      if(d.size()>20) d.resize(20,' '); ot.description = stripTrailing(d);
      if(u.size()>10) u.resize(10,' '); ot.units = stripTrailing(u);
      ot.depend = dep;
      RinexObsHeader::RegisteredRinexObsTypes.push_back(ot);
      return 0;
   }

      // Pretty print a list of standard Rinex observation types
   void DisplayStandardRinexObsTypes(ostream& s)
   {
      s << "The list of standard Rinex obs types:\n";
      s << "  OT Description          Units\n";
      s << "  -- -------------------- ---------\n";
      for(size_t i=0; i<RinexObsHeader::StandardRinexObsTypes.size(); i++) {
         string line;
         line = string("  ")+RinexObsHeader::StandardRinexObsTypes[i].type;
         line += leftJustify(string(" ")+RinexObsHeader::StandardRinexObsTypes[i].description,21);
         line += leftJustify(string(" ")+RinexObsHeader::StandardRinexObsTypes[i].units,11);
         s << line << endl;
      }
   }

      // Pretty print a list of registered extended Rinex observation types
   void DisplayExtendedRinexObsTypes(ostream& s)
   {
      s << "The list of available extended Rinex obs types:\n";
      s << "  OT Description          Units     Required input (EP=ephemeris,PS=Rx Position)\n";
      s << "  -- -------------------- --------- ------------------\n";
      for(size_t i=RinexObsHeader::StandardRinexObsTypes.size();
               i<RinexObsHeader::RegisteredRinexObsTypes.size(); i++) {
         string line;
         line = string("  ")+RinexObsHeader::RegisteredRinexObsTypes[i].type;
         line += leftJustify(string(" ")+RinexObsHeader::RegisteredRinexObsTypes[i].description,21);
         line += leftJustify(string(" ")+RinexObsHeader::RegisteredRinexObsTypes[i].units,11);
         for(int j=1; j<=6; j++) {
            if(j==3 || j==4) continue;
            if(RinexObsHeader::RegisteredRinexObsTypes[i].depend &
               RinexObsHeader::StandardRinexObsTypes[j].depend)
                  line += string(" ")+RinexObsHeader::StandardRinexObsTypes[j].type;
            else line += string("   ");
         }
         if(RinexObsHeader::RegisteredRinexObsTypes[i].depend & RinexObsType::EPdepend)
            line += string(" EP"); else line += string("   ");
         if(RinexObsHeader::RegisteredRinexObsTypes[i].depend & RinexObsType::PSdepend)
            line += string(" PS"); else line += string("   ");
         s << line << endl;
      }
   }

} // namespace gpstk
