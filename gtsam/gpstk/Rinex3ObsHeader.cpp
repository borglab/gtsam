#pragma ident "$Id$"

/**
 * @file Rinex3ObsHeader.cpp
 * Encapsulate header of Rinex observation file, including I/O
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

#include <sstream>
#include <algorithm>

#include "StringUtils.hpp"
#include "SystemTime.hpp"
#include "TimeString.hpp"
#include "Rinex3ObsStream.hpp"
#include "Rinex3ObsHeader.hpp"

using namespace std;
using namespace gpstk::StringUtils;

namespace gpstk
{
   const string Rinex3ObsHeader::stringVersion           = "RINEX VERSION / TYPE";
   const string Rinex3ObsHeader::stringRunBy             = "PGM / RUN BY / DATE";
   const string Rinex3ObsHeader::stringComment           = "COMMENT";
   const string Rinex3ObsHeader::stringMarkerName        = "MARKER NAME";
   const string Rinex3ObsHeader::stringMarkerNumber      = "MARKER NUMBER";
   const string Rinex3ObsHeader::stringMarkerType        = "MARKER TYPE";
   const string Rinex3ObsHeader::stringObserver          = "OBSERVER / AGENCY";
   const string Rinex3ObsHeader::stringReceiver          = "REC # / TYPE / VERS";
   const string Rinex3ObsHeader::stringAntennaType       = "ANT # / TYPE";
   const string Rinex3ObsHeader::stringAntennaPosition   = "APPROX POSITION XYZ";
   const string Rinex3ObsHeader::stringAntennaDeltaHEN   = "ANTENNA: DELTA H/E/N";
   const string Rinex3ObsHeader::stringAntennaDeltaXYZ   = "ANTENNA: DELTA X/Y/Z";
   const string Rinex3ObsHeader::stringAntennaPhaseCtr   = "ANTENNA: PHASECENTER";
   const string Rinex3ObsHeader::stringAntennaBsightXYZ  = "ANTENNA: B.SIGHT XYZ";
   const string Rinex3ObsHeader::stringAntennaZeroDirAzi = "ANTENNA: ZERODIR AZI";
   const string Rinex3ObsHeader::stringAntennaZeroDirXYZ = "ANTENNA: ZERODIR XYZ";
   const string Rinex3ObsHeader::stringCenterOfMass      = "CENTER OF MASS: XYZ";
   const string Rinex3ObsHeader::stringNumObs            = "# / TYPES OF OBSERV";      // R2
   const string Rinex3ObsHeader::stringSystemNumObs      = "SYS / # / OBS TYPES";
   const string Rinex3ObsHeader::stringWaveFact          = "WAVELENGTH FACT L1/2";     // R2
   const string Rinex3ObsHeader::stringSigStrengthUnit   = "SIGNAL STRENGTH UNIT";
   const string Rinex3ObsHeader::stringInterval          = "INTERVAL";
   const string Rinex3ObsHeader::stringFirstTime         = "TIME OF FIRST OBS";
   const string Rinex3ObsHeader::stringLastTime          = "TIME OF LAST OBS";
   const string Rinex3ObsHeader::stringReceiverOffset    = "RCV CLOCK OFFS APPL";
   const string Rinex3ObsHeader::stringSystemDCBSapplied = "SYS / DCBS APPLIED";
   const string Rinex3ObsHeader::stringSystemPCVSapplied = "SYS / PCVS APPLIED";
   const string Rinex3ObsHeader::stringSystemScaleFac    = "SYS / SCALE FACTOR";
   const string Rinex3ObsHeader::stringSystemPhaseShift  = "SYS / PHASE SHIFT";
   const string Rinex3ObsHeader::stringGlonassSlotFreqNo = "GLONASS SLOT / FRQ #";
   const string Rinex3ObsHeader::stringGlonassCodPhsBias = "GLONASS COD/PHS/BIS";
   const string Rinex3ObsHeader::stringLeapSeconds       = "LEAP SECONDS";
   const string Rinex3ObsHeader::stringNumSats           = "# OF SATELLITES";
   const string Rinex3ObsHeader::stringPrnObs            = "PRN / # OF OBS";
   const string Rinex3ObsHeader::stringEoH               = "END OF HEADER";

   void Rinex3ObsHeader::reallyPutRecord(FFStream& ffs) const
      throw(exception, FFStreamError, StringException)
   {
      Rinex3ObsStream& strm = dynamic_cast<Rinex3ObsStream&>(ffs);

      strm.header = *this;

      unsigned long allValid;
      if     (version == 3.00)  allValid = allValid30;
      else if(version == 3.01)  allValid = allValid301;
      else if(version == 3.02)  allValid = allValid302;
      else if(version <  3)     allValid = allValid2;
      else {
         FFStreamError err("Unknown RINEX version: " + asString(version,2));
         err.addText("Make sure to set the version correctly.");
         GPSTK_THROW(err);
      }

      if((valid & allValid) != allValid) {
         ostringstream msg;
         msg << endl;
         msg << "Version = " << version << hex << endl;
         if(version == 3.02)
            msg << "allValid302 = 0x" << setw(8) << nouppercase << allValid302 << endl;
         else if(version == 3.01)
            msg << "allValid301 = 0x" << setw(8) << nouppercase << allValid301 << endl;
         else if(version == 3.00)
            msg << " allValid30 = 0x" << setw(8) << nouppercase << allValid30 << endl;
         else
            msg << "  allValid2 = 0x" << setw(8) << nouppercase << allValid2 << endl;
         msg << "      valid = 0x" << setw(8) << nouppercase << valid << endl;
         msg << "Version         " << setw(8) << (valid & validVersion        ) << endl;
         msg << "Run By          " << setw(8) << (valid & validRunBy          ) << endl;
         msg << "Marker Name     " << setw(8) << (valid & validMarkerName     ) << endl;
         msg << "Marker Type     " << setw(8) << (valid & validMarkerType     ) << endl;
         msg << "Observer        " << setw(8) << (valid & validObserver       ) << endl;
         msg << "Receiver        " << setw(8) << (valid & validReceiver       ) << endl;
         msg << "Antenna Type    " << setw(8) << (valid & validAntennaType    ) << endl;
         msg << "Antenna DHEN    " << setw(8) << (valid & validAntennaDeltaHEN) << endl;
         if(version <  3)
            msg << "# Obs Type      " << setw(8) << (valid & validNumObs) << endl;
         if(version >= 3)
            msg << "Sys Obs Type    " << setw(8) << (valid & validSystemObsType  ) << endl;
         if(version <  3)
            msg << "Wave Fact       " << setw(8) << (valid & validWaveFact) << endl;
         msg << "Sys Phs Shft    " << setw(8) << (valid & validSystemPhaseShift)<< endl;
         msg << "GLO Freq No     " << setw(8) << (valid & validGlonassFreqNo  ) << endl;
         msg << "GLO Cod-Phs Bias" << setw(8) << (valid & validGlonassCodPhsBias) << endl;
         msg << "Interval        " << setw(8) << (valid & validInterval       ) << endl;
         msg << "First Time      " << setw(8) << (valid & validFirstTime      ) << endl;
         msg << "End Header      " << setw(8) << (validEoH ? "true":"false"   );    // no endl
         FFStreamError err("Incomplete or invalid header.");
         err.addText("Make sure you set all header valid bits for all of the available data.");
         err.addText(msg.str());
         GPSTK_THROW(err);
      }

      try {
         WriteHeaderRecords(strm);
      }
      catch(FFStreamError& e) {
         GPSTK_RETHROW(e);
      }
      catch(StringException& e) {
         GPSTK_RETHROW(e);
      }

   }  // end reallyPutRecord


   // This function computes the number of valid header records
   // which WriteHeaderRecords will write.
   // NB not used in Rinex3Obs....
   int Rinex3ObsHeader::NumberHeaderRecordsToBeWritten(void) const throw()
   {
      int n = 0;

      if(valid & validVersion          ) n++;
      if(valid & validRunBy            ) n++;
      if(valid & validComment          ) n += commentList.size();
      if(valid & validMarkerName       ) n++;
      if(valid & validMarkerNumber     ) n++;
      if(valid & validMarkerType       ) n++;
      if(valid & validObserver         ) n++;
      if(valid & validReceiver         ) n++;
      if(valid & validAntennaType      ) n++;
      if(valid & validAntennaPosition  ) n++;
      if(valid & validAntennaDeltaHEN  ) n++;
      if(valid & validAntennaDeltaXYZ  ) n++;
      if(valid & validAntennaPhaseCtr  ) n++;
      if(valid & validAntennaBsightXYZ ) n++;
      if(valid & validAntennaZeroDirAzi) n++;
      if(valid & validAntennaZeroDirXYZ) n++;
      if(valid & validCenterOfMass     ) n++;
      if(version < 3 && (valid & validNumObs))
         n += 1 + (obsTypeList.size()-1)/9;
      if(version >= 3 && (valid & validSystemObsType))
         n += 1 + (obsTypeList.size()-1)/9;
      if(version < 3 && (valid & validWaveFact)) {
         n++;
         if(extraWaveFactList.size()) n += (extraWaveFactList.size()-1)/7;
      }
      if(valid & validSigStrengthUnit  ) n++;
      if(valid & validInterval         ) n++;
      if(valid & validFirstTime        ) n++;
      if(valid & validLastTime         ) n++;
      if(valid & validReceiverOffset   ) n++;
      if(valid & validSystemDCBSapplied) n++;
      if(valid & validSystemPCVSapplied) n++;
      if(valid & validSystemScaleFac   ) n++;
      if(valid & validSystemPhaseShift ) n++;        // one per system at least
      if(valid & validGlonassFreqNo    ) n++;
      if(valid & validGlonassCodPhsBias) n++;
      if(valid & validLeapSeconds      ) n++;
      if(valid & validNumSats          ) n++;
      if(valid & validPrnObs           )
         n += numObsForSat.size() * (1+numObsForSat.begin()->second.size()/9);
      if(validEoH                      ) n++;

      return n;
   }  // end NumberHeaderRecordsToBeWritten


   // This function writes all valid header records.
   void Rinex3ObsHeader::WriteHeaderRecords(FFStream& ffs) const
      throw(FFStreamError, StringException)
   {
      Rinex3ObsStream& strm = dynamic_cast<Rinex3ObsStream&>(ffs);
      string line;

      if(valid & validVersion) {
         line  = rightJustify(asString(version,2), 9);
         line += string(11, ' ');

         if((fileType[0] != 'O') && (fileType[0] != 'o'))
         {
            FFStreamError err("File type is not Observation: " + fileType);
            GPSTK_THROW(err);
         }

         if(fileSysSat.system == RinexSatID::systemUnknown)
         {
            FFStreamError err("Invalid satellite system");
            GPSTK_THROW(err);
         }

         line += leftJustify(string("OBSERVATION DATA"), 20);
         string str;
         if(fileSysSat.system == SatID::systemMixed)
            str = "MIXED";
         else {
            RinexSatID sat(fileSysSat);
            str = sat.systemChar();
            str = str + " (" + sat.systemString() + ")";
         }
         line += leftJustify(str, 20);
         line += stringVersion;
         strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
         strm.lineNumber++;
      }
//    cout << "past validVersion" << endl;
      if(valid & validRunBy)
      {
         line  = leftJustify(fileProgram, 20);
         line += leftJustify(fileAgency , 20);
         SystemTime sysTime;
         string curDate = printTime(sysTime,"%04Y%02m%02d %02H%02M%02S %P");
         line += leftJustify(curDate, 20);
         line += stringRunBy;
         strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
         strm.lineNumber++;
      }
//    cout << "past validRunBy" << endl;
      if(valid & validComment)
      {
         vector<string>::const_iterator itr = commentList.begin();
         while (itr != commentList.end())
         {
            line  = leftJustify((*itr), 60);
            line += stringComment;
            strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
            strm.lineNumber++;
            itr++;
         }
      }
//    cout << "past validComment" << endl;
      if(valid & validMarkerName)
      {
         line  = leftJustify(markerName, 60);
         line += stringMarkerName;
         strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
         strm.lineNumber++;
      }
//    cout << "past validMarkerName" << endl;
      if(valid & validMarkerNumber)
      {
         line  = leftJustify(markerNumber, 20);
         line += string(40, ' ');
         line += stringMarkerNumber;
         strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
         strm.lineNumber++;
      }
//    cout << "past validMarkerNumber" << endl;
      if(valid & validMarkerType)
      {
         line  = leftJustify(markerType, 20);
         line += string(40, ' ');
         line += stringMarkerType;
         strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
         strm.lineNumber++;
      }
//    cout << "past validMarkerType" << endl;
      if(valid & validObserver)
      {
         line  = leftJustify(observer, 20);
         line += leftJustify(agency  , 40);
         line += stringObserver;
         strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
         strm.lineNumber++;
      }
//    cout << "past validObserver" << endl;
      if(valid & validReceiver)
      {
         line  = leftJustify(recNo  , 20);
         line += leftJustify(recType, 20);
         line += leftJustify(recVers, 20);
         line += stringReceiver;
         strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
         strm.lineNumber++;
      }
//    cout << "past validReceiver" << endl;
      if(valid & validAntennaType)
      {
         line  = leftJustify(antNo  , 20);
         line += leftJustify(antType, 20);
         line += string(20, ' ');
         line += stringAntennaType;
         strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
         strm.lineNumber++;
      }
//    cout << "past validAntennaType" << endl;
      if(valid & validAntennaPosition)
      {
         line  = rightJustify(asString(antennaPosition[0], 4), 14);
         line += rightJustify(asString(antennaPosition[1], 4), 14);
         line += rightJustify(asString(antennaPosition[2], 4), 14);
         line += string(18, ' ');
         line += stringAntennaPosition;
         strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
         strm.lineNumber++;
      }
//    cout << "past validAntennaPosition" << endl;
      if(valid & validAntennaDeltaHEN)
      {
         line  = rightJustify(asString(antennaDeltaHEN[0], 4), 14);
         line += rightJustify(asString(antennaDeltaHEN[1], 4), 14);
         line += rightJustify(asString(antennaDeltaHEN[2], 4), 14);
         line += string(18, ' ');
         line += stringAntennaDeltaHEN;
         strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
         strm.lineNumber++;
      }
//    cout << "past validAntennaDeltaHEN" << endl;
      if(valid & validAntennaDeltaXYZ)
      {
         line  = rightJustify(asString(antennaDeltaXYZ[0], 4), 14);
         line += rightJustify(asString(antennaDeltaXYZ[1], 4), 14);
         line += rightJustify(asString(antennaDeltaXYZ[2], 4), 14);
         line += string(18, ' ');
         line += stringAntennaDeltaXYZ;
         strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
         strm.lineNumber++;
      }
//    cout << "past validAntennaDeltaXYZ" << endl;
      if(valid & validAntennaPhaseCtr)
      {
         line  =  leftJustify(antennaSatSys , 1);
         line += string(1, ' ');
         line += rightJustify(antennaObsCode, 3);
         line += rightJustify(asString(antennaPhaseCtr[0], 4),  9);
         line += rightJustify(asString(antennaPhaseCtr[1], 4), 14);
         line += rightJustify(asString(antennaPhaseCtr[2], 4), 14);
         line += string(18, ' ');
         line += stringAntennaPhaseCtr;
         strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
         strm.lineNumber++;
      }
//    cout << "past validAntennaPhaseCtr" << endl;
      if(valid & validAntennaBsightXYZ)
      {
         line  = rightJustify(asString(antennaBsightXYZ[0], 4), 14);
         line += rightJustify(asString(antennaBsightXYZ[1], 4), 14);
         line += rightJustify(asString(antennaBsightXYZ[2], 4), 14);
         line += string(18, ' ');
         line += stringAntennaBsightXYZ;
         strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
         strm.lineNumber++;
      }
//    cout << "past validAntennaBsightXYZ" << endl;
      if(valid & validAntennaZeroDirAzi)
      {
         line  = rightJustify(asString(antennaZeroDirAzi, 4), 14);
         line += string(46, ' ');
         line += stringAntennaZeroDirAzi;
         strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
         strm.lineNumber++;
      }
//    cout << "past validAntennaZeroDirAzi" << endl;
      if(valid & validAntennaZeroDirXYZ)
      {
         line  = rightJustify(asString(antennaZeroDirXYZ[0], 4), 14);
         line += rightJustify(asString(antennaZeroDirXYZ[1], 4), 14);
         line += rightJustify(asString(antennaZeroDirXYZ[2], 4), 14);
         line += string(18, ' ');
         line += stringAntennaZeroDirXYZ;
         strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
         strm.lineNumber++;
      }
//    cout << "past validAntennaZeroDirXYZ" << endl;
      if(valid & validCenterOfMass)
      {
         line  = rightJustify(asString(centerOfMass[0], 4), 14);
         line += rightJustify(asString(centerOfMass[1], 4), 14);
         line += rightJustify(asString(centerOfMass[2], 4), 14);
         line += string(18, ' ');
         line += stringCenterOfMass;
         strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
         strm.lineNumber++;
      }
//    cout << "past validCenterOfMass" << endl;
      if(version < 3 && (valid & validNumObs))          // R2 only
      {
         // write out RinexObsTypes
         const int maxObsPerLine = 9;
         int obsWritten = 0;
         line = ""; // make sure the line contents are reset.

         for(size_t i=0; i<R2ObsTypes.size(); i++) {
            string val;
            // the first line needs to have the # of obs
            if(obsWritten == 0)
               line  = rightJustify(asString(R2ObsTypes.size()), 6);
            // if you hit 9, write out the line and start a new one
            else if((obsWritten % maxObsPerLine) == 0) {
               line += stringNumObs;
               strm << line << endl;
               strm.lineNumber++;
               line  = string(6, ' ');
            }
            val = R2ObsTypes[i];
            line += rightJustify(val, 6);
            obsWritten++;
         }

         line += string(60 - line.size(), ' ');
         line += stringNumObs;
         strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
         strm.lineNumber++;
      }
//    cout << "past validNumObs" << endl;
      if(version >= 3 && (valid & validSystemObsType))
      {
         static const int maxObsPerLine = 13;

         map<string,vector<RinexObsID> >::const_iterator mapIter;
         for(mapIter = mapObsTypes.begin(); mapIter != mapObsTypes.end(); mapIter++)
         {
            int obsWritten = 0;
            line = ""; // make sure the line contents are reset

            vector<RinexObsID> ObsTypeList = mapIter->second;

            for(size_t i = 0; i < ObsTypeList.size(); i++)
            {
               // the first line needs to have the GNSS type and # of obs
               if(obsWritten == 0)
               {
                  line  =  leftJustify(mapIter->first, 1);
                  line += string(2, ' ');
                  line += rightJustify(asString(ObsTypeList.size()), 3);
               }
               // if you hit 13, write out the line and start a new one
               else if((obsWritten % maxObsPerLine) == 0)
               {
                  line += string(2, ' ');
                  line += stringSystemNumObs;
                  strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
                  strm.lineNumber++;
                  line  = string(6, ' ');
               }
               line += string(1, ' ');
               line += rightJustify(ObsTypeList[i].asString(), 3);
               obsWritten++;
            }
            line += string(60 - line.size(), ' ');
            line += stringSystemNumObs;
            strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
            strm.lineNumber++;
         }

      }
//    cout << "past validSystemObsType" << endl;
      if(version < 3 && (valid & validWaveFact))
      {
         line  = rightJustify(asString<short>(wavelengthFactor[0]),6);
         line += rightJustify(asString<short>(wavelengthFactor[1]),6);
         line += string(48, ' ');
         line += stringWaveFact;
         strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
         strm.lineNumber++;
         
            // handle continuation lines
         if(!extraWaveFactList.empty())
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
                     line += stringWaveFact;
                     strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
                     strm.lineNumber++;
                     satsWritten = 0;
                  }
                  vecItr++;
               }
               itr++;
            }
         }
      }
//    cout << "past validWaveFact" << endl;
      if(valid & validSigStrengthUnit && version >= 3)
      {
         line  = leftJustify(sigStrengthUnit, 20);
         line += string(40, ' ');
         line += stringSigStrengthUnit;
         strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
         strm.lineNumber++;
      }
//    cout << "past validSigStrengthUnit" << endl;
      if(valid & validInterval)
      {
         line  = rightJustify(asString(interval, 3), 10);
         line += string(50, ' ');
         line += stringInterval;
         strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
         strm.lineNumber++;
      }
//    cout << "past validInterval" << endl;
      if(valid & validFirstTime)
      {
         line  = writeTime(firstObs);
         line += string(60 - line.size(), ' ');
         line += stringFirstTime;
         strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
         strm.lineNumber++;
      }
//    cout << "past validFirstTime" << endl;
      if(valid & validLastTime)
      {
         line  = writeTime(lastObs);
         line += string(60 - line.size(), ' ');
         line += stringLastTime;
         strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
         strm.lineNumber++;
      }
//    cout << "past validLastTime" << endl;
      if(valid & validReceiverOffset)
      {
         line  = rightJustify(asString(receiverOffset), 6);
         line += string(54, ' ');
         line += stringReceiverOffset;
         strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
         strm.lineNumber++;
      }
//    cout << "past validReceiverOffset" << endl;
      if(valid & validSystemDCBSapplied)
      {
         for(size_t i = 0; i < infoDCBS.size(); i++)
         {
           line  = leftJustify(infoDCBS[i].satSys,  1);
           line += string(1, ' ');
           line += leftJustify(infoDCBS[i].name  , 17);
           line += string(1, ' ');
           line += leftJustify(infoDCBS[i].source, 40);
           line += stringSystemDCBSapplied;
           strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
           strm.lineNumber++;
         }
      }
//    cout << "past validSystemDCBSapplied" << endl;
      if(valid & validSystemPCVSapplied)
      {
         for(size_t i = 0; i < infoPCVS.size(); i++)
         {
           line  = leftJustify(infoPCVS[i].satSys,  1);
           line += string(1, ' ');
           line += leftJustify(infoPCVS[i].name  , 17);
           line += string(1, ' ');
           line += leftJustify(infoPCVS[i].source, 40);
           line += stringSystemPCVSapplied;
           strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
           strm.lineNumber++;
         }
      }
//    cout << "past validSystemPCVSapplied" << endl;
      if(valid & validSystemScaleFac)
      {
         static const int maxObsPerLine = 12;

         static const int size = 4;
         static const int factors[size] = {1,10,100,1000};
         vector<string> obsTypes;

         // loop over GNSSes
         map<string, sfacMap>::const_iterator mapIter;
         for(mapIter = sysSfacMap.begin(); mapIter != sysSfacMap.end(); mapIter++)
         {
            map<RinexObsID, int>::const_iterator iter;

            for(int i = 0; i < size; i++) // loop over possible factors (above)
            {
               int count = 0;
               obsTypes.clear(); // clear the list of Obs Types we're going to make

               for(iter = mapIter->second.begin();      // loop over scale factor map
                     iter != mapIter->second.end(); iter++)
               {
                  if(iter->second == factors[i] )
                  {
                     count++;
                     obsTypes.push_back(iter->first.asString());
                  }
               }

               if(count == 0 ) continue;

               line  =  leftJustify(mapIter->first      , 1);
               line += string(1, ' ');
               line += rightJustify(asString(factors[i]), 4);
               line += string(2, ' ');
               line += rightJustify(asString(count     ), 2);

               for(int j = 0; j < count; j++)
               {
                  if(j > maxObsPerLine-1 && (j % maxObsPerLine) == 0 )
                  {
                  // need continuation; end current line
                     line += string(2, ' ');
                     line += stringSystemScaleFac;
                     strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
                     strm.lineNumber++;
                     line  = string(10, ' ');
                  }
                  line += string(1, ' ');
                  line += rightJustify(obsTypes[j], 3);
               }
               int space = 60 - 10 - 4*(count % maxObsPerLine);
               line += string(space, ' ');
               line += stringSystemScaleFac;
               strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
               strm.lineNumber++;
            }
         }
      }
//    cout << "past validSystemScaleFac" << endl;
      if(valid & validSystemPhaseShift && version >= 3)
      {
         //map<string, map<RinexObsID, map<RinexSatID,double> > > sysPhaseShift;
         map<string, map<RinexObsID, map<RinexSatID,double> > >::const_iterator it;
         for(it=sysPhaseShift.begin(); it!=sysPhaseShift.end(); ++it) {
            string sys(it->first);
       //cout << "Phase shift for system " << sys << endl;
            map<RinexObsID, map<RinexSatID,double> >::const_iterator jt(it->second.begin());
            if(jt == it->second.end()) {
               line  = sys;
               line += string(60-line.length(), ' ');
               line += stringSystemPhaseShift;
               strm << line << endl;
       //cout << "Line >" << line << "<" << endl;
               strm.lineNumber++;
            }
            else for( ; jt!=it->second.end(); ++jt) {
               RinexObsID obsid(jt->first);
               RinexSatID sat(jt->second.begin()->first);
               double corr(jt->second.begin()->second);
               line  = sys;
               if(sat.id == -1) {
                  line += string(60-line.length(), ' ');
                  line += stringSystemPhaseShift;
                  strm << line << endl;
       //cout << "Line >" << line << "<" << endl;
                  strm.lineNumber++;
               }
               else {                  // list of sats
                  line += obsid.asString() + string(" ");
                  line += rightJustify(asString(corr,5),8);
                  setfill('0');
                  line += string("  ") + rightJustify(asString(jt->second.size()),2);
                  setfill(' ');

                  int n(0);
                  map<RinexSatID,double>::const_iterator kt,lt;
                  for(kt=jt->second.begin(); kt!=jt->second.end(); ++kt) {
                     line += string(" ") + kt->first.toString();
                     if(++n == 10 || ++(lt=kt) == jt->second.end()) {   // end this line
                        line += string(60-line.length(), ' ');
                        line += stringSystemPhaseShift;
                        strm << line << endl;
       //cout << "Line >" << line << "<" << endl;
                        strm.lineNumber++;
                        n = 0;
                        // are there more for a continuation line?
                        if(lt != jt->second.end())
                           line = string(18,' ');
                     }
                  }
               }
            }
         }
      }
    //cout << "past validSystemPhaseShift" << endl;
      if(valid & validGlonassFreqNo)
      {
         //map<RinexSatID,int> GlonassFreqNo;
         int n(0),nsat(GlonassFreqNo.size());
         line = rightJustify(asString(nsat),3) + string(" ");
         map<RinexSatID,int>::const_iterator it,kt;
         for(it = GlonassFreqNo.begin(); it != GlonassFreqNo.end(); ++it) {
            line += it->first.toString();
            line += rightJustify(asString(it->second),3);
            if(++n == 8 || ++(kt=it) == GlonassFreqNo.end()) {    // write it
               line += string(60-line.length(), ' ');
               line += stringGlonassSlotFreqNo;
               strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
               strm.lineNumber++;
               n = 0;
               // are there more for a continuation line?
               if(kt != GlonassFreqNo.end())
                  line = string(4,' ');
            }
         }
      }
//    cout << "past validGlonassFreqNo" << endl;
      if(valid & validGlonassCodPhsBias)
      {
         map<RinexObsID,double>::const_iterator it;
         const string labs[4]={"C1C","C1P","C2C","C2P"};
         for(int i=0; i<4; i++) {
            RinexObsID obsid(RinexObsID("R"+labs[i]));
            it = GlonassCodePhaseBias.find(obsid);
            double bias = (it == GlonassCodePhaseBias.end() ? it->second : 0.0);
            line += " " + labs[i] + rightJustify(asString(bias,3),8);
         }
         line += "        " + stringGlonassCodPhsBias;
         strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
         strm.lineNumber++;
      }
//    cout << "past validGlonassCodPhsBias" << endl;
      if(valid & validLeapSeconds)
      {
         line  = rightJustify(asString(leapSeconds), 6);
         line += string(54, ' ');
         line += stringLeapSeconds;
         strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
         strm.lineNumber++;
      }
//    cout << "past validLeapSeconds" << endl;
      if(valid & validNumSats)
      {
         line  = rightJustify(asString(numSVs), 6);
         line += string(54, ' ');
         line += stringNumSats;
         strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
         strm.lineNumber++;
      }
//    cout << "past validNumSats" << endl;
      if(valid & validPrnObs)
      {
         static const int maxObsPerLine = 9;
         map<RinexSatID, vector<int> >::const_iterator itr(numObsForSat.begin());
         // loop over satellites
         while(itr != numObsForSat.end()) {
            int numObsWritten = 0;                                // # of counts written for this sat
            RinexSatID sat(itr->first);                           // the sat
            const vector<int>& numObs(itr->second);               // the vector of ints stored
            vector<int> vec;                                      // the vector of ints to write

            if(version >= 3)
               vec = numObs;
            else {                                                // fill in zeros for version 2
               int j;
               size_t i;
               string sys(string(1,sat.systemChar()));
               map<string, map<string, RinexObsID> >::const_iterator jt(mapSysR2toR3ObsID.find(sys));
               const map<string, RinexObsID> mapVec(jt->second);
               map<string, RinexObsID>::const_iterator kt;
//cout << "mapSys " << sat.toString();
               for(i=0,j=0; i<R2ObsTypes.size(); i++) {
//cout << " " << R2ObsTypes[i] << ":";
                  kt = mapVec.find(R2ObsTypes[i]);
                  string obsid(kt->second.asString());
//cout << obsid;
                  if(obsid == string("   ")) vec.push_back(0.0);
                  else                       vec.push_back(numObs[j++]);
               }
//cout << endl;
            }

            vector<int>::const_iterator vecItr(vec.begin());
            while (vecItr != vec.end()) {
               if(numObsWritten == 0) {                           // start of line
                  try {
                     line = string(3, ' ') + sat.toString();      // '   G01'
                  }
                  catch (Exception& e)
                  {
                     FFStreamError ffse(e);
                     GPSTK_RETHROW(ffse); 
                  }
               }
               else if((numObsWritten % maxObsPerLine) == 0) {    // end of line
                  line += stringPrnObs;
                  strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
                  strm.lineNumber++;
                  line  = string(6, ' ');
               }

               line += rightJustify(asString(*vecItr), 6);        // add num obs to line
               ++vecItr;
               ++numObsWritten;
            }

            // finish last line
            line += string(60 - line.size(), ' ');
            line += stringPrnObs;
            strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
            strm.lineNumber++;
            itr++;
         }
      }
//    cout << "past validPrnObs" << endl;
      if(validEoH)
      {
         line  = string(60, ' ');
         line += stringEoH;
         strm << line << endl;
//       cout << "Line >" << line << "<" << endl;
         strm.lineNumber++;
      }
//    cout << "past validEoH" << endl;
//    cout << "R3ObsHeader: wrote header records" << endl;
   } // end WriteHeaderRecords


   // This function parses a single header record.
   void Rinex3ObsHeader::ParseHeaderRecord(string& line)
      throw(FFStreamError)
   {
      int i;
      string label(line, 60, 20);
         
      if(label == stringVersion)
      {
         version  = asDouble(line.substr( 0,20));
         fileType = strip(   line.substr(20,20));
         fileSys  = strip(   line.substr(40,20));

         if(fileSys[0] != 'M' && fileSys[0] != 'm') {
            RinexSatID sat;
            sat.fromString(fileSys);
            fileSysSat = SatID(sat);
         }
         else
            fileSysSat = SatID(-1,SatID::systemMixed);

         if(fileType[0] != 'O' && fileType[0] != 'o') {
            FFStreamError e("This isn't a RINEX 3 Obs file.");
            GPSTK_THROW(e);
         }

         valid |= validVersion;
      }
      else if(label == stringRunBy)
      {
         fileProgram = strip(line.substr( 0,20));
         fileAgency  = strip(line.substr(20,20));
         date        = strip(line.substr(40,20));
         valid |= validRunBy;
      }
      else if(label == stringComment)
      {
         commentList.push_back(strip(line.substr(0,60)));
         valid |= validComment;
      }
      else if(label == stringMarkerName)
      {
         markerName = strip(line.substr(0,60));
         valid |= validMarkerName;
      }
      else if(label == stringMarkerNumber)
      {
         markerNumber = strip(line.substr(0,20));
         valid |= validMarkerNumber;
      }
      else if(label == stringMarkerType)
      {
         markerType = strip(line.substr(0,20));
         valid |= validMarkerType;
      }
      else if(label == stringObserver)
      {
         observer = strip(line.substr( 0,20));
         agency   = strip(line.substr(20,40));
         valid |= validObserver;
      }
      else if(label == stringReceiver)
      {
         recNo   = strip(line.substr( 0,20));
         recType = strip(line.substr(20,20));
         recVers = strip(line.substr(40,20));
         valid |= validReceiver;
      }
      else if(label ==stringAntennaType)
      {
         antNo   = strip(line.substr( 0,20));
         antType = strip(line.substr(20,20));
         valid |= validAntennaType;
      }
      else if(label == stringAntennaPosition)
      {
         antennaPosition[0] = asDouble(line.substr( 0,14));
         antennaPosition[1] = asDouble(line.substr(14,14));
         antennaPosition[2] = asDouble(line.substr(28,14));
         valid |= validAntennaPosition;
      }
      else if(label == stringAntennaDeltaHEN)
      {
         antennaDeltaHEN[0] = asDouble(line.substr( 0,14));
         antennaDeltaHEN[1] = asDouble(line.substr(14,14));
         antennaDeltaHEN[2] = asDouble(line.substr(28,14));
         valid |= validAntennaDeltaHEN;
      }
      else if(label == stringAntennaDeltaXYZ)
      {
         antennaDeltaXYZ[0] = asDouble(line.substr( 0,14));
         antennaDeltaXYZ[1] = asDouble(line.substr(14,14));
         antennaDeltaXYZ[2] = asDouble(line.substr(28,14));
         valid |= validAntennaDeltaXYZ;
      }
      else if(label == stringAntennaPhaseCtr)
      {
         antennaSatSys  = strip(line.substr(0,2));
         antennaObsCode = strip(line.substr(2,3));
         antennaPhaseCtr[0] = asDouble(line.substr( 5, 9));
         antennaPhaseCtr[1] = asDouble(line.substr(14,14));
         antennaPhaseCtr[2] = asDouble(line.substr(28,14));
         valid |= validAntennaPhaseCtr;
      }
      else if(label == stringAntennaBsightXYZ)
      {
         antennaBsightXYZ[0] = asDouble(line.substr( 0,14));
         antennaBsightXYZ[1] = asDouble(line.substr(14,14));
         antennaBsightXYZ[2] = asDouble(line.substr(28,14));
         valid |= validAntennaBsightXYZ;
      }
      else if(label == stringAntennaZeroDirAzi)
      {
         antennaZeroDirAzi = asDouble(line.substr(0,14));
         valid |= validAntennaBsightXYZ;
      }
      else if(label == stringAntennaZeroDirXYZ)
      {
         antennaZeroDirXYZ[0] = asDouble(line.substr( 0,14));
         antennaZeroDirXYZ[1] = asDouble(line.substr(14,14));
         antennaZeroDirXYZ[2] = asDouble(line.substr(28,14));
         valid |= validAntennaBsightXYZ;
      }
      else if(label == stringCenterOfMass)
      {
         centerOfMass[0] = asDouble(line.substr( 0,14));
         centerOfMass[1] = asDouble(line.substr(14,14));
         centerOfMass[2] = asDouble(line.substr(28,14));
         valid |= validCenterOfMass;
      }
      else if(label == stringNumObs)        // R2 only
      {
         if(version >= 3) {
            FFStreamError e("RINEX 2 record in RINEX 3 file: " + label);
            GPSTK_THROW(e);
         }

         int pos;
         const int maxObsPerLine = 9;
         vector<string> newTypeList;

            // process the first line
         if(!(valid & validNumObs))
         {
            numObs = asInt(line.substr(0,6));

            for(i = 0; (i < numObs) && (i < maxObsPerLine); i++)
            {
               pos = i * 6 + 6 + 4;
               string ot(line.substr(pos,2));
               newTypeList.push_back(ot);
            }
            R2ObsTypes = newTypeList;              // erases what was already there
            valid |= validNumObs;
         }
            // process continuation lines
         else
         {
            newTypeList = R2ObsTypes;
            for(i = newTypeList.size();
                 (i < numObs) && ((i % maxObsPerLine) < maxObsPerLine); i++)
            {
               pos = (i % maxObsPerLine) * 6 + 6 + 4;
               string ot(line.substr(pos,2));
               newTypeList.push_back(ot);
            }
            R2ObsTypes = newTypeList;
         }
      }
      else if(label == stringSystemNumObs)
      {
         if(version < 3) {
            FFStreamError e("RINEX 3 record in RINEX 2 file: " + label);
            GPSTK_THROW(e);
         }

         static const int maxObsPerLine = 13;

         satSysTemp = strip(line.substr(0,1));
         numObs     = asInt(line.substr(3,3));

         try {
            if(satSysTemp == "" ) // it's a continuation line; use previous info.
            {
               satSysTemp = satSysPrev;
               numObs = numObsPrev;
               vector<RinexObsID> newTypeList = mapObsTypes.find(satSysTemp)->second;
               for(i = newTypeList.size();
                  (i < numObs) && ((i % maxObsPerLine) < maxObsPerLine); i++)
               {
                  int position = 4*(i % maxObsPerLine) + 6 + 1;
                  RinexObsID rt(satSysTemp+line.substr(position,3));
                  newTypeList.push_back(rt);
               }
               mapObsTypes[satSysTemp] = newTypeList;
            }
            else                    // it's a new line, use info. read in
            {
               vector<RinexObsID> newTypeList;
               for(i = 0; (i < numObs) && (i < maxObsPerLine); i++)
               {
                  int position = 4*i + 6 + 1;
                  RinexObsID rt(satSysTemp+line.substr(position,3));
                  newTypeList.push_back(rt);
               }
               mapObsTypes[satSysTemp] = newTypeList;
            }
         }
         catch(InvalidParameter& ip) {
            FFStreamError fse("InvalidParameter: "+ip.what());
            GPSTK_THROW(fse);
         }

         // save values in case next line is a continuation line
         satSysPrev = satSysTemp;
         numObsPrev = numObs;

         valid |= validSystemObsType;
      }
      else if(label == stringWaveFact)         // R2 only
      {
            // first time reading this
         if(!(valid & validWaveFact)) {
            wavelengthFactor[0] = asInt(line.substr(0,6));
            wavelengthFactor[1] = asInt(line.substr(6,6));
            valid |= validWaveFact;
         }
            // additional wave fact lines
         else {
            const int maxSatsPerLine = 7;
            int Nsats;
            ExtraWaveFact ewf;
            ewf.wavelengthFactor[0] = asInt(line.substr(0,6));
            ewf.wavelengthFactor[1] = asInt(line.substr(6,6));
            Nsats = asInt(line.substr(12,6));
               
            if(Nsats > maxSatsPerLine)   // > not >=
            {
               FFStreamError e("Invalid number of Sats for " + stringWaveFact);
               GPSTK_THROW(e);
            }
               
            for(i = 0; i < Nsats; i++) {
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
      else if(label == stringSigStrengthUnit)
      {
         sigStrengthUnit = strip(line.substr(0,20));
         valid |= validSigStrengthUnit;
      }
      else if(label == stringInterval)
      {
         interval = asDouble(line.substr(0,10));
         valid |= validInterval;
      }
      else if(label == stringFirstTime)
      {
         firstObs = parseTime(line);
         valid |= validFirstTime;
      }
      else if(label == stringLastTime)
      {
         lastObs = parseTime(line);
         valid |= validLastTime;
      }
      else if(label == stringReceiverOffset)
      {
         receiverOffset = asInt(line.substr(0,6));
         valid |= validReceiverOffset;
      }

      else if(label == stringSystemDCBSapplied)
      {
         Rinex3CorrInfo tempInfo;
         tempInfo.satSys = strip(line.substr( 0, 1));
         tempInfo.name   = strip(line.substr( 2,17));
         tempInfo.source = strip(line.substr(20,40));
         infoDCBS.push_back(tempInfo);
         valid |= validSystemDCBSapplied;
      }
      else if(label == stringSystemPCVSapplied)
      {
         Rinex3CorrInfo tempInfo;
         tempInfo.satSys = strip(line.substr( 0, 1));
         tempInfo.name   = strip(line.substr( 2,17));
         tempInfo.source = strip(line.substr(20,40));
         infoPCVS.push_back(tempInfo);
         valid |= validSystemPCVSapplied;
      }
      else if(label == stringSystemScaleFac)
      {
         static const int maxObsPerLine = 12;

         satSysTemp = strip(line.substr(0,1));
         factor     = asInt(line.substr(2,4));
         numObs     = asInt(line.substr(8,2));

         int startPosition = 0;

         if(satSysTemp == "" )
         {           // it's a continuation line; use prev. info., end pt. to start
            satSysTemp = satSysPrev;
            factor     = factorPrev;
            numObs     = numObsPrev;

            startPosition = sysSfacMap[satSysTemp].size();
         }

         // 0/blank numObs means factor applies to all obs types
         // in appropriate obsTypeList
         if(numObs == 0) numObs = mapObsTypes[satSysTemp].size();

         sfacMap tempSfacMap = sysSfacMap[satSysTemp];
         for(i = startPosition;
                        (i < numObs) && ((i % maxObsPerLine) < maxObsPerLine); i++)
         {
            int position = 4*(i % maxObsPerLine) + 10 + 1;
            RinexObsID tempType(satSysTemp+strip(line.substr(position,3)));
            tempSfacMap.insert(make_pair(tempType,factor));
         }
         sysSfacMap[satSysTemp] = tempSfacMap;

         sfacMap::const_iterator iter;
         sfacMap tempmap;
         tempmap = sysSfacMap[satSysTemp];

         // save values in case next line is a continuation line
         satSysPrev = satSysTemp;
         factorPrev = factor;
         numObsPrev = numObs;

         valid |= validSystemScaleFac;
      }
      else if(label == stringSystemPhaseShift) ///< "SYS / PHASE SHIFT"    R3.01
      {
         //map<string, map<RinexObsID, map<RinexSatID,double> > > sysPhaseShift;
         RinexSatID sat;
         // system
         satSysTemp = strip(line.substr(0,1));

         if(satSysTemp.empty()) {                  // continuation line
            satSysTemp = satSysPrev;

            if(sysPhaseShift[satSysTemp].find(sysPhaseShiftObsID)
                                          == sysPhaseShift[satSysTemp].end())
            {
               FFStreamError e("SYS / PHASE SHIFT: unexpected continuation line");
               GPSTK_THROW(e);
            }

            map<RinexSatID,double>& satcorrmap(sysPhaseShift[satSysTemp][sysPhaseShiftObsID]);
            double cor(sysPhaseShift[satSysTemp][sysPhaseShiftObsID].begin()->second);
            for(i=0; i<10; i++) {
               string str = strip(line.substr(19+4*i,3));
               if(str.empty()) break;
               sat = RinexSatID(str);
               satcorrmap.insert(make_pair(sat,cor));
            }
         }
         else {                                    // not a cont. line
            sat.fromString(satSysTemp);
            if(sysPhaseShift.find(satSysTemp) == sysPhaseShift.end()) {
               map<RinexObsID, map<RinexSatID, double> > obssatcormap;
               sysPhaseShift.insert(make_pair(satSysTemp,obssatcormap));
            }

            // obs id
            string str = strip(line.substr(2,3));

            // obsid and correction may be blank <=> unknown: ignore this
            if(!str.empty()) {
               RinexObsID obsid(satSysTemp+str);
               //cout << "Found Phase shift obsid " << satSysTemp << str
               //<< " " << obsid << endl;
               double cor(asDouble(strip(line.substr(6,8))));
               int nsat(asInt(strip(line.substr(16,2))));
               if(nsat > 0) {          // list of sats
                  map<RinexSatID,double> satcorrmap;
                  for(i=0; i<(nsat < 10 ? nsat : 10); i++) {
                     sat = RinexSatID(strip(line.substr(19+4*i,3)));
                     satcorrmap.insert(make_pair(sat,cor));
                  }
                  sysPhaseShift[satSysTemp].insert(make_pair(obsid,satcorrmap));
                  if(nsat > 10)        // expect continuation
                     sysPhaseShiftObsID = obsid;
               }
               else {                  // no sat, just system
                  map<RinexSatID,double> satcorrmap;
                  satcorrmap.insert(make_pair(sat,cor));
                  sysPhaseShift[satSysTemp].insert(make_pair(obsid,satcorrmap));
               }
            }

            // save for continuation lines
            satSysPrev = satSysTemp;

            valid |= validSystemPhaseShift;
         }
      }
      else if(label == stringGlonassSlotFreqNo)
      {
         //map<RinexSatID,int> GlonassFreqNo;
         int nsat, tmp;
         RinexSatID sat;
         string str(strip(line.substr(0,3)));
         nsat = asInt(str);         // not used!
         for(i=0; i<8; i++) {
            str = strip(line.substr(4+i*7,3));
            if(str.empty()) break;
            sat = RinexSatID(str);
            str = strip(line.substr(8+i*7,2));
            tmp = asInt(str);
            GlonassFreqNo.insert(make_pair(sat,tmp));
         }

         valid |= validGlonassFreqNo;
      }
      else if(label == stringGlonassCodPhsBias) {
         //std::map<RinexObsID,double> GlonassCodePhaseBias; ///< "GLONASS COD/PHS/BIS"            R3.02
         for(i=0; i<4; i++) {
            string str(strip(line.substr(i*13+1,3)));
            if(str.empty()) continue;
            RinexObsID obsid("R"+str);
            double bias(asDouble(strip(line.substr(i*13+5,8))));
            GlonassCodePhaseBias[obsid] = bias;
         }
         valid |= validGlonassCodPhsBias;
      }
      else if(label == stringLeapSeconds)
      {
         leapSeconds = asInt(line.substr(0,6));
         valid |= validLeapSeconds;
      }
      else if(label == stringNumSats)
      {
         numSVs = asInt(line.substr(0,6)) ;
         valid |= validNumSats;
      }
      else if(label == stringPrnObs)
      {
         // this assumes 'PRN / # OF OBS' comes after '# / TYPES OF OBSERV' or 'SYS / # / OBS TYPES'
         static const int maxObsPerLine = 9;

         int j,otmax;
         RinexSatID PRN;
         string prn, GNSS;
         vector<int> numObsList;

         prn = strip(line.substr(3,3));

         if(prn == "" ) // this is a continuation line; use last PRN
         {
            PRN = lastPRN;
            GNSS = PRN.systemChar();
            if(version < 3)
               otmax = R2ObsTypes.size();
            else {
               if(mapObsTypes.find(GNSS) == mapObsTypes.end()) {
                  Exception e("PRN/#OBS for system "+PRN.toString()+" not found in SYS/#/OBS");
                  GPSTK_THROW(e);
               }
               otmax = mapObsTypes[GNSS].size();
            }

            numObsList = numObsForSat[PRN]; // grab the existing list

            for(j=0,i=numObsList.size(); j<maxObsPerLine && i<otmax; i++,j++)
               numObsList.push_back(asInt(line.substr(6*j+6,6)));

            numObsForSat[PRN] = numObsList;
         }
         else             // this is a new PRN line
         {
            PRN = RinexSatID(prn);
            GNSS = PRN.systemChar();
            if(version < 3)
               otmax = R2ObsTypes.size();
            else {
               if(mapObsTypes.find(GNSS) == mapObsTypes.end()) {
                  Exception e("PRN/#OBS for system "+PRN.toString()+" not found in SYS/#/OBS");
                  GPSTK_THROW(e);
               }
               otmax = mapObsTypes[GNSS].size();
            }

            for(i=0; i<maxObsPerLine && i<otmax; i++)
               numObsList.push_back(asInt(line.substr(6*i+6,6)));

            numObsForSat[PRN] = numObsList;

            lastPRN = PRN;
         }

         //cout << "Sat " << PRN.toString() << " (" << numObsList.size() << "):";
         //for(i=0; i<numObsList.size(); i++) cout << " " << numObsList[i];
         //cout << endl;

         valid |= validPrnObs;
      }
      else if(label == stringEoH)
      {
         validEoH = true;
      }
      else
      {
         FFStreamError e("Unidentified label: >" + label + "<");
         GPSTK_THROW(e);
      }
   } // end of ParseHeaderRecord


   // This function parses the entire header from the given stream
   void Rinex3ObsHeader::reallyGetRecord(FFStream& ffs)
      throw(exception, FFStreamError, 
            gpstk::StringUtils::StringException)
   {
      Rinex3ObsStream& strm = dynamic_cast<Rinex3ObsStream&>(ffs);

      // If already read, just return.
      if(strm.headerRead == true) return;

      // Since we're reading a new header, we need to reinitialize all our list
      // structures. All the other objects should be ok.  This also applies if we
      // threw an exception the first time we read the header and are now re-reading
      // it.  Some of these could be full and we need to empty them.
      clear();

      string line;

      while (!validEoH)
      {
         strm.formattedGetLine(line);
         StringUtils::stripTrailing(line);

         if(line.length() == 0)
         {
            FFStreamError e("No data read");
            GPSTK_THROW(e);
         }
         else if(line.length() < 60 || line.length() > 80)
         {
            FFStreamError e("Invalid line length");
            GPSTK_THROW(e);
         }

         try
         {
//            std::cout << "Parse header record >" << line << "<" << std::endl;
            ParseHeaderRecord(line);
         }
         catch(FFStreamError& e)
         {
            GPSTK_RETHROW(e);
         }
         catch(Exception& e)
         {
            FFStreamError fse("Exception: "+e.what());
            GPSTK_THROW(fse);
         }

      } // end while(not end of header)

      // if RINEX 2, define mapObsTypes from R2ObsTypes and system(s)
      // this may have to be corrected later using wavelengthFactor
      // also define mapSysR2toR3ObsID in case version 2 is written out later
      if(version < 3) {
         // try to determine systems included in the file
         vector<string> syss;                // 1-char strings "G" "R" "E" ...
         if(numObsForSat.size() > 0) {       // get syss from PRN/#OBS
            map<RinexSatID, vector<int> >::const_iterator it;
            for(it=numObsForSat.begin(); it != numObsForSat.end(); ++it) {
               string sys(string(1,(it->first).systemChar()));
               //cout << "Sat " << it->first << " sys " << sys << endl;
               if(find(syss.begin(),syss.end(),sys) == syss.end())
                  syss.push_back(sys);
            }
         }
         else if(fileSysSat.system != SatID::systemMixed) {
            // only one system in this file
            syss.push_back(string(1,RinexSatID(fileSysSat).systemChar()));
         }
         else {
            // have to replicate obs type list for all RINEX2 systems
            syss.push_back("G");
            syss.push_back("R");
            syss.push_back("S");    // ??
            syss.push_back("E");
         }
         // are there any sats with non-1 wavelength factors? if so must add "N" types
         //bool haveL1WaveFact(false), haveL2WaveFact(false);
         //bool haveMultipleL1WaveFact(false), haveMultipleL2WaveFact(false);
         //if(find(syss.begin(),syss.end(),"G") != syss.end()) {     // has GPS
         //   if(wavelengthFactor[0] != 1) haveL1WaveFact = true;
         //   if(wavelengthFactor[1] != 1) haveL2WaveFact = true;
         //   for(int i=0; i < extraWaveFactList.size(); i++) {
         //      if(extraWaveFactList[i].wavelengthFactor[0] != 1)
         //         haveMultipleL1WaveFact = true;
         //      if(extraWaveFactList[i].wavelengthFactor[1] != 1)
         //         haveMultipleL2WaveFact = true;
         //   }
         //}

         // given systems and list of R2ObsTypes, compute mapObsTypes and mapSysR2toR3ObsID
         mapSysR2toR3ObsID.clear();
         for(size_t i=0; i<syss.size(); i++) {
            const string s(syss[i]);
            vector<RinexObsID> obsids;
            bool isPrecise(
               find(R2ObsTypes.begin(),R2ObsTypes.end(),"P1") != R2ObsTypes.end() ||
               find(R2ObsTypes.begin(),R2ObsTypes.end(),"P2") != R2ObsTypes.end()
            );
            // loop over R2 obs types
            for(size_t j=0; j<R2ObsTypes.size(); ++j) {
               string ot(R2ObsTypes[j]), obsid;

               // GPS+GLO 1+2
               // GPS and GLO (but GPS w/ wavelengthFactor -> tracking code N)
               // C1 L1 S1 D1     =>  C1C L1C S1C D1C          (C1 not P1)
               // P1 L1 S1 D1     =>  C1P L1P S1P D1P          (P1 not C1)
               // C1 P1 L1 S1 D1  =>  C1C C1P L1P S1P D1P
               // C2 L2 S2 D2     =>  C2C L2C S2C D2C          (C2 not P2)
               // P2 L2 S2 D2     =>  C2P L2P S2P D2P          (P2 not C2)
               // C2 P2 L2 S2 D2  =>  C2C C2P L2P S2P D2P
               if((s=="G" || s=="R") && (ot[1]=='1' || ot[1]=='2')) {
                  string type,tc;
                  if(ot[0] == 'C') { type = tc = "C"; }              // C1
                  else if(ot[0] == 'P') { type = "C"; tc = "P"; }    // P12
                  else { type = ot[0]; tc = (isPrecise ? "P":"C"); } // L12 S12 D12

                  // wavelengthFactor: all sats -> N replaces C/P, some -> N & C/P
                  // TD but this screws up the ordering of obs for Data::really*()
                  //if(ot[1]=='1' && haveMultipleL1WaveFact) {
                  //   OT = RinexObsID(s+type+string(1,ot[1])+tc);
                  //   obsids.push_back(OT);                           // have both
                  //   tc = "N"; // TD there is no C1N or C2N
                  //}
                  //if(ot[1]=='1' && haveL1WaveFact) tc = "N";
                  //if(ot[1]=='2' && haveMultipleL2WaveFact) {
                  //   OT = RinexObsID(s+type+string(1,ot[1])+tc);
                  //   obsids.push_back(OT);
                  //   tc = "N";
                  //}
                  //if(ot[1]=='2' && haveL2WaveFact) tc = "N";

                  obsid = string(s+type+string(1,ot[1])+tc);
               }

               // GPS 5
               // C5 L5 S5 D5     =>  C5X L5X S5X D5X
               else if(s == "G" && ot[1] == '5') {
                  if(ot == "C5") obsid = string("GC5X");            // C5
                  else obsid = string(s+ot+(isPrecise ? "X" : "C"));// L5 S5 D5
               }

               // GAL
               // C1 L1 S1 D1     =>  C1C L1C S1C D1C (E2-L1-E1)
               // C5 L5 S5 D5     =>  C5X L5X S5X D5X (E5a)
               // C6 L6 S6 D6     =>  C6X L6X S6X D6X (E6)
               // C7 L7 S7 D7     =>  C7X L7X S7X D7X (E5b)
               // C8 L8 S8 D8     =>  C8X L8X S8X D8X (E5a+b)
               else if(s == "E") {
                  if(ot[0] != 'P' && ot[1] != '2')   // CLDS x 15678
                     obsid = string(s+ot+(ot[1]=='1' ? "C" : "X"));
                  //else obsid = string("EC1*");      // (P*,*2) not allowed
               }

               // SBAS / GEO
               // C1 L1 D1        =>  C1C L1C D1C S1C
               // C5 L5 D5        =>  C5X L5X D5X S5X
               else if(s == "S") {
                       if(ot == "C1") obsid = string("SC1C");
                  else if(ot == "L1") obsid = string("SL1C");
                  else if(ot == "D1") obsid = string("SD1C");
                  else if(ot == "S1") obsid = string("SS1C");
                  else if(ot == "C5") obsid = string("SC5X");
                  else if(ot == "L5") obsid = string("SL5X");
                  else if(ot == "D5") obsid = string("SD5X");
                  else if(ot == "S5") obsid = string("SS5X");
               }

               // create the obs id and save it
               if(!obsid.empty()) {
                  RinexObsID OT;
                  try {
                     OT = RinexObsID(obsid);
                  }
                  catch(InvalidParameter& ip) {
                     FFStreamError fse("InvalidParameter: "+ip.what());
                     GPSTK_THROW(fse);
                  }

                  obsids.push_back(OT);
                  mapSysR2toR3ObsID[syss[i]][ot] = OT; //map<string, map<string, RinexObsID> >
               }

            }  // end for

            // TD if GPS and have wavelengthFactors, add more ObsIDs with tc=N

            mapObsTypes[syss[i]] = obsids;
         }

         // modify numObsForSat if necessary
         map<RinexSatID, vector<int> >::const_iterator it(numObsForSat.begin());
         for( ; it != numObsForSat.end(); ++it) {
            RinexSatID sat(it->first);
            string sys;
            sys = sat.systemChar();
            vector<int> vec;
            for(size_t i=0; i<R2ObsTypes.size(); i++) {
               if(mapSysR2toR3ObsID[sys][R2ObsTypes[i]].asString() == string("   "))
                  ;
               else
                  vec.push_back(it->second[i]);
            }
            numObsForSat[sat] = vec;
         }

         // TEMP?
         ////std::vector<std::string> R2ObsTypes;
         ////map<string, map<string, RinexObsID> > mapSysR2toR3ObsID;
         //map<string, map<string, RinexObsID> >::iterator jt;
         //sort(R2ObsTypes.begin(), R2ObsTypes.end());
         //cout << "Read (" << R2ObsTypes.size() << ") RINEX ver. 2 Obs Types:";
         //for(int i=0; i<R2ObsTypes.size(); i++) cout << " " << R2ObsTypes[i];
         //cout << endl;

         //for(jt = mapSysR2toR3ObsID.begin(); jt != mapSysR2toR3ObsID.end(); ++jt) {
         //   cout << "R2->R3 Map for sys " << jt->first << " :";
         //   for(int i=0; i<R2ObsTypes.size(); i++)
         //      cout << " " << R2ObsTypes[i]
         //         << ":" << jt->second[R2ObsTypes[i]].asString();
         //   cout << endl;
         //}
      }

      // Since technically the Phase Shift record is required in ver 3.01,
      // create SystemPhaseShift record(s) if not present.
      //map<string, map<RinexObsID, map<RinexSatID,double> > > sysPhaseShift;
      if(version >= 3.01 && (valid & validSystemObsType)
                         && !(valid & validSystemPhaseShift)) {
         // loop over obs types to get systems
         map<string,vector<RinexObsID> >::const_iterator iter;
         for(iter=mapObsTypes.begin(); iter != mapObsTypes.end(); iter++) {
            string sys(iter->first);
            if(sysPhaseShift.find(sys) == sysPhaseShift.end()) {
               map<RinexObsID, map<RinexSatID, double> > dummy;
               sysPhaseShift.insert(make_pair(sys,dummy));
            }
         }
         valid |= validSystemPhaseShift;
      }

      // is the header valid?
      unsigned long allValid;
      if     (version <  3  )  allValid = allValid2;
      else if(version == 3.0)  allValid = allValid30;
      else if(version == 3.01) allValid = allValid301;
      else if(version == 3.02) allValid = allValid302;
      else
      {
         FFStreamError e("Unknown or unsupported RINEX version " + 
                         asString(version));
         GPSTK_THROW(e);
      }

      if((valid & allValid) != allValid)
      {
         FFStreamError e("Incomplete or invalid header");
         GPSTK_THROW(e);
      }

      // If we get here, we should have reached the end of header line.
      strm.header = *this;
      strm.headerRead = true;

      // determine the time system of epochs in this file; cf. R3.02 Table A2
      // 1.determine time system from time tag in TIME OF FIRST OBS record
      // 2.if not given, determine from type in RINEX VERSION / TYPE record
      // 3.(if the type is MIXED, the time system in firstObs is required by RINEX)
      strm.timesystem = firstObs.getTimeSystem();
      if(strm.timesystem == TimeSystem::Any ||
         strm.timesystem == TimeSystem::Unknown)
      {
         if(fileSysSat.system == SatID::systemGPS)
            strm.timesystem = TimeSystem::GPS;
         else if(fileSysSat.system == SatID::systemGlonass)
            strm.timesystem = TimeSystem::UTC;
         else if(fileSysSat.system == SatID::systemGalileo)
            strm.timesystem = TimeSystem::GAL;
         else if(fileSysSat.system == SatID::systemQZSS)
            strm.timesystem = TimeSystem::QZS;
         else if(fileSysSat.system == SatID::systemBeiDou)
            strm.timesystem = TimeSystem::BDT;
         else if(fileSysSat.system == SatID::systemMixed) {
            FFStreamError e("TimeSystem in MIXED files must be given by first obs");
            GPSTK_THROW(e);
         }
         else {
            FFStreamError e("Unknown file system type");
            GPSTK_THROW(e);
         }
      }

   } // end reallyGetRecord


   CivilTime Rinex3ObsHeader::parseTime(const string& line) const
   {
      int year, month, day, hour, min;
      double sec;
      string tsys;
      TimeSystem ts;
   
      year  = asInt(   line.substr(0,   6));
      month = asInt(   line.substr(6,   6));
      day   = asInt(   line.substr(12,  6));
      hour  = asInt(   line.substr(18,  6));
      min   = asInt(   line.substr(24,  6));
      sec   = asDouble(line.substr(30, 13));
      tsys  =          line.substr(48,  3) ;

      ts.fromString(tsys);

      return CivilTime(year, month, day, hour, min, sec, ts);
   } // end parseTime


   string Rinex3ObsHeader::writeTime(const CivilTime& civtime) const
   {
      string line;

      line  = rightJustify(asString<short>(civtime.year    )   ,  6);
      line += rightJustify(asString<short>(civtime.month   )   ,  6);
      line += rightJustify(asString<short>(civtime.day     )   ,  6);
      line += rightJustify(asString<short>(civtime.hour    )   ,  6);
      line += rightJustify(asString<short>(civtime.minute  )   ,  6);
      line += rightJustify(asString(       civtime.second,7)   , 13);
      line += rightJustify((civtime.getTimeSystem()).asString(),  8);

      return line;
   } // end writeTime

   // Compute map of obs types for use in writing version 2 header and data, call before writing
   void Rinex3ObsHeader::PrepareVer2Write(void) throw()
   {
      size_t i;

      version = 2.11;
      valid |= Rinex3ObsHeader::validWaveFact;
      // TD unset R3-specific header members?

      // define these two:
      //std::vector<std::string> R2ObsTypes;
      //map<string, map<string, RinexObsID> > mapSysR2toR3ObsID;
      map<string, map<string, RinexObsID> >::iterator jt;

      // if map is already defined, it was created during reallyGet(version 2)
      if(mapSysR2toR3ObsID.size() == 0) {
         // make a list of R2 obstype strings, and a map R3ObsIDs <= R2 obstypes for each system
         R2ObsTypes.clear();
         map<string,vector<RinexObsID> >::const_iterator mit;
         for(mit = mapObsTypes.begin(); mit != mapObsTypes.end(); mit++) {
            // mit->first is system char as a 1-char string
            //cout << "Sys is " << mit->first << endl;
            map<string, RinexObsID> mapR2toR3ObsID;

            // loop over all ObsIDs for this system
            for(i=0; i<mit->second.size(); i++) {
               string R2ot, lab(mit->second[i].asString());
               // the list of all tracking code characters for this sys, freq
               string allCodes(ObsID::validRinexTrackingCodes[mit->first[0]][lab[1]]);

               //cout << " ObsID " << lab;
                    if(lab == string("C1C")) R2ot = string("C1");
               else if(lab == string("C2C")) R2ot = string("C2");
               else if(lab.substr(0,2) == "C5") R2ot = string("C5");    // R2 has C5 but not P5
               else if(lab[0] == 'C')        R2ot = string("P")+string(1,lab[1]);
               else                          R2ot = lab.substr(0,2);
               //cout << " => R2ot " << R2ot << endl;

               // add to list, if not already there
               vector<string>::iterator it;
               it = find(R2ObsTypes.begin(),R2ObsTypes.end(),R2ot);
               if(it == R2ObsTypes.end()) {    // its not there - add it
                  R2ObsTypes.push_back(R2ot);
                  mapR2toR3ObsID[R2ot] = mit->second[i];
               }
               else {                     // its already there - in list of R2 ots
                  if(mapR2toR3ObsID.find(R2ot) == mapR2toR3ObsID.end()) {
                     mapR2toR3ObsID[R2ot] = mit->second[i];// must also add to sys map
                  }
                  else {                              // its already in sys map ...
                     // .. but is the new tc 'better'?
                     string::size_type posold,posnew;
                     posold = allCodes.find((mapR2toR3ObsID[R2ot].asString())[2]);
                     posnew = allCodes.find(lab[2]);
                     if(posnew < posold)           // replace the R3ObsID in the map
                        mapR2toR3ObsID[R2ot] = mit->second[i];
                  }
               }
            }
            // save for this system
            mapSysR2toR3ObsID[mit->first] = mapR2toR3ObsID;
         }
      }  // end if mapSysR2toR3ObsID is defined already
      // else version 2 was read and R2ObsTypes and mapSysR2toR3ObsID were filled in reallyGet

      // TEMP?
      //sort(R2ObsTypes.begin(), R2ObsTypes.end());
      //cout << "Prepare to write (" << R2ObsTypes.size()<< ") RINEX ver.2 ObsTypes:";
      //for(i=0; i<R2ObsTypes.size(); i++) cout << " " << R2ObsTypes[i];
      //cout << endl;

      //for(jt = mapSysR2toR3ObsID.begin(); jt != mapSysR2toR3ObsID.end(); ++jt) {
      //   cout << "R3->R2 Map for sys " << jt->first << " :";
      //   for(i=0; i<R2ObsTypes.size(); i++)
      //      cout << " " << R2ObsTypes[i]
      //         << ":" << jt->second[R2ObsTypes[i]].asString();
      //   cout << endl;
      //}

   }  // end PrepareVer2Write()

   void Rinex3ObsHeader::dump(ostream& s) const
   {
      size_t i;

      string str;
      if(fileSysSat.system == SatID::systemMixed)
         str = "MIXED";
      else {
         RinexSatID sat(fileSysSat);
         str = sat.systemChar();
         str = str + " (" + sat.systemString() + ")";
      }

      s << "---------------------------------- REQUIRED "
        << "----------------------------------" << endl;
      s << "Rinex Version " << fixed << setw(5) << setprecision(2) << version
        << ",  File type " << fileType << ",  System " << str << "." << endl;
      s << "Prgm: " << fileProgram << ",  Run: " << date
         << ",  By: " << fileAgency << endl;
      s << "Marker name: " << markerName << ", ";
      s << "Marker type: " << markerType << "." << endl;
      s << "Observer : " << observer << ",  Agency: " << agency << endl;
      s << "Rec#: " << recNo << ",  Type: " << recType
         << ",  Vers: " << recVers << endl;
      s << "Antenna # : " << antNo << ",  Type : " << antType << endl;
      s << "Position      (XYZ,m) : " << setprecision(4) << antennaPosition
         << "." << endl;
      s << "Antenna Delta (HEN,m) : " << setprecision(4) << antennaDeltaHEN
         << "." << endl;
      map<string,vector<RinexObsID> >::const_iterator iter;
      for(iter = mapObsTypes.begin(); iter != mapObsTypes.end(); iter++)
      {
         RinexSatID rsid;
         rsid.fromString(iter->first);
         s << rsid.systemString() << " Observation types ("
            << iter->second.size() << "):" << endl;
         for(i = 0; i < iter->second.size(); i++) 
            s << " Type #" << setw(2) << setfill('0') << i+1 << setfill(' ')
            << " (" << iter->second[i].asString() << ") "
              << asString(static_cast<ObsID>(iter->second[i])) << endl;
      }
      s << "Time of first obs "
         << printTime(firstObs,"%04Y/%02m/%02d %02H:%02M:%06.3f %P") << endl;

      unsigned long allValid = 0;
      if     (version == 3.0)   allValid = allValid30;
      else if(version == 3.01)  allValid = allValid301;
      else if(version == 3.02)  allValid = allValid302;

      s << "(This header is ";
      if((valid & allValid) == allValid)
         s << "VALID)" << endl;
      else {
         s << "NOT VALID";
         s << " RINEX " << setprecision(2) << version << ")" << endl;
         s << "valid    = " << hex << setw(8) << valid << endl;
         s << "allValid = " << hex << setw(8) << allValid << endl;
         s << "~v & aV  = " << hex << setw(8) << (~valid & allValid) << endl << dec;

         s << "Invalid header records:" << endl;
         if(!(valid & validVersion)) s << " Version / Type\n";
         if(!(valid & validRunBy)) s << " Pgm / Run By / Date\n";
         if(!(valid & validMarkerName)) s << " Marker Name\n";
         //if(version >= 3 && !(valid & validMarkerType)) s << "Marker Type\n";
         // Not actually required in 3.02 - see note in table A2 of R3.02 doc
         if(!(valid & validObserver)) s << " Observer / Agency\n";
         if(!(valid & validReceiver)) s << " Receiver # / Type\n";
         if(!(valid & validAntennaType)) s << " Antenna Type\n";
         if(!(valid & validAntennaPosition)) s << " Antenna Position\n";
         if(!(valid & validAntennaDeltaHEN)) s << " Antenna Delta HEN\n";
         if(version < 3 && !(valid & validNumObs)) s << " # / TYPES OF OBSERV\n";
         if(version >= 3 && !(valid & validSystemObsType  )) s << " Sys / # / Obs Type\n";
         if(!(valid & validFirstTime)) s << " Time of First Obs\n";
         if(!(valid & validSystemPhaseShift)) s << " Sys / Phase Shifts\n";
         if(version >= 3.01 && !(valid & validSystemPhaseShift)) s << " SYS / PHASE SHIFT\n";
         if(version >= 3.02 && !(valid & validGlonassFreqNo)) s << " GLONASS SLOT / FRQ #\n";
         if(version >= 3.02 && !(valid & validGlonassCodPhsBias)) s << " GLONASS COD/PHS/BIS\n";
         if(!(validEoH)) s << " END OF HEADER\n";
         s << "END Invalid header records." << endl;
      }

      s << "---------------------------------- OPTIONAL "
        << "----------------------------------" << endl;
      if(valid & validMarkerNumber     )
         s << "Marker number : " << markerNumber << endl;
      if(valid & validMarkerType       )
         s << "Marker type : " << markerType << endl;
      if(valid & validAntennaDeltaXYZ  )
         s << "Antenna Delta    (XYZ,m) : "
           << setprecision(4) << antennaDeltaXYZ   << endl;
      if(valid & validAntennaPhaseCtr  )
         s << "Antenna PhaseCtr (XYZ,m) : "
           << setprecision(4) << antennaPhaseCtr   << endl;
      if(valid & validAntennaBsightXYZ )
         s << "Antenna B.sight  (XYZ,m) : "
           << setprecision(4) << antennaBsightXYZ  << endl;
      if(valid & validAntennaZeroDirAzi)
         s << "Antenna ZeroDir  (deg)   : "
           << setprecision(4) << antennaZeroDirAzi << endl;
      if(valid & validAntennaZeroDirXYZ)
         s << "Antenna ZeroDir  (XYZ,m) : "
           << setprecision(4) << antennaZeroDirXYZ << endl;
      if(valid & validCenterOfMass     )
         s << "Center of Mass   (XYZ,m) : "
           << setprecision(4) << antennaPhaseCtr   << endl;
      if(valid & validSigStrengthUnit  )
         s << "Signal Strenth Unit = " << sigStrengthUnit << endl;
      if(valid & validInterval         )
         s << "Interval = "
           << fixed << setw(7) << setprecision(3) << interval << endl;
      if(valid & validLastTime         )
         s << "Time of Last Obs "
           << printTime(lastObs,"%04Y/%02m/%02d %02H:%02M:%06.3f %P") << endl;
      if(valid & validReceiverOffset   )
         s << "Clock offset record is present and offsets "
           << (receiverOffset ? "ARE" : "are NOT") << " applied." << endl;
      if(version < 3 && (valid & validWaveFact)) // TD extraWaveFactList
         s << "Wavelength factor L1: " << wavelengthFactor[0]
                            << " L2: " << wavelengthFactor[1] << endl;
      if(valid & validSystemDCBSapplied)
      {
         for(i = 0; i < infoDCBS.size(); i++)
         {
            RinexSatID rsid;
            rsid.fromString(infoDCBS[i].satSys);
            s << "System DCBS Correction Applied to " << rsid.systemString()
              << " data using program " << infoDCBS[i].name << endl;
            s << " from source " << infoDCBS[i].source << "." << endl;
         }
      }
      if(valid & validSystemPCVSapplied)
      {
         for(i = 0; i < infoPCVS.size(); i++)
         {
            RinexSatID rsid;
            rsid.fromString(infoPCVS[i].satSys);
            s << "System PCVS Correction Applied to " << rsid.systemString()
              << " data using program " << infoPCVS[i].name << endl;
            s << " from source " << infoPCVS[i].source << "." << endl;
         }
      }
      if(valid & validSystemScaleFac   )
      {
         map<string, sfacMap>::const_iterator mapIter;
         // loop over GNSSes
         for(mapIter = sysSfacMap.begin(); mapIter != sysSfacMap.end(); mapIter++)
         {
            RinexSatID rsid;
            rsid.fromString(mapIter->first);
            s << rsid.systemString() << " scale factors applied:" << endl;
            map<RinexObsID,int>::const_iterator iter;
            // loop over scale factor map
            for(iter = mapIter->second.begin(); iter != mapIter->second.end(); iter++)
               s << "   " << iter->first.asString() << " " << iter->second << endl;
         }
      }
      if(valid & validSystemPhaseShift )
      {
         map<string, map<RinexObsID, map<RinexSatID,double> > >::const_iterator it;
         for(it=sysPhaseShift.begin(); it!=sysPhaseShift.end(); ++it) {
            string sys(it->first);
            map<RinexObsID, map<RinexSatID, double> >::const_iterator jt;
            jt = it->second.begin();
            if(jt == it->second.end())
               s << "Phase shift correction for system " << sys << " is empty." << endl;
            for( ; jt!=it->second.end(); ++jt) {
               map<RinexSatID,double>::const_iterator kt;
               for(kt=jt->second.begin(); kt!=jt->second.end(); ++kt)
                  s << "Phase shift correction for system " << sys << ": "
                     << fixed << setprecision(5)
                     << setw(8) << kt->second << " cycles applied to obs type "
                     << jt->first.asString() << " "
                     << RinexSatID(sys).systemString() << endl;
            }
         }
      }
      if(valid & validGlonassFreqNo) {
         int n(0);
         map<RinexSatID,int>::const_iterator it;
         s << "GLONASS frequency channels:\n";
         for(it=GlonassFreqNo.begin(); it!=GlonassFreqNo.end(); ++it) {
            s << " " << it->first.toString() << " " << setw(2) << it->second;
            if(++n > 1 && (n%8)==0) s << endl;
         }
         if((n%8) != 0) s << endl;
      }
      if(valid & validGlonassCodPhsBias) {
         map<RinexObsID,double>::const_iterator it;
         s << "GLONASS Code-phase biases:\n" << fixed << setprecision(3);
         for(it=GlonassCodePhaseBias.begin(); it!=GlonassCodePhaseBias.end(); ++it)
            s << " " << it->first.asString() << " " << setw(8) << it->second;
         s << endl;
      }
      if(valid & validLeapSeconds)
         s << "Leap seconds: " << leapSeconds << endl;
      if(valid & validNumSats)
         s << "Number of Satellites with data : " << numSVs << endl;
      if(valid & validPrnObs) {
         RinexSatID sat, sys(-1,SatID::systemUnknown);
         s << " PRN and number of observations for each obs type:" << endl;
         map<RinexSatID, vector<int> >::const_iterator it = numObsForSat.begin();
         while (it != numObsForSat.end()) {
            sat = it->first;
            if(sat.system != sys.system) {               // print a header: SYS  OT  OT  OT ...
               s << " " << sat.systemString3() << " ";
               iter = mapObsTypes.find(string(1,sat.systemChar()));
               const vector<RinexObsID>& vec(iter->second);
               for(i=0; i<vec.size(); i++)
                  s << setw(7) << vec[i].asString();
               s << endl;
               sys = sat;
            }
            vector<int> obsvec = it->second;
            s << " " << sat.toString() << " ";
            for(i = 0; i < obsvec.size(); i++)           // print the numbers of obss
              s << " " << setw(6) << obsvec[i];
            s << endl;
            it++;
         }
      }
      if(commentList.size()) {
         if(!(valid & validComment)) s << " Comment list is NOT valid" << endl;
         s << "Comments (" << commentList.size() << ") :" << endl;
         for(i=0; i<commentList.size(); i++) s << commentList[i] << endl;
      }

      s << "-------------------------------- END OF HEADER "
        << "--------------------------------" << endl;
   } // end dump


      /* This method returns the numerical index of a given observation
       *
       * @param type String representing the observation type.
       */
   int Rinex3ObsHeader::getObsIndex( std::string type ) const
      throw(InvalidRequest)
   {

         // 'old-style' type: Let's change it to 'new style'.
      if( type.size() == 2 )
      {

         if( type == "C1" ) type = "C1C";
         else if( type == "P1" ) type = "C1P";
         else if( type == "L1" ) type = "L1P";
         else if( type == "D1" ) type = "D1P";
         else if( type == "S1" ) type = "S1P";
         else if( type == "C2" ) type = "C2C";
         else if( type == "P2" ) type = "C2P";
         else if( type == "L2" ) type = "L2P";
         else if( type == "D2" ) type = "D2P";
         else if( type == "S2" ) type = "S2P";
         else
         {
            InvalidRequest exc("Invalid type.");
            GPSTK_THROW(exc);
         }
      }

         // Add GNSS code. By default the system is GPS
      if( type.size() == 3 )
      {
         type = "G" + type;
      }

         // Check if resulting 'type' is valid
      if( !isValidRinexObsID(type) )
      {
         InvalidRequest ir(type + " is not a valid RinexObsID!.");
         GPSTK_THROW(ir);
      }

         // Extract the GNSS from the type
      string sysStr( type, 0, 1 );
      
         // Create a RinexObsID object from current type
      RinexObsID robs(type);

         // We need to look for the GNSS in the map
      map<std::string,vector<RinexObsID> >::const_iterator it;
      it = mapObsTypes.find(sysStr);

         // Check if GNSS was found
      if( it == mapObsTypes.end() )
      {
         InvalidRequest ir(sysStr + " is not a valid GNSS!.");
         GPSTK_THROW(ir);
      }

         // Extract a copy of the vector of observations types
      vector<RinexObsID> vecObs(it->second);

      size_t index(0);
      bool found(false);
      while( !found && index < vecObs.size() )
      {
         found = ( vecObs[index] == robs );
         index++;
      }
      --index;

         // This observation type is not stored
      if( !found )
      {
         InvalidRequest ir(type + " RinexObsID is not stored!.");
         GPSTK_THROW(ir);
      }

      return index;

   }  // End of method 'Rinex3ObsHeader::getObsIndex()'


} // namespace gpstk
