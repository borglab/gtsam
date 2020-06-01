/**
 * @file Rinex3NavHeader.cpp
 * Encapsulate header of RINEX 3 navigation file, including RINEX 2
 * compatibility.
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

#include "StringUtils.hpp"
#include "GNSSconstants.hpp"
#include "SystemTime.hpp"
#include "CivilTime.hpp"
#include "GPSWeekSecond.hpp"
#include "GALWeekSecond.hpp"
#include "TimeString.hpp"
#include "Rinex3NavHeader.hpp"
#include "Rinex3NavStream.hpp"

#include <iostream>

using namespace gpstk::StringUtils;
using namespace std;

namespace gpstk
{
   const string Rinex3NavHeader::stringVersion     = "RINEX VERSION / TYPE";
   const string Rinex3NavHeader::stringRunBy       = "PGM / RUN BY / DATE";
   const string Rinex3NavHeader::stringComment     = "COMMENT";
   const string Rinex3NavHeader::stringIonoCorr    = "IONOSPHERIC CORR";
   const string Rinex3NavHeader::stringTimeSysCorr = "TIME SYSTEM CORR";
   const string Rinex3NavHeader::stringLeapSeconds = "LEAP SECONDS";
   const string Rinex3NavHeader::stringCorrSysTime = "CORR TO SYSTEM TIME"; //R2.10GLO
   const string Rinex3NavHeader::stringDeltaUTC    = "DELTA-UTC: A0,A1,T,W";//R2.11GPS
   const string Rinex3NavHeader::stringDUTC        = "D-UTC A0,A1,T,W,S,U"; //R2.11GEO
   const string Rinex3NavHeader::stringIonAlpha    = "ION ALPHA";           //R2.11
   const string Rinex3NavHeader::stringIonBeta     = "ION BETA";            //R2.11
   const string Rinex3NavHeader::stringEoH         = "END OF HEADER";

   //--------------------------------------------------------------------------
   void Rinex3NavHeader::reallyGetRecord(FFStream& ffs) 
      throw(std::exception, FFStreamError, StringException)
   {
      Rinex3NavStream& strm = dynamic_cast<Rinex3NavStream&>(ffs);
   
      // if already read, just return
      if(strm.headerRead == true) return;
   
      int i;
      valid = 0;
   
      // clear out anything that was unsuccessfully read first
      commentList.clear();
   
      while (!(valid & validEoH)) {
         string line;
         strm.formattedGetLine(line);
         stripTrailing(line);
   
         if(line.length() == 0) continue;
         else if(line.length() < 60 || line.length() > 80) {
            FFStreamError e("Invalid line length");
            GPSTK_THROW(e);
         }
   
         string thisLabel(line, 60, 20);

         // following is huge if else else ... endif for each record type
         if(thisLabel == stringVersion) {                // "RINEX VERSION / TYPE"
            version = asDouble(line.substr( 0,20));

            fileType = strip(line.substr(20,20));
            if(version >= 3) {                        // ver 3
               if(fileType[0] != 'N' && fileType[0] != 'n') {
                  FFStreamError e("File type is not NAVIGATION: " + fileType);
                  GPSTK_THROW(e);
               }
               fileSys = strip(line.substr(40,20));   // not in ver 2
               setFileSystem(fileSys);
            }
            else {                                    // ver 2
               if(fileType[0] == 'N' || fileType[0] == 'n')
                  setFileSystem("G");
               else if(fileType[0] == 'G' || fileType[0] == 'g')
                  setFileSystem("R");
               else if(fileType[0] == 'H' || fileType[0] == 'h')
                  setFileSystem("S");
               else {
                  FFStreamError e("Version 2 file type is invalid: " + fileType);
                  GPSTK_THROW(e);
               }
            }

            fileType = "NAVIGATION";
            valid |= validVersion;
         }

         else if(thisLabel == stringRunBy) {                // "PGM / RUN BY / DATE"
            fileProgram = strip(line.substr( 0,20));
            fileAgency = strip(line.substr(20,20));
            date = strip(line.substr(40,20));         // R2 may not have 'UTC' at end
            valid |= validRunBy;
         }

         else if(thisLabel == stringComment) {                       // "COMMENT"
            commentList.push_back(strip(line.substr(0,60)));
            valid |= validComment;
         }

         else if(thisLabel == stringIonAlpha) {       // GPS alpha "ION ALPHA"  R2.11
            IonoCorr ic("GPSA");
            for(i=0; i < 4; i++)
               ic.param[i] = for2doub(line.substr(2 + 12*i, 12));
            mapIonoCorr[ic.asString()] = ic;
            if(mapIonoCorr.find("GPSB") != mapIonoCorr.end())
               valid |= validIonoCorrGPS;
         }
         else if(thisLabel == stringIonBeta) {        // GPS beta "ION BETA"  R2.11
            IonoCorr ic("GPSB");
            for(i=0; i < 4; i++)
               ic.param[i] = for2doub(line.substr(2 + 12*i, 12));
            mapIonoCorr[ic.asString()] = ic;
            if(mapIonoCorr.find("GPSA") != mapIonoCorr.end())
               valid |= validIonoCorrGPS;
         }
         else if(thisLabel == stringIonoCorr) {                // "IONOSPHERIC CORR"
            IonoCorr ic;
            try { ic.fromString(strip(line.substr(0,4))); }
            catch(Exception& e) {
               FFStreamError fse(e.what());
               GPSTK_THROW(e);
            }
            for(i=0; i < 4; i++)
               ic.param[i] = for2doub(line.substr(5 + 12*i, 12));

            if(ic.type == IonoCorr::GAL) {
               valid |= validIonoCorrGal;
            }
            else if(ic.type == IonoCorr::GPSA) {
               if(mapIonoCorr.find("GPSB") != mapIonoCorr.end())
                  valid |= validIonoCorrGPS;
            }
            else if(ic.type == IonoCorr::GPSB) {
               if(mapIonoCorr.find("GPSA") != mapIonoCorr.end())
                  valid |= validIonoCorrGPS;
            }
            //else
            mapIonoCorr[ic.asString()] = ic;
         }

         else if(thisLabel == stringDeltaUTC) {   // "DELTA-UTC: A0,A1,T,W" R2.11 GPS
            TimeSystemCorrection tc("GPUT");
            tc.A0 = for2doub(line.substr(3,19));
            tc.A1 = for2doub(line.substr(22,19));
            tc.refSOW = asInt(line.substr(41,9));
            tc.refWeek = asInt(line.substr(50,9));
            tc.geoProvider = string("    ");
            tc.geoUTCid = 0;

            mapTimeCorr[tc.asString4()] = tc;
            valid |= validTimeSysCorr;
         }
         // R2.11 but Javad uses it in 3.01
         else if(thisLabel == stringCorrSysTime) { // "CORR TO SYSTEM TIME"  R2.10 GLO
            TimeSystemCorrection tc("GLUT");
            tc.refYr = asInt(line.substr(0,6));
            tc.refMon = asInt(line.substr(6,6));
            tc.refDay = asInt(line.substr(12,6));
            tc.A0 = -for2doub(line.substr(21,19));    // -TauC

            // convert to week,sow
            CivilTime ct(tc.refYr,tc.refMon,tc.refDay,0,0,0.0);
            GPSWeekSecond gws(ct);
            tc.refWeek = gws.week;
            tc.refSOW = gws.sow;

            tc.A1 = 0.0;
            tc.geoProvider = string("    ");
            tc.geoUTCid = 3;                          // UTC(SU)

            mapTimeCorr[tc.asString4()] = tc;
            valid |= validTimeSysCorr;
         }
         else if(thisLabel == stringDUTC) {     // "D-UTC A0,A1,T,W,S,U"  // R2.11 GEO
            TimeSystemCorrection tc("SBUT");
            tc.A0 = for2doub(line.substr(0,19));
            tc.A1 = for2doub(line.substr(19,19));
            tc.refSOW = asInt(line.substr(38,7));
            tc.refWeek = asInt(line.substr(45,5));
            tc.geoProvider = line.substr(51,5);
            tc.geoUTCid = asInt(line.substr(57,2));

            mapTimeCorr[tc.asString4()] = tc;
            valid |= validTimeSysCorr;
         }
         else if(thisLabel == stringTimeSysCorr) {  // R3 only // "TIME SYSTEM CORR"
            TimeSystemCorrection tc;
            try { tc.fromString(strip(line.substr(0,4))); }
            catch(Exception& e) {
               FFStreamError fse(e.what());
               GPSTK_THROW(e);
            }

            tc.A0 = for2doub(line.substr(5,17));
            tc.A1 = for2doub(line.substr(22,16));
            tc.refSOW = asInt(line.substr(38,7));
            tc.refWeek = asInt(line.substr(45,5));
            tc.geoProvider = strip(line.substr(51,6));
            tc.geoUTCid = asInt(line.substr(57,2));

            if(tc.type == TimeSystemCorrection::GLGP ||
               tc.type == TimeSystemCorrection::GLUT ||        // TD ?
               tc.type == TimeSystemCorrection::BDUT ||        // TD ?
               tc.type == TimeSystemCorrection::GPUT ||
               tc.type == TimeSystemCorrection::GPGA ||
               tc.type == TimeSystemCorrection::QZGP ||
               tc.type == TimeSystemCorrection::QZUT)
            {
               GPSWeekSecond gws(tc.refWeek,tc.refSOW);
               CivilTime ct(gws);
               tc.refYr = ct.year;
               tc.refMon = ct.month;
               tc.refDay = ct.day;
            }

            if(tc.type == TimeSystemCorrection::GAUT) {
               GALWeekSecond gws(tc.refWeek,tc.refSOW);
               CivilTime ct(gws);
               tc.refYr = ct.year;
               tc.refMon = ct.month;
               tc.refDay = ct.day;
            }

            //if(tc.type == TimeSystemCorrection::GLUT) {
            //   tc.refYr =  1980;
            //   tc.refMon = 1;
            //   tc.refDay = 6;
            //   tc.refWeek = 0;
            //   tc.refSOW = 0;
            //}

            mapTimeCorr[tc.asString4()] = tc;
            valid |= validTimeSysCorr;
         }

         else if(thisLabel == stringLeapSeconds) {                // "LEAP SECONDS"
            leapSeconds = asInt(line.substr(0,6));
            leapDelta = asInt(line.substr(6,6));      // R3 only
            leapWeek = asInt(line.substr(12,6));      // R3 only
            leapDay = asInt(line.substr(18,6));       // R3 only
            valid |= validLeapSeconds;
         }

         else if(thisLabel == stringEoH) {                        // "END OF HEADER"
            valid |= validEoH;
         }

         else {
            throw(FFStreamError("Unknown header label >" + thisLabel + "< at line " + 
            asString<size_t>(strm.lineNumber)));
         }
      }
   
      unsigned long allValid;
      if(version >= 3.0)
         allValid = allValid3;
      else if(version >= 2 && version < 3)
         allValid = allValid2;
      else {
         FFStreamError e("Unknown or unsupported RINEX version "+asString(version,2));
         GPSTK_THROW(e);
      }
   
      if((allValid & valid) != allValid) {
         FFStreamError e("Incomplete or invalid header");
         GPSTK_THROW(e);
      }
   
      strm.header = *this;
      strm.headerRead = true;
   
   } // end of reallyGetRecord
   
   //--------------------------------------------------------------------------
   void Rinex3NavHeader::reallyPutRecord(FFStream& ffs) const 
   throw(std::exception, FFStreamError, StringException)
   {
      Rinex3NavStream& strm = dynamic_cast<Rinex3NavStream&>(ffs);
   
      strm.header = (*this);
   
      int j;
      unsigned long allValid;
      if(version >= 3.0)
         allValid = allValid3;
      else if(version >= 2 && version < 3)
         allValid = allValid2;
      else {
         FFStreamError err("Unknown RINEX version: " + asString(version,4));
         GPSTK_THROW(err);
      }
   
      if((valid & allValid) != allValid) {
         FFStreamError err("Incomplete or invalid header.");
         GPSTK_THROW(err);
      }
   
      string line;
      if(valid & validVersion) {                         // "RINEX VERSION / TYPE"
         line = rightJustify(asString(version,2), 10);
         line += string(10, ' ');
         line += leftJustify(fileType, 20);
         if(version >= 3) line += leftJustify(fileSys,20);
         else             line += string(20,' ');
         line += leftJustify(stringVersion,20);
         strm << stripTrailing(line) << endl;
         strm.lineNumber++;
      }
   
      if(valid & validRunBy) {                           // "PGM / RUN BY / DATE"
         line = leftJustify(fileProgram,20);
         line += leftJustify(fileAgency ,20);
         SystemTime sysTime;
         string curDate = printTime(sysTime,"%04Y%02m%02d %02H%02M%02S UTC");
         line += leftJustify(curDate, 20);
         line += leftJustify(stringRunBy,20);
         strm << stripTrailing(line) << endl;
         strm.lineNumber++;
      }
   
      if(valid & validComment) {                         // "COMMENT"
         vector<string>::const_iterator itr = commentList.begin();
         while (itr != commentList.end())
         {
            line = leftJustify((*itr), 60);
            line += leftJustify(stringComment,20);
            strm << stripTrailing(line) << endl;
            strm.lineNumber++;
            itr++;
         }
      }
   
      if(valid & validIonoCorrGPS) {                     // "IONOSPHERIC CORR"
         map<string,IonoCorr>::const_iterator it;
         for(it=mapIonoCorr.begin(); it != mapIonoCorr.end(); ++it) {
            switch(it->second.type) {
               case IonoCorr::GAL:
                  line = "GAL  ";
                  for(j=0; j<3; j++)
                     line += doubleToScientific(it->second.param[j],12,4,2);
                  line += doubleToScientific(0.0,12,4,2);
                  line += string(7,' ');
                  line += leftJustify(stringIonoCorr,20);
                  break;
               case IonoCorr::GPSA:
                  if(version >= 3) {
                     line = "GPSA ";
                     for(j=0; j<4; j++)
                        line += doubleToScientific(it->second.param[j],12,4,2);
                     line += string(7,' ');
                     line += leftJustify(stringIonoCorr,20);
                  }
                  else {                                    // "ION ALPHA" // R2.11
                     line = "  ";
                     for(j=0; j<4; j++)
                        line += doubleToScientific(it->second.param[j],12,4,2);
                     line += string(10,' ');
                     line += leftJustify(stringIonAlpha,20);
                  }
                  break;
               case IonoCorr::GPSB:
                  if(version >= 3) {
                     line = "GPSB ";
                     for(j=0; j<4; j++)
                        line += doubleToScientific(it->second.param[j],12,4,2);
                     line += string(7,' ');
                     line += leftJustify(stringIonoCorr,20);
                  }
                  else {                                    // "ION BETA" // R2.11
                     line = "  ";
                     for(j=0; j<4; j++)
                        line += doubleToScientific(it->second.param[j],12,4,2);
                     line += string(10,' ');
                     line += leftJustify(stringIonBeta,20);
                  }
                  break;
            }
            strm << stripTrailing(line) << endl;
            strm.lineNumber++;
         }
      }
   
      if(valid & validTimeSysCorr) {               // "TIME SYSTEM CORR"
         map<string,TimeSystemCorrection>::const_iterator it;
         for(it=mapTimeCorr.begin(); it != mapTimeCorr.end(); ++it) {
            const TimeSystemCorrection& tc(it->second);
            if(version >= 3) {
               line = tc.asString4() + " ";
               line += doubleToScientific(tc.A0,17,10,2);
               //if(tc.type == TimeSystemCorrection::GLUT
               //            || tc.type == TimeSystemCorrection::GLGP)
               //   line += doubleToScientific(0.0,16,9,2);
               //else
                  line += doubleToScientific(tc.A1,16,9,2);

               line += rightJustify(asString<long>(tc.refSOW),7);
               line += rightJustify(asString<long>(tc.refWeek),5);

               if(tc.type == TimeSystemCorrection::SBUT) {
                  line += rightJustify(tc.geoProvider,6);
                  line += " ";
               }
               else
                  line += string(7,' ');

               line += rightJustify(asString<int>(tc.geoUTCid),2);
               line += " ";

               line += leftJustify(stringTimeSysCorr,20);
            }
            else {
               if(tc.asString4() == "GPUT") {     // "DELTA-UTC: A0,A1,T,W" R2.11 GPS
                  line = "   ";
                  line += doubleToScientific(tc.A0,19,12,2);
                  line += doubleToScientific(tc.A1,19,12,2);
                  line += rightJustify(asString<long>(tc.refSOW),9);
                  line += rightJustify(asString<long>(tc.refWeek),9);
                  line += " ";
                  line += leftJustify(stringDeltaUTC,20);
               }
               else if(tc.asString4() == "GLGP") { // "CORR TO SYSTEM TIME" R2.10 GLO
                  line = rightJustify(asString<long>(tc.refYr),6);
                  line += rightJustify(asString<long>(tc.refMon),6);
                  line += rightJustify(asString<long>(tc.refDay),6);
                  line += doubleToScientific(tc.A0,19,12,2);
                  line += string(23,' ');
                  line += leftJustify(stringCorrSysTime,20);
               }
               else if(tc.asString4() == "SBUT") { // "D-UTC A0,A1,T,W,S,U" R2.11 GEO
                  line = doubleToScientific(tc.A0,19,12,2);
                  line += doubleToScientific(tc.A1,19,12,2);
                  line += rightJustify(asString<long>(tc.refSOW),7);
                  line += rightJustify(asString<long>(tc.refWeek),5);
                  line += rightJustify(tc.geoProvider,6);
                  line += " ";
                  line += rightJustify(asString<int>(tc.geoUTCid),2);
                  line += " ";
                  line += leftJustify(stringDUTC,20);
               }
            }

            strm << stripTrailing(line) << endl;
            strm.lineNumber++;
         }
      }
   
      if(valid & validLeapSeconds) {                         // "LEAP SECONDS"
         line = rightJustify(asString(leapSeconds),6);
         if(version >= 3) {                                    // ver 3
            line += rightJustify(asString(leapDelta),6);
            line += rightJustify(asString(leapWeek),6);
            line += rightJustify(asString(leapDay),6);
            line += string(36, ' ');
         }
         else                                                  // ver 2
            line += string(54, ' ');
         line += leftJustify(stringLeapSeconds,20);
         strm << stripTrailing(line) << endl;
         strm.lineNumber++;
      }
   
      if(valid & validEoH) {                                 // "END OF HEADER"
         line = string(60,' ');
         line += leftJustify(stringEoH,20);
         strm << stripTrailing(line) << endl;
         strm.lineNumber++;
      }
   
   } // end of reallyPutRecord

   //--------------------------------------------------------------------------
   void Rinex3NavHeader::dump(ostream& s) const
   {
   
      s << "---------------------------------- REQUIRED "
         << "----------------------------------\n";
   
      s << "Rinex Version " << fixed << setw(5) << setprecision(2) << version
         << ",  File type " << fileType << ", System " << fileSys << ".\n";
      s << "Prgm: " << fileProgram << ",  Run: " << date << ",  By: " << fileAgency
         << endl;
   
      s << "(This header is ";
      if(version >= 3 && (valid & allValid3) == allValid3)
         s << "VALID RINEX version 3";
      else if(version < 3 && (valid & allValid2) == allValid2)
         s << "VALID RINEX version 2";
      else s << "NOT VALID RINEX";
      s << ")." << endl;
   
      if(!(valid & validVersion)) s << " Version is NOT valid\n";
      if(!(valid & validRunBy  )) s << " Run by is NOT valid\n";
      if(!(valid & validEoH    )) s << " End of Header is NOT valid\n";
   
      s << "---------------------------------- OPTIONAL "
         << "----------------------------------\n";
   
      for(map<string,TimeSystemCorrection>::const_iterator tcit
         = mapTimeCorr.begin(); tcit != mapTimeCorr.end(); ++tcit)
            { tcit->second.dump(s); s << endl; }

      map<string,IonoCorr>::const_iterator icit;
      for(icit=mapIonoCorr.begin(); icit != mapIonoCorr.end(); ++icit) {
         s << "Iono correction for " << icit->second.asString() << " : "
            << scientific << setprecision(4);
         switch(icit->second.type) {
            case IonoCorr::GAL: s << "ai0 = " << icit->second.param[0]
                                    << ", ai1 = " << icit->second.param[1]
                                    << ", ai2 = " << icit->second.param[2];
               break;
            case IonoCorr::GPSA: s << "alpha " << icit->second.param[0]
                                    << " " << icit->second.param[1]
                                    << " " << icit->second.param[2]
                                    << " " << icit->second.param[3];
               break;
            case IonoCorr::GPSB: s << "beta  " << icit->second.param[0]
                                    << " " << icit->second.param[1]
                                    << " " << icit->second.param[2]
                                    << " " << icit->second.param[3];
               break;
         }
         s << endl;
      }

      if(valid & validLeapSeconds) {
         s << "Leap seconds: " << leapSeconds;
         if(leapDelta != 0) s << ", change " << leapDelta
            << " at week " << leapWeek << ", day " << leapDay;
         s << endl;
      }
      else s << " Leap seconds is NOT valid\n";
   
      if(commentList.size() > 0) {
         s << "Comments (" << commentList.size() << ") :\n";
         for(size_t i = 0; i < commentList.size(); i++)
         s << commentList[i] << endl;
      }
   
      s << "-------------------------------- END OF HEADER "
         << "-------------------------------\n";

   } // end of dump
   
} // namespace
