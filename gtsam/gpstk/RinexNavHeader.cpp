#pragma ident "$Id$"

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

/**
 * @file RinexNavHeader.cpp
 * Encapsulate header of Rinex navigation file
 */

#include "StringUtils.hpp"
#include "CommonTime.hpp"
#include "CivilTime.hpp"
#include "SystemTime.hpp"
#include "RinexNavHeader.hpp"
#include "RinexNavStream.hpp"

#include <iostream>

using namespace gpstk::StringUtils;
using namespace std;

namespace gpstk
{
   const string RinexNavHeader::endOfHeader = "END OF HEADER";
   const string RinexNavHeader::leapSecondsString = "LEAP SECONDS";
   const string RinexNavHeader::deltaUTCString = "DELTA-UTC: A0,A1,T,W";
   const string RinexNavHeader::ionBetaString = "ION BETA";
   const string RinexNavHeader::ionAlphaString = "ION ALPHA";
   const string RinexNavHeader::commentString = "COMMENT";
   const string RinexNavHeader::runByString = "PGM / RUN BY / DATE";
   const string RinexNavHeader::versionString = "RINEX VERSION / TYPE";

   void RinexNavHeader::reallyPutRecord(FFStream& ffs) const 
      throw(std::exception, FFStreamError, StringException)
   {
      RinexNavStream& strm = dynamic_cast<RinexNavStream&>(ffs);
      
      strm.header = (*this);
      
      unsigned long allValid;
      if (version == 2.0)        allValid = allValid20;
      else if (version == 2.1)   allValid = allValid21;
      else if (version == 2.11)  allValid = allValid211;
      else
      {
         FFStreamError err("Unknown RINEX version: " + asString(version,3));
         err.addText("Make sure to set the version correctly.");
         GPSTK_THROW(err);
      }
      
      if ((valid & allValid) != allValid)
      {
         FFStreamError err("Incomplete or invalid header.");
         err.addText("Make sure you set all header valid bits for all of the available data.");
         GPSTK_THROW(err);
      }
      
      string line;
      
      if (valid & versionValid)
      {
         line  = rightJustify(asString(version,3), 10);
         line += string(10, ' ');
         line += string("NAVIGATION"); //leftJustify(fileType, 20);
         line += string(30, ' ');
         line += versionString;
         strm << line << endl;
         strm.lineNumber++;
      }
      if (valid & runByValid) 
      {
         line  = leftJustify(fileProgram,20);
         line += leftJustify(fileAgency,20);
         SystemTime dt;
         string dat = (static_cast<CivilTime>(dt)).printf("%02m/%02d/%04Y %02H:%02M:%02S");
         line += leftJustify(dat, 20);
         line += runByString;
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
      if (valid & ionAlphaValid)
      {
         line  = string(2, ' ');
         for (int i = 0; i < 4; i++)
         {
            line += rightJustify(doub2for(ionAlpha[i], 12, 2),12);  // should be 12.4
         }
         line += string(10, ' ');
         line += ionAlphaString;
         strm << line << endl;
         strm.lineNumber++;
      }
      if (valid & ionBetaValid)
      {
         line  = string(2, ' ');
         for (int i = 0; i < 4; i++)
         {
            line += rightJustify(doub2for(ionBeta[i], 12, 2),12);
         }
         line += string(10, ' ');
         line += ionBetaString;
         strm << line << endl;
         strm.lineNumber++;
      }
      if (valid & deltaUTCValid)
      {
         line  = string(3, ' ');
         //line += string(2, ' ');
         line += doub2for(A0, 19, 2);
         line += doub2for(A1, 19, 2);
         line += rightJustify(asString(UTCRefTime),9);
         line += rightJustify(asString(UTCRefWeek),9);               
         line += string(1, ' ');
         line += deltaUTCString;
         strm << line << endl;
         strm.lineNumber++;
      }
      if (valid & leapSecondsValid)
      {
         line  = rightJustify(asString(leapSeconds), 6);
         line += string(54, ' ');
         line += leapSecondsString;
         strm << line << endl;
         strm.lineNumber++;
      }
      if (valid & endValid)
      {
         line  = string(60,' ');
         line += endOfHeader;
         strm << line << endl;
         strm.lineNumber++;
      }
      
   }

   void RinexNavHeader::reallyGetRecord(FFStream& ffs) 
      throw(std::exception, FFStreamError, StringException)
   {
      RinexNavStream& strm = dynamic_cast<RinexNavStream&>(ffs);
      
         // if already read, just return
      if (strm.headerRead == true)
         return;
      
      valid = 0;
      
         // clear out anything that was unsuccessfully read the first
      commentList.clear();
      
      while (! (valid & endValid))
      {
         string line;
         strm.formattedGetLine(line);
         StringUtils::stripTrailing(line);

         if (line.length()==0) continue;
         else if (line.length()<60 || line.length()>80)
         {
            FFStreamError e("Invalid line length");
            GPSTK_THROW(e);
         }
         
         string thisLabel(line, 60, 20);
         
         if (thisLabel == versionString)
         {
            version = asDouble(line.substr(0,20));
            fileType = strip(line.substr(20,20));
            if ( (fileType[0] != 'N') &&
                 (fileType[0] != 'n'))
            {
               FFStreamError e("This isn't a Rinex Nav file");
               GPSTK_THROW(e);
            }
            valid |= versionValid;
         }
         else if (thisLabel == runByString)
         {
            fileProgram = strip(line.substr(0,20));
            fileAgency = strip(line.substr(20,20));
            date = strip(line.substr(40,20));
            valid |= runByValid;
         }
         else if (thisLabel == commentString)
         {
            commentList.push_back(strip(line.substr(0,60)));
            valid |= commentValid;
         }
         else if (thisLabel == ionAlphaString)
         {
            for(int i = 0; i < 4; i++)
               ionAlpha[i] = gpstk::StringUtils::for2doub(line.substr(2 + 12 * i,12));
            valid |= ionAlphaValid;
         }
         else if (thisLabel == ionBetaString)
         {
            for(int i = 0; i < 4; i++)
               ionBeta[i] = gpstk::StringUtils::for2doub(line.substr(2 + 12 * i,12));
            valid |= ionBetaValid;
         }
         else if (thisLabel == deltaUTCString)
         {
            A0 = gpstk::StringUtils::for2doub(line.substr(3,19));
            A1 = gpstk::StringUtils::for2doub(line.substr(22,19));
            UTCRefTime = asInt(line.substr(41,9));
            UTCRefWeek = asInt(line.substr(50,9));
            valid |= deltaUTCValid;
         }
         else if (thisLabel == leapSecondsString)
         {
            leapSeconds = asInt(line.substr(0,6));
            valid |= leapSecondsValid;
         }
         else if (thisLabel == endOfHeader)
         {
            valid |= endValid;
         }
         else
         {
            throw(FFStreamError("Unknown header label at line " + 
                                asString<size_t>(strm.lineNumber)));
         }
      }
      
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
      
         // we got here, so something must be right...
      strm.header = *this;
      strm.headerRead = true;      
   }

   void RinexNavHeader::dump(ostream& s) const
   {
      int i;
       s << "---------------------------------- REQUIRED ----------------------------------\n";
      s << "Rinex Version " << fixed << setw(5) << setprecision(2) << version
         << ",  File type " << fileType << ".\n";
      s << "Prgm: " << fileProgram << ",  Run: " << date << ",  By: " << fileAgency << endl;

      s << "(This header is ";
      if((valid & allValid211) == allValid211) s << "VALID 2.11";
      else if((valid & allValid21) == allValid21) s << "VALID 2.1";
      else if((valid & allValid20) == allValid20) s << "VALID 2.0";
      else s << "NOT VALID";
      s << " Rinex.)\n";

      if(!(valid & versionValid)) s << " Version is NOT valid\n";
      if(!(valid & runByValid)) s << " Run by is NOT valid\n";
      if(!(valid & endValid)) s << " End is NOT valid\n";

      s << "---------------------------------- OPTIONAL ----------------------------------\n";
      if(valid & ionAlphaValid) { s << "Ion alpha:";
         for(i=0; i<4; i++) s << " " << scientific << setprecision(4) << ionAlpha[i];
      s << endl; }
      else s << " Ion alpha is NOT valid\n";
      if(valid & ionBetaValid) { s << "Ion beta:";
         for(i=0; i<4; i++) s << " " << scientific << setprecision(4) << ionBeta[i];
      s << endl; }
      else s << " Ion beta is NOT valid\n";
      if(valid & deltaUTCValid) s << "Delta UTC: A0="
         << scientific << setprecision(12) << A0 << ", A1="
         << scientific << setprecision(12) << A1 << ", UTC ref = ("
         << UTCRefWeek << "," << UTCRefTime << ")\n";
      else s << " Delta UTC is NOT valid\n";
      if(valid & leapSecondsValid) s << "Leap seconds: " << leapSeconds << endl;
      else s << " Leap seconds is NOT valid\n";
      if(commentList.size() > 0) {
         s << "Comments (" << commentList.size() << ") :\n";
         for(size_t i=0; i<commentList.size(); i++)
            s << commentList[i] << endl;
      }
      s << "-------------------------------- END OF HEADER -------------------------------\n";
   }

} // namespace
