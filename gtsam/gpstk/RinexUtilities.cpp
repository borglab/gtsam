/// @file RinexUtilities.cpp
/// Miscellaneous RINEX-related utilities.

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

// system includes
#include <map>

// GPSTk includes
#include "RinexObsStream.hpp"
#include "RinexObsHeader.hpp"
#include "RinexNavStream.hpp"
#include "RinexNavHeader.hpp"
#include "RinexNavData.hpp"

#include "Rinex3ObsStream.hpp"
#include "Rinex3ObsHeader.hpp"
#include "Rinex3NavStream.hpp"
#include "Rinex3NavHeader.hpp"
#include "Rinex3NavData.hpp"

#include "SP3Stream.hpp"
#include "SP3Header.hpp"

#include "RinexUtilities.hpp"

namespace gpstk {

using namespace std;
using namespace gpstk;

//------------------------------------------------------------------------------
int RegisterARLUTExtendedTypes(void)
{
try {
   unsigned int EPPS = //0x60
      RinexObsType::EPdepend | RinexObsType::PSdepend;
   unsigned int L1L2 = //0x06
      RinexObsType::L1depend | RinexObsType::L2depend;
   unsigned int P1P2 = //0x18
      RinexObsType::P1depend | RinexObsType::P2depend;
   unsigned int EPEP=RinexObsType::EPdepend;//0x20
   unsigned int PELL=EPPS | L1L2;//0x66
   unsigned int PEPP=EPPS | P1P2;//0x78
   unsigned int PsLs=L1L2 | P1P2;//0x1E
   unsigned int L1P1 = //0x0A
      RinexObsType::L1depend | RinexObsType::P1depend;
   unsigned int L2P2 = //0x14
      RinexObsType::L2depend | RinexObsType::P2depend;
   int j;
   j = RegisterExtendedRinexObsType("ER","Ephemeris range",     "meters", EPPS);
   if(j) return j;
   j = RegisterExtendedRinexObsType("RI","Iono Delay, Range",   "meters", P1P2);
   if(j) return j;
   j = RegisterExtendedRinexObsType("PI","Iono Delay, Phase",   "meters", L1L2);
   if(j) return j;
   j = RegisterExtendedRinexObsType("TR","Tropospheric Delay",  "meters", EPPS);
   if(j) return j;
   j = RegisterExtendedRinexObsType("RL","Relativity Correct.", "meters", EPEP);
   if(j) return j;
   j = RegisterExtendedRinexObsType("SC","SV Clock Bias",       "meters", EPEP);
   if(j) return j;
   j = RegisterExtendedRinexObsType("EL","Elevation Angle",     "degrees",EPPS);
   if(j) return j;
   j = RegisterExtendedRinexObsType("AZ","Azimuth Angle",       "degrees",EPPS);
   if(j) return j;
   j = RegisterExtendedRinexObsType("SR","Slant TEC (PR)",      "TECU",   P1P2);
   if(j) return j;
   j = RegisterExtendedRinexObsType("SP","Slant TEC (Ph)",      "TECU",   L1L2);
   if(j) return j;
   j = RegisterExtendedRinexObsType("VR","Vertical TEC (PR)",   "TECU",   PEPP);
   if(j) return j;
   j = RegisterExtendedRinexObsType("VP","Vertical TEC (Ph)",   "TECU",   PELL);
   if(j) return j;
   j = RegisterExtendedRinexObsType("LA","Lat Iono Intercept",  "degrees",EPPS);
   if(j) return j;
   j = RegisterExtendedRinexObsType("LO","Lon Iono Intercept",  "degrees",EPPS);
   if(j) return j;
   j = RegisterExtendedRinexObsType("P3","TFC(IF) Pseudorange", "meters", P1P2);
   if(j) return j;
   j = RegisterExtendedRinexObsType("L3","TFC(IF) Phase",       "meters", L1L2);
   if(j) return j;
   j = RegisterExtendedRinexObsType("PF","GeoFree Pseudorange", "meters", P1P2);
   if(j) return j;
   j = RegisterExtendedRinexObsType("LF","GeoFree Phase",       "meters", L1L2);
   if(j) return j;
   j = RegisterExtendedRinexObsType("PW","WideLane Pseudorange","meters", P1P2);
   if(j) return j;
   j = RegisterExtendedRinexObsType("LW","WideLane Phase",      "meters", L1L2);
   if(j) return j;
   j = RegisterExtendedRinexObsType("MP","Multipath (=M3)",     "meters", PsLs);
   if(j) return j;
   j = RegisterExtendedRinexObsType("R1","(P1 + L1)/2"         ,"meters", L1P1);
   if(j) return j;
   j = RegisterExtendedRinexObsType("R2","(P2 + L2)/2"         ,"meters", L2P2);
   if(j) return j;
   j = RegisterExtendedRinexObsType("M1","L1 Range minus Phase","meters", L1P1);
   if(j) return j;
   j = RegisterExtendedRinexObsType("M2","L2 Range minus Phase","meters", L2P2);
   if(j) return j;
   j = RegisterExtendedRinexObsType("M3","IF Range minus Phase","meters", PsLs);
   if(j) return j;
   j = RegisterExtendedRinexObsType("M4","GF Range minus Phase","meters", PsLs);
   if(j) return j;
   j = RegisterExtendedRinexObsType("M5","WL Range minus Phase","meters", PsLs);
   if(j) return j;
   j = RegisterExtendedRinexObsType("XR","Non-dispersive Range","meters", PsLs);
   if(j) return j;
   j = RegisterExtendedRinexObsType("XI","Ionospheric delay",   "meters", PsLs);
   if(j) return j;
   j = RegisterExtendedRinexObsType("X1","Range Error L1",      "meters", PsLs);
   if(j) return j;
   j = RegisterExtendedRinexObsType("X2","Range Error L2",      "meters", PsLs);
   if(j) return j;
   j = RegisterExtendedRinexObsType("SX","Satellite ECEF-X",    "meters", EPEP);
   if(j) return j;
   j = RegisterExtendedRinexObsType("SY","Satellite ECEF-Y",    "meters", EPEP);
   if(j) return j;
   j = RegisterExtendedRinexObsType("SZ","Satellite ECEF-Z",    "meters", EPEP);
   if(j) return j;
   return 0;
}
catch(Exception& e) { GPSTK_RETHROW(e); }
catch(exception& e) { Exception E("std except: "+string(e.what())); GPSTK_THROW(E); }
catch(...) { Exception e("Unknown exception"); GPSTK_THROW(e); }
}

//------------------------------------------------------------------------------------
bool isSP3File(const string& file)
{
   try
   {
      SP3Header header;
      SP3Stream strm(file.c_str());
      strm.exceptions(fstream::failbit);
      try
      {
         strm >> header;
      }
      catch(Exception& e)
      {
         return false;
      }
      strm.close();
      return true;
   }
   catch(Exception& e)
   {
      GPSTK_RETHROW(e);
   }
   catch(exception& e)
   {
      Exception E("std except: "+string(e.what()));
      GPSTK_THROW(E);
   }
   catch(...)
   {
      Exception e("Unknown exception");
      GPSTK_THROW(e);
   }
}

//------------------------------------------------------------------------------------
bool isRinexNavFile(const string& file)
{
   try
   {
      RinexNavHeader header;
      RinexNavStream rnstream;
      try {
         rnstream.open(file.c_str(),ios::in);
         if(!rnstream)
            return false;
         rnstream.exceptions(fstream::failbit);
      }
      catch(Exception& e) { return false; }
      catch(exception& e) { return false;}
      try {
         rnstream >> header;
      }
      catch(Exception& e) { return false; }
      catch(exception& e) { return false; }

      rnstream.close();
      return true;
   }
   catch(Exception& e) { GPSTK_RETHROW(e); }
   catch(exception& e) {
      Exception E("std except: "+string(e.what()));
      GPSTK_THROW(E);
   }
   catch(...) {
      Exception e("Unknown exception");
      GPSTK_THROW(e);
   }
}

//------------------------------------------------------------------------------------
bool isRinex3NavFile(const string& file)
{
   try
   {
      Rinex3NavHeader header;
      Rinex3NavStream rnstream;
      try {
         rnstream.open(file.c_str(),ios::in);
         if(!rnstream)
            return false;
         rnstream.exceptions(fstream::failbit);
      }
      catch(Exception& e) { return false; }
      catch(exception& e) { return false; }
      try {
         rnstream >> header;
      }
      catch(Exception& e) { return false; }
      catch(exception& e) { return false; }

      rnstream.close();
      return true;
   }
   catch(Exception& e)
   {
      GPSTK_RETHROW(e);
   }
   catch(exception& e)
   {
      Exception E("std except: "+string(e.what()));
      GPSTK_THROW(E);
   }
   catch(...)
   {
      Exception e("Unknown exception");
      GPSTK_THROW(e);
   }
}

//------------------------------------------------------------------------------------
bool isRinexObsFile(const string& file)
{
   try
   {
      RinexObsHeader header;
      RinexObsStream rostream;
      try {
         rostream.open(file.c_str(),ios::in);
         if(!rostream)
            return false;
         rostream.exceptions(fstream::failbit);
      }
      catch(Exception& e) { return false; }
      catch(exception& e) { return false; }

      try {
         rostream >> header;
      }
      catch(Exception& e) { return false; }
      catch(exception& e) { return false; }

      rostream.close();
      return true;
   }
   catch(Exception& e) { GPSTK_RETHROW(e); }
   catch(exception& e) {
      Exception E("std except: "+string(e.what()));
      GPSTK_THROW(E);
   }
   catch(...) {
      Exception e("Unknown exception");
      GPSTK_THROW(e);
   }
}

//------------------------------------------------------------------------------------
bool isRinex3ObsFile(const string& file)
{
   try
   {
      Rinex3ObsHeader header;
      Rinex3ObsStream rostream;
      try {
         rostream.open(file.c_str(),ios::in);
         if(!rostream)
            return false;
         rostream.exceptions(fstream::failbit);
      }
      catch(Exception& e) { return false; }
      catch(exception& e) { return false; }

      try {
         rostream >> header;
      }
      catch(Exception& e) { return false; }
      catch(exception& e) { return false; }

      rostream.close();
      return true;
   }
   catch(Exception& e) { GPSTK_RETHROW(e); }
   catch(exception& e) {
      Exception E("std except: "+string(e.what()));
      GPSTK_THROW(E);
   }
   catch(...) {
      Exception e("Unknown exception");
      GPSTK_THROW(e);
   }
}

//------------------------------------------------------------------------------------
string sortRinexObsFiles(vector<string>& files) throw(Exception)
{
try {
   string msg;
   if(files.size() <= 1) return msg;

   // build a hash with key = start time, value = filename
   multimap<CommonTime,string> hash;
   for(size_t n=0; n<files.size(); n++) {
      try {
         RinexObsHeader header;
         RinexObsStream rostream(files[n].c_str());
         if(!rostream.is_open()) {
            msg += "Error - Could not open file " + files[n] + "\n";
            continue;
         }
         rostream.exceptions(fstream::failbit);
         rostream >> header;
         rostream.close();
         if(!header.isValid()) {
            msg += "Error - Invalid header in file " + files[n] + "\n";
            continue;
         }
         //hash[header.firstObs] = files[n];
         hash.insert(multimap<CommonTime, string>::value_type(header.firstObs,
                                                                      files[n]));
      }
      catch(Exception& e) {
         //msg += "Exception " + e.what() + " in file " + files[n] + "\n";
         msg += "Error - File "+files[n]+" is not a valid RINEX observation file.\n";
         continue;
      }
   }

   // return the sorted file names
   files.clear();
   multimap<CommonTime,string>::const_iterator it = hash.begin();
   while(it != hash.end()) {
      files.push_back(it->second);
      it++;
   }

   string::size_type pos(msg.length());
   if(pos > 0) msg.erase(pos-1);
   return msg;
}
catch(Exception& e) { GPSTK_RETHROW(e); }
catch(exception& e) { Exception E("std except: "+string(e.what())); GPSTK_THROW(E); }
catch(...) { Exception e("Unknown exception"); GPSTK_THROW(e); }
}

//------------------------------------------------------------------------------------
string sortRinex3ObsFiles(vector<string>& files)
{
   string msg;
   if(files.size() <= 1) { msg = string("No input files!"); return msg; }

   try
   {
      // build a hash with key = start time, value = filename
      multimap<CommonTime,string> hash;
      for(size_t n = 0; n < files.size(); n++)
      {
         try {
            Rinex3ObsHeader header;
            Rinex3ObsStream rostream(files[n].c_str());
            if(!rostream.is_open()) {
               msg += "Error - Could not open file " + files[n] + "\n";
               continue;
            }
            rostream.exceptions(fstream::failbit);
            rostream >> header;
            rostream.close();
            if(!header.isValid()) {
               msg += "Error - Invalid header in file " + files[n] + "\n";
               continue;
            }
            hash.insert(
               multimap<CommonTime, string>::value_type(header.firstObs, files[n])
            );

         }
         catch(Exception& e) {
            msg += "Exception: " + e.what() + "\n";
            continue;
         }
      }

      // return the sorted file names
      files.clear();
      multimap<CommonTime,string>::const_iterator it = hash.begin();
      while(it != hash.end()) {
         files.push_back(it->second);
         it++;
      }
   }
   catch(Exception& e) { GPSTK_RETHROW(e); }
   catch(exception& e) {
      Exception E("std except: "+string(e.what()));
      GPSTK_THROW(E);
   }
   catch(...) {
      Exception e("Unknown exception");
      GPSTK_THROW(e);
   }

   return msg;
}

//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------
int FillEphemerisStore(const vector<string>& files, SP3EphemerisStore& PE,
                                                     GPSEphemerisStore& BCE)
{
   try
   {
      int nread = 0;
      Rinex3NavHeader rnh;
      Rinex3NavData rne;
      for(size_t nfile = 0; nfile < files.size(); nfile++) {
         if(files[nfile].empty()) {
            Exception e("File name is empty");
            GPSTK_THROW(e);
         }

         if(isRinex3NavFile(files[nfile]) || isRinexNavFile(files[nfile])) {
            Rinex3NavStream instrm(files[nfile].c_str());
            instrm.exceptions(fstream::failbit);
            try {
               instrm >> rnh;
               while (instrm >> rne) {
                  // check health...
                  if(rne.health == 0)
                     BCE.addEphemeris(rne);
               }
               nread++;
            }
            catch(Exception& e) {
               GPSTK_RETHROW(e);
            }
         }

         else if(isSP3File(files[nfile])) {
            try {
               PE.loadFile(files[nfile]);
               nread++;
            }
            catch(Exception& e) { GPSTK_RETHROW(e); }
         }
         else {
            Exception e("File " + files[nfile]
                  + " is neither Rinex Nav nor SP3 file.");
            GPSTK_THROW(e);
         }
      }
      return nread;
   }
   catch(Exception& e) { GPSTK_RETHROW(e); }
   catch(exception& e) {
      Exception E("std except: "+string(e.what()));
      GPSTK_THROW(E);
   }
   catch(...) {
      Exception e("Unknown exception");
      GPSTK_THROW(e);
   }
}

} // end namespace gpstk

//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------
