/// @file Rinex3ObsData.cpp
/// Encapsulate RINEX 3 observation file data, including I/O.

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

#include <algorithm>
#include "StringUtils.hpp"
#include "CivilTime.hpp"
#include "TimeString.hpp"
#include "RinexObsID.hpp"
#include "Rinex3ObsStream.hpp"
#include "Rinex3ObsData.hpp"

using namespace gpstk::StringUtils;
using namespace std;

namespace gpstk
{


   void reallyPutRecordVer2( Rinex3ObsStream& strm,
                             const Rinex3ObsData& rod )
      throw(FFStreamError, StringException)
   {

         // is there anything to write?
      if( (rod.epochFlag==0 || rod.epochFlag==1 || rod.epochFlag==6)
          && (rod.numSVs==0 || rod.obs.empty()) ) return;

      if( rod.epochFlag>=2
          && rod.epochFlag<=5
          && rod.auxHeader.NumberHeaderRecordsToBeWritten()==0 ) return;

      // first the epoch line to 'line'
      //line  = writeTime(rod.time); // (ver 2 RinexObsData::writeTime)
      string line;
      if(rod.time == CommonTime::BEGINNING_OF_TIME)
         line = string(26, ' ');
      else
      {
         CivilTime civTime(rod.time);
         line  = string(1, ' ');
         line += rightJustify(asString<short>(civTime.year),2);
         line += string(1, ' ');
         line += rightJustify(asString<short>(civTime.month),2);
         line += string(1, ' ');
         line += rightJustify(asString<short>(civTime.day),2);
         line += string(1, ' ');
         line += rightJustify(asString<short>(civTime.hour),2);
         line += string(1, ' ');
         line += rightJustify(asString<short>(civTime.minute),2);
         line += rightJustify(asString(civTime.second, 7),11);
         line += string(2, ' ');
         line += rightJustify(asString<short>(rod.epochFlag), 1);
         line += rightJustify(asString<short>(rod.numSVs), 3);
      }

         // write satellite ids to 'line'
      const int maxPrnsPerLine = 12;
      int satsWritten = 0;

      Rinex3ObsData::DataMap::const_iterator itr(rod.obs.begin());

      if( rod.epochFlag==0 || rod.epochFlag==1 || rod.epochFlag==6 )
      {
         while( itr != rod.obs.end() && satsWritten < maxPrnsPerLine )
         {
            line += itr->first.toString();
            satsWritten++;
            itr++;
         }

            // add clock offset
         if( rod.clockOffset != 0.0 )
         {
            line += string(68 - line.size(), ' ');
            line += rightJustify( asString(rod.clockOffset, 9), 12 );
         }

            // continuation lines
         while( satsWritten != rod.obs.size() )
         {
            if((satsWritten % maxPrnsPerLine) == 0)
            {
               strm << line << endl;
               strm.lineNumber++;
               line  = string(32, ' ');
            }
            line += itr->first.toString();
            satsWritten++;
            itr++;
         }

      }  // End of 'if( rod.epochFlag==0 || rod.epochFlag==1 || ...'

         // write the epoch line
      strm << line << endl;
      strm.lineNumber++;

         // write the auxiliary header records, if any
      if( rod.epochFlag >= 2 && rod.epochFlag <= 5 )
      {
         try
         {
            rod.auxHeader.WriteHeaderRecords(strm);
         }
         catch(FFStreamError& e)
         {
            GPSTK_RETHROW(e);
         }
         catch(StringException& e)
         {
            GPSTK_RETHROW(e);
         }
      }  // write out data
      else if( rod.epochFlag == 0 || rod.epochFlag == 1 || rod.epochFlag == 6 )
      {
         size_t i;
         const int maxObsPerLine(5);

            // loop over satellites in R3 obs data
         for( itr = rod.obs.begin(); itr != rod.obs.end(); ++itr )
         {

            RinexSatID sat(itr->first);               // current satellite
            string sys(string(1,sat.systemChar()));   // system
            itr = rod.obs.find(sat);           // get data vector to be written
            int obsWritten(0);
            line = string("");

               // loop over R2 obstypes
            for( i=0; i<strm.header.R2ObsTypes.size(); i++ )
            {

                  // get the R3 obs ID from the map
               RinexObsID obsid;
               obsid =
                 strm.header.mapSysR2toR3ObsID[sys][strm.header.R2ObsTypes[i]];

                 // now find index of that data from R3 header
               const vector<RinexObsID>& vecData(strm.header.mapObsTypes[sys]);

               vector<RinexObsID>::const_iterator jt;
               jt = find(vecData.begin(), vecData.end(), obsid);

               int ind(-1);                           // index into vecData

               if( jt != vecData.end() ) ind = jt-vecData.begin();

                  // need a continuation line?
               if( obsWritten != 0 && (obsWritten % maxObsPerLine) == 0 )
               {
                  strm << line << endl;
                  strm.lineNumber++;
                  line = string("");
               }

                  // write the line
               line += rightJustify(asString(            // double 14.3
                          ( ind == -1 ? 0.0 : itr->second[ind].data),3),14 );
               line += (ind == -1 || itr->second[ind].lli == 0)
                       ? string(1, ' ')
                       : rightJustify(asString<short>(itr->second[ind].lli),1);
               line += (ind == -1 || itr->second[ind].ssi == 0)
                       ? string(1, ' ')
                       : rightJustify(asString<short>(itr->second[ind].ssi),1);
               obsWritten++;

            }  // End of 'for( i=0; i<strm.header.R2ObsTypes.size(); i++ )'

            strm << line << endl;
            strm.lineNumber++;

         }  // End of 'for( itr = rod.obs.begin(); itr != rod.obs.end();...'

      }  // Ebf of 'else if( rod.epochFlag == 0 || rod.epochFlag == 1 || ...'

   }  // End of function 'reallyPutRecordVer2()'


      /* This method returns the RinexDatum of a given observation
       *
       * @param sat     Satellite whose observation we want to fetch.
       * @param index   Index representing the observation type. It is
       *                obtained from corresponding RINEX Observation Header
       *                using method 'Rinex3ObsHeader::getObsIndex()'.
       */
   RinexDatum Rinex3ObsData::getObs( const SatID& sat, int index ) const
      throw(InvalidRequest)
   {

      RinexSatID rsat(sat);

         // Look for the satellite in 'DataMap'
      Rinex3ObsData::DataMap::const_iterator it;
      it = obs.find(rsat);

         // Check if satellite was found
      if( it == obs.end() )
      {
         InvalidRequest ir( rsat.toString() + " is not available.");
         GPSTK_THROW(ir);
      }

         // Extract a copy of the data vector
      vector<RinexDatum> vecData(it->second);

         // Return the corresponding data
      return vecData[index];

   }  // End of method 'Rinex3ObsData::getObs()'


     /* This method returns the RinexDatum of a given observation
      *
      * @param sat  Satellite whose observation we want to fetch.
      * @param type String representing the observation type.
      * @param hdr  RINEX Observation Header for current RINEX file.
      */
   RinexDatum Rinex3ObsData::getObs( const SatID& sat, std::string type,
                                             const Rinex3ObsHeader& hdr ) const
      throw(InvalidRequest)
   {

         // We will need the system 'char' of the satellite
      RinexSatID rsat(sat);

         // Add GNSS code if needed
      if( type.size() == 3 )
      {
         char sysCode = rsat.systemChar();
         type = sysCode + type;
      }

         // Get the index corresponding to this observation type
      int index( hdr.getObsIndex(type) );

         // Return the corresponding data
      return getObs(sat, index);

   }  // End of method 'Rinex3ObsData::getValue( const SatID sat,...'


   void Rinex3ObsData::reallyPutRecord(FFStream& ffs) const
      throw(std::exception, FFStreamError, StringException)
   {
      // is there anything to write?
      if( (epochFlag == 0 || epochFlag == 1 || epochFlag == 6)
              && (numSVs==0 || obs.empty())) return;
//    if( (epochFlag >= 2 && epochFlag <= 5) &&
//         auxHeader.NumberHeaderRecordsToBeWritten() == 0 ) return;

      Rinex3ObsStream& strm = dynamic_cast<Rinex3ObsStream&>(ffs);

      // call the version for RINEX ver 2
      if(strm.header.version < 3) {
         try {
            reallyPutRecordVer2(strm, *this);
         }
         catch(Exception& e) { GPSTK_RETHROW(e); }
         return;
      }

      string line;

      // first the epoch line
      line  = ">";
      line += writeTime(time);
      line += string(2, ' ');
      line += rightJustify(asString<short>(epochFlag), 1);
      line += rightJustify(asString<short>(numSVs   ), 3);
      line += string(6, ' ');
      if(clockOffset != 0.0) // optional data; need to test for its existence
      line += rightJustify(asString(clockOffset, 12), 15);

      strm << line << endl;
      strm.lineNumber++;
      line.erase();

      if(epochFlag == 0 || epochFlag == 1 || epochFlag == 6) {
         DataMap::const_iterator itr = obs.begin();

         while(itr != obs.end()) {
            line = itr->first.toString();

            for(size_t i=0; i < itr->second.size(); i++) {
               RinexDatum thisData = itr->second[i];
               line += rightJustify(asString(thisData.data,3),14);

               if(thisData.lli == 0)
                  line += string(1, ' ');
               else
                  line += rightJustify(asString<short>(thisData.lli),1);

               if(thisData.ssi == 0)
                  line += string(1, ' ');
               else
                  line += rightJustify(asString<short>(thisData.ssi),1);
            }
            // write the data line out
            strm << line << endl;
            strm.lineNumber++;
            line.erase();

            itr++;
         } // end loop over sats and data
      }

      // write the auxiliary header records, if any
      else if(epochFlag >= 2 && epochFlag <= 5) {
         try {
            auxHeader.WriteHeaderRecords(strm);
         }
         catch(FFStreamError& e) { GPSTK_RETHROW(e); }
         catch(StringException& e) { GPSTK_RETHROW(e); }
      }

   }   // end Rinex3ObsData::reallyPutRecord

   void reallyGetRecordVer2(Rinex3ObsStream& strm, Rinex3ObsData& rod)
      throw(Exception)
   {
      static CommonTime previousTime(CommonTime::BEGINNING_OF_TIME);

      // get the epoch line and check
      string line;
      while(line.empty())        // ignore blank lines in place of epoch lines
         strm.formattedGetLine(line, true);

      if(line.size()>80 || line[0] != ' ' || line[3] != ' ' || line[6] != ' ') {
         FFStreamError e("Bad epoch line: >" + line + "<");
         GPSTK_THROW(e);
      }

      // process the epoch line, including SV list and clock bias
      rod.epochFlag = asInt(line.substr(28,1));
      if((rod.epochFlag < 0) || (rod.epochFlag > 6)) {
         FFStreamError e("Invalid epoch flag: " + asString(rod.epochFlag));
         GPSTK_THROW(e);
      }

      // Not all epoch flags are required to have a time.
      // Specifically, 0,1,5,6 must have an epoch time; it is optional for 2,3,4.
      // If there is and epoch time, parse it and load it in the member "time".
      // If epoch flag=0, 1, 5, or 6 and there is NO epoch time, then throw.
      // If epoch flag=2, 3, or 4 and there is no epoch time,
      // use the time of the previous record.
      bool noEpochTime = (line.substr(0,26) == string(26, ' '));
      if(noEpochTime && (rod.epochFlag==0 || rod.epochFlag==1 ||
                         rod.epochFlag==5 || rod.epochFlag==6 )) {
         FFStreamError e("Required epoch time missing: " + line);
         GPSTK_THROW(e);
      }
      else if(noEpochTime)
         rod.time = previousTime;
      else {
         try {
            // check if the spaces are in the right place - an easy
            // way to check if there's corruption in the file
            if((line[0] != ' ') || (line[3] != ' ') || (line[6] != ' ') ||
                 (line[9] != ' ') || (line[12] != ' ') || (line[15] != ' '))
            {
               FFStreamError e("Invalid time format");
               GPSTK_THROW(e);
            }

            // if there's no time, just use a bad time
            if(line.substr(0,26) == string(26, ' '))
               rod.time = CommonTime::BEGINNING_OF_TIME;
               //rod.time = previousTime; ??
            else {
               int year, month, day, hour, min;
               double sec;
               int yy = (static_cast<CivilTime>(strm.header.firstObs)).year/100;
               yy *= 100;

               year  = asInt(   line.substr(1,  2 ));
               month = asInt(   line.substr(4,  2 ));
               day   = asInt(   line.substr(7,  2 ));
               hour  = asInt(   line.substr(10, 2 ));
               min   = asInt(   line.substr(13, 2 ));
               sec   = asDouble(line.substr(15, 11));

               // Real Rinex has epochs 'yy mm dd hr 59 60.0' surprisingly often....
               double ds(0);
               if(sec >= 60.) { ds=sec; sec=0.0; }
               CivilTime rv(yy+year, month, day, hour, min, sec, TimeSystem::GPS);
               if(ds != 0) rv.second += ds;

               rod.time = rv.convertToCommonTime();
            }
         }
         // string exceptions for substr are caught here
         catch(exception &e)
         {
            FFStreamError err("std::exception: " + string(e.what()));
            GPSTK_THROW(err);
         }
         catch(Exception& e)
         {
            string text;
            for(size_t i=0; i<e.getTextCount(); i++) text += e.getText(i);
            FFStreamError err("gpstk::Exception in parseTime(): " + text);
            GPSTK_THROW(err);
         }
         // end rod.time = parseTime(line, strm.header);

         // save for next call
         previousTime = rod.time;
      }

      // number of satellites
      rod.numSVs = asInt(line.substr(29,3));

      // clock offset
      if(line.size() > 68 )
         rod.clockOffset = asDouble(line.substr(68, 12));
      else
         rod.clockOffset = 0.0;

      // Read the observations ...
      if(rod.epochFlag==0 || rod.epochFlag==1 || rod.epochFlag==6) {
         // first read the SatIDs off the epoch line
         int isv, ndx, line_ndx;
         string satsys;
         RinexSatID sat;
         vector<RinexSatID> satIndex(rod.numSVs);
         for(isv=1, ndx=0; ndx<rod.numSVs; isv++, ndx++) {
            if(!(isv % 13)) {                   // get a new continuation line
               strm.formattedGetLine(line);
               isv = 1;
               if(line.size() > 80) {
                  FFStreamError err("Invalid line size:" + asString(line.size()));
                  GPSTK_THROW(err);
               }
            }

            // read the sat id
            try {
               sat = RinexSatID(line.substr(30+isv*3-1, 3));
               satIndex[ndx] = sat;
               //// if this system does not have obs types assigned, do so
               //string satsys = asString(sat.systemChar());
               //if(strm.header.mapObsTypes[satsys].size() == 0) {
               //   strm.header.mapObsTypes[satsys] = strm.header.mapObsTypes["G"];
               //}
            }
            catch (Exception& e) {
               FFStreamError ffse(e);
               GPSTK_THROW(ffse);
            }
         }  // end loop over numSVs

         // loop over all sats, reading obs data
         int numObs(strm.header.R2ObsTypes.size());// number of R2 OTs in header
         rod.obs.clear();
         for(isv=0; isv < rod.numSVs; isv++) {
            //strm.formattedGetLine(line);           // get a line
            //line.resize(80, ' ');                  // pad just in case
            sat = satIndex[isv];                   // sat for this data
            satsys = asString(sat.systemChar());   // system for this sat
            vector<RinexDatum> data;
            // loop over data in the line
            for(ndx=0, line_ndx=0; ndx < numObs; ndx++, line_ndx++) {
               if(! (line_ndx % 5)) {              // get a new line
                  strm.formattedGetLine(line);
                  line.resize(80, ' ');            // pad just in case
                  line_ndx = 0;
                  if(line.size() > 80) {
                     FFStreamError err("Invalid line size:" + asString(line.size()));
                     GPSTK_THROW(err);
                  }
               }

               // does this R2 OT map into a valid R3 ObsID?
               string R2ot(strm.header.R2ObsTypes[ndx]);
               string R3ot(strm.header.mapSysR2toR3ObsID[satsys][R2ot].asString());
               if(R3ot != string("   ")) {
                  RinexDatum tempData;
                  tempData.data = asDouble(line.substr(line_ndx*16,   14));
                  tempData.lli =     asInt(line.substr(line_ndx*16+14, 1));
                  tempData.ssi =     asInt(line.substr(line_ndx*16+15, 1));
                  data.push_back(tempData);
               }
            }
            rod.obs[sat] = data;

         }  // end loop over sats to read obs data
      }

      // ... or the auxiliary header information
      else if(rod.numSVs > 0) {
         rod.auxHeader.clear();
         for(int i=0; i<rod.numSVs; i++)
         {
            strm.formattedGetLine(line);
            StringUtils::stripTrailing(line);
            try {
               rod.auxHeader.ParseHeaderRecord(line);
            }
            catch(FFStreamError& e) { GPSTK_RETHROW(e); }
            catch(StringException& e) { GPSTK_RETHROW(e); }
         }
      }
   }  // end void reallyGetRecordVer2(Rinex3ObsStream& strm, Rinex3ObsData& rod)


   void Rinex3ObsData::reallyGetRecord(FFStream& ffs)
      throw(exception, FFStreamError, gpstk::StringUtils::StringException)
   {
      Rinex3ObsStream& strm = dynamic_cast<Rinex3ObsStream&>(ffs);

      // If the header hasn't been read, read it.
      if(!strm.headerRead) strm >> strm.header;

      // call the version for RINEX ver 2
      if(strm.header.version < 3) {
         try {
            reallyGetRecordVer2(strm, *this);
         }
         catch(Exception& e) { GPSTK_RETHROW(e); }
         return;
      }

      string line;
      Rinex3ObsData rod;

      // clear out this ObsData
      *this = rod;

      // read the first (epoch) line
      strm.formattedGetLine(line, true);

      // Check and parse the epoch line -----------------------------------
      // Check for epoch marker ('>') and following space.
      if(line[0] != '>' || line[1] != ' ') {
         FFStreamError e("Bad epoch line: >" + line + "<");
         GPSTK_THROW(e);
      }

      epochFlag = asInt(line.substr(31,1));
      if(epochFlag < 0 || epochFlag > 6) {
         FFStreamError e("Invalid epoch flag: " + asString(epochFlag));
         GPSTK_THROW(e);
      }

      time = parseTime(line, strm.header, strm.timesystem);

      numSVs = asInt(line.substr(32,3));

      if(line.size() > 41)
         clockOffset = asDouble(line.substr(41,15));
      else
         clockOffset = 0.0;

      // Read the observations: SV ID and data ----------------------------
      if(epochFlag == 0 || epochFlag == 1 || epochFlag == 6) {
         vector<RinexSatID> satIndex(numSVs);
         map<RinexSatID, vector<RinexDatum> > tempDataMap;

         for(int isv = 0; isv < numSVs; isv++) {
            strm.formattedGetLine(line);

            // get the SV ID
            try {
               satIndex[isv] = RinexSatID(line.substr(0,3));
            }
            catch (Exception& e) {
               FFStreamError ffse(e);
               GPSTK_THROW(ffse);
            }

            // get the # data items (# entries in ObsType map of maps from header)
            string gnss = asString(satIndex[isv].systemChar());
            int size = strm.header.mapObsTypes[gnss].size();

            // Some receivers leave blanks for missing Obs (which is OK by RINEX 3).
            // If the last Obs are the ones missing, it won't necessarily be padded
            // with spaces, so the parser will break.  This adds the padding to let
            // the parser do its job and interpret spaces as zeroes.
            size_t minSize = 3 + 16*size;
            if(line.size() < minSize)
               line += string(minSize-line.size(), ' ');

            // get the data (# entries in ObsType map of maps from header)
            vector<RinexDatum> data;
            for(int i = 0; i < size; i++) {
               size_t pos = 3 + 16*i;
               RinexDatum tempData;
               tempData.data = asDouble(line.substr(pos   , 14));
               if( line.size() > pos+14 )
                  tempData.lli  = asInt( line.substr(pos+14,  1));
               if( line.size() > pos+15 )
                  tempData.ssi  = asInt( line.substr(pos+15,  1));
               data.push_back(tempData);
            }
            obs[satIndex[isv]] = data;
         }
      }

      // ... or the auxiliary header information
      else if(numSVs > 0) {
         auxHeader.clear();
         for(int i = 0; i < numSVs; i++) {
            strm.formattedGetLine(line);
            StringUtils::stripTrailing(line);
            try {
               auxHeader.ParseHeaderRecord(line);
            }
            catch(FFStreamError& e) { GPSTK_RETHROW(e); }
            catch(StringException& e) { GPSTK_RETHROW(e); }
         }
      }

      return;

   } // end of reallyGetRecord()


   CommonTime Rinex3ObsData::parseTime(const string& line,
                                       const Rinex3ObsHeader& hdr,
                                       const TimeSystem& ts) const
      throw(FFStreamError)
   {
   try {
      // check if the spaces are in the right place - an easy
      // way to check if there's corruption in the file
      if( (line[ 1] != ' ') || (line[ 6] != ' ') || (line[ 9] != ' ') ||
          (line[12] != ' ') || (line[15] != ' ') || (line[18] != ' ') ||
          (line[29] != ' ') || (line[30] != ' '))
      {
         FFStreamError e("Invalid time format");
         GPSTK_THROW(e);
      }

      // if there's no time, just return a bad time
      if(line.substr(2,27) == string(27, ' '))
         return CommonTime::BEGINNING_OF_TIME;

      int year, month, day, hour, min;
      double sec;

      year  = asInt(   line.substr( 2,  4));
      month = asInt(   line.substr( 7,  2));
      day   = asInt(   line.substr(10,  2));
      hour  = asInt(   line.substr(13,  2));
      min   = asInt(   line.substr(16,  2));
      sec   = asDouble(line.substr(19, 11));

      // Real Rinex has epochs 'yy mm dd hr 59 60.0' surprisingly often.
      double ds = 0;
      if(sec >= 60.) { ds = sec; sec = 0.0; }

      CommonTime rv = CivilTime(year,month,day,hour,min,sec).convertToCommonTime();
      if(ds != 0) rv += ds;

      rv.setTimeSystem(ts);

      return rv;
   }
   // string exceptions for substr are caught here
   catch (std::exception &e) {
      FFStreamError err("std::exception: " + string(e.what()));
      GPSTK_THROW(err);
   }
   catch (gpstk::Exception& e) {
      string text;
     for(size_t i=0; i<e.getTextCount(); i++) text += e.getText(i);
      FFStreamError err("gpstk::Exception in parseTime(): " + text);
      GPSTK_THROW(err);
   }
   }  // end parseTime

   string Rinex3ObsData::writeTime(const CommonTime& ct) const
      throw(StringException)
   {
      if(ct == CommonTime::BEGINNING_OF_TIME)
         return string(26, ' ');

      CivilTime civtime(ct);
      string line;

      line  = string(1, ' ');
      line += rightJustify(asString<short>(civtime.year    ), 4);
      line += string(1, ' ');
      line += rightJustify(asString<short>(civtime.month   ), 2, '0');
      line += string(1, ' ');
      line += rightJustify(asString<short>(civtime.day     ), 2, '0');
      line += string(1, ' ');
      line += rightJustify(asString<short>(civtime.hour    ), 2, '0');
      line += string(1, ' ');
      line += rightJustify(asString<short>(civtime.minute  ), 2, '0');
      line += rightJustify(asString       (civtime.second,7),11);

      return line;
   }  // end writeTime


   void Rinex3ObsData::dump(ostream& s) const
   {
      if(obs.empty())
         return;

      s << "Dump of Rinex3ObsData" << endl << " - time: " << writeTime(time)
         << " epochFlag: " << " " << epochFlag
         << " numSVs: " << numSVs
         << fixed << setprecision(9) << " clk offset: " << clockOffset << endl;

      if(epochFlag == 0 || epochFlag == 1) {
         DataMap::const_iterator jt;
         for(jt = obs.begin(); jt != obs.end(); jt++) {
            s << " " << (jt->first).toString() << ":" << fixed << setprecision(3);
            for(size_t i = 0; i < jt->second.size(); i++)
            {
               s << " " << setw(12) << jt->second[i].data
                  << "/" << jt->second[i].lli << "/" << jt->second[i].ssi;
            }
            s << endl;
         }
      }
      else {
         s << "aux. header info:\n";
         auxHeader.dump(s);
      }
   }  // end dump


   void Rinex3ObsData::dump(ostream& os, Rinex3ObsHeader& head) const
   {
      os << "Dump of Rinex3ObsData: "
         << printTime(time,"%4F/%w/%10.3g = %04Y/%02m/%02d %02H:%02M:%02S")
         << " flag " << epochFlag << " NSVs " << numSVs
         << fixed << setprecision(6) << " clk " << clockOffset;

      if(obs.empty()) { os << " : EMPTY" << endl; return; }
      else os << endl;

      if(epochFlag >= 2) {
         os << "Auxiliary header:\n";
         auxHeader.dump(os);
         return;
      }

      for(DataMap::const_iterator jt=obs.begin(); jt != obs.end(); jt++) {
         RinexSatID sat(jt->first);
         const vector<RinexObsID> types(head.mapObsTypes[sat.toString().substr(0,1)]);
         os << " " << sat.toString() << fixed << setprecision(3);
         for(size_t i=0; i<jt->second.size(); i++)
            os << " " << setw(13) << jt->second[i].data
               << "/" << jt->second[i].lli
               << "/" << jt->second[i].ssi
               << "/" << types[i].asString()
               ;
         os << endl;
      }

   }

} // namespace
