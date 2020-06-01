/// @file SP3Data.cpp
/// Encapsulate SP3 file data, including I/O

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

#include "SP3Stream.hpp"
#include "SP3Header.hpp"
#include "SP3Data.hpp"
#include "StringUtils.hpp"
#include "CivilTime.hpp"
#include "GPSWeekSecond.hpp"

using namespace gpstk::StringUtils;
using namespace std;

namespace gpstk
{
   void SP3Data::reallyGetRecord(FFStream& ffs)
      throw(exception, FFStreamError, StringException)
   {
      // cast the stream to be an SP3Stream
      SP3Stream& strm = dynamic_cast<SP3Stream&>(ffs);

      // status says which records have been written already this call
      // status = 1 if an epoch record was read
      // status = 2 if an P or V record was read
      // status = 3 if an EP or EV record was read
      int status = 0;

      // version to be written out is determined by written header (stored in strm)
      //bool isVerA = (strm.header.getVersion() == SP3Header::SP3a);
      bool isVerC = (strm.header.getVersion() == SP3Header::SP3c);

      // correlation flag will be set if there is an EP/EV record
      correlationFlag = false;

      // default for the 'c' arrays is all zeros
      sdev[0] = sdev[1] = sdev[2] = sdev[3] = 0;
      sig[0] = sig[1] = sig[2] = sig[3] = 0;
      correlation[0] = correlation[1] = correlation[2]
         = correlation[3] = correlation[4] = correlation[5] = 0;

      // TimeSystem for this stream
      TimeSystem timeSystem;
      timeSystem.fromString(strm.header.timeSystemString());

      // loop until an error occurs, or until the entire record (which may consist
      // of two lines) is read.
      bool unexpectedEOF=false;
      while(1) {

         // set the time to be the last epoch read by the stream
         time = strm.currentEpoch;

         // process the lastLine string stored in strm - contains the last line read
         // empty record
         if(strm.lastLine.size() < 3) {      // 3 b/c it may contain "EOF"
            // nothing in lastLine - do nothing here, get another line
            ;
         }

         // EOF has been read
         else if(strm.lastLine.substr(0,3) == string("EOF")) {
            // if a data record has already been processed during this call,
            // then return here, and let the next call process this EOF.
            // This gives the caller a chance to process the data before hitting EOF.

            // if an epoch record was processed this call, that's an error
            if(status == 1) {
               FFStreamError ffse("EOF was found immediately after epoch line");
               GPSTK_THROW(ffse);
            }

            // some other record was processed this call, so quit
            if(status > 1) break;

            // this next read had better fail - if it does not, then there is
            // an "EOF" that is followed by something other than the file EOF.
            try {
               strm.formattedGetLine(strm.lastLine, true);     // true is 'expect EOF'
            }
            catch(EndOfFile& err) {
               break; // normal exit; the stream will now be at eof()
            }
         
            // the GetLine succeeded, so this is an error
            FFStreamError err("EOF text found, followed by: " + strm.lastLine);
            GPSTK_THROW(err);
         }

         // Epoch line read
         else if(strm.lastLine[0] == '*') {

            // if an epoch record was already processed this call, that's an error
            // TD consider simply removing this if(status==1) entirely. Some SP3 files
            // particularly those generated from a realtime stream, have consecutive
            // epoch lines. Why would we not just ignore the epoch with no data?
            if(status == 1) {
               FFStreamError ffse("Consecutive epoch records found");
               GPSTK_THROW(ffse);
            }

            // if another record has been process during this call, quit now
            if(status > 1) break;

            // process an epoch record
            status = 1;

            // mark this record as non-data, in case another P|V record is not read
            sat = SatID();

            // warn if the line is short but not too short
            if(strm.lastLine.size() <= 30 && strm.lastLine.size() > 26)
               strm.warnings.push_back(string("Warning (SP3 std): short epoch line: ")
                                       + strm.lastLine);

            // throw if the line is short
            if(strm.lastLine.size() <= 26) { // some igs files cut seconds short 30)
               FFStreamError err("Invalid line length "
                                + asString(strm.lastLine.size()));
               GPSTK_THROW(err);                  
            }

            // parse the epoch line
            RecType = strm.lastLine[0];
            int year = asInt(strm.lastLine.substr(3,4));
            int month = asInt(strm.lastLine.substr(8,2));
            int dom = asInt(strm.lastLine.substr(11,2));
            int hour = asInt(strm.lastLine.substr(14,2));
            int minute = asInt(strm.lastLine.substr(17,2));
            double second = asInt(strm.lastLine.substr(20,10));
            CivilTime t;
            try {
               t = CivilTime(year, month, dom, hour, minute, second, timeSystem);
            }
            catch (gpstk::Exception& e) {
               FFStreamError fe("Invalid time in:" + strm.lastLine);
               GPSTK_THROW(fe);
            }          
            time = strm.currentEpoch = static_cast<CommonTime>(t);
         }

         // P or V record read
         else if(strm.lastLine[0] == 'P' || strm.lastLine[0] == 'V') {

            // if another record has been process during this call, quit now
            if(status > 0) break;

            // process this P|V
            status = 2;
            RecType = strm.lastLine[0];     // P or V

            // if its version c and the line is short (<80) but valid(>59),
            // then add blanks at the end to make it 80 character.
            if (isVerC && strm.lastLine.size() < 80 && strm.lastLine.size() > 59)
               leftJustify(strm.lastLine,80);

            // throw if the line is short
            if ((!isVerC && strm.lastLine.size() < 60) ||
                 (isVerC && strm.lastLine.size() < 80) ) {
               FFStreamError err("Invalid line length ("
                                  + asString(strm.lastLine.size())
                                  + ") for line:\n" + strm.lastLine);
               GPSTK_THROW(err);
            }

            // parse the line
            sat = static_cast<SatID>(SP3SatID(strm.lastLine.substr(1,3)));

            x[0] = asDouble(strm.lastLine.substr(4,14));             // XYZ
            x[1] = asDouble(strm.lastLine.substr(18,14));
            x[2] = asDouble(strm.lastLine.substr(32,14));
            clk = asDouble(strm.lastLine.substr(46,14));             // Clock

            // the rest is version c only
            if(isVerC) {
               sig[0] = asInt(strm.lastLine.substr(61,2));           // sigma XYZ
               sig[1] = asInt(strm.lastLine.substr(64,2));
               sig[2] = asInt(strm.lastLine.substr(67,2));
               sig[3] = asInt(strm.lastLine.substr(70,3));           // sigma clock

               if(RecType == 'P') {                                  // P flags
                  clockEventFlag = clockPredFlag
                     = orbitManeuverFlag = orbitPredFlag = false;
                  if(strm.lastLine[74] == 'E') clockEventFlag = true;
                  if(strm.lastLine[75] == 'P') clockPredFlag = true;
                  if(strm.lastLine[78] == 'M') orbitManeuverFlag = true;
                  if(strm.lastLine[79] == 'P') orbitPredFlag = true;
               }
            }
         }

         // EP or EV correlation record read
         else if(strm.lastLine[0] == 'E' &&
            (strm.lastLine[1] == 'P' || strm.lastLine[1] == 'V'))
         {
            // throw if correlation record did not follow corresponding P|V record
            if(status != 2 || strm.lastLine[1] != RecType) {
               Exception e("correlation EP|V record mismatched with previous record");
               GPSTK_THROW(e);
            }

            // process EP|V record
            status = 3;

            // throw if line is short
            if(strm.lastLine.size()<80) {
               FFStreamError err("Invalid SP3c correlation line length ("
                                  + asString(strm.lastLine.size())
                                  + ") for line:\n" + strm.lastLine);
               GPSTK_THROW(err);
            }

            // parse the line
            sdev[0] = abs(asInt(strm.lastLine.substr(4,4)));
            sdev[1] = abs(asInt(strm.lastLine.substr(9,4)));
            sdev[2] = abs(asInt(strm.lastLine.substr(14,4)));
            sdev[3] = abs(asInt(strm.lastLine.substr(19,7)));
            correlation[0] = asInt(strm.lastLine.substr(27,8));
            correlation[1] = asInt(strm.lastLine.substr(36,8));
            correlation[2] = asInt(strm.lastLine.substr(45,8));
            correlation[3] = asInt(strm.lastLine.substr(54,8));
            correlation[4] = asInt(strm.lastLine.substr(63,8));
            correlation[5] = asInt(strm.lastLine.substr(72,8));

            // tell the caller that correlation data is now present
            correlationFlag = true;
         }

         else {                              // Unknown record
            FFStreamError err("Unknown record label " + strm.lastLine.substr(0,2));
            GPSTK_THROW(err);
         }

         // be tolerant of files without EOF -- IGS!
         // if previous iteration of the loop found unexpected EOF, then quit here.
         if(unexpectedEOF) {
            // add a warning
            strm.warnings.push_back(string("Warning (SP3 std): Unexpected EOF"));

            // clear the buffer so it won't be (re)processed next call
            strm.lastLine = string("EOF");

            // clear the eof bit and return, so the user will process this SP3Data
            strm.clear(ios::eofbit);
            status = 3;
         }
         else {            // normal flow
            // read next line into the lastLine
            try {
               strm.formattedGetLine(strm.lastLine);
            }
            catch(FFStreamError& err) {
               string what = err.what().substr(0,21);
               //cout << "Found unexpected EOF" << endl;
               //cout << "what is " << what << endl;
               //cout << "Here is the buffer:\n" << strm.lastLine << endl;
               if(what == string("text 0:Unexpected EOF")) {
                  unexpectedEOF = true;
   
                  // there could still be unprocessed data in the buffer:
                  if(strm.lastLine.size() < 3) status = 3;  // nothing there, quit
                  else                         status = 0;  // go back and process it
               }
               else
                  GPSTK_RETHROW(err);
            }
         }

         if(status == 3) break;  // quit if found EOF or EP|EV was processed, but
                                 // go back if lastLine was empty  (0)
                                 //      or if epoch was processed (1)
                                 //      or if P|V was processed   (2)

      }  // end while loop processing records

   }   // end reallyGetRecord()

   void SP3Data::reallyPutRecord(FFStream& ffs) const 
      throw(exception, FFStreamError, StringException)
   {
      string line;

      // cast the stream to be an SP3Stream
      SP3Stream& strm = dynamic_cast<SP3Stream&>(ffs);

      // version to be written out is determined by written header (stored in strm)
      bool isVerA = (strm.header.getVersion() == SP3Header::SP3a);
      bool isVerC = (strm.header.getVersion() == SP3Header::SP3c);

      // output Epoch Header Record
      if(RecType == '*') {
         CivilTime civTime(time);
         line = "* ";
         line += civTime.printf(" %4Y %2m %2d %2H %2M");
         line += " " + rightJustify(civTime.printf("%.8f"),11);
      }

      // output Position and Clock OR Velocity and Clock Rate Record
      else {
         line = RecType;                                    // P or V
         if(isVerA) {
            if(sat.system != SatID::systemGPS) {
               FFStreamError fse("Cannot output non-GPS to SP3a");
               GPSTK_THROW(fse);
            }
            line += rightJustify(asString(sat.id),3);
         }
         else
            line += static_cast<SP3SatID>(sat).toString();  // sat ID

         line += rightJustify(asString(x[0],6),14);         // XYZ
         line += rightJustify(asString(x[1],6),14);
         line += rightJustify(asString(x[2],6),14);
         line += rightJustify(asString(clk,6),14);          // Clock

         if(isVerC) {
            line += rightJustify(asString(sig[0]),3);       // sigma XYZ
            line += rightJustify(asString(sig[1]),3);
            line += rightJustify(asString(sig[2]),3);
            line += rightJustify(asString(sig[3]),4);       // sigma Clock
            
            if(RecType == 'P') {                            // flags or blanks
               line += string(" ");
               line += (clockEventFlag ? string("E") : string(" "));
               line += (clockPredFlag ? string("P") : string(" "));
               line += string("  ");
               line += (orbitManeuverFlag ? string("M") : string(" "));
               line += (orbitPredFlag ? string("P") : string(" "));
            }
         }

         // if version is 'c' and correlation flag is set,
         // then output the P|V Correlation Record
         if(isVerC && correlationFlag) {

            // first output the P|V record you just built
            strm << line << endl;
            strm.lineNumber++;

            // now build and output the correlation record
            if(RecType == 'P')                                 // P or V
               line = "EP ";
            else
               line = "EV ";
            line += rightJustify(asString(sdev[0]),5);         // stddev X
            line += rightJustify(asString(sdev[1]),5);         // stddev Y
            line += rightJustify(asString(sdev[2]),5);         // stddev Z
            line += rightJustify(asString(sdev[3]),8);         // stddev Clk
            for(int i=0; i<6; i++)                             // correlations
               line += rightJustify(asString(correlation[i]),9);
         }
      }

      // write the line just built
      strm << line << endl;
      strm.lineNumber++;

   }  // end reallyPutRecord()

   void SP3Data::dump(ostream& s, bool includeC) const throw()
   {
      // dump record type (PV*), sat id, and current epoch
      s << RecType << " " << static_cast<SP3SatID>(sat).toString() << " "
         << (static_cast<CivilTime>(time)).printf("%Y/%02m/%02d %2H:%02M:%06.3f")
         << " = "
         << (static_cast<GPSWeekSecond>(time)).printf("%F/%10.3g");

      if(RecType != '*') {                   // not epoch line
         s << fixed << setprecision(6)
           << " X=" << setw(14) << x[0]      // XYZ
           << " Y=" << setw(14) << x[1]
           << " Z=" << setw(14) << x[2]
           << " C=" << setw(14) << clk;      // clk

         if(includeC) {
            s << " sX=" << setw(2) << sig[0]    // sigma XYZ
              << " sY=" << setw(2) << sig[1]
              << " sZ=" << setw(2) << sig[2]
              << " sC=" << setw(3) << sig[3];   // sigma clock

            if(RecType == 'P')                  // flags
              s << " " << (clockEventFlag ? "clockEvent" : "-")
                << " " << (clockPredFlag ? "clockPrediction" : "-")
                << " " << (orbitManeuverFlag ? "orbitManeuver" : "-")
                << " " << (orbitPredFlag ? "orbitPrediction" : "-");

            if(correlationFlag)                 // stddevs and correlations
               s << endl
                 << "    and E" << RecType
                 << " cXX=" << setw(4) << sdev[0]
                 << " cYY=" << setw(4) << sdev[1]
                 << " cZZ=" << setw(4) << sdev[2]
                 << " cCC=" << setw(7) << sdev[3]
                 << " cXY=" << setw(8) << correlation[0]
                 << " cXZ=" << setw(8) << correlation[1]
                 << " cXC=" << setw(8) << correlation[2]
                 << " cYZ=" << setw(8) << correlation[3]
                 << " cYC=" << setw(8) << correlation[4]
                 << " cZC=" << setw(8) << correlation[5];
         }
      }

      s << endl;

   }  // end dump()

} // namespace
