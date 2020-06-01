#pragma ident "$Id$"

/**
 * @file Rinex3ClockHeader.cpp
 * Encapsulate header of RINEX3 clock file, including I/O
 * See more at: ftp://igscb.jpl.nasa.gov/pub/data/format/rinex_clock.txt
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
//  Octavian Andrei - FGI ( http://www.fgi.fi ). 2008-2010
//
//============================================================================

//system
#include<cmath>
//GPSTk
#include "CivilTime.hpp"
#include "Rinex3ClockHeader.hpp"
#include "Rinex3ClockStream.hpp"
#include "StringUtils.hpp"

using namespace std;
using namespace gpstk::StringUtils;

namespace gpstk
{
   const string Rinex3ClockHeader::versionString     =  "RINEX VERSION / TYPE";
   const string Rinex3ClockHeader::runByString       =  "PGM / RUN BY / DATE";
   const string Rinex3ClockHeader::commentString     =  "COMMENT";
   const string Rinex3ClockHeader::numObsString      =  "SYS / # / OBS TYPES";
   const string Rinex3ClockHeader::timeSystemString  =  "TIME SYSTEM ID";
   const string Rinex3ClockHeader::leapSecondsString =  "LEAP SECONDS";
   const string Rinex3ClockHeader::sysDCBString      =  "SYS / DCBS APPLIED";
   const string Rinex3ClockHeader::sysPCVString      =  "SYS / PCVS APPLIED";
   const string Rinex3ClockHeader::dataTypesString   =  "# / TYPES OF DATA";
   const string Rinex3ClockHeader::stationNameString =  "STATION NAME / NUM";
   const string Rinex3ClockHeader::calibrationClkString = "STATION CLK REF";
   const string Rinex3ClockHeader::acNameString      =   "ANALYSIS CENTER";
   const string Rinex3ClockHeader::numRefClkString   =  "# OF CLK REF";
   const string Rinex3ClockHeader::analysisClkRefString = "ANALYSIS CLK REF";
   const string Rinex3ClockHeader::numStationsString = "# OF SOLN STA / TRF";
   const string Rinex3ClockHeader::solnStaNameString =  "SOLN STA NAME / NUM";
   const string Rinex3ClockHeader::numSatsString     =  "# OF SOLN SATS";
   const string Rinex3ClockHeader::prnListString     =  "PRN LIST";
   const string Rinex3ClockHeader::endOfHeader       =  "END OF HEADER";


   const RinexClkType
      Rinex3ClockHeader::UN("UN", "Unknown or Invalid");
   const RinexClkType
      Rinex3ClockHeader::AR("AR", "analysis data for receiver clocks");
   const RinexClkType
      Rinex3ClockHeader::AS("AS", "analysis data for satellite clocks");
   const RinexClkType
      Rinex3ClockHeader::CR("CR", "calibration data");
   const RinexClkType
      Rinex3ClockHeader::DR("DR", "discontinuity data");
   const RinexClkType
      Rinex3ClockHeader::MS("MS", "monitor data");

      // Clear (empty out) header
   void Rinex3ClockHeader::clear(void)
   {

      version = 0.0;
      valid = 0;
      numObsTyp = 0;
      numSta = 0;
      numSVs = 0;
      leapSeconds = 0;
      ac = "";
      acName = "";
      fileProgram = "";
      fileAgency = "";
      date = "";
      clk0Name = "";
      calName = "";
      trfName = "";
      commentList.clear();
      obsTypeList.clear();
      dataTypeList.clear();
      refClkList.clear();
      clkNameList.clear();
      staCoordList.clear();
      timeFirst = CommonTime::BEGINNING_OF_TIME;

      return;

   }  // End of method 'Rinex3ClockHeader::clear()'



   void Rinex3ClockHeader::reallyPutRecord(FFStream& ffs) const
      throw(std::exception, FFStreamError, StringException)
   {
      cout << "WARNING: There is no implementation for "
           << "Rinex3ClockHeader::reallyPutRecord()"
           << endl;

      return;

   }  // End of method 'Rinex3ClockHeader::reallyPutRecord()'



      // This function parses the entire header from the given stream
   void Rinex3ClockHeader::reallyGetRecord(FFStream& ffs)
      throw(std::exception, FFStreamError,
            StringUtils::StringException)
   {

      Rinex3ClockStream& strm = dynamic_cast<Rinex3ClockStream&>(ffs);

         // if already read, just return
      if (strm.headerRead == true)
         return;

         // since we're reading a new header, we need to reinitialize
         // all our list structures.  all the other objects should be ok.
         // this also applies if we threw an exception the first time we read
         // the header and are now re-reading it. some of these could be full
         // and we need to empty them.
      clear();

         // one file line
      string line;

      while ( !(valid & endValid) )
      {
         strm.formattedGetLine(line);
         StringUtils::stripTrailing(line);

         if ( line.length() == 0 )
         {
            FFStreamError ffse("No data read!");
            GPSTK_THROW(ffse);
         }
         else if ( line.length() < 60 || line.length() > 80 )
         {
            FFStreamError ffse("Invalid line length");
            GPSTK_THROW(ffse);
         }

         try
         {
            ParseHeaderRecord(line);
         }
         catch(FFStreamError& ffse)
         {
            GPSTK_RETHROW(ffse);
         }

      }   // end while(not end of header)


         // If we get here, we should have reached the end of header line
      strm.header = *this;
      strm.headerRead = true;

      return;

   }  // End of method 'Rinex3ClockHeader::reallyGetRecord(FFStream& ffs)'



      // this function parses a single header record
   void Rinex3ClockHeader::ParseHeaderRecord(string& line)
      throw(FFStreamError)
   {

      string label(line, 60, 20);

         // RINEX VERSION / TYPE
      if ( label == versionString )
      {

         version  =  asDouble(line.substr(0,9));
         fileType =  strip(line.substr(20, 20));

            // check version
         if ( version <= 0.0 ||
              version > 3.0 )
         {
            FFStreamError e( "This isn't an anticipated version number." +
                              asString(version) );
            GPSTK_THROW(e);
         }

            // check type
         if ( (fileType[0] != 'C') &&
              (fileType[0] != 'c'))
         {
            FFStreamError e( "This isn't a Rinex3 Clock file type." );
            GPSTK_THROW(e);
         }

            // get satellite system
         string system_str = strip(line.substr(40, 20));
         try
         {
            system.fromString(system_str);
         }
         catch (Exception& e)
         {
            FFStreamError ffse( "Input satellite system is unsupported: "
                                 + system_str + e.getText() );
            GPSTK_THROW(ffse);
         }

         valid |= versionValid;

      }
         // PGM / RUN BY / DATE
      else if ( label == runByString )
      {

         fileProgram =  strip(line.substr( 0, 20));
         fileAgency  =  strip(line.substr(20, 20));
         date        =  strip(line.substr(40, 20));
         isPGM = true;

         valid |= runByValid;

      }
         // COMMENT
      else if ( label == commentString )
      {

         string s = strip(line.substr(0, 60));
         commentList.push_back(s);

         valid |= commentValid;

      }
         // SYS / # / OBS TYPES
      else if ( label == numObsString )
      {

         numObsTyp   =  asInt( line.substr(3,3) );

         // TODO: more work needed

         valid |= numObsValid;

      }
         // TIME SYSTEM ID
      else if ( label == timeSystemString )
      {

         timeSystem = line.substr(3,3);
         valid |= timeSystemValid;

      }
         // LEAP SECONDS
      else if ( label == leapSecondsString )
      {

         leapSeconds = asInt(line.substr(0,6));

         valid |= leapSecondsValid;

      }
         // DCBS APPLIED
      else if ( label == sysDCBString )
      {

         valid |= sysDCBsValid;

      }
         // PCVS APPLIED
      else if ( label == sysPCVString )
      {

         valid |= sysPCVsValid;

      }
         // # / TYPES OF DATA
      else if ( label == dataTypesString )
      {

            // number of clock data types
         int nTyp = asInt(line.substr(0,6));
            // allocate memory
         dataTypeList.resize(nTyp);
            // add clock data types
         for( int iTyp = 0; iTyp < nTyp; iTyp++ )
         {
            dataTypeList[iTyp] = strip( line.substr(6*(iTyp+1),6) );
         }

         valid |= dataTypesValid;

      }
         // STATION NAME / NUM
      else if ( label == stationNameString )
      {

         clk0Name = line.substr(0,4);

         valid |= stationNameValid;

      }
         // STATION CLK REF
      else if ( label == calibrationClkString )
      {
         calName = strip( line.substr(0,60) );

         valid |= calibrationClkValid;

      }
         // ANALYSIS CENTER
      else if ( label == acNameString )
      {

         ac       =  line.substr(0, 3);
         acName   =  strip(line.substr(5,55));
         isAC     =  true;

         valid |= acNameValid;

      }
         // # OF CLK REF
      else if ( label == numRefClkString )
      {

            // new reference clock record
         RefClkRecord ref;
            // get the number of reference clocks for this record
         ref.nRef = asInt( line.substr(0,6) );
            // epoch
         if( asInt(line.substr(7,4)) )
         {
            CommonTime T0 = parseTime( line.substr(7,26) );
               // only one time
            if( timeFirst == CommonTime::BEGINNING_OF_TIME )
            {
               timeFirst = T0;
            }
               // left boundary
            ref.refWin[0] = T0 - timeFirst;
               // right boundary
            T0 = parseTime( line.substr(34,26) );
            ref.refWin[1] = T0 - timeFirst;

               // time inconsistency
            if (T0 < timeFirst)
            {
               FFStreamError e( "Wrong epoch of file, expected epoch: " +
                                 timeFirst.asString() + " detected epoch: " +
                                 T0.asString() );
               GPSTK_THROW(e);
            }
         }

            // add the ref clk record to the list
         refClkList.push_back(ref);

         valid |= numRefClkValid;

      }
         /// ANALYSIS CLK REF
      else if ( label == analysisClkRefString )
      {

            // get the previous reference clock record
         std::list<RefClkRecord>::iterator iRef = refClkList.end();
         --iRef;

            // how many ref clk have been stored
         size_t nClks = iRef->clk.size();

            // is there any inconsistency?
         if ( nClks < iRef->nRef )
         {

            RefClk refclk;
               // reference clock info
            refclk.name    =  line.substr(0,4);
            refclk.sigma   =  asDouble( strip( line.substr(40,20) ) );
            refclk.sigma   *= 1e6; // ms^2
               // add into the list
            iRef->clk.push_back(refclk);

         }
         else
         {
            FFStreamError e( string("Number of items found in header ") +
                             "is inconsitent to the entry in header" );
            GPSTK_THROW(e);
         }

         valid |= analysisClkRefValid;

      }
         /// # OF SOLN STA / TRF
      else if ( label == numStationsString )
      {

         numSta   =  asInt(line.substr( 0,  6));
         trfName  =  strip(line.substr(10, 50));

         valid |= numStationsValid;

      }
         /// SOLN STA NAME / NUM
      else if ( label == solnStaNameString )
      {

            // get 4-character station name
         string name = line.substr(0,4);
            // add it into the list
         clkNameList.push_back( name );

            // get integer & decimal part of the coordinates
         int X    =  asInt( strip(line.substr(25,8)) );
         int Xf   =  asInt( strip(line.substr(33,3)) );

         int Y    =  asInt( strip(line.substr(37,8)) );
         int Yf   =  asInt( strip(line.substr(45,3)) );

         int Z    =  asInt( strip(line.substr(49,8)) );
         int Zf   =  asInt( strip(line.substr(57,3)) );

            // geocentric coordinates (be careful to the sign)
         double x = X>0 ? X*1.0+Xf*1e-3 : X*1.0-Xf*1e-3;
         double y = Y>0 ? Y*1.0+Yf*1e-3 : Y*1.0-Yf*1e-3;
         double z = Z>0 ? Z*1.0+Zf*1e-3 : Z*1.0-Zf*1e-3;

            // check the coordinates
         double radius = std::sqrt(x*x+y*y+z*z);
            // add them into the map
         if (radius >= 5000.0e3 && radius < 12000.0e3)
         {
            staCoordList.push_back( Triple(X,Y,Z) );
         }
         else
         {
            staCoordList.push_back( Triple(0.0,0.0,0.0) );
         }

         valid |= solnStaNameValid;

      }
         // # OF SOLN SATS
      else if ( label == numSatsString )
      {

         numSVs = asInt(line.substr(0,6));

         valid |= numSatsValid;

      }
         // PRN LIST
      else if ( label == prnListString )
      {

         string s    =  line.substr(0,60);
         string word =  stripFirstWord(s);

         while ( !word.empty() )
         {
            clkNameList.push_back( word.append(" ") );
            word = stripFirstWord(s);
         }

         valid |= prnListValid;

      }
         // END OF HEADER
      else if ( label == endOfHeader )
      {

         valid |= endValid;

      }
      else
      {

         FFStreamError e("Unidentified label: " + label);
         GPSTK_THROW(e);

      }

      return;

   }   // End of method 'Rinex3ClockHeader::ParseHeaderRecord(string& line)'



      /** This function sets the time for this header.
       * It looks at \a line to obtain the needed information.
       */
   CommonTime Rinex3ClockHeader::parseTime(const string& line) const
   {

      int year, month, day, hour, min;
      double sec;

      year  = asInt(   line.substr( 0, 4 ));
      month = asInt(   line.substr( 4, 3 ));
      day   = asInt(   line.substr( 7, 3 ));
      hour  = asInt(   line.substr(10, 3 ));
      min   = asInt(   line.substr(13, 3 ));
      sec   = asDouble(line.substr(16, 10));

      return CivilTime(year, month, day, hour, min, sec).convertToCommonTime();

   }  // End of method 'Rinex3ClockHeader::parseTime(const string& line)'


      /// Converts the CommonTime \a dt into a Rinex3 Clock time
      /// string for the header
   string Rinex3ClockHeader::writeTime(const CommonTime& dt) const
   {

      if (dt == CommonTime::BEGINNING_OF_TIME)
      {
         return string(36, ' ');
      }

      string line;
      CivilTime civTime(dt);
      line  = rightJustify(asString<short>(civTime.year), 4);
      line += rightJustify(asString<short>(civTime.month), 3);
      line += rightJustify(asString<short>(civTime.day), 3);
      line += rightJustify(asString<short>(civTime.hour), 3);
      line += rightJustify(asString<short>(civTime.minute), 3);
      line += rightJustify(asString(civTime.second, 6), 10);

      return line;

   }  // End of method 'Rinex3ClockHeader::writeTime(const CommonTime& dt)'



      // Debug output function.
   void Rinex3ClockHeader::dump(ostream& s) const
   {
      size_t i;
      s << "---------------------------------- REQUIRED ----------------------------------\n";
      string str;
      str = system.systemChar();
      str = str + " (" + system.systemString() + ")";
      s << "Rinex Version " << fixed << setw(4) << setprecision(1) << version
         << ",  File type " << fileType
         << ",  System " << str
         << endl;
      s << "Prgm: " << fileProgram
         << ",  Run: " << date
         << ",  By: " << fileAgency
        << endl;
      s << "Clock data types (" << dataTypeList.size() << ") :" << endl;
      for(i=0; i<dataTypeList.size(); i++)
         s << " Type #" << i << " = "
            << " " << dataTypeList[i] << endl;
      if(valid & acNameValid)
         s << "Analysis Center: " << ac << " (" << acName << ")" << endl;
      if(valid & numRefClkValid)
         s << "Number of analysis clock references: " << refClkList.size() << endl;
      if(valid & analysisClkRefValid) {
         for(std::list<RefClkRecord>::const_iterator it = refClkList.begin();
            it != refClkList.end();
            it++)
         {
            s << "CLK REF  ";
            for(std::list<RefClk>::const_iterator jt = it->clk.begin();
            jt != it->clk.end();
            jt++)
            {
               s << setw(5) << jt->name;
               s << " from " << setw(7) << it->refWin[0]
                 << " to "   << setw(7) << it->refWin[1];
            }
            s << endl;
         }
         s << endl;
      }
      if(valid & numStationsValid)
         s << "Number of Stations with data : " << numSta << endl;
      if(valid & solnStaNameValid) {
         s << "STA  ";
         for(int j=0; j < numSta; j++)
            s << setw(5) << clkNameList[j];
         s << endl;
      }
      if(valid & numSatsValid)
         s << "Number of Satellites with data : " << numSVs << endl;
      if(valid & prnListValid) {
         s << "SAT  ";
         for(i=numSta; i<clkNameList.size(); i++)
            s << setw(5) << clkNameList[i];
         s << endl;
      }
      s << "(This header is ";
      if(valid)
      {
         if (version == 3.0) s << "VALID 3.0";
         if (version == 2.0) s << "VALID 2.0";
      }
      else s << "NOT VALID";
      s << " Rinex Clock.)\n";

      if(!(valid & versionValid)) s << " Version is NOT valid\n";
      if(!(valid & runByValid)) s << " Run by is NOT valid\n";
      if(!(valid & numObsValid)) s << " Observation type is NOT valid\n";
      if(!(valid & timeSystemValid)) s << " Time system is NOT valid\n";
      if(!(valid & sysDCBsValid)) s << " DCBs applied is NOT valid\n";
      if(!(valid & sysPCVsValid)) s << " PCVs applied is NOT valid\n";
      if(!(valid & stationNameValid)) s << " Station name is NOT valid\n";
      if(!(valid & calibrationClkValid)) s << " External reference clock is NOT valid\n";
      if(!(valid & acNameValid)) s << " Analysis Center is NOT valid\n";
      if(!(valid & numRefClkValid)) s << " Number of analysis clock references is NOT valid\n";
      if(!(valid & analysisClkRefValid)) s << " List of the analysis clock references is NOT valid\n";
      if(!(valid & solnStaNameValid)) s << " Number of receivers is NOT valid\n";
      if(!(valid & numSatsValid)) s << " Number of satellites is NOT valid\n";
      if(!(valid & prnListValid)) s << " PRN list is NOT valid\n";
      if(!(valid & endValid)) s << " End is NOT valid\n";

      s << "---------------------------------- OPTIONAL ----------------------------------\n";
      if(valid & leapSecondsValid) s << "Leap seconds: " << leapSeconds << endl;
      if(commentList.size() && !(valid & commentValid)) s << " Comment is NOT valid\n";
      s << "Comments (" << commentList.size() << ") :\n";
      for(i=0; i<commentList.size(); i++)
         s << commentList[i] << endl;
      s << "-------------------------------- END OF HEADER -------------------------------\n";

   }  // End of method 'Rinex3ClockHeader::dump(ostream& s)'


}  // End of namespace gpstk
