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





//
//
#include <stdio.h>
// gpstk
#include "SVExclusionList.hpp"
#include "CommonTime.hpp"
#include "TimeString.hpp"


namespace gpstk
{

   using namespace std; 

//--------------- Methods for SVExclusionList ---------------
   SVExclusionList::SVExclusionList( ) 
     :earliestTime(gpstk::CommonTime::END_OF_TIME),
      latestTime(gpstk::CommonTime::BEGINNING_OF_TIME)
   {
      readFailCount = 0;
      timeSpecString = "%F %g";
   }
   
   SVExclusionList::SVExclusionList( const std::string filename ) 
             throw(SVExclusionList::SVExclusionFileNotFound)
     :earliestTime(gpstk::CommonTime::END_OF_TIME),
      latestTime(gpstk::CommonTime::BEGINNING_OF_TIME)
   {
      readFailCount = 0;
      timeSpecString = "%F %g";
      addFile( filename );
   }
   
   void SVExclusionList::addFile( const std::string filename )
               throw(SVExclusionList::SVExclusionFileNotFound)
   {
      char file[100];
      sscanf(filename.c_str(),"%s",file);
      FILE* inf = fopen(file, "rt");
      if (inf==0)
      {
         char text[200];
         sprintf(text,"Exclusion file not found.  Filename: %s",filename.c_str());
         std::string sout = text;
         SVExclusionFileNotFound noSVXFile( sout );
         GPSTK_THROW(noSVXFile);
      }
      
      std::string temps;
      CommonTime tempDTs;
      std::string tempe;
      CommonTime tempDTe;
      int lineCount =0;
      char fileLine[200];
      while (fgets(fileLine, 200, inf))
      {
         lineCount++;
         //cout << "------------- Line " << lineCount << endl;
         string whiteSpace = " \t\n\r";
         string lineIn = fileLine;
         
            // strip trailing whitespace
         string::size_type endIndex = lineIn.find_last_not_of(whiteSpace);
         lineIn = lineIn.substr( 0, endIndex+1 );
         string lead2Chars = lineIn.substr(0,2);
         if (lead2Chars.compare("TS")==0)
         {
            string::size_type q1 = lineIn.find('"');
            string::size_type q2 = lineIn.find('"',(q1+1));
            if (q1!=string::npos && q2!=string::npos)
            {
               timeSpecString = lineIn.substr((q1+1),(q2-q1-1));
                  // debug
               //cout << " New timeSpecString '" << timeSpecString << "'." << endl;
            }
            else
            {
               readFailCount++;
               string failString = buildFailString(
                  "Invalid TS specification at",
                  lineCount,
                  filename);
               readFailList.push_back( failString );
            }
         }
         if (lead2Chars.compare("EX")==0)
         {
            string::size_type c1 = lineIn.find(',');
            string::size_type c2 = lineIn.find(',',(c1+1));
            string::size_type c3 = lineIn.find(',',(c2+1));
            if (c1!=string::npos && c2!=string::npos)
            {
               std::string comment = "";
               int PRNID = StringUtils::asInt(lineIn.substr(2, (c1-1)) );
               if (PRNID<0 || PRNID>gpstk::MAX_PRN)
               {
                  readFailCount++;
                  string failString = buildFailString(
                     "PRN ID out of range",
                     lineCount,
                     filename);
                  readFailList.push_back( failString );
                  continue;
               }
               temps = lineIn.substr((c1+1),(c2-c1-1));
               string::size_type nonWhiteBeg = temps.find_first_not_of(whiteSpace);
               string::size_type nonWhiteEnd = temps.find_last_not_of(whiteSpace);
               //cout << "Before trimming.  temps, Beg, End = '" << temps << "', " << nonWhiteBeg << ", " << nonWhiteEnd << "." << endl;
               if (nonWhiteEnd!=string::npos)
                   temps = temps.substr(nonWhiteBeg,nonWhiteEnd-nonWhiteBeg+1);
               else
                   temps = temps.substr(nonWhiteBeg);
               
               //cout << "c3, string::npos = " << c3 << ", " << string::npos << endl;
               if (c3!=string::npos)
               {
                  tempe = lineIn.substr(c2+1, (c3-c2-1) );
                  comment = lineIn.substr(c3+1);
                  nonWhiteBeg = comment.find_first_not_of(whiteSpace);
                  comment = comment.substr(nonWhiteBeg);
               }
               else
                  tempe = lineIn.substr(c2+1);

               nonWhiteBeg = tempe.find_first_not_of(whiteSpace);
               nonWhiteEnd = tempe.find_last_not_of(whiteSpace);
               //cout << "Before trimming.  tempe, Beg, End = '" << tempe << "', " << nonWhiteBeg << ", " << nonWhiteEnd << "." << endl;
               if (nonWhiteEnd!=string::npos)
                   tempe = tempe.substr(nonWhiteBeg,nonWhiteEnd-nonWhiteBeg+1);
               else
                   tempe = tempe.substr(nonWhiteBeg);
               try
               {
                  //cout << "Input start string: '" << temps << "'." << endl;
                  scanTime(tempDTs, temps, timeSpecString );
                  //cout << "Input   end string: '" << tempe << "'." << endl;
                  scanTime(tempDTe, tempe, timeSpecString );
                  if (tempDTs<=tempDTe)
                  {
                     SVExclusion svEx( tempDTs, tempDTe, PRNID, comment );


       
                        // Add exclusion to the multimap
                     addExclusion( svEx );
                  }
                  else
                  {
                     readFailCount++;
                     string failString = buildFailString(
                        "Start time after end time",
                        lineCount,
                        filename);
                     readFailList.push_back( failString );
                  }
               }
               catch (InvalidRequest& ir)
               {
                  readFailCount++;
                  string failString = buildFailString(
                     ir.getText(),
                     lineCount,
                     filename);
                  readFailList.push_back( failString );
               }
               catch (StringUtils::StringException& se)
               {
                  readFailCount++;
                  string failString = buildFailString(
                     se.getText(),
                     lineCount,
                     filename);
                     readFailList.push_back( failString );
               }
            }
            else
            {
               readFailCount++;
               string failString = buildFailString(
                  "Invalid EX line format at",
                  lineCount,
                  filename);
               readFailList.push_back( failString );
            }
         }
      }
      fclose(inf);
   }

   void SVExclusionList::addExclusion( const SVExclusion svx )
   {
         // The sun compiler's *really* picky about its consts and says this doesn't work:
         //exclusionMap.insert( make_pair( svx.getPRNID(), svx ) );

         // Here's some tedium to make it happy:
      pair<const int, gpstk::SVExclusion> temp( svx.getPRNID(), svx );
      exclusionMap.insert( temp );
      
         // Update the earliest/latest info
      if (svx.getBeginTime() < earliestTime) earliestTime = svx.getBeginTime();
      if (svx.getEndTime() > latestTime) latestTime = svx.getEndTime();
   }

   bool SVExclusionList::isExcluded( 
                   const int PRN, 
                   const gpstk::CommonTime dt ) const
   {
      SVXListPair p = exclusionMap.equal_range( PRN );
      for (SVXListCI ci=p.first; ci != p.second; ++ci)
      {
         if (ci->second.isApplicable( PRN, dt )) return(true);
      }
      return(false);
   }
   
   const SVExclusion& SVExclusionList::getApplicableExclusion(
                     const int PRN, const gpstk::CommonTime dt) 
                     const throw(SVExclusionList::NoSVExclusionFound)
   {
      SVXListPair p = exclusionMap.equal_range( PRN );
      for (SVXListCI ci=p.first; ci != p.second; ++ci)
      {
         if (ci->second.isApplicable( PRN, dt )) return( ci->second );
      }
      
         // Failed to find an exclusion corresponding to the request
      char textOut[80];
      sprintf(textOut,"No SVExclusion found for PRN %02d at %s.",
         PRN, printTime(dt,"week %F SOW %g, %02m/%02d/%02y %02H:%02M:%02S").c_str());
      std::string sout = textOut;
      NoSVExclusionFound noSVX( sout );
      GPSTK_THROW(noSVX);
   }

   void SVExclusionList::dumpList( FILE* fp ) const
   {
      if (fp==0) return;
      std::string timeString = "Wk %F SOW %6.0g, %02m/%02d/%02y (DOY %03j) %02H:%02M:%02S";
      fprintf(fp,"List of SV Exclusion from SVExclusionList\n\n");
      for (int PRN=1; PRN<=gpstk::MAX_PRN; ++PRN)
      {
         fprintf(fp,"\nExclusions for PRN %02d\n",PRN);
         SVXListPair p = exclusionMap.equal_range( PRN );
         for (SVXListCI ci=p.first; ci != p.second; ++ci)
         {
            fprintf(fp,"  %s to %s\n",
               printTime(ci->second.getBeginTime(),timeString).c_str(),
               printTime(ci->second.getEndTime(),timeString).c_str());
         }
      }
   }
   
   void SVExclusionList::listOfReadFailures() const
   {
      typedef list<string>::const_iterator LI;
      for (LI i=readFailList.begin(); i!=readFailList.end(); ++i)
      {
         cerr << *i << endl;
      }
   }
   
   void SVExclusionList::listOfReadFailures( FILE* fpout ) const
   {
      if (fpout==0) return;
      typedef list<string>::const_iterator LI;
      for (LI i=readFailList.begin(); i!=readFailList.end(); ++i)
      {
         fprintf(fpout,"%s\n",(*i).c_str());
      }
   }
   
   void SVExclusionList::listOfReadFailures( std::ofstream fsout ) const
   {
      if (!fsout.is_open()) return;
      typedef list<string>::const_iterator LI;
      for (LI i=readFailList.begin(); i!=readFailList.end(); ++i)
      {
         fsout << *i << endl;
      }
   }
   
   std::string SVExclusionList::buildFailString(const std::string s,
                     const int lineCount, const std::string filename )
   {
      string outString = s;
      outString += " at line "; 
      outString += StringUtils::asString(lineCount);
      outString += " of file "; 
      outString += filename;
      outString += ".";
      return(outString);
   }                     
   
//--------------- Methods for SVExclusion ---------------
   SVExclusion::SVExclusion( const gpstk::CommonTime begin, 
                             const gpstk::CommonTime end,
                             const int PRNID, 
                             const std::string commentArg )
   {
      begExclude = begin;
      endExclude = end;
      PRN_IDENTIFIER = PRNID;
      comment = commentArg;
   }

   bool SVExclusion::isApplicable( const int PRNID, const gpstk::CommonTime dt ) const
   {
      if (dt>=begExclude && dt<=endExclude && PRN_IDENTIFIER==PRNID) return(true);
      return(false);
   }

}     // end namespace gpstk
