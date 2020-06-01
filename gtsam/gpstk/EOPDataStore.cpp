#pragma ident "$Id$"

/**
* @file EOPDataStore.cpp
* 
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
//  Wei Yan - Chinese Academy of Sciences . 2011
//
//============================================================================


#include "EOPDataStore.hpp"
#include "MiscMath.hpp"
#include <fstream>
#include "MJD.hpp"
#include "Exception.hpp"

namespace gpstk
{
   using namespace std;
  
      // Add to the store directly
   void EOPDataStore::addEOPData(const CommonTime& utc,
                                  const EOPDataStore::EOPData& d)
      throw(InvalidRequest)
   {
      if(!(utc.getTimeSystem()==TimeSystem::UTC)) throw Exception();

      std::vector<double> data(5,0.0);
      
      data[0] = d.xp;
      data[1] = d.yp;
      data[2] = d.UT1mUTC;
      
      data[3] = d.dPsi;
      data[4] = d.dEps;

      addData(utc, data);

   }  // End of 'EOPDataStore::addEOPData()'

   
   EOPDataStore::EOPData EOPDataStore::getEOPData(const CommonTime& utc) const
         throw(InvalidRequest)
   {
      if(!(utc.getTimeSystem()==TimeSystem::UTC)) throw Exception();
	  
      std::vector<double> data = getData(utc);

      return EOPData(data[0],data[1],data[2],data[3],data[4]);
   
   }  // End of method 'EOPDataStore::getEOPData()'


      
   void EOPDataStore::loadIERSFile(std::string iersFile)
      throw(FileMissingException)
   {
      ifstream inpf(iersFile.c_str());
      if(!inpf) 
      {
         FileMissingException fme("Could not open IERS file " + iersFile);
         GPSTK_THROW(fme);
      }
      
      clear();

      bool ok (true);
      while(!inpf.eof() && inpf.good()) 
      {
         string line;
         getline(inpf,line);
         StringUtils::stripTrailing(line,'\r');
         if(inpf.eof()) break;

         // line length is actually 185
         if(inpf.bad() || line.size() < 70) { ok = false; break; }

         double mjd = StringUtils::asDouble(line.substr(7,8));      
         double xp = StringUtils::asDouble(line.substr(18,9));      // arcseconds
         double yp = StringUtils::asDouble(line.substr(37,9));      // arcseconds
         double UT1mUTC = StringUtils::asDouble(line.substr(58,10));// arcseconds
         
         double dPsi(0.0);
         double dEps(0.0);    
         if(line.size()>=185)
         {
            dPsi = StringUtils::asDouble(line.substr(165,10))/1000.0;   //
            dEps = StringUtils::asDouble(line.substr(175,10))/1000.0;   // 
         }
         
         addEOPData(MJD(mjd,TimeSystem::UTC), EOPData(xp,yp,UT1mUTC,dPsi,dEps));
      };
      inpf.close();

      if(!ok) 
      {
         FileMissingException fme("IERS File " + iersFile 
                                  + " is corrupted or wrong format");
         GPSTK_THROW(fme);
      }
   }

   void EOPDataStore::loadIGSFile(std::string igsFile)
      throw(FileMissingException)
   {
      ifstream inpf(igsFile.c_str());
      if(!inpf) 
      {
         FileMissingException fme("Could not open IERS file " + igsFile);
         GPSTK_THROW(fme);
      }

      clear();

      // first we skip the header section
      // skip the header

      //version 2
      //EOP  SOLUTION
      //  MJD         X        Y     UT1-UTC    LOD   Xsig   Ysig   UTsig LODsig  Nr Nf Nt     Xrt    Yrt  Xrtsig Yrtsig   dpsi    deps
      //               10**-6"        .1us    .1us/d    10**-6"     .1us  .1us/d                10**-6"/d    10**-6"/d        10**-6

      string temp;
      getline(inpf,temp);	
      getline(inpf,temp);  
      getline(inpf,temp);  
      getline(inpf,temp);  

      bool ok (true);
      while(!inpf.eof() && inpf.good()) 
      {
         string line;
         getline(inpf,line);
         StringUtils::stripTrailing(line,'\r');
         if(inpf.eof()) break;

         // line length is actually 185
         if(inpf.bad() || line.size() < 120) { ok = false; break; }

         istringstream istrm(line);
         
         double mjd(0.0),xp(0.0),yp(0.0),UT1mUTC(0.0),dPsi(0.0),dEps(0.0);
         
         istrm >> mjd >> xp >> yp >> UT1mUTC;

         for(int i=0;i<12;i++) istrm >> temp;

         istrm >> dPsi >> dEps;
         
         xp *= 1e-6;
         yp *= 1e-6;
         UT1mUTC *= 1e-7;
         
         dPsi *= 1e-6;
         dEps *= 1e-6;

         addEOPData(MJD(mjd,TimeSystem::UTC), EOPData(xp,yp,UT1mUTC,dPsi,dEps));
      };
      inpf.close();

      if(!ok) 
      {
         FileMissingException fme("IERS File " + igsFile
                                  + " is corrupted or wrong format");
         GPSTK_THROW(fme);
      }
   }

      /** Add EOPs to the store via a flat STK file. 
       *  EOP-v1.1.txt
       *  http://celestrak.com/SpaceData/EOP-format.asp
       *
       *  @param stkFile  Name of file to read, including path.
       */
   void EOPDataStore::loadSTKFile(std::string stkFile)
      throw(FileMissingException)
   {
      std::ifstream fstk(stkFile.c_str());

      int  numData = 0;
      bool bData = false;

      std::string buf;
      while(getline(fstk,buf))
      {   
         if(buf.substr(0,19) == "NUM_OBSERVED_POINTS")
         {
            numData = StringUtils::asInt(buf.substr(20));
            continue;
         }
         else if(buf.substr(0,14) == "BEGIN OBSERVED")
         {
            bData = true;
            continue;
         }
         else if(buf.substr(0,13) == "END PREDICTED")
         {
            bData = false;
            break;
         }
         if(!StringUtils::isDigitString(buf.substr(0,4)))
         {
            // for observed data and predicted data
            continue;
         }

         if(bData)
         {
            // # FORMAT(I4,I3,I3,I6,2F10.6,2F11.7,4F10.6,I4)
            //int year = StringUtils::asInt(buf.substr(0,4));
            //int month = StringUtils::asInt(buf.substr(4,3));
            //int day = StringUtils::asInt(buf.substr(7,3));
            double mjd = StringUtils::asInt(buf.substr(10,6));

            double xp = StringUtils::asDouble(buf.substr(16,10));
            double yp = StringUtils::asDouble(buf.substr(26,10));
            double UT1mUTC = StringUtils::asDouble(buf.substr(36,11));
            double dPsi = 0.0;
            double dEps = 0.0;

            addEOPData(MJD(mjd,TimeSystem::UTC), EOPData(xp,yp,UT1mUTC,dPsi,dEps));
         }

      }  // End of 'while'

      fstk.close();
   }

   ostream& operator<<(std::ostream& os, const EOPDataStore::EOPData& d)
   {
      os << " " << setw(18) << setprecision(8) << d.xp
         << " " << setw(18) << setprecision(8) << d.yp
         << " " << setw(18) << setprecision(8) << d.UT1mUTC
         << " " << setw(18) << setprecision(8) << d.dPsi
         << " " << setw(18) << setprecision(8) << d.dEps;


      return os;
   }

}  // End of namespace 'gpstk'






