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

/**
 * @file YumaData.cpp
 * Encapsulate Yuma almanac file data, including I/O
 */

#include "StringUtils.hpp"
#include "GNSSconstants.hpp"

#include "YumaData.hpp"
#include "YumaStream.hpp"


using namespace gpstk::StringUtils;
using namespace std;

namespace gpstk
{
   short YumaData::nearFullWeek = 0;

   const std::string YumaData::sID   = "ID:";
   const std::string YumaData::sHlth = "Health:";
   const std::string YumaData::sEcc  = "Eccentricity:";
   const std::string YumaData::sTOA  = "Time of Applicability(s):";
   const std::string YumaData::sOrbI = "Orbital Inclination(rad):";
   const std::string YumaData::sRRA  = "Rate of Right Ascen(r/s):";
   const std::string YumaData::sSqrA = "SQRT(A)  (m 1/2):";
   const std::string YumaData::sRtAs = "Right Ascen at Week(rad):";
   const std::string YumaData::sArgP = "Argument of Perigee(rad):";
   const std::string YumaData::sMnAn = "Mean Anom(rad):";
   const std::string YumaData::sAf0  = "Af0(s):";
   const std::string YumaData::sAf1  = "Af1(s/s):";
   const std::string YumaData::sweek = "week:";

   void YumaData::reallyPutRecord(FFStream& ffs) const
      throw(std::exception, FFStreamError,
               gpstk::StringUtils::StringException)
   {
      YumaStream& strm = dynamic_cast<YumaStream&>(ffs);

      const int width=27;
      strm << "******** Week" << setw(5) << (week % 1024)
           << " almanac for PRN-" << PRN
           << " ********" << endl
           << left
           << setw(width) << sID   << PRN << endl
           << setw(width) << sHlth << hex << SV_health << endl
           << setw(width) << sEcc  << ecc << endl
           << setw(width) << sTOA  << Toa << endl
           << setw(width) << sOrbI << (i_offset + 54.0 * (gpstk::PI / 180.0)) << endl
           << setw(width) << sRRA  << OMEGAdot << endl
           << setw(width) << sSqrA << Ahalf << endl
           << setw(width) << sRtAs << OMEGA0 << endl
           << setw(width) << sArgP << w << endl
           << setw(width) << sMnAn << M0 << endl
           << setw(width) << sAf0  << AF0 << endl
           << setw(width) << sAf1  << AF1 << endl
           << setw(width) << sweek << week << endl;
   }   // end YumaData::reallyPutRecord


   string YumaData::lineParser(const string& line, const string& s)
      const throw(FFStreamError)
   {
      int i = line.find_first_of(":");

      // Gotta have a colon or the format is wrong
      if (i == string::npos)
         GPSTK_THROW(FFStreamError("Format error in YumaData"));

      // Only compare the first five characters since some files differ after that
      int w = std::min(5, std::min(i, (int)s.size()));
      if (line.substr(0,w) != s.substr(0,w))
         GPSTK_THROW(FFStreamError("Format error in YumaData"));

      return stripLeading(line.substr(i+1), " ");
   }


   void YumaData::reallyGetRecord(FFStream& ffs)
      throw(std::exception, FFStreamError,
               gpstk::StringUtils::StringException)
   {
      YumaStream& strm = dynamic_cast<YumaStream&>(ffs);

      string line;

      // We don't need first line as we will get all the information from the others
      strm.formattedGetLine(line, true);

      //Second Line - PRN
      strm.formattedGetLine(line, true);
      PRN = asInt(lineParser(line, sID));

      //Third Line - Satellite Health
      strm.formattedGetLine(line, true);
      SV_health = asInt(lineParser(line, sHlth));

      //Fourth Line - Eccentricity
      strm.formattedGetLine(line, true);
      ecc = asDouble(lineParser(line, sEcc));

      //Fifth Line - Time of Applicability
      strm.formattedGetLine(line, true);
      Toa = (long) asDouble(lineParser(line, sTOA));

      //Sixth Line - Orbital Inclination
      strm.formattedGetLine(line, true);
      double i_total = asDouble(lineParser(line, sOrbI));
      i_offset = i_total - 54.0 * (gpstk::PI / 180.0);

      //Seventh Line - Rate of Right Ascen
      strm.formattedGetLine(line, true);
      OMEGAdot = asDouble(lineParser(line, sRRA));

      //Eigth Line - SqrtA
      strm.formattedGetLine(line, true);
      Ahalf = asDouble(lineParser(line, sSqrA));

      //Ninth Line - Right Ascen at Week
      strm.formattedGetLine(line, true);
      OMEGA0 = asDouble(lineParser(line, sRtAs));

      //Tenth Line - Argument of Perigee
      strm.formattedGetLine(line, true);
      w = asDouble(lineParser(line, sArgP));

      //Eleventh Line - Mean Anomaly
      strm.formattedGetLine(line, true);
      M0 = asDouble(lineParser(line, sMnAn));

      //Twelfth Line - Af0
      strm.formattedGetLine(line, true);
      AF0 = asDouble(lineParser(line, sAf0));

      //Thirteenth Line - Af1
      strm.formattedGetLine(line, true);
      AF1 = asDouble(lineParser(line, sAf1));

      //Fourteenth Line - week
      // Its unclear whether this is a full week or week % 1024
      strm.formattedGetLine(line, true);
      week = asInt(lineParser(line, sweek));

      if (nearFullWeek > 0)
      {
            // In case a full week is provided.
         week %= 1024;
         week += (nearFullWeek / 1024) * 1024;
         short diff = nearFullWeek - week;
         if (diff > 512)
            week += 512;
         else if(diff < -512)
            week -= 512;
      }

      xmit_time = 0;
      strm.formattedGetLine(line,true);

   } // end of reallyGetRecord()

   void YumaData::dump(ostream& s) const
   {
      cout << "PRN = " << PRN << endl;
      cout << "week = " << week << endl;
      cout << "SV_health = " << SV_health << endl;
      cout << "ecc = " << ecc << endl;
      cout << "Toa = " << Toa << endl;
      cout << "i_offset = " << i_offset << endl;
      cout << "OMEGAdot = " << OMEGAdot << endl;
      cout << "Ahalf = " << Ahalf << endl;
      cout << "OMEGA0 = " << OMEGA0 << endl;
      cout << "w = " << w << endl;
      cout << "M0 = " << M0 << endl;
      cout << "AF0 = " << AF0 << endl;
      cout << "AF1 = " << AF1 << endl;
      cout << "xmit_time = " << xmit_time << endl;

   } // end of dump()

   YumaData::operator AlmOrbit() const
   {
      AlmOrbit ao(PRN, ecc,i_offset, OMEGAdot, Ahalf, OMEGA0,
                   w, M0, AF0, AF1, Toa, xmit_time, week, SV_health);

      return ao;

   } // end of AlmOrbit()
} // namespace
