/// @file Rinex3EphemerisStore.cpp
/// Read and store RINEX formated navigation message (Rinex3Nav) data, following
/// the RINEX 3.02 spec. Support for GNSS GPS, GAL, GLO, BDS, QZS.

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

#include "Rinex3EphemerisStore.hpp"

#include "Rinex3NavStream.hpp"
#include "Rinex3NavData.hpp"
#include "GPSEphemeris.hpp"
#include "GalEphemeris.hpp"
#include "BDSEphemeris.hpp"
#include "QZSEphemeris.hpp"

using namespace std;

namespace gpstk
{
   // add a Rinex3NavData to the store
   // @param Rinex3NavData Rdata data to be added
   // @return true if data was added, false otherwise
   bool Rinex3EphemerisStore::addEphemeris(const Rinex3NavData& inRdata)
   {
      Rinex3NavData Rdata(inRdata);
      TimeSystem ts = TimeSystem::GPS;
#pragma unused(ts)
       
      switch(Rdata.sat.system) {
         case SatID::systemGPS:
         {
            Rdata.time = correctTimeSystem(Rdata.time, TimeSystem::GPS);
            GPSEphemeris eph(Rdata);
            return ORBstore.addEphemeris(dynamic_cast<OrbitEph*>(&eph));
            break;
         }

         case SatID::systemGalileo:
         {
            Rdata.time = correctTimeSystem(Rdata.time, TimeSystem::GAL);
            GalEphemeris eph(Rdata);
            return ORBstore.addEphemeris(dynamic_cast<OrbitEph*>(&eph));
            break;
         }

         case SatID::systemBeiDou:
         {
            Rdata.time = correctTimeSystem(Rdata.time, TimeSystem::BDT);
            BDSEphemeris eph(Rdata);
            return ORBstore.addEphemeris(dynamic_cast<OrbitEph*>(&eph));
            break;
         }

         case SatID::systemQZSS:
         {
            Rdata.time = correctTimeSystem(Rdata.time, TimeSystem::QZS);
            QZSEphemeris eph(Rdata);
            return ORBstore.addEphemeris(dynamic_cast<OrbitEph*>(&eph));
            break;
         }

         case SatID::systemGlonass:
            Rdata.time = correctTimeSystem(Rdata.time, TimeSystem::GLO);
            return GLOstore.addEphemeris(GloEphemeris(Rdata));
            break;

         case SatID::systemGeosync:
            //return GEOstore.addEphemeris(GeoEphemeris(Rdata));
            break;

         default:
            break;
      }
      return false;
   }

   // load the given Rinex navigation file
   // return -1 failed to open file,
   //        -2 failed to read header (this->Rhead),
   //        -3 failed to read data (this->Rdata),
   //       >=0 number of nav records read
   int Rinex3EphemerisStore::loadFile(const string& filename, bool dump, ostream& s)
   {
      try {
         int nread(0);
         Rinex3NavStream strm;
         what = string();
         
         strm.open(filename.c_str(), ios::in);
         if(!strm.is_open()) {
            what = string("File ") + filename + string(" could not be opened.");
            return -1;
         }
         strm.exceptions(ios::failbit);

         try { strm >> Rhead; }
         catch(Exception& e) {
            what = string("Failed to read header of file ") + filename
               + string(" : ") + e.getText();
            return -2;
         }
         if(dump) Rhead.dump(s);

         // add to FileStore
         NavFiles.addFile(filename, Rhead);

         // add to mapTimeCorr
         if(Rhead.mapTimeCorr.size() > 0) {
            map<string, TimeSystemCorrection>::const_iterator it;
            for(it=Rhead.mapTimeCorr.begin(); it!=Rhead.mapTimeCorr.end(); ++it)
               addTimeCorr(it->second);
         }

         while(1) {
            // read the record
            try { strm >> Rdata; }
            catch(Exception& e) {
               what = string("Failed to read data in file ") + filename
                  + string(" : ") + e.getText();
               return -3;
            }
            catch(exception& e) {
               what = string("std excep: ") + e.what();
               return -3;
            }
            catch(...) {
               what = string("Unknown exception while reading data of file ")
                  + filename;
               return -3;
            }

            if(!strm.good() || strm.eof()) break;

            nread++;
            if(dump) Rdata.dump(s);

            try {
               addEphemeris(Rdata);
            }
            catch(Exception& e) {
               cout << "addEphemeris caught excp " << e.what();
               GPSTK_RETHROW(e);
            }
         }

         return nread;
      }
      catch(Exception& e) {
         GPSTK_RETHROW(e);
      }

   } // end Rinex3EphemerisStore::loadFile

   // Find the appropriate time system correction object in the collection for the
   // given time systems, and dump it to a string and return that string.
   string Rinex3EphemerisStore::dumpTimeSystemCorrection(
      const TimeSystem fromSys, const TimeSystem toSys) const
   {
      string msg;
      ostringstream oss;
      oss << "Convert from " << fromSys.asString()
         << " to " << toSys.asString() << " : ";

      if(toSys == fromSys) {
         oss << "time systems are the same";
         return oss.str();
      }

      // look up the TimeSystemCorr in list, and dump it
      map<string, TimeSystemCorrection>::const_iterator it;
      for(it = mapTimeCorr.begin(); it != mapTimeCorr.end(); ++it) {
         if(it->second.isConverterFor(fromSys, toSys)) {
            it->second.dump(oss);
            return oss.str();
         }
      }
      oss << "conversion not found!";
      return oss.str();
   }

   // Utility routine for getXvt and addEphemeris to convert time systems.
   // Convert ttag to the target time system, using the first appropriate correction
   // in the list, and return it. If no correction is found, ttag is unchanged and
   // an exception is thrown.
   CommonTime Rinex3EphemerisStore::correctTimeSystem(const CommonTime ttag,
                                                      const TimeSystem targetSys)
      const
   {
      CommonTime toReturn(ttag);
      TimeSystem fromSys(ttag.getTimeSystem());

      // is a conversion necessary?
      if(fromSys == targetSys)
         return toReturn;

      // first correct for leap seconds
      const CivilTime civt(ttag);
      double dt = TimeSystem::Correction(fromSys, targetSys,
                              civt.year, civt.month, civt.day);
      toReturn += dt;
      toReturn.setTimeSystem(targetSys);
      // the corrected timetag: now only the system, not the value, matters
      toReturn.setTimeSystem(targetSys);

      // look up the TimeSystemCorr in list, and do the conversion
      map<string, TimeSystemCorrection>::const_iterator it;
      for(it = mapTimeCorr.begin(); it != mapTimeCorr.end(); ++it) {
         if(it->second.isConverterFor(fromSys, targetSys)) {
            dt = it->second.Correction(ttag);
            toReturn += dt;
            return toReturn;
         }
      }

      // failure
      InvalidRequest e("Unable to convert time systems from "
         + ttag.getTimeSystem().asString() + " to " + targetSys.asString());
      GPSTK_THROW(e);

      return toReturn;      // never reached, satisfy some compilers
   }

   // Returns the position, velocity, and clock offset of the indicated
   // object in ECEF coordinates (meters) at the indicated time.
   // @param[in] sat the satellite of interest
   // @param[in] ttag the time to look up
   // @return the Xvt of the object at the indicated time
   // @throw InvalidRequest If the request can not be completed for any
   //    reason, this is thrown. The text may have additional
   //    information as to why the request failed.
   Xvt Rinex3EphemerisStore::getXvt(const SatID& sat, const CommonTime& inttag) const
   {
      try {
         Xvt xvt;
         CommonTime ttag;
         TimeSystem ts;

         switch(sat.system) {
            case SatID::systemGPS:
            case SatID::systemGalileo:
            case SatID::systemBeiDou:
            case SatID::systemQZSS:
               if(sat.system == SatID::systemGPS    ) ts = TimeSystem::GPS;
               if(sat.system == SatID::systemGalileo) ts = TimeSystem::GAL;
               if(sat.system == SatID::systemBeiDou ) ts = TimeSystem::BDT;
               if(sat.system == SatID::systemQZSS   ) ts = TimeSystem::QZS;
               ttag = correctTimeSystem(inttag, ts);
               xvt = ORBstore.getXvt(sat,ttag);
               break;
            case SatID::systemGlonass:
               ttag = correctTimeSystem(inttag, TimeSystem::GLO);
               xvt = GLOstore.getXvt(sat,ttag);
               break;
            //case SatID::systemGeosync:
            //   ttag = correctTimeSystem(inttag, TimeSystem::GEO);
            //   xvt = GEOstore.getXvt(sat,ttag);
            //   break;
            default:
               InvalidRequest e("Unsupported satellite system");
               GPSTK_THROW(e);
               break;
         }

         return xvt;
      }
      catch(InvalidRequest& ir) { GPSTK_RETHROW(ir); }
   }

   // Dump information about the store to an ostream.
   // @param[in] os ostream to receive the output; defaults to cout
   // @param[in] detail integer level of detail to provide; allowed values are
   //    0: number of satellites, time step and time limits (default)
   //    1: above plus flags, gap and interval values, and number of data/sat
   //    2: above plus all the data tables
   void Rinex3EphemerisStore::dump(ostream& os, short detail) const
   {
      os << "Dump of Rinex3EphemerisStore:\n";
      // dump the time system corrections
      map<string,TimeSystemCorrection>::const_iterator tcit;
      for(tcit=mapTimeCorr.begin(); tcit != mapTimeCorr.end(); ++tcit) {
         tcit->second.dump(os);
         os << "\n";
      }

      NavFiles.dump(os, detail);

      if(ORBstore.size()) {
         os << "Dump of GPS/GAL/BDS/QZS ephemeris store:\n";
         ORBstore.dump(os, detail);
      }

      if(GLOstore.size()) {
         os << "Dump of GLO ephemeris store:\n";
         GLOstore.dump(os, detail);
      }

      //if(GEOstore.size()) {
         //os << "Dump of GEO ephemeris store:\n";
         //GEOstore.dump(os, detail);
      //}

      os << "End dump of Rinex3EphemerisStore\n";
   }

   // Determine the earliest time for which this object can successfully 
   // determine the Xvt for any object.
   // @return the earliest time in the table
   // @throw InvalidRequest if the object has no data.
   CommonTime Rinex3EphemerisStore::getInitialTime(void) const
   {
      try {
         CommonTime retTime(CommonTime::END_OF_TIME),time;

         // CommonTime does not allow comparisions unless TimeSystems agree,
         // or if one is "Any"
         retTime.setTimeSystem(TimeSystem::Any);
         
         time = ORBstore.getInitialTime();
         if(time < retTime) {
            retTime = time;
            retTime.setTimeSystem(TimeSystem::Any);
         }
         time = GLOstore.getInitialTime();
         if(time < retTime) {
            retTime = time;
            retTime.setTimeSystem(TimeSystem::Any);
         }
         //time = GEOstore.getInitialTime();
         //if(time < retTime) {
         //   retTime = time;
         //   retTime.setTimeSystem(TimeSystem::Any);
         //}

         return retTime;
      }
      catch(InvalidRequest& ir) { GPSTK_RETHROW(ir); }
   }

   // Determine the latest time for which this object can successfully 
   // determine the Xvt for any object.
   // @return the latest time in the table
   // @throw InvalidRequest if the object has no data.
   CommonTime Rinex3EphemerisStore::getFinalTime(void) const
   {
      try {
         CommonTime retTime(CommonTime::BEGINNING_OF_TIME),time;

         // CommonTime does not allow comparisions unless TimeSystems agree,
         // or if one is "Any"
         retTime.setTimeSystem(TimeSystem::Any);
         
         time = ORBstore.getInitialTime();
         if(time > retTime) {
            retTime = time;
            retTime.setTimeSystem(TimeSystem::Any);
         }
         time = GLOstore.getInitialTime();
         if(time > retTime) {
            retTime = time;
            retTime.setTimeSystem(TimeSystem::Any);
         }
         //time = GEOstore.getInitialTime();
         //if(time > retTime) {
         //   retTime = time;
         //   retTime.setTimeSystem(TimeSystem::Any);
         //}

         return retTime;
      }
      catch(InvalidRequest& ir) { GPSTK_RETHROW(ir); }
   }

   // Determine the earliest time for which this object can successfully 
   // determine the Xvt for any object.
   // @param SatID sat satellite, or system if sat.id==-1
   // @return the earliest time in the table
   // @throw InvalidRequest if the object has no data.
   CommonTime Rinex3EphemerisStore::getInitialTime(const SatID& sat) const
   {
      try {
         if(sat.system == SatID::systemMixed)
            return getInitialTime();

         CommonTime retTime(CommonTime::END_OF_TIME),time;
         retTime.setTimeSystem(TimeSystem::Any);

         switch(sat.system) {
            case SatID::systemGPS:
            case SatID::systemGalileo:
            case SatID::systemBeiDou:
            case SatID::systemQZSS:
               retTime = ORBstore.getInitialTime(sat);
            case SatID::systemGlonass:
               retTime = GLOstore.getInitialTime(sat);
            //case SatID::systemGeosync:
               //retTime = GEOstore.getInitialTime(sat);
            default:
               break;
         }

         return retTime;
      }
      catch(InvalidRequest& ir) { GPSTK_RETHROW(ir); }
   }

   // Determine the latest time for which this object can successfully 
   // determine the Xvt for any object.
   // @param SatID sat satellite, or system if sat.id==-1
   // @return the latest time in the table
   // @throw InvalidRequest if the object has no data.
   CommonTime Rinex3EphemerisStore::getFinalTime(const SatID& sat) const
   {
      try {
         if(sat.system == SatID::systemMixed)
            return getFinalTime();

         CommonTime retTime(CommonTime::BEGINNING_OF_TIME);
         retTime.setTimeSystem(TimeSystem::Any);

         switch(sat.system) {
            case SatID::systemGPS:
            case SatID::systemGalileo:
            case SatID::systemBeiDou:
            case SatID::systemQZSS:
               retTime = ORBstore.getFinalTime(sat);
            case SatID::systemGlonass:
               retTime = GLOstore.getFinalTime(sat);
            //case SatID::systemGeosync:
               //retTime = GEOstore.getFinalTime(sat);
            default:
               break;
         }


         return retTime;
      }
      catch(InvalidRequest& ir) { GPSTK_RETHROW(ir); }
   }

   // use to access the data records in the store in bulk
   int Rinex3EphemerisStore::addToList(list<Rinex3NavData>& theList, SatID sysSat)
      const
   {
      int n(0);

      // pure fussiness
      const bool keepAll(sysSat.system == SatID::systemMixed);
      const bool keepGPS(keepAll || sysSat.system==SatID::systemGPS);
      const bool keepGAL(keepAll || sysSat.system==SatID::systemGalileo);
      const bool keepGLO(keepAll || sysSat.system==SatID::systemGlonass);
      const bool keepBDS(keepAll || sysSat.system==SatID::systemBeiDou);
      const bool keepQZS(keepAll || sysSat.system==SatID::systemQZSS);
      const bool keepGEO(keepAll || sysSat.system==SatID::systemGeosync);
      const bool keepOrb(keepAll || keepGPS || keepGAL || keepBDS || keepQZS);
#pragma unused(keepGEO)
       
      if(keepOrb) {
         list<OrbitEph*> OElist;
         ORBstore.addToList(OElist);

         list<OrbitEph*>::const_iterator it;
         for(it=OElist.begin(); it != OElist.end(); ++it) {
            OrbitEph *ptr = *it;
            if((ptr->satID).system == SatID::systemGPS && keepGPS) {
               GPSEphemeris *sysptr = dynamic_cast<GPSEphemeris*>(ptr);
               theList.push_back(Rinex3NavData(*sysptr));
               n++;
            }
            else if((ptr->satID).system == SatID::systemGalileo && keepGAL) {
               GalEphemeris *sysptr = dynamic_cast<GalEphemeris*>(ptr);
               theList.push_back(Rinex3NavData(*sysptr));
               n++;
            }
            else if((ptr->satID).system == SatID::systemBeiDou && keepBDS) {
               BDSEphemeris *sysptr = dynamic_cast<BDSEphemeris*>(ptr);
               theList.push_back(Rinex3NavData(*sysptr));
               n++;
            }
            else if((ptr->satID).system == SatID::systemQZSS && keepQZS) {
               QZSEphemeris *sysptr = dynamic_cast<QZSEphemeris*>(ptr);
               theList.push_back(Rinex3NavData(*sysptr));
               n++;
            }
         }
      }
      if(keepGLO) {
         list<GloEphemeris> GLOlist;
         n += GLOstore.addToList(GLOlist);

         list<GloEphemeris>::const_iterator it;
         for(it=GLOlist.begin(); it != GLOlist.end(); ++it)
            theList.push_back(Rinex3NavData(*it));
      }
      /*
      if(keepGEO) {
         list<GeoRecord> GEOlist;
         n += GEOstore.addToList(GEOlist);

         list<GeoRecord>::const_iterator it;
         for(it=GEOlist.begin(); it != GEOlist.end(); ++it)
            theList.push_back(Rinex3NavData(*it));
      }
      */
      return n;
   }

}  // namespace gpstk
