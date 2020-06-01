#pragma ident "$Id$"

/**
 * @file TimeSystemCorr.hpp
 * Encapsulate time system corrections, defined by header of RINEX 3 navigation file,
 * including RINEX 2, and used to convert CommonTime between systems.
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

#ifndef GPSTK_TIMESYSTEMCORRECTION_INCLUDE
#define GPSTK_TIMESYSTEMCORRECTION_INCLUDE

#include "GNSSconstants.hpp"
#include "CommonTime.hpp"
#include "GPSWeekSecond.hpp"
#include "CivilTime.hpp"

namespace gpstk {

      /// Time System Corrections as defined in the RINEX version 3 Navigation header.
   class TimeSystemCorrection
   {
   public:
         /// Supported time system correction types, cf. RINEX version 3 spec.
      enum CorrType
      {
         Unknown=0,
         GPUT,    ///< GPS  to UTC using A0, A1
         GAUT,    ///< GAL  to UTC using A0, A1
         SBUT,    ///< SBAS to UTC using A0, A1, incl. provider and UTC ID
         GLUT,    ///< GLO  to UTC using A0 = -TauC , A1 = 0
         GPGA,    ///< GPS  to GAL using A0 = A0G   , A1 = A1G
         GLGP,    ///< GLO  to GPS using A0 = -TauGPS, A1 = 0
         QZGP,    ///< QZS  to GPS using A0, A1
         QZUT,    ///< QZS  to UTC using A0, A1
         BDUT,    ///< BDT  to UTC using A0, A1
         BDGP    ///< BDT  to GPS using A0, A1  // not in RINEX
      };

         //// Member data
      CorrType type;
      TimeSystem frTS,toTS;
      double A0, A1;
      long refWeek,refSOW;       ///< reference time for polynominal (week,sow)
      long refYr,refMon,refDay;  ///< reference time (yr,mon,day) for RINEX ver 2 GLO
      std::string geoProvider;   ///< string 'EGNOS' 'WAAS' or 'MSAS'
      int geoUTCid;              ///< UTC Identifier [0 unknown, 1=UTC(NIST),
                                 ///<  2=UTC(USNO), 3=UTC(SU), 4=UTC(BIPM),
                                 ///<  5=UTC(Europe), 6=UTC(CRL)]

         /// Empty constructor
      TimeSystemCorrection()
         : type(Unknown), frTS(TimeSystem::Unknown), toTS(TimeSystem::Unknown)
         { }

         /// Constructor from string
      TimeSystemCorrection(std::string str) { this->fromString(str); }

      void fromString(const std::string str)
      {
         std::string STR(gpstk::StringUtils::upperCase(str));
         if(STR == std::string("GPUT"))
            { type = GPUT; frTS = TimeSystem::GPS; toTS = TimeSystem::UTC; }

         else if(STR == std::string("GAUT"))
            { type = GAUT; frTS = TimeSystem::GAL; toTS = TimeSystem::UTC; }

         else if(STR == std::string("SBUT"))
            // TD ??
            { type = SBUT; frTS = TimeSystem::GPS; toTS = TimeSystem::UTC; }

         else if(STR == std::string("GLUT"))
            { type = GLUT; frTS = TimeSystem::GLO; toTS = TimeSystem::UTC; }
            
         else if(STR == std::string("GPGA"))
            { type = GPGA; frTS = TimeSystem::GPS; toTS = TimeSystem::GAL; }
            
         else if(STR == std::string("GLGP"))
            { type = GLGP; frTS = TimeSystem::GLO; toTS = TimeSystem::GPS; }
            
         else if(STR == std::string("QZGP"))
            { type = QZGP; frTS = TimeSystem::QZS; toTS = TimeSystem::GPS; }
            
         else if(STR == std::string("QZUT"))
            { type = QZUT; frTS = TimeSystem::QZS; toTS = TimeSystem::UTC; }
            
         else if(STR == std::string("BDUT"))
            { type = BDUT; frTS = TimeSystem::BDT; toTS = TimeSystem::UTC; }
            
         else if(STR == std::string("BDGP"))
            { type = BDGP; frTS = TimeSystem::BDT; toTS = TimeSystem::GPS; }
            
         else {
            Exception e("Unknown TimeSystemCorrection type: " + str);
            GPSTK_THROW(e);
         }
      }

         /// Return readable string version of CorrType
      std::string asString() const
      {
         switch(type) {
            case GPUT: return std::string("GPS to UTC"); break;
            case GAUT: return std::string("GAL to UTC"); break;
            case SBUT: return std::string("SBAS to UTC");
               break;
            case GLUT: return std::string("GLO to UTC"); break;
            case GPGA: return std::string("GPS to GAL"); break;
            case GLGP: return std::string("GLO to GPS"); break;
            case QZGP: return std::string("QZS to GPS"); break;
            case QZUT: return std::string("QZS to UTC"); break;
            case BDUT: return std::string("BDT to UTC"); break;
            case BDGP: return std::string("BDT to GPS"); break;
            default:   return std::string("ERROR"); break;
         }
      }

         /// Return 4-char string version of CorrType
      std::string asString4() const
      {
         switch(type) {
            case GPUT: return std::string("GPUT"); break;
            case GAUT: return std::string("GAUT"); break;
            case SBUT: return std::string("SBUT"); break;
            case GLUT: return std::string("GLUT"); break;
            case GPGA: return std::string("GPGA"); break;
            case GLGP: return std::string("GLGP"); break;
            case QZGP: return std::string("QZGP"); break;
            case QZUT: return std::string("QZUT"); break;
            case BDUT: return std::string("BDUT"); break;
            case BDGP: return std::string("BDGP"); break;
            default:   return std::string("ERROR"); break;
         }
      }

         /// dump
      void dump(std::ostream& s) const
      {
         s << "Time system correction for " << asString4() << ": "
            << asString() << std::scientific << std::setprecision(12);
         switch(type) {
            case TimeSystemCorrection::GPUT:
               s << ", A0 = " << A0 << ", A1 = " << A1
                 << ", RefTime = week/sow " << refWeek << "/" << refSOW;
               break;
            case TimeSystemCorrection::GAUT:
               s << ", A0 = " << A0 << ", A1 = " << A1
                 << ", RefTime = week/sow " << refWeek << "/" << refSOW;
               break;
            case TimeSystemCorrection::SBUT:
               s << ", A0 = " << A0 << ", A1 = " << A1
                 << ", RefTime = week/sow " << refWeek << "/" << refSOW
                 << ", provider " << geoProvider << ", UTC ID = " << geoUTCid;
               break;
            case TimeSystemCorrection::GLUT:
               s << ", -TauC = " << A0
                 << ", RefTime = week/sow " << refWeek << "/" << refSOW;
               break;
            case TimeSystemCorrection::GPGA:
               s << ", A0G = " << A0 << ", A1G = " << A1
                 << ", RefTime = week/sow " << refWeek << "/" << refSOW;
               break;
            case TimeSystemCorrection::GLGP:
               s << ", TauGPS = " << A0 << " sec, RefTime = yr/mon/day "
                 << refYr << "/" << refMon << "/" << refDay;
               break;
            case TimeSystemCorrection::QZGP:
               s << ", A0 = " << A0 << ", A1 = " << A1
                 << ", RefTime = week/sow " << refWeek << "/" << refSOW;
               break;
            case TimeSystemCorrection::QZUT:
               s << ", A0 = " << A0 << ", A1 = " << A1
                 << ", RefTime = week/sow " << refWeek << "/" << refSOW;
               break;
            case TimeSystemCorrection::BDUT:
               s << ", A0 = " << A0 << ", A1 = " << A1
                 << ", RefTime = week/sow " << refWeek << "/" << refSOW;
               break;
            case TimeSystemCorrection::BDGP:
               s << ", A0 = " << A0 << ", A1 = " << A1
                 << ", RefTime = week/sow " << refWeek << "/" << refSOW;
               break;
            default:
               break;
         }
         //s << std::endl;
      }

         /// Equal operator
      inline bool operator==(const TimeSystemCorrection& tc)
      { return tc.type == type; }

         /// Less than operator - required for map.find()
      inline bool operator<(const TimeSystemCorrection& tc)
      { return tc.type < type; }

      /// Return true if this object provides the correction necessary to convert
      /// between the two given time systems. Throw if the time systems are the
      /// same or either is TimeSystem::Unknown.
      /// @param ts1 and ts2  TimeSystems of interest
      /// @return true if this object will convert ts1 <=> ts2
      /// @throw if either TimeSystem is Unknown, or if they are identical
      bool isConverterFor(const TimeSystem& ts1, const TimeSystem& ts2) const
      {
         if(ts1 == ts2) {
            Exception e("Identical time systems");
            GPSTK_THROW(e);
         }
         if(ts1 == TimeSystem::Unknown || ts2 == TimeSystem::Unknown) {
            Exception e("Unknown time systems");
            GPSTK_THROW(e);
         }
         if((ts1 == frTS && ts2 == toTS) || (ts2 == frTS && ts1 == toTS)) {
            return true;
         }
         return false;
      }

      /// Compute the conversion (in seconds) at the given time for this object
      /// (TimeSystemCorrection). The caller must ensure that the input time has the
      /// appropriate TimeSystem, it will determine the sign of the correction; it is
      /// such that it should ALWAYS be ADDED to the input time.
      /// For example, suppose this object is a "GPUT" (GPS=>UTC) correction. Then
      ///    ct(GPS) + Correction(ct) will yield ct(UTC), and
      ///    ct(UTC) + Correction(ct) will yield ct(GPS).
      ///    [That is, Correction(ct) in the two cases differ in sign]
      /// Throw an Exception if the TimeSystem of the input does not match either of
      /// the systems in this object.
      /// @param CommonTime ct, the time at which to compute the correction; the
      ///        TimeSystem of ct will determine the sign of the correction.
      /// @return the correction (sec) to be added to ct to change its TimeSystem
      /// @throw if the input TimeSystem matches neither system in this object.
      double Correction(const CommonTime& ct) const
      {
         double corr(0.0), dt;
         TimeSystem fromTS(ct.getTimeSystem());
         GPSWeekSecond gpsws;
         CommonTime refTime;
         Exception e("Unable to compute correction - wrong TimeSystem");
         Exception eSBAS("TimeSystemCorr SBAS <=> UTC has not been implemented");

         switch(type) {
            case GPUT:
               if(fromTS != TimeSystem::GPS && fromTS != TimeSystem::UTC)
                  { GPSTK_THROW(e); }

               // dt = fromTime - refTime
               gpsws = GPSWeekSecond(refWeek,refSOW);
               refTime = gpsws.convertToCommonTime();
               refTime.setTimeSystem(fromTS);
               dt = ct - refTime;

               if(fromTS == TimeSystem::GPS)             // GPS => UTC
                  corr = -A0-A1*dt;
               else                                      // UTC => GPS
                  corr = A0+A1*dt;

               break;

            case GAUT:
               if(fromTS != TimeSystem::GAL && fromTS != TimeSystem::UTC)
                  { GPSTK_THROW(e); }

               // dt = fromTime - refTime
               gpsws = GPSWeekSecond(refWeek,refSOW);
               refTime = gpsws.convertToCommonTime();
               refTime.setTimeSystem(fromTS);
               dt = ct - refTime;

               if(fromTS == TimeSystem::GAL)             // GAL => UTC
                  corr = A0+A1*dt;
               else                                      // UTC => GAL
                  corr = -A0-A1*dt;

               break;

            case SBUT:
               GPSTK_THROW(eSBAS);
               break;

            case GLUT:
               if(fromTS != TimeSystem::GLO && fromTS != TimeSystem::UTC)
                  { GPSTK_THROW(e); }

               if(fromTS == TimeSystem::GLO)             // GLO => UTC
                  corr = A0;
               else                                      // UTC => GLO
                  corr = -A0;

               break;

            case GPGA:
               if(fromTS != TimeSystem::GPS && fromTS != TimeSystem::GAL)
                  { GPSTK_THROW(e); }

               // dt = fromTime - refTime
               gpsws = GPSWeekSecond(refWeek,refSOW);
               refTime = gpsws.convertToCommonTime();
               refTime.setTimeSystem(fromTS);
               dt = ct - refTime;

               if(fromTS == TimeSystem::GPS)             // GPS => GAL
                  corr = A0+A1*dt;
               else                                      // GAL => GPS
                  corr = -A0-A1*dt;

               break;

            case GLGP:
               if(fromTS != TimeSystem::GLO && fromTS != TimeSystem::GPS)
                  { GPSTK_THROW(e); }

               if(fromTS == TimeSystem::GLO)             // GLO => GPS
                  corr = A0;
               else                                      // GPS => GLO
                  corr = -A0;

               break;

            case QZGP:
               if(fromTS != TimeSystem::QZS && fromTS != TimeSystem::GPS)
                  { GPSTK_THROW(e); }

               if(fromTS == TimeSystem::QZS)             // QZS => GPS
                  corr = 0.0;    // TD?
               else                                      // GPS => QZS
                  corr = 0.0;    // TD?

               break;

            case QZUT:
               if(fromTS != TimeSystem::QZS && fromTS != TimeSystem::UTC)
                  { GPSTK_THROW(e); }

               // dt = fromTime - refTime
               gpsws = GPSWeekSecond(refWeek,refSOW);
               refTime = gpsws.convertToCommonTime();
               refTime.setTimeSystem(fromTS);
               dt = ct - refTime;

               if(fromTS == TimeSystem::QZS)             // QZS => UTC
                  corr = A0+A1*dt;
               else                                      // UTC => QZS
                  corr = -A0-A1*dt;

               break;

            case BDUT:
               if(fromTS != TimeSystem::BDT && fromTS != TimeSystem::UTC)
                  { GPSTK_THROW(e); }

               // dt = fromTime - refTime
               gpsws = GPSWeekSecond(refWeek,refSOW);
               refTime = gpsws.convertToCommonTime();
               refTime.setTimeSystem(fromTS);
               dt = ct - refTime;

               if(fromTS == TimeSystem::BDT)             // BDT => UTC
                  corr = A0+A1*dt;
               else                                      // UTC => BDT
                  corr = -A0-A1*dt;

               break;

            case BDGP:
               if(fromTS != TimeSystem::BDT && fromTS != TimeSystem::GPS)
                  { GPSTK_THROW(e); }

               // dt = fromTime - refTime
               gpsws = GPSWeekSecond(refWeek,refSOW);
               refTime = gpsws.convertToCommonTime();
               refTime.setTimeSystem(fromTS);
               dt = ct - refTime;

               if(fromTS == TimeSystem::BDT)             // BDT => GPS
                  corr = A0;
               else                                      // GPS => BDT
                  corr = -A0;

               break;

            default:
               Exception e("TimeSystemCorrection is not defined.");
               GPSTK_THROW(e);
               break;
         }

         return corr;
      }

   }; // End of class 'TimeSystemCorrection'

};    // end namespace

#endif // GPSTK_TIMESYSTEMCORRECTION_INCLUDE
