/// @file Rinex3NavData.hpp
/// Encapsulates RINEX ver 3.02 Navigation data

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

#ifndef GPSTK_RINEXNAVDATA_HPP
#define GPSTK_RINEXNAVDATA_HPP

#include <list>
#include <string>

#include "StringUtils.hpp"
#include "CommonTime.hpp"
#include "FFStream.hpp"
#include "Rinex3NavBase.hpp"
#include "Rinex3NavStream.hpp"
#include "EngEphemeris.hpp"         // GPS only, deprecated
#include "GloEphemeris.hpp"
#include "GPSEphemeris.hpp"
#include "GalEphemeris.hpp"
#include "BDSEphemeris.hpp"
#include "QZSEphemeris.hpp"
#include "RinexSatID.hpp"
#include "RinexNavData.hpp"

namespace gpstk
{
   /** @addtogroup Rinex3Nav */
   //@{

      /**
       * This class models a RINEX 3 Nav record.
       *
       * \sa FunctionalTests/Rinex3NavText for example.
       * \sa Rinex3NavHeader and Rinex3NavStream classes.
       */

   class Rinex3NavData : public Rinex3NavBase
   {
   public:
         /** Constructor
          * @warning CHECK THE PRNID TO SEE IF THIS DATA IS VALID BEFORE
          *          USING!!!.
          */
      Rinex3NavData(void)
        : time(CommonTime::BEGINNING_OF_TIME), PRNID(-1), fitint(4)
      {}

         /// Initializes the nav data with a GloEphemeris
      Rinex3NavData(const GloEphemeris& gloe);

         /// Create from a RinexNavData (for backward compatibility)
      Rinex3NavData(const RinexNavData& rnd);

         /// Initializes the nav data with an EngEphemeris
         /// EngEphemeris is deprecated; use GPSEphemeris
      Rinex3NavData(const EngEphemeris& ee);

         /// Initializes the nav data with a GPSEphemeris
      Rinex3NavData(const GPSEphemeris& gpseph);

         /// Initializes the nav data with a GalEphemeris
      Rinex3NavData(const GalEphemeris& galeph);

         /// Initializes the nav data with a BDSEphemeris
      Rinex3NavData(const BDSEphemeris& bdseph);

         /// Initializes the nav data with a QZSEphemeris
      Rinex3NavData(const QZSEphemeris& qzseph);

         /// Destructor
      virtual ~Rinex3NavData() {}

         /// Rinex3NavData is "data" so this function always returns true.
      virtual bool isData(void) const {return true;}

         /// Write selected info (system dependent) as a single line
      std::string dumpString(void) const;

         /// A debug output function.
         /// Prints the PRN id and the IODC for this record.
      virtual void dump(std::ostream& s) const;

         /// deprecated; use GPSEphemeris, GPS-only.
         /// Converts Rinex3NavData to an EngEphemeris object.
      operator EngEphemeris() const throw();

         /// Converts Rinex3NavData to a GPSEphemeris object.
      operator GPSEphemeris() const throw();

         /// Converts this Rinex3NavData to a GloEphemeris object.
      operator GloEphemeris() const throw();

         /// Converts Rinex3NavData to a GalEphemeris object.
      operator GalEphemeris() const throw();

         /// Converts Rinex3NavData to a BDSEphemeris object.
      operator BDSEphemeris() const throw();

         /// Converts Rinex3NavData to a QZSEphemeris object.
      operator QZSEphemeris() const throw();

         /// Converts the (non-CommonTime) data to an easy list
         /// for comparison operators.
      std::list<double> toList() const;

         /// Sort on time, then satellite; for use with Rinex3EphemerisStore
      bool operator<(const Rinex3NavData& right) const
      {
         CommonTime t(time),r(right.time);
         t.setTimeSystem(TimeSystem::Any);
         r.setTimeSystem(TimeSystem::Any);
         if(t == r) return (sat < right.sat);
         return (t < r);
      }


      /** @name EpochDataGeneral */
      //@{
      CommonTime time;     ///< Time according to the sat/epoch record (TOC)
      std::string satSys;  ///< Satellite system of Epoch: G,R,E,S,C
      short PRNID;         ///< SV PRN ID
      RinexSatID sat;      ///< RinexSatID (from PRNID & satSys)
      long HOWtime;        ///< Time of subframe 1-3 (sec of week)
      short weeknum;       ///< GPS full week corresponding to HOWtime of SF1
                           ///< (N.B.:in RINEX files, week number corresponds
                           /// >to ToE, not GLO)
      double accuracy;     ///< SV accuracy (m)
      short health;        ///< SV health
      //@}

      /** @name EpochDataGPS */
      //@{
      short   codeflgs;    ///< L2 codes
      short   L2Pdata;     ///< L2 P data flag 
      double  IODC;        ///< Index of data-clock
      double  IODE;        ///< Index of data-eph
      //@}

      /** @name EpochDataGLO */
      //@{
      double  TauN;        ///< SV clock bias (sec)
      double  GammaN;      ///< SV relative frequency bias
      double  MFTraw;      ///< Message frame time (sec of UTC week) <double>
      long    MFtime;      ///< Message frame time (sec of UTC week) <long>
      short   freqNum;     ///< Frequency number (-7..+12)
      double  ageOfInfo;   ///< Age of oper. information (days)
      //@}

      /** @name EpochDataGAL */
      //@{
      short   datasources; ///< Data sources
      double  IODnav;      ///< Index of data-eph
      //@}

      /** @name EpochDataGEO */
      //@{
      double  accCode;     ///< Accuracy code (URA, meters)
      double  IODN;        ///< Issue of data navigation, DO229,
                           ///< 8 first bits after Message type if MT9
      //@}

      /** @name ClockInformation */
      //@{
      double  Toc;         ///< Time of clock (sec of week)
      double  af0;         ///< SV clock error (sec)
      double  af1;         ///< SV clock drift (sec/sec)
      double  af2;         ///< SV clock drift rate (sec/sec**2)
      double  Tgd;         ///< Group delay diff. (sec) (GPS, BDS:B1/B3 GAL:E5a/E1)
      double  Tgd2;        ///< Group delay differential (sec) (BDS:B2/B3 GAL:E5b/E1)
      //@}

      /** @name HarmonicPerturbations */
      //@{
      double  Cuc;         ///< Cosine latitude (rad)
      double  Cus;         ///< Sine latitude (rad)
      double  Crc;         ///< Cosine radius (m)
      double  Crs;         ///< Sine radius (m)
      double  Cic;         ///< Cosine inclination (rad)
      double  Cis;         ///< Sine inclination (rad)
      //@}

      /** @name MajorEphemerisParameters */
      //@{
      double  Toe;         ///< Ephemeris epoch (sec of week)
      double  M0;          ///< Mean anomaly (rad)
      double  dn;          ///< Correction to mean motion (rad/sec)
      double  ecc;         ///< Eccentricity
      double  Ahalf;       ///< SQRT of semi-major axis (m**1/2)
      double  OMEGA0;      ///< Rt ascension of ascending node (rad)
      double  i0;          ///< Inclination (rad)
      double  w;           ///< Argument of perigee (rad)
      double  OMEGAdot;    ///< Rate of Rt ascension (rad/sec)
      double  idot;        ///< Rate of inclination angle (rad/sec)
      double  fitint;      ///< Fit interval
      //@}

      /** @name TabularEphemerisParameters */
      //@{
      double  px, py, pz;  ///< SV position
      double  vx, vy, vz;  ///< SV velocity
      double  ax, ay, az;  ///< SV acceleration
      //@}


   private:

         /** Parses string \a currentLine to obtain PRN id and epoch.
          *  @param strm RINEX Nav stream
          */
      void getPRNEpoch(Rinex3NavStream& strm)
         throw(StringUtils::StringException, FFStreamError);


         /** @name OrbitParameters
          * Obtain orbit parameters from strint \a currentLine.
          */
         //@{
         /**  Read and parse the nth record after the epoch record
          *   @param int n record number (1-7), for nth record after the epoch line
          *   @param Rinex3NavStream strm stream to read from
          */
      void getRecord(const int& n, Rinex3NavStream& strm)
         throw(StringUtils::StringException, FFStreamError);
         //@}

         /** Generates the PRN/epoch line and outputs it to strm
          *  @param strm RINEX Nav stream
          */
      void putPRNEpoch(Rinex3NavStream& strm) const
         throw(StringUtils::StringException);


         /// @name OrbitParameters
         /// Generate orbit parameter lines from data to be output to a file
         //@{
         /** Construct and write the nth record after the epoch record
          *  @param int n                 Record number (1-7), for nth record
          *                               after the epoch line.
          *  @param Rinex3NavStream strm  Stream to read from.
          */
      void putRecord(const int& n, Rinex3NavStream& strm) const
         throw(StringUtils::StringException, FFStreamError);
         //@}

         /// Helper routine for constructors of this from OrbitEph-based Ephemerides
      void loadFrom(const OrbitEph *oeptr);

         /// Helper routine for casts from this to OrbitEph-based Ephemerides
      void castTo(OrbitEph *oeptr) const;

   protected:

         /** This function retrieves a RINEX 3 NAV record from the given
          *  FFStream.
          *  If an error is encountered in reading from the stream, the stream
          *  is returned to its original position and its fail-bit is set.
          *  @throws StringException when a StringUtils function fails.
          *  @throws FFStreamError when exceptions(failbit) is set and a read
          *          or formatting error occurs. This also resets the stream
          *          to its pre-read position.
          */
      virtual void reallyGetRecord(FFStream& s)
         throw(std::exception, FFStreamError, StringUtils::StringException);


         /// Outputs the record to the FFStream \a s.
      virtual void reallyPutRecord(FFStream& s) const 
         throw(std::exception, FFStreamError, StringUtils::StringException);

   }; // End of class 'Rinex3NavData'

   //@}

}  // End of namespace gpstk

#endif   // GPSTK_RINEXNAVDATA_HPP
