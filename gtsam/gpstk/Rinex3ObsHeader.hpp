#pragma ident "$Id$"

/**
 * @file Rinex3ObsHeader.hpp
 * Encapsulate header of Rinex observation file, including I/O
 */

#ifndef GPSTK_RINEX3OBSHEADER_HPP
#define GPSTK_RINEX3OBSHEADER_HPP

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

#include <vector>
#include <list>
#include <map>
#include <iostream>
#include <iomanip>

#include "CivilTime.hpp"
#include "FFStream.hpp"
#include "Rinex3ObsBase.hpp"
#include "Triple.hpp"
#include "RinexSatID.hpp"
#include "RinexObsID.hpp"

namespace gpstk
{
  /** @addtogroup Rinex3Obs */
  //@{

  /**
   * This class models the header for a RINEX 3 Observation File.
   * @sa gpstk::Rinex3ObsData and gpstk::Rinex3ObsStream.
   * @sa rinex_obs_test.cpp and rinex_obs_read_write.cpp for examples.
   */

   class Rinex3ObsHeader : public Rinex3ObsBase
   {
   public:

      /// A Simple Constructor.
      Rinex3ObsHeader() : valid(0), validEoH(false)
         {}

      /// Clear (empty out) header
      inline void clear()
      {
         commentList.clear();
         obsTypeList.clear();
         numObsForSat.clear();
         mapObsTypes.clear();
         wavelengthFactor[0] = wavelengthFactor[1] = 1;
         extraWaveFactList.clear();
         valid  = 0;
         validEoH = false;
         numObs = 0;
         lastPRN.id = -1;
      }

      /// @name Rinex3ObsHeaderFormatStrings
      /// RINEX observation file header formatting strings
      //@{
      static const std::string stringVersion;           ///< "RINEX VERSION / TYPE"
      static const std::string stringRunBy;             ///< "PGM / RUN BY / DATE"
      static const std::string stringComment;           ///< "COMMENT"
      static const std::string stringMarkerName;        ///< "MARKER NAME"
      static const std::string stringMarkerNumber;      ///< "MARKER NUMBER"
      static const std::string stringMarkerType;        ///< "MARKER TYPE"
      static const std::string stringObserver;          ///< "OBSERVER / AGENCY"
      static const std::string stringReceiver;          ///< "REC # / TYPE / VERS"
      static const std::string stringAntennaType;       ///< "ANT # / TYPE"
      static const std::string stringAntennaPosition;   ///< "APPROX POSITION XYZ"
      static const std::string stringAntennaDeltaHEN;   ///< "ANTENNA: DELTA H/E/N"
      static const std::string stringAntennaDeltaXYZ;   ///< "ANTENNA: DELTA X/Y/Z"
      static const std::string stringAntennaPhaseCtr;   ///< "ANTENNA: PHASECENTER"
      static const std::string stringAntennaBsightXYZ;  ///< "ANTENNA: B.SIGHT XYZ"
      static const std::string stringAntennaZeroDirAzi; ///< "ANTENNA: ZERODIR AZI"
      static const std::string stringAntennaZeroDirXYZ; ///< "ANTENNA: ZERODIR XYZ"
      static const std::string stringCenterOfMass;      ///< "CENTER OF MASS: XYZ"
      static const std::string stringNumObs;            ///< "# / TYPES OF OBSERV"   R2 only
      static const std::string stringSystemNumObs;      ///< "SYS / # / OBS TYPES"
      static const std::string stringWaveFact;          ///< "WAVELENGTH FACT L1/2"  R2 only
      static const std::string stringSigStrengthUnit;   ///< "SIGNAL STRENGTH UNIT"
      static const std::string stringInterval;          ///< "INTERVAL"
      static const std::string stringFirstTime;         ///< "TIME OF FIRST OBS"
      static const std::string stringLastTime;          ///< "TIME OF LAST OBS"
      static const std::string stringReceiverOffset;    ///< "RCV CLOCK OFFS APPL"
      static const std::string stringSystemDCBSapplied; ///< "SYS / DCBS APPLIED"
      static const std::string stringSystemPCVSapplied; ///< "SYS / PCVS APPLIED"
      static const std::string stringSystemScaleFac;    ///< "SYS / SCALE FACTOR"
      static const std::string stringSystemPhaseShift;  ///< "SYS / PHASE SHIFT"
      static const std::string stringGlonassSlotFreqNo; ///< "GLONASS SLOT / FRQ #"
      static const std::string stringGlonassCodPhsBias; ///< "GLONASS COD/PHS/BIS"
      static const std::string stringLeapSeconds;       ///< "LEAP SECONDS"
      static const std::string stringNumSats;           ///< "# OF SATELLITES"
      static const std::string stringPrnObs;            ///< "PRN / # OF OBS"
      static const std::string stringEoH;               ///< "END OF HEADER"
      //@}

      /// Validity bits for the RINEX Observation Header - please keep ordered as strings above
      enum validBits
      {
         validVersion           =        0x1, ///< "RINEX VERSION / TYPE"
         validRunBy             =        0x2, ///< "PGM / RUN BY / DATE"
         validComment           =        0x4, ///< "COMMENT"               optional
         validMarkerName        =        0x8, ///< "MARKER NAME"
         validMarkerNumber      =       0x10, ///< "MARKER NUMBER"         optional
         validMarkerType        =       0x20, ///< "MARKER TYPE"                    R3
         validObserver          =       0x40, ///< "OBSERVER / AGENCY"
         validReceiver          =       0x80, ///< "REC # / TYPE / VERS"
         validAntennaType       =      0x100, ///< "ANT # / TYPE"
         validAntennaPosition   =      0x200, ///< "APPROX POSITION XYZ"   req except optional R3+moving
         validAntennaDeltaHEN   =      0x400, ///< "ANTENNA: DELTA H/E/N"
         validAntennaDeltaXYZ   =      0x800, ///< "ANTENNA: DELTA X/Y/Z"  optional R3
         validAntennaPhaseCtr   =     0x1000, ///< "ANTENNA: PHASECENTER"  optional R3
         validAntennaBsightXYZ  =     0x2000, ///< "ANTENNA: B.SIGHT XYZ"  optional R3
         validAntennaZeroDirAzi =     0x4000, ///< "ANTENNA: ZERODIR AZI"  optional R3
         validAntennaZeroDirXYZ =     0x8000, ///< "ANTENNA: ZERODIR XYZ"  optional R3
         validCenterOfMass      =    0x10000, ///< "CENTER OF MASS: XYZ"   optional R3
         validNumObs            =    0x20000, ///< "# / TYPES OF OBSERV"            R2 only
         validSystemObsType     =    0x20000, ///< "SYS / # / OBS TYPES"            R3
         validWaveFact          =    0x40000, ///< "WAVELENGTH FACT L1/2"  optional R2 only
         validSigStrengthUnit   =    0x40000, ///< "SIGNAL STRENGTH UNIT"  optional R3
         validInterval          =    0x80000, ///< "INTERVAL"              optional
         validFirstTime         =   0x100000, ///< "TIME OF FIRST OBS"
         validLastTime          =   0x200000, ///< "TIME OF LAST OBS"      optional
         validReceiverOffset    =   0x400000, ///< "RCV CLOCK OFFS APPL"   optional
         validSystemDCBSapplied =   0x800000, ///< "SYSTEM DCBS APPLIED"   optional R3
         validSystemPCVSapplied =  0x1000000, ///< "SYSTEM PCVS APPLIED"   optional R3
         validSystemScaleFac    =  0x2000000, ///< "SYSTEM SCALE FACTOR"   optional R3
         validSystemPhaseShift  =  0x4000000, ///< "SYS / PHASE SHIFT"              R3.01,3.02
         validGlonassFreqNo     =  0x8000000, ///< "GLONASS SLOT / FRQ #"           R3.01
         validGlonassCodPhsBias = 0x10000000, ///< "GLONASS COD/PHS/BIS"            R3.02
         validLeapSeconds       = 0x20000000, ///< "LEAP SECONDS"          optional
         validNumSats           = 0x40000000, ///< "# OF SATELLITES"       optional
         validPrnObs            = 0x80000000, ///< "PRN / # OF OBS"        optional
         //do away with this  validEoH               =0x100000000, ///< "END OF HEADER"
   
         /// This mask is for all required valid fields
         allValid2              = 0x001207CB, // RINEX 2

         //allValid30           = 0x001207EB, // RINEX 3.0 for static receivers - AntennaPosition present
         allValid30             = 0x001205EB, // RINEX 3.0 for moving receivers -- make default

         //allValid301            = 0x0C1205CB, // RINEX 3.01
         //allValid302            = 0x1C1205CB, // RINEX 3.02
         // NB 19Jun2013 MGEX data does not include GLONASS SLOT and GLONASS COD/PHS/BIS records
         allValid301            = 0x041205CB, // RINEX 3.01
         allValid302            = 0x041205CB // RINEX 3.02
      };
   
      /// RINEX 3 DCBS/PCVS info (for differential code bias and phase center variations corr.)
      struct Rinex3CorrInfo
      {
         std::string satSys,  ///< 1-char SV system (G/R/E/S)
                     name,    ///< program name used to apply corrections
                     source;  ///< source of corrections (URL)
      };

      /// RINEX 2 extra "WAVELENGTH FACT" lines
      struct ExtraWaveFact
      {
         /// List of Sats with this wavelength factor
         std::vector<SatID> satList;
         /// vector of wavelength factor values
         short wavelengthFactor[2];
      };

      /// Storage for R2 <-> R3 conversion of obstypes during reallyGet/Put
      /// Vector of strings containing ver 2 obs types (e.g. "C1" "L2") defined in reallyGet;
      /// also defined in PrepareVer2Write() from R3 ObsIDs
      std::vector<std::string> R2ObsTypes;
      /// map between RINEX ver 3 ObsIDs and ver 2 obstypes for each system: reallyPut
      std::map<std::string, std::map<std::string, RinexObsID> > mapSysR2toR3ObsID;

      /// Scale Factor corrections for observations
      typedef std::map<RinexObsID,int> sfacMap; ///< scale factor map <ObsType, ScaleFactor>
      std::map<std::string,sfacMap> sysSfacMap; ///< sat. system map of scale factor maps
                                                ///< <(G/R/E/S), <Rinex3ObsType, scalefactor>>

      /// @name Rinex3ObsHeaderValues
      //@{
      double version;                              ///< RINEX 3 version/type
      std::string fileType,                        ///< RINEX 3 file type
                  fileSys;                         ///< file sys char: RinexSatID system OR Mixed
      SatID fileSysSat;                            ///< fileSys as a SatID
      std::string fileProgram,                     ///< program used to generate file
                  fileAgency,                      ///< who ran program
                  date;                            ///< when program was run
      std::vector<std::string> commentList;        ///< comments in header             (optional)
      std::string markerName,                      ///< MARKER NAME
                  markerNumber,                    ///< MARKER NUMBER                  (optional)
                  markerType;                      ///< MARKER TYPE
      std::string observer,                        ///< who collected the data
                  agency;                          ///< observer's agency
      std::string recNo,                           ///< receiver number
                  recType,                         ///< receiver type
                  recVers;                         ///< receiver version
      std::string antNo,                           ///< antenna number
                  antType;                         ///< antenna type
      gpstk::Triple antennaPosition,               ///< APPROX POSITION XYZ  (optional if moving)
                    antennaDeltaHEN,               ///< ANTENNA: DELTA H/E/N
                    antennaDeltaXYZ;               ///< ANTENNA: DELTA X/Y/Z           (optional)
      std::string antennaSatSys,                   ///< ANTENNA P.CTR BLOCK: SAT SYS   (optional)
                  antennaObsCode;                  ///< ANTENNA P.CTR BLOCK: OBS CODE  (optional)
      gpstk::Triple antennaPhaseCtr;               ///< ANTENNA P.CTR BLOCK: PCTR POS  (optional)
      gpstk::Triple antennaBsightXYZ;              ///< ANTENNA B.SIGHT XYZ            (optional)
      double        antennaZeroDirAzi;             ///< ANTENNA ZERODIR AZI            (optional)
      gpstk::Triple antennaZeroDirXYZ;             ///< ANTENNA ZERODIR XYZ            (optional)
      short wavelengthFactor[2];                   ///< default WAVELENGTH FACT        R2 only
      std::vector<ExtraWaveFact> extraWaveFactList;///< extra (per sat) WAVELENGTH FACT R2 only
      gpstk::Triple centerOfMass;                  ///< vehicle CENTER OF MASS: XYZ    (optional)
      std::vector<RinexObsID> obsTypeList;         ///< number & types of observations R2 only
      std::map<std::string,std::vector<RinexObsID> > mapObsTypes; ///< map <sys char, vec<ObsID> >;
                                                        ///< NB defines data vec in ObsData
      std::string sigStrengthUnit;                 ///< SIGNAL STRENGTH UNIT           (optional)
      double interval;                             ///< INTERVAL                       (optional)
      CivilTime firstObs,                          ///< TIME OF FIRST OBS
                 lastObs;                          ///< TIME OF LAST OBS               (optional)
      int receiverOffset;                          ///< RCV CLOCK OFFS APPL            (optional)
      std::vector<Rinex3CorrInfo> infoDCBS;        ///< DCBS INFO                      (optional)
      std::vector<Rinex3CorrInfo> infoPCVS;        ///< PCVS INFO                      (optional)
      int factor, factorPrev;                      ///< scale factor (temp holders)
      RinexObsID sysPhaseShiftObsID;               ///< save ObsID for cont. "PHASE SHIFT" R3.01
      std::map<std::string, std::map<RinexObsID, std::map<RinexSatID,double> > > sysPhaseShift;
      std::map<RinexSatID,int> GlonassFreqNo;      ///< "GLONASS SLOT / FRQ #"    (optional) R3.01
      std::map<RinexObsID,double> GlonassCodePhaseBias; ///< "GLONASS COD/PHS/BIS"            R3.02
      int leapSeconds;                             ///< LEAP SECONDS              (optional)
      short numSVs;                                ///< # OF SATELLITES           (optional)
      std::map<RinexSatID,std::vector<int> > numObsForSat; ///< PRN / # OF OBS         (optional)
      unsigned long valid;                         ///< bits set when header rec.s present & valid
      bool validEoH;                               ///< true if found END OF HEADER
      std::string satSysTemp,                      ///< save the syschar while reading ScaleFactor
                  satSysPrev;                      ///< recall the prev sat. sys for cont. lines
      int numObs,                                  ///< save OBS # / TYPES and Sys / SCALE FACTOR
                                                   ///< for cont. lines
            numObsPrev;                            ///< recall the prev # obs for cont. lines
      RinexSatID lastPRN;                          ///< save PRN while reading PRN/OBS cont. lines
      //@}

      /// Destructor
      virtual ~Rinex3ObsHeader()
         {}

      // The next four lines comprise our common interface.

      /// Rinex3ObsHeader is a "header" so this function always returns true.
      virtual bool isHeader() const
      { return true; }

      /// This is a simple Debug output function.
      /// It simply outputs the version, name and antenna number of this
      /// RINEX header.
      virtual void dump(std::ostream& s) const;


         /** This method returns the numerical index of a given observation
          *
          * @param type String representing the observation type.
          */
      virtual int getObsIndex( std::string type ) const
         throw(InvalidRequest);


      /// Parse a single header record, and modify valid accordingly.
      /// Used by reallyGetRecord for both Rinex3ObsHeader and Rinex3ObsData.
      void ParseHeaderRecord(std::string& line)
         throw(FFStreamError);

      /// Compute number of valid header records that WriteHeaderRecords() will write
      int NumberHeaderRecordsToBeWritten(void) const throw();

      /// Write all valid header records to the given stream.
      /// Used by reallyPutRecord for both Rinex3ObsHeader and Rinex3ObsData.
      void WriteHeaderRecords(FFStream& s) const
         throw(FFStreamError, gpstk::StringUtils::StringException);

      /// Return boolean : is this a valid Rinex header?
      bool isValid() const
      {
         if(!validEoH) return false;
         unsigned long allValid;
         if(     version < 3.00) allValid = allValid2;
         else if(version < 3.01) allValid = allValid30;
         else if(version < 3.02) allValid = allValid301;  
         else                    allValid = allValid302;
         return ((valid & allValid) == allValid);
      }

      /// Compute map of obs types for use in writing version 2 header and data
      void PrepareVer2Write(void) throw();


   protected:


      /// outputs this record to the stream correctly formatted.
      virtual void reallyPutRecord(FFStream& s) const
         throw(std::exception, FFStreamError, gpstk::StringUtils::StringException);

      /// This function retrieves the RINEX Header from the given FFStream.
      /// If an stream error is encountered, the stream is reset to its
      ///  original position and its fail-bit is set.
      /// @throws StringException when a StringUtils function fails
      /// @throws FFStreamError when exceptions(failbit) is set and
      ///  a read or formatting error occurs.  This also resets the
      ///  stream to its pre-read position.
      virtual void reallyGetRecord(FFStream& s)
         throw(std::exception, FFStreamError, gpstk::StringUtils::StringException);

      friend class Rinex3ObsData;


   private:

      /// Converts the daytime \a dt into a Rinex Obs time
      /// string for the header
      std::string writeTime(const CivilTime& civtime) const;

      /// This function sets the time for this header.
      /// It looks at \a line to obtain the needed information.
      CivilTime parseTime(const std::string& line) const;


   }; // end class Rinex3ObsHeader

   //@}

} // namespace

#endif // GPSTK_RINEX3OBSHEADER_HPP
