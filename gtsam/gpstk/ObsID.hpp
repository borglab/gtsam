/// @file ObsID.hpp
/// gpstk::ObsID - navigation system, receiver, and file specification
/// independent representation of the types of observation data that can
/// be collected.  This class is analogous to the RinexObsType class that
/// is used to represent the observation codes in a RINEX file. It is
/// intended to support at least everything in section 5.1 of the RINEX 3
/// specifications.

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

#ifndef OBSID_HPP
#define OBSID_HPP

#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <map>

#include "Exception.hpp"
#include "SatID.hpp"
#include "ObsIDInitializer.hpp"

namespace gpstk
{
   class ObsID;
   namespace StringUtils
   {
      /// convert this object to a string representation
      std::string asString(const ObsID& id);
   }


   /// stream output for ObsID
   std::ostream& operator<<(std::ostream& s, const ObsID& p);


   class ObsID
   {
   public:
      /// The type of observation.
      enum ObservationType
      {
         otUnknown,
         otAny,       ///< Used to match any observation type
         otRange,     ///< pseudorange, in meters
         otPhase,     ///< accumulated phase, in meters
         otDoppler,   ///< Doppler, in Hz
         otSNR,       ///< Signal strength, in dB-Hz
         otChannel,   ///< Channel number
         otIono,      ///< Ionospheric delay (see RINEX3 section 5.12)
         otSSI,       ///< Signal Strength Indicator (RINEX)
         otLLI,       ///< Loss of Lock Indicator (RINEX)
         otTrackLen,  ///< Number of continuous epochs of 'good' tracking
         otNavMsg,    ///< Navigation Message data
         otUndefined, ///< Undefined
         otLast       ///< Used to verify that all items are described at compile time
      };


      /// The frequency band this obs was collected from.
      enum CarrierBand
      {
         cbUnknown,
         cbAny,  ///< Used to match any carrier band
         cbZero, ///< Used with the channel observation type (see RINEx3 section 5.13)
         cbL1,   ///< GPS L1, Galileo E2-L1-E1, SBAS L1, QZSS L1
         cbL2,   ///< GPS L2, QZSS L2
         cbL5,   ///< GPS L5, Galileo E5a, SBAS L5, QZSS L5
         cbG1,   ///< Glonass G1
         cbG2,   ///< Glonass G2
         cbG3,   ///< Glonass G3
         cbE5b,  ///< Galileo E5b, BeiDou L7
         cbE5ab, ///< Galileo E5a+b
         cbE6,   ///< Galileo E6, QZSS LEX
         cbB1,   ///< BeiDou L1
         cbB2,   ///< BeiDou L7
         cbB3,   ///< BeiDou L6
         cbL1L2, ///< Combined L1L2 (like an ionosphere free obs)
         cbUndefined,
         cbLast  ///< Used to verify that all items are described at compile time
      };


      /// The code used to collect the observation. Each of these should uniquely
      /// identify a code that was correlated against to track the signal. While the
      /// notation generally follows section 5.1 of RINEX 3, due to ambiguities in
      /// that specification some extensions are made. Note that as concrete
      /// specifications for the codes are released, this list may need to be
      /// adjusted. Specifically, this lists assumes that the same I & Q codes will be
      /// used on all three of the Galileo carriers. If that is not true, more
      /// identifiers need to be allocated
      enum TrackingCode
      {
         tcUnknown,
         tcAny,     ///< Used to match any tracking code
         tcCA,      ///< Legacy GPS civil code
         tcP,       ///< Legacy GPS precise code
         tcY,       ///< Encrypted legacy GPS precise code
         tcW,       ///< Encrypted legacy GPS precise code, codeless Z tracking
         tcN,       ///< Encrypted legacy GPS precise code, squaring codeless tracking
         tcD,       ///< Encrypted legacy GPS precise code, other codeless tracking
         tcM,       ///< Modernized GPS military unique code
         tcC2M,     ///< Modernized GPS L2 civil M code
         tcC2L,     ///< Modernized GPS L2 civil L code
         tcC2LM,    ///< Modernized GPS L2 civil M+L combined tracking (such as Trimble NetRS, Septrentrio, and ITT)
         tcI5,      ///< Modernized GPS L5 civil in-phase
         tcQ5,      ///< Modernized GPS L5 civil quadrature
         tcIQ5,     ///< Modernized GPS L5 civil I+Q combined tracking

         tcGCA,     ///< Legacy Glonass civil signal
         tcGP,      ///< Legacy Glonass precise signal
         tcIR3,     ///< Glonass L3 I code
         tcQR3,     ///< Glonass L3 Q code
         tcIQR3,    ///< Glonass L3 I+Q combined tracking

         tcA,       ///< Galileo L1 PRS code
         tcB,       ///< Galileo OS/CS/SoL code
         tcC,       ///< Galileo Dataless code
         tcBC,      ///< Galileo B+C combined tracking
         tcABC,     ///< Galileo A+B+C combined tracking
         tcIE5,     ///< Galileo L5 I code
         tcQE5,     ///< Galileo L5 Q code
         tcIQE5,    ///< Galileo L5 I+Q combined tracking

         tcSCA,     ///< SBAS civil code
         tcSI5,     ///< SBAS L5 I code
         tcSQ5,     ///< SBAS L5 Q code
         tcSIQ5,    ///< SBAS L5 I+Q code

         tcJCA,     ///< QZSS civil code
         tcJD1,     ///< QZSS L1C(D)
         tcJP1,     ///< QZSS L1C(P)
         tcJX1,     ///< QZSS L1C(D+P)
         tcJZ1,     ///< QZSS L1-SAIF
         tcJM2,     ///< QZSS L2C(M)
         tcJL2,     ///< QZSS L2C(L)
         tcJX2,     ///< QZSS L2C(M+L)
         tcJI5,     ///< QZSS L5 in-phase
         tcJQ5,     ///< QZSS L5 quadrature
         tcJIQ5,    ///< QZSS L5 I+Q combined tracking
         tcJI6,     ///< QZSS LEX(6) in-phase
         tcJQ6,     ///< QZSS LEX(6) quadrature
         tcJIQ6,    ///< QZSS LEX(6) I+Q combined tracking

         tcCI1,     ///< BeiDou B1 I code
         tcCQ1,     ///< BeiDou B1 Q code
         tcCIQ1,    ///< BeiDou B1 I code
         tcCI7,     ///< BeiDou B2 I+Q code
         tcCQ7,     ///< BeiDou B2 Q code
         tcCIQ7,    ///< BeiDou B2 I+Q code
         tcCI6,     ///< BeiDou B3 I code
         tcCQ6,     ///< BeiDou B3 Q code
         tcCIQ6,    ///< BeiDou B3 I+Q code

         tcUndefined,
         tcLast     ///< Used to verify that all items are described at compile time
      };

      /// empty constructor, creates a wildcard object.
      ObsID()
         : type(otUnknown), band(cbUnknown), code(tcUnknown) {};

      /// Explicit constructor
      ObsID(ObservationType ot, CarrierBand cb, TrackingCode tc)
         : type(ot), band(cb), code(tc) {};

      /// This string contains the system characters for all RINEX systems.
      static std::string validRinexSystems;

      /// These maps transform between 1-char and 3-char system descriptors,
      /// e.g. map1to3sys["G"] = "GPS" and map3to1sys["GPS"] = "G"
      static std::map<std::string, std::string> map1to3sys;
      static std::map<std::string, std::string> map3to1sys;

      /// This string contains the (1-digit) frequency code for all RINEX systems.
      static std::string validRinexFrequencies;

      /// This map[sys][freq] = valid codes gives valid tracking codes for RINEX
      /// observations given the system and frequency;
      /// eg. validRinexTrackingCodes['G']['1']="CSLXPWYMN* "
      /// The only exception is there is no pseudorange (C) on GPS L1/L2 N (codeless)
      /// NB These tracking code characters are ORDERED, basically 'best' to 'worst'
      static std::map<char, std::map<char, std::string> > validRinexTrackingCodes;

      /// Constructor from a string (Rinex 3 style descriptor). If this string is 3 
      /// characters long, the system is assumed to be GPS. If this string is 4
      /// characters long, the first character is the system designator as
      /// described in the Rinex 3 specification. If the Rinex 3 style descriptor
      /// isn't currently defined, a new one is silently automatically created
      /// with a blank description for the new characters.
      explicit ObsID(const std::string& id) throw(InvalidParameter);

      /// Constructor from c-style string; see c'tor from a string.
      explicit ObsID(const char* id) throw(InvalidParameter)
         { *this=ObsID(std::string(id));}

      /// Equality requires all fields to be the same
      virtual bool operator==(const ObsID& right) const;

      /// This ordering is somewhat arbitrary but is required to be able
      /// to use an ObsID as an index to a std::map. If an application needs
      /// some other ordering, inherit and override this function. One 'feature'
      /// that has been added is that an Any code/carrier/type will match
      /// any other code/carrier/type in the equality operator. The intent is to
      /// support performing an operation like "tell me if this is a pseudorange 
      /// that was collected on L1 from *any* code".
      virtual bool operator<(const ObsID& right) const;

      bool operator!=(const ObsID& right) const
      { return !(operator==(right)); };

      bool operator>(const ObsID& right) const
      {  return (!operator<(right) && !operator==(right)); };

      bool operator<=(const ObsID& right) const
      { return (operator<(right) || operator==(right)); };

      bool operator>=(const ObsID& right) const
      { return !(operator<(right)); };

      /// Convenience output method
      virtual std::ostream& dump(std::ostream& s) const;

      /// Destructor
      virtual ~ObsID() {};

      // Extend the standard identifiers with a new Rinex 3 style identifier. If
      // the specified id is already defined, an exception is thrown and the
      // existing definitions are not touched. If not then each character of the
      // specification is examined and the new ones are created. The returned
      // ObsID can then be examined for the assigned values.
      static ObsID newID(const std::string& id,
                         const std::string& desc="") throw(InvalidParameter);

      // Note that these are the only data members of objects of this class.
      ObservationType  type;
      CarrierBand      band;
      TrackingCode     code;

      /// These strings are for forming a somewhat verbose description
      static std::map< TrackingCode,    std::string > tcDesc;
      static std::map< CarrierBand,     std::string > cbDesc;
      static std::map< ObservationType, std::string > otDesc;

      /// These strings are used to translate this object to and from a string id
      static std::map< char, ObservationType> char2ot;
      static std::map< char, CarrierBand> char2cb;
      static std::map< char, TrackingCode> char2tc;
      static std::map< ObservationType, char > ot2char;
      static std::map< CarrierBand, char > cb2char;
      static std::map< TrackingCode, char> tc2char;

   private:
      static ObsID idCreator(const std::string& id, const std::string& desc="");

   }; // class ObsID


} // namespace gpstk
#endif   // OBSID_HPP
