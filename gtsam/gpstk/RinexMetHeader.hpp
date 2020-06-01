#pragma ident "$Id$"

/**
 * @file RinexMetHeader.hpp
 * Encapsulates header of RINEX 2 & 3 Met file data, including I/O.
 */

#ifndef GPSTK_RINEXMETHEADER_HPP
#define GPSTK_RINEXMETHEADER_HPP

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

#include "StringUtils.hpp"
#include "FFStream.hpp"
#include "RinexMetBase.hpp"
#include "Triple.hpp"

namespace gpstk
{
  /** @addtogroup RinexMet */
  //@{

      // forward declarations
   struct sensorType;

   struct sensorPosType;

  /**
   * This class models the RINEX MET header for a RINEX MET file.
   * When writing data, this class determines what data gets written from
   * a RinexMetData object to the RinexMetStream.
   *
   * @sa rinex_met_read_write.cpp for an example.
   * @sa rinex_met_test.cpp for an example.
   * @sa RinexMetHeader::obsTypeList for writing files.
   * @sa RinexMetData and RinexMetStream.
   */
  class RinexMetHeader : public RinexMetBase
  {

  public:

    /// Enum for the different types of data in this file.
    enum RinexMetType
    {
      PR, ///< Pressure (mbar)
      TD, ///< Dry temperature (deg C)
      HR, ///< Relative humidity (percent)
      ZW, ///< Wet zenith path delay (mm)
      ZD, ///< Dry component of zenith path delay (mm)
      ZT, ///< Total zenith path delay (mm)
      WD, ///< Wind azmiuth from where the wind blows (deg)
      WS, ///< Wind speed (m/s)
      RI, ///< "Rain increment": Rain accumulation since last measurement (1/10 mm)
      HI  ///< Hail indicator non-zero: Hail detected since last mesaurement
    };

    /// Constructor
    RinexMetHeader() : valid(0), version(2.1) {}

    /// Destructor
    virtual ~RinexMetHeader() {}

    // The next four lines is our common interface
    /// RinexMetHeader is a "header" so this function always returns true.
    virtual bool isHeader(void) const {return true;}

    /// A debug function that outputs the header to \a s.
    virtual void dump(std::ostream& s) const;

    /// sets the obs type array given an obs type line
    static RinexMetType convertObsType(const std::string& oneObs)
      throw(FFStreamError);

    /// Converts a RinexMetType to its string equivalent.
    static std::string convertObsType(const RinexMetType& oneObs)
      throw(FFStreamError);

    /// Tell me, Am I valid?
    unsigned long valid;

    /// These are validity bits used in checking the RINEX MET header.
    enum validBits
    {
      validVersion      = 0x01,   ///< Set if the RINEX version is valid.
      validRunBy        = 0x02,   ///< Set if the Run-by value is valid.
      validComment      = 0x04,   ///< Set if the comments are valid. Very subjective...
      validMarkerName   = 0x08,   ///< Set if the Marker Name is valid.
      validMarkerNumber = 0x010,  ///< Set if the Marker Number is valid.
      validObsType      = 0x020,  ///< Set if Num/Types of Obs is valid.
      validSensorType   = 0x040,  ///< Set if Sensor Mod/Type/Acc is valid.
      validSensorPos    = 0x080,  ///< Set if Sensor position is valid.

      validEoH = 0x080000000,     ///< Set if the end of header is valid.

      /// This bitset checks that all required header items are
      /// available for a Rinex2.0 file.
      allValid20  = 0x0800000EB,
      /// This bitset checks that all required header items are
      /// available for a Rinex2.1 file.
      allValid21  = 0x0800000EB,
      /// This bitset checks that all required header items are
      /// available for a Rinex2.11 file.
      allValid211 = 0x0800000EB
    };

    static inline std::string bitsAsString(validBits b)
    {
      switch (b)
      {
        case validVersion:
          return stringVersion;
        case validRunBy:
          return stringRunBy;
        case validComment:
          return stringComment;
        case validMarkerName:
          return stringMarkerName;
        case validMarkerNumber:
          return stringMarkerNumber;
        case validObsType:
          return stringObsType;
        case validSensorType:
          return stringSensorType;
        case validSensorPos:
          return stringSensorPos;
        case validEoH:
          return stringEoH;
		default: break; //NB Determine if additional enumeration values need to be handled
      }

      return "*UNKNOWN/INVALID BITS*";
    }

    /* @return a string composed the RINEX header strings
     * represented by the validBits set in vb, surrounded by the
     * quote character, separated by sep.
     * @param vb bit field with valid bits set
     * @param quote quote character to use around RINEX header strings.
     * @param sep separator used between rinex header strings.
     */
    static std::string bitString(unsigned long vb, char quote='\"',
                                 std::string sep=", ");


    /** @name HeaderValues
     */
    //@{

    double version; ///< RINEX Version

    std::string fileType;                      ///< The type of file it is.
    std::string fileProgram;                   ///< The program that generated it.
    std::string fileAgency;                    ///< The group who generated it.
    std::string date;                          ///< When the file was written.
    std::vector<std::string> commentList;      ///< A list of comments.
    std::string markerName;                    ///< The name of the location.
    std::string markerNumber;                  ///< The number of the location.

    /**
     * A list of obs types in the file.
     * @warning This variable determines what data gets written to
     * an output RinexObsStream.  Set this with the RinexMetTypes
     * that you want written to the file.
     */
    std::vector<RinexMetType> obsTypeList;
    std::vector<sensorType> sensorTypeList;    ///< A list of sensors used in the file
    std::vector<sensorPosType> sensorPosList;  ///< A list of sensor positions used in the file

    //@}

    static const int maxObsPerLine;       ///< Holds the max records per line

    /** @name FormattingStrings
     */
    //@{
    static const std::string stringVersion;       ///< "RINEX VERSION / TYPE"
    static const std::string stringRunBy;         ///< "PGM / RUN BY / DATE"
    static const std::string stringComment;       ///< "COMMENT"
    static const std::string stringMarkerName;    ///< "MARKER NAME"
    static const std::string stringMarkerNumber;  ///< "MARKER NUMBER"
    static const std::string stringObsType;       ///< "# / TYPES OF OBSERV"
    static const std::string stringSensorType;    ///< "SENSOR MOD/TYPE/ACC"
    static const std::string stringSensorPos;     ///< "SENSOR POS XYZ/H"
    static const std::string stringEoH;           ///< "END OF HEADER"
    //@}

  protected:

  /// Writes the RINEX Met header to the stream \a s.
    virtual void reallyPutRecord(FFStream& s) const
      throw(std::exception, FFStreamError,
            gpstk::StringUtils::StringException);

    /**
     * This function reads the RINEX MET header from the given FFStream.
     * If an error is encountered in reading from the stream, the stream
     * is reset to its original position and its fail-bit is set.
     * @throws StringException when a StringUtils function fails
     * @throws FFStreamError when exceptions(failbit) is set and
     *  a read or formatting error occurs.  This also resets the
     *  stream to its pre-read position.
     */
    virtual void reallyGetRecord(FFStream& s)
      throw(std::exception, FFStreamError,
            gpstk::StringUtils::StringException);

  }; // class RinexMetHeader

  /// Struct for holding information about a sensor
   struct sensorType
   {
      std::string model;    ///< The manufacturer of the sensor.
      std::string type;     ///< The specific type of sensor.
      double accuracy;      ///< The accuracy of the sensor.
      RinexMetHeader::RinexMetType obsType; ///< The obs type this sensor corresponds to.
   };

   /// Struct for holding info about a sensor position
   struct sensorPosType
   {
     /**
      * XYZ and height of sensor coordinates.  Use ITRF or WGS-84 coordinates.
      */
      gpstk::Triple position;
      double height;
      /// The obs type of the sensor this position corresponds to.
      RinexMetHeader::RinexMetType obsType;
   };


  //@}

} // namespace

#endif
