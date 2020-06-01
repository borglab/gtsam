/// @file RinexUtilities.hpp
/// Miscellaneous RINEX-related utilities.

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

#ifndef GPSTK_RINEX_UTILITIES_INCLUDE
#define GPSTK_RINEX_UTILITIES_INCLUDE

//------------------------------------------------------------------------------------
// system includes
#include <string>
#include <vector>

// GPSTk
#include "GPSEphemerisStore.hpp"
#include "SP3EphemerisStore.hpp"

namespace gpstk {
//------------------------------------------------------------------------------------
/** @defgroup rinexutils Rinex Utilities */
//@{
/// Register commonly used extended Rinex observation types.
/// @return 0 ok, or the return value of RegisterExtendedRinexObsType().
int RegisterARLUTExtendedTypes(void);

/// Determine if the given file is an SP3 format file.
/// @param file the filename
/// @return true if the file is SP3 format.
bool isSP3File(const std::string& file);

/// Determine if the given file is a RINEX clock format file.
/// @param file the filename
/// @return true if the file is RINEX clock format.
//NB This was removed since it is never defined.
// bool isRinexClockFile(const std::string& file);

/// Determine if the given file is a RINEX navigation file.
/// Open the file, read the header, and test its validity.
/// @param file the filename
/// @return true if the file is a valid Rinex navigation file.
bool isRinexNavFile(const std::string& file);

/// Determine if the given file is a RINEX 3 navigation file.
/// Open the file, read the header, and test its validity.
/// @param file the filename
/// @return true if the file is a valid Rinex navigation file.
bool isRinex3NavFile(const std::string& file);

/// Determine if the given file is a RINEX observation file.
/// Open the file, read the header, and test its validity.
/// @param file the filename
/// @return true if the file is a valid Rinex observation file.
bool isRinexObsFile(const std::string& file);

/// Determine if the given file is a RINEX 3 observation file.
/// Open the file, read the header, and test its validity.
/// @param file the filename
/// @return true if the file is a valid Rinex observation file.
bool isRinex3ObsFile(const std::string& file);

/// Sort a vector of RINEX obs file names on the time of the first observation
/// as found in the header. Return the sorted list in the calling argument.
/// Do not include files that are found not to be RINEX obs files, or that have
/// invalid headers.
/// @param files  vector<string> containing filenames, with path
/// @return string containing error messages, if any
std::string sortRinexObsFiles(std::vector<std::string>& files) throw(Exception);

/// Sort a vector of RINEX 3 obs file names on the time of the first observation
/// as found in the header. Return the sorted list in the calling argument.
/// Do not include files that are found not to be RINEX 3 obs files, or that have
/// invalid headers.
/// @param files  vector<string> containing filenames, with path
/// @return string containing error messages, if any
std::string sortRinex3ObsFiles(std::vector<std::string>& files);

/// Open the files and add to the appropriate EphemerisStore.
/// @param files vector of the filenames.
/// @param PE an SP3EphemerisStore into which to put SP3 ephemeris data.
/// @param BCE a GPSEphemerisStore into which to put broadcast
///    (Rinex Nav) ephemeris data.
/// @return the number of files successfully read.
int FillEphemerisStore(const std::vector<std::string>& files,
                       gpstk::SP3EphemerisStore& PE,
                       gpstk::GPSEphemerisStore& BCE);

//@}

} // end namespace gpstk

#endif
//------------------------------------------------------------------------------------
