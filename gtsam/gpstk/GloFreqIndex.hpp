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

//============================================================================
//
// This software developed by Applied Research Laboratories at the University
// of Texas at Austin, under contract to an agency or agencies within the U.S. 
// Department of Defense. The U.S. Government retains all rights to use,
// duplicate, distribute, disclose, or release this software. 
//
// Pursuant to DoD Directive 523024 
//
// DISTRIBUTION STATEMENT A: This software has been approved for public 
//                           release, distribution is unlimited.
//
//============================================================================

/**
 * @file GloFreqIndex.hpp
 * Calculate GLONASS SV frequency index from range & phase data and store it.
 */

#ifndef GPSTK_GLOFREQINDEX_HPP
#define GPSTK_GLOFREQINDEX_HPP

#include <iostream>
#include <string>
#include <vector>
#include <map>

#include "CommonTime.hpp"
#include "RinexSatID.hpp"
#include "StringUtils.hpp"

namespace gpstk
{
   //@{

   /**
    * This class determines the GLONASS frequency (channel) indexes
    * from Obs data.  The slope of Range-minus-Phase v. Phase is a
    * function of delta-lambda/lambda.  First differences are used
    * to allow for implementation of simple outlier rejection.
    * 
    * Data is input using SatPass (lib/geomatics), i.e. one pass of
    * a single SV at a time, and stored in a map of vectors of
    * structs only if the pass generated a valid solution.  After
    * all Obs data has been added, the user should call calcIndex()
    * to generate the simple SV ID v. index lookup map.
    * 
    * There is also a knownIndex() method which fills the lookup
    * map with GLONASS frequency indexes known as of January 2010.
    * This is mostly for testing.  While it could be updated, that
    * frequency indexes can change over time makes it unreliable
    * for general use.
    */

   class GloFreqIndex
   {
   public:

      /// Default constructor -- takes no arguments.

      GloFreqIndex()
      { knownIndex(); }

      /// Destructor.

      virtual ~GloFreqIndex()
      {}

      /// Data type declarations.

      struct IndexData
      {
         CommonTime tt;   // time of epoch
         int    pG1, pG2; // number of points in pass
         double fG1, fG2; // float index solutions
         double dG1, dG2; // uncertainty on the float solutions
         int    nG1, nG2; // integer index solutions
      };

      /// Vector of Data structs to store multiple passes.
      typedef std::vector<IndexData> passData;

      /// Method to get frequency index from known table.  For testing purposes only.

      void knownIndex()
         throw();

      /// Method to get the frequency indexes from RINEX 3 Nav data.
      /// Error codes:
      ///   0 = no error
      ///   1 = existing entry disagrees with new data

      int getFromRinex3Nav( const std::string& filename )
         throw();

      /// Method to generate frequency index map from accumulated SatPass data.
      /// Set verbose != 0 to get pass-by-pass info via cout during execution.

      void calcIndex( const int& verbose = 0 )
         throw();

      /// Method to calculate frequency index for a single satellite from
      /// range & phase data in a single pass for G1 and G2 bands.  The
      /// integer returned is the band index determined from data (also
      /// appended to the internal data vector).
      /// This method assumes relatively clean data, but does scrub for
      /// outliers (likely cycle slips) after initial slope determination.
      /// The int returned is an error code:
      ///  -3  no data (or only 1 point) left after filtering
      ///  -2  divide by zero for G2 slope
      ///  -1  divide by zero for G1 slope
      ///   0  no errors
      ///   1  G1 range and phase vector lengths not equal
      ///   2  G2 range and phase vector lengths not equal
      ///   3  G1 & G2 results disagree
      ///   4  uncertainty on G1 result too large
      ///   5  uncertainty on G2 result too large
      /// CommonTime tt should be the start time of the pass.

      int addPass( const RinexSatID& id, const CommonTime& tt,
                   const std::vector<double>& r1, const std::vector<double>& p1,
                   const std::vector<double>& r2, const std::vector<double>& p2,
                   const int& verbose = 0                                       )
         throw();

      /// Method to calculate frequency index for a single satellite from
      /// range & phase data in a single pass for the G1 band only.  This
      /// method provides empty G2 vectors to the above method.  Note that
      /// if one provides range & phase data for only one band, it is
      /// assumed to be G1!
      /// CommonTime tt should be the start time of the pass.
      /// Method currently implemented, but all solutions will be rejected.

      int addPass( const RinexSatID& id, const CommonTime& tt,
                   const std::vector<double>& r1, const std::vector<double>& p1,
                   const int& verbose = 0                                       )
         throw();

      /// This method returns the GLONASS truth index for a given SV.
      /// It returns -100 if there is no entry for the given SatID.

      int getIndexTruth( const RinexSatID& id )
         throw();

      /// This method returns the GLONASS index from data for a given SV.
      /// It returns -100 if there is no entry for the given SatID.

      int getIndex( const RinexSatID& id )
         throw();

      /// This method returns the GLONASS frequency for a given SV and band.
      /// It calls getIndexTruth(id) to get the channel index frmo the truth
      /// table, then looks up the frequency in icd_glo_constants.  The error
      /// codes are:
      ///    0  no error
      ///    1  no entry for the given SatID
      ///    2  invalid frequency band

      double getFreqTruth( const RinexSatID& id, const int& band, int& error )
         throw();

      /// This method returns the GLONASS frequency for a given SV and band.
      /// It calls getIndex(id) to get the channel index, then looks up
      /// the frequency in icd_glo_constants.  The error codes are:
      ///    0  no error
      ///    1  no entry for the given SatID
      ///    2  invalid frequency band

      double getFreq( const RinexSatID& id, const int& band, int& error )
         throw();

      /// Dump the contents of a single pass to cout in a nice format.

      void dumpPass( const RinexSatID& id, const IndexData& data ) const;

      /// Dump the contents of the data store to a stream s in a nice format.

      void dump( std::ostream& s ) const;

   protected:



   private: /// All data goes here -- use public accessors to add & view data.

      /// Map of data (vector of IndexData structs) by SV ID.
      std::map< RinexSatID, passData > dataMap;

      /// Map of index solutions (single integer) by SV ID.
      std::map< RinexSatID, int > freqIndex;

      /// Map of index truth solutions (single integer) by SV ID.
      std::map< RinexSatID, int > freqIndexTruth;

      /// Max. allowable point-to-point phase jump (meters).
      static const double maxDist;

      /// Max. allowable range-phase point-to-point shift (log, e.g. 1 is x10).
      static const double maxRPshift;

      /// Number of GLONASS SVs possible.
      static const int numSats;

   }; // class GloFreqIndex

   //@}

} // namespace

#endif // GPSTK_GLOFREQINDEX_HPP
