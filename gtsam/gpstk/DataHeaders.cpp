//============================================================================
//
//  This file is part of GPSTk, the GPS Toolkit.
//
//  The GPSTk is free software; you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published
//  by the Free Software Foundation; either version 3.0 of the License, or
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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2007, 2008, 2009, 2011
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

/**
 * @file DataHeaders.cpp
 * Set of several headers to be used by data structures.
 */

#include "DataHeaders.hpp"


using namespace std;


namespace gpstk
{


      // Assignment operator
   sourceHeader& sourceHeader::operator=(const sourceHeader& right)
   {

      if ( this == &right )
      {
         return (*this);
      }

      source = right.source;

      return (*this);

   }  // End of operator 'sourceHeader::operator=()'



      // Convenience output method for sourceHeader
   std::ostream& sourceHeader::dump(std::ostream& s) const
   {

      s << source;

      return s;

   }  // End of method 'sourceHeader::dump()'



      // stream output for sourceHeader
   std::ostream& operator<<( std::ostream& s,
                             const sourceHeader& sh )
   {

      sh.dump(s);

      return s;

   }  // End of 'operator<<' for sourceHeader



      // Assignment operator
   sourceEpochHeader& sourceEpochHeader::operator=(
                                                const sourceEpochHeader& right )
   {

      if ( this == &right )
      {
         return (*this);
      }

      source = right.source;
      epoch = right.epoch;

      return (*this);

   }  // End of 'sourceEpochHeader::operator=()'



      // Convenience output method for sourceEpochHeader
   std::ostream& sourceEpochHeader::dump(std::ostream& s) const
   {

      s << source << " " << epoch;

      return s;

   }  // End of method 'sourceEpochHeader::dump()'



      // stream output for sourceEpochHeader
   std::ostream& operator<<( std::ostream& s,
                             const sourceEpochHeader& seh )
   {

        seh.dump(s);

        return s;

   }  // End of 'operator<<' for sourceEpochHeader



      // Explicit constructor
   sourceEpochRinexHeader::sourceEpochRinexHeader( const sourceHeader& sh,
                                                   const CommonTime& time,
                                                   const std::string& antType,
                                                   const Triple& antPos,
                                                   const short& flag )
      : antennaType(antType), antennaPosition(antPos), epochFlag(flag)
   {

      source.sourceName = sh.source.sourceName;
      source.type = sh.source.type;
      epoch = time;

   }  // End of constructor 'sourceEpochRinexHeader::sourceEpochRinexHeader()'



      // Assignment operator
   sourceEpochRinexHeader& sourceEpochRinexHeader::operator=(
                                          const sourceEpochRinexHeader& right)
   {

      if ( this == &right )
      {
         return (*this);
      }

      source = right.source;
      epoch = right.epoch;
      antennaType = right.antennaType;
      antennaPosition = right.antennaPosition;
      epochFlag = right.epochFlag;

      return (*this);

   }  // End of operator 'sourceEpochRinexHeader::operator=()'



      // Convenience output method for sourceEpochRinexHeader
   std::ostream& sourceEpochRinexHeader::dump(std::ostream& s) const
   {

      s << source          << " "
        << epoch           << " "
        << antennaType     << " "
        << antennaPosition << " "
        << epochFlag;

        return s;

   }  // End of method 'sourceEpochRinexHeader::dump()'



      // stream output for sourceEpochRinexHeader
   std::ostream& operator<<( std::ostream& s,
                             const sourceEpochRinexHeader& serh )
   {

      serh.dump(s);

      return s;

   }  // End of 'operator<<' for sourceEpochRinexHeader



      // Assignment operator from a sourceTypeHeader
   sourceTypeHeader& sourceTypeHeader::operator=(const sourceTypeHeader& right)
   {

      if ( this == &right )
      {
         return (*this);
      }

      source = right.source;
      type = right.type;

      return (*this);

   }  // End of operator 'sourceTypeHeader::operator=()'



      // Convenience output method for sourceTypeHeader
   std::ostream& sourceTypeHeader::dump(std::ostream& s) const
   {

      s << source << " " << type;

      return s;

   }  // End of method 'sourceTypeHeader::dump()'



      // stream output for sourceTypeHeader
   std::ostream& operator<<( std::ostream& s,
                             const sourceTypeHeader& sth )
   {

      sth.dump(s);

      return s;

   }  // End of 'operator<<' for sourceTypeHeader



      // Assignment operator from a sourceSatHeader
   sourceSatHeader& sourceSatHeader::operator=(const sourceSatHeader& right)
   {

      if ( this == &right )
      {
         return (*this);
      }

      source = right.source;
      satellite = right.satellite;

      return (*this);

   }  // End of operator 'sourceSatHeader::operator=()'



      // Convenience output method for sourceSatHeader
   std::ostream& sourceSatHeader::dump(std::ostream& s) const
   {

      s << source << " " << satellite;

      return s;

   }  // End of method 'sourceSatHeader::dump()'



      // stream output for sourceSatHeader
   std::ostream& operator<<( std::ostream& s,
                             const sourceSatHeader& ssh )
   {

      ssh.dump(s);

      return s;

   }  // End of 'operator<<' for sourceSatHeader



      // Assignment operator from a sourceEpochSatHeader
   sourceEpochSatHeader& sourceEpochSatHeader::operator=(
                                             const sourceEpochSatHeader& right )
   {

      if ( this == &right )
      {
         return (*this);
      }

      source = right.source;
      epoch = right.epoch;
      satellite = right.satellite;

      return (*this);

   }  // End of operator 'sourceEpochSatHeader::operator=()'



      // Convenience output method for sourceEpochSatHeader
   std::ostream& sourceEpochSatHeader::dump(std::ostream& s) const
   {

      s << source << " "
        << epoch  << " "
        << satellite;

      return s;

   }  // End of method 'sourceEpochSatHeader::dump()'



      // stream output for sourceEpochSatHeader
   std::ostream& operator<<( std::ostream& s,
                             const sourceEpochSatHeader& sesh )
   {

      sesh.dump(s);

      return s;

   }  // End of 'operator<<' for sourceEpochSatHeader



      // Assignment operator from a sourceEpochTypeHeader
   sourceEpochTypeHeader& sourceEpochTypeHeader::operator=(
                                          const sourceEpochTypeHeader& right )
   {

      if ( this == &right )
      {
         return (*this);
      }

      source = right.source;
      epoch = right.epoch;
      type = right.type;

      return (*this);

   }  // End of operator 'sourceEpochTypeHeader::operator=()'



      // Convenience output method for sourceEpochTypeHeader
   std::ostream& sourceEpochTypeHeader::dump(std::ostream& s) const
   {

      s << source << " "
        << epoch  << " "
        << type;

      return s;

   }  // End of method 'sourceEpochTypeHeader::dump()'



      // stream output for sourceEpochTypeHeader
   std::ostream& operator<<( std::ostream& s,
                             const sourceEpochTypeHeader& seth )
   {

      seth.dump(s);

      return s;

   }  // End of 'operator<<' for sourceEpochTypeHeader


}  // End of namespace gpstk
