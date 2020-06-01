#pragma ident "$Id$"

/**
 * @file SourceID.hpp
 * gpstk::SourceID - Simple index to represent the source of data.
 */

#ifndef GPSTK_SOURCEID_HPP
#define GPSTK_SOURCEID_HPP

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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2006, 2007, 2008
//
//============================================================================


#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <map>



namespace gpstk
{

      /** @addtogroup DataStructures */

   class SourceID
   {
   public:

         /// The type of source.
      enum SourceType
      {
         Unknown,
         GPS,        ///< GPS data
         Galileo,    ///< Galileo data
         Glonass,    ///< Glonass data
         Geosync,    ///< Geostationary data
         LEO,        ///< LEO data
         Transit,    ///< Transit data
         Compass,    ///< Compass data
         Mixed,      ///< Mixed data
         UserDefined,///< UserDefined
         DGPS,       ///< Differential GPS data
         RTK,        ///< Real Time Kinematic data
         INS,        ///< Inertial System data
         Last,       ///< used to extend this...
         Placeholder = Last+1000
      };


         /// empty constructor, creates an unknown source data object
      SourceID()
         : type(Unknown), sourceName("")
      {};


         /// Explicit constructor
      SourceID( SourceType st,
                std::string name )
         : type(st), sourceName(name)
      {};


         /// Copy constructor
      SourceID(const SourceID& s)
         : type(s.type), sourceName(s.sourceName)
      {};


         /// Equality operator requires all fields to be the same.
      virtual bool operator==(const SourceID& right) const;


         /// Ordering is arbitrary but required to be able to use a SourceID
         /// as an index to a std::map. If an application needs
         /// some other ordering, inherit and override this function.
      virtual bool operator<(const SourceID& right) const;


         /// Inequality operator
      bool operator!=(const SourceID& right) const
      { return !(operator==(right)); }


         /// 'Greater than' operator
      bool operator>(const SourceID& right) const
      {  return (!operator<(right) && !operator==(right)); }


         /// 'Less or equal than' operator
      bool operator<=(const SourceID& right) const
      { return (operator<(right) || operator==(right)); }


         /// 'Greater or equal than' operator
      bool operator>=(const SourceID& right) const
      { return !(operator<(right)); }


         /// Assignment operator
      SourceID& operator=(const SourceID& right);


         /// Convenience output method
      virtual std::ostream& dump(std::ostream& s) const;


         /// Returns true if this is a valid SourceID. Basically just
         /// checks that none of the fields are undefined.
      virtual bool isValid() const;


         /// Destructor
      virtual ~SourceID() {};


         /// Method to create a new source type.
      static SourceType newSourceType(const std::string& s);


         // Fields

         /// Type of the data source (GPS receiver, Inertial system, etc)
      SourceType  type;


         /// Name of the data source
      std::string sourceName;


   private:


      static std::map< SourceType, std::string > stStrings;


   public:

      #ifndef SWIG
      class Initializer
      {
      public:
         Initializer();
      };

      static Initializer SourceIDsingleton;
      #endif

   }; // End of class 'SourceID'



   namespace StringUtils
   {

         /// Convert this object to a string representation
      std::string asString(const SourceID& p);

   }



      /// Stream output for SourceID
   std::ostream& operator<<( std::ostream& s,
                             const SourceID& p );



}  // End of namespace gpstk
#endif   // GPSTK_SOURCEID_HPP
