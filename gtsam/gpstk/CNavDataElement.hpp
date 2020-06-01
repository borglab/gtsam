#pragma ident "$Id:$"

/**
 * @file CNavDataElement.hpp
 *  This class encapsulates the "least common denominator"
 *  for non orbit/clock data defined in the CNAV and CNAV-2 
 *  sections of the GPS signal interface specifications.
 *  This includes items such as the group delay, ISC, UTC, IONO,
 *  EOP, GGTO, and text information.  The items in common across these
 *  data sets are unified here.  This includes the concept of an
 *  epoch time, a transmit time, a source SV, and a source
 *  carrier/code.
 *
 *  This is a pure virtual class.   Each of the data elements are 
 *  fully implemented (including load methods) in descendent
 *  classes.  The use of a common base class makes it practical to
 *  store the elements together in larger stores while maintaining
 *  the characteristics of the individual data types.
 */

#ifndef GPSTK_CNAVDATAELEMENT_HPP
#define GPSTK_CNAVDATAELEMENT_HPP

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
//  Copyright 2013, The University of Texas at Austin
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


#include "ObsID.hpp"
#include "Exception.hpp"
#include "CommonTime.hpp"
#include "GNSSconstants.hpp"
#include "SatID.hpp"
#include "GPSWeekSecond.hpp"

namespace gpstk
{
   class CNavDataElement
   {
   public:
         /// Constructors
	      /// Default constuctor
	   CNavDataElement( );

         /// Destructor
      virtual ~CNavDataElement() { }

         /// Clone method.
	 /// Return pointer to new copy of this type.
	 /// Implication of the "= 0" at this end is that this is a 
	 /// "pure virtual" method and that makes CNavDataElement an abstract 
	 /// class.  That is to say no objects of type CNavDataElement may
	 /// be constructed.   This is a good thing since CNavDataElement
	 /// doesn't even provide methods to load its' members.  
	 /// Only its' descendents may be instantiated.
      virtual CNavDataElement* clone() const = 0;

	 /**
          *   Return true if data have been loaded.
          *   Returns false if the object has been instantiated,
          *   but no data have been loaded.
          */ 
      virtual bool dataLoaded( ) const;

      virtual std::string getName() const = 0;

      virtual std::string getNameLong() const = 0;

         // Returns true if this two objects are 
         //   a.) same concrete type, and
         //   b.) same data contents.
         // This is intended as a "data uniqueness test" to allow
         // detection of successive transmissions of same data
         // and avoid duplicate storage.  The exact rules for 
         // uniqueness will vary by descendent class. 
      virtual bool isSameData(const CNavDataElement* right) const = 0;
      
         /** Output the contents of this orbit data to the given stream. 
          * @throw Invalid Request if the required data has not been stored.
          */
      static void shortcut(std::ostream & os, const long HOW);

      static void timeDisplay(std::ostream & os, const CommonTime& t);
/*
      virtual void dumpTerse(std::ostream& s = std::cout) const
         throw( InvalidRequest ) = 0;
*/
      virtual void dumpHeader(std::ostream& s = std::cout) const
         throw( InvalidRequest );

      virtual void dumpBody(std::ostream& s = std::cout) const
         throw( InvalidRequest );

      virtual void dumpFooter(std::ostream& s = std::cout) const
         throw( InvalidRequest );
     
      virtual void dump(std::ostream& s = std::cout) const 
         throw( InvalidRequest );
     
         /// Overhead information
         //@{
      bool    dataLoadedFlag;  /**< True if data is present, False otherwise */
      SatID   satID;	          /**< Define satellite system and specific SV
                                    that transmitted the data */
      ObsID   obsID;           /**< Defines carrier and tracking code */
      CommonTime ctEpoch;      /**< Epoch time                        */
      CommonTime ctXmit;       /**< Time of transmission              */
              //@}
         
   }; // end class CNavDataElement

   //@}
   
   std::ostream& operator<<(std::ostream& s, 
                                      const CNavDataElement& eph);

} // end namespace

#endif // GPSTK_CNavDataElement_HPP
