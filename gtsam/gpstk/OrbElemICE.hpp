#pragma ident "$Id$"

/**
 * @file OrbElemICE.hpp
 * Designed to support loading CNAV2 data
 */

#ifndef GPSTK_ORBELEMICE_HPP
#define GPSTK_ORBELEMICE_HPP

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

#include <string>
#include <iostream>

#include "OrbElem.hpp"
#include "FICData.hpp"


namespace gpstk
{
   class OrbElemICE : public OrbElem
   {
   public:
         /// Default constructor
      OrbElemICE();
  
         /// Destructor
      virtual ~OrbElemICE() {}

         /// Clone method
         /// Implication of the "= 0" at this end is that this is a 
         /// "pure virtual" method and that makes OrbElemICE an abstract 
         /// class.  That is to say no objects of type OrbElemICE may
         /// be constructed.   This is a good thing since OrbElemICE
         /// doesn't even provide methods to load its' members.  
         /// Only its' descendents may be instantiated.         
      virtual OrbElemICE* clone() const = 0;

      virtual std::string getName() const
      {
         return "OrbElemICE";
      }

      virtual std::string getNameLong() const
      {
         return "General ICE Message";
      }
      
      virtual void dumpHeader(std::ostream& s = std::cout) const
         throw( InvalidRequest );

         /** Generate a formatted human-readable one-line output that summarizes
           * the critical times associated with this object and send it to the
           * designated output stream (default to cout).
           * @throw Invalid Parameter if the object has been instantiated, but not loaded.
           */   
      void dumpTerse(std::ostream& s = std::cout) const
         throw( InvalidRequest );

         /** Compute dependent URA values    
           * See IS-GPS-800 3.5.3.5
           * Elevation in degrees
           */
      double getAdjNomURAed(const double elevation) const
         throw( InvalidRequest );

      double getCompositeIAURA(const CommonTime& t, const double elevation) const
         throw( InvalidRequest );
   
      double getAdjIAURAed(const double elevation) const
         throw( InvalidRequest );
         
         /** Compute non-elevative dependent URA values
           * See IS-GPS-800 3.5.3
           */
      double getIAURAned(const CommonTime& t) const
         throw( InvalidRequest );      

        /* Should only be used by GPSOrbElemStore::rationalize()
         */
      void adjustBeginningValidity();

      CommonTime transmitTime;     /**< Estimated beginning time of this sample */
      CommonTime ctTop;		   /**< Predicted time of week */
  
      short  URAed;		   /**< ED accuracy index */
      short  URAned0;		   /**< NED accuracy index */
      short  URAned1;		   /**< NED accuracy change index */
      short  URAned2;		   /**< NED accuracy change rate index */
      bool   IntegrityStatusFlag;  /**< Integrity Status Flag */

      static long ONE_HOUR;
      static long TWO_HOURS;
      static long NINTY_MINUTES; 
            
   }; // end class OrbElemICE

   std::ostream& operator<<(std::ostream& s, 
                                    const OrbElemICE& eph);
} // end namespace

#endif // GPSTK_OrbElemICE_HPP

