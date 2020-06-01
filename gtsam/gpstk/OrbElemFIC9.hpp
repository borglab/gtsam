#pragma ident "$Id$"

/**
 * @file OrbElemFIC9.hpp
 * SF 1/2/3 data from an FIC Block 9 encapsulated in engineering terms.
 * Class inherits from OrbElem and adds those items unique to and FIC Block 9
 */

/**
*
*/

#ifndef GPSTK_ORBELEMFIC9_HPP
#define GPSTK_ORBELEMFIC9_HPP

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

#include "OrbElemLNav.hpp"
#include "FICData.hpp"


namespace gpstk
{
   class OrbElemFIC9 : public OrbElemLNav
   {
   public:
         /// Default constructor
      OrbElemFIC9();
        
        /**
         *Construct an object from an existing FIC 9 data record
         * @throw InvalidParameter if FICData object is not a Block 9 record.
         */ 
      OrbElemFIC9( const FICData& fic9 )
	 throw( InvalidParameter); 

         /// Destructor
      virtual ~OrbElemFIC9() {}

         /// Clone function
      virtual OrbElemFIC9* clone() const;

        /**
         * Load the data FIC 9 data record into an existing object.
         * All data already present are replaced.
         * @throw InvalidParameter if FICData object is not a Block 9 record.
         */
      void loadData( const FICData& fic9 )
	 throw( InvalidParameter); 

       virtual std::string getName() const
       {
          return "OrbElemFIC9";
       }

       virtual std::string getNameLong() const
       {
          return "FIC Block 9";
       }
      
   }; // end class OrbElemFIC9

   //@}

} // end namespace

#endif // GPSTK_ORBELEMFIC9_HPP
