#pragma ident "$Id$"

/**
 * @file OrbElemFIC109.hpp
 *  Designed to accomodate loading GPS legacy navigation message 
 *  data stored in block 109s in the FIC data format.
 */

#ifndef GPSTK_ORBELEMFIC109_HPP
#define GPSTK_ORBELEMFIC109_HPP

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

namespace gpstk
{
   class OrbElemFIC109 : public OrbElemLNav
   {
   public:
         /// Default constructor
      OrbElemFIC109();

         /** Create an object based on the contents of a FICData
          *  block 109.
          *  @throw InvalidParameter if the FICData object does not contain an FIC Block 109.
          */ 
      OrbElemFIC109( const FICData& fic109 )
	 throw( InvalidParameter); 

         /** Load the object from the navigation message data contained in the
          *  arguments. Any existing data in the object is overwritten with the
          *  new data.
          *  See the corresponding constructor for a description of the arguments.
          *  @throw InvalidParameter if the input data are inconsistent.
          */ 

         /// Destructor
      virtual ~OrbElemFIC109() {}

        /// Clone function
      virtual OrbElemFIC109* clone() const;

         /// Load a FIC 9 into an existing object
      void loadData( const FICData& fic109 )
	 throw( InvalidParameter); 

      virtual std::string getName() const
      {
         return "OrbElemFIC109";
      }

      virtual std::string getNameLong() const
      {
         return "FIC Block 109";
      }

   }; // end class OrbElemFIC109

   //@}

} // end namespace

#endif // GPSTK_ORBELEMFIC109_HPP
